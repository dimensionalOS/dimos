# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Robot-independent Zenoh topic provider: one relay source over a raw session.

Run the pinned runtime with ``dimos-topic-service --config <json>``. The relay
must run with open topics (implied without an auth file); with an auth file the
config also carries the source's key as ``token``. The process owns its Zenoh
session across relay reconnections; no selected checkout, Module, coordinator,
typed message or blueprint is imported.
"""

import argparse
import asyncio
from collections.abc import Callable
from contextlib import suppress
import os
from pathlib import Path
import signal
import sys
import time
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, SecretStr, ValidationError
import zenoh

from dimos.protocol.service.zenoh_wire import LOOPBACK_INTERFACE, LOOPBACK_LISTEN, make_zenoh_config
from dimos.utils.logging_config import setup_logger
from dimos.web.relay_bridge.protocol import DataFrame, FrameHeader, Msg, ProtocolError
from dimos.web.relay_bridge.topic_catalog import TopicCatalog
from dimos.web.relay_bridge.topic_guard import GuardedPublishers
from dimos.web.relay_bridge.topic_mailbox import TopicIngressBudget, TopicMailbox, TopicSample
from dimos.web.relay_bridge.topic_metadata import MetadataReceiver
from dimos.web.relay_bridge.topic_profiles import (
    MEDIA_PROFILES,
    PROFILE_OPTIONS,
    SETTING_NAMES,
    normalize_settings,
)
from dimos.web.relay_bridge.topic_protocol import (
    MAX_TOPIC_ROUTES,
    TopicActivate,
    TopicCapabilities,
    TopicClose,
    TopicError,
    TopicGuardClosed,
    TopicGuardCommandMeta,
    TopicGuardHeartbeat,
    TopicGuardOpenMeta,
    TopicGuardStop,
    TopicList,
    TopicMsg,
    TopicOpen,
    TopicOpened,
    TopicPage,
    TopicPublishMeta,
    TopicPubResult,
    TopicSettings,
    TopicSource,
    TopicSources,
    TopicVariant,
    TopicVariantClose,
    TopicVariantReady,
)
from dimos.web.relay_bridge.topic_put import TopicPuts
from dimos.web.relay_bridge.topic_transform import TopicTransforms
from dimos.web.relay_bridge.topic_transport import LatestTopicWriter, TopicByteBudget
from dimos.web.relay_bridge.wt_client import RelayClient, RelayRejectedError, fetch_relay_info

_loop_factory: Callable[[], asyncio.AbstractEventLoop] = asyncio.new_event_loop
try:
    import uvloop
except ImportError:
    pass
else:
    _loop_factory = uvloop.new_event_loop

logger = setup_logger()


def _local_zenoh() -> dict[str, Any]:
    return {
        "mode": "peer",
        "listen": [LOOPBACK_LISTEN],
        "interface": LOOPBACK_INTERFACE,
        "gossip": False,
        "multicast": True,
    }


class TopicServiceConfig(BaseModel):
    model_config = ConfigDict(extra="forbid", strict=True)
    source: TopicSource
    relay_url: str
    # The source key from the relay's auth file; None for a relay without one.
    token: SecretStr | None = None
    read: bool = True
    write: bool = False
    media: bool = True
    guarded_publishers: bool = False
    zenoh: dict[str, Any] = Field(default_factory=_local_zenoh)


def _encode_key(settings: dict[str, Any]) -> list[Any]:
    """The settings that define an encoder; the rate is not one of them."""
    return [settings.get(name) for name in SETTING_NAMES if name != "maxHz"]


class _Variant:
    """One encoder configuration of a media route.

    Variants share the route's Zenoh subscription, mailbox and sequence
    numbers; each has its own writer, so the per-key latest slots of one do
    not replace another's, and its own due times.
    """

    def __init__(self, name: str, settings: dict[str, Any], writer: LatestTopicWriter) -> None:
        self.name = name
        self.settings = settings
        self.writer = writer
        self.next_due: dict[str, float] = {}
        # Keys whose newest held sample this variant has not encoded yet.
        self.pending: set[str] = set()

    @property
    def period(self) -> float:
        return 1 / float(self.settings["maxHz"])


class _Route:
    def __init__(
        self,
        session: zenoh.Session,
        client: RelayClient,
        spec: TopicOpen,
        settings: dict[str, Any] | None,
        ingress_budget: TopicIngressBudget,
        send_budget: TopicByteBudget,
        catalog: TopicCatalog,
        transforms: TopicTransforms,
        on_closed: Callable[[str], None],
    ) -> None:
        self._session = session
        self._client = client
        self.spec = spec
        self._catalog = catalog
        self._transforms = transforms
        self._media = MEDIA_PROFILES.get(spec.profile)
        self._send_budget = send_budget
        self._on_closed = on_closed
        self._loop = asyncio.get_running_loop()
        self._changed = asyncio.Event()
        self._active = asyncio.Event()
        self._mailbox = TopicMailbox(spec.delivery, float(spec.maxHz), ingress_budget, self._wake)
        self._writer = (
            None
            if self._media
            else client.topic_writer(spec.route, spec.delivery, send_budget, self._failed)
        )
        self._variants: dict[str, _Variant] = {}
        # The newest selected sample per key, held until every variant that
        # still needs it is due; a slow variant never encodes an obsolete frame.
        self._latest: dict[str, TopicSample] = {}
        # An older relay opens a media route without settings and drives its
        # one encoder's rate through topic_activate.
        self._plain = self._media is not None and settings is None
        if self._media:
            self._add_variant(
                "v1", settings or normalize_settings(spec.profile, {"maxHz": spec.maxHz})
            )
        self._error: str | None = None
        self._subscriber: zenoh.Subscriber[None] | None = None
        self._task: asyncio.Task[None] | None = None
        self._timer: asyncio.TimerHandle | None = None
        self._sequence = 0
        self.decode_failures = 0
        self._last_decode_warning = float("-inf")

    @property
    def variants(self) -> dict[str, dict[str, Any]]:
        return {name: dict(variant.settings) for name, variant in self._variants.items()}

    def _wake(self) -> None:
        self._loop.call_soon_threadsafe(self._changed.set)

    def _failed(self, code: str) -> None:
        self._error = code
        self._changed.set()

    def _sample(self, sample: zenoh.Sample) -> None:
        # No payload copy or transform on Zenoh's callback thread. This
        # timestamp is ingress, never a replacement for source acquisition.
        stamp = sample.timestamp
        self._mailbox.offer(
            TopicSample(
                key=str(sample.key_expr),
                encoding=str(sample.encoding),
                payload=sample.payload,
                ingress_ts=time.time(),
                source_ts=None if stamp is None else stamp.get_time().timestamp(),
                kind="delete" if sample.kind == zenoh.SampleKind.DELETE else "put",
            )
        )

    async def start(self) -> None:
        expression = zenoh.KeyExpr(self.spec.keyExpr)  # Zenoh owns validation/matching
        opening = asyncio.create_task(
            asyncio.to_thread(self._session.declare_subscriber, expression, self._sample)
        )
        try:
            self._subscriber = await asyncio.shield(opening)
        except asyncio.CancelledError:
            # Preserve cancellation even if the shielded Zenoh open fails.
            with suppress(zenoh.ZError):
                subscriber = await opening
                await asyncio.to_thread(subscriber.undeclare)
            raise
        if self._writer is not None:
            self._writer.start()
        self._task = asyncio.create_task(self._run())

    def activate(self, max_hz: float | None = None) -> None:
        if max_hz is not None:
            if self._media is None:
                self._mailbox.set_rate(max_hz)
            elif self._plain:
                variant = self._variants["v1"]
                variant.settings = normalize_settings(
                    self.spec.profile, {**variant.settings, "maxHz": max_hz}
                )
                self._sync_rate()
        self._active.set()
        self._changed.set()

    def _add_variant(self, name: str, settings: dict[str, Any]) -> None:
        writer = self._client.topic_writer(
            self.spec.route, "latest", self._send_budget, self._failed
        )
        assert isinstance(writer, LatestTopicWriter)
        writer.start()
        variant = _Variant(name, settings, writer)
        variant.pending = set(self._latest)
        self._variants[name] = variant
        self._sync_rate()

    def _sync_rate(self) -> None:
        """The mailbox samples at the fastest variant; each variant gates itself."""
        if self._media and self._variants:
            self._mailbox.set_rate(max(float(v.settings["maxHz"]) for v in self._variants.values()))
        self._changed.set()

    def set_variant(self, name: str, requested: dict[str, Any]) -> dict[str, Any]:
        """Open a variant, or change the rate of an open one; returns its settings."""
        settings = normalize_settings(self.spec.profile, requested)
        existing = self._variants.get(name)
        if existing is not None and _encode_key(existing.settings) == _encode_key(settings):
            existing.settings = settings
            self._sync_rate()
        else:
            if existing is not None:
                existing.writer.stop()
                del self._variants[name]
            self._add_variant(name, settings)
        return settings

    async def close_variant(self, name: str) -> None:
        variant = self._variants.pop(name, None)
        if variant is None:
            return
        self._prune_latest()
        self._sync_rate()
        await variant.writer.aclose()

    def _absorb(self, batch: list[TopicSample]) -> None:
        for sample in batch:
            previous = self._latest.get(sample.key)
            if previous is not None:
                self._mailbox.release(previous)
            self._latest[sample.key] = sample
            for variant in self._variants.values():
                variant.pending.add(sample.key)

    def _prune_latest(self) -> None:
        for key, sample in list(self._latest.items()):
            if not any(key in variant.pending for variant in self._variants.values()):
                del self._latest[key]
                self._mailbox.release(sample)

    def _metadata(self, sample: TopicSample) -> dict[str, Any]:
        metadata: dict[str, Any] = {
            "key": sample.key,
            "encoding": sample.encoding,
            "gen": self.spec.gen,
            "ingressTs": sample.ingress_ts,
            "kind": sample.kind,
        }
        if sample.source_ts is not None:
            metadata["sourceTs"] = sample.source_ts
        return metadata

    async def _run(self) -> None:
        try:
            await self._active.wait()
            coalesce = self.spec.delivery == "reliable"
            while True:
                await self._changed.wait()
                self._changed.clear()
                # Coalesce small event bursts before the QUIC packet builder.
                # Every event keeps its own frame and callback. Commands and
                # their acknowledgements use the separate control lanes.
                # Backlogged batches drain immediately, without a rate cap.
                if coalesce and self._mailbox.coalesce:
                    await asyncio.sleep(0.09)
                if self._timer is not None:
                    self._timer.cancel()
                    self._timer = None
                failure = self._error or self._mailbox.error
                if failure is not None:
                    self._client.send_control_frame(
                        TopicError(
                            id=self.spec.route,
                            code=failure,
                            message=failure.replace("_", " "),
                            outcome="interrupted",
                        )
                    )
                    return
                now = time.monotonic()
                if self._media:
                    delay = await self._encode_round(now)
                else:
                    delay = self._forward_round(now)
                coalesce = self.spec.delivery == "reliable" and delay is None
                if delay is not None:
                    self._timer = self._loop.call_later(delay, self._changed.set)
        finally:
            await self._dispose()
            self._on_closed(self.spec.route)

    def _forward_round(self, now: float) -> float | None:
        assert self._writer is not None
        batch, delay = self._mailbox.take(now, limit=128)
        try:
            for sample in batch:
                self._catalog.observe(sample.key, sample.encoding)
                self._sequence += 1
                self._writer.offer(
                    FrameHeader(
                        ch=self.spec.route,
                        seq=self._sequence,
                        ts=time.time(),
                        delivery=self.spec.delivery,
                        meta=self._metadata(sample),
                    ),
                    bytes(sample.payload),
                )
        except (BufferError, ConnectionError, ValueError, OSError, ImportError):
            self._failed("topic_forward_failed")
        finally:
            for sample in batch:
                self._mailbox.release(sample)
        return delay

    async def _encode_round(self, now: float) -> float | None:
        """Encode the newest sample of every key for each variant that is due.

        One worker permit is held per encode, never across two, so an event
        route waiting for a worker gets one between a route's variants.
        """
        batch, delay = self._mailbox.take(now, limit=128)
        self._absorb(batch)
        for variant in list(self._variants.values()):
            for key in sorted(variant.pending):
                due = variant.next_due.get(key, 0.0)
                if due > now:
                    delay = due - now if delay is None else min(delay, due - now)
                    continue
                await self._transforms.slots.acquire()
                try:
                    # Select again with the permit in hand: the wait for a
                    # worker must not leave an obsolete frame to encode.
                    fresh, _ = self._mailbox.take(time.monotonic(), limit=128)
                    self._absorb(fresh)
                    sample = self._latest.get(key)
                    if sample is None or self._variants.get(variant.name) is not variant:
                        continue
                    variant.pending.discard(key)
                    variant.next_due[key] = now + variant.period
                    await self._encode_one(variant, sample)
                finally:
                    self._transforms.slots.release()
        self._prune_latest()
        return delay

    async def _encode_one(self, variant: _Variant, sample: TopicSample) -> None:
        self._catalog.observe(sample.key, sample.encoding)
        metadata = self._metadata(sample)
        metadata["variant"] = variant.name
        payload = bytes(sample.payload)
        if sample.kind == "put":
            metadata["queueMs"] = max(0.0, (time.time() - sample.ingress_ts) * 1000)
            try:
                output = await self._transforms.encode(self.spec.profile, payload, variant.settings)
            except (ValueError, OSError) as error:
                self.decode_failures += 1
                now = time.monotonic()
                if now - self._last_decode_warning >= 5:
                    logger.warning(
                        "topic media sample dropped",
                        route=self.spec.route,
                        profile=self.spec.profile,
                        dropped=self.decode_failures,
                        error=str(error),
                    )
                    self._last_decode_warning = now
                return
            payload = output.payload
            metadata.update(output.metadata)
            metadata["encoding"] = output.encoding
            metadata["decodeFailures"] = self.decode_failures
        self._sequence += 1
        try:
            variant.writer.offer(
                FrameHeader(
                    ch=self.spec.route,
                    seq=self._sequence,
                    ts=time.time(),
                    delivery="latest",
                    meta=metadata,
                ),
                payload,
            )
        except (BufferError, ConnectionError, ValueError, OSError):
            self._failed("topic_forward_failed")

    async def _dispose(self) -> None:
        # Closing the mailbox returns every charged byte, held samples included.
        self._mailbox.close()
        self._latest.clear()
        if self._timer is not None:
            self._timer.cancel()
        subscriber, self._subscriber = self._subscriber, None
        if subscriber is not None:
            await asyncio.to_thread(subscriber.undeclare)
        for variant in self._variants.values():
            await variant.writer.aclose()
        if self._writer is not None:
            await self._writer.aclose()

    async def close(self) -> None:
        if self._task is not None:
            self._task.cancel()
            # A child's cancellation is expected; cancellation of this caller
            # must still reach the provider's shutdown. suppress() conflates
            # the two when viewer unsubscribe races provider termination.
            (result,) = await asyncio.gather(self._task, return_exceptions=True)
            if isinstance(result, Exception):
                raise result
        # A task cancelled before its first step never runs its finally block.
        await self._dispose()
        self._on_closed(self.spec.route)


class TopicProvider:
    """One authenticated relay connection; all routes die with that connection."""

    def __init__(
        self,
        config: TopicServiceConfig,
        session: zenoh.Session,
        client: RelayClient,
        puts: TopicPuts,
        catalog: TopicCatalog | None = None,
    ) -> None:
        self.config = config
        self._session = session
        self._client = client
        self._puts = puts
        self._generation: int | float | None = None
        self._routes: dict[str, _Route] = {}
        self._catalog = catalog if catalog is not None else TopicCatalog()
        self._ingress_budget = TopicIngressBudget()
        self._send_budget = TopicByteBudget()
        self._transforms = TopicTransforms()
        self._guards: GuardedPublishers | None = None
        if config.write and config.guarded_publishers:
            loop = asyncio.get_running_loop()

            def emit(message: TopicMsg) -> None:
                loop.call_soon_threadsafe(client.send_control_frame, message)

            self._guards = GuardedPublishers(
                self._guard_put,
                emit,
            )

    def _guard_put(self, key: str, payload: bytes, encoding: str) -> None:
        self._session.put(
            key,
            payload,
            encoding=zenoh.Encoding(encoding),
            congestion_control=zenoh.CongestionControl.DROP,
            priority=zenoh.Priority.REAL_TIME,
            express=True,
        )

    def _error(self, request_id: str, code: str) -> None:
        self._client.send_control_frame(
            TopicError(id=request_id, code=code, message=code.replace("_", " "), outcome="rejected")
        )

    def _route_closed(self, route: str) -> None:
        self._routes.pop(route, None)

    async def run(self) -> None:
        async def control() -> None:
            async for message in self._client.control_messages():
                await self._message(message)

        # Three lanes, any failure ends the provider (a TaskGroup needs 3.11).
        lanes = [
            asyncio.create_task(self._publications()),
            asyncio.create_task(self._guard_frames()),
            asyncio.create_task(control()),
        ]
        try:
            if self._guards is not None:
                self._guards.start()
            done, _ = await asyncio.wait(lanes, return_when=asyncio.FIRST_EXCEPTION)
            for lane in done:
                lane.result()
        finally:
            for lane in lanes:
                lane.cancel()
            await asyncio.gather(*lanes, return_exceptions=True)
            if self._guards is not None:
                await asyncio.to_thread(self._guards.close)
            for route in list(self._routes.values()):
                await route.close()
            self._routes.clear()

    async def _guard_frames(self) -> None:
        async for frame in self._client.topic_publications(guarded=True):
            opening = (frame.header.meta or {}).get("action") == "open"
            try:
                meta = (
                    TopicGuardOpenMeta.model_validate(frame.header.meta)
                    if opening
                    else TopicGuardCommandMeta.model_validate(frame.header.meta)
                )
                if frame.header.delivery != "reliable":
                    raise ValueError("invalid guard delivery")
            except (ValidationError, ValueError):
                raise ProtocolError("invalid publisher provenance") from None
            code = ""
            if self._guards is None or meta.source != self.config.source:
                code = "guarded_publishers_disabled"
            elif meta.gen != self._generation:
                code = "stale_generation"
            elif isinstance(meta, TopicGuardOpenMeta):
                try:
                    zenoh.KeyExpr(meta.key)
                    if "*" in meta.key:
                        code = "concrete_key_required"
                except zenoh.ZError:
                    code = "invalid_key_expression"
            if code:
                self._client.send_control_frame(
                    TopicGuardClosed(handle=meta.handle, gen=meta.gen, code=code)
                )
                continue
            assert self._guards is not None
            if isinstance(meta, TopicGuardOpenMeta):
                self._guards.open(meta, frame.payload)
            elif frame.header.seq > 0 and int(frame.header.seq) == frame.header.seq:
                self._guards.command(meta, int(frame.header.seq), frame.payload)
            else:
                raise ProtocolError("invalid command sequence")

    async def _publications(self) -> None:
        async for frame in self._client.topic_publications():
            try:
                meta = TopicPublishMeta.model_validate(frame.header.meta)
            except ValidationError:
                # The relay stamps these; malformed provenance is a broken
                # connection, not a browser-selected import or decoder.
                raise ProtocolError("invalid topic publication provenance") from None
            code = ""
            outcome: Literal["accepted", "rejected", "unknown"] = "accepted"
            if not self.config.write or meta.source != self.config.source:
                code = "topic_write_denied"
            elif meta.gen != self._generation:
                code = "stale_generation"
            elif frame.header.delivery != "reliable":
                code = "invalid_topic_publication"
            else:
                try:
                    zenoh.KeyExpr(meta.key)
                    if "*" in meta.key:
                        code = "concrete_key_required"
                except zenoh.ZError:
                    code = "invalid_key_expression"
            if code:
                outcome = "rejected"
            else:
                outcome, code = await self._puts.put(
                    meta.key, frame.payload, meta.encoding, received_at=frame.received_at
                )
            self._client.send_control_frame(
                TopicPubResult(
                    id=meta.id,
                    outcome=outcome,
                    code=code,
                    message=code.replace("_", " "),
                    providerTs=time.time(),
                )
            )

    async def _message(self, message: Msg | DataFrame) -> None:
        if isinstance(message, (TopicGuardHeartbeat, TopicGuardStop)):
            if self._guards is not None and message.gen == self._generation:
                if isinstance(message, TopicGuardHeartbeat):
                    self._guards.heartbeat(message)
                else:
                    self._guards.stop(message.handle, message.gen)
        elif isinstance(message, TopicSources):
            if len(message.sources) != 1 or message.sources[0].source != self.config.source:
                raise ProtocolError("topic provider registration mismatch")
            generation = message.sources[0].gen
            if self._generation is not None and self._generation != generation:
                raise ProtocolError("topic provider generation changed on a live connection")
            self._generation = generation
        elif isinstance(message, TopicOpen):
            if not self.config.read:
                self._error(message.route, "topic_read_denied")
                return
            if message.gen != self._generation:
                self._error(message.route, "stale_generation")
                return
            if message.profile != "raw" and (
                not self.config.media or message.profile not in MEDIA_PROFILES
            ):
                self._error(message.route, "unsupported_profile")
                return
            if message.profile != "raw" and message.delivery != "latest":
                self._error(message.route, "profile_requires_latest")
                return
            if message.route in self._routes or len(self._routes) >= MAX_TOPIC_ROUTES:
                self._error(message.route, "topic_route_limit")
                return
            settings = None
            if message.settings is not None and message.profile != "raw":
                try:
                    settings = normalize_settings(
                        message.profile, message.settings.model_dump(exclude_none=True)
                    )
                except ValueError as error:
                    self._error(message.route, str(error))
                    return
            route = _Route(
                self._session,
                self._client,
                message,
                settings,
                self._ingress_budget,
                self._send_budget,
                self._catalog,
                self._transforms,
                self._route_closed,
            )
            self._routes[message.route] = route
            try:
                await route.start()
            except zenoh.ZError:
                await route.close()
                self._routes.pop(message.route, None)
                self._error(message.route, "invalid_key_expression")
                return
            self._client.send_control_frame(TopicOpened(route=message.route, gen=message.gen))
        elif isinstance(message, (TopicActivate, TopicClose, TopicVariant, TopicVariantClose)):
            existing_route = self._routes.get(message.route)
            if existing_route is None or message.gen != self._generation:
                return
            if isinstance(message, TopicActivate):
                existing_route.activate(float(message.maxHz) if message.maxHz is not None else None)
            elif isinstance(message, TopicClose):
                await existing_route.close()
                self._routes.pop(message.route, None)
            elif isinstance(message, TopicVariant):
                if existing_route.spec.profile == "raw":
                    return
                try:
                    settings = existing_route.set_variant(
                        message.variant, message.settings.model_dump(exclude_none=True)
                    )
                except ValueError as error:
                    self._error(message.route, str(error))
                    return
                self._client.send_control_frame(
                    TopicVariantReady(
                        route=message.route,
                        gen=message.gen,
                        variant=message.variant,
                        settings=TopicSettings(**settings),
                    )
                )
            else:
                await existing_route.close_variant(message.variant)
        elif isinstance(message, TopicList):
            if message.source != self.config.source or not self.config.read:
                self._error(message.id, "topic_read_denied")
                return
            if self._generation is None:
                self._error(message.id, "source_not_ready")
                return
            try:
                entries, cursor = self._catalog.page(
                    message.prefix, message.cursor, int(message.limit)
                )
            except ValueError:
                self._error(message.id, "catalog_changed")
                return
            self._client.send_control_frame(
                TopicPage(
                    id=message.id,
                    source=self.config.source,
                    gen=self._generation,
                    entries=entries,
                    cursor=cursor,
                )
            )


def capabilities(config: TopicServiceConfig) -> TopicCapabilities:
    """What this provider advertises at hello, ranges per profile included."""
    profiles = ["raw", *MEDIA_PROFILES] if config.media else ["raw"]
    return TopicCapabilities(
        read=config.read,
        write=config.write,
        profiles=profiles,
        guardedPublishers=config.write and config.guarded_publishers,
        profileOptions={name: PROFILE_OPTIONS[name] for name in profiles},
    )


async def serve(config: TopicServiceConfig, stop: asyncio.Event) -> None:
    """Own the direct Zenoh session, relay retries and all subscription cleanup."""
    zconfig = make_zenoh_config(config.zenoh)
    session = await asyncio.to_thread(zenoh.open, zconfig)
    puts = TopicPuts(session)
    stop_task = asyncio.create_task(stop.wait())
    catalog = TopicCatalog()
    metadata = MetadataReceiver(session, catalog)
    try:
        await metadata.start()
        while not stop.is_set():
            try:
                info = await fetch_relay_info(config.relay_url)
                async with await RelayClient.connect(info.wt_url, "robot") as client:
                    await client.hello(
                        topic_source=config.source,
                        token=None if config.token is None else config.token.get_secret_value(),
                        topic_capabilities=capabilities(config),
                    )
                    client.start_keepalive()
                    provider = TopicProvider(config, session, client, puts, catalog)
                    task = asyncio.create_task(provider.run())
                    logger.info("topic source connected", source=config.source)
                    try:
                        await asyncio.wait({task, stop_task}, return_when=asyncio.FIRST_COMPLETED)
                    finally:
                        task.cancel()
                        try:
                            await task
                        except asyncio.CancelledError:
                            pass
                        except ProtocolError as error:
                            # One bad message on a live connection drops that
                            # connection only. The next one starts clean.
                            logger.warning(
                                "topic relay sent a bad message; reconnecting",
                                source=config.source,
                                error=str(error),
                            )
            except RelayRejectedError as error:
                # A configuration/auth rejection needs an operator change, not
                # a hot retry loop. Never include credentials in the log.
                if error.code != "topic_source_conflict":
                    logger.error(
                        "topic provider registration rejected",
                        source=config.source,
                        code=error.code,
                    )
                    raise
                logger.warning("topic source still registered; retrying", source=config.source)
            except (OSError, asyncio.TimeoutError):  # OSError covers ConnectionError
                logger.warning("topic relay disconnected; retrying", source=config.source)
            if not stop.is_set():
                with suppress(asyncio.TimeoutError):
                    await asyncio.wait_for(asyncio.shield(stop_task), 1)
    finally:
        stop_task.cancel()
        await metadata.close()
        await asyncio.to_thread(session.close)
        await puts.wait_idle()


async def _main(config: TopicServiceConfig, *, exit_on_stdin_eof: bool = False) -> None:
    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGTERM, signal.SIGINT):
        loop.add_signal_handler(sig, stop.set)
    if exit_on_stdin_eof:

        def parent_input() -> None:
            if not os.read(sys.stdin.fileno(), 1024):
                loop.remove_reader(sys.stdin.fileno())
                stop.set()

        loop.add_reader(sys.stdin.fileno(), parent_input)
    try:
        await serve(config, stop)
    finally:
        if exit_on_stdin_eof:
            loop.remove_reader(sys.stdin.fileno())
        for sig in (signal.SIGTERM, signal.SIGINT):
            loop.remove_signal_handler(sig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument(
        "--exit-on-stdin-eof",
        action="store_true",
        help="stop when the supervising parent closes stdin",
    )
    args = parser.parse_args()
    try:
        config = TopicServiceConfig.model_validate_json(args.config.read_bytes())
    except (OSError, ValidationError):
        raise SystemExit("invalid or unreadable topic service configuration") from None
    # Desktop's pinned runtime includes libuv's event loop for batched UDP I/O.
    # Keep the standalone service usable with the full project's asyncio setup.
    with asyncio.Runner(loop_factory=_loop_factory) as runner:
        runner.run(_main(config, exit_on_stdin_eof=args.exit_on_stdin_eof))


if __name__ == "__main__":
    main()
