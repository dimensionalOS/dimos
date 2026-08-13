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

"""RelayBridgeModule: the robot side of the cockpit relay.

Registers the robot (id + channel manifest) with a relay - spawned locally in
--local-relay mode, or a remote one via relay_url - and forwards robot streams
to it. Encoding is lazy: an input is subscribed, and frames are encoded, only
while the relay reports at least one viewer subscribed to that channel, so a
robot with no open cockpit does no encode work at all.

Threading: input callbacks fire on the transport (LCM) thread, which gates on
maxHz and encodes there (RerunBridge precedent, ~3 ms per JPEG), then hands
the payload to the module event loop; all relay/session state lives on the
loop. The supervisor task consumes relay subs snapshots and survives relay
restarts (respawning the local child when it died).
"""

from __future__ import annotations

import asyncio
from collections.abc import AsyncIterator, Callable
from dataclasses import dataclass, field
import functools
import json
import socket
import threading
import time
from typing import Any, TypeVar
import webbrowser

from pydantic import Field

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.logging_config import setup_logger
from dimos.web.relay_bridge.locate import find_web_dir
from dimos.web.relay_bridge.manifest import parse_manifest
from dimos.web.relay_bridge.protocol import (
    Delivery,
    RobotInfo,
    RobotManifest,
    Subs,
)
from dimos.web.relay_bridge.relay_process import RelayProcess, ensure_cockpit_dist
from dimos.web.relay_bridge.wt_client import (
    RelayClient,
    RelayRejectedError,
    connect_with_backoff,
)

logger = setup_logger()

_T = TypeVar("_T")
_FrameMeta = dict[str, Any] | None
_Sender = Callable[[bytes, _FrameMeta], None]

_RECONNECT_PAUSE_S = 2.0

# A SIGKILLed relay child sends no CONNECTION_CLOSE, so the QUIC session only
# notices at idle timeout (tens of seconds). The child watchdog polls the
# process instead and force-closes the session to trigger a prompt respawn.
_CHILD_POLL_S = 1.0

# Bounded wait for the build worker thread after cancelling it (the child
# dies within the SIGTERM-to-SIGKILL grace; this adds a margin on top).
_BUILD_CANCEL_WAIT_S = 8.0


def _probe_local_port(port: int) -> None:
    if port == 0:
        return
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
            # SO_REUSEADDR matches how the relay itself binds: a live
            # listener still fails the probe, but the FIN_WAIT/TIME_WAIT
            # remnants of a just-killed relay (a browser tab was
            # attached) must not block an immediate restart.
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            probe.bind(("127.0.0.1", port))
    except OSError as e:
        raise RuntimeError(f"cannot start local relay: port {port} is unavailable") from e


async def _blocking_call(func: Callable[..., _T], *args: Any) -> _T:
    """Run blocking process work without abandoning its thread on cancellation."""
    work = asyncio.create_task(asyncio.to_thread(func, *args))
    cancelled: asyncio.CancelledError | None = None
    while not work.done():
        try:
            await asyncio.shield(work)
        except asyncio.CancelledError as exc:
            cancelled = cancelled or exc
        except BaseException:
            break
    try:
        result = work.result()
    except BaseException as exc:
        if cancelled is not None:
            raise cancelled from exc
        raise
    if cancelled is not None:
        raise cancelled
    return result


async def _cancel_task(task: asyncio.Task[None] | None, name: str) -> None:
    if task is None:
        return
    task.cancel()
    try:
        await task
    except asyncio.CancelledError:
        pass
    except Exception:
        # A task that already died re-raises here; teardown must continue.
        logger.exception(f"relay bridge {name} task failed during teardown")


class RelayBridgeConfig(ModuleConfig):
    relay_url: str | None = None
    """Attach to a running relay (wtUrl). None: spawn a local one."""
    local_port: int = 7780
    """HTTP port of the spawned local relay; 0 picks an ephemeral port (tests)."""
    open_browser: bool = True
    """Open the local relay's page once it is up (local mode only)."""
    cockpit_build: bool = True
    """Build the Cockpit dist before spawning the local relay when it is
    missing or stale (checkouts only; wheels ship it pre-built)."""
    robot_id: str = ""
    """Relay identity; empty falls back to g.robot_id, then the hostname."""
    robot_name: str = ""
    """Display name; empty falls back to robot_id."""
    jpeg_quality: int = 75
    image_max_hz: float = Field(default=30.0, gt=0.0)
    odom_max_hz: float = Field(default=20.0, gt=0.0)
    available_channels: tuple[str, ...] | None = None
    """Composition-provided channel allowlist; None derives from bound inputs."""


def _encode_image(module: RelayBridgeModule, msg: Image) -> tuple[bytes, dict[str, Any] | None]:
    # TurboJPEG via the message's own encoder (handles BGR/RGB/gray inputs).
    return (
        msg.to_jpeg_bytes(quality=module.config.jpeg_quality),
        {"w": msg.width, "h": msg.height},
    )


def _encode_odom(
    module: RelayBridgeModule, msg: PoseStamped
) -> tuple[bytes, dict[str, Any] | None]:
    pose = {
        "x": msg.position.x,
        "y": msg.position.y,
        "z": msg.position.z,
        "yaw": msg.yaw,
        "ts": msg.ts,
    }
    return json.dumps(pose, separators=(",", ":")).encode(), None


@dataclass(frozen=True)
class ChannelDef:
    ch: str
    encoding: str
    delivery: Delivery
    max_hz: Callable[[RelayBridgeConfig], float]
    encode: Callable[[RelayBridgeModule, Any], tuple[bytes, _FrameMeta]]


@dataclass(slots=True)
class _Session:
    client: RelayClient
    senders: dict[str, _Sender]
    last_n: int | float = 0
    unsubs: dict[str, Callable[[], None]] = field(default_factory=dict)
    retired: threading.Event = field(default_factory=threading.Event)


# The v0 channel table; every entry needs a matching `In` on the module.
CHANNELS: tuple[ChannelDef, ...] = (
    ChannelDef("color_image", "jpeg.v1", "latest", lambda c: c.image_max_hz, _encode_image),
    ChannelDef("odom", "pose.json.v1", "reliable", lambda c: c.odom_max_hz, _encode_odom),
)


def build_manifest(config: RelayBridgeConfig, channels: tuple[ChannelDef, ...]) -> RobotManifest:
    # Routed through the domain parser so an invalid channel table (duplicate
    # ids, bad rates) fails module start instead of poisoning the relay.
    manifest = parse_manifest(
        {
            "channels": [
                {
                    "ch": cd.ch,
                    "encoding": cd.encoding,
                    "delivery": cd.delivery,
                    "maxHz": cd.max_hz(config),
                }
                for cd in channels
            ]
        }
    )
    return RobotManifest(channels=manifest.channels)


def resolve_robot_info(config: RelayBridgeConfig) -> RobotInfo:
    robot_id = config.robot_id or config.g.robot_id or socket.gethostname()
    return RobotInfo(
        id=robot_id,
        name=config.robot_name or robot_id,
        model=config.g.robot_model or "",
    )


class RelayBridgeModule(Module):
    """Bridges robot streams to the relay; encodes only while viewers watch."""

    config: RelayBridgeConfig
    # Exact producer types (GO2Connection outputs) so autoconnect matches.
    color_image: In[Image]
    odom: In[PoseStamped]
    # NEVER add handle_color_image/handle_odom methods here: _auto_bind_handlers
    # subscribes any handle_<input> eagerly at start(), defeating lazy encode.

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._relay: RelayProcess | None = None
        self._build_cancel: threading.Event | None = None
        self._session: _Session | None = None
        self._url: str | None = None
        self._robot_info: RobotInfo | None = None
        self._manifest: RobotManifest | None = None
        self._channel_defs: tuple[ChannelDef, ...] = ()
        self._min_interval: dict[str, float] = {}
        self._last_input: dict[str, float] = {}
        self.encoded: dict[str, int] = {cd.ch: 0 for cd in CHANNELS}

    async def main(self) -> AsyncIterator[None]:
        supervisor: asyncio.Task[None] | None = None
        try:
            self._robot_info = resolve_robot_info(self.config)
            allowed = (
                None
                if self.config.available_channels is None
                else frozenset(self.config.available_channels)
            )
            self._channel_defs = tuple(
                cd
                for cd in CHANNELS
                if (allowed is None or cd.ch in allowed)
                and self.inputs[cd.ch].transport is not None
            )
            self._min_interval = {cd.ch: 1.0 / cd.max_hz(self.config) for cd in self._channel_defs}
            self._manifest = build_manifest(self.config, self._channel_defs)
            self._url = self.config.relay_url or self.config.g.relay_url
            if self._url is None:
                # Probe before the (expensive) build: a start that will lose
                # the port must not rewrite the dist a running relay serves.
                _probe_local_port(self.config.local_port)
                if self.config.cockpit_build:
                    try:
                        await self._build_cockpit()
                    except Exception:
                        logger.exception("cockpit build failed; continuing with the relay only")
                self._url = await _blocking_call(self._spawn_relay, self.config.open_browser)
            # The first connect fails fast: a relay that cannot be reached at
            # startup should fail the module start visibly, not retry forever.
            session = await self._connect_and_hello()
            self._session = session
            supervisor = asyncio.create_task(self._supervise(session))
            logger.info(f"relay bridge up: robot={self._robot_info.id} relay={self._url}")
            yield
        finally:
            try:
                await _cancel_task(supervisor, "supervisor")
                await self._disconnect()
            finally:
                if self._relay is not None:
                    try:
                        await _blocking_call(self._relay.stop)
                    except Exception:
                        # An orphaned Deno child would outlive this process holding
                        # its port (it has no PDEATHSIG), so surface cleanup failure.
                        logger.exception("relay bridge: stopping the local relay failed")

    async def _build_cockpit(self) -> None:
        """Build the Cockpit dist (checkouts only) before spawning the relay.

        The blocking build runs in a thread but stays cancellable: on
        cancellation (or module close, via _close_module) the build child is
        killed and the worker reaped within a bounded grace, so teardown
        never waits out the 600 s build timeout.
        """
        cancel = threading.Event()
        self._build_cancel = cancel
        work = asyncio.create_task(
            asyncio.to_thread(ensure_cockpit_dist, find_web_dir(), cancel=cancel)
        )
        try:
            await asyncio.shield(work)
        except (asyncio.CancelledError, GeneratorExit):
            # Generator finalization (aclose) delivers GeneratorExit instead
            # of CancelledError; both mean the same thing here.
            cancel.set()
            await asyncio.wait({work}, timeout=_BUILD_CANCEL_WAIT_S)
            raise
        finally:
            self._build_cancel = None

    def _close_module(self) -> None:
        # Runs on every stop path, including a stop() racing a still-starting
        # main(): an in-flight cockpit build must die now, not at its timeout.
        cancel = self._build_cancel
        if cancel is not None:
            cancel.set()
        super()._close_module()

    def _spawn_relay(self, open_browser: bool) -> str:
        """Start a fresh local relay child (blocking; run via to_thread)."""
        _probe_local_port(self.config.local_port)
        self._relay = RelayProcess(port=self.config.local_port)
        info = self._relay.start()
        logger.info(f"local relay ready: {info.open_url}")
        if open_browser:
            webbrowser.open_new_tab(info.open_url)
        return info.wt_url

    async def _connect_and_hello(self) -> _Session:
        assert self._url is not None and self._robot_info is not None and self._manifest is not None
        client = await connect_with_backoff(self._url, "robot", max_attempts=4)
        try:
            await client.hello(robot=self._robot_info, manifest=self._manifest)
            senders = self._build_senders(client)
        except BaseException:
            try:
                await client.close()
            except Exception:
                logger.exception("relay bridge: closing a failed relay session failed")
            raise
        return _Session(client, senders)

    def _build_senders(self, client: RelayClient) -> dict[str, _Sender]:
        senders: dict[str, _Sender] = {}
        for cd in self._channel_defs:
            if cd.delivery == "latest":
                senders[cd.ch] = client.latest_writer(cd.ch).offer
            else:
                senders[cd.ch] = functools.partial(self._send_reliable, client, cd.ch)
        return senders

    def _send_reliable(
        self, client: RelayClient, ch: str, payload: bytes, meta: dict[str, Any] | None
    ) -> None:
        client.send_frame(ch, payload, delivery="reliable", meta=meta)

    async def _supervise(self, session: _Session) -> None:
        """Consume subs snapshots; on session loss, reconnect (and respawn a
        dead local relay child) until the module stops."""
        watchdog: asyncio.Task[None] | None = None
        try:
            if self._relay is not None:
                watchdog = asyncio.create_task(self._watch_child())
            while True:
                crashed = False
                try:
                    async for msg in session.client.control_messages():
                        if isinstance(msg, Subs) and msg.n > session.last_n:
                            session.last_n = msg.n
                            self._reconcile(session, set(msg.chs))
                    # The iterator only ends when the session closed.
                except Exception:
                    # An unguarded error here would silently end supervision while
                    # the module stays "up" (never CancelledError: stop() cancels).
                    crashed = True
                    logger.exception("relay bridge supervisor error; recycling the relay session")
                await self._disconnect(session)
                if crashed:
                    # Reconnect only pauses on FAILED connects; without this a
                    # persistent reconcile error would recycle at handshake speed.
                    await asyncio.sleep(_RECONNECT_PAUSE_S)
                logger.warning("relay session lost; encoders stopped, reconnecting")
                reconnected = await self._reconnect()
                if reconnected is None:
                    return
                session = reconnected
                self._session = session
        finally:
            await _cancel_task(watchdog, "watchdog")
            await self._disconnect(session)

    async def _watch_child(self) -> None:
        """Close the session promptly when the local relay child dies (a kill
        sends no CONNECTION_CLOSE; waiting for QUIC idle timeout is too slow).
        The supervisor then respawns and reconnects."""
        while True:
            await asyncio.sleep(_CHILD_POLL_S)
            relay, session = self._relay, self._session
            if (
                relay is not None
                and not relay.is_running()
                and session is not None
                and not session.client.is_closed
            ):
                logger.warning("local relay child died; closing the session to reconnect")
                await session.client.close()

    async def _reconnect(self) -> _Session | None:
        while True:
            if self._relay is not None and not self._relay.is_running():
                # The child is gone (crash, kill, or a previous respawn that
                # failed): its QUIC port and cert die with it, so respawn and
                # re-read the ready line. The browser page reconnects itself
                # via the stable HTTP port. `not is_running()` rather than
                # `poll() is not None`: a failed start leaves no process and
                # poll() would read None forever, latching respawns off.
                logger.warning("local relay child died; respawning")
                try:
                    await _blocking_call(self._relay.stop)
                    self._url = await _blocking_call(self._spawn_relay, False)
                except Exception:
                    logger.exception("relay respawn failed; retrying")
                    await asyncio.sleep(_RECONNECT_PAUSE_S)
                    continue
            try:
                return await self._connect_and_hello()
            except RelayRejectedError as e:
                logger.error(f"relay rejected reconnect ({e.code}: {e.message}); not retrying")
                return None
            except Exception as e:
                logger.warning(f"relay reconnect failed ({e}); retrying")
                await asyncio.sleep(_RECONNECT_PAUSE_S)

    def _reconcile(self, session: _Session, want: set[str]) -> None:
        """Subscribe/unsubscribe inputs so exactly `want` is being encoded."""
        for cd in self._channel_defs:
            active = cd.ch in session.unsubs
            should = cd.ch in want
            if should and not active:
                session.unsubs[cd.ch] = self.inputs[cd.ch].subscribe(
                    functools.partial(self._on_input, session, cd, session.senders[cd.ch])
                )
                logger.info(f"relay bridge: viewer subscribed to {cd.ch}; encoding started")
            elif active and not should:
                unsubscribe = session.unsubs[cd.ch]
                unsubscribe()
                del session.unsubs[cd.ch]
                logger.info(f"relay bridge: no viewers on {cd.ch}; encoding stopped")
        unknown = want - {cd.ch for cd in self._channel_defs}
        if unknown:
            logger.debug(f"relay bridge: ignoring unknown channels {sorted(unknown)}")

    def _on_input(self, session: _Session, cd: ChannelDef, sender: _Sender, msg: Any) -> None:
        """Transport-thread callback: maxHz gate, encode, hand to the loop."""
        if session.retired.is_set():
            return
        now = time.monotonic()
        if now - self._last_input.get(cd.ch, 0.0) < self._min_interval[cd.ch]:
            return
        self._last_input[cd.ch] = now
        try:
            payload, meta = cd.encode(self, msg)
        except Exception:
            logger.exception(f"relay bridge: encoding {cd.ch} failed")
            return
        self.encoded[cd.ch] += 1
        loop = self._loop
        if loop is not None and loop.is_running():
            loop.call_soon_threadsafe(self._offer, session, sender, payload, meta)

    def _offer(self, session: _Session, sender: _Sender, payload: bytes, meta: _FrameMeta) -> None:
        if session.retired.is_set() or self._session is not session:
            return
        try:
            sender(payload, meta)
        except Exception:
            # Session mid-teardown (dead writer pump / closed connection): the
            # supervisor is already reconnecting and will rebuild the senders.
            return

    async def _disconnect(self, session: _Session | None = None) -> None:
        target = self._session if session is None else session
        if target is None:
            return
        target.retired.set()
        if self._session is target:
            self._session = None
        for ch, unsubscribe in tuple(target.unsubs.items()):
            try:
                unsubscribe()
            except Exception:
                logger.exception(f"relay bridge: unsubscribing {ch} failed")
            finally:
                target.unsubs.pop(ch, None)
        try:
            await target.client.close()
        except Exception:
            logger.exception("relay bridge: closing the relay session failed")


def with_relay_bridge(blueprint: Blueprint) -> Blueprint:
    """Append a relay bridge wired to the blueprint's channel producers."""
    # An external or explicitly composed blueprint may already own a customized
    # relay bridge. Preserve that atom rather than overriding its kwargs.
    if any(atom.module is RelayBridgeModule for atom in blueprint.blueprints):
        return blueprint

    producer_keys: set[tuple[str, type]] = set()
    for atom in blueprint.active_blueprints:
        for stream in atom.streams:
            if stream.direction != "out":
                continue
            effective_name = blueprint.remapping_map.get((atom.name, stream.name), stream.name)
            if isinstance(effective_name, str):
                producer_keys.add((effective_name, stream.type))

    bridge_atom = RelayBridgeModule.blueprint().blueprints[0]
    bridge_input_types = {
        stream.name: stream.type for stream in bridge_atom.streams if stream.direction == "in"
    }
    available_channels = tuple(
        channel.ch
        for channel in CHANNELS
        if (channel_type := bridge_input_types.get(channel.ch)) is not None
        and (channel.ch, channel_type) in producer_keys
    )
    return autoconnect(
        blueprint,
        RelayBridgeModule.blueprint(available_channels=available_channels),
    )
