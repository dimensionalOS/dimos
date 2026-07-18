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
import contextlib
from dataclasses import dataclass
import functools
import json
import socket
import time
from typing import Any
import webbrowser

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.logging_config import setup_logger
from dimos.web.relay_bridge.protocol import (
    ChannelSpec,
    Delivery,
    RobotInfo,
    RobotManifest,
    Subs,
)
from dimos.web.relay_bridge.relay_process import RelayProcess, kill_stale_port_holder
from dimos.web.relay_bridge.wt_client import RelayClient, connect_with_backoff

logger = setup_logger()

_RECONNECT_PAUSE_S = 2.0

# A SIGKILLed relay child sends no CONNECTION_CLOSE, so the QUIC session only
# notices at idle timeout (tens of seconds). The child watchdog polls the
# process instead and force-closes the session to trigger a prompt respawn.
_CHILD_POLL_S = 1.0


class RelayBridgeConfig(ModuleConfig):
    relay_url: str | None = None
    """Attach to a running relay (wtUrl). None: spawn a local one."""
    local_port: int = 7780
    """HTTP port of the spawned local relay; 0 picks an ephemeral port (tests)."""
    open_browser: bool = True
    """Open the local relay's page once it is up (local mode only)."""
    robot_id: str = ""
    """Relay identity; empty falls back to g.robot_id, then the hostname."""
    robot_name: str = ""
    """Display name; empty falls back to robot_id."""
    jpeg_quality: int = 75
    image_max_hz: float = 15.0  # go2 publishes ~14 Hz
    odom_max_hz: float = 20.0


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
    encode: Callable[[RelayBridgeModule, Any], tuple[bytes, dict[str, Any] | None]]


# The v0 channel table; every entry needs a matching `In` on the module. A
# blueprint lacking one of these streams still works: the channel is
# advertised, never fires, and viewers see 0 Hz.
CHANNELS: tuple[ChannelDef, ...] = (
    ChannelDef("color_image", "jpeg.v1", "latest", lambda c: c.image_max_hz, _encode_image),
    ChannelDef("odom", "pose.json.v1", "reliable", lambda c: c.odom_max_hz, _encode_odom),
)


def build_manifest(config: RelayBridgeConfig) -> RobotManifest:
    return RobotManifest(
        channels=[
            ChannelSpec(
                ch=cd.ch, encoding=cd.encoding, delivery=cd.delivery, maxHz=cd.max_hz(config)
            )
            for cd in CHANNELS
        ]
    )


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
        self._client: RelayClient | None = None
        self._url: str | None = None
        self._robot_info: RobotInfo | None = None
        self._manifest: RobotManifest | None = None
        self._supervisor: asyncio.Task[None] | None = None
        self._watchdog: asyncio.Task[None] | None = None
        self._stopping = False
        self._last_n: int | float = 0
        # ch -> sender(payload, meta); rebuilt for every relay session.
        self._senders: dict[str, Callable[[bytes, dict[str, Any] | None], None]] = {}
        # ch -> unsubscribe; present only while >= 1 viewer wants the channel.
        self._unsubs: dict[str, Callable[[], None]] = {}
        self._last_input: dict[str, float] = {}
        self.encoded: dict[str, int] = {cd.ch: 0 for cd in CHANNELS}
        self.sent: dict[str, int] = {cd.ch: 0 for cd in CHANNELS}

    async def main(self) -> AsyncIterator[None]:
        self._robot_info = resolve_robot_info(self.config)
        self._manifest = build_manifest(self.config)
        self._url = self.config.relay_url or self.config.g.relay_url
        if self._url is None:
            self._url = await asyncio.to_thread(self._spawn_relay, self.config.open_browser)
        # The first connect fails fast: a relay that cannot be reached at
        # startup should fail the module start visibly, not retry forever.
        try:
            self._client = await self._connect_and_hello()
        except BaseException:
            # A start that dies pre-yield never reaches the teardown below;
            # reap the child we just spawned (BaseException: cancellation too).
            if self._relay is not None:
                await asyncio.to_thread(self._relay.stop)
            raise
        self._supervisor = asyncio.ensure_future(self._supervise())
        if self._relay is not None:
            self._watchdog = asyncio.ensure_future(self._watch_child())
        logger.info(f"relay bridge up: robot={self._robot_info.id} relay={self._url}")
        yield
        self._stopping = True
        try:
            for task in (self._supervisor, self._watchdog):
                if task is None:
                    continue
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
                except Exception:
                    # A task that already died with its own exception re-raises
                    # it here; teardown must keep going regardless.
                    logger.exception("relay bridge task failed during teardown")
            try:
                self._reconcile(set())
            except Exception:
                logger.exception("relay bridge: unsubscribing inputs during stop failed")
            if self._client is not None:
                try:
                    await self._client.close()
                except Exception:
                    logger.exception("relay bridge: closing the relay session failed")
        finally:
            # Always reached: an orphaned Deno child would outlive this process
            # holding its port (it has no PDEATHSIG).
            if self._relay is not None:
                await asyncio.to_thread(self._relay.stop)

    def _spawn_relay(self, open_browser: bool) -> str:
        """Start a fresh local relay child (blocking; run via to_thread)."""
        if self.config.local_port != 0:
            kill_stale_port_holder(self.config.local_port)
        self._relay = RelayProcess(port=self.config.local_port)
        info = self._relay.start()
        logger.info(f"local relay ready: {info.debug_url}")
        if open_browser:
            webbrowser.open_new_tab(info.debug_url)
        return info.wt_url

    async def _connect_and_hello(self) -> RelayClient:
        assert self._url is not None and self._robot_info is not None
        client = await connect_with_backoff(self._url, "robot", max_attempts=4)
        try:
            await client.hello(robot=self._robot_info, manifest=self._manifest)
        except BaseException:
            await client.close()
            raise
        self._last_n = 0
        self._build_senders(client)
        return client

    def _build_senders(self, client: RelayClient) -> None:
        self._senders = {}
        for cd in CHANNELS:
            if cd.delivery == "latest":
                self._senders[cd.ch] = client.latest_writer(cd.ch).offer
            else:
                self._senders[cd.ch] = functools.partial(self._send_reliable, client, cd.ch)

    def _send_reliable(
        self, client: RelayClient, ch: str, payload: bytes, meta: dict[str, Any] | None
    ) -> None:
        client.send_frame(ch, payload, delivery="reliable", meta=meta)

    async def _supervise(self) -> None:
        """Consume subs snapshots; on session loss, reconnect (and respawn a
        dead local relay child) until the module stops."""
        client = self._client
        assert client is not None
        while True:
            crashed = False
            try:
                async for msg in client.control_messages():
                    if isinstance(msg, Subs) and msg.n > self._last_n:
                        self._last_n = msg.n
                        self._reconcile(set(msg.chs))
                # The iterator only ends when the session closed.
            except Exception:
                # An unguarded error here would silently end supervision while
                # the module stays "up" (never CancelledError: stop() cancels).
                crashed = True
                logger.exception("relay bridge supervisor error; recycling the relay session")
            # Release the session before replacing it: only close() reaches
            # aioquic's transport.close(), else the UDP socket leaks until
            # cycle GC. Idempotent, so the watchdog/teardown may close too.
            with contextlib.suppress(Exception):
                await client.close()
            try:
                self._reconcile(set())
            except Exception:
                logger.exception("relay bridge: stopping encoders failed")
            if self._stopping:
                return
            if crashed:
                # Reconnect only pauses on FAILED connects; without this a
                # persistent reconcile error would recycle at handshake speed.
                await asyncio.sleep(_RECONNECT_PAUSE_S)
            logger.warning("relay session lost; encoders stopped, reconnecting")
            reconnected = await self._reconnect()
            if reconnected is None:
                return
            client = self._client = reconnected

    async def _watch_child(self) -> None:
        """Close the session promptly when the local relay child dies (a kill
        sends no CONNECTION_CLOSE; waiting for QUIC idle timeout is too slow).
        The supervisor then respawns and reconnects."""
        while not self._stopping:
            await asyncio.sleep(_CHILD_POLL_S)
            relay, client = self._relay, self._client
            if (
                relay is not None
                and not relay.is_running()
                and client is not None
                and not client.is_closed
            ):
                logger.warning("local relay child died; closing the session to reconnect")
                await client.close()

    async def _reconnect(self) -> RelayClient | None:
        while not self._stopping:
            if self._relay is not None and not self._relay.is_running():
                # The child is gone (crash, kill, or a previous respawn that
                # failed): its QUIC port and cert die with it, so respawn and
                # re-read the ready line. The browser page reconnects itself
                # via the stable HTTP port. `not is_running()` rather than
                # `poll() is not None`: a failed start leaves no process and
                # poll() would read None forever, latching respawns off.
                logger.warning("local relay child died; respawning")
                try:
                    await asyncio.to_thread(self._relay.stop)
                    self._url = await asyncio.to_thread(self._spawn_relay, False)
                except Exception:
                    logger.exception("relay respawn failed; retrying")
                    await asyncio.sleep(_RECONNECT_PAUSE_S)
                    continue
            try:
                return await self._connect_and_hello()
            except Exception as e:
                logger.warning(f"relay reconnect failed ({e}); retrying")
                await asyncio.sleep(_RECONNECT_PAUSE_S)
        return None

    def _reconcile(self, want: set[str]) -> None:
        """Subscribe/unsubscribe inputs so exactly `want` is being encoded."""
        for cd in CHANNELS:
            active = cd.ch in self._unsubs
            should = cd.ch in want
            if should and not active:
                if self.inputs[cd.ch].transport is None:
                    # Advertised but unwired (standalone wiring): In.subscribe
                    # would raise. Skipping self-heals on the next snapshot.
                    logger.warning(
                        f"relay bridge: {cd.ch} input has no transport wired; cannot encode"
                    )
                    continue
                self._unsubs[cd.ch] = self.inputs[cd.ch].subscribe(
                    functools.partial(self._on_input, cd)
                )
                logger.info(f"relay bridge: viewer subscribed to {cd.ch}; encoding started")
            elif active and not should:
                self._unsubs.pop(cd.ch)()
                logger.info(f"relay bridge: no viewers on {cd.ch}; encoding stopped")
        unknown = want - {cd.ch for cd in CHANNELS}
        if unknown:
            logger.debug(f"relay bridge: ignoring unknown channels {sorted(unknown)}")

    def _on_input(self, cd: ChannelDef, msg: Any) -> None:
        """Transport-thread callback: maxHz gate, encode, hand to the loop."""
        now = time.monotonic()
        if now - self._last_input.get(cd.ch, 0.0) < 1.0 / cd.max_hz(self.config):
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
            loop.call_soon_threadsafe(self._offer, cd.ch, payload, meta)

    def _offer(self, ch: str, payload: bytes, meta: dict[str, Any] | None) -> None:
        sender = self._senders.get(ch)
        if sender is None:
            return
        try:
            sender(payload, meta)
        except Exception:
            # Session mid-teardown (dead writer pump / closed connection): the
            # supervisor is already reconnecting and will rebuild the senders.
            return
        self.sent[ch] += 1
