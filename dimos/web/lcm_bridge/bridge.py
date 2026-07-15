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

"""LCM <-> WebSocket bridge: the dimos bus, in the browser.

Both directions speak the standard LCM small-message wire format
``[magic u32 BE][seq u32 BE][channel utf-8][\\0][payload]``, so
``@dimos/msgs`` in the browser decodes bus traffic unchanged, and messages
the browser publishes are indistinguishable from any other bus peer's.
``static/lcm_client.js`` is the matching browser client.

Extracted from the Babylon scene viewer's ``/lcm-ws`` endpoint, where this
design accumulated its flow control under real load (a Rust voxel mapper
publishing multi-MB pointclouds at 10 Hz into a browser tab):

- **Latest-wins buffering.** One slot per (client, channel). When the bus
  emits faster than a client's socket drains, the slot is overwritten and
  old packets drop on the floor — the browser gets the freshest state at
  its own drain rate. The naive alternative (queue one send per packet)
  backed asyncio up with hundreds of pending sends and put the browser
  10-15 s behind real time.
- **One in-flight send per client.** A single drain task owns each socket's
  send side; a stalled client times out and is closed so its browser-side
  auto-reconnect fires, instead of holding buffers hostage.
- **Per-channel rate caps.** Server-side throttling of the WebSocket leg
  only — bus consumers are unaffected. Note: a capped packet is dropped,
  not deferred, so a channel that goes quiet right after a burst can leave
  the client one update stale until the next publish.

This is deliberately a server-level v0 of the QoS story sketched in the
Dimos Web proposals (#2708, #2710) and the TS API spec (#2502): rate caps
and allow/block lists are bridge configuration here, not yet per-connection
requests from the client.
"""

from __future__ import annotations

import asyncio
from collections.abc import Callable, Mapping
from contextlib import suppress
from fnmatch import fnmatchcase
import struct
import threading
import time

import lcm as lcmlib
from starlette.routing import WebSocketRoute
from starlette.websockets import WebSocket, WebSocketDisconnect

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

#: Magic prefix of the LCM small-message (non-fragmented) packet, "LC02".
LCM_MAGIC_SHORT = 0x4C433032

#: Fragmented LCM messages (>~64 KB) use a different magic and are not yet
#: bridged; big payloads (video, raw pointclouds) should be encoded or
#: decimated server-side before they reach the bus channel a browser reads.
_WS_SEND_TIMEOUT_S = 1.0
_WS_CLOSE_TIMEOUT_S = 0.5


def encode_packet(channel: str, payload: bytes, seq: int = 0) -> bytes:
    """Frame ``payload`` as an LCM small-message packet for ``channel``."""
    return struct.pack(">II", LCM_MAGIC_SHORT, seq) + channel.encode("utf-8") + b"\x00" + payload


def decode_packet(packet: bytes) -> tuple[str, bytes] | None:
    """Parse an LCM small-message packet into ``(channel, payload)``.

    Returns None for anything that isn't a well-formed small-message
    packet (wrong magic, truncated header, unterminated channel).
    """
    if len(packet) < 8:
        return None
    magic, _seq = struct.unpack(">II", packet[:8])
    if magic != LCM_MAGIC_SHORT:
        return None
    try:
        null_idx = packet.index(b"\x00", 8)
    except ValueError:
        return None
    channel = packet[8:null_idx].decode("utf-8", errors="replace")
    return channel, packet[null_idx + 1 :]


def _topic_of(channel: str) -> str:
    # dimos LCM channels are "<topic>#<package.MessageType>".
    return channel.split("#", 1)[0]


class LcmWebSocketBridge:
    """Bridges every LCM bus channel to browser WebSocket clients.

    Embeddable: a host Starlette app adds :meth:`routes` next to its own
    (the Babylon viewer pattern), or
    :class:`dimos.web.lcm_bridge.module.LcmWebSocketBridgeModule` runs it
    as a standalone server. Call :meth:`start` before serving and
    :meth:`stop` on shutdown.

    ``topic_allowlist``/``topic_blocklist`` take :mod:`fnmatch` patterns
    matched against both the topic (``/odom``) and the full channel
    (``/odom#geometry_msgs.PoseStamped``); the blocklist wins. Filtering
    applies to the bus->browser direction only. ``channel_rate_hz`` maps
    the same kind of pattern to a per-client forward-rate cap.

    ``on_bus_message`` is a tap called with ``(channel, payload)`` for
    every bus message on the LCM reader thread — hosts use it to observe
    traffic they'd otherwise need a second subscribe-all handle for. It
    must not block.
    """

    def __init__(
        self,
        *,
        lcm_url: str | None = None,
        channel_rate_hz: Mapping[str, float] | None = None,
        topic_allowlist: list[str] | None = None,
        topic_blocklist: list[str] | None = None,
        on_bus_message: Callable[[str, bytes], None] | None = None,
        ws_path: str = "/lcm-ws",
    ) -> None:
        self._lcm_url = lcm_url
        self._channel_rate_hz = dict(channel_rate_hz or {})
        self._topic_allowlist = list(topic_allowlist) if topic_allowlist is not None else None
        self._topic_blocklist = list(topic_blocklist or [])
        self._on_bus_message = on_bus_message
        self._ws_path = ws_path

        self._lcm: lcmlib.LCM | None = None
        self._subscription: object | None = None
        self._handle_thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._seq = 0

        self._clients: set[WebSocket] = set()
        # Per-client latest-packet-per-channel slot + wake event + drain
        # task. The LCM thread overwrites slots (latest-wins); each drain
        # task is the only sender on its socket, so sends never compete.
        self._pending: dict[WebSocket, dict[str, bytes]] = {}
        self._wake: dict[WebSocket, asyncio.Event] = {}
        self._drain_tasks: dict[WebSocket, asyncio.Task[None]] = {}
        self._last_forward_ts: dict[WebSocket, dict[str, float]] = {}

        # Per-channel filter/rate decisions are pattern matches; cache them
        # by exact channel so the hot path is two dict lookups.
        self._forward_cache: dict[str, bool] = {}
        self._min_interval_cache: dict[str, float] = {}

        # Debug counters, exposed by the standalone module's status page.
        self.forwarded = 0
        self.rate_capped = 0
        self.filtered = 0
        self.published_from_clients = 0

    # ---- lifecycle -------------------------------------------------------

    def start(self) -> None:
        """Open the LCM handle and start the bus reader thread."""
        self._lcm = lcmlib.LCM(self._lcm_url) if self._lcm_url else lcmlib.LCM()
        subscription = self._lcm.subscribe(".*", self._on_bus_msg)
        subscription.set_queue_capacity(10000)
        self._subscription = subscription
        self._handle_thread = threading.Thread(
            target=self._handle_loop,
            name="lcm-ws-bridge",
            daemon=True,
        )
        self._handle_thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._handle_thread is not None and self._handle_thread.is_alive():
            self._handle_thread.join(timeout=2.0)

    def routes(self) -> list[WebSocketRoute]:
        """Starlette routes to mount on the serving app."""
        return [WebSocketRoute(self._ws_path, self.websocket_endpoint)]

    @property
    def lcm(self) -> lcmlib.LCM | None:
        """The bridge's bus handle (None before :meth:`start`)."""
        return self._lcm

    @property
    def client_count(self) -> int:
        return len(self._clients)

    # ---- bus -> browser --------------------------------------------------

    def _handle_loop(self) -> None:
        assert self._lcm is not None
        while not self._stop_event.is_set():
            try:
                self._lcm.handle_timeout(100)
            except Exception:
                logger.exception("lcm-ws bridge: bus handle failed")

    def _on_bus_msg(self, channel: str, data: bytes) -> None:
        # Runs on the LCM reader thread. Framing and fan-out happen on the
        # asyncio loop via call_soon_threadsafe so the per-client drain
        # serialises naturally.
        if self._on_bus_message is not None:
            try:
                self._on_bus_message(channel, data)
            except Exception:
                logger.exception("lcm-ws bridge: on_bus_message tap failed")
        if not self._clients:
            return
        if not self._forward_allowed(channel):
            self.filtered += 1
            return
        loop = self._loop
        if loop is None:
            return
        loop.call_soon_threadsafe(self._enqueue, channel, data)

    def _forward_allowed(self, channel: str) -> bool:
        cached = self._forward_cache.get(channel)
        if cached is not None:
            return cached
        topic = _topic_of(channel)

        def _matches(patterns: list[str]) -> bool:
            return any(
                fnmatchcase(topic, pattern) or fnmatchcase(channel, pattern) for pattern in patterns
            )

        allowed = not _matches(self._topic_blocklist) and (
            self._topic_allowlist is None or _matches(self._topic_allowlist)
        )
        self._forward_cache[channel] = allowed
        return allowed

    def _min_interval_s(self, channel: str) -> float:
        cached = self._min_interval_cache.get(channel)
        if cached is not None:
            return cached
        topic = _topic_of(channel)
        interval = 0.0
        for pattern, rate_hz in self._channel_rate_hz.items():
            if rate_hz > 0 and (fnmatchcase(topic, pattern) or fnmatchcase(channel, pattern)):
                interval = max(interval, 1.0 / rate_hz)
        self._min_interval_cache[channel] = interval
        return interval

    def _enqueue(self, channel: str, data: bytes) -> None:
        # Runs on the asyncio loop thread. Synthesise the wire bytes once,
        # then for each client overwrite its latest-packet slot for this
        # channel and wake its drain task.
        if not self._clients:
            return
        min_interval = self._min_interval_s(channel)
        now = time.monotonic() if min_interval > 0.0 else 0.0
        packet = encode_packet(channel, data, self._seq)
        self._seq = (self._seq + 1) & 0xFFFFFFFF
        for websocket in tuple(self._clients):
            pending = self._pending.get(websocket)
            wake = self._wake.get(websocket)
            if pending is None or wake is None:
                continue
            if min_interval > 0.0:
                last = self._last_forward_ts.get(websocket, {})
                if now - last.get(channel, 0.0) < min_interval:
                    self.rate_capped += 1
                    continue  # rate-cap: drop this packet for this client
                last[channel] = now
            pending[channel] = packet
            wake.set()
        self.forwarded += 1

    async def _drain_loop(self, websocket: WebSocket) -> None:
        # One per connected client; the sole sender on its socket. Waits
        # for the wake event, snapshots the pending dict, clears it, sends
        # every captured packet. A send that stalls past the timeout (or
        # errors) closes the client; the receive coroutine wakes on the
        # disconnect and runs the normal cleanup, and the browser client's
        # auto-reconnect takes it from there.
        pending = self._pending.get(websocket)
        wake = self._wake.get(websocket)
        if pending is None or wake is None:
            return
        try:
            while True:
                await wake.wait()
                wake.clear()
                if not pending:
                    continue
                snapshot = list(pending.values())
                pending.clear()
                for packet in snapshot:
                    await asyncio.wait_for(websocket.send_bytes(packet), timeout=_WS_SEND_TIMEOUT_S)
        except asyncio.CancelledError:
            pass
        except Exception:
            logger.warning("lcm-ws bridge: drain failed, closing client", exc_info=True)
            with suppress(Exception):
                await asyncio.wait_for(websocket.close(), timeout=_WS_CLOSE_TIMEOUT_S)

    # ---- browser -> bus ----------------------------------------------------

    def publish_packet(self, packet: bytes) -> None:
        """Republish a browser-sent LCM small-message packet onto the bus."""
        if self._lcm is None:
            return
        decoded = decode_packet(packet)
        if decoded is None:
            logger.warning("lcm-ws bridge: dropping malformed packet from client")
            return
        channel, payload = decoded
        self._lcm.publish(channel, payload)
        self.published_from_clients += 1

    # ---- endpoint ----------------------------------------------------------

    async def websocket_endpoint(self, websocket: WebSocket) -> None:
        await websocket.accept()
        # The serving loop is whichever loop accepts clients; capture it
        # here so the bridge needs no lifespan hook from the host app.
        self._loop = asyncio.get_running_loop()
        self._pending[websocket] = {}
        self._wake[websocket] = asyncio.Event()
        self._last_forward_ts[websocket] = {}
        self._clients.add(websocket)
        drain_task = asyncio.create_task(self._drain_loop(websocket))
        self._drain_tasks[websocket] = drain_task
        logger.info("lcm-ws bridge: client connected", clients=len(self._clients))
        try:
            while True:
                packet = await websocket.receive_bytes()
                self.publish_packet(packet)
        except WebSocketDisconnect:
            pass
        except Exception:
            logger.exception("lcm-ws bridge: receive failed")
        finally:
            self._clients.discard(websocket)
            self._pending.pop(websocket, None)
            self._wake.pop(websocket, None)
            self._last_forward_ts.pop(websocket, None)
            task = self._drain_tasks.pop(websocket, None)
            if task is not None:
                task.cancel()
                with suppress(asyncio.CancelledError, Exception):
                    await task
            logger.info("lcm-ws bridge: client disconnected", clients=len(self._clients))
