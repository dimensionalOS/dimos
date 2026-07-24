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

"""In-process WebRTC loopback: two peer connections over localhost UDP.

Unlike the in-memory mock providers (dict dispatch, no network), this runs
the full aiortc stack — DTLS crypto, SCTP framing, localhost UDP — so it
measures and tests what a real DataChannel link does, keyless and offline.
Host candidates only: no STUN, no TURN, no signaling server (the SDP
offer/answer is exchanged directly between the two in-process peers).
"""

from __future__ import annotations

import asyncio
from collections import defaultdict
from collections.abc import Callable
from typing import TYPE_CHECKING, Any

from dimos.protocol.pubsub.impl.webrtc.providers.spec import (
    WEBRTC_AVAILABLE,
    AsyncProviderBase,
    Provider,
    ProviderConfig,
    wait_connected,
    wait_open,
)
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from aiortc import RTCDataChannel, RTCPeerConnection

logger = setup_logger()

# aiortc's SDP-negotiated SCTP max-message-size. Larger sends raise in the
# SCTP stack; chunking above this limit is a framing-layer concern.
MAX_MSG_SIZE = 64 * 1024


class LoopbackConfig(ProviderConfig):
    """Loopback link settings. Defaults are ordered+reliable so the contract
    grid and benchmark measure the link, not message loss."""

    ordered: bool = True
    max_retransmits: int | None = None

    def _create(self) -> Provider:
        return LoopbackProvider(self)


class LoopbackProvider(AsyncProviderBase):
    """One sender and one receiver RTCPeerConnection in the same process.

    Topics map to lazily-created negotiated DataChannel pairs (same id on
    both peers, allocated locally — both ends are ours, so no signaling is
    needed after the initial offer/answer). Traffic direction is send-side →
    recv-side, mirroring the Cloudflare loopback pair.
    """

    def __init__(self, config: LoopbackConfig | None = None) -> None:
        if not WEBRTC_AVAILABLE:
            raise RuntimeError("aiortc required: pip install dimos[webrtc]")
        super().__init__()
        self._config = config or LoopbackConfig()
        self._send_pc: RTCPeerConnection | None = None
        self._recv_pc: RTCPeerConnection | None = None
        self._channel_lock: asyncio.Lock | None = None
        self._next_dc_id = 1  # id 0 is the pre-offer placeholder

        # Guarded by self._lock (from the base); never held across an await.
        self._channels: dict[str, RTCDataChannel] = {}
        self._callbacks: dict[str, list[Callable[[bytes, str], None]]] = defaultdict(list)

    # ─── Connect / Disconnect (loop thread) ──────────────────────────

    async def _connect(self) -> None:
        from aiortc import RTCConfiguration, RTCPeerConnection

        self._channel_lock = asyncio.Lock()
        self._next_dc_id = 1
        # Empty iceServers: host candidates only. aiortc's default config
        # queries Google STUN, stalling gathering ~5s per peer when blocked.
        ice = RTCConfiguration(iceServers=[])
        self._send_pc = RTCPeerConnection(configuration=ice)
        self._recv_pc = RTCPeerConnection(configuration=ice)

        # A pre-offer channel puts the SCTP m-line in the SDP; per-topic
        # channels are then negotiated additions needing no renegotiation.
        self._send_pc.createDataChannel("_init", negotiated=True, id=0)
        self._recv_pc.createDataChannel("_init", negotiated=True, id=0)

        offer = await self._send_pc.createOffer()
        await self._send_pc.setLocalDescription(offer)  # gathers host candidates
        await self._recv_pc.setRemoteDescription(self._send_pc.localDescription)
        answer = await self._recv_pc.createAnswer()
        await self._recv_pc.setLocalDescription(answer)
        await self._send_pc.setRemoteDescription(self._recv_pc.localDescription)

        await wait_connected(self._send_pc)
        await wait_connected(self._recv_pc)

    async def _disconnect(self) -> None:
        if self._send_pc:
            await self._send_pc.close()
            self._send_pc = None
        if self._recv_pc:
            await self._recv_pc.close()
            self._recv_pc = None
        with self._lock:
            self._channels.clear()

    # ─── Channel management (loop thread) ────────────────────────────

    async def _ensure_channel(self, topic: str) -> RTCDataChannel:
        assert self._channel_lock and self._send_pc and self._recv_pc
        async with self._channel_lock:
            with self._lock:
                ch = self._channels.get(topic)
            if ch is not None:
                return ch
            dc_id = self._next_dc_id
            self._next_dc_id += 1
            options: dict[str, Any] = {
                "negotiated": True,
                "id": dc_id,
                "ordered": self._config.ordered,
                "maxRetransmits": self._config.max_retransmits,
            }
            send_ch = self._send_pc.createDataChannel(topic, **options)
            recv_ch = self._recv_pc.createDataChannel(topic, **options)

            @recv_ch.on("message")
            def _on_msg(payload: Any) -> None:
                if isinstance(payload, str):
                    payload = payload.encode()
                with self._lock:
                    callbacks = list(self._callbacks.get(topic, ()))
                for cb in callbacks:
                    try:
                        cb(payload, topic)
                    except Exception:
                        logger.exception("WebRTC subscriber callback error")

            await wait_open(send_ch)
            await wait_open(recv_ch)
            with self._lock:
                self._channels[topic] = send_ch
            return send_ch

    # ─── Public API (Provider) ───────────────────────────────────────

    def publish(self, topic: str, data: bytes) -> None:
        if not self.is_connected:
            self.start()
        if isinstance(data, (bytearray, memoryview)):
            data = bytes(data)
        if len(data) > MAX_MSG_SIZE:
            logger.warning("WebRTC msg on %r exceeds %d bytes", topic, MAX_MSG_SIZE)
        with self._lock:
            ch = self._channels.get(topic)
        if ch is None:
            ch = self._run_sync(self._ensure_channel(topic))
        with self._lock:
            if not self._started or self._loop is None:
                return
            self._loop.call_soon_threadsafe(ch.send, data)

    def subscribe(self, topic: str, callback: Callable[[bytes, str], None]) -> Callable[[], None]:
        if not self.is_connected:
            self.start()
        with self._lock:
            self._callbacks[topic].append(callback)
        try:
            self._run_sync(self._ensure_channel(topic))
        except BaseException:
            # Failed subscribe returns no unsub handle — deregister, or the
            # callback would start firing once a later subscribe succeeds.
            with self._lock:
                if callback in self._callbacks[topic]:
                    self._callbacks[topic].remove(callback)
            raise

        def _unsub() -> None:
            with self._lock:
                try:
                    self._callbacks[topic].remove(callback)
                except ValueError:
                    pass

        return _unsub
