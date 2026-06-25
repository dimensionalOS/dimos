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

"""Direct Cloudflare Realtime SFU provider — TEST AND BENCHMARK ONLY.

Never deploy this on a robot: it authenticates with the CF *app secret*,
which must not leave server-side/dev environments (robots carry revocable
``dtk_live_*`` keys and go through the dimensional-teleop broker — see
``broker.py``). CF DataChannels are unidirectional, so this provider holds
two CF sessions — one publishing, one subscribing — giving a single process
loopback pubsub through the CF edge: exactly what the integration tests and
the pubsub benchmark need, and nothing else.

``app_id`` / ``app_secret`` reach ``CloudflareConfig`` via the standard
blueprint config flow (``-o transports.cloudflare.app_id=...`` or
``TRANSPORTS__CLOUDFLARE__APP_ID=...``).
"""

from __future__ import annotations

import asyncio
from collections import defaultdict
from collections.abc import Callable
import hashlib
import re
from typing import Any

from dimos.protocol.pubsub.impl.webrtc.providers.spec import (
    WEBRTC_AVAILABLE,
    AsyncProviderBase,
    ProviderConfig,
    wait_connected,
    wait_open,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

if WEBRTC_AVAILABLE:
    from aiortc import (
        RTCConfiguration,
        RTCDataChannel,
        RTCIceServer,
        RTCPeerConnection,
        RTCSessionDescription,
    )
    import httpx
else:
    RTCDataChannel = Any  # type: ignore[misc,assignment]

# Forces an SCTP m-line into the offer; CF-assigned channel ids start low, so
# a high fixed id stays clear of them.
_PLACEHOLDER_DC_ID = 100
# CF Realtime drops DataChannel messages larger than this (observed: 100% loss
# from 64KB up in the pubsub benchmark); also matches the SCTP fragmentation unit.
MAX_MSG_SIZE = 16 * 1024


def _dc_name(topic: str) -> str:
    """CF DataChannel name for a topic (ASCII, <=64 chars, collision-safe)."""
    safe = re.sub(r"[^a-zA-Z0-9_-]", "_", topic)
    if safe != topic:
        safe = f"{safe[:55]}_{hashlib.sha1(topic.encode()).hexdigest()[:8]}"
    return safe[:64] or "dc"


class CloudflareConfig(ProviderConfig):
    """Direct CF Realtime access. ``app_id`` and ``app_secret`` are required.

    The provider is a loopback pair (its own pub + sub session), so delivery
    spans only transports sharing the provider — i.e. one process. Bridging
    distinct peers (robot ↔ operator) is the broker provider's job.
    """

    app_id: str | None = None
    app_secret: str | None = None
    stun_url: str = "stun:stun.cloudflare.com:3478"
    ordered: bool = False
    max_retransmits: int | None = 0

    def _create(self) -> CloudflareProvider:
        return CloudflareProvider(self)


class CloudflareProvider(AsyncProviderBase):
    """Two-session (pub + sub) Cloudflare Realtime SFU provider."""

    def __init__(self, config: CloudflareConfig | None = None) -> None:
        if not WEBRTC_AVAILABLE:
            raise RuntimeError("aiortc and httpx required: pip install dimos[webrtc]")
        super().__init__()
        config = config or CloudflareConfig()
        if not config.app_id or not config.app_secret:
            raise RuntimeError(
                "CloudflareConfig.app_id and app_secret required "
                "(set -o transports.cloudflare.app_id=... and app_secret=..., or the "
                "TRANSPORTS__CLOUDFLARE__APP_ID / TRANSPORTS__CLOUDFLARE__APP_SECRET env form)"
            )
        self._app_secret = config.app_secret
        self._config = config
        self._base_url = f"https://rtc.live.cloudflare.com/v1/apps/{config.app_id}"

        self._http: httpx.AsyncClient | None = None
        self._pub_pc: RTCPeerConnection | None = None
        self._sub_pc: RTCPeerConnection | None = None
        self.pub_session_id: str | None = None
        self.sub_session_id: str | None = None
        self._channel_lock: asyncio.Lock | None = None

        # Guarded by self._lock (from the base); never held across an await.
        self._pub_channels: dict[str, RTCDataChannel] = {}
        self._sub_channels: dict[str, RTCDataChannel] = {}
        self._callbacks: dict[str, list[Callable[[bytes, str], None]]] = defaultdict(list)

    @property
    def _headers(self) -> dict[str, str]:
        return {"Authorization": f"Bearer {self._app_secret}", "Content-Type": "application/json"}

    # ─── Connect / Disconnect (loop thread) ──────────────────────────

    async def _connect(self) -> None:
        self._http = httpx.AsyncClient(timeout=30.0)
        self._channel_lock = asyncio.Lock()
        ice = RTCConfiguration(iceServers=[RTCIceServer(urls=[self._config.stun_url])])

        self.pub_session_id = await self._create_session()
        self.sub_session_id = await self._create_session()
        self._pub_pc = RTCPeerConnection(configuration=ice)
        self._sub_pc = RTCPeerConnection(configuration=ice)
        await self._establish_transport(self._pub_pc, self.pub_session_id)
        await self._establish_transport(self._sub_pc, self.sub_session_id)
        await wait_connected(self._pub_pc)
        await wait_connected(self._sub_pc)
        logger.info(
            "CF provider connected: pub=%s sub=%s", self.pub_session_id[:8], self.sub_session_id[:8]
        )

    async def _disconnect(self) -> None:
        if self._pub_pc:
            await self._pub_pc.close()
            self._pub_pc = None
        if self._sub_pc:
            await self._sub_pc.close()
            self._sub_pc = None
        if self._http:
            await self._http.aclose()
            self._http = None
        with self._lock:
            self._pub_channels.clear()
            self._sub_channels.clear()

    # ─── CF REST API (loop thread) ───────────────────────────────────

    async def _create_session(self) -> str:
        assert self._http
        r = await self._http.post(f"{self._base_url}/sessions/new", headers=self._headers)
        if r.status_code not in (200, 201):
            raise RuntimeError(f"CF /sessions/new: {r.status_code} {r.text}")
        return str(r.json()["sessionId"])

    async def _establish_transport(self, pc: RTCPeerConnection, session_id: str) -> None:
        assert self._http
        pc.createDataChannel("_placeholder", negotiated=True, id=_PLACEHOLDER_DC_ID)
        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)

        if pc.iceGatheringState != "complete":
            ev = asyncio.Event()

            @pc.on("icegatheringstatechange")
            def _on_gathering() -> None:
                if pc.iceGatheringState == "complete":
                    ev.set()

            await asyncio.wait_for(ev.wait(), 10.0)

        r = await self._http.post(
            f"{self._base_url}/sessions/{session_id}/datachannels/establish",
            headers=self._headers,
            json={
                "dataChannel": {"location": "remote", "dataChannelName": "server-events"},
                "sessionDescription": {"type": "offer", "sdp": pc.localDescription.sdp},
            },
        )
        if r.status_code not in (200, 201):
            raise RuntimeError(f"CF /establish: {r.status_code} {r.text}")
        data = r.json()

        await pc.setRemoteDescription(
            RTCSessionDescription(
                sdp=data["sessionDescription"]["sdp"], type=data["sessionDescription"]["type"]
            )
        )
        if data.get("requiresImmediateRenegotiation"):
            answer = await pc.createAnswer()
            await pc.setLocalDescription(answer)
            r2 = await self._http.put(
                f"{self._base_url}/sessions/{session_id}/renegotiate",
                headers=self._headers,
                json={"sessionDescription": {"sdp": answer.sdp, "type": "answer"}},
            )
            if r2.status_code != 200:
                raise RuntimeError(f"CF /renegotiate: {r2.status_code} {r2.text}")

    async def _new_datachannel(self, body: dict[str, Any], session_id: str) -> int:
        assert self._http
        r = await self._http.post(
            f"{self._base_url}/sessions/{session_id}/datachannels/new",
            headers=self._headers,
            json={"dataChannels": [body]},
        )
        if r.status_code not in (200, 201):
            raise RuntimeError(f"CF /datachannels/new: {r.status_code} {r.text}")
        return int(r.json()["dataChannels"][0]["id"])

    # ─── Channel management (loop thread) ────────────────────────────

    async def _ensure_pub(self, topic: str) -> RTCDataChannel:
        assert self._channel_lock and self.pub_session_id and self._pub_pc
        async with self._channel_lock:
            with self._lock:
                ch = self._pub_channels.get(topic)
            if ch is not None:
                return ch
            name = _dc_name(f"pub_{topic}")
            dc_id = await self._new_datachannel(
                {"location": "local", "dataChannelName": name}, self.pub_session_id
            )
            ch = self._pub_pc.createDataChannel(
                name,
                negotiated=True,
                id=dc_id,
                ordered=self._config.ordered,
                maxRetransmits=self._config.max_retransmits,
            )
            await wait_open(ch)
            with self._lock:
                self._pub_channels[topic] = ch
            return ch

    async def _ensure_sub(self, topic: str) -> None:
        assert self._channel_lock and self.sub_session_id and self._sub_pc
        await self._ensure_pub(topic)
        async with self._channel_lock:
            with self._lock:
                if topic in self._sub_channels:
                    return
            name = _dc_name(f"pub_{topic}")
            dc_id = await self._new_datachannel(
                {"location": "remote", "sessionId": self.pub_session_id, "dataChannelName": name},
                self.sub_session_id,
            )
            ch = self._sub_pc.createDataChannel(
                f"sub_{name}"[:64],
                negotiated=True,
                id=dc_id,
                ordered=self._config.ordered,
                maxRetransmits=self._config.max_retransmits,
            )

            @ch.on("message")
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

            await wait_open(ch)
            with self._lock:
                self._sub_channels[topic] = ch

    # ─── Public API (Provider) ───────────────────────────────────────

    def publish(self, topic: str, data: bytes) -> None:
        if not self.is_connected:
            self.start()
        if isinstance(data, (bytearray, memoryview)):
            data = bytes(data)
        if len(data) > MAX_MSG_SIZE:
            logger.warning("WebRTC msg on %r exceeds %d bytes", topic, MAX_MSG_SIZE)
        with self._lock:
            ch = self._pub_channels.get(topic)
        if ch is None:
            ch = self._run_sync(self._ensure_pub(topic))
        with self._lock:
            if not self._started or self._loop is None:
                return
            self._loop.call_soon_threadsafe(ch.send, data)

    def subscribe(self, topic: str, callback: Callable[[bytes, str], None]) -> Callable[[], None]:
        if not self.is_connected:
            self.start()
        with self._lock:
            self._callbacks[topic].append(callback)
        self._run_sync(self._ensure_sub(topic))

        def _unsub() -> None:
            with self._lock:
                try:
                    self._callbacks[topic].remove(callback)
                except ValueError:
                    pass

        return _unsub


__all__ = ["MAX_MSG_SIZE", "CloudflareConfig", "CloudflareProvider"]
