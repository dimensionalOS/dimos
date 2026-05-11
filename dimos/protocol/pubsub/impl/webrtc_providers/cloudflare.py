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

# Copyright 2025-2026 Dimensional Inc.
# Licensed under the Apache License, Version 2.0

"""Cloudflare Realtime SFU DataChannel provider.

Manages two CF sessions (pub + sub) so a single process can do loopback
pubsub through the CF SFU. Per-topic DataChannel pairs are created lazily.

CF DataChannels are unidirectional, hence two sessions. The SFU routes
messages from publisher session to subscriber session via /datachannels/new
with location "local" (pub) and "remote" (sub).

Env vars:
    CF_TELEOP_APP_ID     — Cloudflare Realtime app id
    CF_TELEOP_APP_SECRET — Cloudflare Realtime app secret
"""

from __future__ import annotations

import asyncio
from collections import defaultdict
from collections.abc import Callable
import concurrent.futures
import os
import re
import threading
from typing import Any

from dimos.protocol.pubsub.impl.webrtcpubsub import DataChannelProvider
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

try:
    from aiortc import (
        RTCConfiguration,
        RTCDataChannel,
        RTCIceServer,
        RTCPeerConnection,
        RTCSessionDescription,
    )
    import httpx

    CLOUDFLARE_AVAILABLE = True
except ImportError:
    CLOUDFLARE_AVAILABLE = False
    RTCConfiguration = None  # type: ignore[assignment,misc]
    RTCDataChannel = Any  # type: ignore[misc,assignment]
    RTCIceServer = None  # type: ignore[assignment,misc]
    RTCPeerConnection = None  # type: ignore[assignment,misc]
    RTCSessionDescription = None  # type: ignore[assignment,misc]
    httpx = None  # type: ignore[assignment]

_PLACEHOLDER_DC_ID = 100
_MAX_MSG_SIZE = 1 * 1024 * 1024


def _sanitize_topic(topic: str) -> str:
    """Sanitize topic name for CF DataChannel naming (ASCII, <=64 chars)."""
    return re.sub(r"[^a-zA-Z0-9_-]", "_", topic)[:64] or "dc"


class CloudflareProvider(DataChannelProvider):
    """Cloudflare Realtime SFU DataChannel provider.

    Creates two CF sessions: one for publishing, one for subscribing.
    Runs aiortc on a dedicated background asyncio thread.
    """

    def __init__(
        self,
        app_id: str | None = None,
        app_secret: str | None = None,
        *,
        publisher_session_id: str | None = None,
        stun_url: str = "stun:stun.cloudflare.com:3478",
        ordered: bool = False,
        max_retransmits: int | None = 0,
    ) -> None:
        if not CLOUDFLARE_AVAILABLE:
            raise RuntimeError("aiortc and httpx required: pip install aiortc httpx")

        self._app_id = app_id or os.environ.get("CF_TELEOP_APP_ID", "")
        self._app_secret = app_secret or os.environ.get("CF_TELEOP_APP_SECRET", "")
        if not self._app_id or not self._app_secret:
            raise RuntimeError("CF_TELEOP_APP_ID and CF_TELEOP_APP_SECRET required")

        self._base_url = f"https://rtc.live.cloudflare.com/v1/apps/{self._app_id}"
        self._stun_url = stun_url
        self._external_pub_id = publisher_session_id
        self._ordered = ordered
        self._max_retransmits = max_retransmits

        # State
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._ready = threading.Event()
        self._stop_ev: asyncio.Event | None = None
        self._started = False
        self._lock = threading.RLock()

        # CF objects
        self.pub_session_id: str | None = None
        self.sub_session_id: str | None = None
        self._pub_pc: RTCPeerConnection | None = None
        self._sub_pc: RTCPeerConnection | None = None
        self._http: httpx.AsyncClient | None = None

        # Channels & callbacks
        self._pub_channels: dict[str, RTCDataChannel] = {}
        self._sub_channels: dict[str, RTCDataChannel] = {}
        self._callbacks: dict[str, list[Callable[[bytes, str], None]]] = defaultdict(list)

    @property
    def is_connected(self) -> bool:
        return self._started

    @property
    def _headers(self) -> dict[str, str]:
        return {"Authorization": f"Bearer {self._app_secret}", "Content-Type": "application/json"}

    # ─── Lifecycle ───────────────────────────────────────────────────

    def start(self) -> None:
        with self._lock:
            if self._started:
                return
            self._thread = threading.Thread(target=self._run_loop, daemon=True, name="cf-webrtc")
            self._thread.start()
            if not self._ready.wait(timeout=5.0):
                raise RuntimeError("CF event loop failed to start")
            self._run_sync(self._connect())
            self._started = True

    def stop(self) -> None:
        with self._lock:
            if not self._started:
                return
            if self._loop and self._loop.is_running():
                try:
                    self._run_sync(self._disconnect())
                except Exception:
                    logger.exception("Error during CF disconnect")
            if self._thread:
                self._thread.join(timeout=5.0)
            self._thread = None
            self._loop = None
            self._ready.clear()
            self._started = False
            self._pub_channels.clear()
            self._sub_channels.clear()

    def _run_loop(self) -> None:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        executor = concurrent.futures.ThreadPoolExecutor(max_workers=4, thread_name_prefix="cf-aio")
        loop.set_default_executor(executor)
        self._loop = loop
        self._stop_ev = asyncio.Event()
        self._ready.set()
        try:
            loop.run_until_complete(self._stop_ev.wait())
        finally:
            for task in asyncio.all_tasks(loop):
                task.cancel()
            loop.run_until_complete(
                asyncio.gather(*asyncio.all_tasks(loop), return_exceptions=True)
            )
            loop.run_until_complete(loop.shutdown_default_executor())
            executor.shutdown(wait=True)
            loop.close()

    def _run_sync(self, coro: Any, timeout: float = 30.0) -> Any:
        assert self._loop is not None
        return asyncio.run_coroutine_threadsafe(coro, self._loop).result(timeout=timeout)

    # ─── Connect / Disconnect ────────────────────────────────────────

    async def _connect(self) -> None:
        self._http = httpx.AsyncClient(timeout=30.0)
        ice = RTCConfiguration(iceServers=[RTCIceServer(urls=[self._stun_url])])

        self.pub_session_id = await self._create_session()
        self.sub_session_id = await self._create_session()

        self._pub_pc = RTCPeerConnection(configuration=ice)
        self._sub_pc = RTCPeerConnection(configuration=ice)

        await self._establish_transport(self._pub_pc, self.pub_session_id)
        await self._establish_transport(self._sub_pc, self.sub_session_id)

        await self._wait_connected(self._pub_pc)
        await self._wait_connected(self._sub_pc)
        logger.info(
            "CF provider connected: pub=%s sub=%s", self.pub_session_id[:8], self.sub_session_id[:8]
        )

    async def _disconnect(self) -> None:
        if self._pub_pc:
            await self._pub_pc.close()
        if self._sub_pc:
            await self._sub_pc.close()
        if self._http:
            await self._http.aclose()
        if self._stop_ev:
            self._stop_ev.set()

    # ─── CF REST API ─────────────────────────────────────────────────

    async def _create_session(self) -> str:
        assert self._http
        r = await self._http.post(f"{self._base_url}/sessions/new", headers=self._headers)
        assert r.status_code in (200, 201), f"CF /sessions/new: {r.status_code} {r.text}"
        return str(r.json()["sessionId"])

    async def _establish_transport(self, pc: RTCPeerConnection, session_id: str) -> None:
        assert self._http
        # Placeholder DC forces SCTP in SDP without conflicting with CF-assigned IDs
        pc.createDataChannel("_placeholder", negotiated=True, id=_PLACEHOLDER_DC_ID)
        offer = await pc.createOffer()
        await pc.setLocalDescription(offer)

        if pc.iceGatheringState != "complete":
            ev = asyncio.Event()

            @pc.on("icegatheringstatechange")
            def _():
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
        assert r.status_code in (200, 201), f"CF /establish: {r.status_code} {r.text}"
        data = r.json()

        if data.get("requiresImmediateRenegotiation"):
            await pc.setRemoteDescription(
                RTCSessionDescription(
                    sdp=data["sessionDescription"]["sdp"], type=data["sessionDescription"]["type"]
                )
            )
            answer = await pc.createAnswer()
            await pc.setLocalDescription(answer)
            r2 = await self._http.put(
                f"{self._base_url}/sessions/{session_id}/renegotiate",
                headers=self._headers,
                json={"sessionDescription": {"sdp": answer.sdp, "type": "answer"}},
            )
            assert r2.status_code == 200, f"CF /renegotiate: {r2.status_code} {r2.text}"
        else:
            await pc.setRemoteDescription(
                RTCSessionDescription(
                    sdp=data["sessionDescription"]["sdp"], type=data["sessionDescription"]["type"]
                )
            )

    async def _publish_dc(self, dc_name: str) -> int:
        assert self._http and self.pub_session_id
        r = await self._http.post(
            f"{self._base_url}/sessions/{self.pub_session_id}/datachannels/new",
            headers=self._headers,
            json={"dataChannels": [{"location": "local", "dataChannelName": dc_name}]},
        )
        assert r.status_code in (200, 201), f"CF pub DC: {r.status_code} {r.text}"
        return int(r.json()["dataChannels"][0]["id"])

    async def _subscribe_dc(self, dc_name: str, pub_session_id: str) -> int:
        assert self._http and self.sub_session_id
        r = await self._http.post(
            f"{self._base_url}/sessions/{self.sub_session_id}/datachannels/new",
            headers=self._headers,
            json={
                "dataChannels": [
                    {"location": "remote", "sessionId": pub_session_id, "dataChannelName": dc_name}
                ]
            },
        )
        assert r.status_code in (200, 201), f"CF sub DC: {r.status_code} {r.text}"
        return int(r.json()["dataChannels"][0]["id"])

    @staticmethod
    async def _wait_connected(pc: RTCPeerConnection, timeout: float = 15.0) -> None:
        if pc.connectionState == "connected":
            return
        ev = asyncio.Event()

        @pc.on("connectionstatechange")
        def _():
            if pc.connectionState in ("connected", "failed", "closed"):
                ev.set()

        await asyncio.wait_for(ev.wait(), timeout)
        assert pc.connectionState == "connected", f"PC state: {pc.connectionState}"

    @staticmethod
    async def _wait_open(ch: RTCDataChannel, timeout: float = 15.0) -> None:
        if ch.readyState == "open":
            return
        ev = asyncio.Event()

        @ch.on("open")
        def _():
            ev.set()

        await asyncio.wait_for(ev.wait(), timeout)

    # ─── Channel management ──────────────────────────────────────────

    async def _ensure_pub(self, topic: str) -> RTCDataChannel:
        if topic in self._pub_channels:
            return self._pub_channels[topic]
        dc_name = _sanitize_topic(f"pub_{topic}")
        dc_id = await self._publish_dc(dc_name)
        assert self._pub_pc
        ch = self._pub_pc.createDataChannel(
            dc_name,
            negotiated=True,
            id=dc_id,
            ordered=self._ordered,
            maxRetransmits=self._max_retransmits,
        )
        await self._wait_open(ch)
        self._pub_channels[topic] = ch
        return ch

    async def _ensure_sub(self, topic: str) -> RTCDataChannel:
        if topic in self._sub_channels:
            return self._sub_channels[topic]
        pub_sid = self._external_pub_id
        if pub_sid is None:
            await self._ensure_pub(topic)
            pub_sid = self.pub_session_id
        assert pub_sid
        dc_name = _sanitize_topic(f"pub_{topic}")
        dc_id = await self._subscribe_dc(dc_name, pub_sid)
        assert self._sub_pc
        ch = self._sub_pc.createDataChannel(
            f"sub_{dc_name}",
            negotiated=True,
            id=dc_id,
            ordered=self._ordered,
            maxRetransmits=self._max_retransmits,
        )
        cbs = self._callbacks

        @ch.on("message")
        def _on_msg(payload: Any) -> None:
            if isinstance(payload, str):
                payload = payload.encode()
            for cb in list(cbs.get(topic, ())):
                try:
                    cb(payload, topic)
                except Exception:
                    logger.exception("WebRTC subscriber callback error")

        await self._wait_open(ch)
        self._sub_channels[topic] = ch
        return ch

    # ─── Public API (DataChannelProvider) ────────────────────────────

    def publish(self, topic: str, data: bytes) -> None:
        if not self._started:
            self.start()
        if isinstance(data, (bytearray, memoryview)):
            data = bytes(data)
        if len(data) > _MAX_MSG_SIZE:
            logger.warning("WebRTC msg on %r exceeds %d bytes", topic, _MAX_MSG_SIZE)
        ch = self._pub_channels.get(topic)
        if ch is None:
            ch = self._run_sync(self._ensure_pub(topic))
        assert self._loop
        self._loop.call_soon_threadsafe(ch.send, data)

    def subscribe(self, topic: str, callback: Callable[[bytes, str], None]) -> Callable[[], None]:
        if not self._started:
            self.start()
        with self._lock:
            self._callbacks[topic].append(callback)
        if topic not in self._sub_channels:
            self._run_sync(self._ensure_sub(topic))

        def _unsub() -> None:
            with self._lock:
                try:
                    self._callbacks[topic].remove(callback)
                except ValueError:
                    pass

        return _unsub


__all__ = ["CLOUDFLARE_AVAILABLE", "CloudflareProvider"]
