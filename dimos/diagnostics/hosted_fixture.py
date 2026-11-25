# Copyright 2025-2026 Dimensional Inc.
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

"""Explicit integration fixture for an owned broker session; never attaches to a robot.

The robot side is the shipped BrokerProvider in a separate process. This
operator implements the broker's public join/bridge/leave protocol. Credentials
must belong to a dedicated test robot; no session discovery or takeover occurs.
"""

from __future__ import annotations

import asyncio
from collections import defaultdict
from collections.abc import Callable
import contextlib
from typing import Any

from dimos.protocol.pubsub.impl.webrtc.providers.sdp import propagate_bundle_candidates
from dimos.protocol.pubsub.impl.webrtc.providers.spec import (
    WEBRTC_AVAILABLE,
    AsyncProviderBase,
    ProviderConfig,
    wait_connected,
    wait_open,
)

if WEBRTC_AVAILABLE:
    import aiohttp
    from aiortc import (
        RTCBundlePolicy,
        RTCConfiguration,
        RTCIceServer,
        RTCPeerConnection,
        RTCSessionDescription,
    )


class OperatorFixtureConfig(ProviderConfig):
    broker_url: str
    token: str
    session_id: str

    def _create(self) -> OperatorFixture:
        return OperatorFixture(self)


class OperatorFixture(AsyncProviderBase):
    def __init__(self, config: OperatorFixtureConfig) -> None:
        super().__init__()
        self.config = config
        self.http: Any = None
        self.pc: Any = None
        self.channels: dict[str, Any] = {}
        self.callbacks: dict[str, list[Callable[[bytes, str], None]]] = defaultdict(list)
        self.heartbeat: asyncio.Task[None] | None = None
        self.joined = False

    async def request(self, method: str, suffix: str, body: Any = None) -> Any:
        url = self.config.broker_url.rstrip("/") + "/api/v1/sessions" + suffix
        async with self.http.request(method, url, json=body) as response:
            if response.status not in (200, 201, 204):
                # Do not put SDP, auth tokens or service responses into evidence.
                raise RuntimeError(f"hosted fixture {method} failed: HTTP {response.status}")
            return await response.json() if response.status != 204 else None

    async def _connect(self) -> None:
        self.http = aiohttp.ClientSession(
            headers={"Authorization": "Bearer " + self.config.token},
            timeout=aiohttp.ClientTimeout(total=30),
        )
        ice = await self.request("GET", "/turn-credentials")
        self.pc = RTCPeerConnection(
            RTCConfiguration(
                bundlePolicy=RTCBundlePolicy.MAX_BUNDLE,
                iceServers=[RTCIceServer(**s) for s in ice["ice_servers"]],
            )
        )
        self.pc.addTransceiver("video", direction="recvonly")
        self.pc.createDataChannel("_sctp_init", negotiated=True, id=0)
        await self.pc.setLocalDescription(await self.pc.createOffer())
        base = "/" + self.config.session_id
        data = await self.request(
            "POST",
            base + "/join",
            {
                "role": "operator",
                "sdp_offer": self.pc.localDescription.sdp,
            },
        )
        self.joined = True
        await self.pc.setRemoteDescription(
            RTCSessionDescription(
                sdp=propagate_bundle_candidates(data["sdp_answer"]),
                type="answer",
            )
        )
        await wait_connected(self.pc, 30)
        bridge = await self.request("POST", base + "/bridge-datachannel")
        for name, field in [
            ("state_reliable", "state_channel_id"),
            ("state_reliable_back", "state_back_channel_id"),
        ]:
            channel = self.pc.createDataChannel(
                name, negotiated=True, id=bridge[field], ordered=True
            )
            self.channels[name] = channel

            def received(data: Any, topic: str = name) -> None:
                raw = data.encode() if isinstance(data, str) else bytes(data)
                with self._lock:
                    callbacks = list(self.callbacks[topic])
                for callback in callbacks:
                    callback(raw, topic)

            channel.on("message", received)
        if bridge.get("video_offer"):
            await self.pc.setRemoteDescription(
                RTCSessionDescription(
                    sdp=propagate_bundle_candidates(bridge["video_offer"]),
                    type="offer",
                )
            )
            await self.pc.setLocalDescription(await self.pc.createAnswer())
            await self.request(
                "POST", base + "/renegotiate-answer", {"sdp_answer": self.pc.localDescription.sdp}
            )
        for channel in self.channels.values():
            await wait_open(channel, 20)
        self.heartbeat = asyncio.create_task(self._heartbeat())

    async def _heartbeat(self) -> None:
        while True:
            await self.request("POST", "/" + self.config.session_id + "/op-heartbeat")
            await asyncio.sleep(1)

    async def _disconnect(self) -> None:
        if self.heartbeat:
            self.heartbeat.cancel()
            with contextlib.suppress(asyncio.CancelledError, Exception):
                await self.heartbeat
        try:
            if self.joined and self.http:
                await self.request("POST", "/" + self.config.session_id + "/leave")
        finally:
            self.joined = False
            if self.pc:
                await self.pc.close()
            if self.http:
                await self.http.close()
            self.channels.clear()

    def publish(self, topic: str, data: bytes) -> None:
        if topic != "state_reliable":
            raise ValueError("operator fixture may publish only its outbound reliable channel")
        if len(data) > 32768:
            raise ValueError("hosted message exceeds provider limit")
        assert self._loop is not None

        def send() -> None:
            channel = self.channels[topic]
            if channel.readyState == "open":
                channel.send(data)

        self._loop.call_soon_threadsafe(send)

    def subscribe(self, topic: str, callback: Callable[[bytes, str], None]) -> Callable[[], None]:
        if topic != "state_reliable_back":
            raise ValueError("operator fixture may receive only its inbound reliable channel")
        with self._lock:
            self.callbacks[topic].append(callback)

        def dispose() -> None:
            with self._lock:
                if callback in self.callbacks[topic]:
                    self.callbacks[topic].remove(callback)

        return dispose
