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

"""Broker-mediated Cloudflare Realtime SFU DataChannel provider.

Unlike :class:`CloudflareProvider` (which creates CF sessions directly with
app credentials), this provider works through a hosted teleop **broker**
(``dimensional-teleop``) that manages session creation, operator join/leave,
and DataChannel bridging on behalf of the robot.

Flow:
  1. Robot sends SDP offer to broker ``POST /api/v1/sessions``
  2. Broker creates CF session, returns SDP answer + session_id
  3. Robot starts heartbeat loop to broker
  4. When operator joins, broker bridges a DataChannel; heartbeat ack
     returns the ``cmd_channel_subscriber_id``
  5. Robot creates a negotiated DataChannel with that ID to receive commands
  6. Messages (LCM-encoded) arrive on the single multiplexed channel

Env vars (fallback when constructor args omitted):
    TELEOP_BROKER_URL   — e.g. https://teleop.dimensionalos.com
    TELEOP_API_KEY      — robot API key (dtk_live_*)
    TELEOP_ROBOT_ID     — robot identifier
    TELEOP_ROBOT_NAME   — human-readable robot name
"""

from __future__ import annotations

import asyncio
from collections import defaultdict
from collections.abc import Callable
import os
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

    BROKER_AVAILABLE = True
except ImportError:
    BROKER_AVAILABLE = False
    RTCConfiguration = None  # type: ignore[assignment,misc]
    RTCDataChannel = Any  # type: ignore[misc,assignment]
    RTCIceServer = None  # type: ignore[assignment,misc]
    RTCPeerConnection = None  # type: ignore[assignment,misc]
    RTCSessionDescription = None  # type: ignore[assignment,misc]
    httpx = None  # type: ignore[assignment]


_PLACEHOLDER_DC_ID = 100


class BrokerProvider(DataChannelProvider):
    """Cloudflare Realtime SFU provider via hosted teleop broker.

    The broker handles CF session creation and operator management.
    The robot just connects, heartbeats, and receives commands on a
    single multiplexed DataChannel.

    All subscribers receive all inbound bytes; filtering by message type
    (LCM fingerprint) is handled at the transport layer.
    """

    def __init__(
        self,
        broker_url: str | None = None,
        api_key: str | None = None,
        robot_id: str | None = None,
        robot_name: str | None = None,
        *,
        stun_url: str = "stun:stun.cloudflare.com:3478",
        heartbeat_hz: float = 1.0,
        reconnect_delay_s: float = 2.0,
        max_reconnect_attempts: int = 10,
        ordered: bool = False,
        max_retransmits: int | None = 0,
    ) -> None:
        if not BROKER_AVAILABLE:
            raise RuntimeError("aiortc and httpx required: pip install dimos[webrtc]")

        self._broker_url = (broker_url or os.environ.get("TELEOP_BROKER_URL", "")).rstrip("/")
        self._api_key = api_key or os.environ.get("TELEOP_API_KEY", "")
        self._robot_id = robot_id or os.environ.get("TELEOP_ROBOT_ID", "")
        self._robot_name = robot_name or os.environ.get("TELEOP_ROBOT_NAME", "robot")

        if not self._broker_url:
            raise RuntimeError("TELEOP_BROKER_URL or broker_url required")
        if not self._api_key:
            raise RuntimeError("TELEOP_API_KEY or api_key required")
        if not self._robot_id:
            raise RuntimeError("TELEOP_ROBOT_ID or robot_id required")

        self._stun_url = stun_url
        self._heartbeat_hz = heartbeat_hz
        self._reconnect_delay_s = reconnect_delay_s
        self._max_reconnect_attempts = max_reconnect_attempts
        self._ordered = ordered
        self._max_retransmits = max_retransmits

        # State
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._ready = threading.Event()
        self._stop_ev: asyncio.Event | None = None
        self._started = False
        self._lock = threading.RLock()

        # Session state
        self._session_id: str | None = None
        self._cf_session_id: str | None = None
        self._pc: RTCPeerConnection | None = None
        self._http: httpx.AsyncClient | None = None
        self._cmd_dc: RTCDataChannel | None = None
        self._cmd_dc_id: int | None = None
        self._has_operator = threading.Event()

        # Subscribers (topic → list of callbacks)
        self._callbacks: dict[str, list[Callable[[bytes, str], None]]] = defaultdict(list)

    @property
    def is_connected(self) -> bool:
        return self._started and self._pc is not None

    @property
    def has_operator(self) -> bool:
        """Whether an operator is currently connected."""
        return self._has_operator.is_set()

    @property
    def session_id(self) -> str | None:
        """Broker session ID (not CF session ID)."""
        return self._session_id

    @property
    def _headers(self) -> dict[str, str]:
        return {
            "Authorization": f"Bearer {self._api_key}",
            "Content-Type": "application/json",
        }

    # ─── Lifecycle ───────────────────────────────────────────────────

    def start(self) -> None:
        with self._lock:
            if self._started:
                return
            self._thread = threading.Thread(
                target=self._run_loop, daemon=True, name="broker-webrtc"
            )
            self._thread.start()
            if not self._ready.wait(timeout=5.0):
                raise RuntimeError("Broker provider event loop failed to start")
            self._run_sync(self._connect())
            self._started = True
            logger.info(
                "BrokerProvider started: session=%s robot=%s",
                self._session_id,
                self._robot_id,
            )

    def stop(self) -> None:
        with self._lock:
            if not self._started:
                return
            self._started = False
            if self._loop and self._loop.is_running():
                try:
                    self._run_sync(self._disconnect())
                except Exception:
                    logger.exception("Error during broker disconnect")
            if self._thread:
                self._thread.join(timeout=5.0)
            self._thread = None
            self._loop = None
            self._ready.clear()
            self._has_operator.clear()
            self._cmd_dc = None
            self._cmd_dc_id = None

    def _run_loop(self) -> None:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
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
            loop.close()

    def _run_sync(self, coro: Any, timeout: float = 30.0) -> Any:
        assert self._loop is not None
        return asyncio.run_coroutine_threadsafe(coro, self._loop).result(timeout=timeout)

    # ─── Connect / Disconnect ────────────────────────────────────────

    async def _connect(self) -> None:
        self._http = httpx.AsyncClient(timeout=30.0)
        ice = RTCConfiguration(iceServers=[RTCIceServer(urls=[self._stun_url])])
        self._pc = RTCPeerConnection(configuration=ice)

        # Create placeholder DC to force SCTP in SDP
        self._pc.createDataChannel("_placeholder", negotiated=True, id=_PLACEHOLDER_DC_ID)

        # Generate SDP offer
        offer = await self._pc.createOffer()
        await self._pc.setLocalDescription(offer)

        # Wait for ICE gathering
        if self._pc.iceGatheringState != "complete":
            ev = asyncio.Event()

            @self._pc.on("icegatheringstatechange")
            def _() -> None:
                if self._pc is not None and self._pc.iceGatheringState == "complete":
                    ev.set()

            await asyncio.wait_for(ev.wait(), 10.0)

        # Register with broker
        assert self._pc.localDescription is not None
        r = await self._http.post(
            f"{self._broker_url}/api/v1/sessions",
            headers=self._headers,
            json={
                "robot_id": self._robot_id,
                "robot_name": self._robot_name,
                "sdp_offer": self._pc.localDescription.sdp,
            },
        )
        if r.status_code not in (200, 201):
            raise RuntimeError(f"Broker session create failed: {r.status_code} {r.text}")

        data = r.json()
        self._session_id = data["session_id"]
        self._cf_session_id = data["cf_session_id"]

        # Set remote description (SDP answer from broker/CF)
        await self._pc.setRemoteDescription(
            RTCSessionDescription(sdp=data["sdp_answer"], type="answer")
        )

        # Wait for connection
        await self._wait_connected(self._pc)
        logger.info(
            "BrokerProvider connected to CF SFU: cf_session=%s",
            self._cf_session_id[:8] if self._cf_session_id else "?",
        )

        # Start heartbeat loop
        assert self._loop is not None
        self._loop.create_task(self._heartbeat_loop())

    async def _disconnect(self) -> None:
        # Delete session from broker
        if self._http and self._session_id:
            try:
                await self._http.delete(
                    f"{self._broker_url}/api/v1/sessions/{self._session_id}",
                    headers=self._headers,
                )
            except Exception:
                pass  # Best-effort cleanup

        if self._pc:
            await self._pc.close()
            self._pc = None
        if self._http:
            await self._http.aclose()
            self._http = None
        if self._stop_ev:
            self._stop_ev.set()

    @staticmethod
    async def _wait_connected(pc: RTCPeerConnection, timeout: float = 15.0) -> None:
        if pc.connectionState == "connected":
            return
        ev = asyncio.Event()

        @pc.on("connectionstatechange")
        def _() -> None:
            if pc.connectionState in ("connected", "failed", "closed"):
                ev.set()

        await asyncio.wait_for(ev.wait(), timeout)
        if pc.connectionState != "connected":
            raise RuntimeError(f"PeerConnection failed: {pc.connectionState}")

    # ─── Heartbeat ───────────────────────────────────────────────────

    async def _heartbeat_loop(self) -> None:
        """Periodic heartbeat to the broker. Detects operator join via ack."""
        interval = 1.0 / self._heartbeat_hz
        while self._started:
            try:
                await self._heartbeat_once()
            except Exception:
                logger.exception("Heartbeat error")
            await asyncio.sleep(interval)

    async def _heartbeat_once(self) -> None:
        if not self._http or not self._session_id:
            return

        r = await self._http.post(
            f"{self._broker_url}/api/v1/sessions/{self._session_id}/heartbeat",
            headers=self._headers,
            json={"safety_state": "nominal"},
        )
        if r.status_code != 200:
            logger.warning("Heartbeat failed: %d %s", r.status_code, r.text[:200])
            return

        data = r.json()
        dc_id = data.get("cmd_channel_subscriber_id")

        if dc_id is not None and self._cmd_dc is None:
            # Operator has joined and broker bridged the DataChannel
            await self._open_cmd_channel(int(dc_id))

    async def _open_cmd_channel(self, dc_id: int) -> None:
        """Create the negotiated DataChannel to receive operator commands."""
        if self._pc is None:
            return

        self._cmd_dc_id = dc_id
        ch = self._pc.createDataChannel(
            "cmd_unreliable",
            negotiated=True,
            id=dc_id,
            ordered=self._ordered,
            maxRetransmits=self._max_retransmits,
        )

        @ch.on("message")
        def _on_msg(payload: Any) -> None:
            if isinstance(payload, str):
                payload = payload.encode()
            # Deliver to ALL subscribers on all topics (transport-layer filters)
            for topic_cbs in list(self._callbacks.values()):
                for cb in list(topic_cbs):
                    try:
                        cb(payload, "cmd_unreliable")
                    except Exception:
                        logger.exception("BrokerProvider subscriber callback error")

        # Wait for channel to open
        if ch.readyState != "open":
            ev = asyncio.Event()

            @ch.on("open")
            def _on_open() -> None:
                ev.set()

            await asyncio.wait_for(ev.wait(), 15.0)

        self._cmd_dc = ch
        self._has_operator.set()
        logger.info("BrokerProvider: operator joined, cmd DC id=%d open", dc_id)

    # ─── Public API (DataChannelProvider) ────────────────────────────

    def publish(self, topic: str, data: bytes) -> None:
        """Send bytes to the operator (robot → operator direction).

        Note: outbound DataChannel (robot → operator) requires the broker
        to bridge a reverse channel. Currently not implemented in the
        broker — this is a placeholder for future bidirectional support.
        """
        if not self._started:
            self.start()
        # TODO: implement outbound DC bridging in broker
        logger.warning("BrokerProvider.publish() not yet supported (no outbound DC bridge)")

    def subscribe(self, topic: str, callback: Callable[[bytes, str], None]) -> Callable[[], None]:
        """Subscribe to inbound bytes from operator.

        All subscribers receive all messages on the multiplexed channel.
        Filtering by message type (LCM fingerprint) should be done at the
        transport layer.
        """
        if not self._started:
            self.start()
        with self._lock:
            self._callbacks[topic].append(callback)

        def _unsub() -> None:
            with self._lock:
                try:
                    self._callbacks[topic].remove(callback)
                except ValueError:
                    pass

        return _unsub

    def wait_for_operator(self, timeout: float | None = None) -> bool:
        """Block until an operator connects. Returns True if connected."""
        return self._has_operator.wait(timeout=timeout)


__all__ = ["BROKER_AVAILABLE", "BrokerProvider"]
