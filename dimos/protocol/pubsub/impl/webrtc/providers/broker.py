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

"""Broker-mediated Cloudflare Realtime provider (hosted teleop).

The robot dials out to the ``dimensional-teleop`` broker, which owns CF
session creation and operator lifecycle. SCTP ids for all bridged channels
arrive via heartbeat acks; we open/close the negotiated channels to track
the broker's view (operator join / leave / rejoin).

Channel plan (topic == DataChannel name):
    cmd_unreliable      operator → robot   commands (unordered, lossy)
    state_reliable      operator → robot   control plane (reliable)
    state_reliable_back robot → operator   telemetry (reliable) — publishable

Video: the robot's session offer always carries one sendonly video track
(the broker stores its mid/track and the operator pulls it on join). Feed
frames with ``set_video_frame()`` — typically via ``CloudflareVideoTransport``
bound to a blueprint's Image stream; unfed, the track simply never emits.
The aiortc/CF quirks this inherits (MAX_BUNDLE, the id=0 throwaway channel)
are documented in ``dimos/teleop/quest_hosted/README.md``.

``robot_id`` is a per-transport label (set on ``CloudflareTransport`` /
``BrokerConfig``) distinguishing multiple robots under one key — it lives on
the transport, not the process, so two robots on one machine stay distinct.

Env vars (fallback when config fields are unset):
    TELEOP_BROKER_URL — default https://teleop.dimensionalos.com
    TELEOP_API_KEY    — robot API key (dtk_live_*); the broker derives the
                        robot identity from it
    TELEOP_ROBOT_NAME — human-readable robot name
"""

from __future__ import annotations

import asyncio
from collections import defaultdict
from collections.abc import Callable
import contextlib
from dataclasses import dataclass
import os
from typing import TYPE_CHECKING, Any

from dimos.protocol.pubsub.impl.webrtc.providers.sdp import propagate_bundle_candidates
from dimos.protocol.pubsub.impl.webrtc.providers.spec import (
    WEBRTC_AVAILABLE,
    AsyncProviderBase,
    ProviderConfig,
    wait_connected,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

if TYPE_CHECKING:
    from aiortc import RTCDataChannel, RTCIceServer, RTCPeerConnection
    import httpx


@dataclass(frozen=True)
class BrokerConfig(ProviderConfig):
    """Hosted teleop broker access. Credentials default from TELEOP_* env."""

    broker_url: str | None = None
    api_key: str | None = None
    robot_id: str | None = None
    robot_name: str | None = None
    stun_url: str = "stun:stun.cloudflare.com:3478"
    heartbeat_hz: float = 1.0
    ordered: bool = False
    max_retransmits: int | None = 0

    def _create(self) -> BrokerProvider:
        return BrokerProvider(self)


class BrokerProvider(AsyncProviderBase):
    """Bidirectional broker provider.

    Inbound (operator → robot): ``cmd_unreliable`` + ``state_reliable``;
    subscribers get the bytes of the channel matching their topic, and typed
    demux by LCM fingerprint happens at the transport layer. Outbound
    (robot → operator): ``publish()`` on ``state_reliable_back``; while no
    operator is connected the channel doesn't exist and messages drop, which
    is normal pubsub behaviour. Together with ``CloudflareTransport`` this
    replaces ``HostedTeleopModule`` for the data planes; video remains there
    until this provider grows media-track support.
    """

    INBOUND_CHANNELS = ("cmd_unreliable", "state_reliable")
    OUTBOUND_CHANNELS = ("state_reliable_back",)

    def __init__(self, config: BrokerConfig | None = None) -> None:
        if not WEBRTC_AVAILABLE:
            raise RuntimeError("aiortc and httpx required: pip install dimos[webrtc]")
        super().__init__()
        config = config or BrokerConfig()
        self._broker_url = (
            config.broker_url
            or os.environ.get("TELEOP_BROKER_URL", "https://teleop.dimensionalos.com")
        ).rstrip("/")
        self._api_key = config.api_key or os.environ.get("TELEOP_API_KEY", "")
        self._robot_id = config.robot_id or ""
        self._robot_name = config.robot_name or os.environ.get("TELEOP_ROBOT_NAME", "robot")
        if not self._api_key:
            raise RuntimeError(
                "TELEOP_API_KEY or BrokerConfig.api_key required "
                "(create one in the teleop dashboard: New Key)"
            )
        self._config = config

        self._http: httpx.AsyncClient | None = None
        self._pc: RTCPeerConnection | None = None
        self.session_id: str | None = None
        self._hb_task: asyncio.Task[None] | None = None
        # name → open negotiated channel / its SCTP id. Mutated on the loop
        # thread (heartbeat); read from any thread under self._lock.
        self._dcs: dict[str, RTCDataChannel] = {}
        self._dc_ids: dict[str, int | None] = {}
        self._dropped_publish_warned = False
        # Sendonly camera track, present in the initial offer so the broker
        # can bridge video without renegotiating the robot side.
        from dimos.protocol.pubsub.impl.webrtc.providers.video_track import CameraVideoTrack

        self._video_track = CameraVideoTrack()

        # Guarded by self._lock (from the base).
        self._callbacks: dict[str, list[Callable[[bytes, str], None]]] = defaultdict(list)

    @property
    def _headers(self) -> dict[str, str]:
        return {"X-Robot-API-Key": self._api_key, "Content-Type": "application/json"}

    # ─── Connect / Disconnect (loop thread) ──────────────────────────

    async def _fetch_ice_servers(self) -> list[RTCIceServer]:
        """STUN + short-lived TURN relay credentials minted by the broker.

        TURN must be in the PC's config at construction for relay candidates
        to gather with the offer; robots on UDP-blocked networks (CGNAT,
        corporate) only connect via the turns:443 relay. Best-effort —
        STUN-only on any failure or when the broker has no TURN configured.
        """
        from aiortc import RTCIceServer

        assert self._http is not None
        stun_only = [RTCIceServer(urls=[self._config.stun_url])]
        try:
            r = await self._http.get(
                f"{self._broker_url}/api/v1/sessions/turn-credentials",
                headers=self._headers,
            )
            if r.status_code != 200:
                logger.warning("TURN credential fetch failed (%d); STUN only", r.status_code)
                return stun_only
            servers = [
                RTCIceServer(
                    urls=s["urls"],
                    username=s.get("username"),
                    credential=s.get("credential"),
                )
                for s in r.json().get("ice_servers", [])
                if s.get("urls")
            ]
            return servers or stun_only
        except Exception:
            logger.warning("TURN credential fetch failed; STUN only", exc_info=True)
            return stun_only

    async def _connect(self) -> None:
        from aiortc import (
            RTCBundlePolicy,
            RTCConfiguration,
            RTCPeerConnection,
            RTCSessionDescription,
        )
        import httpx

        self._http = httpx.AsyncClient(timeout=30.0)
        # MAX_BUNDLE + the id=0 throwaway channel are CF/aiortc workarounds —
        # see dimos/teleop/quest_hosted/README.md before changing.
        self._pc = RTCPeerConnection(
            RTCConfiguration(
                iceServers=await self._fetch_ice_servers(),
                bundlePolicy=RTCBundlePolicy.MAX_BUNDLE,
            )
        )
        # addTrack must precede createDataChannel (CF/aiortc workaround — see
        # the quest_hosted README).
        self._pc.addTrack(self._video_track)
        self._pc.createDataChannel("_sctp_init", negotiated=True, id=0)

        offer = await self._pc.createOffer()
        await self._pc.setLocalDescription(offer)
        if self._pc.iceGatheringState != "complete":
            ev = asyncio.Event()
            pc = self._pc

            @pc.on("icegatheringstatechange")
            def _on_gathering() -> None:
                if pc.iceGatheringState == "complete":
                    ev.set()

            await asyncio.wait_for(ev.wait(), 10.0)

        r = await self._http.post(
            f"{self._broker_url}/api/v1/sessions",
            headers=self._headers,
            json={
                # robot_id is optional — the broker derives it from the API
                # key; sending it only adds a consistency check.
                **({"robot_id": self._robot_id} if self._robot_id else {}),
                "robot_name": self._robot_name,
                "sdp_offer": self._pc.localDescription.sdp,
            },
        )
        if r.status_code not in (200, 201):
            raise RuntimeError(f"Broker session create failed: {r.status_code} {r.text[:500]}")
        data = r.json()
        self.session_id = data["session_id"]
        await self._pc.setRemoteDescription(
            RTCSessionDescription(
                sdp=propagate_bundle_candidates(data["sdp_answer"]), type="answer"
            )
        )
        await wait_connected(self._pc)
        self._video_track.arm()  # deliver frames from "now", not boot
        logger.info(
            "Broker provider connected: session=%s robot=%s",
            self.session_id,
            self._robot_id or "(derived from API key)",
        )
        self._hb_task = asyncio.get_running_loop().create_task(self._heartbeat_loop())

    async def _disconnect(self) -> None:
        if self._hb_task is not None:
            self._hb_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._hb_task
            self._hb_task = None
        if self._http and self.session_id:
            with contextlib.suppress(Exception):  # best-effort deregistration
                await self._http.delete(
                    f"{self._broker_url}/api/v1/sessions/{self.session_id}",
                    headers=self._headers,
                )
        for name in list(self._dcs):
            self._close_channel(name)
        if self._pc:
            await self._pc.close()
            self._pc = None
        if self._http:
            await self._http.aclose()
            self._http = None
        self.session_id = None

    # ─── Heartbeat (loop thread; cancelled in _disconnect) ───────────

    async def _heartbeat_loop(self) -> None:
        interval = 1.0 / max(self._config.heartbeat_hz, 0.1)
        while True:
            try:
                await self._heartbeat_once()
            except Exception:
                logger.exception("Broker heartbeat failed")
            await asyncio.sleep(interval)

    async def _heartbeat_once(self) -> None:
        if self._http is None or self.session_id is None:
            return
        r = await self._http.post(
            f"{self._broker_url}/api/v1/sessions/{self.session_id}/heartbeat",
            headers=self._headers,
            json={},
        )
        if r.status_code != 200:
            logger.warning("Heartbeat failed: %d %s", r.status_code, r.text[:200])
            return
        ack = r.json()
        ids = {
            "cmd_unreliable": ack.get("cmd_channel_subscriber_id"),
            "state_reliable": ack.get("state_channel_subscriber_id"),
            "state_reliable_back": ack.get("state_back_channel_publisher_id"),
        }
        # Track the broker's view: open on join, close on leave, re-open on
        # rejoin (the broker assigns fresh SCTP ids per operator session).
        for name, raw_id in ids.items():
            sctp_id = int(raw_id) if raw_id is not None else None
            if sctp_id != self._dc_ids.get(name):
                self._close_channel(name)
                self._dc_ids[name] = sctp_id
                if sctp_id is not None:
                    self._open_channel(name, sctp_id)

    def _channel_options(self, name: str) -> dict[str, Any]:
        if name == "cmd_unreliable":
            return {"ordered": self._config.ordered, "maxRetransmits": self._config.max_retransmits}
        return {"ordered": True}  # state channels are reliable

    def _open_channel(self, name: str, sctp_id: int) -> None:
        assert self._pc is not None
        logger.info("Opening negotiated %s on SCTP id %d", name, sctp_id)
        ch = self._pc.createDataChannel(
            name, negotiated=True, id=sctp_id, **self._channel_options(name)
        )

        if name in self.INBOUND_CHANNELS:

            @ch.on("message")
            def _on_msg(payload: Any) -> None:
                if isinstance(payload, str):
                    payload = payload.encode()
                with self._lock:
                    callbacks = list(self._callbacks.get(name, ()))
                for cb in callbacks:
                    try:
                        cb(payload, name)
                    except Exception:
                        logger.exception("Broker subscriber callback error")

        with self._lock:
            self._dcs[name] = ch

    def _close_channel(self, name: str) -> None:
        with self._lock:
            ch = self._dcs.pop(name, None)
        if ch is not None:
            with contextlib.suppress(Exception):
                ch.close()

    # ─── Public API (Provider) ───────────────────────────────────────

    def publish(self, topic: str, data: bytes) -> None:
        """Robot → operator. Only outbound channels are publishable; messages
        drop while no operator is connected (the channel doesn't exist yet)."""
        if topic not in self.OUTBOUND_CHANNELS:
            raise ValueError(
                f"Robot can only publish on {self.OUTBOUND_CHANNELS}; "
                f"{topic!r} is an operator→robot channel"
            )
        if isinstance(data, (bytearray, memoryview)):
            data = bytes(data)
        with self._lock:
            if not self._started or self._loop is None:
                return
            ch = self._dcs.get(topic)
            if ch is None or ch.readyState != "open":
                if not self._dropped_publish_warned:
                    self._dropped_publish_warned = True
                    logger.info("Dropping %s publish: no operator connected", topic)
                return
            self._dropped_publish_warned = False
            self._loop.call_soon_threadsafe(ch.send, data)

    def set_video_frame(self, img: Any) -> None:
        """Robot → operator video: publish the latest camera frame.

        Thread-safe; frames are dropped until the PC is connected and armed.
        """
        self._video_track.set_latest(img)

    def subscribe(self, topic: str, callback: Callable[[bytes, str], None]) -> Callable[[], None]:
        """Subscribers receive the bytes of the inbound channel matching
        their topic; the transport layer filters by LCM fingerprint."""
        if not self.is_connected:
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


__all__ = ["BrokerConfig", "BrokerProvider"]
