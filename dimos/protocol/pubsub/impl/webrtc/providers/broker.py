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

Credentials reach ``BrokerConfig`` via the standard blueprint config flow —
either ``-o transports.broker.api_key=dtk_live_...`` on the CLI or the
``TRANSPORTS__BROKER__API_KEY=...`` env form. The broker derives the robot
identity from the API key; ``robot_id`` is optional.
"""

from __future__ import annotations

import asyncio
from collections import defaultdict
from collections.abc import Callable
import contextlib
import json
import time
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


class BrokerConfig(ProviderConfig):
    """Hosted teleop broker access. ``api_key`` is required; the rest defaults."""

    broker_url: str = "https://teleop.dimensionalos.com"
    api_key: str | None = None
    robot_id: str | None = None
    robot_name: str = "robot"
    stun_url: str = "stun:stun.cloudflare.com:3478"
    heartbeat_hz: float = 1.0
    ordered: bool = False
    max_retransmits: int | None = 0
    # Preferred video codec ("h264" / "vp8"; "" = aiortc default order).
    # Opt-in: falls back to defaults if the codec isn't in local capabilities.
    video_codec: str = ""

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
        if not config.api_key:
            raise RuntimeError(
                "BrokerConfig.api_key required "
                "(set -o transports.broker.api_key=dtk_live_... or "
                "TRANSPORTS__BROKER__API_KEY=dtk_live_...; "
                "create one in the teleop dashboard: New Key)"
            )
        self._broker_url = config.broker_url.rstrip("/")
        self._api_key = config.api_key
        self._robot_id = config.robot_id or ""
        self._robot_name = config.robot_name
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

        # Roll back partial state on failure so a retry doesn't leak.
        try:
            self._http = httpx.AsyncClient(timeout=30.0)
            # MAX_BUNDLE + the id=0 throwaway channel are CF/aiortc workarounds —
            # see dimos/teleop/quest_hosted/README.md before changing.
            self._pc = RTCPeerConnection(
                RTCConfiguration(
                    iceServers=await self._fetch_ice_servers(),
                    bundlePolicy=RTCBundlePolicy.MAX_BUNDLE,
                )
            )
            # addTrack must precede createDataChannel (CF/aiortc workaround).
            self._pc.addTrack(self._video_track)
            if self._config.video_codec:
                self._prefer_video_codec(self._config.video_codec)
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
                    # robot_id is optional — broker derives it from the API key.
                    **({"robot_id": self._robot_id} if self._robot_id else {}),
                    "robot_name": self._robot_name,
                    "sdp_offer": self._pc.localDescription.sdp,
                },
            )
            if r.status_code not in (200, 201):
                # 200-char cap — SDP carries short-lived ICE ufrag/pwd.
                raise RuntimeError(f"Broker session create failed: {r.status_code} {r.text[:200]}")
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
        except Exception:
            await self._disconnect()
            raise

    def _prefer_video_codec(self, codec: str) -> None:
        """Reorder the video transceiver's codec preferences (e.g. h264 first).

        Best-effort: unknown codec → warn and keep aiortc's default order, so a
        misconfigured knob can't kill the connection."""
        from aiortc import RTCRtpSender

        want = f"video/{codec}".lower()
        caps = RTCRtpSender.getCapabilities("video")
        preferred = [c for c in caps.codecs if c.mimeType.lower() == want]
        if not preferred:
            logger.warning("video_codec=%r not in local capabilities — using defaults", codec)
            return
        rest = [c for c in caps.codecs if c.mimeType.lower() != want]
        assert self._pc is not None
        for t in self._pc.getTransceivers():
            if t.kind == "video":
                t.setCodecPreferences(preferred + rest)
                logger.info("video codec preference: %s first", want)

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
        # Stop after 5 consecutive 401/404 — session force-deleted or key
        # revoked; otherwise the loop log-floods at heartbeat_hz forever.
        terminal_streak = 0
        while True:
            try:
                status = await self._heartbeat_once()
            except Exception:
                logger.exception("Broker heartbeat failed")
                status = None
            if status in (401, 404):
                terminal_streak += 1
                if terminal_streak >= 5:
                    logger.error(
                        "Heartbeat terminal: %d consecutive %d responses — stopping loop",
                        terminal_streak,
                        status,
                    )
                    return
            else:
                terminal_streak = 0
            await asyncio.sleep(interval)

    async def _heartbeat_once(self) -> int | None:
        """Return the HTTP status code (or None if skipped)."""
        if self._http is None or self.session_id is None:
            return None
        r = await self._http.post(
            f"{self._broker_url}/api/v1/sessions/{self.session_id}/heartbeat",
            headers=self._headers,
            json={},
        )
        if r.status_code != 200:
            logger.warning("Heartbeat failed: %d %s", r.status_code, r.text[:200])
            return r.status_code
        ack = r.json()
        # state_reliable_back first so the state_reliable ping handler can
        # find it in _dcs if a ping arrives during channel bring-up.
        ids = {
            "cmd_unreliable": ack.get("cmd_channel_subscriber_id"),
            "state_reliable_back": ack.get("state_back_channel_publisher_id"),
            "state_reliable": ack.get("state_channel_subscriber_id"),
        }
        # Track the broker's view: open on join, close on leave, re-open on
        # rejoin (the broker assigns fresh SCTP ids per operator session).
        for name, raw_id in ids.items():
            sctp_id = int(raw_id) if raw_id is not None else None
            if sctp_id != self._dc_ids.get(name):
                # Operator left (or was reaped): the command plane is gone.
                # Tell subscribers so the robot can stop motion / reset state.
                if (
                    name == "state_reliable"
                    and self._dc_ids.get(name) is not None
                    and sctp_id is None
                ):
                    self._notify_operator_lost()
                self._close_channel(name)
                self._dc_ids[name] = sctp_id
                if sctp_id is not None:
                    self._open_channel(name, sctp_id)
        return 200

    def _notify_operator_lost(self) -> None:
        """Synthetic {"type":"operator_lost"} to state_reliable subscribers.

        Rides the normal inbound plumbing (same thread, same callback shape as
        a real operator message) so modules get one uniform signal on both
        transports without new provider API.
        """
        payload = b'{"type": "operator_lost"}'
        with self._lock:
            callbacks = list(self._callbacks.get("state_reliable", ()))
        for cb in callbacks:
            try:
                cb(payload, "state_reliable")
            except Exception:
                logger.exception("operator_lost subscriber callback error")

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
                if name == "state_reliable":
                    self._maybe_answer_ping(payload)
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

    def _maybe_answer_ping(self, payload: bytes) -> None:
        """Answer the web client's clock-sync ping inline on the loop thread.

        The operator measures RTT/offset from ping→pong timing, so the reply
        must not ride a module hop (stream dispatch latency would inflate
        every sample, and keep-latest mailboxes could drop pings outright).
        The ping still fans out to subscribers afterwards — the provider stays
        a transparent relay with this one reflex attached.
        """
        if not payload.startswith(b"{"):
            return  # LCM binary or other non-JSON — not ours
        try:
            msg = json.loads(payload)
        except ValueError:
            return
        if msg.get("type") != "ping" or msg.get("client_ts") is None:
            return
        pong = json.dumps({"type": "pong", "client_ts": msg["client_ts"], "robot_ts": time.time()})
        with self._lock:
            ch = self._dcs.get("state_reliable_back")
        # Pong MUST go on state_reliable_back — CF bridges one direction only;
        # a robot send on state_reliable would be silently dropped.
        if ch is not None and ch.readyState == "open":
            ch.send(pong)
        else:
            logger.warning("ping received but state_reliable_back not open — pong dropped")

    # ─── Public API (Provider) ───────────────────────────────────────

    def publish(self, topic: str, data: bytes) -> None:
        """Robot → operator. Only outbound channels are publishable; messages
        drop while no operator is connected (the channel doesn't exist yet)."""
        from aiortc.exceptions import InvalidStateError

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

            # Recheck readyState on the loop thread — the channel can close
            # between here and the tick; aiortc's send() raises InvalidStateError
            # on a closed channel. Best-effort: a lost publish to a just-closed
            # operator channel is expected on disconnect, so warn (not raise).
            channel: RTCDataChannel = ch  # narrowed non-None above; capture for the closure

            def _send_safe() -> None:
                if channel.readyState != "open":
                    return
                try:
                    channel.send(data)
                except InvalidStateError:
                    logger.warning("Dropping %s publish: channel closed mid-send", topic)

            self._loop.call_soon_threadsafe(_send_safe)

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
