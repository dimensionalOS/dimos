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

"""Broker-mediated LiveKit provider (hosted teleop).

The robot asks the ``dimensional-teleop`` broker for a LiveKit room + JWT
(``POST /api/v1/sessions {transport:"livekit"}`` → ``{url, token, room}``),
then connects straight to the LiveKit SFU. Unlike the Cloudflare ``broker.py``
path there is no SDP relay, no SCTP-id juggling, and no heartbeat-driven
channel lifecycle: LiveKit data is bidirectional and topic-addressed, so a
single room carries every channel.

Topics (kept identical to the Cloudflare path so the typed-fingerprint demux at
the transport layer is unchanged):
    cmd_unreliable      operator → robot   commands (lossy)
    state_reliable      operator → robot   control plane (reliable)
    state_reliable_back robot → operator   telemetry (reliable)

Video: ``set_video_frame()`` pushes camera frames into a sendonly LiveKit track
(published lazily on the first frame) — typically via ``LiveKitVideoTransport``
bound to a blueprint's Image stream.

Config comes from the blueprint's ``transports.broker.*`` flow (env form
``TRANSPORTS__BROKER__<FIELD>``, or ``-o transports.broker.<field>=...``), the
same scheme as the Cloudflare ``BrokerConfig``:
    TRANSPORTS__BROKER__BROKER_URL  — default https://teleop.dimensionalos.com
    TRANSPORTS__BROKER__API_KEY     — robot API key (dtk_live_*); derives identity
    TRANSPORTS__BROKER__ROBOT_ID    — optional robot identifier override
    TRANSPORTS__BROKER__ROBOT_NAME  — human-readable robot name
"""

from __future__ import annotations

import asyncio
from collections import defaultdict
from collections.abc import Callable
import contextlib
import importlib.util
import json
import time
from typing import TYPE_CHECKING, Any

from dimos.protocol.pubsub.impl.webrtc.providers.spec import (
    AsyncProviderBase,
    ProviderConfig,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# find_spec instead of importing: the livekit rtc SDK pulls native libs and
# core.transport imports this module everywhere. Imported lazily on start().
LIVEKIT_AVAILABLE = (
    importlib.util.find_spec("livekit") is not None
    and importlib.util.find_spec("httpx") is not None
)

if TYPE_CHECKING:
    import httpx
    from livekit import rtc

    from dimos.msgs.sensor_msgs.Image import Image


class LiveKitBrokerConfig(ProviderConfig):
    """Hosted teleop over LiveKit. Config from transports.broker.* (TRANSPORTS__BROKER__*)."""

    broker_url: str | None = None
    api_key: str | None = None
    robot_id: str | None = None
    robot_name: str | None = None
    heartbeat_hz: float = 1.0
    # Publish-side encoder caps (0 = LiveKit defaults). Bounds uplink usage on
    # constrained links instead of letting congestion surface as drops/freezes.
    video_max_bitrate_bps: int = 0
    video_max_fps: float = 0.0

    def _create(self) -> LiveKitBrokerProvider:
        return LiveKitBrokerProvider(self)


def _image_to_rgba(img: Image) -> tuple[int, int, bytes]:
    """Pack a dimos Image into (width, height, RGBA bytes) for a LiveKit frame."""
    import numpy as np

    from dimos.msgs.sensor_msgs.Image import ImageFormat

    arr = img.data
    if arr.dtype == np.uint16:
        arr = (arr >> 8).astype(np.uint8)  # scale 16-bit (e.g. GRAY16) to 8-bit, not truncate
    elif arr.dtype != np.uint8:
        arr = arr.astype(np.uint8)
    h, w = arr.shape[:2]
    fmt = img.format
    if fmt == ImageFormat.RGBA:
        rgba = arr
    elif fmt == ImageFormat.BGRA:
        rgba = arr[..., [2, 1, 0, 3]]
    elif fmt == ImageFormat.RGB:
        rgba = np.dstack([arr, np.full((h, w), 255, np.uint8)])
    elif fmt in (ImageFormat.GRAY, ImageFormat.GRAY16):
        g = arr if arr.ndim == 2 else arr[..., 0]
        rgba = np.dstack([g, g, g, np.full((h, w), 255, np.uint8)])
    else:  # BGR and anything else: treat as BGR
        rgba = np.dstack([arr[..., 2], arr[..., 1], arr[..., 0], np.full((h, w), 255, np.uint8)])
    return w, h, np.ascontiguousarray(rgba).tobytes()


class _VideoPublisher:
    """Lazily-published sendonly LiveKit video track fed from an Image stream.

    Frames arrive from the producer thread; the source/track are created and the
    track published on the first frame (so dimensions come from real data), all
    marshalled onto the provider's loop thread where the room lives.
    """

    def __init__(self) -> None:
        self._room: rtc.Room | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._source: rtc.VideoSource | None = None
        self._publish_task: asyncio.Task[None] | None = None
        # (max_bitrate_bps, max_fps); zeros = LiveKit defaults.
        self._encoding: tuple[int, float] = (0, 0.0)

    def bind(
        self,
        room: rtc.Room,
        loop: asyncio.AbstractEventLoop,
        encoding: tuple[int, float] = (0, 0.0),
    ) -> None:
        self._room = room
        self._loop = loop
        self._encoding = encoding

    def reset(self) -> None:
        """Drop per-session state so a later bind() (reconnect) re-publishes the
        track on the new room. Called from the provider's _disconnect()."""
        self._room = None
        self._loop = None
        self._source = None
        self._publish_task = None

    def set_latest(self, img: Image) -> None:
        loop = self._loop
        if loop is None or not loop.is_running():
            return  # not connected yet; pre-connect frames are dropped
        try:
            w, h, buf = _image_to_rgba(img)
        except Exception:
            logger.debug("video: frame conversion failed", exc_info=True)
            return
        loop.call_soon_threadsafe(self._capture, w, h, buf)

    def _capture(self, w: int, h: int, buf: bytes) -> None:
        from livekit import rtc

        if self._source is None:
            self._source = rtc.VideoSource(w, h)
            self._publish_task = asyncio.ensure_future(self._publish())
        frame = rtc.VideoFrame(w, h, rtc.VideoBufferType.RGBA, buf)
        self._source.capture_frame(frame)

    async def _publish(self) -> None:
        from livekit import rtc

        assert self._room is not None and self._source is not None
        try:
            track = rtc.LocalVideoTrack.create_video_track("camera", self._source)
            opts = rtc.TrackPublishOptions(source=rtc.TrackSource.SOURCE_CAMERA)
            max_bitrate, max_fps = self._encoding
            if max_bitrate > 0 or max_fps > 0:
                opts.video_encoding = rtc.VideoEncoding(
                    max_bitrate=max_bitrate or 3_000_000,
                    max_framerate=max_fps or 30.0,
                )
            await self._room.local_participant.publish_track(track, opts)
        except Exception:
            # Clear _source so the next captured frame retries publish, instead
            # of feeding frames forever into a never-published source.
            logger.warning("LiveKit video track publish failed; will retry", exc_info=True)
            self._source = None
            self._publish_task = None
            return
        logger.info("LiveKit video track published")


class LiveKitBrokerProvider(AsyncProviderBase):
    """Bidirectional broker-mediated LiveKit provider.

    Inbound (operator → robot): ``cmd_unreliable`` + ``state_reliable``,
    delivered to subscribers by topic. Outbound (robot → operator):
    ``publish()`` on any topic (LiveKit is bidirectional); ``cmd_unreliable``
    rides the lossy channel, everything else reliable. Together with
    ``LiveKitTransport`` / ``LiveKitVideoTransport`` this is the LiveKit analog
    of ``BrokerProvider``.
    """

    LOSSY_TOPICS = ("cmd_unreliable",)

    def __init__(self, config: LiveKitBrokerConfig | None = None) -> None:
        if not LIVEKIT_AVAILABLE:
            raise RuntimeError("livekit and httpx required: pip install dimos[livekit]")
        super().__init__()
        config = config or LiveKitBrokerConfig()
        # Config is populated from transports.broker.* (= TRANSPORTS__BROKER__*
        # env / -o overrides), same scheme as the Cloudflare BrokerConfig.
        self._broker_url = (config.broker_url or "https://teleop.dimensionalos.com").rstrip("/")
        self._api_key = config.api_key or ""
        self._robot_id = config.robot_id or ""
        self._robot_name = config.robot_name or "robot"
        if not self._api_key:
            raise RuntimeError(
                "transports.broker.api_key required "
                "(TRANSPORTS__BROKER__API_KEY=dtk_live_...; create one in the "
                "teleop dashboard: New Key)"
            )
        self._config = config

        self._http: httpx.AsyncClient | None = None
        self._room: rtc.Room | None = None
        self.session_id: str | None = None
        self.room: str | None = None
        self._hb_task: asyncio.Task[None] | None = None
        self._video = _VideoPublisher()
        # topic → subscriber callbacks. Guarded by self._lock (from the base).
        self._callbacks: dict[str, list[Callable[[bytes, str], None]]] = defaultdict(list)

    @property
    def _headers(self) -> dict[str, str]:
        return {"X-Robot-API-Key": self._api_key, "Content-Type": "application/json"}

    # ─── Connect / Disconnect (loop thread) ──────────────────────────

    async def _connect(self) -> None:
        import httpx
        from livekit import rtc

        self._http = httpx.AsyncClient(timeout=30.0)
        r = await self._http.post(
            f"{self._broker_url}/api/v1/sessions",
            headers=self._headers,
            json={
                "transport": "livekit",
                "robot_name": self._robot_name,
                **({"robot_id": self._robot_id} if self._robot_id else {}),
            },
        )
        if r.status_code not in (200, 201):
            raise RuntimeError(f"Broker session create failed: {r.status_code} {r.text[:500]}")
        data = r.json()
        self.session_id = data["session_id"]
        self.room = data.get("room")
        url, token = data["url"], data["token"]

        self._room = rtc.Room()

        @self._room.on("data_received")  # type: ignore[untyped-decorator]
        def _on_data(packet: Any) -> None:
            self._dispatch(packet)

        # Operator leaving the room = command plane gone. Same synthetic
        # signal shape as BrokerProvider._notify_operator_lost so modules
        # handle one message on both transports.
        @self._room.on("participant_disconnected")  # type: ignore[untyped-decorator]
        def _on_participant_gone(_participant: Any) -> None:
            self._notify_operator_lost()

        await self._room.connect(url, token)
        self._video.bind(
            self._room,
            asyncio.get_running_loop(),
            encoding=(self._config.video_max_bitrate_bps, self._config.video_max_fps),
        )
        logger.info(
            "LiveKit broker provider connected: session=%s room=%s robot=%s",
            self.session_id,
            data.get("room"),
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
        if self._room is not None:
            with contextlib.suppress(Exception):
                await self._room.disconnect()
            self._room = None
        self._video.reset()  # clear per-session video state so a restart re-publishes
        if self._http:
            await self._http.aclose()
            self._http = None
        self.session_id = None

    # ─── Heartbeat (loop thread; metrics/liveness only) ──────────────

    async def _heartbeat_loop(self) -> None:
        interval = 1.0 / max(self._config.heartbeat_hz, 0.1)
        # Terminal condition mirrors BrokerProvider: 5 consecutive 401/404
        # means the key was revoked or the session force-deleted — retrying
        # forever just log-floods at heartbeat_hz.
        terminal_streak = 0
        while True:
            status: int | None = None
            try:
                if self._http is not None and self.session_id is not None:
                    r = await self._http.post(
                        f"{self._broker_url}/api/v1/sessions/{self.session_id}/heartbeat",
                        headers=self._headers,
                        json={},
                    )
                    status = r.status_code
                    if status != 200:
                        logger.warning("LiveKit heartbeat failed: %d %s", status, r.text[:200])
            except Exception:
                logger.warning("LiveKit heartbeat failed", exc_info=True)
            if status in (401, 404):
                terminal_streak += 1
                if terminal_streak >= 5:
                    logger.error(
                        "LiveKit heartbeat terminal: %d consecutive %d responses — stopping loop",
                        terminal_streak,
                        status,
                    )
                    return
            else:
                terminal_streak = 0
            await asyncio.sleep(interval)

    # ─── Dispatch (loop thread) ──────────────────────────────────────

    def _notify_operator_lost(self) -> None:
        """Synthetic {"type":"operator_lost"} to state_reliable subscribers
        (mirrors BrokerProvider — one uniform signal on both transports)."""
        payload = b'{"type": "operator_lost"}'
        with self._lock:
            callbacks = list(self._callbacks.get("state_reliable", ()))
        for cb in callbacks:
            try:
                cb(payload, "state_reliable")
            except Exception:
                logger.exception("operator_lost subscriber callback error")

    def _dispatch(self, packet: Any) -> None:
        topic = getattr(packet, "topic", "") or ""
        payload = getattr(packet, "data", b"")
        if isinstance(payload, (bytearray, memoryview)):
            payload = bytes(payload)
        if topic == "state_reliable":
            self._maybe_answer_ping(payload)
        with self._lock:
            callbacks = list(self._callbacks.get(topic, ()))
        for cb in callbacks:
            try:
                cb(payload, topic)
            except Exception:
                logger.exception("LiveKit subscriber callback error")

    def _maybe_answer_ping(self, payload: bytes) -> None:
        """Answer the operator's clock-sync ping inline on the loop thread.

        Mirrors BrokerProvider._maybe_answer_ping so LiveKit robots produce
        the same {client_ts, robot_ts} pong on state_reliable_back — without
        this the operator's RTT/offset never converge.
        """
        if not payload.startswith(b"{"):
            return
        try:
            msg = json.loads(payload)
        except ValueError:
            return
        if msg.get("type") != "ping" or msg.get("client_ts") is None:
            return
        pong = json.dumps(
            {"type": "pong", "client_ts": msg["client_ts"], "robot_ts": time.time()}
        ).encode()
        if self._room is None or self._loop is None:
            return
        coro = self._room.local_participant.publish_data(
            pong, reliable=True, topic="state_reliable_back"
        )
        asyncio.run_coroutine_threadsafe(coro, self._loop)

    # ─── Public API (Provider) ───────────────────────────────────────

    def publish(self, topic: str, data: bytes) -> None:
        """Robot → operator on any topic (LiveKit is bidirectional). Messages
        drop while no room/operator is connected — normal pubsub behaviour."""
        if isinstance(data, (bytearray, memoryview)):
            data = bytes(data)
        reliable = topic not in self.LOSSY_TOPICS
        with self._lock:
            if not self._started or self._loop is None or self._room is None:
                return
            coro = self._room.local_participant.publish_data(data, reliable=reliable, topic=topic)
            asyncio.run_coroutine_threadsafe(coro, self._loop)

    def set_video_frame(self, img: Image) -> None:
        """Robot → operator video: publish the latest camera frame (thread-safe)."""
        self._video.set_latest(img)

    def subscribe(self, topic: str, callback: Callable[[bytes, str], None]) -> Callable[[], None]:
        if not self.is_connected:
            self.start()
        with self._lock:
            self._callbacks[topic].append(callback)

        def _unsub() -> None:
            with self._lock:
                with contextlib.suppress(ValueError):
                    self._callbacks[topic].remove(callback)

        return _unsub


__all__ = ["LiveKitBrokerConfig", "LiveKitBrokerProvider"]
