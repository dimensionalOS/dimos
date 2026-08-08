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

"""LiveKit edge module for hosted teleoperation.

One module owns one LiveKit room and translates its data and media planes into
the existing hosted teleoperation ports. It deliberately does not implement a
generic DimOS transport.
"""

from __future__ import annotations

import asyncio
import contextlib
import importlib.util
import json
import threading
from typing import TYPE_CHECKING, Any

import numpy as np
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.teleop.hosted.livekit_broker_client import LiveKitBrokerClient, LiveKitSession
from dimos.teleop.hosted.robot_type import RobotType
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

LIVEKIT_AVAILABLE = (
    importlib.util.find_spec("livekit") is not None
    and importlib.util.find_spec("httpx") is not None
)

if TYPE_CHECKING:
    from livekit import rtc


class LiveKitTeleopConfig(ModuleConfig):
    """Broker access and connection policy for one robot LiveKit room."""

    broker_url: str | None = None
    api_key: str | None = None
    robot_id: str | None = None
    robot_name: str = "robot"
    robot_type: RobotType | None = None
    operator_identity: str | None = None
    heartbeat_hz: float = 1.0


class LiveKitTeleopModule(Module):
    """LiveKit room boundary for the hosted teleoperation module graph."""

    config: LiveKitTeleopConfig

    state_json: Out[bytes]
    camera_select: Out[bytes]
    cmd_raw: Out[bytes]
    cmd_vel_in: Out[TwistStamped]

    mux_image: In[Image]
    telemetry_out: In[bytes]
    cmd_ack: In[bytes]
    map_out: In[bytes]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._loop: asyncio.AbstractEventLoop | None = None
        self._worker: threading.Thread | None = None
        self._ready = threading.Event()
        self._stop_event = threading.Event()
        self._lock = threading.RLock()
        self._start_error: Exception | None = None
        self._room: rtc.Room | None = None
        self._video_source: rtc.VideoSource | None = None
        self._video_track: rtc.LocalVideoTrack | None = None
        self._video_publish_task: asyncio.Task[None] | None = None
        self._room_task: asyncio.Task[None] | None = None
        self._operator_present = False
        self._operator_lost = False
        self._broker = LiveKitBrokerClient(self.config.broker_url, self.config.api_key)

    @rpc
    def start(self) -> None:
        if not LIVEKIT_AVAILABLE:
            raise RuntimeError("livekit and httpx required: pip install dimos[livekit]")
        super().start()
        self._ready.clear()
        self._stop_event.clear()
        self._start_error = None
        self._register_streams()
        self._worker = threading.Thread(target=self._run, daemon=True, name="livekit-teleop")
        self._worker.start()
        if not self._ready.wait(timeout=30.0):
            self.stop()
            raise RuntimeError("LiveKit room did not connect within 30 seconds")
        if self._start_error is not None:
            error = self._start_error
            self.stop()
            raise RuntimeError("LiveKit room failed to connect") from error

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        with self._lock:
            loop, room_task = self._loop, self._room_task
        if loop is not None and room_task is not None and not room_task.done():
            loop.call_soon_threadsafe(room_task.cancel)

        worker = self._worker
        if worker is not None:
            worker.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            if worker.is_alive():
                logger.error("LiveKit teleop worker did not stop")
            else:
                self._worker = None
        super().stop()

    def _register_streams(self) -> None:
        self.register_disposable(Disposable(self.mux_image.subscribe(self._publish_video)))
        self.register_disposable(
            Disposable(
                self.telemetry_out.subscribe(
                    lambda data: self._publish_data("state_reliable_back", data, reliable=True)
                )
            )
        )
        self.register_disposable(
            Disposable(
                self.cmd_ack.subscribe(
                    lambda data: self._publish_data("state_reliable_back", data, reliable=True)
                )
            )
        )
        self.register_disposable(
            Disposable(
                self.map_out.subscribe(
                    lambda data: self._publish_data("map_unreliable", data, reliable=False)
                )
            )
        )

    def _run(self) -> None:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        room_task = loop.create_task(self._run_room())
        with self._lock:
            self._loop = loop
            self._room_task = room_task
        try:
            with contextlib.suppress(asyncio.CancelledError):
                loop.run_until_complete(room_task)
        finally:
            pending = asyncio.all_tasks(loop)
            for task in pending:
                task.cancel()
            loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            loop.close()
            with self._lock:
                self._loop = None
                self._room_task = None

    async def _run_room(self) -> None:
        session: LiveKitSession | None = None
        heartbeat_task: asyncio.Task[None] | None = None
        disconnected = asyncio.Event()
        try:
            session = await self._broker.create_session(
                self.config.robot_id, self.config.robot_name, self.config.robot_type
            )
            await self._connect_room(session, disconnected)
            heartbeat_task = asyncio.create_task(
                self._broker.heartbeat(session.session_id, self.config.heartbeat_hz)
            )
            self._ready.set()
            while not self._stop_event.is_set() and not disconnected.is_set():
                await asyncio.sleep(0.1)
            if disconnected.is_set() and not self._stop_event.is_set():
                logger.warning("LiveKit room disconnected; ending teleop session")
        except Exception as error:
            self._start_error = error
            self._ready.set()
        finally:
            if heartbeat_task is not None:
                heartbeat_task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await heartbeat_task
            await self._disconnect_room()
            if session is not None:
                await self._broker.close_session(session.session_id)

    async def _connect_room(self, session: LiveKitSession, disconnected: asyncio.Event) -> None:
        from livekit import rtc

        room = rtc.Room()
        with self._lock:
            self._room = room

        @room.on("data_received")  # type: ignore[untyped-decorator]
        def _data_received(packet: Any) -> None:
            self._on_data(
                getattr(packet, "topic", "") or "",
                bytes(packet.data),
                getattr(packet, "participant", None),
            )

        @room.on("participant_connected")  # type: ignore[untyped-decorator]
        def _participant_connected(participant: Any) -> None:
            if self._is_operator(participant):
                with self._lock:
                    self._operator_present = True
                    self._operator_lost = False

        @room.on("participant_disconnected")  # type: ignore[untyped-decorator]
        def _participant_disconnected(participant: Any) -> None:
            identity = getattr(participant, "identity", None)
            if self._is_operator(participant) and not self._has_operator(
                room, excluded_identity=identity
            ):
                self._on_operator_lost()

        @room.on("disconnected")  # type: ignore[untyped-decorator]
        def _disconnected(_reason: Any) -> None:
            self._on_operator_lost()
            disconnected.set()

        await room.connect(session.url, session.token)
        with self._lock:
            self._operator_present = self._has_operator(room)
            self._operator_lost = False
        logger.info("LiveKit teleop connected", room=session.room)

    async def _disconnect_room(self) -> None:
        with self._lock:
            room = self._room
            self._room = None
            self._video_source = None
            self._video_track = None
            self._video_publish_task = None
        if room is not None:
            with contextlib.suppress(Exception):
                await room.disconnect()

    def _is_operator(self, participant: Any) -> bool:
        identity = getattr(participant, "identity", None)
        if self.config.operator_identity is not None:
            return identity == self.config.operator_identity
        # The hosted broker mints operator identities as op-<user_id> and
        # viewer identities as viewer-<user_id>-<suffix>.
        return isinstance(identity, str) and identity.startswith("op-")

    def _has_operator(self, room: rtc.Room, excluded_identity: str | None = None) -> bool:
        return any(
            self._is_operator(participant)
            and getattr(participant, "identity", None) != excluded_identity
            for participant in room.remote_participants.values()
        )

    def _publish_data(self, topic: str, data: bytes, reliable: bool) -> None:
        with self._lock:
            loop, room = self._loop, self._room
        if loop is None or room is None or not loop.is_running():
            return
        future = asyncio.run_coroutine_threadsafe(
            room.local_participant.publish_data(bytes(data), reliable=reliable, topic=topic), loop
        )
        future.add_done_callback(self._log_publish_failure)

    def _publish_video(self, image: Image) -> None:
        with self._lock:
            loop, room = self._loop, self._room
        if loop is None or room is None or not loop.is_running():
            return
        try:
            width, height, rgba = self._image_to_rgba(image)
        except Exception:
            logger.warning("LiveKit video frame conversion failed", exc_info=True)
            return
        loop.call_soon_threadsafe(self._capture_video, width, height, rgba)

    def _on_data(self, topic: str, data: bytes, participant: Any = None) -> None:
        if not self._is_operator(participant):
            return
        if topic == "state_reliable":
            self.state_json.publish(data)
            self.camera_select.publish(data)
        elif topic == "cmd_unreliable":
            self.cmd_raw.publish(data)
            with contextlib.suppress(Exception):
                self.cmd_vel_in.publish(TwistStamped.lcm_decode(data))

    def _on_operator_lost(self) -> None:
        with self._lock:
            if self._stop_event.is_set() or not self._operator_present or self._operator_lost:
                return
            self._operator_lost = True
            self._operator_present = False
        logger.warning("LiveKit operator link lost")
        self.state_json.publish(json.dumps({"type": "operator_lost"}).encode())

    def _capture_video(self, width: int, height: int, rgba: bytes) -> None:
        from livekit import rtc

        if self._room is None:
            return
        if self._video_source is None:
            self._video_source = rtc.VideoSource(width, height)
            self._video_publish_task = asyncio.create_task(self._publish_video_track())
        self._video_source.capture_frame(
            rtc.VideoFrame(width, height, rtc.VideoBufferType.RGBA, rgba)
        )

    async def _publish_video_track(self) -> None:
        from livekit import rtc

        with self._lock:
            room, source = self._room, self._video_source
        if room is None or source is None:
            return
        try:
            track = rtc.LocalVideoTrack.create_video_track("camera", source)
            await room.local_participant.publish_track(
                track, rtc.TrackPublishOptions(source=rtc.TrackSource.SOURCE_CAMERA)
            )
        except Exception:
            with self._lock:
                self._video_source = None
                self._video_track = None
                self._video_publish_task = None
            logger.warning("LiveKit video track publish failed", exc_info=True)
            return
        with self._lock:
            self._video_track = track

    @staticmethod
    def _image_to_rgba(image: Image) -> tuple[int, int, bytes]:
        data = image.data
        if data.dtype == np.uint16:
            data = (data >> 8).astype(np.uint8)
        elif data.dtype != np.uint8:
            data = data.astype(np.uint8)
        height, width = data.shape[:2]
        if image.format == ImageFormat.RGBA:
            rgba = data
        elif image.format == ImageFormat.BGRA:
            rgba = data[..., [2, 1, 0, 3]]
        elif image.format == ImageFormat.RGB:
            rgba = np.dstack([data, np.full((height, width), 255, dtype=np.uint8)])
        elif image.format in (ImageFormat.GRAY, ImageFormat.GRAY16):
            gray = data if data.ndim == 2 else data[..., 0]
            rgba = np.dstack([gray, gray, gray, np.full((height, width), 255, dtype=np.uint8)])
        else:
            rgba = np.dstack(
                [
                    data[..., 2],
                    data[..., 1],
                    data[..., 0],
                    np.full((height, width), 255, dtype=np.uint8),
                ]
            )
        return width, height, np.ascontiguousarray(rgba).tobytes()

    @staticmethod
    def _log_publish_failure(future: Any) -> None:
        with contextlib.suppress(asyncio.CancelledError):
            try:
                future.result()
            except Exception:
                logger.warning("LiveKit data publish failed", exc_info=True)
