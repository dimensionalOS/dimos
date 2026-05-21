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

"""Real-hardware Go2 connection over Unitree's WebRTC stack.

Composes `UnitreeWebRtcSession` for the asyncio loop / handshake / move /
publish_request. This file adds Go2-specific stream wiring (lidar, odom,
video, camera_info), sport-mode RPCs, and TF publishing.
"""

from __future__ import annotations

from dataclasses import dataclass
from threading import Event, Thread
import time
from typing import Any, TypeAlias, TypeVar

import numpy as np
from numpy.typing import NDArray
from reactivex import operators as ops
from reactivex.disposable import Disposable
from reactivex.observable import Observable
from reactivex.subject import Subject
import rerun.blueprint as rrb
from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.connection_registry import connection
from dimos.robot.tf_utils import odom_to_tf
from dimos.robot.unitree.go2.camera import _camera_info_static
from dimos.robot.unitree.go2.config import ConnectionConfig, Go2Mode
from dimos.robot.unitree.type.lidar import pointcloud2_from_webrtc_lidar
from dimos.robot.unitree.type.odometry import Odometry as OdometryConverter
from dimos.robot.unitree.webrtc_session import UnitreeWebRtcSession
from dimos.spec.perception import Camera, Pointcloud
from dimos.types.timestamped import Timestamped
from dimos.utils.decorators.decorators import simple_mcache
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

VideoMessage: TypeAlias = NDArray[np.uint8]

logger = setup_logger()

_T = TypeVar("_T", bound=Timestamped)


def time_is_now(x: _T) -> _T:
    x.ts = time.time()
    return x


@dataclass
class SerializableVideoFrame:
    """Pickleable wrapper for av.VideoFrame with all metadata."""

    data: np.ndarray  # type: ignore[type-arg]
    pts: int | None = None
    time: float | None = None
    dts: int | None = None
    width: int | None = None
    height: int | None = None
    format: str | None = None

    @classmethod
    def from_av_frame(cls, frame: Any) -> SerializableVideoFrame:
        return cls(
            data=frame.to_ndarray(format="rgb24"),
            pts=frame.pts,
            time=frame.time,
            dts=frame.dts,
            width=frame.width,
            height=frame.height,
            format=frame.format.name if hasattr(frame, "format") and frame.format else None,
        )

    def to_ndarray(self, format: str | None = None) -> np.ndarray:  # type: ignore[type-arg]
        return self.data


_SPORT_API_ID_RAGEMODE: int = 2059


@connection(robot="go2", backend="webrtc")
class Go2WebRtcConnection(Module, Camera, Pointcloud):
    """Real-hardware Go2 connection over Unitree's WebRTC stack."""

    dedicated_worker = True

    config: ConnectionConfig
    cmd_vel: In[Twist]
    pointcloud: Out[PointCloud2]
    odom: Out[PoseStamped]
    lidar: Out[PointCloud2]
    color_image: Out[Image]
    camera_info: Out[CameraInfo]

    camera_info_static: CameraInfo = _camera_info_static()
    _camera_info_thread: Thread | None = None
    _latest_video_frame: Image | None = None

    @classmethod
    def rerun_views(cls):  # type: ignore[no-untyped-def]
        return [
            rrb.Spatial2DView(
                name="Camera",
                origin="world/robot/camera/rgb",
            ),
        ]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self.session = UnitreeWebRtcSession(self.config.ip)

    @rpc
    def start(self) -> None:
        super().start()
        self.session.start()

        def onimage(image: Image) -> None:
            self.color_image.publish(image)
            self._latest_video_frame = image

        self.register_disposable(self._lidar_stream().subscribe(self.lidar.publish))
        self.register_disposable(self._odom_stream().subscribe(self._publish_tf))
        self.register_disposable(self._video_stream().subscribe(onimage))
        self.register_disposable(Disposable(self.cmd_vel.subscribe(self.move)))

        self._camera_info_thread = Thread(
            target=self._publish_camera_info,
            daemon=True,
        )
        self._camera_info_thread.start()

        self.standup()
        time.sleep(3)
        self.balance_stand()

        if self.config.mode == Go2Mode.RAGE:
            self.enable_rage_mode()

        self.set_obstacle_avoidance(self.config.g.obstacle_avoidance)

    @rpc
    def stop(self) -> None:
        self.liedown()
        self.session.stop()

        from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT

        if self._camera_info_thread and self._camera_info_thread.is_alive():
            self._camera_info_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)

        super().stop()

    @simple_mcache
    def _raw_lidar_stream(self) -> Observable:  # type: ignore[type-arg]
        return backpressure(self.session.sub_stream(RTC_TOPIC["ULIDAR_ARRAY"]))

    @simple_mcache
    def _raw_odom_stream(self) -> Observable:  # type: ignore[type-arg]
        return backpressure(self.session.sub_stream(RTC_TOPIC["ROBOTODOM"]))

    @simple_mcache
    def _lidar_stream(self) -> Observable[PointCloud2]:
        return backpressure(
            self._raw_lidar_stream().pipe(
                ops.map(pointcloud2_from_webrtc_lidar),
                ops.map(time_is_now),
            )
        )

    @simple_mcache
    def _odom_stream(self) -> Observable[PoseStamped]:
        return backpressure(
            self._raw_odom_stream().pipe(
                ops.map(OdometryConverter.from_msg),
                ops.map(time_is_now),
            )
        )

    @simple_mcache
    def _video_stream(self) -> Observable[Image]:
        return backpressure(
            self._raw_video_stream().pipe(
                ops.filter(lambda frame: frame is not None),
                ops.map(
                    lambda frame: Image.from_numpy(
                        frame.to_ndarray(format="rgb24"),  # type: ignore[attr-defined]
                        format=ImageFormat.RGB,
                        frame_id="camera_optical",
                    ),
                ),
                ops.map(time_is_now),
            )
        )

    @simple_mcache
    def _raw_video_stream(self) -> Observable[SerializableVideoFrame]:
        subject: Subject[SerializableVideoFrame] = Subject()
        stop_event = Event()

        from aiortc import MediaStreamTrack

        conn = self.session.conn
        loop = self.session.loop

        async def accept_track(track: MediaStreamTrack) -> None:
            while True:
                if stop_event.is_set():
                    return
                frame = await track.recv()
                serializable_frame = SerializableVideoFrame.from_av_frame(frame)
                subject.on_next(serializable_frame)

        conn.video.add_track_callback(accept_track)

        def switch_video_channel() -> None:
            conn.video.switchVideoChannel(True)

        loop.call_soon_threadsafe(switch_video_channel)

        def stop() -> None:
            stop_event.set()
            conn.video.track_callbacks.remove(accept_track)

            def switch_video_channel_off() -> None:
                conn.video.switchVideoChannel(False)

            loop.call_soon_threadsafe(switch_video_channel_off)

        return subject.pipe(ops.finally_action(stop))

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        """Send movement command to robot."""
        return self.session.move(twist, duration)

    @rpc
    def standup(self) -> bool:
        """Make the robot stand up."""
        return bool(
            self.session.publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["StandUp"]})
        )

    @rpc
    def liedown(self) -> bool:
        """Make the robot lie down."""
        return bool(
            self.session.publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["StandDown"]})
        )

    @rpc
    def balance_stand(self) -> bool:
        """Enter BalanceStand: neutral state for switching locomotion modes."""
        return bool(
            self.session.publish_request(
                RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["BalanceStand"]}
            )
        )

    @rpc
    def enable_rage_mode(self) -> bool:
        """Enable Rage Mode (~2.5 m/s forward velocity envelope).

        Ensures BalanceStand precondition regardless of current FSM state.
        """
        # Force BalanceStand first.
        self.session.publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD["BalanceStand"]})
        time.sleep(0.3)

        rage_ok = bool(
            self.session.publish_request(
                RTC_TOPIC["SPORT_MOD"],
                {"api_id": _SPORT_API_ID_RAGEMODE, "parameter": {"data": True}},
            )
        )
        time.sleep(2.0)

        joystick_ok = bool(
            self.session.publish_request(
                RTC_TOPIC["SPORT_MOD"],
                {"api_id": SPORT_CMD["SwitchJoystick"], "parameter": {"data": True}},
            )
        )
        logger.info("Rage Mode enabled")
        return rage_ok and joystick_ok

    def set_obstacle_avoidance(self, enabled: bool = True) -> None:
        self.session.publish_request(
            RTC_TOPIC["OBSTACLES_AVOID"],
            {"api_id": 1001, "parameter": {"enable": int(enabled)}},
        )

    @rpc
    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any]:
        """Publish a request to the underlying connection."""
        return self.session.publish_request(topic, data)  # type: ignore[no-any-return]

    def _publish_tf(self, msg: PoseStamped) -> None:
        self.tf.publish(*odom_to_tf(msg))
        self.odom.publish(msg)

    def _publish_camera_info(self) -> None:
        while True:
            self.camera_info.publish(self.camera_info_static)
            time.sleep(1.0)

    @skill
    def observe(self) -> Image | None:
        """Returns the latest video frame from the robot camera. Use this skill for any visual world queries.

        This skill provides the current camera view for perception tasks.
        Returns None if no frame has been captured yet.
        """
        return self._latest_video_frame
