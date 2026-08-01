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

from collections.abc import Callable
import functools
from typing import Any

from reactivex import Observable, Subject

from dimos.core.global_config import GlobalConfig
from dimos.core.transport import PubSubTransport
from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.simulation.dimsim.dimsim_process import DimSimProcess
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_WIDTH = 640
_HEIGHT = 288
_FOV_DEG = 46


class DimSimConnection:
    camera_info_static: CameraInfo = CameraInfo.from_fov(
        fov_deg=_FOV_DEG,
        width=_WIDTH,
        height=_HEIGHT,
        axis="horizontal",
        frame_id="camera_optical",
    )

    def __init__(self, global_config: GlobalConfig) -> None:
        self._dimsim_process: DimSimProcess = DimSimProcess(global_config)
        self._odom_transport: PubSubTransport[PoseStamped] = make_transport("/odom", PoseStamped)
        self._lidar_transport: PubSubTransport[PointCloud2] = make_transport("/lidar", PointCloud2)
        self._video_transport: PubSubTransport[Image] = make_transport("/color_image", Image)
        self._cmd_vel_transport: PubSubTransport[Twist] = make_transport("/cmd_vel", Twist)
        self._tf_transport: PubSubTransport[TFMessage] = make_transport("/tf", TFMessage)
        self._unsubscribes: list[Callable[[], None]] = []
        self._latest_sensor_ts = {
            "odom": float("-inf"),
            "lidar": float("-inf"),
            "video": float("-inf"),
        }

    def start(self) -> None:
        self._dimsim_process.start()
        for transport in (
            self._odom_transport,
            self._lidar_transport,
            self._video_transport,
            self._cmd_vel_transport,
            self._tf_transport,
        ):
            transport.start()
        self._unsubscribes = [
            self._odom_transport.subscribe(self._handle_odom),
            self._lidar_transport.subscribe(self._handle_lidar),
            self._video_transport.subscribe(self._handle_video),
        ]

    def stop(self) -> None:
        for unsubscribe in self._unsubscribes:
            unsubscribe()
        self._unsubscribes.clear()
        for transport in (
            self._cmd_vel_transport,
            self._video_transport,
            self._lidar_transport,
            self._odom_transport,
            self._tf_transport,
        ):
            transport.stop()
        self._dimsim_process.stop()

    @functools.cache
    def lidar_stream(self) -> Subject[PointCloud2]:
        return Subject()

    @functools.cache
    def odom_stream(self) -> Subject[PoseStamped]:
        return Subject()

    @functools.cache
    def video_stream(self) -> Subject[Image]:
        return Subject()

    @functools.cache
    def lowstate_stream(self) -> Observable[Any]:
        return Subject()

    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        self._cmd_vel_transport.publish(twist)
        return True

    def standup(self) -> bool:
        return True

    def liedown(self) -> bool:
        return True

    def balance_stand(self) -> bool:
        return True

    def sport_command(self, api_id: int) -> bool:
        return True

    def stop_movement(self) -> None:
        # No webrtc deadman timer in sim; the cmd_vel timeout covers it.
        pass

    def set_obstacle_avoidance(self, enabled: bool = True) -> bool:
        return True

    def set_rage_mode(self, enable: bool) -> bool:
        return True

    def set_light(self, level: int) -> bool:
        return True

    def switch_joystick(self, enable: bool = True) -> bool:
        return True

    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any]:
        return {}

    def _handle_odom(self, msg: PoseStamped) -> None:
        if not self._is_new_sensor_sample("odom", msg.ts):
            return
        self._tf_transport.publish(TFMessage(*_odom_to_tf(msg)))
        self.odom_stream().on_next(msg)

    def _handle_lidar(self, msg: PointCloud2) -> None:
        if self._is_new_sensor_sample("lidar", msg.ts):
            self.lidar_stream().on_next(msg)

    def _handle_video(self, msg: Image) -> None:
        if self._is_new_sensor_sample("video", msg.ts):
            self.video_stream().on_next(msg)

    def _is_new_sensor_sample(self, stream: str, timestamp: float) -> bool:
        if timestamp <= self._latest_sensor_ts[stream]:
            return False
        self._latest_sensor_ts[stream] = timestamp
        return True


def _odom_to_tf(odom: PoseStamped) -> list[Transform]:
    """Build transform chain from odometry pose.

    Transform tree: world -> base_link -> {camera_link -> camera_optical, lidar_link}
    """
    camera_link = Transform(
        translation=Vector3(0.3, 0.0, 0.0),  # camera 30cm forward
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id="base_link",
        child_frame_id="camera_link",
        ts=odom.ts,
    )

    camera_optical = Transform(
        translation=Vector3(0.0, 0.0, 0.0),
        rotation=Quaternion(-0.5, 0.5, -0.5, 0.5),
        frame_id="camera_link",
        child_frame_id="camera_optical",
        ts=odom.ts,
    )

    lidar_link = Transform(
        translation=Vector3(0.0, 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id="base_link",
        child_frame_id="lidar_link",
        ts=odom.ts,
    )

    return [
        Transform.from_pose("base_link", odom),
        camera_link,
        camera_optical,
        lidar_link,
    ]
