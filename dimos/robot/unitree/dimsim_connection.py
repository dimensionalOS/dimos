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

import functools
from collections.abc import Callable
from typing import Any

from reactivex import Observable, Subject

from dimos.core.global_config import GlobalConfig
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.protocol.tf.tf import LCMTF
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
        self._odom_transport: LCMTransport[PoseStamped] = LCMTransport("/odom", PoseStamped)
        self._unsubscribe_odom: Callable[[], None] | None = None
        self._tf = LCMTF()

    def start(self) -> None:
        self._dimsim_process.start()
        self._odom_transport.start()
        self._unsubscribe_odom = self._odom_transport.subscribe(self._handle_odom)
        self._tf.start()

    def stop(self) -> None:
        self._tf.stop()
        if self._unsubscribe_odom is not None:
            self._unsubscribe_odom()
        self._odom_transport.stop()
        self._dimsim_process.stop()

    @functools.cache
    def lidar_stream(self) -> Observable[PointCloud2]:
        return Subject()

    @functools.cache
    def odom_stream(self) -> Observable[PoseStamped]:
        return Subject()

    @functools.cache
    def video_stream(self) -> Observable[Image]:
        return Subject()

    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        return True

    def standup(self) -> bool:
        return True

    def liedown(self) -> bool:
        return True

    def balance_stand(self) -> bool:
        return True

    def set_obstacle_avoidance(self, enabled: bool = True) -> None:
        pass

    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any]:
        return {}

    def _handle_odom(self, msg: PoseStamped) -> None:
        self._tf.publish(*_odom_to_tf(msg))


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
