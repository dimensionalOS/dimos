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
import threading
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
# The bridge publishes odom at 50 Hz from the moment it reports ready, so the
# first message is a quick end-to-end check that the sim's bus reaches us.
_FIRST_ODOM_TIMEOUT = 30.0


class DimSimConnection:
    camera_info_static: CameraInfo = CameraInfo.from_fov(
        fov_deg=_FOV_DEG,
        width=_WIDTH,
        height=_HEIGHT,
        axis="horizontal",
        frame_id="camera_optical",
    )

    def __init__(self, global_config: GlobalConfig) -> None:
        self._global_config = global_config
        self._dimsim_process: DimSimProcess = DimSimProcess(global_config)
        self._odom_transport: PubSubTransport[PoseStamped] = make_transport("/odom", PoseStamped)
        self._tf_transport: PubSubTransport[TFMessage] = make_transport("/tf", TFMessage)
        self._unsubscribe_odom: Callable[[], None] | None = None
        self._first_odom = threading.Event()

    def start(self) -> None:
        # Subscribe before the sim comes up so its first odom can't be missed.
        self._first_odom.clear()
        self._odom_transport.start()
        self._unsubscribe_odom = self._odom_transport.subscribe(self._handle_odom)
        try:
            self._dimsim_process.start()
            if self._global_config.dimsim_headless and not self._first_odom.wait(
                _FIRST_ODOM_TIMEOUT
            ):
                raise TimeoutError(
                    f"DimSim reported ready but no /odom arrived within {_FIRST_ODOM_TIMEOUT:.0f} s. "
                    "DimSim publishes over LCM only; the transport is "
                    f"{self._global_config.transport!r}."
                )
        except BaseException:
            self.stop()
            raise

    def stop(self) -> None:
        if self._unsubscribe_odom is not None:
            self._unsubscribe_odom()
            self._unsubscribe_odom = None
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

    @functools.cache
    def lowstate_stream(self) -> Observable[Any]:
        return Subject()

    def move(self, twist: Twist, duration: float = 0.0) -> bool:
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
        self._first_odom.set()
        self._tf_transport.publish(TFMessage(*_odom_to_tf(msg)))


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
