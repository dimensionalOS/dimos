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

from contextlib import ExitStack
import threading
from typing import Any
import uuid

from reactivex import Observable, Subject

from dimos.core.global_config import GlobalConfig
from dimos.core.transport import LCMTransport, PubSubTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.simulation.dimsim.dimsim_process import DimSimProcess
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_WIDTH = 640
_HEIGHT = 288
_FOV_DEG = 46


class DimSimConnection:
    """Private simulator LCM input; GO2Connection owns all public outputs."""

    world_frame = "world"

    camera_info_static: CameraInfo = CameraInfo.from_fov(
        fov_deg=_FOV_DEG,
        width=_WIDTH,
        height=_HEIGHT,
        axis="horizontal",
        frame_id="camera_optical",
    )

    def __init__(self, global_config: GlobalConfig) -> None:
        # A fresh administratively scoped multicast group isolates each simulator
        # from public LCM topics without a bind/release port-allocation race.
        group = uuid.uuid4().bytes
        self.wire_url = f"udpm://239.{192 + group[0] % 64}.{group[1]}.{group[2]}:{global_config.dimsim_port}?ttl=0"
        self._dimsim_process = DimSimProcess(global_config, lcm_url=self.wire_url)
        self._cmd_transport: PubSubTransport[Twist] = LCMTransport(
            "/cmd_vel", Twist, url=self.wire_url
        )
        self._odom: Subject[PoseStamped] = Subject()
        self._video: Subject[Image] = Subject()
        self._lidar: Subject[PointCloud2] = Subject()
        self._lowstate: Subject[Any] = Subject()
        self._inputs: list[tuple[PubSubTransport[Any], Subject[Any]]] = [
            (LCMTransport("/odom", PoseStamped, url=self.wire_url), self.odom_stream()),
            (LCMTransport("/color_image", Image, url=self.wire_url), self.video_stream()),
            (LCMTransport("/lidar", PointCloud2, url=self.wire_url), self.lidar_stream()),
        ]
        self._resources: ExitStack | None = None
        self._lifecycle_lock = threading.Lock()
        self._command_lock = threading.Lock()
        self._command_ready = False

    def start(self) -> None:
        with self._lifecycle_lock:
            if self._resources is not None:
                return
            with ExitStack() as resources:
                resources.callback(self._cmd_transport.stop)
                self._cmd_transport.start()
                for transport, subject in self._inputs:
                    resources.callback(transport.stop)
                    transport.start()
                    resources.callback(transport.subscribe(subject.on_next))
                # Stop the producer before detaching subscribers or closing LCM.
                resources.callback(self._dimsim_process.stop)
                self._dimsim_process.start()
                self._resources = resources.pop_all()
                with self._command_lock:
                    self._command_ready = True

    def stop(self) -> None:
        with self._lifecycle_lock:
            with self._command_lock:
                self._command_ready = False
            resources, self._resources = self._resources, None
            if resources is not None:
                resources.close()

    def lidar_stream(self) -> Subject[PointCloud2]:
        return self._lidar

    def odom_stream(self) -> Subject[PoseStamped]:
        return self._odom

    def video_stream(self) -> Subject[Image]:
        return self._video

    def lowstate_stream(self) -> Observable[Any]:
        return self._lowstate

    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        with self._command_lock:
            if not self._command_ready:
                return False
            self._cmd_transport.publish(twist)
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
        self.move(Twist())

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

    def odom_to_tf(self, odom: PoseStamped, prefix: str = "") -> list[Transform]:
        transforms = _odom_to_tf(odom)
        if prefix:
            for transform in transforms:
                if transform.frame_id != odom.frame_id:
                    transform.frame_id = f"{prefix}/{transform.frame_id}"
                transform.child_frame_id = f"{prefix}/{transform.child_frame_id}"
        return transforms


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
