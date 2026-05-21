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

"""Go2 connection backed by the DimSim simulator.

A self-contained Module (no shared base, mirroring `Go2ReplayConnection`)
that owns a `DimSimConnection`. DimSim publishes odom/TF over its own LCM
transport, so the wrapped connection's `*_stream()` observables are inert
Subjects; subscribing to them is harmless and keeps the start/stop wiring
identical to the other Go2 backends so `Blueprint.with_backend` can swap
between them.
"""

from __future__ import annotations

from threading import Thread
import time
from typing import Any

from reactivex.disposable import Disposable
import rerun.blueprint as rrb

from dimos.agents.annotation import skill
from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.connection_registry import connection
from dimos.robot.tf_utils import odom_to_tf
from dimos.robot.unitree.dimsim_connection import DimSimConnection
from dimos.robot.unitree.go2.config import ConnectionConfig
from dimos.spec.perception import Camera, Pointcloud
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@connection(robot="go2", backend="dimsim")
class Go2DimSimConnection(Module, Camera, Pointcloud):
    """Go2 connection driven by the DimSim simulator."""

    config: ConnectionConfig
    cmd_vel: In[Twist]
    pointcloud: Out[PointCloud2]
    odom: Out[PoseStamped]
    lidar: Out[PointCloud2]
    color_image: Out[Image]
    camera_info: Out[CameraInfo]

    camera_info_static: CameraInfo = DimSimConnection.camera_info_static
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
        self._connection = DimSimConnection(self.config.g)

    @rpc
    def start(self) -> None:
        super().start()
        self._connection.start()

        def onimage(image: Image) -> None:
            self.color_image.publish(image)
            self._latest_video_frame = image

        self.register_disposable(self._connection.lidar_stream().subscribe(self.lidar.publish))
        self.register_disposable(self._connection.odom_stream().subscribe(self._publish_tf))
        self.register_disposable(self._connection.video_stream().subscribe(onimage))
        self.register_disposable(Disposable(self.cmd_vel.subscribe(self.move)))

        self._camera_info_thread = Thread(
            target=self._publish_camera_info,
            daemon=True,
        )
        self._camera_info_thread.start()

    @rpc
    def stop(self) -> None:
        if self._camera_info_thread and self._camera_info_thread.is_alive():
            self._camera_info_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self._connection.stop()
        super().stop()

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        return self._connection.move(twist, duration)

    @rpc
    def standup(self) -> bool:
        return self._connection.standup()

    @rpc
    def liedown(self) -> bool:
        return self._connection.liedown()

    @rpc
    def balance_stand(self) -> bool:
        return self._connection.balance_stand()

    @rpc
    def enable_rage_mode(self) -> bool:
        return self._connection.enable_rage_mode()

    def set_obstacle_avoidance(self, enabled: bool = True) -> None:
        self._connection.set_obstacle_avoidance(enabled)

    @rpc
    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any]:
        return self._connection.publish_request(topic, data)

    def _publish_tf(self, msg: PoseStamped) -> None:
        self.tf.publish(*odom_to_tf(msg))
        self.odom.publish(msg)

    def _publish_camera_info(self) -> None:
        while True:
            self.camera_info.publish(self.camera_info_static)
            time.sleep(1.0)

    @skill
    def observe(self) -> Image | None:
        """Returns the latest video frame from the robot camera."""
        return self._latest_video_frame
