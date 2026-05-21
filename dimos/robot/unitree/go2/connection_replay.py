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

"""Go2 connection that replays sensor streams from a recorded dataset.

Self-contained: no transport (datasets are read in-process), stream wiring,
and stubbed RPC surface in one Module class. No shared base.
"""

from __future__ import annotations

from threading import Thread
import time
from typing import Any

from reactivex.disposable import Disposable
from reactivex.observable import Observable
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
from dimos.robot.unitree.go2.camera import _camera_info_static
from dimos.robot.unitree.go2.config import ConnectionConfig
from dimos.spec.perception import Camera, Pointcloud
from dimos.utils.decorators.decorators import simple_mcache
from dimos.utils.logging_config import setup_logger
from dimos.utils.testing.replay import TimedSensorReplay

logger = setup_logger()


@connection(robot="go2", backend="replay")
class Go2ReplayConnection(Module, Camera, Pointcloud):
    """Go2 connection that replays a previously recorded dataset."""

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
        self.dataset = self.config.g.replay_db
        self.replay_config: dict[str, Any] = {
            "loop": True,
            "seek": None,
            "duration": None,
        }

    @rpc
    def start(self) -> None:
        super().start()

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

    @rpc
    def stop(self) -> None:
        if self._camera_info_thread and self._camera_info_thread.is_alive():
            self._camera_info_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()

    @simple_mcache
    def _lidar_stream(self) -> Observable[PointCloud2]:
        store: TimedSensorReplay[PointCloud2] = TimedSensorReplay(f"{self.dataset}/lidar")
        return store.stream(**self.replay_config)

    @simple_mcache
    def _odom_stream(self) -> Observable[PoseStamped]:
        store: TimedSensorReplay[PoseStamped] = TimedSensorReplay(f"{self.dataset}/odom")
        return store.stream(**self.replay_config)

    @simple_mcache
    def _video_stream(self) -> Observable[Image]:
        store: TimedSensorReplay[Image] = TimedSensorReplay(f"{self.dataset}/color_image")
        return store.stream(**self.replay_config)

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> bool:
        return True

    @rpc
    def standup(self) -> bool:
        return True

    @rpc
    def liedown(self) -> bool:
        return True

    @rpc
    def balance_stand(self) -> bool:
        return True

    @rpc
    def enable_rage_mode(self) -> bool:
        return True

    def set_obstacle_avoidance(self, enabled: bool = True) -> None:
        pass

    @rpc
    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any]:
        return {"status": "ok", "message": "Fake publish"}

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
