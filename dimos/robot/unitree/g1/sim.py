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


import threading
from threading import Thread
import time
from typing import Any

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.g1.connection import G1ConnectionBase
from dimos.robot.unitree.mujoco_connection import MujocoConnection
from dimos.robot.unitree.type.odometry import Odometry as SimOdometry
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class G1SimConfig(ModuleConfig):
    ip: str = Field(default_factory=lambda m: m["g"].robot_ip)


class G1SimConnection(G1ConnectionBase[G1SimConfig]):
    default_config = G1SimConfig

    cmd_vel: In[Twist]
    lidar: Out[PointCloud2]
    odom: Out[PoseStamped]
    color_image: Out[Image]
    camera_info: Out[CameraInfo]
    connection: MujocoConnection | None = None
    _camera_info_thread: Thread | None = None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop_event = threading.Event()

    @rpc
    def start(self) -> None:
        super().start()

        from dimos.robot.unitree.mujoco_connection import MujocoConnection

        self.connection = MujocoConnection(self.config.g)
        assert self.connection is not None
        self.connection.start()

        self._disposables.add(Disposable(self.cmd_vel.subscribe(self.move)))
        self._disposables.add(self.connection.odom_stream().subscribe(self._publish_sim_odom))
        self._disposables.add(self.connection.lidar_stream().subscribe(self.lidar.publish))
        self._disposables.add(self.connection.video_stream().subscribe(self.color_image.publish))

        self._camera_info_thread = Thread(
            target=self._publish_camera_info_loop,
            daemon=True,
        )
        self._camera_info_thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        assert self.connection is not None
        self.connection.stop()
        if self._camera_info_thread and self._camera_info_thread.is_alive():
            self._camera_info_thread.join(timeout=1.0)
        super().stop()

    def _publish_camera_info_loop(self) -> None:
        assert self.connection is not None
        info = self.connection.camera_info_static
        while not self._stop_event.is_set():
            self.camera_info.publish(info)
            self._stop_event.wait(1.0)

    def _publish_tf(self, msg: PoseStamped) -> None:
        self.odom.publish(msg)

        self.tf.publish(Transform.from_pose("base_link", msg))

        # Publish camera_link and camera_optical transforms
        camera_link = Transform(
            translation=Vector3(0.05, 0.0, 0.6),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="base_link",
            child_frame_id="camera_link",
            ts=time.time(),
        )

        camera_optical = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion(-0.5, 0.5, -0.5, 0.5),
            frame_id="camera_link",
            child_frame_id="camera_optical",
            ts=time.time(),
        )

        map_to_world = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="map",
            child_frame_id="world",
            ts=time.time(),
        )

        self.tf.publish(camera_link, camera_optical, map_to_world)

    def _publish_sim_odom(self, msg: SimOdometry) -> None:
        self._publish_tf(
            PoseStamped(
                ts=msg.ts,
                frame_id=msg.frame_id,
                position=msg.position,
                orientation=msg.orientation,
            )
        )

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> None:
        assert self.connection is not None
        self.connection.move(twist, duration)

    @rpc
    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any]:
        logger.info(f"Publishing request to topic: {topic} with data: {data}")
        assert self.connection is not None
        return self.connection.publish_request(topic, data)
