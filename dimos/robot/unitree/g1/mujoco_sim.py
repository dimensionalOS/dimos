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
from typing import Any

from pydantic import Field
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import ModuleConfig
from dimos.core.stream import In, Out
from dimos.message_utils import geometry, time as message_time
from dimos.messages.geometry_msgs.msg import (
    PoseStamped,
    Quaternion,
    Transform,
    TransformStamped,
    Twist,
    Vector3,
)
from dimos.messages.sensor_msgs.msg import CameraInfo, Image, PointCloud2
from dimos.messages.tf2_msgs.msg import TFMessage
from dimos.robot.unitree.g1.connection import G1ConnectionBase
from dimos.robot.unitree.mujoco_connection import MujocoConnection
from dimos.simulation.mujoco.config import MujocoConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class G1SimConfig(ModuleConfig):
    mujoco: MujocoConfig = Field(default_factory=MujocoConfig)
    ip: str = Field(default_factory=lambda m: m["g"].robot_ip)


class G1SimConnection(G1ConnectionBase):
    config: G1SimConfig
    cmd_vel: In[Twist]
    lidar: Out[PointCloud2]
    odom: Out[PoseStamped]
    color_image: Out[Image]
    camera_info: Out[CameraInfo]
    tf: Out[TFMessage]
    connection: MujocoConnection | None = None
    _camera_info_thread: Thread | None = None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._stop_event = threading.Event()
        self.connection = MujocoConnection(
            self.config.mujoco, robot_model=self.config.g.robot_model
        )

    @rpc
    def start(self) -> None:
        super().start()
        assert self.connection is not None
        self.connection.start()

        self.register_disposable(Disposable(self.cmd_vel.subscribe(self.move)))
        self.register_disposable(self.connection.odom_stream().subscribe(self._publish_sim_odom))
        self.register_disposable(self.connection.lidar_stream().subscribe(self.lidar.publish))
        self.register_disposable(self.connection.video_stream().subscribe(self.color_image.publish))

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
            self._camera_info_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()

    def _publish_camera_info_loop(self) -> None:
        assert self.connection is not None
        info = self.connection.camera_info_static
        while not self._stop_event.is_set():
            self.camera_info.publish(info)
            self._stop_event.wait(1.0)

    def _publish_tf(self, msg: PoseStamped) -> None:
        self.odom.publish(msg)
        base_link = geometry.transform_from_pose(msg, child_frame_id="base_link")
        camera_link = TransformStamped(
            message_time.header_now("base_link"),
            "camera_link",
            Transform(Vector3(0.05, 0.0, 0.6), Quaternion(0.0, 0.0, 0.0, 1.0)),
        )
        camera_optical = TransformStamped(
            message_time.header_now("camera_link"),
            "camera_optical",
            Transform(Vector3(0.0, 0.0, 0.0), Quaternion(-0.5, 0.5, -0.5, 0.5)),
        )
        map_to_world = TransformStamped(
            message_time.header_now("map"),
            "world",
            Transform(Vector3(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        )
        self.tf.publish(TFMessage([base_link, camera_link, camera_optical, map_to_world]))

    def _publish_sim_odom(self, msg: PoseStamped) -> None:
        self._publish_tf(msg)

    @rpc
    def move(self, twist: Twist, duration: float = 0.0) -> None:
        assert self.connection is not None
        self.connection.move(twist, duration)

    @rpc
    def publish_request(self, topic: str, data: dict[str, Any]) -> dict[Any, Any]:
        logger.info(f"Publishing request to topic: {topic} with data: {data}")
        assert self.connection is not None
        return self.connection.publish_request(topic, data)
