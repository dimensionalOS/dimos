# Copyright 2025 Dimensional Inc.
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


import time
from typing import TYPE_CHECKING, Any

from reactivex.disposable import Disposable

from dimos.core import In, Module, Out, rpc
from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs import (
    PoseStamped,
    Quaternion,
    Transform,
    Twist,
    Vector3,
)
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.odometry import Odometry as SimOdometry
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.robot.unitree_webrtc.mujoco_connection import MujocoConnection

logger = setup_logger()


class G1SimConnection(Module):
    cmd_vel: In[Twist]
    lidar: Out[LidarMessage]
    odom: Out[PoseStamped]
    ip: str | None
    _global_config: GlobalConfig

    def __init__(
        self,
        ip: str | None = None,
        global_config: GlobalConfig | None = None,
        *args: Any,
        **kwargs: Any,
    ) -> None:
        self._global_config = global_config or GlobalConfig()
        self.ip = ip if ip is not None else self._global_config.robot_ip
        self.connection: MujocoConnection | None = None
        super().__init__(*args, **kwargs)

    @rpc
    def start(self) -> None:
        super().start()

        from dimos.robot.unitree_webrtc.mujoco_connection import MujocoConnection

        self.connection = MujocoConnection(self._global_config)
        assert self.connection is not None
        self.connection.start()

        self._disposables.add(Disposable(self.cmd_vel.subscribe(self.move)))
        self._disposables.add(self.connection.odom_stream().subscribe(self._publish_sim_odom))
        self._disposables.add(self.connection.lidar_stream().subscribe(self.lidar.publish))

    @rpc
    def stop(self) -> None:
        assert self.connection is not None
        self.connection.stop()
        super().stop()

    def _publish_tf(self, msg: PoseStamped) -> None:
        self.odom.publish(msg)

        self.tf.publish(Transform.from_pose("base_link", msg))

        # Publish camera_link transform
        camera_link = Transform(
            translation=Vector3(0.3, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="base_link",
            child_frame_id="camera_link",
            ts=time.time(),
        )

        map_to_world = Transform(
            translation=Vector3(0.0, 0.0, 0.0),
            rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
            frame_id="map",
            child_frame_id="world",
            ts=time.time(),
        )

        self.tf.publish(camera_link, map_to_world)

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


g1_sim_connection = G1SimConnection.blueprint


__all__ = ["G1SimConnection", "g1_sim_connection"]
