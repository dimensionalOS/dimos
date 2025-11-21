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

from reactivex.disposable import Disposable

from dimos import spec
from dimos.core import DimosCluster, In, Module, Out, rpc
from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs import (
    PoseStamped,
    Quaternion,
    Transform,
    Twist,
    Vector3,
)
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.robot.unitree.connection.connection import UnitreeWebRTCConnection
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage
from dimos.robot.unitree_webrtc.type.odometry import Odometry as SimOdometry
from dimos.utils.logging_config import setup_logger

logger = setup_logger(__file__)


class G1Connection(Module):
    cmd_vel: In[Twist] = None  # type: ignore
    lidar: Out[LidarMessage] = None  # type: ignore
    odom: Out[PoseStamped] = None  # type: ignore
    ip: str
    connection_type: str | None = None
    _global_config: GlobalConfig

    def __init__(
        self,
        ip: str | None = None,
        connection_type: str | None = None,
        global_config: GlobalConfig | None = None,
        *args,
        **kwargs,
    ) -> None:
        self._global_config = global_config or GlobalConfig()
        self.ip = ip if ip is not None else self._global_config.robot_ip
        self.connection_type = connection_type or self._global_config.unitree_connection_type
        self.connection = None
        Module.__init__(self, *args, **kwargs)

    @rpc
    def start(self) -> None:
        super().start()

        match self.connection_type:
            case "webrtc":
                self.connection = UnitreeWebRTCConnection(self.ip)
            case "replay":
                raise ValueError("Replay connection not implemented for G1 robot")
            case "mujoco":
                from dimos.robot.unitree_webrtc.mujoco_connection import MujocoConnection

                self.connection = MujocoConnection(self._global_config)
            case _:
                raise ValueError(f"Unknown connection type: {self.connection_type}")

        self.connection.start()

        self._disposables.add(Disposable(self.cmd_vel.subscribe(self.move)))

        if self.connection_type == "mujoco":
            unsub = self.connection.odom_stream().subscribe(self._publish_sim_odom)
            self._disposables.add(unsub)

            unsub = self.connection.lidar_stream().subscribe(self.lidar.publish)
            self._disposables.add(unsub)

    @rpc
    def stop(self) -> None:
        self.connection.stop()
        super().stop()

    def _publish_tf(self, msg: PoseStamped) -> None:
        if self.odom.transport:
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

    def _publish_odom(self, msg: Odometry) -> None:
        self._publish_tf(
            PoseStamped(
                ts=msg.ts,
                frame_id=msg.frame_id,
                position=msg.pose.pose.position,
                orientation=msg.pose.orientation,
            )
        )

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
        self.connection.move(twist, duration)

    @rpc
    def publish_request(self, topic: str, data: dict):
        logger.info(f"Publishing request to topic: {topic} with data: {data}")
        return self.connection.publish_request(topic, data)


g1_connection = G1Connection.blueprint


def deploy(dimos: DimosCluster, ip: str, local_planner: spec.LocalPlanner) -> G1Connection:
    connection = dimos.deploy(G1Connection, ip)
    connection.cmd_vel.connect(local_planner.cmd_vel)
    connection.start()
    return connection


__all__ = ["G1Connection", "deploy", "g1_connection"]
