# Copyright 2026 Dimensional Inc.
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

"""Adapt one virtual robot to standard DimOS sensor and command streams."""

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from microduck_world.robot_io import (
    Observation,
    RobotCommand,
    RobotState,
    RobotVision,
    allowed_generation,
)
from reactivex.disposable import Disposable


class ConnectionConfig(ModuleConfig):
    generation: str


class SimRobotConnection(Module):
    config: ConnectionConfig
    hardware_state: In[RobotState]
    hardware_vision: In[RobotVision]
    hardware_command: Out[RobotCommand]
    cmd_vel: In[Twist]
    policy_request: In[str]
    respawn_request: In[bool]
    ball_drop_request: In[str]
    odom: Out[PoseStamped]
    joint_state: Out[JointState]
    policy_state: Out[str]
    color_image: Out[Image]
    depth_image: Out[Image]
    camera_info: Out[CameraInfo]
    pointcloud: Out[PointCloud2]
    observation: Out[Observation]
    tf: Out[TFMessage]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.hardware_state.subscribe(self.receive_state)))
        self.register_disposable(Disposable(self.hardware_vision.subscribe(self.receive_vision)))
        self.register_disposable(Disposable(self.cmd_vel.subscribe(self.drive)))
        self.register_disposable(Disposable(self.policy_request.subscribe(self.request_policy)))
        self.register_disposable(Disposable(self.respawn_request.subscribe(self.respawn)))
        self.register_disposable(Disposable(self.ball_drop_request.subscribe(self.drop_ball)))

    def receive_state(self, state: RobotState) -> None:
        if not allowed_generation(state.generation, self.config.generation):
            return
        self.odom.publish(state.odom)
        self.joint_state.publish(state.joints)
        self.policy_state.publish(state.policy)

    def receive_vision(self, vision: RobotVision) -> None:
        if not allowed_generation(vision.generation, self.config.generation):
            return
        self.color_image.publish(vision.image)
        self.depth_image.publish(vision.depth)
        self.camera_info.publish(vision.camera_info)
        self.tf.publish(vision.tf)
        self.observation.publish(
            Observation(vision.image, vision.depth, vision.camera_info, vision.camera_pose)
        )
        self.pointcloud.publish(
            PointCloud2.from_numpy(vision.points, frame_id="world", timestamp=vision.image.ts)
        )

    def drive(self, twist: Twist) -> None:
        self.hardware_command.publish(
            RobotCommand(
                self.config.generation,
                "twist",
                (float(twist.linear.x), float(twist.linear.y), float(twist.angular.z)),
            )
        )

    def request_policy(self, request: str) -> None:
        self.hardware_command.publish(RobotCommand(self.config.generation, "policy", request))

    def drop_ball(self, ball: str) -> None:
        self.hardware_command.publish(RobotCommand(self.config.generation, "drop_ball", ball))

    def respawn(self, requested: bool) -> None:
        if requested is True:
            self.hardware_command.publish(RobotCommand(self.config.generation, "respawn", ""))

    @rpc
    def stop(self) -> None:
        try:
            self.drive(Twist())
        finally:
            super().stop()
