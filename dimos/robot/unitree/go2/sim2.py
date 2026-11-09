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


import asyncio
from collections.abc import AsyncIterator
import math
from pathlib import Path
from typing import Any

from dimos_lcm.world_msgs import WorldAction, WorldObservations
import numpy as np

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.assets.model import UsdModel
from dimos.robot.unitree.go2.constants import CMD_VEL_TIMEOUT
from dimos.robot.unitree.go2.go2_mid360_static_transforms import (
    CAMERA_XYZ,
    L1_MOUNT,
    L1_XYZ,
    MID360_PITCH_DOWN,
    MID360_XYZ,
    OPTICAL_RPY,
)
from dimos.sim2.actions import JointCommand, world_action
from dimos.sim2.mujoco.sensor_world import rotation_matrix
from dimos.sim2.world_observations import ACT, WorldMirror
from dimos.simulation.go2_legged.policy import Proprioception, load_policy
from dimos.simulation.go2_legged.robot import COMMAND_SLEW, projected_gravity, yaw_of


def go2_drives() -> dict[str, tuple[float, float, float]]:
    policy = load_policy(None)
    return {
        joint: (float(kp), float(kd), float(q))
        for joint, kp, kd, q in zip(
            policy.joint_names, policy.kp, policy.kd, policy.default_pose, strict=True
        )
    }


class Go2SimConfig(ModuleConfig):
    robot_id: str = "go2"
    policy: Path | None = None
    odom_frame_id: str = "world"
    heading_gain: float = 2.0


class Go2Sim(Module):
    usd = UsdModel.from_file(Path(__file__).parent / "assets" / "go2.usda").with_frames(
        {
            "utlidar_lidar": (
                "base",
                L1_XYZ,
                (math.radians(L1_MOUNT[0]), math.radians(L1_MOUNT[1]), math.radians(L1_MOUNT[2])),
            ),
            "front_camera": ("base", CAMERA_XYZ, (0.0, 0.0, 0.0)),
            "mid360_link": ("front_camera", MID360_XYZ, (0.0, MID360_PITCH_DOWN, 0.0)),
            "camera_optical": ("front_camera", (0.0, 0.0, 0.0), OPTICAL_RPY),
        }
    )
    l1_mount_frame = "utlidar_lidar"
    lidar_mount_frame = "mid360_link"
    camera_mount_frame = "camera_optical"

    config: Go2SimConfig
    cmd_vel: In[Twist]
    world_observations: In[WorldObservations]
    actions: Out[WorldAction]
    odom: Out[PoseStamped]
    tf: Out[TFMessage]
    joint_state: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._policy = load_policy(self.config.policy)
        self._desired = np.zeros(3)
        self._desired_expires = 0.0
        self._command = np.zeros(3)
        self._heading: float | None = None

    async def main(self) -> AsyncIterator[None]:
        self._world = WorldMirror()
        loop = asyncio.get_running_loop()

        def on_frame(world: WorldMirror) -> None:
            if world.got(self.config.robot_id, ACT):
                loop.call_soon_threadsafe(self._act)

        self._world.on_frame(on_frame)
        self._world.subscribe(self.world_observations)
        yield

    async def handle_cmd_vel(self, twist: Twist) -> None:
        self._desired = np.array([twist.linear.x, twist.linear.y, twist.angular.z])
        # Sim time, like every other rate here: on wall time a command expires in a quarter of
        # its intended interval at speed=4, and the robot forgets what it was told.
        self._desired_expires = self._world.stamp() + CMD_VEL_TIMEOUT

    def _act(self) -> None:
        policy, world = self._policy, self._world
        with world.lock:
            base = world.entity(self.config.robot_id)
            names, rows = world.joints(self.config.robot_id)
            stamp = world.stamp()
        if base is None or base.pose is None or base.velocity is None:
            return
        x, y, z, w = base.pose[3:]
        quat = np.array([w, x, y, z])
        desired = self._desired if stamp < self._desired_expires else np.zeros(3)
        self._command += np.clip(desired - self._command, -COMMAND_SLEW, COMMAND_SLEW)
        if desired[2] == 0:
            self._heading = yaw_of(quat) if self._heading is None else self._heading
            turn = self.config.heading_gain * math.remainder(self._heading - yaw_of(quat), math.tau)
        else:
            self._heading, turn = None, 0.0
        joints = {name: row for name, row in zip(names, rows, strict=True)}
        observation = Proprioception(
            angular_velocity=rotation_matrix(quat).T @ base.velocity[3:],
            gravity=projected_gravity(quat),
            joint_position=np.array([joints[name][0] for name in policy.joint_names]),
            joint_velocity=np.array([joints[name][1] for name in policy.joint_names]),
        )
        targets = policy.act(observation, self._command + np.array([0.0, 0.0, turn]))
        self.actions.publish(
            world_action(
                world,
                JointCommand(
                    self.config.robot_id,
                    tuple(policy.joint_names),
                    position=tuple(map(float, targets)),
                ),
            )
        )
        self.joint_state.publish(
            JointState(
                ts=stamp,
                name=list(policy.joint_names),
                position=observation.joint_position.tolist(),
                velocity=observation.joint_velocity.tolist(),
            )
        )
        self._publish_pose(base.pose[:3], quat, stamp)

    def _publish_pose(self, position: np.ndarray, quat_wxyz: np.ndarray, stamp: float) -> None:
        w, x, y, z = quat_wxyz
        odom = PoseStamped(
            ts=stamp,
            frame_id=self.config.odom_frame_id,
            position=position.tolist(),
            orientation=[x, y, z, w],
        )
        self.tf.publish(TFMessage(Transform.from_pose("base_link", odom)))
        self.odom.publish(odom)
