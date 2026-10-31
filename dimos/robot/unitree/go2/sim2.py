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

"""A simulated Go2 with GO2Connection's cmd_vel/odom/tf: the walking policy, observing world_change and sending joint commands."""

import asyncio
from collections.abc import AsyncIterator
import math
from pathlib import Path
import time
from typing import Any

from dimos_lcm.world_msgs import WorldAction, WorldChange
import numpy as np

from dimos.core.module import Module, ModuleConfig
from dimos.core.shared_config import shared_config
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.go2.constants import CMD_VEL_TIMEOUT
from dimos.sim2.actions import JointCommand, world_action
from dimos.sim2.module import Simulation
from dimos.sim2.spec import Servo, SimulatedRobot
from dimos.sim2.view import rotation_matrix
from dimos.sim2.world_change import WorldMirror
from dimos.simulation.go2_legged.policy import Proprioception, load_policy
from dimos.simulation.go2_legged.robot import COMMAND_SLEW, CONTROL_DT, projected_gravity, yaw_of
from dimos.utils.data import get_data


class Go2SimConfig(ModuleConfig):
    robot_id: str = "go2"
    xyz: tuple[float, float, float] = (0.0, 0.0, 0.35)
    policy: Path | None = None
    odom_frame_id: str = "world"
    # Yaw rate commanded per radian of heading error while no turn is commanded.
    heading_gain: float = 2.0


class Go2Sim(Module):
    config: Go2SimConfig
    cmd_vel: In[Twist]
    world_change: In[WorldChange]
    actions: Out[WorldAction]
    odom: Out[PoseStamped]
    tf: Out[TFMessage]
    # Named as the Go2's URDF names them, for UrdfManager.
    joint_state: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._policy = load_policy(self.config.policy)
        self._desired = np.zeros(3)
        self._desired_expires = 0.0
        self._command = np.zeros(3)
        self._heading: float | None = None

    @shared_config(Simulation)
    def simulated(self) -> dict[str, Any]:
        config, policy = self.config, self._policy
        servos = zip(policy.joint_names, policy.kp, policy.kd, policy.default_pose, strict=True)
        return {
            "simulated": SimulatedRobot(
                config.robot_id,
                get_data("go2_sim") / "unitree_go2" / "go2.xml",
                "base",
                xyz=config.xyz,
                control_period=CONTROL_DT,
                servos=tuple(
                    Servo(joint, float(kp), float(kd), float(q)) for joint, kp, kd, q in servos
                ),
            )
        }

    async def main(self) -> AsyncIterator[None]:
        self._world = WorldMirror()
        loop = asyncio.get_running_loop()

        # The world holds every CONTROL_DT of sim time until the policy has acted, as in training.
        def on_frame(world: WorldMirror) -> None:
            if world.has_event(self.config.robot_id, "events/act"):
                loop.call_soon_threadsafe(self._act)

        self._world.on_frame(on_frame)
        self._world.subscribe(self.world_change)
        yield

    async def handle_cmd_vel(self, twist: Twist) -> None:
        self._desired = np.array([twist.linear.x, twist.linear.y, twist.angular.z])
        self._desired_expires = time.monotonic() + CMD_VEL_TIMEOUT

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
        desired = self._desired if time.monotonic() < self._desired_expires else np.zeros(3)
        self._command += np.clip(desired - self._command, -COMMAND_SLEW, COMMAND_SLEW)
        # The bundled policy drifts in yaw even standing still; with no turn commanded, hold heading.
        if desired[2] == 0:
            self._heading = yaw_of(quat) if self._heading is None else self._heading
            turn = self.config.heading_gain * math.remainder(self._heading - yaw_of(quat), math.tau)
        else:
            self._heading, turn = None, 0.0
        joints = {name: row for name, row in zip(names, rows, strict=True)}
        observation = Proprioception(
            # The world's angular velocity is world-aligned; the policy's gyro reads in the base's frame.
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
        self.tf.publish(TFMessage(*GO2Connection._odom_to_tf(odom)))
        self.odom.publish(odom)
