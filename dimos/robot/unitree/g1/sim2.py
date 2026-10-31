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

"""G1Sim: the sim twin of G1WholeBodyConnection, with its motor_command in and motor_states and pelvis imu out."""

import asyncio
from collections.abc import AsyncIterator
import time
from typing import Any

from dimos_lcm.world_msgs import WorldAction, WorldChange

from dimos.core.module import Module, ModuleConfig
from dimos.core.shared_config import shared_config
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.robot.unitree.g1.wholebody_connection import G1_JOINT_NAMES
from dimos.sim2.actions import JointCommand, world_action
from dimos.sim2.module import Simulation, SimulationSpec
from dimos.sim2.spec import Servo, SimulatedRobot
from dimos.sim2.view import WorldView, open_view
from dimos.sim2.world_change import WorldMirror
from dimos.simulation.mujoco.menagerie import menagerie_path

# Where it starts: pelvis height, and per joint (Menagerie's order, which is the motors' order) the pose it stands in
# and the PD gains that hold it there until the first motor_command.
HOME_HEIGHT = 0.755
_LEG = (
    ("hip_pitch", -0.312, 75, 2),
    ("hip_roll", 0, 75, 2),
    ("hip_yaw", 0, 75, 2),
    ("knee", 0.669, 75, 2),
    ("ankle_pitch", -0.363, 20, 1),
    ("ankle_roll", 0, 2, 0.2),
)
_WAIST = (("waist_yaw", 0, 75, 2), ("waist_roll", 0, 75, 2), ("waist_pitch", 0.073, 75, 2))
_ARM = (
    ("shoulder_pitch", 0.2, 75, 2),
    ("shoulder_roll", 0.2, 75, 2),
    ("shoulder_yaw", 0, 75, 2),
    ("elbow", 0.6, 75, 2),
    ("wrist_roll", 0, 2, 0.2),
    ("wrist_pitch", 0, 2, 0.2),
    ("wrist_yaw", 0, 2, 0.2),
)
SERVOS = (
    tuple(
        Servo(f"{side}_{name}_joint", kp, kd, initial)
        for side in ("left", "right")
        for name, initial, kp, kd in _LEG
    )
    + tuple(Servo(f"{name}_joint", kp, kd, initial) for name, initial, kp, kd in _WAIST)
    + tuple(
        Servo(
            f"{side}_{name}_joint",
            kp,
            kd,
            -initial if side == "right" and name == "shoulder_roll" else initial,
        )
        for side in ("left", "right")
        for name, initial, kp, kd in _ARM
    )
)


class G1SimConfig(ModuleConfig):
    robot_id: str = "g1"
    xy: tuple[float, float] = (0.0, 0.0)
    frame_id: str = "g1_pelvis"
    # As G1WholeBodyConnection: full damping at once, stiffness and torque fading in over this long.
    soft_start_seconds: float = 3.0
    # As G1WholeBodyConnection's, but per sim second.
    publish_rate_hz: float = 500.0


class G1Sim(Module):
    config: G1SimConfig
    motor_command: In[MotorCommandArray]
    world_change: In[WorldChange]
    actions: Out[WorldAction]
    # Named and ordered as G1WholeBodyConnection's.
    motor_states: Out[JointState]
    imu: Out[Imu]
    _simulation: SimulationSpec
    _view: WorldView | None = None
    _soft_start: float | None = None

    @shared_config(Simulation)
    def simulated(self) -> dict[str, Any]:
        x, y = self.config.xy
        return {
            "simulated": SimulatedRobot(
                self.config.robot_id,
                menagerie_path() / "unitree_g1" / "g1.xml",
                "pelvis",
                xyz=(x, y, HOME_HEIGHT),
                servos=SERVOS,
            )
        }

    async def main(self) -> AsyncIterator[None]:
        self._world = WorldMirror()
        self._world.subscribe(self.world_change)
        self._view = open_view(self._simulation.describe())
        self._publishing = asyncio.create_task(self._publish())
        yield

    async def handle_motor_command(self, command: MotorCommandArray) -> None:
        if command.num_joints != len(SERVOS):
            return
        now = time.monotonic()
        if self._soft_start is None:
            self._soft_start = now
        duration = self.config.soft_start_seconds
        scale = min(1.0, (now - self._soft_start) / duration) if duration > 0 else 1.0
        self.actions.publish(
            world_action(
                self._world,
                JointCommand(
                    self.config.robot_id,
                    tuple(servo.joint for servo in SERVOS),
                    position=tuple(command.q),
                    velocity=tuple(command.dq),
                    effort=tuple(tau * scale for tau in command.tau),
                    kp=tuple(kp * scale for kp in command.kp),
                    kd=tuple(command.kd),
                ),
            )
        )

    async def _publish(self) -> None:
        robot, view, world = self.config.robot_id, self._view, self._world
        assert view is not None
        async for _ in world.ticks(self.config.publish_rate_hz):
            with world.lock:
                names, rows = world.joints(robot)
                stamp = world.stamp()
            if not names:
                continue
            index = {name: i for i, name in enumerate(names)}
            rows = rows[[index[servo.joint] for servo in SERVOS]]
            self.motor_states.publish(
                JointState(
                    ts=stamp,
                    frame_id=self.config.frame_id,
                    name=G1_JOINT_NAMES,
                    position=rows[:, 0].tolist(),
                    velocity=rows[:, 1].tolist(),
                    effort=rows[:, 2].tolist(),
                )
            )
            view.update(world)
            gyro, accelerometer, (w, x, y, z) = view.imu(f"{robot}/imu_in_pelvis")
            self.imu.publish(
                Imu(
                    angular_velocity=Vector3(*gyro),
                    linear_acceleration=Vector3(*accelerometer),
                    orientation=Quaternion(x, y, z, w),
                    frame_id=self.config.frame_id,
                    ts=stamp,
                )
            )
