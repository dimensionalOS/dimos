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

"""Simulation-only R1 Pro servo and three-wheel steering controller.

Consumes ordinary encoder measurements and emits sim2's atomic motor actions.
Base commands are body-frame vx/vy/yaw-rate; no root-pose teleportation is used.
"""

import math
import threading
import time
from typing import Any

from dimos_lcm.common_msgs import SampleHeader
from dimos_lcm.control_msgs import ActionCommand, JointCommand
import numpy as np

from dimos.core.core import rpc
from dimos.core.module import ModuleConfig
from dimos.core.shared_config import ConfigSenders, SharedConfigModule
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.robot.galaxea.r1pro.sim.config import (
    CONTROL,
    HOME,
    JOINTS,
    KD,
    KP,
    SERVO_JOINTS,
    UPPER_JOINTS,
)
from dimos.robot.galaxea.r1pro.sim.model import (
    FINGER_JOINTS,
    FINGER_SPEED_M_S,
    FINGER_TRAVEL_M,
    STEER_JOINTS,
    WHEEL_RADIUS,
)


def steer_targets(
    vx: float, vy: float, wz: float, x: float, y: float, current: float
) -> tuple[float, float]:
    """Choose the shortest legal steering move, reversing wheel direction if needed."""
    wheel_x, wheel_y = vx - wz * y, vy + wz * x
    speed = math.hypot(wheel_x, wheel_y)
    if speed < 1e-6:
        return current, 0.0
    angle = math.atan2(wheel_y, wheel_x)
    reverse = (angle + 2 * math.pi) % (2 * math.pi) - math.pi
    target, sign = min(((angle, 1.0), (reverse, -1.0)), key=lambda row: abs(row[0] - current))
    # Do not drive sideways while steering into alignment.
    velocity = sign * speed / WHEEL_RADIUS * max(0.0, math.cos(target - current))
    return target, velocity


class R1ProSimControlConfig(ModuleConfig):
    upper_command_joints: tuple[str, ...] = UPPER_JOINTS


class R1ProSimControl(SharedConfigModule):
    config: R1ProSimControlConfig
    joint_state: In[JointState]
    cmd_vel: In[Twist]
    upper_command: In[JointState]
    left_gripper_command: In[Float32]
    right_gripper_command: In[Float32]
    actions: Out[ActionCommand]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.RLock()
        self._clock = ""
        self._last_ns = -1
        self._last_action: ActionCommand | None = None
        self._velocity = np.zeros(3)
        self._ramped_velocity = np.zeros(3)
        self._until = 0.0
        self._targets = np.array((*HOME, 0.0, 0.0, 0.0, 0.0))
        self._ramped_targets = self._targets.copy()
        self._target_rates = np.array((0.5,) * len(UPPER_JOINTS) + (FINGER_SPEED_M_S,) * 4)
        self._limits: dict[str, tuple[float, float]] = {}
        self._wheel_positions: list[tuple[float, float]] = []
        self._steer_limits: dict[str, tuple[float, float]] = {}

    def config_receive(self, config_senders: ConfigSenders) -> None:
        clocks = [
            config["sim_clock_id"] for _, config in config_senders if "sim_clock_id" in config
        ]
        if len(clocks) != 1:
            raise ValueError("R1 Pro controller requires one simulation clock")
        self._clock = clocks[0]
        joints = {
            joint.name: joint
            for _, config in config_senders
            for model in config.get("robot_models", {}).values()
            for joint in model.joints
        }
        for name in SERVO_JOINTS:
            joint = joints[name]
            assert joint.lower is not None and joint.upper is not None
            self._limits[name] = (joint.lower, joint.upper)
        self._wheel_positions = [tuple(joints[name].origin_xyz[:2]) for name in STEER_JOINTS]
        for name in STEER_JOINTS:
            joint = joints[name]
            assert joint.lower is not None and joint.upper is not None
            self._steer_limits[name] = (joint.lower, joint.upper)

    @rpc
    def move(self, twist: Twist, duration: float = 0.3) -> None:
        values = (twist.linear.x, twist.linear.y, twist.angular.z)
        if not np.isfinite((*values, duration)).all() or not 0 < duration <= 10:
            raise ValueError("finite velocity and duration in (0, 10] seconds required")
        with self._lock:
            self._velocity = np.clip(values, [-0.4, -0.4, -0.5], [0.4, 0.4, 0.5])
            self._until = time.monotonic() + duration

    @rpc
    def set_joint(self, name: str, position: float) -> None:
        if (
            name not in self._limits
            or not self._limits[name][0] <= position <= self._limits[name][1]
        ):
            raise ValueError("upper-body joint target outside model limits")
        with self._lock:
            self._targets[SERVO_JOINTS.index(name)] = position

    @rpc
    def home(self) -> None:
        with self._lock:
            self._targets[: len(UPPER_JOINTS)] = HOME
            self._velocity[:] = 0
            self._until = 0

    async def handle_cmd_vel(self, command: Twist) -> None:
        self.move(command)

    def _set_gripper(self, side: str, opening: float) -> None:
        if not math.isfinite(opening) or not 0 <= opening <= 1:
            raise ValueError("gripper opening must be in [0, 1]")
        with self._lock:
            for finger, sign in ((1, 1), (2, -1)):
                name = f"r1pro/{side}_gripper_finger_joint{finger}"
                self._targets[SERVO_JOINTS.index(name)] = sign * opening * FINGER_TRAVEL_M

    async def handle_left_gripper_command(self, command: Float32) -> None:
        self._set_gripper("left", command.data)

    async def handle_right_gripper_command(self, command: Float32) -> None:
        self._set_gripper("right", command.data)

    async def handle_upper_command(self, command: JointState) -> None:
        if len(command.name) != len(command.position):
            raise ValueError("joint names and positions must match")
        with self._lock:
            for name, position in zip(command.name, command.position, strict=True):
                if (
                    name not in self._limits
                    or not self._limits[name][0] <= position <= self._limits[name][1]
                ):
                    raise ValueError("upper-body joint target outside model limits")
            for name, position in zip(command.name, command.position, strict=True):
                # The arm coordinator also emits passive torso holds. The demo
                # keeps torso ownership here so re-clutching cannot accumulate sag.
                if name in self.config.upper_command_joints and name not in FINGER_JOINTS:
                    self._targets[SERVO_JOINTS.index(name)] = position

    def compute(self, sample: JointState) -> ActionCommand | None:
        period = CONTROL.period_ns
        now = round(sample.ts * 1e9 / period) * period
        if abs(sample.ts * 1e9 - now) > 2 + math.ulp(sample.ts) * 1e9:
            return None
        with self._lock:
            if not self._clock or len(self._wheel_positions) != 3:
                raise RuntimeError("controller has not received its model and clock")
            if now < self._last_ns:
                return None
            if now == self._last_ns:
                return self._last_action
            measured = dict(zip(sample.name, sample.position, strict=True))
            dt = period / 1e9
            desired = self._velocity if time.monotonic() < self._until else np.zeros(3)
            self._ramped_velocity += np.clip(desired - self._ramped_velocity, -0.4 * dt, 0.4 * dt)
            self._ramped_targets += np.clip(
                self._targets - self._ramped_targets,
                -self._target_rates * dt,
                self._target_rates * dt,
            )
            steering, wheels = [], []
            for name, (x, y) in zip(STEER_JOINTS, self._wheel_positions, strict=True):
                target, velocity = steer_targets(
                    float(self._ramped_velocity[0]),
                    float(self._ramped_velocity[1]),
                    float(self._ramped_velocity[2]),
                    x,
                    y,
                    measured[name],
                )
                # Soft physical stops can allow a measured angle just outside
                # the legal command range, especially while holding still.
                lower, upper = self._steer_limits[name]
                steering.append(min(upper, max(lower, target)))
                wheels.append(velocity)
            positions = [*self._ramped_targets, *steering, 0.0, 0.0, 0.0]
            velocities = [0.0] * (len(JOINTS) - len(wheels)) + wheels
            rows = [
                ("position", positions),
                ("velocity", velocities),
                ("kp", KP),
                ("kd", KD),
                ("effort", [0.0] * len(JOINTS)),
            ]
            action = ActionCommand(
                header=SampleHeader(
                    clock_id=self._clock,
                    epoch=0,
                    time_ns=now,
                    source_id=CONTROL.controller_id,
                    sequence=now // period,
                ),
                based_on_time_ns=now,
                apply_at_ns=now,
                valid_until_ns=now + period,
                joint_commands_length=len(rows),
                joint_commands=[
                    JointCommand(
                        joint_names_length=len(JOINTS),
                        joint_names=list(JOINTS),
                        interface_name=interface,
                        values_length=len(values),
                        values=list(values),
                    )
                    for interface, values in rows
                ],
            )
            self._last_ns, self._last_action = now, action
            return action

    async def handle_joint_state(self, sample: JointState) -> None:
        action = self.compute(sample)
        if action is not None:
            self.actions.publish(action)
