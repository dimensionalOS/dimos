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

"""Hardware command binding for the R1 spatial operator (no simulation gains)."""

import math
import threading
import time
from typing import Any

import numpy as np
from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.msgs.std_msgs.Float32 import Float32
from dimos.robot.galaxea.r1pro.config import R1PRO_MODEL
from dimos.robot.galaxea.r1pro.connection import R1PRO_WHOLE_BODY_JOINTS
from dimos.teleop.webxr.controller_types import Buttons


def height_step(q: np.ndarray, axis: float, dt: float, limits: np.ndarray) -> np.ndarray:
    """Move the equal-link torso vertically, preserving reach, pitch and yaw."""
    if not math.isfinite(axis) or not -1 <= axis <= 1 or not axis:
        return q.copy()
    length = 0.35
    x = length * (math.sin(q[0]) + math.sin(q[0] + q[1]))
    z = length * (math.cos(q[0]) + math.cos(q[0] + q[1]))
    if (axis < 0 and z <= 0.49) or (axis > 0 and z >= 0.7):
        return q.copy()
    target_z = z + axis * 0.025 * dt
    reach = (x * x + target_z * target_z - 2 * length * length) / (2 * length * length)
    elbow = (-1 if q[1] <= 0 else 1) * math.acos(float(np.clip(reach, -1, 1)))
    hip = math.atan2(x, target_z) - math.atan2(math.sin(elbow), 1 + math.cos(elbow))
    pitch = q[0] + q[1] - q[2]
    target = np.array([hip, elbow, hip + elbow - pitch, q[3]])
    if np.any(target < limits[:, 0]) or np.any(target > limits[:, 1]):
        return q.copy()
    delta = target - q
    scale = min(1.0, 0.15 * dt / max(float(np.max(np.abs(delta))), 1e-9))
    return np.asarray(q + scale * delta)


class R1SpatialControlConfig(ModuleConfig):
    max_joint_speed: float = Field(default=0.25, gt=0, le=0.5)
    input_timeout_s: float = Field(default=0.3, gt=0, le=0.5)


class R1SpatialControl(Module):
    """Own torso and grippers; forward clutched IK arms through the vendor tracker."""

    config: R1SpatialControlConfig
    dedicated_worker = True
    motor_states: In[JointState]
    upper_command: In[JointState]
    teleop_buttons: In[Buttons]
    torso_height_command: In[Float32]
    left_gripper_command: In[Float32]
    right_gripper_command: In[Float32]
    cmd_vel: In[Twist]
    motor_command: Out[MotorCommandArray]
    hardware_cmd_vel: Out[Twist]
    joint_state: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._measured: np.ndarray | None = None
        self._command: np.ndarray | None = None
        self._target: np.ndarray | None = None
        self._feedback_at = 0.0
        self._buttons_at = 0.0
        self._upper_at = [0.0, 0.0]
        self._grips = (False, False)
        self._torso = 0.0
        self._torso_at = 0.0
        self._drive = Twist.zero()
        self._drive_at = 0.0
        self._owned = False
        self._limits = np.empty((18, 2))

    @rpc
    def build(self) -> None:
        super().build()
        joints = {j.name: j for j in R1PRO_MODEL.load().joints}
        self._limits = np.array(
            [(joints[n].lower, joints[n].upper) for n in R1PRO_WHOLE_BODY_JOINTS[:18]], dtype=float
        )

    @rpc
    def start(self) -> None:
        super().start()
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
            self._thread = None
        self.hardware_cmd_vel.publish(Twist.zero())
        super().stop()

    @rpc
    def set_joint(self, name: str, position: float) -> None:
        raise ValueError("Hardware joint jogging is not exposed by this teleop stack")

    @rpc
    def home(self) -> None:
        raise ValueError("Hardware homing requires a separately planned trajectory")

    async def handle_motor_states(self, message: JointState) -> None:
        if len(message.name) != len(message.position):
            return
        values = dict(zip(message.name, message.position, strict=True))
        if not set(R1PRO_WHOLE_BODY_JOINTS) <= values.keys():
            return
        q = np.array([values[n] for n in R1PRO_WHOLE_BODY_JOINTS])
        if not np.isfinite(q).all() or abs(time.time() - message.ts) > 0.5:
            return
        with self._lock:
            if (
                self._feedback_at
                and time.monotonic() - self._feedback_at > self.config.input_timeout_s
            ):
                # Feedback recovery never resumes an old target. Require new input.
                self._command = q.copy()
                self._target = q.copy()
                self._owned = False
                self._grips = (False, False)
                self._torso = 0.0
                self._drive = Twist.zero()
            self._measured = q
            self._feedback_at = time.monotonic()
            if self._command is None or not self._owned:
                self._command = q.copy()
                self._target = q.copy()
        self.joint_state.publish(message)

    async def handle_teleop_buttons(self, buttons: Buttons) -> None:
        with self._lock:
            grips = (buttons.left_grip, buttons.right_grip)
            if (
                self._measured is not None
                and self._target is not None
                and self._command is not None
            ):
                for i, grip in enumerate(grips):
                    if grip != self._grips[i]:
                        sl = slice(4 + 7 * i, 11 + 7 * i)
                        self._target[sl] = self._command[sl] = self._measured[sl]
                self._owned |= any(grips)
            self._grips, self._buttons_at = grips, time.monotonic()

    async def handle_upper_command(self, message: JointState) -> None:
        with self._lock:
            if (
                self._target is None
                or time.monotonic() - self._buttons_at > self.config.input_timeout_s
            ):
                return
            for name, value in zip(message.name, message.position, strict=True):
                if name not in R1PRO_WHOLE_BODY_JOINTS[4:18]:
                    continue
                index = R1PRO_WHOLE_BODY_JOINTS.index(name)
                if (
                    self._grips[(index - 4) // 7]
                    and math.isfinite(value)
                    and self._limits[index, 0] <= value <= self._limits[index, 1]
                ):
                    self._target[index] = value
                    self._upper_at[(index - 4) // 7] = time.monotonic()

    async def handle_torso_height_command(self, message: Float32) -> None:
        if not math.isfinite(message.data) or not -1 <= message.data <= 1:
            return
        with self._lock:
            if (
                message.data
                and not self._torso
                and self._measured is not None
                and self._command is not None
            ):
                self._command[:4] = self._measured[:4]
            self._torso, self._torso_at = message.data, time.monotonic()
            self._owned |= message.data != 0

    def _gripper(self, index: int, message: Float32) -> None:
        if not math.isfinite(message.data) or not 0 <= message.data <= 1:
            return
        with self._lock:
            if self._target is not None:
                self._target[index] = message.data * 100
                self._owned = True

    async def handle_left_gripper_command(self, message: Float32) -> None:
        self._gripper(18, message)

    async def handle_right_gripper_command(self, message: Float32) -> None:
        self._gripper(19, message)

    async def handle_cmd_vel(self, message: Twist) -> None:
        values = (message.linear.x, message.linear.y, message.angular.z)
        if not np.isfinite(values).all():
            return
        with self._lock:
            self._drive, self._drive_at = message, time.monotonic()

    def tick(self, now: float, dt: float = 0.02) -> None:
        with self._lock:
            fresh = (
                self._measured is not None and now - self._feedback_at < self.config.input_timeout_s
            )
            drive = (
                self._drive
                if fresh and now - self._drive_at < self.config.input_timeout_s
                else Twist.zero()
            )
            self.hardware_cmd_vel.publish(drive)
            if not fresh or not self._owned or self._target is None or self._command is None:
                return
            assert self._measured is not None
            for i in range(2):
                if (
                    not self._grips[i]
                    or now - self._buttons_at >= self.config.input_timeout_s
                    or now - self._upper_at[i] >= self.config.input_timeout_s
                ):
                    sl = slice(4 + 7 * i, 11 + 7 * i)
                    self._target[sl] = self._command[sl] = self._measured[sl]
            axis = self._torso if now - self._torso_at < self.config.input_timeout_s else 0.0
            torso = height_step(self._command[:4], axis, dt, self._limits[:4])
            if np.max(np.abs(torso - self._measured[:4])) < math.radians(5):
                self._command[:4] = torso
            # No more than 10 degrees ahead of measured feedback, even if input outruns tracking.
            target = np.clip(
                self._target[4:18],
                self._measured[4:18] - math.radians(10),
                self._measured[4:18] + math.radians(10),
            )
            step = self.config.max_joint_speed * dt
            self._command[4:18] += np.clip(target - self._command[4:18], -step, step)
            self._command[18:] = np.clip(self._target[18:], 0, 100)
            self.motor_command.publish(
                MotorCommandArray(q=self._command.tolist(), dq=[self.config.max_joint_speed] * 20)
            )

    def _run(self) -> None:
        while not self._stop.wait(0.02):
            self.tick(time.monotonic())
