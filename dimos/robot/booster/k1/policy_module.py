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

"""K1 policy host for standard IMU and encoder streams on hardware or sim2."""

import threading
import time
from typing import Any

import numpy as np
from pydantic import Field
import torch

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.hardware.whole_body.spec import IMUState
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.sensor_msgs.MotorCommandArray import MotorCommandArray
from dimos.robot.booster.k1.control_config import (
    JOINTS,
    KD,
    KP,
    LOWER,
    PERIOD_NS,
    POLICY_PATH,
    UPPER,
)
from dimos.robot.booster.k1.policy import K1Policy


class K1PolicyConfig(ModuleConfig):
    command_timeout: float = Field(default=0.3, gt=0, allow_inf_nan=False)


class K1PolicyModule(Module):
    config: K1PolicyConfig
    joint_state: In[JointState]
    imu: In[Imu]
    cmd_vel: In[Twist]
    motor_command: Out[MotorCommandArray]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._policy = K1Policy(POLICY_PATH)
        self._lock = threading.RLock()
        self._joints: dict[int, JointState] = {}
        self._imus: dict[int, Imu] = {}
        self._last_ns: int | None = None
        self._next_ns: int | None = None
        self._velocity = np.zeros(3, dtype=np.float32)
        self._until = 0.0
        self._inferences = 0

    @rpc
    def build(self) -> None:
        super().build()
        torch.set_num_threads(1)
        self._policy.start()
        self.motor_command.transport.start()

    @rpc
    def move(self, twist: Twist, duration: float = 0.3) -> None:
        values = (twist.linear.x, twist.linear.y, twist.angular.z)
        if not np.isfinite((*values, duration)).all() or duration <= 0:
            raise ValueError("finite velocity and positive duration required")
        with self._lock:
            self._velocity = np.clip(values, [-0.3, -0.3, -0.6], [0.5, 0.3, 0.6]).astype(np.float32)
            self._until = time.monotonic() + duration

    async def handle_cmd_vel(self, twist: Twist) -> None:
        self.move(twist, self.config.command_timeout)

    async def handle_joint_state(self, sample: JointState) -> None:
        self._receive(sample)

    async def handle_imu(self, sample: Imu) -> None:
        self._receive(sample)

    def _receive(self, sample: JointState | Imu) -> None:
        if not np.isfinite(sample.ts) or sample.ts < 0:
            raise ValueError("sensor acquisition time must be finite and nonnegative")
        # Nanosecond round trips through the standard float-seconds codecs can
        # differ by one ns. Both hardware messages carry one acquisition stamp.
        identity = round(sample.ts * 1e6)
        with self._lock:
            ns = identity * 1000
            if self._last_ns is not None and ns <= self._last_ns:
                return
            if self._next_ns is not None and ns + 2000 < self._next_ns:
                return
            if isinstance(sample, JointState):
                self._joints[identity] = sample
            else:
                self._imus[identity] = sample
            if identity in self._joints and identity in self._imus:
                command = self.compute(self._joints.pop(identity), self._imus.pop(identity))
                self.motor_command.publish(command)
                self._last_ns = ns
                if self._next_ns is None:
                    self._next_ns = ns + PERIOD_NS
                else:
                    self._next_ns += max(1, (ns - self._next_ns) // PERIOD_NS + 1) * PERIOD_NS
            cutoff = identity - 100_000
            self._joints = {k: v for k, v in self._joints.items() if k > cutoff}
            self._imus = {k: v for k, v in self._imus.items() if k > cutoff}

    def compute(self, joints: JointState, imu: Imu) -> MotorCommandArray:
        if abs(joints.ts - imu.ts) > 2e-6:
            raise ValueError("K1 requires matching encoder and IMU acquisition times")
        if (
            len(joints.name) != 22
            or set(joints.name) != set(JOINTS)
            or len(joints.position) != 22
            or len(joints.velocity) != 22
        ):
            raise ValueError("K1 requires its 22 named serial joints")
        if imu.orientation_covariance[0] < 0:
            raise ValueError("K1 requires an attitude estimate or ideal simulated orientation")
        indices = [joints.name.index(n) for n in JOINTS]
        q, w, a = imu.orientation, imu.angular_velocity, imu.linear_acceleration
        state = IMUState((q.w, q.x, q.y, q.z), (w.x, w.y, w.z), (a.x, a.y, a.z))
        with self._lock:
            velocity = (
                self._velocity if time.monotonic() < self._until else np.zeros(3, dtype=np.float32)
            )
            targets = self._policy.step(
                np.asarray(joints.position, dtype=np.float32)[indices],
                np.asarray(joints.velocity, dtype=np.float32)[indices],
                state,
                velocity,
            )
            self._inferences += 1
            return MotorCommandArray(
                q=np.clip(targets, LOWER, UPPER).tolist(),
                kp=list(KP),
                kd=list(KD),
                timestamp=joints.ts,
            )

    @rpc
    def reset_policy(self) -> None:
        with self._lock:
            self._policy.reset()
            self._last_ns = None
            self._next_ns = None
            self._joints.clear()
            self._imus.clear()
            self._velocity.fill(0)
            self._until = 0

    @rpc
    def status(self) -> dict[str, Any]:
        with self._lock:
            return {"inferences": self._inferences, "last_acquisition_ns": self._last_ns}
