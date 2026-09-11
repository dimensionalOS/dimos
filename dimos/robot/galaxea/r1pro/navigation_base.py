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

"""Body-frame velocity interface to the existing physical MuJoCo planar servos."""

from typing import Any

import numpy as np
from numpy.typing import NDArray

MAX_SPEED = 0.08
MAX_YAW_RATE = 0.12
MAX_ACCEL = 0.06
MAX_YAW_ACCEL = 0.08
COMMAND_TIMEOUT = 0.3


class PlanarVelocityServo:
    """Integrate bounded Twist commands into actuator targets, never live poses."""

    def __init__(
        self,
        pose: NDArray[Any],
        *,
        max_speed: float = MAX_SPEED,
        max_accel: float = MAX_ACCEL,
    ) -> None:
        if not np.isfinite([max_speed, max_accel]).all() or min(max_speed, max_accel) <= 0:
            raise ValueError("Base speed and acceleration must be finite and positive")
        self.max_speed = max_speed
        self.max_accel = max_accel
        self.target = np.asarray(pose, dtype=np.float64).copy()
        self.velocity = np.zeros(3)
        self.command = np.zeros(3)
        self.command_time = float("-inf")

    def command_twist(self, velocity: NDArray[Any], now: float) -> None:
        velocity = np.asarray(velocity, dtype=np.float64)
        if velocity.shape != (3,) or not np.isfinite(velocity).all():
            raise ValueError("Base command needs three finite body-frame velocities")
        self.command[:2] = velocity[:2] * min(
            1.0, self.max_speed / max(float(np.linalg.norm(velocity[:2])), 1e-9)
        )
        self.command[2] = np.clip(velocity[2], -MAX_YAW_RATE, MAX_YAW_RATE)
        self.command_time = now

    def stop(self, pose: NDArray[Any]) -> None:
        self.target[:] = pose
        self.velocity[:] = 0
        self.command[:] = 0
        self.command_time = float("-inf")

    def step(self, pose: NDArray[Any], dt: float, now: float) -> NDArray[np.float64]:
        command = self.command if now - self.command_time <= COMMAND_TIMEOUT else np.zeros(3)
        delta = command - self.velocity
        delta[:2] *= min(1.0, self.max_accel * dt / max(float(np.linalg.norm(delta[:2])), 1e-9))
        delta[2] = np.clip(delta[2], -MAX_YAW_ACCEL * dt, MAX_YAW_ACCEL * dt)
        self.velocity += delta
        c, s = np.cos(pose[2]), np.sin(pose[2])
        vx, vy, wz = self.velocity
        self.target += dt * np.array([c * vx - s * vy, s * vx + c * vy, wz])
        # Limit integrator windup if physical contact prevents movement.
        self.target = np.asarray(pose) + np.clip(
            self.target - pose, [-0.025, -0.025, -0.05], [0.025, 0.025, 0.05]
        )
        return self.target.copy()
