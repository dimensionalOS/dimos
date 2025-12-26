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

import math
from typing import Protocol

import numpy as np
from numpy.typing import NDArray

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs import Twist, Vector3
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.utils.transform_utils import normalize_angle


class Controller(Protocol):
    def advance(self, lookahead_point: NDArray[np.float64], current_odom: PoseStamped) -> Twist: ...

    def rotate(self, yaw_error: float) -> Twist: ...

    def reset_errors(self) -> None: ...

    def reset_yaw_error(self, value: float) -> None: ...


class PdController:
    _global_config: GlobalConfig
    _speed: float
    _control_frequency: float

    _min_linear_velocity: float = 0.2
    _min_angular_velocity: float = 0.2
    _k_angular: float = 0.5
    _k_derivative: float = 0.15
    _max_angular_accel: float = 2.0
    _rotation_threshold: float = 90

    _prev_yaw_error: float
    _prev_angular_velocity: float

    def __init__(self, global_config: GlobalConfig, speed: float, control_frequency: float):
        self._global_config = global_config
        self._speed = speed
        self._control_frequency = control_frequency

        self._prev_yaw_error = 0.0
        self._prev_angular_velocity = 0.0

    def advance(self, lookahead_point: NDArray[np.float64], current_odom: PoseStamped) -> Twist:
        current_pos = np.array([current_odom.position.x, current_odom.position.y])
        direction = lookahead_point - current_pos
        distance = np.linalg.norm(direction)

        if distance < 1e-6:
            # Robot is coincidentally at the lookahead point; skip this cycle.
            return Twist()

        robot_yaw = current_odom.orientation.euler[2]
        desired_yaw = np.arctan2(direction[1], direction[0])
        yaw_error = normalize_angle(desired_yaw - robot_yaw)

        # Rotate-then-drive: if heading error is large, rotate in place first
        rotation_threshold = self._rotation_threshold * math.pi / 180
        if abs(yaw_error) > rotation_threshold:
            angular_velocity = self._compute_angular_velocity(yaw_error)
            return Twist(
                linear=Vector3(0.0, 0.0, 0.0),
                angular=Vector3(0.0, 0.0, angular_velocity),
            )

        # When aligned, drive forward with proportional angular correction
        angular_velocity = self._compute_angular_velocity(yaw_error)
        linear_velocity = self._speed * (1.0 - abs(yaw_error) / rotation_threshold)
        linear_velocity = self._apply_min_velocity(linear_velocity, self._min_linear_velocity)

        return Twist(
            linear=Vector3(linear_velocity, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, angular_velocity),
        )

    def rotate(self, yaw_error: float) -> Twist:
        angular_velocity = self._compute_angular_velocity(yaw_error)
        return self._angular_twist(angular_velocity)

    def _compute_angular_velocity(self, yaw_error: float) -> float:
        dt = 1.0 / self._control_frequency

        # PD control: proportional + derivative damping
        yaw_error_derivative = (yaw_error - self._prev_yaw_error) / dt
        angular_velocity = self._k_angular * yaw_error - self._k_derivative * yaw_error_derivative

        # Rate limiting: limit angular acceleration to prevent jerky corrections
        max_delta = self._max_angular_accel * dt
        angular_velocity = np.clip(
            angular_velocity,
            self._prev_angular_velocity - max_delta,
            self._prev_angular_velocity + max_delta,
        )

        # Clamp to max speed
        angular_velocity = np.clip(angular_velocity, -self._speed, self._speed)

        # Apply minimum velocity threshold
        angular_velocity = self._apply_min_velocity(angular_velocity, self._min_angular_velocity)

        # Update state for next iteration
        self._prev_yaw_error = yaw_error
        self._prev_angular_velocity = angular_velocity

        return float(angular_velocity)

    def reset_errors(self) -> None:
        self._prev_yaw_error = 0.0
        self._prev_angular_velocity = 0.0

    def reset_yaw_error(self, value: float) -> None:
        self._prev_yaw_error = value

    def _apply_min_velocity(self, velocity: float, min_velocity: float) -> float:
        """Apply minimum velocity threshold, preserving sign. Returns 0 if velocity is 0."""
        if velocity == 0.0:
            return 0.0
        if abs(velocity) < min_velocity:
            return min_velocity if velocity > 0 else -min_velocity
        return velocity

    def _angular_twist(self, angular_velocity: float) -> Twist:
        # In simulation, add a small forward velocity to help the locomotion
        # policy execute rotation (some policies don't handle pure in-place rotation).
        linear_x = 0.18 if self._global_config.simulation else 0.0

        return Twist(
            linear=Vector3(linear_x, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, angular_velocity),
        )
