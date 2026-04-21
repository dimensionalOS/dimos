# Copyright 2025-2026 Dimensional Inc.
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

"""Path-following controllers for the ControlCoordinator's PathFollowerTask.

PurePursuitController: Geometric path tracking via lookahead-point curvature.
PIDCrossTrackController: Lateral error correction via PID on cross-track error.

These run inside the coordinator tick loop (called by PathFollowerTask.compute()).
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.trigonometry import angle_diff


class PIDCrossTrackController:
    """PID controller on lateral (cross-track) error to tighten path tracking.

    Produces an additional angular velocity term that is added on top
    of the base yaw command from PurePursuit.

    The Integral term eliminates steady-state lateral bias (e.g. robot
    consistently drifts to one side). Anti-windup clamps the integral.
    """

    def __init__(
        self,
        control_frequency: float,
        k_p: float = 1.5,
        k_i: float = 0.1,
        k_d: float = 0.2,
        max_correction: float = 0.6,
        max_integral: float = 0.3,
    ) -> None:
        self._dt = 1.0 / control_frequency
        self._k_p = k_p
        self._k_i = k_i
        self._k_d = k_d
        self._max_correction = max_correction
        self._max_integral = max_integral

        self._prev_error = 0.0
        self._integral = 0.0

    def reset(self) -> None:
        """Reset internal state (call when starting a new path)."""
        self._prev_error = 0.0
        self._integral = 0.0

    def compute_correction(self, cross_track_error: float) -> float:
        """Return angular velocity correction for given lateral error.

        Positive error = robot left of path; negative = right.

        Returns:
            Angular velocity correction (rad/s).
        """
        error = float(cross_track_error)

        # Proportional
        p_term = self._k_p * error

        # Integral with windup protection
        self._integral += error * self._dt
        max_integral_raw = self._max_integral / max(self._k_i, 1e-6)
        self._integral = float(np.clip(self._integral, -max_integral_raw, max_integral_raw))
        i_term = self._k_i * self._integral

        # Derivative
        d_term = self._k_d * (error - self._prev_error) / self._dt

        correction = float(
            np.clip(p_term + i_term + d_term, -self._max_correction, self._max_correction)
        )
        self._prev_error = error
        return correction


class PurePursuitController:
    """Pure Pursuit path-following controller with adaptive lookahead.

    Geometric path-tracking algorithm:
    1. Find a lookahead point on the path.
    2. Compute the arc curvature to reach that point.
    3. Derive angular velocity from curvature and forward speed.

    Handles high speeds better than simple P/PD controllers because
    the steering command scales with both speed and geometric error.
    """

    def __init__(
        self,
        global_config: GlobalConfig,
        control_frequency: float,
        min_lookahead: float = 0.3,
        max_lookahead: float = 2.0,
        lookahead_gain: float = 0.5,
        max_linear_speed: float = 0.8,
        k_angular: float = 0.6,
        max_angular_velocity: float = 1.2,
        min_linear_velocity: float = 0.05,
    ) -> None:
        self._global_config = global_config
        self._control_frequency = control_frequency
        self._min_lookahead = min_lookahead
        self._max_lookahead = max_lookahead
        self._lookahead_gain = lookahead_gain
        self._max_linear_speed = max_linear_speed
        self._k_angular = k_angular
        self._max_angular_velocity = max_angular_velocity
        self._min_linear_velocity = min_linear_velocity

    def advance(
        self,
        lookahead_point: NDArray[np.float64],
        current_odom: PoseStamped,
        current_speed: float = 0.0,
        path_curvature: float | None = None,
    ) -> Twist:
        """Compute control command using Pure Pursuit geometry.

        Args:
            lookahead_point: Target point on path [x, y].
            current_odom: Current robot pose.
            current_speed: Desired forward speed (m/s) from velocity profiler.
            path_curvature: Local path curvature (1/m), optional.

        Returns:
            Twist command.
        """
        current_pos = np.array([current_odom.position.x, current_odom.position.y])
        robot_yaw = current_odom.orientation.euler[2]

        to_lookahead = lookahead_point - current_pos
        distance_to_lookahead = float(np.linalg.norm(to_lookahead))

        if distance_to_lookahead < 1e-6:
            return Twist()

        angle_to_lookahead = float(np.arctan2(to_lookahead[1], to_lookahead[0]))
        heading_error = angle_diff(angle_to_lookahead, robot_yaw)

        # Pure Pursuit curvature: kappa = 2 * sin(alpha) / L
        curvature = (
            0.0
            if abs(heading_error) < 1e-6
            else 2.0 * np.sin(heading_error) / distance_to_lookahead
        )

        current_speed = min(current_speed, self._max_linear_speed)

        # Angular velocity: omega = v * kappa + proportional heading correction
        if current_speed > 0.1:
            angular_velocity = current_speed * curvature + self._k_angular * heading_error
        else:
            angular_velocity = self._k_angular * heading_error

        angular_velocity = float(
            np.clip(angular_velocity, -self._max_angular_velocity, self._max_angular_velocity)
        )

        linear_velocity = current_speed

        # Rotate-then-drive behaviour
        abs_heading = abs(heading_error)
        if abs_heading > np.pi / 2.0:
            linear_velocity = 0.0
        elif abs_heading > np.pi / 4.0:
            linear_velocity *= 0.3

        # Curvature-based speed limit
        if path_curvature is not None and path_curvature > 1e-6:
            v_curv = float(np.sqrt(0.8 / path_curvature))
            linear_velocity = min(linear_velocity, v_curv)

        linear_velocity = float(np.clip(linear_velocity, 0.0, self._max_linear_speed))
        if 0.0 < linear_velocity < self._min_linear_velocity:
            linear_velocity = self._min_linear_velocity

        return Twist(
            linear=Vector3(linear_velocity, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, angular_velocity),
        )

    def rotate(self, yaw_error: float) -> Twist:
        """Rotate in place to correct heading."""
        angular_velocity = float(
            np.clip(
                self._k_angular * yaw_error, -self._max_angular_velocity, self._max_angular_velocity
            )
        )
        linear_x = 0.18 if self._global_config.simulation else 0.0
        return Twist(
            linear=Vector3(linear_x, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, angular_velocity),
        )


__all__ = [
    "PIDCrossTrackController",
    "PurePursuitController",
]
