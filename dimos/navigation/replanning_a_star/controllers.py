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

import math
from typing import Protocol

import numpy as np
from numpy.typing import NDArray

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs import Twist, Vector3
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.utils.trigonometry import angle_diff


class Controller(Protocol):
    def advance(self, lookahead_point: NDArray[np.float64], current_odom: PoseStamped) -> Twist: ...

    def rotate(self, yaw_error: float) -> Twist: ...

    def reset_errors(self) -> None: ...

    def reset_yaw_error(self, value: float) -> None: ...


class PController:
    _global_config: GlobalConfig
    _speed: float
    _control_frequency: float

    _min_linear_velocity: float = 0.2
    _min_angular_velocity: float = 0.2
    _k_angular: float = 0.5
    _max_angular_accel: float = 2.0
    _rotation_threshold: float = 90 * (math.pi / 180)

    def __init__(self, global_config: GlobalConfig, speed: float, control_frequency: float):
        self._global_config = global_config
        self._speed = speed
        self._control_frequency = control_frequency

    def advance(self, lookahead_point: NDArray[np.float64], current_odom: PoseStamped) -> Twist:
        current_pos = np.array([current_odom.position.x, current_odom.position.y])
        direction = lookahead_point - current_pos
        distance = np.linalg.norm(direction)

        if distance < 1e-6:
            # Robot is coincidentally at the lookahead point; skip this cycle.
            return Twist()

        robot_yaw = current_odom.orientation.euler[2]
        desired_yaw = np.arctan2(direction[1], direction[0])
        yaw_error = angle_diff(desired_yaw, robot_yaw)

        angular_velocity = self._compute_angular_velocity(yaw_error)

        # Rotate-then-drive: if heading error is large, rotate in place first
        if abs(yaw_error) > self._rotation_threshold:
            return self._angular_twist(angular_velocity)

        # When aligned, drive forward with proportional angular correction
        linear_velocity = self._speed * (1.0 - abs(yaw_error) / self._rotation_threshold)
        linear_velocity = self._apply_min_velocity(linear_velocity, self._min_linear_velocity)

        return Twist(
            linear=Vector3(linear_velocity, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, angular_velocity),
        )

    def rotate(self, yaw_error: float) -> Twist:
        angular_velocity = self._compute_angular_velocity(yaw_error)
        return self._angular_twist(angular_velocity)

    def _compute_angular_velocity(self, yaw_error: float) -> float:
        angular_velocity = self._k_angular * yaw_error
        angular_velocity = np.clip(angular_velocity, -self._speed, self._speed)
        angular_velocity = self._apply_min_velocity(angular_velocity, self._min_angular_velocity)
        return float(angular_velocity)

    def reset_errors(self) -> None:
        pass

    def reset_yaw_error(self, value: float) -> None:
        pass

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


class PdController(PController):
    _k_derivative: float = 0.15

    _prev_yaw_error: float
    _prev_angular_velocity: float

    def __init__(self, global_config: GlobalConfig, speed: float, control_frequency: float):
        super().__init__(global_config, speed, control_frequency)

        self._prev_yaw_error = 0.0
        self._prev_angular_velocity = 0.0

    def reset_errors(self) -> None:
        self._prev_yaw_error = 0.0
        self._prev_angular_velocity = 0.0

    def reset_yaw_error(self, value: float) -> None:
        self._prev_yaw_error = value

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

        angular_velocity = np.clip(angular_velocity, -self._speed, self._speed)
        angular_velocity = self._apply_min_velocity(angular_velocity, self._min_angular_velocity)

        self._prev_yaw_error = yaw_error
        self._prev_angular_velocity = angular_velocity

        return float(angular_velocity)


class PIDCrossTrackController:
    """PID controller on lateral (cross‑track) error to tighten path tracking.

    Produces an additional angular velocity term (Δω) that you add on top
    of the base yaw command.
    
    The Integral term helps eliminate steady-state lateral bias, especially
    useful when the robot consistently drifts to one side of the path.
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
        """Initialize PID cross-track controller.
        
        Args:
            control_frequency: Control loop frequency (Hz)
            k_p: Proportional gain
            k_i: Integral gain
            k_d: Derivative gain
            max_correction: Maximum total correction (rad/s)
            max_integral: Maximum integral term to prevent windup (rad/s)
        """
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
        """Return additional angular velocity (Δω) for given lateral error.

        Positive error means robot is to the left of the path; negative to the right.
        
        Args:
            cross_track_error: Signed lateral distance from path (m)
            
        Returns:
            Angular velocity correction (rad/s)
        """
        error = float(cross_track_error)
        
        # Proportional term
        p_term = self._k_p * error
        
        # Integral term (with windup protection)
        self._integral += error * self._dt
        # Clamp integral to prevent windup
        self._integral = float(np.clip(
            self._integral,
            -self._max_integral / max(self._k_i, 1e-6),
            self._max_integral / max(self._k_i, 1e-6),
        ))
        i_term = self._k_i * self._integral
        
        # Derivative term
        derr = (error - self._prev_error) / self._dt
        d_term = self._k_d * derr
        
        # Total PID correction
        correction = p_term + i_term + d_term
        correction = float(np.clip(correction, -self._max_correction, self._max_correction))

        self._prev_error = error
        return correction


class PurePursuitController:
    """Pure Pursuit path-following controller with adaptive lookahead.
    
    Pure Pursuit is a geometric path-tracking algorithm that:
    1. Finds a lookahead point on the path
    2. Computes the arc to reach that point
    3. Uses the arc's curvature to determine angular velocity
    
    This controller is path-aware and handles high speeds better than P/PD controllers.
    """
    
    _global_config: GlobalConfig
    _control_frequency: float
    
    # Lookahead parameters
    _min_lookahead: float = 0.2  # Minimum lookahead distance (m)
    _max_lookahead: float = 0.8  # Maximum lookahead distance (m)
    _lookahead_gain: float = 0.25  # Gain for adaptive lookahead (speed-based)
    
    # Control parameters
    _k_angular: float = 0.6  # Angular velocity gain
    _max_angular_velocity: float = 1.2  # rad/s
    _min_linear_velocity: float = 0.05  # m/s
    _max_linear_speed: float = 0.8      # new: cap linear speed here too
    
    def __init__(
        self,
        global_config: GlobalConfig,
        control_frequency: float,
        min_lookahead: float = 0.3,
        max_lookahead: float = 2.0,
        lookahead_gain: float = 0.5,
    ):
        """Initialize Pure Pursuit controller.
        
        Args:
            global_config: Global configuration
            control_frequency: Control loop frequency (Hz)
            min_lookahead: Minimum lookahead distance (m)
            max_lookahead: Maximum lookahead distance (m)
            lookahead_gain: Gain for speed-adaptive lookahead
        """
        self._global_config = global_config
        self._control_frequency = control_frequency
        self._min_lookahead = min_lookahead
        self._max_lookahead = max_lookahead
        self._lookahead_gain = lookahead_gain
    
    def advance(
        self,
        lookahead_point: NDArray[np.float64],
        current_odom: PoseStamped,
        current_speed: float = 0.0,
        path_curvature: float | None = None,
    ) -> Twist:
        """Compute control command using Pure Pursuit.
        
        Args:
            lookahead_point: Lookahead point on path [x, y]
            current_odom: Current robot pose
            current_speed: Current forward speed (m/s) for adaptive lookahead
            path_curvature: Curvature at lookahead point (1/m), optional
            
        Returns:
            Twist command
        """
        current_pos = np.array([current_odom.position.x, current_odom.position.y])
        robot_yaw = current_odom.orientation.euler[2]
        
        # Vector from robot to lookahead point
        to_lookahead = lookahead_point - current_pos
        distance_to_lookahead = np.linalg.norm(to_lookahead)
        
        if distance_to_lookahead < 1e-6:
            return Twist()
        
        # Angle to lookahead point
        angle_to_lookahead = np.arctan2(to_lookahead[1], to_lookahead[0])
        heading_error = angle_diff(angle_to_lookahead, robot_yaw)

        # Pure Pursuit geometry: signed curvature of arc to lookahead point.
        # Standard formula: κ = 2 * sin(α) / L  (sign comes from sin(α))
        if distance_to_lookahead < 1e-6:
            return Twist()

        if abs(heading_error) < 1e-6:
            curvature = 0.0
        else:
            curvature = 2.0 * np.sin(heading_error) / distance_to_lookahead

        # Clamp requested speed to controller's own max
        current_speed = min(current_speed, self._max_linear_speed)
        
        # Compute angular velocity from curvature
        # ω = v * κ (for differential drive)
        # But we also add proportional correction for heading error
        if current_speed > 0.1:
            # Use curvature-based control when moving
            angular_velocity = current_speed * curvature
            # Add proportional correction
            angular_velocity += self._k_angular * heading_error
        else:
            # When slow/stopped, use pure proportional control
            angular_velocity = self._k_angular * heading_error
        
        # Limit angular velocity
        angular_velocity = np.clip(angular_velocity, -self._max_angular_velocity, self._max_angular_velocity)
        
        # Base linear velocity on requested speed
        linear_velocity = current_speed

        # Rotate-then-drive behavior:
        # - if heading error > 90°, rotate in place (no forward motion)
        # - if 45°–90°, move very slowly while turning
        abs_heading = abs(heading_error)
        if abs_heading > np.pi / 2.0:  # > 90 degrees
            linear_velocity = 0.0
        elif abs_heading > np.pi / 4.0:  # 45–90 degrees
            linear_velocity *= 0.3

        # Also limit speed based on curvature if provided
        if path_curvature is not None and path_curvature > 1e-6:
            # v_max ≈ sqrt(a_max / κ) with a_max ≈ 0.8 m/s² (very conservative)
            v_curv = float(np.sqrt(0.8 / path_curvature))
            linear_velocity = min(linear_velocity, v_curv)

        # Enforce bounds and minimum crawl speed
        linear_velocity = float(np.clip(linear_velocity, 0.0, self._max_linear_speed))
        if 0.0 < linear_velocity < self._min_linear_velocity:
            linear_velocity = self._min_linear_velocity

        return Twist(
            linear=Vector3(linear_velocity, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, float(angular_velocity)),
        )
    
    def rotate(self, yaw_error: float) -> Twist:
        """Rotate in place.
        
        Args:
            yaw_error: Yaw error in radians
            
        Returns:
            Twist command for rotation
        """
        angular_velocity = self._k_angular * yaw_error
        angular_velocity = np.clip(angular_velocity, -self._max_angular_velocity, self._max_angular_velocity)
        
        # In simulation, add small forward velocity
        linear_x = 0.18 if self._global_config.simulation else 0.0
        
        return Twist(
            linear=Vector3(linear_x, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, float(angular_velocity)),
        )
    
    def reset_errors(self) -> None:
        """Reset controller state."""
        pass
    
    def reset_yaw_error(self, value: float) -> None:
        """Reset yaw error (not used in Pure Pursuit)."""
        pass
