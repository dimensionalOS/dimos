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

"""Minimal visual servoing controller for drone camera (forward- or downward-facing)."""

from typing import TypeAlias

from dimos.utils.simple_controller import PIDController

# Type alias for PID parameters tuple
PIDParams: TypeAlias = tuple[float, float, float, tuple[float, float], float | None, int]


class DroneVisualServoingController:
    """Visual servoing for drone camera using velocity-only control."""

    # Constant forward speed (m/s) while following; forward from image will be added later.
    DEFAULT_FORWARD_SPEED = 0.2
    # Gain: vertical pixel error -> vz (m/s). NED: positive = down. Object below center -> descend.
    DEFAULT_VERTICAL_ERROR_GAIN = 0.0012
    MAX_VZ = 0.45  # m/s
    # Gain: lateral pixel error -> yaw rate (rad/s). Object right of center -> positive yaw_rate (turn right).
    DEFAULT_LATERAL_ERROR_TO_YAW_RATE = 0.001
    MAX_YAW_RATE = 0.5  # rad/s

    def __init__(
        self,
        x_pid_params: PIDParams,
        y_pid_params: PIDParams,
        z_pid_params: PIDParams | None = None,
        forward_camera: bool = True,
        forward_speed: float | None = None,
        vertical_error_gain: float | None = None,
        lateral_error_to_yaw_rate: float | None = None,
    ) -> None:
        """
        Initialize drone visual servoing controller.

        Args:
            x_pid_params: Reserved for forward from image later.
            y_pid_params: Reserved (lateral error drives yaw rate, not strafe).
            z_pid_params: Optional; unused when using vertical_error_gain.
            forward_camera: Reserved for later.
            forward_speed: Constant vx (m/s). Default 0.2.
            vertical_error_gain: Image vertical error (px) -> vz. Default 0.0008.
            lateral_error_to_yaw_rate: Image lateral error (px) -> yaw_rate (rad/s). Default 0.001.
        """
        self.x_pid = PIDController(*x_pid_params)
        self.y_pid = PIDController(*y_pid_params)
        self.z_pid = PIDController(*z_pid_params) if z_pid_params else None
        self.forward_camera = forward_camera
        self.forward_speed = forward_speed if forward_speed is not None else self.DEFAULT_FORWARD_SPEED
        self.vertical_error_gain = (
            vertical_error_gain if vertical_error_gain is not None else self.DEFAULT_VERTICAL_ERROR_GAIN
        )
        self.lateral_error_to_yaw_rate = (
            lateral_error_to_yaw_rate
            if lateral_error_to_yaw_rate is not None
            else self.DEFAULT_LATERAL_ERROR_TO_YAW_RATE
        )

    def compute_velocity_control(
        self,
        target_x: float,
        target_y: float,  # Target position in image (pixels or normalized)
        center_x: float = 0.0,
        center_y: float = 0.0,  # Desired position (usually image center)
        target_z: float | None = None,
        desired_z: float | None = None,  # Optional altitude control
        dt: float = 0.1,
        lock_altitude: bool = True,
    ) -> tuple[float, float, float, float]:
        """
        Compute velocity and yaw-rate commands to center target in camera view.

        - vx: constant forward speed (no lateral velocity vy).
        - Image X error -> yaw rate (rad/s): object right of center -> turn right.
        - Image Y error -> vz (altitude): object below center -> descend (NED).

        Args:
            target_x: Target X position in image
            target_y: Target Y position in image
            center_x: Desired X position (default 0)
            center_y: Desired Y position (default 0)
            target_z: Unused
            desired_z: Unused
            dt: Time step (unused for proportional gains)
            lock_altitude: If True, vz will always be 0

        Returns:
            tuple: (vx, vy, vz, yaw_rate) — vy is always 0; yaw_rate in rad/s.
        """
        error_x = target_x - center_x  # Lateral: positive = target right of center
        error_y = target_y - center_y  # Vertical: positive = target below center

        vx = self.forward_speed
        vy = 0.0

        # Lateral error -> yaw rate (turn toward target). Right of center -> positive yaw_rate.
        yaw_rate = self.lateral_error_to_yaw_rate * error_x
        yaw_rate = max(-self.MAX_YAW_RATE, min(self.MAX_YAW_RATE, yaw_rate))

        if lock_altitude:
            vz = 0.0
        else:
            vz = self.vertical_error_gain * error_y
            vz = max(-self.MAX_VZ, min(self.MAX_VZ, vz))

        return vx, vy, vz, yaw_rate

    def reset(self) -> None:
        """Reset all PID controllers."""
        self.x_pid.integral = 0.0
        self.x_pid.prev_error = 0.0
        self.y_pid.integral = 0.0
        self.y_pid.prev_error = 0.0
        if self.z_pid:
            self.z_pid.integral = 0.0
            self.z_pid.prev_error = 0.0
