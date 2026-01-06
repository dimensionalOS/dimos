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

"""Minimal visual servoing controller for drone with downward-facing camera."""

from typing import TypeAlias

from dimos.utils.simple_controller import PIDController

# Type alias for PID parameters tuple
PIDParams: TypeAlias = tuple[float, float, float, tuple[float, float], float | None, int]


class DroneVisualServoingController:
    """Minimal visual servoing for downward-facing drone camera using velocity-only control."""

    def __init__(
        self,
        x_pid_params: PIDParams,
        y_pid_params: PIDParams,
        z_pid_params: PIDParams | None = None,
    ) -> None:
        """
        Initialize drone visual servoing controller.

        Args:
            x_pid_params: (kp, ki, kd, output_limits, integral_limit, deadband) for forward/back
            y_pid_params: (kp, ki, kd, output_limits, integral_limit, deadband) for left/right
            z_pid_params: Optional params for altitude control
        """
        self.x_pid = PIDController(*x_pid_params)  # type: ignore[no-untyped-call]
        self.y_pid = PIDController(*y_pid_params)  # type: ignore[no-untyped-call]
        self.z_pid = PIDController(*z_pid_params) if z_pid_params else None  # type: ignore[no-untyped-call]

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
    ) -> tuple[float, float, float]:
        """
        Compute velocity commands to center target in camera view.

        For downward camera:
        - Image X error -> Drone Y velocity (left/right strafe)
        - Image Y error -> Drone X velocity (forward/backward)

        Args:
            target_x: Target X position in image
            target_y: Target Y position in image
            center_x: Desired X position (default 0)
            center_y: Desired Y position (default 0)
            target_z: Current altitude (optional)
            desired_z: Desired altitude (optional)
            dt: Time step
            lock_altitude: If True, vz will always be 0

        Returns:
            tuple: (vx, vy, vz) velocities in m/s
        """
        # Compute errors (positive = target is to the right/below center)
        error_x = target_x - center_x  # Lateral error in image
        error_y = target_y - center_y  # Forward error in image

        # PID control (swap axes for downward camera)
        # For downward camera: object below center (positive error_y) = object is behind drone
        # Need to negate: positive error_y should give negative vx (move backward)
        vy = self.y_pid.update(error_x, dt)  # type: ignore[no-untyped-call]  # Image X -> Drone Y (strafe)
        vx = -self.x_pid.update(error_y, dt)  # type: ignore[no-untyped-call]  # Image Y -> Drone X (NEGATED)

        # Optional altitude control
        vz = 0.0
        if not lock_altitude and self.z_pid and target_z is not None and desired_z is not None:
            error_z = target_z - desired_z
            vz = self.z_pid.update(error_z, dt)  # type: ignore[no-untyped-call]

        return vx, vy, vz

    def reset(self) -> None:
        """Reset all PID controllers."""
        self.x_pid.integral = 0.0
        self.x_pid.prev_error = 0.0
        self.y_pid.integral = 0.0
        self.y_pid.prev_error = 0.0
        if self.z_pid:
            self.z_pid.integral = 0.0
            self.z_pid.prev_error = 0.0
