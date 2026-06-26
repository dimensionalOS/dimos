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

"""Per-channel velocity tracking PID controllers.

Sits between the path-following controller and the robot hardware.
Ensures that when the outer loop requests vx=0.4 m/s, the robot
actually tracks 0.4 m/s by comparing against odom feedback and
adjusting the command sent to the robot.

Each channel (vx, vy, wz) has an independent PID with anti-windup.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class VelocityPIDConfig:
    """PID gains for one velocity channel.

    Start with P-only (Ki=Kd=0), then add I to eliminate steady-state
    error, then D if needed for damping.
    """

    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.0
    max_integral: float = 0.5  # anti-windup clamp
    output_min: float = -1.0
    output_max: float = 1.0


@dataclass
class VelocityTrackingConfig:
    """Configuration for all three velocity channels."""

    vx: VelocityPIDConfig = None  # type: ignore[assignment]
    vy: VelocityPIDConfig = None  # type: ignore[assignment]
    wz: VelocityPIDConfig = None  # type: ignore[assignment]
    dt: float = 0.1  # control period (s)

    def __post_init__(self) -> None:
        if self.vx is None:
            self.vx = VelocityPIDConfig(kp=1.0, ki=0.0, kd=0.0, output_min=-1.0, output_max=1.0)
        if self.vy is None:
            self.vy = VelocityPIDConfig(kp=1.0, ki=0.0, kd=0.0, output_min=-1.0, output_max=1.0)
        if self.wz is None:
            self.wz = VelocityPIDConfig(kp=1.0, ki=0.0, kd=0.0, output_min=-1.0, output_max=1.0)


class SingleChannelPID:
    """PID controller for one velocity channel."""

    def __init__(self, config: VelocityPIDConfig, dt: float) -> None:
        self._cfg = config
        self._dt = dt
        self._integral = 0.0
        self._prev_error = 0.0
        self._first_call = True

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = 0.0
        self._first_call = True

    def compute(self, desired: float, actual: float) -> float:
        """Compute adjusted command to track desired velocity.

        Args:
            desired: Target velocity from outer loop.
            actual: Measured velocity from odom.

        Returns:
            Adjusted command to send to robot.
        """
        error = desired - actual

        # Proportional
        p_term = self._cfg.kp * error

        # Integral with anti-windup
        self._integral += error * self._dt
        self._integral = _clamp(self._integral, -self._cfg.max_integral, self._cfg.max_integral)
        i_term = self._cfg.ki * self._integral

        # Derivative (skip on first call to avoid spike)
        if self._first_call:
            d_term = 0.0
            self._first_call = False
        else:
            d_term = self._cfg.kd * (error - self._prev_error) / self._dt

        self._prev_error = error

        # Feedforward + PID correction
        # The feedforward is the desired value itself - PID corrects the error
        output = desired + p_term + i_term + d_term
        return _clamp(output, self._cfg.output_min, self._cfg.output_max)


class VelocityTrackingPID:
    """Three independent PIDs for (vx, vy, wz) velocity tracking.

    Usage:
        pid = VelocityTrackingPID(config)

        # Each control tick:
        adjusted_vx, adjusted_vy, adjusted_wz = pid.compute(
            desired_vx, desired_vy, desired_wz,
            actual_vx, actual_vy, actual_wz,
        )
    """

    def __init__(self, config: VelocityTrackingConfig | None = None) -> None:
        cfg = config or VelocityTrackingConfig()
        self._pid_vx = SingleChannelPID(cfg.vx, cfg.dt)
        self._pid_vy = SingleChannelPID(cfg.vy, cfg.dt)
        self._pid_wz = SingleChannelPID(cfg.wz, cfg.dt)

    def compute(
        self,
        desired_vx: float,
        desired_vy: float,
        desired_wz: float,
        actual_vx: float,
        actual_vy: float,
        actual_wz: float,
    ) -> tuple[float, float, float]:
        """Compute adjusted commands for all three channels.

        Args:
            desired_*: Target velocities from outer loop (Lyapunov controller).
            actual_*: Measured velocities from odom.

        Returns:
            (adjusted_vx, adjusted_vy, adjusted_wz) to send to robot.
        """
        return (
            self._pid_vx.compute(desired_vx, actual_vx),
            self._pid_vy.compute(desired_vy, actual_vy),
            self._pid_wz.compute(desired_wz, actual_wz),
        )

    def reset(self) -> None:
        self._pid_vx.reset()
        self._pid_vy.reset()
        self._pid_wz.reset()


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


__all__ = [
    "VelocityPIDConfig",
    "VelocityTrackingConfig",
    "VelocityTrackingPID",
]
