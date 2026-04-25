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

"""Holonomic plants for trajectory control tests (921 P3-2, P5-2).

``IntegratedHolonomicPlant`` is an ideal omnidirectional base: commanded body
``Twist`` is integrated in the plan frame with yaw rate ``angular.z`` (no lag).

``ActuatedHolonomicPlant`` adds deterministic first-order velocity lag toward
the command plus per-axis acceleration limits (slew), and optional bounded
noise when a caller-supplied ``random.Random`` is configured. Same
``step`` / ``measured_sample`` surface as the ideal plant for controller harnesses.
"""

from __future__ import annotations

import math
import random

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.trajectory_types import TrajectoryMeasuredSample


def _pose_from_planar(x: float, y: float, yaw_rad: float) -> Pose:
    return Pose(
        x,
        y,
        0.0,
        0.0,
        0.0,
        math.sin(yaw_rad / 2.0),
        math.cos(yaw_rad / 2.0),
    )


class IntegratedHolonomicPlant:
    """Plan-frame pose state updated from body-frame ``cmd_vel``."""

    def __init__(self, *, x: float = 0.0, y: float = 0.0, yaw_rad: float = 0.0) -> None:
        self._x = float(x)
        self._y = float(y)
        self._yaw = float(yaw_rad)

    @property
    def x(self) -> float:
        return self._x

    @property
    def y(self) -> float:
        return self._y

    @property
    def yaw_rad(self) -> float:
        return self._yaw

    def measured_sample(self, time_s: float, cmd_applied: Twist) -> TrajectoryMeasuredSample:
        """Build a measurement sample; ``cmd_applied`` is the twist used at this instant."""
        return TrajectoryMeasuredSample(
            time_s=float(time_s),
            pose_plan=_pose_from_planar(self._x, self._y, self._yaw),
            twist_body=Twist(cmd_applied),
        )

    def step(self, cmd_body: Twist, dt_s: float) -> None:
        """Integrate one step with body-frame planar command (same convention as ``cmd_vel``)."""
        if not (math.isfinite(dt_s) and dt_s > 0.0):
            raise ValueError(f"dt_s must be finite and positive, got {dt_s!r}")
        vx = float(cmd_body.linear.x)
        vy = float(cmd_body.linear.y)
        wz = float(cmd_body.angular.z)
        c = math.cos(self._yaw)
        s = math.sin(self._yaw)
        vx_w = c * vx - s * vy
        vy_w = s * vx + c * vy
        self._x += vx_w * dt_s
        self._y += vy_w * dt_s
        self._yaw += wz * dt_s
        self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))


class ActuatedHolonomicPlant:
    """Plan-frame pose with lagged, rate-limited body-frame velocity (921 P5-2).

    Each ``step``:

    1. Optionally adds bounded uniform noise to the command (linear ``x``,
       ``y``, yaw rate ``z``) when ``noise_rng`` and positive noise caps are set.
    2. Updates an internal body-frame velocity toward the (noisy) command with
       first-order dynamics ``tau`` on linear and yaw channels. A time constant
       of zero disables that channel's lag (the velocity still moves only as fast
       as the acceleration limit allows).
    3. Clamps the velocity increment per axis using ``max_linear_accel_m_s2`` for
       ``linear.x`` / ``linear.y`` and ``max_yaw_accel_rad_s2`` for ``angular.z``.
    4. Integrates planar pose using the updated velocity (same convention as
       ``IntegratedHolonomicPlant``).

    ``measured_sample`` reports the **realized** body twist (internal velocity),
    not necessarily the last raw command.
    """

    def __init__(
        self,
        *,
        x: float = 0.0,
        y: float = 0.0,
        yaw_rad: float = 0.0,
        linear_lag_time_constant_s: float = 0.0,
        yaw_lag_time_constant_s: float = 0.0,
        max_linear_accel_m_s2: float = 1.0,
        max_yaw_accel_rad_s2: float = 1.0,
        noise_rng: random.Random | None = None,
        noise_linear_max_m_s: float = 0.0,
        noise_yaw_max_rad_s: float = 0.0,
    ) -> None:
        self._x = float(x)
        self._y = float(y)
        self._yaw = float(yaw_rad)
        self._vx = 0.0
        self._vy = 0.0
        self._wz = 0.0
        self._linear_lag_tc = float(linear_lag_time_constant_s)
        self._yaw_lag_tc = float(yaw_lag_time_constant_s)
        self._max_lin_a = float(max_linear_accel_m_s2)
        self._max_yaw_a = float(max_yaw_accel_rad_s2)
        self._noise_rng = noise_rng
        self._noise_lin = float(noise_linear_max_m_s)
        self._noise_yaw = float(noise_yaw_max_rad_s)
        self._validate_params()

    def _validate_params(self) -> None:
        for name, v in (
            ("linear_lag_time_constant_s", self._linear_lag_tc),
            ("yaw_lag_time_constant_s", self._yaw_lag_tc),
        ):
            if not math.isfinite(v) or v < 0.0:
                raise ValueError(f"{name} must be a non-negative finite float, got {v!r}")
        for name, v in (("max_linear_accel_m_s2", self._max_lin_a), ("max_yaw_accel_rad_s2", self._max_yaw_a)):
            if not math.isfinite(v) or v <= 0.0:
                raise ValueError(f"{name} must be a positive finite float, got {v!r}")
        for name, v in (("noise_linear_max_m_s", self._noise_lin), ("noise_yaw_max_rad_s", self._noise_yaw)):
            if not math.isfinite(v) or v < 0.0:
                raise ValueError(f"{name} must be a non-negative finite float, got {v!r}")
        if self._noise_rng is None and (self._noise_lin > 0.0 or self._noise_yaw > 0.0):
            raise ValueError("noise_rng is required when noise caps are positive")

    @property
    def x(self) -> float:
        return self._x

    @property
    def y(self) -> float:
        return self._y

    @property
    def yaw_rad(self) -> float:
        return self._yaw

    @property
    def twist_body_velocity(self) -> Twist:
        """Current realized body-frame velocity (odometry-like)."""
        return Twist(
            linear=Vector3(self._vx, self._vy, 0.0),
            angular=Vector3(0.0, 0.0, self._wz),
        )

    def reset(
        self,
        *,
        x: float | None = None,
        y: float | None = None,
        yaw_rad: float | None = None,
        clear_velocity: bool = True,
    ) -> None:
        """Reset pose and optionally zero internal velocity."""
        if x is not None:
            self._x = float(x)
        if y is not None:
            self._y = float(y)
        if yaw_rad is not None:
            self._yaw = float(yaw_rad)
            self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))
        if clear_velocity:
            self._vx = 0.0
            self._vy = 0.0
            self._wz = 0.0

    def measured_sample(self, time_s: float, cmd_applied: Twist) -> TrajectoryMeasuredSample:
        """Measured pose and **realized** body twist (internal velocity)."""
        del cmd_applied  # same signature as ``IntegratedHolonomicPlant``; realized twist is internal state
        return TrajectoryMeasuredSample(
            time_s=float(time_s),
            pose_plan=_pose_from_planar(self._x, self._y, self._yaw),
            twist_body=self.twist_body_velocity,
        )

    def _maybe_noise(self, ux: float, uy: float, uw: float) -> tuple[float, float, float]:
        if self._noise_rng is None or (self._noise_lin == 0.0 and self._noise_yaw == 0.0):
            return ux, uy, uw
        if self._noise_lin > 0.0:
            ux += self._noise_rng.uniform(-self._noise_lin, self._noise_lin)
            uy += self._noise_rng.uniform(-self._noise_lin, self._noise_lin)
        if self._noise_yaw > 0.0:
            uw += self._noise_rng.uniform(-self._noise_yaw, self._noise_yaw)
        return ux, uy, uw

    def step(self, cmd_body: Twist, dt_s: float) -> None:
        if not (math.isfinite(dt_s) and dt_s > 0.0):
            raise ValueError(f"dt_s must be finite and positive, got {dt_s!r}")
        ux = float(cmd_body.linear.x)
        uy = float(cmd_body.linear.y)
        uw = float(cmd_body.angular.z)
        ux, uy, uw = self._maybe_noise(ux, uy, uw)

        ex = ux - self._vx
        ey = uy - self._vy
        ew = uw - self._wz

        if self._linear_lag_tc > 0.0:
            dvx = (dt_s / self._linear_lag_tc) * ex
            dvy = (dt_s / self._linear_lag_tc) * ey
        else:
            dvx, dvy = ex, ey
        if self._yaw_lag_tc > 0.0:
            dw = (dt_s / self._yaw_lag_tc) * ew
        else:
            dw = ew

        lim_lin = self._max_lin_a * dt_s
        lim_yaw = self._max_yaw_a * dt_s
        dvx = max(-lim_lin, min(lim_lin, dvx))
        dvy = max(-lim_lin, min(lim_lin, dvy))
        dw = max(-lim_yaw, min(lim_yaw, dw))

        self._vx += dvx
        self._vy += dvy
        self._wz += dw

        c = math.cos(self._yaw)
        s = math.sin(self._yaw)
        vx_w = c * self._vx - s * self._vy
        vy_w = s * self._vx + c * self._vy
        self._x += vx_w * dt_s
        self._y += vy_w * dt_s
        self._yaw += self._wz * dt_s
        self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))


__all__ = ["ActuatedHolonomicPlant", "IntegratedHolonomicPlant"]
