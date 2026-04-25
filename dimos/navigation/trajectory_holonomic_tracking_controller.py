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

"""Holonomic planar trajectory tracking (P3-2 / T-09).

Cartesian law in the plan frame: position error ``(p_ref - p_meas)`` is
rotated into the measured body frame, then a proportional correction is added
to the reference body ``Twist`` (feedforward). Heading uses the same
``angle_diff`` convention as ``trajectory_metrics.pose_errors_vs_reference``.

This is a standard omnidirectional tracking law, not a path-curvature or
lookahead car-style law (no Pure Pursuit).
"""

from __future__ import annotations

import math

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.trajectory_command_limits import HolonomicCommandLimits
from dimos.navigation.trajectory_types import TrajectoryMeasuredSample, TrajectoryReferenceSample
from dimos.utils.trigonometry import angle_diff


def _planar_yaw_rad(pose_plan: object) -> float:
    return float(pose_plan.orientation.euler.z)


def _scale_planar_twist(cmd: Twist, max_planar_speed_m_s: float) -> Twist:
    sp = math.hypot(float(cmd.linear.x), float(cmd.linear.y))
    if sp <= max_planar_speed_m_s or sp < 1e-15:
        return Twist(cmd)
    f = max_planar_speed_m_s / sp
    return Twist(
        linear=Vector3(
            float(cmd.linear.x) * f,
            float(cmd.linear.y) * f,
            float(cmd.linear.z),
        ),
        angular=Vector3(
            float(cmd.angular.x),
            float(cmd.angular.y),
            float(cmd.angular.z),
        ),
    )


def _clamp_yaw_rate(cmd: Twist, max_abs_wz: float) -> Twist:
    wz = float(cmd.angular.z)
    wz = max(-max_abs_wz, min(max_abs_wz, wz))
    return Twist(
        linear=Vector3(
            float(cmd.linear.x),
            float(cmd.linear.y),
            float(cmd.linear.z),
        ),
        angular=Vector3(
            float(cmd.angular.x),
            float(cmd.angular.y),
            wz,
        ),
    )


class HolonomicTrackingController:
    """Feedforward body twist plus proportional pose tracking in the holonomic plane."""

    def __init__(
        self,
        *,
        k_position_per_s: float,
        k_yaw_per_s: float,
    ) -> None:
        if not math.isfinite(k_position_per_s) or k_position_per_s < 0.0:
            raise ValueError("k_position_per_s must be a non-negative finite float")
        if not math.isfinite(k_yaw_per_s) or k_yaw_per_s < 0.0:
            raise ValueError("k_yaw_per_s must be a non-negative finite float")
        self._kp = float(k_position_per_s)
        self._ky = float(k_yaw_per_s)
        self._limits: HolonomicCommandLimits | None = None

    def configure(self, limits: HolonomicCommandLimits) -> None:
        self._limits = limits

    def reset(self) -> None:
        pass

    def control(
        self,
        reference: TrajectoryReferenceSample,
        measurement: TrajectoryMeasuredSample,
    ) -> Twist:
        x_r = float(reference.pose_plan.position.x)
        y_r = float(reference.pose_plan.position.y)
        yaw_r = _planar_yaw_rad(reference.pose_plan)
        x_m = float(measurement.pose_plan.position.x)
        y_m = float(measurement.pose_plan.position.y)
        yaw_m = _planar_yaw_rad(measurement.pose_plan)

        dx_w = x_r - x_m
        dy_w = y_r - y_m
        c = math.cos(yaw_m)
        s = math.sin(yaw_m)
        ex_b = c * dx_w + s * dy_w
        ey_b = -s * dx_w + c * dy_w

        ref = reference.twist_body
        yaw_ref_to_meas = yaw_r - yaw_m
        c_rm = math.cos(yaw_ref_to_meas)
        s_rm = math.sin(yaw_ref_to_meas)
        vx_ff = c_rm * float(ref.linear.x) - s_rm * float(ref.linear.y)
        vy_ff = s_rm * float(ref.linear.x) + c_rm * float(ref.linear.y)
        vx = vx_ff + self._kp * ex_b
        vy = vy_ff + self._kp * ey_b
        e_psi = angle_diff(yaw_m, yaw_r)
        wz = float(ref.angular.z) - self._ky * e_psi

        raw = Twist(
            linear=Vector3(vx, vy, float(ref.linear.z)),
            angular=Vector3(
                float(ref.angular.x),
                float(ref.angular.y),
                wz,
            ),
        )
        lim = self._limits
        if lim is None:
            return raw
        out = _scale_planar_twist(raw, lim.max_planar_speed_m_s)
        return _clamp_yaw_rate(out, lim.max_yaw_rate_rad_s)


__all__ = ["HolonomicTrackingController"]
