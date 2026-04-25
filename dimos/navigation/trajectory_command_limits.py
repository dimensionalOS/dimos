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

"""Holonomic body-frame command limits and saturation (P1-3).

Limits apply to the same planar ``cmd_vel`` convention as
``trajectory_metrics.commanded_planar_speed`` (``linear.x``, ``linear.y`` in
the body frame) and to yaw rate ``angular.z``.

Non-planar components (``linear.z``, ``angular.x``, ``angular.y``) are not
slew-limited here; they are passed through from ``raw_cmd`` so a future
controller or stack can extend DOFs without this helper fighting it.

Slewing: acceleration limits are applied in the 2D linear velocity plane, then
planar speed and yaw rate limits cap the result.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3


@dataclass(frozen=True)
class HolonomicCommandLimits:
    """Upper bounds for holonomic body-frame command saturation."""

    max_planar_speed_m_s: float
    max_yaw_rate_rad_s: float
    max_planar_linear_accel_m_s2: float
    max_yaw_accel_rad_s2: float

    def __post_init__(self) -> None:
        for name, value in (
            ("max_planar_speed_m_s", self.max_planar_speed_m_s),
            ("max_yaw_rate_rad_s", self.max_yaw_rate_rad_s),
            ("max_planar_linear_accel_m_s2", self.max_planar_linear_accel_m_s2),
            ("max_yaw_accel_rad_s2", self.max_yaw_accel_rad_s2),
        ):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be a non-negative finite float, got {value!r}")


def clamp_holonomic_cmd_vel(
    previous_cmd: Twist,
    raw_cmd: Twist,
    limits: HolonomicCommandLimits,
    dt_s: float,
) -> Twist:
    """Apply acceleration then rate limits; call each tick with the previous *published* command.

    Parameters
    ----------
    previous_cmd
        Last command actually applied or sent, used for acceleration bounding.
    raw_cmd
        Unsaturated request (for example a controller output).
    limits
        Speed and acceleration envelopes.
    dt_s
        Controller period in seconds. Must be positive and finite.
    """
    if not (math.isfinite(dt_s) and dt_s > 0.0):
        raise ValueError(f"dt_s must be a finite positive scalar, got {dt_s!r}")

    p_x, p_y = float(previous_cmd.linear.x), float(previous_cmd.linear.y)
    p_wz = float(previous_cmd.angular.z)
    r_x, r_y = float(raw_cmd.linear.x), float(raw_cmd.linear.y)
    r_wz = float(raw_cmd.angular.z)

    d_x, d_y = r_x - p_x, r_y - p_y
    mag = math.hypot(d_x, d_y)
    max_d = limits.max_planar_linear_accel_m_s2 * dt_s
    if mag > 1e-15 and mag > max_d:
        s = max_d / mag
        s_x, s_y = p_x + d_x * s, p_y + d_y * s
    else:
        s_x, s_y = r_x, r_y

    sp = math.hypot(s_x, s_y)
    v_max = limits.max_planar_speed_m_s
    if sp > v_max and sp > 1e-15:
        f = v_max / sp
        s_x, s_y = s_x * f, s_y * f

    w_delta = r_wz - p_wz
    max_w_step = limits.max_yaw_accel_rad_s2 * dt_s
    w_delta = max(-max_w_step, min(max_w_step, w_delta))
    wz = p_wz + w_delta
    w_max = limits.max_yaw_rate_rad_s
    wz = max(-w_max, min(w_max, wz))

    return Twist(
        linear=Vector3(
            s_x,
            s_y,
            float(raw_cmd.linear.z),
        ),
        angular=Vector3(
            float(raw_cmd.angular.x),
            float(raw_cmd.angular.y),
            wz,
        ),
    )


__all__ = [
    "HolonomicCommandLimits",
    "clamp_holonomic_cmd_vel",
]
