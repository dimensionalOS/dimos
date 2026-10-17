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

"""Holonomic planar trajectory tracking.

Cartesian law in the plan frame: the reference velocity (feedforward) plus a
proportional pull toward the reference position, rotated into the measured body
frame; yaw rate is a proportional pull toward the reference heading. A standard
omnidirectional tracking law, not Pure Pursuit.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.utils.trigonometry import angle_diff


@dataclass(frozen=True)
class PlanarState:
    """Position, heading and velocity in the plan frame."""

    x: float
    y: float
    yaw: float
    vx: float = 0.0
    vy: float = 0.0


def track(
    reference: PlanarState,
    x: float,
    y: float,
    yaw: float,
    k_position_per_s: float,
    k_yaw_per_s: float,
    max_speed_m_s: float,
    max_yaw_rate_rad_s: float,
) -> Twist:
    """Body twist that drives the pose ``(x, y, yaw)`` onto ``reference``."""
    wx = reference.vx + k_position_per_s * (reference.x - x)
    wy = reference.vy + k_position_per_s * (reference.y - y)
    c, s = math.cos(yaw), math.sin(yaw)
    vx, vy = c * wx + s * wy, -s * wx + c * wy
    speed = math.hypot(vx, vy)
    if speed > max_speed_m_s:
        vx, vy = vx * max_speed_m_s / speed, vy * max_speed_m_s / speed
    wz = k_yaw_per_s * angle_diff(reference.yaw, yaw)
    wz = max(-max_yaw_rate_rad_s, min(max_yaw_rate_rad_s, wz))
    return Twist(linear=Vector3(vx, vy, 0.0), angular=Vector3(0.0, 0.0, wz))
