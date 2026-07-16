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

"""``clamp_holonomic_cmd_vel`` branch coverage in two chains.

Closed-loop followers exercise this in motion; here only the limiter decisions
that are hard to see from arrival alone: planar cap/direction/accel/slew, yaw
cap/accel.
"""

from __future__ import annotations

import math

import pytest

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.dannav.holonomic_tc.command_limits import (
    HolonomicCommandLimits,
    clamp_holonomic_cmd_vel,
)


def test_clamp_planar_speed_accel_and_slew() -> None:
    dt_s = 0.1
    a_max_m_s2 = 2.0
    v_max_m_s = 1.0

    speed_out = clamp_holonomic_cmd_vel(
        Twist(),
        Twist(linear=Vector3(3.0, 4.0, 0.0)),
        HolonomicCommandLimits(v_max_m_s, 10.0, 10.0, 10.0),
        1.0,
    )
    assert math.hypot(speed_out.linear.x, speed_out.linear.y) == pytest.approx(v_max_m_s)
    assert speed_out.linear.x / speed_out.linear.y == pytest.approx(3.0 / 4.0)

    accel_limits = HolonomicCommandLimits(10.0, 10.0, a_max_m_s2, 10.0)
    from_rest = clamp_holonomic_cmd_vel(
        Twist(),
        Twist(linear=Vector3(5.0, 0.0, 0.0)),
        accel_limits,
        dt_s,
    )
    assert from_rest.linear.x == pytest.approx(a_max_m_s2 * dt_s)

    prev = Twist(linear=Vector3(0.8, 0.0, 0.0))
    slew = clamp_holonomic_cmd_vel(
        prev,
        Twist(linear=Vector3(5.0, 0.0, 0.0)),
        HolonomicCommandLimits(10.0, 10.0, 1.0, 10.0),
        dt_s,
    )
    assert slew.linear.x == pytest.approx(0.8 + 1.0 * dt_s)

    prev_moving = Twist(linear=Vector3(0.6, 0.8, 0.0))
    reversal = clamp_holonomic_cmd_vel(
        prev_moving,
        Twist(linear=Vector3(-0.6, -0.8, 0.0)),
        accel_limits,
        dt_s,
    )
    delta = math.hypot(
        reversal.linear.x - prev_moving.linear.x,
        reversal.linear.y - prev_moving.linear.y,
    )
    assert delta == pytest.approx(a_max_m_s2 * dt_s)
    assert math.hypot(reversal.linear.x, reversal.linear.y) < math.hypot(
        prev_moving.linear.x, prev_moving.linear.y
    )


def test_clamp_yaw_rate_and_accel() -> None:
    w_max_rad_s = 0.5
    rate_capped = clamp_holonomic_cmd_vel(
        Twist(),
        Twist(angular=Vector3(0.0, 0.0, 1.0)),
        HolonomicCommandLimits(1.0, w_max_rad_s, 1.0, 5.0),
        1.0,
    )
    assert abs(rate_capped.angular.z) == pytest.approx(w_max_rad_s)

    dt_s = 0.1
    w_accel_rad_s2 = 2.0
    accel_capped = clamp_holonomic_cmd_vel(
        Twist(),
        Twist(angular=Vector3(0.0, 0.0, 1.0)),
        HolonomicCommandLimits(1.0, 10.0, 1.0, w_accel_rad_s2),
        dt_s,
    )
    assert accel_capped.angular.z == pytest.approx(w_accel_rad_s2 * dt_s)
