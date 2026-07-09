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

"""``HolonomicTrackingController`` branch coverage in two chains.

Closed-loop followers exercise this in motion; here only tracking-law decisions
not visible from arrival alone: frame rotation, pose P, heading sign, one-sided
damping, configured speed caps.
"""

from __future__ import annotations

import math

import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.dannav.holonomic_tc.command_limits import (
    HolonomicCommandLimits,
)
from dimos.navigation.dannav.holonomic_tc.holonomic_tracking_controller import (
    HolonomicTrackingController,
)
from dimos.navigation.dannav.holonomic_tc.types import (
    TrajectoryMeasuredSample,
    TrajectoryReferenceSample,
)
from dimos.utils.trigonometry import angle_diff


def _pose_xy_yaw(x: float, y: float, yaw: float) -> Pose:
    return Pose(
        x,
        y,
        0.0,
        0.0,
        0.0,
        math.sin(yaw / 2.0),
        math.cos(yaw / 2.0),
    )


def _ref(pose: Pose, twist: Twist) -> TrajectoryReferenceSample:
    return TrajectoryReferenceSample(time_s=0.0, pose_plan=pose, twist_body=twist)


def _meas(pose: Pose, twist: Twist) -> TrajectoryMeasuredSample:
    return TrajectoryMeasuredSample(time_s=0.0, pose_plan=pose, twist_body=twist)


def _planar_speed(cmd: Twist) -> float:
    return math.hypot(float(cmd.linear.x), float(cmd.linear.y))


def test_tracking_feedforward_frame_and_pose_correction() -> None:
    origin = _pose_xy_yaw(0.0, 0.0, 0.0)
    ref_twist = Twist(linear=Vector3(0.4, -0.1, 0.0), angular=Vector3(0.0, 0.0, 0.05))
    aligned_pose = _pose_xy_yaw(1.0, -2.0, math.pi / 6)

    feedforward = HolonomicTrackingController(k_position_per_s=0.0, k_yaw_per_s=0.0)
    out = feedforward.control(_ref(aligned_pose, ref_twist), _meas(aligned_pose, Twist()))
    assert out.linear.x == pytest.approx(ref_twist.linear.x)
    assert out.linear.y == pytest.approx(ref_twist.linear.y)
    assert out.angular.z == pytest.approx(ref_twist.angular.z)

    ref_rotated = _pose_xy_yaw(0.0, 1.0, math.pi / 2.0)
    frame_out = feedforward.control(
        _ref(ref_rotated, Twist(linear=Vector3(0.5, 0.0, 0.0))),
        _meas(origin, Twist()),
    )
    assert frame_out.linear.x == pytest.approx(0.0, abs=1e-12)
    assert frame_out.linear.y == pytest.approx(0.5)

    position = HolonomicTrackingController(k_position_per_s=1.0, k_yaw_per_s=0.0)
    position_out = position.control(
        _ref(_pose_xy_yaw(1.0, 0.0, 0.0), Twist()),
        _meas(origin, Twist()),
    )
    assert position_out.linear.x == pytest.approx(1.0)
    assert position_out.linear.y == pytest.approx(0.0)

    heading = HolonomicTrackingController(k_position_per_s=0.0, k_yaw_per_s=2.0)
    heading_out = heading.control(
        _ref(_pose_xy_yaw(0.0, 0.0, 0.5), Twist()),
        _meas(origin, Twist()),
    )
    e_psi = angle_diff(0.0, 0.5)
    assert heading_out.angular.z == pytest.approx(-2.0 * e_psi)


def test_tracking_damping_is_one_sided_and_respects_limits() -> None:
    origin = _pose_xy_yaw(0.0, 0.0, 0.0)
    ref_twist = Twist(linear=Vector3(0.5, 0.2, 0.0), angular=Vector3(0.0, 0.0, 0.4))
    overspeed_meas = Twist(linear=Vector3(1.1, 0.6, 0.0), angular=Vector3(0.0, 0.0, 0.9))
    underspeed_meas = Twist(linear=Vector3(0.2, -0.1, 0.0), angular=Vector3(0.0, 0.0, 0.1))

    undamped = HolonomicTrackingController(k_position_per_s=0.0, k_yaw_per_s=0.0)
    undamped_out = undamped.control(_ref(origin, ref_twist), _meas(origin, overspeed_meas))
    assert undamped_out.linear.x == pytest.approx(ref_twist.linear.x)
    assert abs(undamped_out.angular.z) == pytest.approx(abs(ref_twist.angular.z))

    damped = HolonomicTrackingController(
        k_position_per_s=0.0,
        k_yaw_per_s=0.0,
        k_velocity_per_s=0.5,
        k_yaw_rate_per_s=0.5,
    )
    damped_over = damped.control(_ref(origin, ref_twist), _meas(origin, overspeed_meas))
    assert _planar_speed(damped_over) < _planar_speed(ref_twist)
    assert abs(damped_over.angular.z) < abs(ref_twist.angular.z)

    damped_under = damped.control(_ref(origin, ref_twist), _meas(origin, underspeed_meas))
    assert damped_under.linear.x == pytest.approx(ref_twist.linear.x)
    assert damped_under.linear.y == pytest.approx(ref_twist.linear.y)
    assert damped_under.angular.z == pytest.approx(ref_twist.angular.z)

    max_planar_m_s = 0.5
    max_yaw_rad_s = 0.2
    capped = HolonomicTrackingController(k_position_per_s=100.0, k_yaw_per_s=100.0)
    capped.configure(
        HolonomicCommandLimits(
            max_planar_speed_m_s=max_planar_m_s,
            max_yaw_rate_rad_s=max_yaw_rad_s,
            max_planar_linear_accel_m_s2=10.0,
            max_yaw_accel_rad_s2=10.0,
        ),
    )
    planar_out = capped.control(
        _ref(_pose_xy_yaw(10.0, 0.0, 0.0), Twist()),
        _meas(origin, Twist()),
    )
    yaw_out = capped.control(
        _ref(_pose_xy_yaw(0.0, 0.0, 10.0), Twist()),
        _meas(origin, Twist()),
    )
    assert _planar_speed(planar_out) == pytest.approx(max_planar_m_s)
    assert abs(yaw_out.angular.z) == pytest.approx(max_yaw_rad_s)
