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

"""Holonomic tracking controller unit tests."""

from __future__ import annotations

import math

import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.holonomic_trajectory_controller.trajectory_command_limits import (
    HolonomicCommandLimits,
)
from dimos.navigation.holonomic_trajectory_controller.trajectory_holonomic_tracking_controller import HolonomicTrackingController
from dimos.navigation.holonomic_trajectory_controller.trajectory_types import TrajectoryMeasuredSample, TrajectoryReferenceSample
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


def _ref(time_s: float, pose: Pose, twist: Twist) -> TrajectoryReferenceSample:
    return TrajectoryReferenceSample(time_s=time_s, pose_plan=pose, twist_body=twist)


def _meas(time_s: float, pose: Pose, twist: Twist) -> TrajectoryMeasuredSample:
    return TrajectoryMeasuredSample(time_s=time_s, pose_plan=pose, twist_body=twist)


def test_tracking_feedforward_when_aligned() -> None:
    ctrl = HolonomicTrackingController(k_position_per_s=2.0, k_yaw_per_s=1.5)
    ref_twist = Twist(linear=Vector3(0.4, -0.1, 0.0), angular=Vector3(0.0, 0.0, 0.05))
    p = _pose_xy_yaw(1.0, -2.0, math.pi / 6)
    out = ctrl.control(_ref(0.0, p, ref_twist), _meas(0.0, p, Twist()))
    assert out.linear.x == pytest.approx(0.4)
    assert out.linear.y == pytest.approx(-0.1)
    assert out.angular.z == pytest.approx(0.05)


def test_default_damping_gains_ignore_measured_velocity() -> None:
    ctrl = HolonomicTrackingController(k_position_per_s=2.0, k_yaw_per_s=1.5)
    ref_twist = Twist(linear=Vector3(0.4, -0.1, 0.0), angular=Vector3(0.0, 0.0, 0.05))
    measured_twist = Twist(linear=Vector3(1.2, -0.8, 0.0), angular=Vector3(0.0, 0.0, 0.7))
    p = _pose_xy_yaw(1.0, -2.0, math.pi / 6)
    out = ctrl.control(_ref(0.0, p, ref_twist), _meas(0.0, p, measured_twist))
    assert out.linear.x == pytest.approx(0.4)
    assert out.linear.y == pytest.approx(-0.1)
    assert out.angular.z == pytest.approx(0.05)


def test_velocity_damping_reduces_planar_command_when_measured_velocity_exceeds_reference() -> None:
    ctrl = HolonomicTrackingController(
        k_position_per_s=0.0,
        k_yaw_per_s=0.0,
        k_velocity_per_s=0.5,
    )
    ref_twist = Twist(linear=Vector3(0.5, 0.2, 0.0), angular=Vector3())
    measured_twist = Twist(linear=Vector3(1.1, 0.6, 0.0), angular=Vector3())
    p = _pose_xy_yaw(0.0, 0.0, 0.0)

    out = ctrl.control(_ref(0.0, p, ref_twist), _meas(0.0, p, measured_twist))

    assert out.linear.x == pytest.approx(0.2)
    assert out.linear.y == pytest.approx(0.0)
    assert math.hypot(out.linear.x, out.linear.y) < math.hypot(
        ref_twist.linear.x, ref_twist.linear.y
    )


def test_yaw_rate_damping_reduces_command_when_measured_rate_exceeds_reference() -> None:
    ctrl = HolonomicTrackingController(
        k_position_per_s=0.0,
        k_yaw_per_s=0.0,
        k_yaw_rate_per_s=0.5,
    )
    ref_twist = Twist(linear=Vector3(), angular=Vector3(0.0, 0.0, 0.4))
    measured_twist = Twist(linear=Vector3(), angular=Vector3(0.0, 0.0, 0.9))
    p = _pose_xy_yaw(0.0, 0.0, 0.0)

    out = ctrl.control(_ref(0.0, p, ref_twist), _meas(0.0, p, measured_twist))

    assert out.angular.z == pytest.approx(0.15)
    assert abs(out.angular.z) < abs(ref_twist.angular.z)


def test_velocity_damping_does_not_increase_planar_command_when_measured_velocity_is_below_reference() -> (
    None
):
    ctrl = HolonomicTrackingController(
        k_position_per_s=0.0,
        k_yaw_per_s=0.0,
        k_velocity_per_s=0.5,
    )
    ref_twist = Twist(linear=Vector3(0.5, -0.4, 0.0), angular=Vector3())
    measured_twist = Twist(linear=Vector3(0.2, -0.1, 0.0), angular=Vector3())
    p = _pose_xy_yaw(0.0, 0.0, 0.0)

    out = ctrl.control(_ref(0.0, p, ref_twist), _meas(0.0, p, measured_twist))

    assert out.linear.x == pytest.approx(0.5)
    assert out.linear.y == pytest.approx(-0.4)


def test_yaw_rate_damping_does_not_increase_command_when_measured_rate_is_below_reference() -> None:
    ctrl = HolonomicTrackingController(
        k_position_per_s=0.0,
        k_yaw_per_s=0.0,
        k_yaw_rate_per_s=0.5,
    )
    ref_twist = Twist(linear=Vector3(), angular=Vector3(0.0, 0.0, 0.4))
    measured_twist = Twist(linear=Vector3(), angular=Vector3(0.0, 0.0, 0.1))
    p = _pose_xy_yaw(0.0, 0.0, 0.0)

    out = ctrl.control(_ref(0.0, p, ref_twist), _meas(0.0, p, measured_twist))

    assert out.angular.z == pytest.approx(0.4)


@pytest.mark.parametrize("value", [-0.1, math.inf, math.nan])
def test_velocity_damping_gain_validation_rejects_negative_or_non_finite(value: float) -> None:
    with pytest.raises(ValueError, match="k_velocity_per_s"):
        HolonomicTrackingController(
            k_position_per_s=1.0,
            k_yaw_per_s=1.0,
            k_velocity_per_s=value,
        )


@pytest.mark.parametrize("value", [-0.1, math.inf, math.nan])
def test_yaw_rate_damping_gain_validation_rejects_negative_or_non_finite(value: float) -> None:
    with pytest.raises(ValueError, match="k_yaw_rate_per_s"):
        HolonomicTrackingController(
            k_position_per_s=1.0,
            k_yaw_per_s=1.0,
            k_yaw_rate_per_s=value,
        )


def test_tracking_rotates_reference_feedforward_into_measured_body_frame() -> None:
    ctrl = HolonomicTrackingController(k_position_per_s=0.0, k_yaw_per_s=0.0)
    ref_p = _pose_xy_yaw(0.0, 1.0, math.pi / 2.0)
    meas_p = _pose_xy_yaw(0.0, 0.0, 0.0)
    ref_twist = Twist(linear=Vector3(0.5, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
    out = ctrl.control(_ref(0.0, ref_p, ref_twist), _meas(0.0, meas_p, Twist()))
    assert out.linear.x == pytest.approx(0.0, abs=1e-12)
    assert out.linear.y == pytest.approx(0.5)


def test_tracking_pushes_toward_reference_in_body_frame() -> None:
    ctrl = HolonomicTrackingController(k_position_per_s=1.0, k_yaw_per_s=0.0)
    ref_p = _pose_xy_yaw(1.0, 0.0, 0.0)
    meas_p = _pose_xy_yaw(0.0, 0.0, 0.0)
    out = ctrl.control(
        _ref(0.0, ref_p, Twist()),
        _meas(0.0, meas_p, Twist()),
    )
    assert out.linear.x == pytest.approx(1.0)
    assert out.linear.y == pytest.approx(0.0)


def test_tracking_heading_correction_sign() -> None:
    ctrl = HolonomicTrackingController(k_position_per_s=0.0, k_yaw_per_s=2.0)
    ref_p = _pose_xy_yaw(0.0, 0.0, 0.5)
    meas_p = _pose_xy_yaw(0.0, 0.0, 0.0)
    out = ctrl.control(_ref(0.0, ref_p, Twist()), _meas(0.0, meas_p, Twist()))
    e_psi = angle_diff(0.0, 0.5)
    assert out.angular.z == pytest.approx(-2.0 * e_psi)


def test_configure_caps_planar_speed_and_yaw_rate() -> None:
    ctrl = HolonomicTrackingController(k_position_per_s=100.0, k_yaw_per_s=100.0)
    ctrl.configure(
        HolonomicCommandLimits(
            max_planar_speed_m_s=0.5,
            max_yaw_rate_rad_s=0.2,
            max_planar_linear_accel_m_s2=10.0,
            max_yaw_accel_rad_s2=10.0,
        ),
    )
    ref_p = _pose_xy_yaw(10.0, 0.0, 0.0)
    meas_p = _pose_xy_yaw(0.0, 0.0, 0.0)
    out = ctrl.control(_ref(0.0, ref_p, Twist()), _meas(0.0, meas_p, Twist()))
    assert math.hypot(out.linear.x, out.linear.y) == pytest.approx(0.5)
    ref_p2 = _pose_xy_yaw(0.0, 0.0, 10.0)
    out2 = ctrl.control(_ref(0.0, ref_p2, Twist()), _meas(0.0, meas_p, Twist()))
    assert abs(out2.angular.z) == pytest.approx(0.2)
