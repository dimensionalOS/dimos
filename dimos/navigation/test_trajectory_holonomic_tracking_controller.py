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

"""Holonomic tracking controller and plant-in-the-loop tests (P3-2 / T-09)."""

from __future__ import annotations

import math

import pytest

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.trajectory_command_limits import HolonomicCommandLimits, clamp_holonomic_cmd_vel
from dimos.navigation.trajectory_holonomic_plant import IntegratedHolonomicPlant
from dimos.navigation.trajectory_holonomic_tracking_controller import HolonomicTrackingController
from dimos.navigation.trajectory_metrics import planar_position_divergence, pose_errors_vs_reference
from dimos.navigation.trajectory_types import TrajectoryMeasuredSample, TrajectoryReferenceSample
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


def test_closed_loop_straight_line_cross_track_converges() -> None:
    """Plant-in-the-loop: offset laterally from a constant-speed straight-line reference."""
    dt = 0.05
    limits = HolonomicCommandLimits(
        max_planar_speed_m_s=1.2,
        max_yaw_rate_rad_s=1.5,
        max_planar_linear_accel_m_s2=4.0,
        max_yaw_accel_rad_s2=4.0,
    )
    ctrl = HolonomicTrackingController(k_position_per_s=2.2, k_yaw_per_s=2.5)
    ctrl.configure(limits)
    plant = IntegratedHolonomicPlant(x=0.0, y=0.35, yaw_rad=0.0)
    v_line = 0.35
    prev_cmd = Twist()
    t_end = 6.0
    n = int(math.ceil(t_end / dt))
    for i in range(n):
        t = i * dt
        ref_pose = _pose_xy_yaw(v_line * t, 0.0, 0.0)
        ref_twist = Twist(linear=Vector3(v_line, 0.0, 0.0), angular=Vector3(0.0, 0.0, 0.0))
        meas = plant.measured_sample(t, prev_cmd)
        raw = ctrl.control(_ref(t, ref_pose, ref_twist), meas)
        prev_cmd = clamp_holonomic_cmd_vel(prev_cmd, raw, limits, dt)
        plant.step(prev_cmd, dt)

    xf = plant.x
    yf = plant.y
    yawf = plant.yaw_rad
    e_at, e_ct, e_psi = pose_errors_vs_reference(xf, yf, yawf, v_line * t_end, 0.0, 0.0)
    div = planar_position_divergence(e_at, e_ct)
    assert div < 0.08
    assert abs(e_psi) < 0.12


def test_closed_loop_in_place_spin_reduces_heading_error() -> None:
    dt = 0.05
    limits = HolonomicCommandLimits(
        max_planar_speed_m_s=0.5,
        max_yaw_rate_rad_s=1.0,
        max_planar_linear_accel_m_s2=5.0,
        max_yaw_accel_rad_s2=5.0,
    )
    ctrl = HolonomicTrackingController(k_position_per_s=1.0, k_yaw_per_s=3.0)
    ctrl.configure(limits)
    plant = IntegratedHolonomicPlant(x=0.0, y=0.0, yaw_rad=0.0)
    target_yaw = math.pi / 3
    prev_cmd = Twist()
    for i in range(200):
        t = i * dt
        ref_pose = _pose_xy_yaw(0.0, 0.0, target_yaw)
        ref_twist = Twist(angular=Vector3(0.0, 0.0, 0.0))
        meas = plant.measured_sample(t, prev_cmd)
        raw = ctrl.control(_ref(t, ref_pose, ref_twist), meas)
        prev_cmd = clamp_holonomic_cmd_vel(prev_cmd, raw, limits, dt)
        plant.step(prev_cmd, dt)

    assert abs(angle_diff(plant.yaw_rad, target_yaw)) < 0.05


def test_rejects_negative_gains() -> None:
    with pytest.raises(ValueError, match="k_position_per_s"):
        HolonomicTrackingController(k_position_per_s=-1.0, k_yaw_per_s=1.0)
