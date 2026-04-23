# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on the "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Tests lock the P0-2 definitions in ``dimos.navigation.trajectory_metrics``."""

import math

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.trajectory_metrics import (
    TrackingTolerance,
    along_track_progress_error,
    commanded_planar_speed,
    planar_pose_divergence,
    planar_position_divergence,
    pose_errors_vs_reference,
    project_to_polyline,
    scale_tolerance_by_clearance,
)


def test_pose_errors_reference_frame_signs() -> None:
    """Ahead and left of reference are positive along- and cross-track."""
    e_at, e_ct, e_psi = pose_errors_vs_reference(
        x_meas=2.0,
        y_meas=1.0,
        yaw_meas=0.0,
        x_ref=0.0,
        y_ref=0.0,
        yaw_ref=0.0,
    )
    assert e_at == pytest.approx(2.0)
    assert e_ct == pytest.approx(1.0)
    assert e_psi == pytest.approx(0.0)

    e_at2, e_ct2, _ = pose_errors_vs_reference(
        x_meas=-1.0,
        y_meas=-2.0,
        yaw_meas=0.0,
        x_ref=0.0,
        y_ref=0.0,
        yaw_ref=0.0,
    )
    assert e_at2 == pytest.approx(-1.0)
    assert e_ct2 == pytest.approx(-2.0)


def test_planar_position_divergence_hypot() -> None:
    e_at, e_ct, _ = pose_errors_vs_reference(3.0, 4.0, 0.0, 0.0, 0.0, 0.0)
    assert planar_position_divergence(e_at, e_ct) == pytest.approx(5.0)


def test_planar_pose_divergence_yaw_weight() -> None:
    div0 = planar_pose_divergence(0.0, 0.0, math.pi / 2, yaw_weight_rad_to_m=0.0)
    assert div0 == pytest.approx(0.0)
    div1 = planar_pose_divergence(0.0, 0.0, math.pi / 2, yaw_weight_rad_to_m=1.0 / math.pi)
    assert div1 == pytest.approx(0.5)


def test_commanded_planar_speed_matches_twist_norm() -> None:
    cmd = Twist(linear=Vector3(0.6, 0.8, 0.0), angular=Vector3(0.0, 0.0, 1.0))
    assert commanded_planar_speed(cmd) == pytest.approx(1.0)


def test_project_polyline_single_point_is_radial_offset() -> None:
    poly = np.array([[0.0, 0.0]], dtype=np.float64)
    pr = project_to_polyline(3.0, 4.0, poly)
    assert pr.signed_cross_track_m == pytest.approx(5.0)
    assert pr.s_along_path_m == pytest.approx(0.0)


def test_project_polyline_horizontal_cross_track() -> None:
    poly = np.array([[0.0, 0.0], [10.0, 0.0]], dtype=np.float64)
    pr = project_to_polyline(5.0, 2.0, poly)
    assert pr.foot_xy == pytest.approx((5.0, 0.0))
    assert pr.signed_cross_track_m == pytest.approx(2.0)
    assert pr.s_along_path_m == pytest.approx(5.0)
    assert pr.tangent_yaw == pytest.approx(0.0)

    pr2 = project_to_polyline(5.0, -1.5, poly)
    assert pr2.signed_cross_track_m == pytest.approx(-1.5)


def test_along_track_progress_signed() -> None:
    assert along_track_progress_error(10.0, 12.0) == pytest.approx(-2.0)


def test_tracking_tolerance_and_clearance_scaling() -> None:
    base = TrackingTolerance(0.2, 0.15, 0.5)
    tight = scale_tolerance_by_clearance(base, 0.2)
    assert tight.along_track_m < base.along_track_m
    loose = scale_tolerance_by_clearance(base, 5.0)
    assert loose.cross_track_m >= base.cross_track_m


def test_trajectory_metrics_module_defines_plot_sampling_contract() -> None:
    """Guardrail: module docstring names speed sampling and divergence semantics."""
    import dimos.navigation.trajectory_metrics as tm

    doc = tm.__doc__ or ""
    assert "Along-track" in doc and "cross-track" in doc.lower()
    assert "commanded_planar_speed" in doc or "Commanded speed" in doc
    assert "clearance" in doc.lower()
