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

"""Polyline speed profile behavior tests."""

import math

import numpy as np
import pytest

from dimos.navigation.holonomic_trajectory_controller.trajectory_path_speed_profile import (
    PathSpeedProfileLimits,
    profile_speed_along_polyline,
    speed_at_progress_m,
)


def _straight_line_path(length_m: float) -> tuple[np.ndarray, np.ndarray]:
    path_xy = np.array([[0.0, 0.0], [length_m, 0.0]], dtype=np.float64)
    cumulative = np.array([length_m], dtype=np.float64)
    return path_xy, cumulative


def test_limits_reject_bad_values() -> None:
    with pytest.raises(ValueError, match="max_speed_m_s"):
        PathSpeedProfileLimits(
            max_speed_m_s=-1.0,
            max_tangent_accel_m_s2=1.0,
            max_normal_accel_m_s2=1.0,
        )


def test_line_long_segment_reaches_speed_plateau() -> None:
    """Straight 10 m line: enough length to hit ``v_max`` before braking."""
    limits = PathSpeedProfileLimits(
        max_speed_m_s=2.0,
        max_tangent_accel_m_s2=1.0,
        max_normal_accel_m_s2=10.0,
    )
    path_xy, cumulative = _straight_line_path(10.0)
    s, v = profile_speed_along_polyline(
        path_xy,
        cumulative,
        limits,
        goal_decel_m_s2=limits.max_tangent_accel_m_s2,
        num_samples=2001,
        start_speed_m_s=0.0,
    )
    assert s[0] == pytest.approx(0.0)
    assert s[-1] == pytest.approx(10.0)
    assert v[0] == pytest.approx(0.0)
    assert v[-1] == pytest.approx(0.0)
    assert max(v) == pytest.approx(2.0, rel=0, abs=0.02)
    mid = len(v) // 2
    assert v[mid] == pytest.approx(2.0, rel=0, abs=0.02)


def test_line_short_segment_triangular_peak() -> None:
    """1 m line, high nominal ``v_max``: peak limited by ``sqrt(a * L)`` triangle."""
    limits = PathSpeedProfileLimits(
        max_speed_m_s=10.0,
        max_tangent_accel_m_s2=1.0,
        max_normal_accel_m_s2=10.0,
    )
    length_m = 1.0
    path_xy, cumulative = _straight_line_path(length_m)
    _, v = profile_speed_along_polyline(
        path_xy,
        cumulative,
        limits,
        goal_decel_m_s2=limits.max_tangent_accel_m_s2,
        num_samples=4001,
        start_speed_m_s=0.0,
    )
    expected_peak = math.sqrt(limits.max_tangent_accel_m_s2 * length_m)
    assert max(v) == pytest.approx(expected_peak, rel=0, abs=0.01)


def test_tight_corner_centripetal_cap_below_max_speed() -> None:
    """90 deg corner: circumradius caps speed below ``max_speed_m_s``."""
    limits = PathSpeedProfileLimits(
        max_speed_m_s=3.0,
        max_tangent_accel_m_s2=2.0,
        max_normal_accel_m_s2=1.0,
    )
    leg_m = 0.25
    path_xy = np.array(
        [[0.0, 0.0], [leg_m, 0.0], [leg_m, leg_m]],
        dtype=np.float64,
    )
    segments = path_xy[1:] - path_xy[:-1]
    cumulative = np.cumsum(np.linalg.norm(segments, axis=1))
    corner_s = float(cumulative[0])
    s_profile, v_profile = profile_speed_along_polyline(
        path_xy,
        cumulative,
        limits,
        goal_decel_m_s2=limits.max_tangent_accel_m_s2,
        num_samples=200,
        start_speed_m_s=0.0,
    )
    at_corner_speed = speed_at_progress_m(corner_s, s_profile, v_profile)
    assert at_corner_speed < limits.max_speed_m_s
    assert at_corner_speed < 0.6


def test_zero_length_returns_origin() -> None:
    limits = PathSpeedProfileLimits(1.0, 1.0, 1.0)
    path_xy = np.array([[0.0, 0.0]], dtype=np.float64)
    s, v = profile_speed_along_polyline(
        path_xy,
        np.array([], dtype=np.float64),
        limits,
        goal_decel_m_s2=1.0,
    )
    assert s == [0.0]
    assert v == [0.0]


def test_polyline_profile_decelerates_before_corner() -> None:
    limits = PathSpeedProfileLimits(
        max_speed_m_s=2.0,
        max_tangent_accel_m_s2=0.5,
        max_normal_accel_m_s2=0.1,
    )
    path_xy = np.array(
        [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [2.0, 1.0]],
        dtype=np.float64,
    )
    segments = path_xy[1:] - path_xy[:-1]
    cumulative = np.cumsum(np.linalg.norm(segments, axis=1))
    s_profile, v_profile = profile_speed_along_polyline(
        path_xy,
        cumulative,
        limits,
        goal_decel_m_s2=0.5,
        num_samples=200,
    )
    corner_s = 1.0
    before_corner_speed = speed_at_progress_m(0.85, s_profile, v_profile)
    at_corner_speed = speed_at_progress_m(corner_s, s_profile, v_profile)
    cruise_speed = speed_at_progress_m(0.4, s_profile, v_profile)

    assert cruise_speed > before_corner_speed
    assert before_corner_speed > at_corner_speed
    assert at_corner_speed < 0.6
