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

"""Line and arc tests for speed along path (P3-1 / T-08)."""

import math

import pytest

from dimos.navigation.trajectory_path_speed_profile import (
    PathSpeedProfileLimits,
    circular_arc_geometry_speed_cap_m_s,
    line_segment_geometry_speed_cap_m_s,
    profile_speed_along_segment,
    time_s_from_speed_profile,
)


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
    geom = line_segment_geometry_speed_cap_m_s(limits)
    s, v = profile_speed_along_segment(10.0, geom, limits, num_samples=2001)
    assert s[0] == pytest.approx(0.0)
    assert s[-1] == pytest.approx(10.0)
    assert v[0] == pytest.approx(0.0)
    assert v[-1] == pytest.approx(0.0)
    assert max(v) == pytest.approx(2.0, rel=0, abs=0.02)
    # Mid-path should sit on the plateau
    mid = len(v) // 2
    assert v[mid] == pytest.approx(2.0, rel=0, abs=0.02)


def test_line_short_segment_triangular_peak() -> None:
    """1 m line, high nominal ``v_max``: peak limited by ``sqrt(a * L)`` triangle."""
    limits = PathSpeedProfileLimits(
        max_speed_m_s=10.0,
        max_tangent_accel_m_s2=1.0,
        max_normal_accel_m_s2=10.0,
    )
    geom = line_segment_geometry_speed_cap_m_s(limits)
    L = 1.0
    s, v = profile_speed_along_segment(L, geom, limits, num_samples=4001)
    expected_peak = math.sqrt(limits.max_tangent_accel_m_s2 * L)
    assert max(v) == pytest.approx(expected_peak, rel=0, abs=0.01)


def test_arc_centripetal_cap_below_line_speed() -> None:
    """Tight radius caps geometric speed below ``max_speed_m_s``."""
    limits = PathSpeedProfileLimits(
        max_speed_m_s=3.0,
        max_tangent_accel_m_s2=2.0,
        max_normal_accel_m_s2=1.0,
    )
    R = 0.25
    v_cap = circular_arc_geometry_speed_cap_m_s(R, limits)
    assert v_cap == pytest.approx(0.5)
    L = R * (math.pi / 2)
    s, v = profile_speed_along_segment(L, v_cap, limits, num_samples=2001)
    assert max(v) <= v_cap + 1e-6
    assert v[0] == pytest.approx(0.0)
    assert v[-1] == pytest.approx(0.0)


def test_arc_half_circle_respects_normal_accel() -> None:
    """Half circle R=1 m: ``v <= sqrt(a_n * R)`` everywhere on the profile."""
    limits = PathSpeedProfileLimits(
        max_speed_m_s=10.0,
        max_tangent_accel_m_s2=5.0,
        max_normal_accel_m_s2=2.0,
    )
    R = 1.0
    v_geom = circular_arc_geometry_speed_cap_m_s(R, limits)
    assert v_geom == pytest.approx(math.sqrt(2.0))
    L = math.pi * R
    _, v = profile_speed_along_segment(L, v_geom, limits, num_samples=3001)
    for vi in v:
        assert vi <= v_geom + 1e-5
        assert vi * vi / R <= limits.max_normal_accel_m_s2 + 1e-4


def test_time_from_speed_profile_monotone() -> None:
    limits = PathSpeedProfileLimits(
        max_speed_m_s=2.0,
        max_tangent_accel_m_s2=1.0,
        max_normal_accel_m_s2=4.0,
    )
    s, v = profile_speed_along_segment(
        5.0,
        line_segment_geometry_speed_cap_m_s(limits),
        limits,
        num_samples=501,
    )
    t = time_s_from_speed_profile(s, v)
    assert len(t) == len(s)
    assert t[0] == pytest.approx(0.0)
    for i in range(len(t) - 1):
        assert t[i + 1] >= t[i]


def test_zero_length_returns_origin() -> None:
    limits = PathSpeedProfileLimits(1.0, 1.0, 1.0)
    s, v = profile_speed_along_segment(0.0, 1.0, limits, num_samples=2)
    assert s == [0.0]
    assert v == [0.0]
