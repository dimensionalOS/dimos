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

"""``profile_speed_along_polyline`` branch coverage in one chain.

Curvature and goal decel show up in closed-loop ``cmd_vel`` via
``test_holonomic_path_follower.py``; here only profile shape decisions not
reduced to a single max-speed bound: cruise plateau, zero at ends, corner cap and
braking order along arc length.
"""

from __future__ import annotations

import numpy as np
import pytest

from dimos.navigation.dannav.geometry.path_speed_profile import (
    PathSpeedProfileLimits,
    profile_speed_along_polyline,
    speed_at_progress_m,
)


def _straight_path(length_m: float) -> tuple[np.ndarray, np.ndarray]:
    path_xy = np.array([[0.0, 0.0], [length_m, 0.0]], dtype=np.float64)
    return path_xy, np.array([length_m], dtype=np.float64)


def _cumulative(path_xy: np.ndarray) -> np.ndarray:
    segments = path_xy[1:] - path_xy[:-1]
    return np.cumsum(np.linalg.norm(segments, axis=1))


def test_profile_speed_respects_cruise_corners_and_goal_endpoints() -> None:
    cruise_limits = PathSpeedProfileLimits(
        max_speed_m_s=2.0,
        max_tangent_accel_m_s2=1.0,
        max_normal_accel_m_s2=10.0,
    )
    straight_xy, straight_cum = _straight_path(10.0)
    _, straight_v = profile_speed_along_polyline(
        straight_xy,
        straight_cum,
        cruise_limits,
        goal_decel_m_s2=cruise_limits.max_tangent_accel_m_s2,
        num_samples=2001,
        start_speed_m_s=0.0,
    )
    assert straight_v[0] == pytest.approx(0.0)
    assert straight_v[-1] == pytest.approx(0.0)
    assert max(straight_v) == pytest.approx(cruise_limits.max_speed_m_s, abs=0.02)

    corner_limits = PathSpeedProfileLimits(
        max_speed_m_s=2.0,
        max_tangent_accel_m_s2=0.5,
        max_normal_accel_m_s2=0.1,
    )
    corner_xy = np.array(
        [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [2.0, 1.0]],
        dtype=np.float64,
    )
    corner_cum = _cumulative(corner_xy)
    s_profile, v_profile = profile_speed_along_polyline(
        corner_xy,
        corner_cum,
        corner_limits,
        goal_decel_m_s2=0.5,
        num_samples=200,
    )
    cruise_speed = speed_at_progress_m(0.4, s_profile, v_profile)
    before_corner_speed = speed_at_progress_m(0.85, s_profile, v_profile)
    at_corner_speed = speed_at_progress_m(1.0, s_profile, v_profile)

    assert cruise_speed > before_corner_speed > at_corner_speed
    assert at_corner_speed < corner_limits.max_speed_m_s
