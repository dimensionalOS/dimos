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

"""Speed vs arc length for planar polylines.

Builds a scalar speed profile along a polyline using per-sample geometry caps
and a forward-backward pass on arc length (``v^2 <= v_0^2 + 2 a \\Delta s``).

- **Straight segments:** cap is ``limits.max_speed_m_s``.
- **Corners:** cap is ``min(max_speed_m_s, sqrt(max_normal_accel_m_s2 * |R|))``
  from the circumradius of the local vertex triangle.

The live planner seeds the forward pass at cruise speed so replanned paths do not
force zero speed at the origin.
"""

from __future__ import annotations

import bisect
from collections.abc import Sequence
from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True)
class PathSpeedProfileLimits:
    """Scalar limits for profiling speed along one planar path segment."""

    max_speed_m_s: float
    max_tangent_accel_m_s2: float
    max_normal_accel_m_s2: float

    def __post_init__(self) -> None:
        for name, value in (
            ("max_speed_m_s", self.max_speed_m_s),
            ("max_tangent_accel_m_s2", self.max_tangent_accel_m_s2),
            ("max_normal_accel_m_s2", self.max_normal_accel_m_s2),
        ):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be a non-negative finite float, got {value!r}")


def _circular_arc_geometry_speed_cap_m_s(
    abs_radius_m: float, limits: PathSpeedProfileLimits
) -> float:
    """Upper speed from centripetal bound ``v^2 / R <= a_n`` and ``max_speed_m_s``."""
    if not math.isfinite(abs_radius_m) or abs_radius_m <= 0.0:
        raise ValueError(f"abs_radius_m must be finite and positive, got {abs_radius_m!r}")
    v_curve = math.sqrt(limits.max_normal_accel_m_s2 * abs_radius_m)
    return min(limits.max_speed_m_s, v_curve)


def _line_segment_geometry_speed_cap_m_s(limits: PathSpeedProfileLimits) -> float:
    """Geometric cap for a straight segment (no curvature binding)."""
    return float(limits.max_speed_m_s)


def _vertex_progress_along_polyline_m(
    cumulative_segment_s_m: NDArray[np.float64],
) -> NDArray[np.float64]:
    """Arc length to each vertex; segment cumulatives are PathDistancer-style."""
    vertex_s = np.zeros(len(cumulative_segment_s_m) + 1, dtype=np.float64)
    vertex_s[1:] = cumulative_segment_s_m
    return vertex_s


def _polyline_geometry_speed_cap_m_s(
    path_xy: NDArray[np.float64],
    vertex_s_m: Sequence[float],
    progress_m: float,
    limits: PathSpeedProfileLimits,
) -> float:
    """Geometry speed cap at arc length ``progress_m`` on a planar polyline."""
    if len(path_xy) < 3:
        return _line_segment_geometry_speed_cap_m_s(limits)

    for i in range(1, len(path_xy) - 1):
        vertex_s = float(vertex_s_m[i])
        prev_s = float(vertex_s_m[i - 1])
        next_s = float(vertex_s_m[i + 1])
        local_scale = max(vertex_s - prev_s, next_s - vertex_s, 1.0)
        if abs(float(progress_m) - vertex_s) > max(1e-9, local_scale * 1e-9):
            continue
        p0, p1, p2 = path_xy[i - 1], path_xy[i], path_xy[i + 1]
        a = float(np.linalg.norm(p1 - p0))
        b = float(np.linalg.norm(p2 - p1))
        c = float(np.linalg.norm(p2 - p0))
        v10 = p1 - p0
        v20 = p2 - p0
        area2 = abs(float(v10[0] * v20[1] - v10[1] * v20[0]))
        if min(a, b, c, area2) <= 1e-9:
            return _line_segment_geometry_speed_cap_m_s(limits)
        radius = (a * b * c) / (2.0 * area2)
        return _circular_arc_geometry_speed_cap_m_s(radius, limits)
    return _line_segment_geometry_speed_cap_m_s(limits)


def _profile_sample_distances_m(
    total_length_m: float,
    vertex_s_m: Sequence[float],
    num_samples: int,
) -> list[float]:
    """Uniform arc-length samples plus every polyline vertex."""
    if total_length_m <= 0.0:
        return [0.0]
    if num_samples < 3:
        raise ValueError(f"num_samples must be at least 3, got {num_samples}")
    distances = {0.0, float(total_length_m), *(float(s) for s in vertex_s_m)}
    for i in range(num_samples):
        distances.add(total_length_m * float(i) / float(num_samples - 1))
    return sorted(distances)


def _speed_profile_from_geometry_caps(
    s_m: Sequence[float],
    geometry_cap_m_s: Sequence[float],
    *,
    max_tangent_accel_m_s2: float,
    goal_decel_m_s2: float,
    start_speed_m_s: float = 0.0,
) -> list[float]:
    """Forward tangent-accel and backward goal-decel envelope over geometry caps.

    ``start_speed_m_s`` seeds the forward pass at ``s_m[0]``. Use ``0`` for an
    offline rest-to-rest segment profile. For the live planner, pass cruise speed
    so acceleration along open path is not forced to zero at the path origin.
    """
    if len(s_m) != len(geometry_cap_m_s):
        raise ValueError("speed profile distances and caps must have the same length")
    if not s_m:
        return []

    v_forward = [0.0] * len(s_m)
    if start_speed_m_s > 0.0:
        v_forward[0] = min(float(geometry_cap_m_s[0]), start_speed_m_s)
    for i in range(len(s_m) - 1):
        ds = max(0.0, float(s_m[i + 1]) - float(s_m[i]))
        v_next_sq = v_forward[i] * v_forward[i] + 2.0 * max_tangent_accel_m_s2 * ds
        v_forward[i + 1] = min(float(geometry_cap_m_s[i + 1]), math.sqrt(max(0.0, v_next_sq)))

    v_backward = [0.0] * len(s_m)
    for i in range(len(s_m) - 1, 0, -1):
        ds = max(0.0, float(s_m[i]) - float(s_m[i - 1]))
        v_prev_sq = v_backward[i] * v_backward[i] + 2.0 * goal_decel_m_s2 * ds
        v_backward[i - 1] = min(float(geometry_cap_m_s[i - 1]), math.sqrt(max(0.0, v_prev_sq)))

    return [
        min(float(geometry_cap_m_s[i]), v_forward[i], v_backward[i]) for i in range(len(s_m))
    ]


def profile_speed_along_polyline(
    path_xy: NDArray[np.float64],
    cumulative_segment_s_m: NDArray[np.float64],
    limits: PathSpeedProfileLimits,
    goal_decel_m_s2: float,
    *,
    num_samples: int = 200,
    start_speed_m_s: float | None = None,
) -> tuple[list[float], list[float]]:
    """Speed profile along a polyline with anticipatory corner decel."""
    if len(path_xy) < 2:
        return [0.0], [0.0]

    if start_speed_m_s is None:
        start_speed_m_s = limits.max_speed_m_s

    vertex_s_m = _vertex_progress_along_polyline_m(cumulative_segment_s_m)
    total_length_m = float(vertex_s_m[-1])
    s_profile = _profile_sample_distances_m(total_length_m, vertex_s_m, num_samples)
    caps = [
        _polyline_geometry_speed_cap_m_s(path_xy, vertex_s_m, s, limits) for s in s_profile
    ]
    v_profile = _speed_profile_from_geometry_caps(
        s_profile,
        caps,
        max_tangent_accel_m_s2=limits.max_tangent_accel_m_s2,
        goal_decel_m_s2=goal_decel_m_s2,
        start_speed_m_s=start_speed_m_s,
    )
    return s_profile, v_profile


def speed_at_progress_m(
    progress_m: float, s_m: Sequence[float], v_m_s: Sequence[float]
) -> float:
    """Linearly interpolate profile speed at arc length ``progress_m``."""
    if not s_m:
        return 0.0
    if len(s_m) == 1:
        return float(v_m_s[0])

    s = float(np.clip(progress_m, s_m[0], s_m[-1]))
    if s <= s_m[0]:
        return float(v_m_s[0])
    if s >= s_m[-1]:
        return float(v_m_s[-1])

    idx = bisect.bisect_right(s_m, s) - 1
    idx = max(0, min(idx, len(s_m) - 2))
    s0, s1 = float(s_m[idx]), float(s_m[idx + 1])
    v0, v1 = float(v_m_s[idx]), float(v_m_s[idx + 1])
    if s1 <= s0:
        return v0
    u = (s - s0) / (s1 - s0)
    return v0 + (v1 - v0) * u


__all__ = [
    "PathSpeedProfileLimits",
    "profile_speed_along_polyline",
    "speed_at_progress_m",
]
