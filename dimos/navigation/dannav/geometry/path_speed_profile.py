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
and a forward-backward pass on arc length (``v^2 <= v_0^2 + 2 a Delta s``).

- **Straight segments:** cap is ``limits.max_speed_m_s``.
- **Corners:** cap is ``min(max_speed_m_s, sqrt(max_normal_accel_m_s2 * |R|))``
  from the circumradius of the local vertex triangle.

The live planner seeds the forward pass at cruise speed so replanned paths do not
force zero speed at the origin.
"""

from __future__ import annotations

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


def _vertex_progress_along_polyline_m(
    cumulative_segment_s_m: NDArray[np.float64],
) -> NDArray[np.float64]:
    """Arc length to each vertex; segment cumulatives are PathDistancer-style."""
    vertex_s = np.zeros(len(cumulative_segment_s_m) + 1, dtype=np.float64)
    vertex_s[1:] = cumulative_segment_s_m
    return vertex_s


def _polyline_geometry_speed_caps_m_s(
    path_xy: NDArray[np.float64],
    vertex_s_m: NDArray[np.float64],
    s_profile: Sequence[float],
    limits: PathSpeedProfileLimits,
) -> list[float]:
    """Geometry speed cap at every arc length in ``s_profile`` on a planar polyline.

    The cap is ``max_speed_m_s`` everywhere except at interior vertices, where a
    corner binds it via the centripetal bound ``v^2 / R <= a_n`` to
    ``sqrt(max_normal_accel_m_s2 * R)``. ``R = a b c / (2 * area)`` is the
    circumradius of the local vertex triangle. A degenerate triangle
    (near-collinear or a zero-length side) leaves the cap at ``max_speed_m_s``.

    Corner caps depend only on the fixed vertices, so they are computed once for
    all interior vertices and scattered onto the matching ``s_profile`` samples
    (each vertex arc length is itself a sample). Every other sample keeps the
    straight-segment cap ``max_speed_m_s``.
    """
    max_cap = float(limits.max_speed_m_s)
    s_arr = np.asarray(s_profile, dtype=np.float64)
    caps = np.full(s_arr.shape[0], max_cap, dtype=np.float64)
    if len(path_xy) < 3 or s_arr.shape[0] == 0:
        return caps.tolist()

    p0 = path_xy[:-2]
    p1 = path_xy[1:-1]
    p2 = path_xy[2:]
    side_a = np.linalg.norm(p1 - p0, axis=1)
    side_b = np.linalg.norm(p2 - p1, axis=1)
    side_c = np.linalg.norm(p2 - p0, axis=1)
    v10 = p1 - p0
    v20 = p2 - p0
    area2 = np.abs(v10[:, 0] * v20[:, 1] - v10[:, 1] * v20[:, 0])

    degenerate = np.minimum(np.minimum(side_a, side_b), np.minimum(side_c, area2)) <= 1e-9
    with np.errstate(divide="ignore", invalid="ignore"):
        radius = (side_a * side_b * side_c) / (2.0 * area2)
        v_curve = np.sqrt(limits.max_normal_accel_m_s2 * radius)
    corner = np.where(degenerate, max_cap, np.minimum(max_cap, v_curve))

    vertex_s = np.asarray(vertex_s_m, dtype=np.float64)
    interior_s = vertex_s[1:-1]
    local_scale = np.maximum(np.maximum(interior_s - vertex_s[:-2], vertex_s[2:] - interior_s), 1.0)
    tol = np.maximum(1e-9, local_scale * 1e-9)

    # Each interior vertex arc length is present as a sample, so match every
    # vertex to the nearest sample; the match is exact within ``tol``.
    m = s_arr.shape[0]
    idx = np.searchsorted(s_arr, interior_s, side="left")
    left = np.clip(idx - 1, 0, m - 1)
    right = np.clip(idx, 0, m - 1)
    dl = np.abs(s_arr[left] - interior_s)
    dr = np.abs(s_arr[right] - interior_s)
    nearest = np.where(dl <= dr, left, right)
    within = np.minimum(dl, dr) <= tol

    # Lowest vertex index wins a shared sample: assign high-to-low so index 0 is
    # written last (matches the original first-match-wins scan over vertices).
    order = np.nonzero(within)[0][::-1]
    caps[nearest[order]] = corner[order]
    return caps.tolist()


def _profile_sample_distances_m(
    total_length_m: float,
    vertex_s_m: NDArray[np.float64],
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

    return [min(float(geometry_cap_m_s[i]), v_forward[i], v_backward[i]) for i in range(len(s_m))]


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
    caps = _polyline_geometry_speed_caps_m_s(path_xy, vertex_s_m, s_profile, limits)
    v_profile = _speed_profile_from_geometry_caps(
        s_profile,
        caps,
        max_tangent_accel_m_s2=limits.max_tangent_accel_m_s2,
        goal_decel_m_s2=goal_decel_m_s2,
        start_speed_m_s=start_speed_m_s,
    )
    return s_profile, v_profile


def speed_at_progress_m(progress_m: float, s_m: Sequence[float], v_m_s: Sequence[float]) -> float:
    """Linearly interpolate profile speed at arc length ``progress_m``."""
    if not s_m:
        return 0.0
    return float(np.interp(progress_m, s_m, v_m_s))
