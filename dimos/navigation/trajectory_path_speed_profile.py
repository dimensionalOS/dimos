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

"""Speed vs arc length for planar path segments (P3-1).

Computes a **rest-to-rest** scalar speed profile along a single segment of
length ``length_m`` using a constant tangential acceleration limit and a
constant geometric speed cap (from ``max_speed_m_s`` and curvature).

- **Line:** cap is ``limits.max_speed_m_s`` (no centripetal binding).
- **Circular arc:** cap is ``min(max_speed_m_s, sqrt(max_normal_accel_m_s2 * |R|))``
  so that ``v^2 / |R| <= max_normal_accel_m_s2``.

The profile is built with a forward-backward pass on arc length (classic
``v^2 <= v_0^2 + 2 a \\Delta s`` envelope), then ``min`` of the two passes, so
tangential acceleration never exceeds ``max_tangent_accel_m_s2`` in the
discrete sense.

Time along the sample grid is obtained by integrating ``dt \\approx ds`` using
trapezoidal ``v`` between samples (see ``time_s_from_speed_profile``).
"""

from __future__ import annotations

from dataclasses import dataclass
import math


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


def circular_arc_geometry_speed_cap_m_s(abs_radius_m: float, limits: PathSpeedProfileLimits) -> float:
    """Upper speed from centripetal bound ``v^2 / R <= a_n`` and ``max_speed_m_s``."""
    if not math.isfinite(abs_radius_m) or abs_radius_m <= 0.0:
        raise ValueError(f"abs_radius_m must be finite and positive, got {abs_radius_m!r}")
    v_curve = math.sqrt(limits.max_normal_accel_m_s2 * abs_radius_m)
    return min(limits.max_speed_m_s, v_curve)


def line_segment_geometry_speed_cap_m_s(limits: PathSpeedProfileLimits) -> float:
    """Geometric cap for a straight segment (no curvature binding)."""
    return float(limits.max_speed_m_s)


def profile_speed_along_segment(
    length_m: float,
    geometry_speed_cap_m_s: float,
    limits: PathSpeedProfileLimits,
    *,
    num_samples: int,
) -> tuple[list[float], list[float]]:
    """Rest-to-rest speed ``v(s)`` on ``s in [0, length_m]``.

    Parameters
    ----------
    length_m
        Total arc length of the segment. Non-positive length yields ``([0.0], [0.0])``.
    geometry_speed_cap_m_s
        Constant upper bound on speed from geometry (line or arc); already merged
        with path-specific rules if needed. Must be positive and finite for positive
        ``length_m``; values above ``limits.max_speed_m_s`` are clamped.
    limits
        Tangential acceleration and global speed cap.
    num_samples
        Number of uniformly spaced samples along ``s`` (at least 2 for positive length).

    Returns
    -------
    s_m, v_m_s
        Monotonic ``s`` and corresponding speeds, same length.
    """
    if length_m <= 0.0:
        return [0.0], [0.0]
    if num_samples < 2:
        raise ValueError(f"num_samples must be at least 2, got {num_samples}")
    if not math.isfinite(geometry_speed_cap_m_s) or geometry_speed_cap_m_s <= 0.0:
        raise ValueError(
            "geometry_speed_cap_m_s must be finite and positive for a positive-length segment, "
            f"got {geometry_speed_cap_m_s!r}"
        )

    v_cap = min(limits.max_speed_m_s, geometry_speed_cap_m_s)
    n = num_samples
    ds = length_m / float(n - 1)
    a = limits.max_tangent_accel_m_s2

    v_forward = [0.0] * n
    for i in range(n - 1):
        v_next_sq = v_forward[i] * v_forward[i] + 2.0 * a * ds
        v_forward[i + 1] = min(v_cap, math.sqrt(max(0.0, v_next_sq)))

    v_backward = [0.0] * n
    for i in range(n - 1, 0, -1):
        v_prev_sq = v_backward[i] * v_backward[i] + 2.0 * a * ds
        v_backward[i - 1] = min(v_cap, math.sqrt(max(0.0, v_prev_sq)))

    v_out = [min(v_forward[i], v_backward[i]) for i in range(n)]
    s_out = [i * ds for i in range(n)]
    return s_out, v_out


def time_s_from_speed_profile(s_m: list[float], v_m_s: list[float]) -> list[float]:
    """Cumulative time at each ``s`` sample using trapezoidal ``ds / v`` segments."""
    if len(s_m) != len(v_m_s):
        raise ValueError("s_m and v_m_s must have the same length")
    if not s_m:
        return []
    t = [0.0] * len(s_m)
    eps = 1e-9
    for i in range(len(s_m) - 1):
        ds = s_m[i + 1] - s_m[i]
        v0, v1 = v_m_s[i], v_m_s[i + 1]
        denom = v0 + v1
        if denom > eps:
            dt = 2.0 * ds / denom
        else:
            # Degenerate: both speeds ~0; treat as zero motion over this interval
            dt = 0.0
        t[i + 1] = t[i] + dt
    return t


__all__ = [
    "PathSpeedProfileLimits",
    "circular_arc_geometry_speed_cap_m_s",
    "line_segment_geometry_speed_cap_m_s",
    "profile_speed_along_segment",
    "time_s_from_speed_profile",
]
