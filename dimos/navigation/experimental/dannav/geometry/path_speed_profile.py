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

"""Speed along a planar polyline, per vertex.

Speed is capped by the cruise speed, the centripetal limit ``sqrt(a_n / kappa)`` and
the yaw-rate limit ``w / kappa``, so a sharp corner is taken nearly at rest, then
bounded by the tangent acceleration from the start (``v^2 <= v_0^2 + 2 a Delta s``)
and the goal deceleration into the end.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True)
class PathSpeedProfileLimits:
    """Scalar limits for profiling speed along one planar path."""

    max_speed_m_s: float
    max_tangent_accel_m_s2: float
    max_normal_accel_m_s2: float
    max_yaw_rate_rad_s: float


def profile_speed(
    s: NDArray[np.float64],
    curvature: NDArray[np.float64],
    limits: PathSpeedProfileLimits,
    goal_decel_m_s2: float,
) -> NDArray[np.float64]:
    """Speed at every vertex, reaching zero at the last one."""
    k = np.abs(curvature) + 1e-9
    v: NDArray[np.float64] = np.minimum.reduce(
        [
            np.full_like(s, limits.max_speed_m_s),
            np.sqrt(limits.max_normal_accel_m_s2 / k),
            limits.max_yaw_rate_rad_s / k,
        ]
    )
    for i in range(1, len(s)):
        reach = v[i - 1] ** 2 + 2.0 * limits.max_tangent_accel_m_s2 * (s[i] - s[i - 1])
        v[i] = min(v[i], math.sqrt(reach))
    v[-1] = 0.0
    for i in range(len(s) - 2, -1, -1):
        reach = v[i + 1] ** 2 + 2.0 * goal_decel_m_s2 * (s[i + 1] - s[i])
        v[i] = min(v[i], math.sqrt(reach))
    return v
