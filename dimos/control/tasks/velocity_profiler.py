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

"""Curvature-aware velocity profiler for path-following control.

Computes a speed limit at each waypoint by:
1. Estimating local curvature via three-point heading change.
2. Limiting speed from centripetal acceleration: v_max = sqrt(a_max / kappa).
3. Forward pass: enforce acceleration constraint.
4. Backward pass: enforce deceleration constraint.
"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from dimos.msgs.nav_msgs import Path


class VelocityProfiler:
    """Compute an optimal speed profile along a path."""

    def __init__(
        self,
        max_linear_speed: float = 0.8,
        max_angular_speed: float = 1.5,
        max_linear_accel: float = 1.0,
        max_linear_decel: float = 2.0,
        max_centripetal_accel: float = 1.0,
        min_speed: float = 0.05,
    ) -> None:
        self._max_linear_speed = max_linear_speed
        self._max_angular_speed = max_angular_speed
        self._max_linear_accel = max_linear_accel
        self._max_linear_decel = max_linear_decel
        self._max_centripetal_accel = max_centripetal_accel
        self._min_speed = min_speed

        self._cached_path_id: int | None = None
        self._cached_profile: NDArray[np.float64] | None = None

    def compute_profile(self, path: Path) -> NDArray[np.float64]:
        """Compute velocity profile for entire path.

        Returns:
            Array of speed limits (m/s) per waypoint.
        """
        if len(path.poses) < 2:
            return np.array([self._min_speed])

        pts = np.array([[p.position.x, p.position.y] for p in path.poses])
        curvatures = self._compute_curvatures(pts)
        max_speeds = self._curvature_speed_limits(curvatures)
        velocities = self._acceleration_pass(pts, max_speeds, forward=True)
        velocities = self._acceleration_pass(pts, velocities, forward=False)
        return np.maximum(velocities, self._min_speed)

    def get_velocity_at_index(self, path: Path, index: int) -> float:
        """Get cached velocity at a specific path index."""
        path_id = id(path)
        if self._cached_path_id != path_id or self._cached_profile is None:
            self._cached_profile = self.compute_profile(path)
            self._cached_path_id = path_id
        idx = min(max(0, index), len(self._cached_profile) - 1)
        return float(self._cached_profile[idx])

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _compute_curvatures(self, pts: NDArray[np.float64]) -> NDArray[np.float64]:
        n = len(pts)
        if n < 3:
            return np.zeros(n)

        curvatures = np.zeros(n)

        # First point
        ds1 = float(np.linalg.norm(pts[1] - pts[0]))
        if n > 2:
            ds2 = float(np.linalg.norm(pts[2] - pts[1]))
            dtheta = self._angle_between(pts[0], pts[1], pts[2])
            if ds1 + ds2 > 1e-6:
                curvatures[0] = abs(dtheta) / (ds1 + ds2)

        # Middle points
        for i in range(1, n - 1):
            d1 = float(np.linalg.norm(pts[i] - pts[i - 1]))
            d2 = float(np.linalg.norm(pts[i + 1] - pts[i]))
            dtheta = self._angle_between(pts[i - 1], pts[i], pts[i + 1])
            if d1 + d2 > 1e-6:
                curvatures[i] = abs(dtheta) / (d1 + d2)

        # Last point
        if n > 2:
            ds1 = float(np.linalg.norm(pts[-1] - pts[-2]))
            ds2 = float(np.linalg.norm(pts[-2] - pts[-3]))
            dtheta = self._angle_between(pts[-3], pts[-2], pts[-1])
            if ds1 + ds2 > 1e-6:
                curvatures[-1] = abs(dtheta) / (ds1 + ds2)

        return curvatures

    @staticmethod
    def _angle_between(
        p0: NDArray[np.float64], p1: NDArray[np.float64], p2: NDArray[np.float64]
    ) -> float:
        v1 = p1 - p0
        v2 = p2 - p1
        n1 = float(np.linalg.norm(v1))
        n2 = float(np.linalg.norm(v2))
        if n1 < 1e-6 or n2 < 1e-6:
            return 0.0
        cos_a = float(np.clip(np.dot(v1 / n1, v2 / n2), -1.0, 1.0))
        angle = float(np.arccos(cos_a))
        cross = v1[0] * v2[1] - v1[1] * v2[0]
        return -angle if cross < 0 else angle

    def _curvature_speed_limits(self, curvatures: NDArray[np.float64]) -> NDArray[np.float64]:
        limits = np.full(len(curvatures), self._max_linear_speed)
        mask = curvatures > 1e-6
        if np.any(mask):
            limits[mask] = np.minimum(
                limits[mask],
                np.sqrt(self._max_centripetal_accel / curvatures[mask]),
            )
        return limits

    def _acceleration_pass(
        self,
        pts: NDArray[np.float64],
        max_speeds: NDArray[np.float64],
        forward: bool,
    ) -> NDArray[np.float64]:
        v = max_speeds.copy()
        a = self._max_linear_accel if forward else self._max_linear_decel
        rng = range(1, len(pts)) if forward else range(len(pts) - 2, -1, -1)
        for i in rng:
            j = i - 1 if forward else i + 1
            ds = float(np.linalg.norm(pts[i] - pts[j]))
            if ds > 1e-6:
                v[i] = min(v[i], float(np.sqrt(v[j] ** 2 + 2 * a * ds)))
        return v


__all__ = ["VelocityProfiler"]
