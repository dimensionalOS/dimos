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

"""Path distance utilities for path-following control.

PathDistancer wraps a nav_msgs/Path and provides:
- Cumulative arc-length distances along the path.
- Closest-point and lookahead-point queries.
- Signed cross-track error for PID lateral correction.
- Local curvature estimation.
"""

from __future__ import annotations

from typing import cast

import numpy as np
from numpy.typing import NDArray

from dimos.msgs.nav_msgs import Path


class PathDistancer:
    """Geometric utilities over a discretised path."""

    _lookahead_dist: float = 0.5

    def __init__(self, path: Path) -> None:
        self._path = np.array([[p.position.x, p.position.y] for p in path.poses])
        self._cumulative_dists = _make_cumulative_distance_array(self._path)

    def find_lookahead_point(self, start_idx: int) -> NDArray[np.float64]:
        """Find the point that is ``_lookahead_dist`` ahead of *start_idx*."""
        if start_idx >= len(self._path) - 1:
            return cast("NDArray[np.float64]", self._path[-1])

        base_dist = self._cumulative_dists[start_idx - 1] if start_idx > 0 else 0.0
        target_dist = base_dist + self._lookahead_dist
        idx = int(np.searchsorted(self._cumulative_dists, target_dist))

        if idx >= len(self._cumulative_dists):
            return cast("NDArray[np.float64]", self._path[-1])

        prev_cum = self._cumulative_dists[idx - 1] if idx > 0 else 0.0
        seg = self._cumulative_dists[idx] - prev_cum
        rem = target_dist - prev_cum

        if seg > 0:
            t = rem / seg
            return cast(
                "NDArray[np.float64]",
                self._path[idx] + t * (self._path[idx + 1] - self._path[idx]),
            )
        return cast("NDArray[np.float64]", self._path[idx])

    def distance_to_goal(self, current_pos: NDArray[np.float64]) -> float:
        return float(np.linalg.norm(self._path[-1] - current_pos))

    def get_distance_to_path(self, pos: NDArray[np.float64]) -> float:
        index = self.find_closest_point_index(pos)
        return float(np.linalg.norm(self._path[index] - pos))

    def find_closest_point_index(self, pos: NDArray[np.float64]) -> int:
        distances = np.linalg.norm(self._path - pos, axis=1)
        return int(np.argmin(distances))

    def find_adaptive_lookahead_point(
        self,
        start_idx: int,
        current_speed: float,
        min_lookahead: float = 0.3,
        max_lookahead: float = 2.0,
    ) -> NDArray[np.float64]:
        """Adaptive lookahead: faster speed -> longer lookahead distance."""
        lookahead_gain = 0.5
        adaptive_dist = float(
            np.clip(
                min_lookahead + lookahead_gain * current_speed,
                min_lookahead,
                max_lookahead,
            )
        )
        original = self._lookahead_dist
        self._lookahead_dist = adaptive_dist
        try:
            return self.find_lookahead_point(start_idx)
        finally:
            self._lookahead_dist = original

    def get_curvature_at_index(self, index: int) -> float:
        """Three-point curvature estimate at *index*."""
        if len(self._path) < 3 or index < 0 or index >= len(self._path):
            return 0.0

        if index == 0:
            p0, p1, p2 = self._path[0], self._path[1], self._path[2]
        elif index == len(self._path) - 1:
            p0, p1, p2 = self._path[-3], self._path[-2], self._path[-1]
        else:
            p0, p1, p2 = self._path[index - 1], self._path[index], self._path[index + 1]

        v1, v2 = p1 - p0, p2 - p1
        n1, n2 = float(np.linalg.norm(v1)), float(np.linalg.norm(v2))
        if n1 < 1e-6 or n2 < 1e-6:
            return 0.0

        cos_a = float(np.clip(np.dot(v1 / n1, v2 / n2), -1.0, 1.0))
        angle = float(np.arccos(cos_a))
        arc = (n1 + n2) / 2.0
        return abs(angle) / arc if arc > 1e-6 else 0.0

    def get_signed_cross_track_error(self, pos: NDArray[np.float64]) -> float:
        """Signed lateral distance from path (left positive, right negative)."""
        index = self.find_closest_point_index(pos)
        p = self._path[index]

        if index < len(self._path) - 1:
            tangent = self._path[index + 1] - p
        elif index > 0:
            tangent = p - self._path[index - 1]
        else:
            return 0.0

        norm_t = float(np.linalg.norm(tangent))
        if norm_t < 1e-6:
            return 0.0

        tangent = tangent / norm_t
        v = pos - p
        return float(tangent[0] * v[1] - tangent[1] * v[0])


def _make_cumulative_distance_array(pts: NDArray[np.float64]) -> NDArray[np.float64]:
    if len(pts) < 2:
        return np.array([0.0])
    segments = pts[1:] - pts[:-1]
    return np.cumsum(np.linalg.norm(segments, axis=1))


__all__ = ["PathDistancer"]
