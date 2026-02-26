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

from typing import cast

import numpy as np
from numpy.typing import NDArray

from dimos.msgs.nav_msgs import Path


class PathDistancer:
    _lookahead_dist: float = 0.5
    _path: NDArray[np.float64]
    _cumulative_dists: NDArray[np.float64]

    def __init__(self, path: Path) -> None:
        self._path = np.array([[p.position.x, p.position.y] for p in path.poses])
        self._cumulative_dists = _make_cumulative_distance_array(self._path)

    def find_lookahead_point(self, start_idx: int) -> NDArray[np.float64]:
        """
        Given a path, and a precomputed array of cumulative distances, find the
        point which is `lookahead_dist` ahead of the current point.
        """

        if start_idx >= len(self._path) - 1:
            return cast("NDArray[np.float64]", self._path[-1])

        # Distance from path[0] to path[start_idx].
        base_dist = self._cumulative_dists[start_idx - 1] if start_idx > 0 else 0.0
        target_dist = base_dist + self._lookahead_dist

        # Binary search: cumulative_dists[i] = distance from path[0] to path[i+1]
        idx = int(np.searchsorted(self._cumulative_dists, target_dist))

        if idx >= len(self._cumulative_dists):
            return cast("NDArray[np.float64]", self._path[-1])

        # Interpolate within segment from path[idx] to path[idx+1].
        prev_cum_dist = self._cumulative_dists[idx - 1] if idx > 0 else 0.0
        segment_dist = self._cumulative_dists[idx] - prev_cum_dist
        remaining_dist = target_dist - prev_cum_dist

        if segment_dist > 0:
            t = remaining_dist / segment_dist
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
        """Find the index of the closest point on the path."""
        distances = np.linalg.norm(self._path - pos, axis=1)
        return int(np.argmin(distances))

    def find_adaptive_lookahead_point(
        self, start_idx: int, current_speed: float, min_lookahead: float = 0.3, max_lookahead: float = 2.0
    ) -> NDArray[np.float64]:
        """Find lookahead point with adaptive distance based on speed.
        
        Args:
            start_idx: Starting path index
            current_speed: Current robot speed (m/s)
            min_lookahead: Minimum lookahead distance (m)
            max_lookahead: Maximum lookahead distance (m)
            
        Returns:
            Lookahead point [x, y]
        """
        # Adaptive lookahead: faster = longer lookahead
        lookahead_gain = 0.5
        adaptive_dist = min_lookahead + lookahead_gain * current_speed
        adaptive_dist = np.clip(adaptive_dist, min_lookahead, max_lookahead)
        
        # Temporarily set lookahead distance
        original_lookahead = self._lookahead_dist
        self._lookahead_dist = adaptive_dist
        try:
            return self.find_lookahead_point(start_idx)
        finally:
            self._lookahead_dist = original_lookahead

    def get_curvature_at_index(self, index: int) -> float:
        """Get curvature at a specific path index.
        
        Args:
            index: Path point index
            
        Returns:
            Curvature (1/m) at that point
        """
        if len(self._path) < 3 or index < 0 or index >= len(self._path):
            return 0.0
        
        # Use three-point method
        if index == 0:
            p0, p1, p2 = self._path[0], self._path[1], self._path[2]
        elif index == len(self._path) - 1:
            p0, p1, p2 = (
                self._path[-3],
                self._path[-2],
                self._path[-1],
            )
        else:
            p0, p1, p2 = (
                self._path[index - 1],
                self._path[index],
                self._path[index + 1],
            )
        
        # Compute angle change
        v1 = p1 - p0
        v2 = p2 - p1
        
        norm1 = np.linalg.norm(v1)
        norm2 = np.linalg.norm(v2)
        
        if norm1 < 1e-6 or norm2 < 1e-6:
            return 0.0
        
        v1 = v1 / norm1
        v2 = v2 / norm2
        
        cos_angle = np.clip(np.dot(v1, v2), -1.0, 1.0)
        angle = np.arccos(cos_angle)
        
        # Curvature = angle / arc_length
        arc_length = (norm1 + norm2) / 2.0
        if arc_length > 1e-6:
            return float(abs(angle) / arc_length)
        
        return 0.0

    def get_signed_cross_track_error(self, pos: NDArray[np.float64]) -> float:
        """Signed lateral distance from robot to path (left positive, right negative).
        
        Args:
            pos: Robot position [x, y]
            
        Returns:
            Signed cross-track error (m). Positive = left of path, negative = right of path.
        """
        index = self.find_closest_point_index(pos)
        p = self._path[index]

        # Choose a path tangent at this index
        if index < len(self._path) - 1:
            t = self._path[index + 1] - p
        elif index > 0:
            t = p - self._path[index - 1]
        else:
            return 0.0

        norm_t = np.linalg.norm(t)
        if norm_t < 1e-6:
            return 0.0

        t = t / norm_t  # Normalize tangent
        v = pos - p  # Vector from path point to robot

        # 2D cross product z-component: t × v
        # Positive = robot left of path, negative = robot right of path
        cross_z = t[0] * v[1] - t[1] * v[0]
        return float(cross_z)


def _make_cumulative_distance_array(array: NDArray[np.float64]) -> NDArray[np.float64]:
    """
    For an array representing 2D points, create an array of all the distances
    between the points.
    """

    if len(array) < 2:
        return np.array([0.0])

    segments = array[1:] - array[:-1]
    segment_dists = np.linalg.norm(segments, axis=1)
    return np.cumsum(segment_dists)
