# Copyright 2025 Dimensional Inc.
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
