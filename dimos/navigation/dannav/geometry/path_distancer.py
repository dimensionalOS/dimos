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

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import cast

import numpy as np
from numpy.typing import NDArray

from dimos.msgs.nav_msgs.Path import Path


@dataclass(frozen=True)
class PolylineProjection:
    """Foot of a query point on a polyline, in the polyline (map) frame.

    - ``foot_xy``: closest point on the polyline to the query point.
    - ``segment_start_index``: index ``i`` of the segment ``[i, i + 1]`` holding
      the foot.
    - ``tangent_yaw``: heading of that segment, ``atan2(dy, dx)`` in radians.
    - ``s_along_path_m``: arc length from the polyline start to the foot.
    - ``signed_cross_track_m``: lateral offset from the foot to the query point,
      positive when the point is left of the segment direction.
    """

    foot_xy: tuple[float, float]
    segment_start_index: int
    tangent_yaw: float
    s_along_path_m: float
    signed_cross_track_m: float


def project_to_polyline(x: float, y: float, polyline_xy: NDArray[np.float64]) -> PolylineProjection:
    """Project a point onto a ``(N, 2)`` polyline in the map frame.

    Answers "given this fixed polyline, what arc-length s is the robot at,
    and how far off to the side," returning a PolylineProjection
    """
    if polyline_xy.ndim != 2 or polyline_xy.shape[1] != 2:
        raise ValueError("polyline_xy must have shape (N, 2)")
    n = polyline_xy.shape[0]
    if n == 0:
        raise ValueError("polyline_xy must be non-empty")
    if n == 1:
        fx = float(polyline_xy[0, 0])
        fy = float(polyline_xy[0, 1])
        lat = float(math.hypot(x - fx, y - fy))
        return PolylineProjection(
            (fx, fy),
            0,
            0.0,
            0.0,
            lat,
        )

    p = np.array([x, y], dtype=np.float64)
    starts = polyline_xy[:-1]
    seg_vecs = polyline_xy[1:] - starts
    seg_len2 = np.einsum("ij,ij->i", seg_vecs, seg_vecs)
    seg_lens = np.sqrt(seg_len2)

    prefix_s = np.zeros(n, dtype=np.float64)
    np.cumsum(seg_lens, out=prefix_s[1:])

    # Foot on each segment: projection parameter clipped to the segment, with
    # zero-length segments pinned to their start point (t=0).
    nondegenerate = seg_len2 > 1e-18
    t_params = np.zeros(n - 1, dtype=np.float64)
    t_params[nondegenerate] = (
        np.einsum("ij,ij->i", (p - starts)[nondegenerate], seg_vecs[nondegenerate])
        / seg_len2[nondegenerate]
    )
    np.clip(t_params, 0.0, 1.0, out=t_params)
    feet = starts + t_params[:, None] * seg_vecs
    d2 = np.einsum("ij,ij->i", p - feet, p - feet)

    best_seg = int(np.argmin(d2))
    best_foot = feet[best_seg]
    best_t = float(t_params[best_seg])
    best_seg_len = float(seg_lens[best_seg]) if nondegenerate[best_seg] else 0.0
    s_along = float(prefix_s[best_seg] + best_t * best_seg_len)

    a = polyline_xy[best_seg]
    b = polyline_xy[best_seg + 1]
    ab = b - a
    seg_len = float(np.linalg.norm(ab))
    if seg_len < 1e-9:
        yaw = 0.0
        tx, ty = 1.0, 0.0
    else:
        tx, ty = float(ab[0] / seg_len), float(ab[1] / seg_len)
        yaw = math.atan2(ab[1], ab[0])
    nx, ny = -ty, tx
    vx = x - float(best_foot[0])
    vy = y - float(best_foot[1])
    signed_ct = vx * nx + vy * ny
    return PolylineProjection(
        (float(best_foot[0]), float(best_foot[1])),
        best_seg,
        yaw,
        s_along,
        float(signed_ct),
    )


class PathDistancer:
    """Arc-length queries over a fixed ``Path`` polyline.

    Built once per path: ``_path`` holds the pose xy as an ``(N, 2)`` array and
    ``_cumulative_dists`` the cumulative arc length after each segment (length
    ``N - 1``, so ``_cumulative_dists[-1]`` is the total path length).
    ``_lookahead_dist`` is the fixed lookahead in metres used by the follower.

    Query methods run against that cached geometry: ``point_at_progress`` and
    ``yaw_at_progress`` map an arc length to a position and heading, ``project``
    maps a position to its foot on the path, and ``distance_to_goal`` gives the
    straight-line distance to the final pose.
    """

    _lookahead_dist: float = 0.5
    _path: NDArray[np.float64]
    _cumulative_dists: NDArray[np.float64]

    def __init__(self, path: Path) -> None:
        self._path = np.array([[p.position.x, p.position.y] for p in path.poses])
        self._cumulative_dists = _make_cumulative_distance_array(self._path)

    @property
    def lookahead_distance_m(self) -> float:
        return self._lookahead_dist

    @property
    def path_length_m(self) -> float:
        if len(self._path) < 2:
            return 0.0
        return float(self._cumulative_dists[-1])

    def point_at_progress(self, progress_m: float) -> NDArray[np.float64]:
        if len(self._path) == 0:
            raise ValueError("path must be non-empty")
        if len(self._path) == 1:
            return cast("NDArray[np.float64]", self._path[0].copy())

        s = float(np.clip(progress_m, 0.0, self.path_length_m))
        idx = int(np.searchsorted(self._cumulative_dists, s))
        if idx >= len(self._cumulative_dists):
            return cast("NDArray[np.float64]", self._path[-1].copy())

        prev_s = self._cumulative_dists[idx - 1] if idx > 0 else 0.0
        segment_s = self._cumulative_dists[idx] - prev_s
        if segment_s <= 1e-12:
            return cast("NDArray[np.float64]", self._path[idx].copy())
        alpha = (s - prev_s) / segment_s
        point = self._path[idx] + alpha * (self._path[idx + 1] - self._path[idx])
        return cast("NDArray[np.float64]", point)

    def yaw_at_progress(self, progress_m: float) -> float:
        if len(self._path) < 2:
            return 0.0
        s = float(np.clip(progress_m, 0.0, self.path_length_m))
        idx = int(np.searchsorted(self._cumulative_dists, s))
        idx = min(max(idx, 0), len(self._path) - 2)
        direction = self._path[idx + 1] - self._path[idx]
        return float(np.arctan2(direction[1], direction[0]))

    def project(self, pos: NDArray[np.float64]) -> PolylineProjection:
        return project_to_polyline(float(pos[0]), float(pos[1]), self._path)

    def distance_to_goal(self, current_pos: NDArray[np.float64]) -> float:
        return float(np.linalg.norm(self._path[-1] - current_pos))


def _make_cumulative_distance_array(array: NDArray[np.float64]) -> NDArray[np.float64]:
    """For a 2D point array, return cumulative arc length at each vertex."""
    if len(array) < 2:
        return np.array([0.0])

    segments = array[1:] - array[:-1]
    segment_dists = np.linalg.norm(segments, axis=1)
    return np.cumsum(segment_dists)
