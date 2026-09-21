# Copyright 2026 Dimensional Inc.
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

import numpy as np
from numpy.typing import NDArray

from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid


def _circular_disk(radius_cells: int) -> NDArray[np.bool_]:
    y, x = np.ogrid[-radius_cells : radius_cells + 1, -radius_cells : radius_cells + 1]
    return np.asarray((x * x + y * y) <= radius_cells * radius_cells)


class VisitationHistory:
    """Tracks visited locations in world coordinates, independent of occupancy grid changes.

    When a new occupancy grid arrives, the visited mask is rebuilt from stored
    world-coordinate points.  To avoid unbounded growth, when the visited
    saturation reaches ``_saturation_threshold`` the oldest half of the stored
    points are discarded and the mask is rebuilt.
    """

    _saturation_threshold = 0.50
    _min_distance_m = 0.05

    def __init__(self, clearance_radius_m: float) -> None:
        self._points: list[tuple[float, float]] = []
        self._visited: NDArray[np.bool_] | None = None
        self._grid: OccupancyGrid | None = None
        self._clearance_radius_m = clearance_radius_m
        self._clearance_radius_cells: int = 0
        self._clearance_disk: NDArray[np.bool_] = np.ones((1, 1), dtype=bool)

    @property
    def visited(self) -> NDArray[np.bool_] | None:
        return self._visited

    @property
    def clearance_radius_cells(self) -> int:
        return self._clearance_radius_cells

    @property
    def clearance_disk(self) -> NDArray[np.bool_]:
        return self._clearance_disk

    def update_grid(self, grid: OccupancyGrid) -> None:
        self._grid = grid
        self._clearance_radius_cells = int(np.ceil(self._clearance_radius_m / grid.resolution))
        self._clearance_disk = _circular_disk(self._clearance_radius_cells)
        self._rebuild()

    def handle_odom(self, x: float, y: float) -> None:
        if self._points:
            lx, ly = self._points[-1]
            if (x - lx) ** 2 + (y - ly) ** 2 < self._min_distance_m**2:
                return
        self._points.append((x, y))
        if self._visited is None or self._grid is None:
            return
        self._stamp(x, y)
        if self.get_saturation() >= self._saturation_threshold:
            n = len(self._points)
            self._points = self._points[n // 2 :]
            self._rebuild()

    def get_saturation(self) -> float:
        grid = self._grid
        visited = self._visited
        if grid is None or visited is None:
            return 0.0
        free_mask = grid.grid == 0
        total = int(np.count_nonzero(free_mask))
        if total == 0:
            return 0.0
        visited_free = int(np.count_nonzero(visited & free_mask))
        return visited_free / total

    def reset(self) -> None:
        self._points.clear()
        self._visited = None
        self._grid = None

    def _rebuild(self) -> None:
        grid = self._grid
        if grid is None:
            return
        self._visited = np.zeros((grid.height, grid.width), dtype=bool)
        for x, y in self._points:
            self._stamp(x, y)

    def _stamp(self, x: float, y: float) -> None:
        grid = self._grid
        visited = self._visited
        if grid is None or visited is None:
            return
        r = self._clearance_radius_cells
        grid_pos = grid.world_to_grid((x, y))
        col, row = int(grid_pos.x), int(grid_pos.y)
        if row + r < 0 or row - r >= grid.height or col + r < 0 or col - r >= grid.width:
            return
        r_min = max(0, row - r)
        r_max = min(grid.height, row + r + 1)
        c_min = max(0, col - r)
        c_max = min(grid.width, col + r + 1)
        d_r_min = r_min - (row - r)
        d_r_max = d_r_min + (r_max - r_min)
        d_c_min = c_min - (col - r)
        d_c_max = d_c_min + (c_max - c_min)
        visited[r_min:r_max, c_min:c_max] |= self._clearance_disk[d_r_min:d_r_max, d_c_min:d_c_max]
