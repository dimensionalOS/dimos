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
from scipy.ndimage import binary_erosion

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.navigation.patrolling.routers.base_patrol_router import BasePatrolRouter
from dimos.navigation.patrolling.utilities import point_to_pose_stamped


class RandomPatrolRouter(BasePatrolRouter):
    def next_goal(self) -> PoseStamped | None:
        with self._lock:
            if self._occupancy_grid is None or self._visited is None:
                return None
            occupancy_grid = self._occupancy_grid
            visited = self._visited.copy()
        point = _random_empty_spot(
            occupancy_grid, clearance_m=self._clearance_radius_m, visited=visited
        )
        if point is None:
            return None
        return point_to_pose_stamped(point)


def _random_empty_spot(
    occupancy_grid: OccupancyGrid,
    clearance_m: float,
    visited: NDArray[np.bool_] | None = None,
) -> tuple[float, float] | None:
    clearance_cells = int(np.ceil(clearance_m / occupancy_grid.resolution))

    free_mask = occupancy_grid.grid == 0
    if not np.any(free_mask):
        return None

    # Erode the free mask by the clearance radius so only cells with full clearance remain.
    structure = np.ones((2 * clearance_cells + 1, 2 * clearance_cells + 1), dtype=bool)
    safe_mask = binary_erosion(free_mask, structure=structure)

    # Prefer unvisited cells; fall back to all safe cells if everything is visited.
    if visited is not None:
        unvisited_safe = safe_mask & ~visited
        if np.any(unvisited_safe):
            safe_mask = unvisited_safe

    safe_indices = np.argwhere(safe_mask)
    if len(safe_indices) == 0:
        return None

    idx = safe_indices[np.random.randint(len(safe_indices))]
    row, col = idx
    world = occupancy_grid.grid_to_world((col, row, 0))
    return (world.x, world.y)
