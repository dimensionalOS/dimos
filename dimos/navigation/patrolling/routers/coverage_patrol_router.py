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

from dimos.mapping.occupancy.gradient import gradient, voronoi_gradient
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.patrolling.routers.base_patrol_router import BasePatrolRouter
from dimos.navigation.patrolling.utilities import point_to_pose_stamped
from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar


class CoveragePatrolRouter(BasePatrolRouter):
    _costmap: OccupancyGrid | None
    _safe_mask: NDArray[np.bool_] | None
    _sampling_weights: NDArray[np.float64] | None
    _candidates_to_consider: int = 7

    def __init__(self, clearance_radius_m: float) -> None:
        super().__init__(clearance_radius_m)
        self._costmap = None
        self._safe_mask = None
        self._sampling_weights = None

    def handle_occupancy_grid(self, msg: OccupancyGrid) -> None:
        with self._lock:
            prev = self._occupancy_grid
            super().handle_occupancy_grid(msg)
            if self._occupancy_grid is prev:
                # Throttled — no update happened.
                return
            self._costmap = gradient(msg, max_distance=1.5)

            # Precompute the safe mask (cells with enough clearance from obstacles).
            clearance_cells = self._visitation.clearance_radius_cells
            free_mask = msg.grid == 0
            structure = np.ones((2 * clearance_cells + 1, 2 * clearance_cells + 1), dtype=bool)
            self._safe_mask = binary_erosion(free_mask, structure=structure).astype(bool)

            # Precompute voronoi-based sampling weights so candidates are spread
            # across different corridors/regions rather than clustering in large
            # open areas.  Low voronoi cost = on the skeleton (equidistant from
            # walls) = high sampling weight.
            voronoi = voronoi_gradient(msg, max_distance=1.5)
            voronoi_cost = voronoi.grid.astype(np.float64)
            # Invert: skeleton cells (cost 0) become weight 100, walls (100) become 0.
            # Clamp negatives (unknown = -1) to 0.
            weights = np.clip(100.0 - voronoi_cost, 0.0, 100.0)
            self._sampling_weights = weights

    def next_goal(self) -> PoseStamped | None:
        with self._lock:
            if (
                self._occupancy_grid is None
                or self._visited is None
                or self._safe_mask is None
                or self._costmap is None
                or self._sampling_weights is None
            ):
                return None
            occupancy_grid = self._occupancy_grid
            costmap = self._costmap
            safe_mask = self._safe_mask
            sampling_weights = self._sampling_weights
            visited = self._visited.copy()
            pose = self._pose

        if pose is None:
            return None

        start = (pose.position.x, pose.position.y)

        # Get candidate points from unvisited safe cells.
        unvisited_safe = safe_mask & ~visited
        if not np.any(unvisited_safe):
            # Fall back to all safe cells if everything visited.
            unvisited_safe = safe_mask
        if not np.any(unvisited_safe):
            return None

        safe_indices = np.argwhere(unvisited_safe)
        n_candidates = min(self._candidates_to_consider, len(safe_indices))

        # Weight candidates by voronoi score so they spread across corridors
        # rather than clustering in large open areas.
        weights = sampling_weights[safe_indices[:, 0], safe_indices[:, 1]]
        weight_sum = weights.sum()
        if weight_sum > 0:
            probs = weights / weight_sum
        else:
            probs = None
        chosen = safe_indices[
            np.random.choice(len(safe_indices), size=n_candidates, replace=False, p=probs)
        ]

        best_score = -1
        best_point = None

        for row, col in chosen:
            world = occupancy_grid.grid_to_world((col, row, 0))
            candidate = (world.x, world.y)

            path = min_cost_astar(costmap, candidate, start, unknown_penalty=1.0, use_cpp=True)
            if path is None:
                continue

            # Count how many new (unvisited) cells would be covered along this path.
            new_cells = self._count_new_coverage(path, visited, occupancy_grid, safe_mask)
            if new_cells > best_score:
                best_score = new_cells
                best_point = candidate

        if best_point is None:
            return None
        return point_to_pose_stamped(best_point)

    def _count_new_coverage(
        self,
        path: Path,
        visited: NDArray[np.bool_],
        occupancy_grid: OccupancyGrid,
        safe_mask: NDArray[np.bool_],
    ) -> int:
        r = self._visitation.clearance_radius_cells
        h, w = visited.shape
        covered = np.zeros_like(visited)

        # Sample every few poses to avoid redundant work on dense paths.
        step = max(1, r)
        poses = path.poses[::step]

        for pose in poses:
            grid = occupancy_grid.world_to_grid((pose.position.x, pose.position.y))
            col, row = int(grid.x), int(grid.y)
            r_min = max(0, row - r)
            r_max = min(h, row + r + 1)
            c_min = max(0, col - r)
            c_max = min(w, col + r + 1)
            d_r_min = r_min - (row - r)
            d_r_max = d_r_min + (r_max - r_min)
            d_c_min = c_min - (col - r)
            d_c_max = d_c_min + (c_max - c_min)
            covered[r_min:r_max, c_min:c_max] |= self._visitation.clearance_disk[
                d_r_min:d_r_max, d_c_min:d_c_max
            ]

        # New coverage = cells in covered that are not yet visited and are free space.
        new = covered & ~visited & safe_mask
        return int(np.count_nonzero(new))
