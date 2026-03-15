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

from typing import Any

import numpy as np
from numpy.typing import NDArray
from scipy.ndimage import binary_erosion, label

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.navigation.patrolling.routers.base_patrol_router import BasePatrolRouter
from dimos.navigation.patrolling.utilities import point_to_pose_stamped


class FrontierPatrolRouter(BasePatrolRouter):
    """Patrol router that picks goals based on unvisited frontier clusters.

    This router:
      1. Finds connected components of unvisited safe cells.
      2. Scores each component by  size / euclidean_distance  from the robot.
      3. Within the best component, picks the point farthest from the robot
         to create long sweeping paths through unvisited territory.
    """

    _safe_mask: NDArray[np.bool_] | None
    _min_cluster_cells: int = 20

    def __init__(self, clearance_radius_m: float) -> None:
        super().__init__(clearance_radius_m)
        self._safe_mask = None

    def handle_occupancy_grid(self, msg: OccupancyGrid) -> None:
        with self._lock:
            prev = self._occupancy_grid
            super().handle_occupancy_grid(msg)
            if self._occupancy_grid is prev:
                return

            clearance_cells = self._visitation.clearance_radius_cells
            free_mask = msg.grid == 0
            structure = np.ones((2 * clearance_cells + 1, 2 * clearance_cells + 1), dtype=bool)
            self._safe_mask = binary_erosion(free_mask, structure=structure).astype(bool)

    def next_goal(self) -> PoseStamped | None:
        with self._lock:
            if self._occupancy_grid is None or self._visited is None or self._safe_mask is None:
                return None
            occupancy_grid = self._occupancy_grid
            safe_mask = self._safe_mask
            visited = self._visited.copy()
            pose = self._pose

        if pose is None:
            return None

        # Robot position in grid coordinates.
        grid_pos = occupancy_grid.world_to_grid((pose.position.x, pose.position.y))
        robot_col, robot_row = grid_pos.x, grid_pos.y

        # Unvisited safe cells.
        unvisited_safe = safe_mask & ~visited
        if not np.any(unvisited_safe):
            unvisited_safe = safe_mask
        if not np.any(unvisited_safe):
            return None

        # Find connected components of unvisited safe space.
        labeled, n_components = label(unvisited_safe)
        if n_components == 0:
            return None

        # Compute size and centroid of each component using vectorized ops.
        component_ids = np.arange(1, n_components + 1)
        rows, cols = np.where(labeled > 0)
        labels_flat = labeled[rows, cols]

        # Size and centroid of each component.
        sizes = np.bincount(labels_flat, minlength=n_components + 1)[1:]
        sum_rows = np.bincount(labels_flat, weights=rows, minlength=n_components + 1)[1:]
        sum_cols = np.bincount(labels_flat, weights=cols, minlength=n_components + 1)[1:]

        # Filter out tiny clusters.
        valid = sizes >= self._min_cluster_cells
        if not np.any(valid):
            valid = sizes > 0

        valid_ids = component_ids[valid]
        valid_sizes = sizes[valid].astype(np.float64)

        # Euclidean distance from robot to each cluster centroid.
        centroid_rows = sum_rows[valid] / valid_sizes
        centroid_cols = sum_cols[valid] / valid_sizes
        dr = centroid_rows - robot_row
        dc = centroid_cols - robot_col
        distances = np.maximum(np.sqrt(dr * dr + dc * dc), 1.0)

        # Score: prefer large, nearby clusters.
        scores = valid_sizes / distances

        best_idx = int(np.argmax(scores))

        # Within the best cluster, pick the point farthest from the robot.
        # This creates long sweeping paths through unvisited territory instead
        # of tiny movements toward a barely-shifting centroid.
        cluster_mask = labeled == valid_ids[best_idx]
        cluster_indices = np.argwhere(cluster_mask)
        cluster_dr: NDArray[np.floating[Any]] = cluster_indices[:, 0] - robot_row
        cluster_dc: NDArray[np.floating[Any]] = cluster_indices[:, 1] - robot_col
        dists_sq = cluster_dr * cluster_dr + cluster_dc * cluster_dc
        goal_row, goal_col = cluster_indices[np.argmax(dists_sq)]

        world = occupancy_grid.grid_to_world((int(goal_col), int(goal_row), 0))
        return point_to_pose_stamped((world.x, world.y))
