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

"""Map-based checks for in-place rotation clearance in holonomic local planning."""

from __future__ import annotations

import math

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid

PATH_CLEARANCE_OBSTACLE_COST_THRESHOLD = 50


def can_rotate_in_place(
    costmap: OccupancyGrid,
    pose: PoseStamped,
    rotation_radius_m: float,
    *,
    obstacle_cost_threshold: int = PATH_CLEARANCE_OBSTACLE_COST_THRESHOLD,
) -> bool:
    """Return True when a full in-place spin fits inside free map cells.

    Samples every grid cell whose center lies inside a circle of
    ``rotation_radius_m`` around ``pose``. Any unknown or occupied cell
    (cost >= ``obstacle_cost_threshold``) makes rotation unsafe.
    """
    if rotation_radius_m <= 0.0 or not math.isfinite(rotation_radius_m):
        return True

    grid_center = costmap.world_to_grid(pose.position)
    center_x = int(round(float(grid_center.x)))
    center_y = int(round(float(grid_center.y)))
    radius_cells = int(math.ceil(rotation_radius_m / costmap.resolution))
    radius_cells_sq = radius_cells * radius_cells

    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy > radius_cells_sq:
                continue
            cell_x = center_x + dx
            cell_y = center_y + dy
            if not (0 <= cell_x < costmap.width and 0 <= cell_y < costmap.height):
                return False
            cell_value = int(costmap.grid[cell_y, cell_x])
            if cell_value == CostValues.UNKNOWN or cell_value >= obstacle_cost_threshold:
                return False

    return True


__all__ = ["can_rotate_in_place"]
