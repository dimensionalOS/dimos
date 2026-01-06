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

import numpy as np
from scipy import ndimage  # type: ignore[import-untyped]

from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid


def smooth_occupied(
    occupancy_grid: OccupancyGrid, min_neighbor_fraction: float = 0.4
) -> OccupancyGrid:
    """Smooth occupied zones by removing unsupported protrusions.

    Removes occupied cells that don't have sufficient neighboring occupied
    cells.

    Args:
        occupancy_grid: Input occupancy grid
        min_neighbor_fraction: Minimum fraction of 8-connected neighbors
            that must be occupied for a cell to remain occupied.
    Returns:
        New OccupancyGrid with smoothed occupied zones
    """
    grid_array = occupancy_grid.grid
    occupied_mask = grid_array >= CostValues.OCCUPIED

    # Count occupied neighbors for each cell (8-connectivity).
    kernel = np.array([[1, 1, 1], [1, 0, 1], [1, 1, 1]], dtype=np.uint8)
    neighbor_count = ndimage.convolve(
        occupied_mask.astype(np.uint8), kernel, mode="constant", cval=0
    )

    # Remove cells with too few occupied neighbors.
    min_neighbors = int(np.ceil(8 * min_neighbor_fraction))
    unsupported = occupied_mask & (neighbor_count < min_neighbors)

    result_grid = grid_array.copy()
    result_grid[unsupported] = CostValues.FREE

    return OccupancyGrid(
        grid=result_grid,
        resolution=occupancy_grid.resolution,
        origin=occupancy_grid.origin,
        frame_id=occupancy_grid.frame_id,
        ts=occupancy_grid.ts,
    )


def overlay_occupied(base: OccupancyGrid, overlay: OccupancyGrid) -> OccupancyGrid:
    """Overlay occupied zones from one grid onto another.

    Marks cells as occupied in the base grid wherever they are occupied
    in the overlay grid.

    Args:
        base: The base occupancy grid
        overlay: The grid whose occupied zones will be overlaid onto base
    Returns:
        New OccupancyGrid with combined occupied zones
    """
    if base.grid.shape != overlay.grid.shape:
        raise ValueError(
            f"Grid shapes must match: base {base.grid.shape} vs overlay {overlay.grid.shape}"
        )

    result_grid = base.grid.copy()
    overlay_occupied_mask = overlay.grid >= CostValues.OCCUPIED
    result_grid[overlay_occupied_mask] = CostValues.OCCUPIED

    return OccupancyGrid(
        grid=result_grid,
        resolution=base.resolution,
        origin=base.origin,
        frame_id=base.frame_id,
        ts=base.ts,
    )
