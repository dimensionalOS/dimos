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

from copy import deepcopy
from typing import TypeVar

import numpy as np
from numpy.typing import NDArray
from scipy import ndimage

from dimos.messages.nav_msgs.msg import OccupancyGrid as CdrOccupancyGrid
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.occupancy import CostValues, occupancy_view

_GridT = TypeVar("_GridT", OccupancyGrid, CdrOccupancyGrid)


def _grid_values(message: _GridT) -> NDArray[np.int8]:
    if isinstance(message, CdrOccupancyGrid):
        return occupancy_view(message)
    return message.grid


def _with_grid(message: _GridT, cells: NDArray[np.int8]) -> _GridT:
    if isinstance(message, CdrOccupancyGrid):
        if cells.shape != (message.info.height, message.info.width):
            raise ValueError("Replacement occupancy cells must retain map dimensions")
        result = deepcopy(message)
        result.data = np.array(cells, dtype=np.int8, copy=True).reshape(-1)
        return result
    return OccupancyGrid(
        grid=cells,
        resolution=message.resolution,
        origin=message.origin,
        frame_id=message.frame_id,
        ts=message.ts,
    )


def smooth_occupied(occupancy_grid: _GridT, min_neighbor_fraction: float = 0.4) -> _GridT:
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
    grid_array = _grid_values(occupancy_grid)
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

    return _with_grid(occupancy_grid, result_grid)


def overlay_occupied(base: _GridT, overlay: _GridT) -> _GridT:
    """Overlay occupied zones from one grid onto another.

    Marks cells as occupied in the base grid wherever they are occupied
    in the overlay grid.

    Args:
        base: The base occupancy grid
        overlay: The grid whose occupied zones will be overlaid onto base
    Returns:
        New OccupancyGrid with combined occupied zones
    """
    if _grid_values(base).shape != _grid_values(overlay).shape:
        raise ValueError(
            f"Grid shapes must match: base {_grid_values(base).shape} vs overlay {_grid_values(overlay).shape}"
        )

    result_grid = _grid_values(base).copy()
    overlay_occupied_mask = _grid_values(overlay) >= CostValues.OCCUPIED
    result_grid[overlay_occupied_mask] = CostValues.OCCUPIED

    return _with_grid(base, result_grid)
