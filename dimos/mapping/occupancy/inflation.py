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

import numpy as np
from scipy import ndimage

from dimos.mapping.occupancy.operations import _grid_values, _GridT, _with_grid
from dimos.message_utils.occupancy import CostValues


def simple_inflate(occupancy_grid: _GridT, radius: float) -> _GridT:
    """Inflate obstacles by a given radius (binary inflation).
    Args:
        radius: Inflation radius in meters
    Returns:
        New OccupancyGrid with inflated obstacles
    """
    # Convert radius to grid cells
    cell_radius = int(np.ceil(radius / occupancy_grid.info.resolution))

    # Get grid as numpy array
    grid_array = _grid_values(occupancy_grid)

    # Create circular kernel for binary inflation
    y, x = np.ogrid[-cell_radius : cell_radius + 1, -cell_radius : cell_radius + 1]
    kernel = (x**2 + y**2 <= cell_radius**2).astype(np.uint8)

    # Find occupied cells
    occupied_mask = grid_array >= CostValues.OCCUPIED

    # Binary inflation
    inflated = ndimage.binary_dilation(occupied_mask, structure=kernel)
    result_grid = grid_array.copy()
    result_grid[inflated] = CostValues.OCCUPIED

    # Create new OccupancyGrid with inflated data using numpy constructor
    return _with_grid(occupancy_grid, result_grid)
