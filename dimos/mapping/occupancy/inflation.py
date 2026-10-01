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
import math

from dimos_generated.nav_msgs.msg import OccupancyGrid
import numpy as np
from scipy import ndimage

from dimos.msgs.occupancy import occupancy_extent, occupancy_view


def simple_inflate(occupancy_grid: OccupancyGrid, radius: float) -> OccupancyGrid:
    """Inflate obstacles by a given radius (binary inflation).
    Args:
        radius: Inflation radius in meters
    Returns:
        New OccupancyGrid with inflated obstacles
    """
    occupancy_extent(occupancy_grid)
    if not math.isfinite(radius) or radius < 0:
        raise ValueError("inflation radius must be finite and nonnegative")

    # Convert radius to grid cells
    cell_radius = int(np.ceil(radius / occupancy_grid.info.resolution))

    # Get grid as numpy array
    grid_array = occupancy_view(occupancy_grid)

    # Create circular kernel for binary inflation
    y, x = np.ogrid[-cell_radius : cell_radius + 1, -cell_radius : cell_radius + 1]
    kernel = (x**2 + y**2 <= cell_radius**2).astype(np.uint8)

    # Find occupied cells
    occupied_mask = grid_array >= 100

    # Binary inflation
    inflated = ndimage.binary_dilation(occupied_mask, structure=kernel)
    result_grid = grid_array.copy()
    result_grid[inflated] = 100

    result = deepcopy(occupancy_grid)
    result.data = result_grid.ravel()
    return result
