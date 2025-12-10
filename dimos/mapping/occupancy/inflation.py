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


def simple_inflate(occupancy_grid: OccupancyGrid, radius: float) -> OccupancyGrid:
    """Inflate obstacles by a given radius (binary inflation).
    Args:
        radius: Inflation radius in meters
    Returns:
        New OccupancyGrid with inflated obstacles
    """
    # Convert radius to grid cells
    cell_radius = int(np.ceil(radius / occupancy_grid.resolution))

    # Get grid as numpy array
    grid_array = occupancy_grid.grid

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
    return OccupancyGrid(
        grid=result_grid,
        resolution=occupancy_grid.resolution,
        origin=occupancy_grid.origin,
        frame_id=occupancy_grid.frame_id,
        ts=occupancy_grid.ts,
    )


def voronoi_inflate(occupancy_grid: OccupancyGrid, radius: float) -> OccupancyGrid:
    """Inflate obstacles by a given radius using Voronoi partitioning.

    Unlike simple_inflate, this function ensures that separate occupied zones
    do not merge together during inflation. Each zone grows into free space
    up to the specified radius, stopping at Voronoi boundaries to maintain
    separation between distinct obstacles.

    Args:
        occupancy_grid: Input occupancy grid
        radius: Inflation radius in meters
    Returns:
        New OccupancyGrid with inflated obstacles (zones remain separate)
    """
    # Convert radius to grid cells
    cell_radius = radius / occupancy_grid.resolution

    grid_array = occupancy_grid.grid
    occupied_mask = grid_array >= CostValues.OCCUPIED

    # Label connected occupied regions (8-connectivity).
    structure_8conn = np.ones((3, 3), dtype=np.uint8)
    labeled_zones, num_zones = ndimage.label(occupied_mask, structure=structure_8conn)

    if num_zones == 0:
        return OccupancyGrid(
            grid=grid_array.copy(),
            resolution=occupancy_grid.resolution,
            origin=occupancy_grid.origin,
            frame_id=occupancy_grid.frame_id,
            ts=occupancy_grid.ts,
        )

    # Compute distance from each free pixel to nearest occupied pixel
    # and get indices of which occupied pixel is nearest.
    distances, nearest_indices = ndimage.distance_transform_edt(~occupied_mask, return_indices=True)

    # Voronoi assignment: for each pixel, which zone is it closest to.
    voronoi_labels = labeled_zones[nearest_indices[0], nearest_indices[1]]

    # Find Voronoi boundaries: pixels adjacent to a pixel with a different
    # label. This creates a 1-pixel boundary between zones.
    boundary = np.zeros_like(voronoi_labels, dtype=bool)
    boundary[:-1, :] |= voronoi_labels[:-1, :] != voronoi_labels[1:, :]
    boundary[1:, :] |= voronoi_labels[1:, :] != voronoi_labels[:-1, :]
    boundary[:, :-1] |= voronoi_labels[:, :-1] != voronoi_labels[:, 1:]
    boundary[:, 1:] |= voronoi_labels[:, 1:] != voronoi_labels[:, :-1]

    # Inflate: mark cells within radius that are not on Voronoi boundaries.
    result_grid = grid_array.copy()
    inflate_mask = (distances <= cell_radius) & (distances > 0) & ~boundary
    result_grid[inflate_mask] = CostValues.OCCUPIED

    return OccupancyGrid(
        grid=result_grid,
        resolution=occupancy_grid.resolution,
        origin=occupancy_grid.origin,
        frame_id=occupancy_grid.frame_id,
        ts=occupancy_grid.ts,
    )
