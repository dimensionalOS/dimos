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


def gradient(
    occupancy_grid: OccupancyGrid, obstacle_threshold: int = 50, max_distance: float = 2.0
) -> OccupancyGrid:
    """Create a gradient OccupancyGrid for path planning.

    Creates a gradient where free space has value 0 and values increase near obstacles.
    This can be used as a cost map for path planning algorithms like A*.

    Args:
        obstacle_threshold: Cell values >= this are considered obstacles (default: 50)
        max_distance: Maximum distance to compute gradient in meters (default: 2.0)

    Returns:
        New OccupancyGrid with gradient values:
        - -1: Unknown cells (preserved as-is)
        - 0: Free space far from obstacles
        - 1-99: Increasing cost as you approach obstacles
        - 100: At obstacles

    Note: Unknown cells remain as unknown (-1) and do not receive gradient values.
    """

    # Remember which cells are unknown
    unknown_mask = occupancy_grid.grid == CostValues.UNKNOWN

    # Create binary obstacle map
    # Consider cells >= threshold as obstacles (1), everything else as free (0)
    # Unknown cells are not considered obstacles for distance calculation
    obstacle_map = (occupancy_grid.grid >= obstacle_threshold).astype(np.float32)

    # Compute distance transform (distance to nearest obstacle in cells)
    # Unknown cells are treated as if they don't exist for distance calculation
    distance_cells = ndimage.distance_transform_edt(1 - obstacle_map)

    # Convert to meters and clip to max distance
    distance_meters = np.clip(distance_cells * occupancy_grid.resolution, 0, max_distance)  # type: ignore[operator]

    # Invert and scale to 0-100 range
    # Far from obstacles (max_distance) -> 0
    # At obstacles (0 distance) -> 100
    gradient_values = (1 - distance_meters / max_distance) * 100

    # Ensure obstacles are exactly 100
    gradient_values[obstacle_map > 0] = CostValues.OCCUPIED

    # Convert to int8 for OccupancyGrid
    gradient_data = gradient_values.astype(np.int8)

    # Preserve unknown cells as unknown (don't apply gradient to them)
    gradient_data[unknown_mask] = CostValues.UNKNOWN

    # Create new OccupancyGrid with gradient
    gradient_grid = OccupancyGrid(
        grid=gradient_data,
        resolution=occupancy_grid.resolution,
        origin=occupancy_grid.origin,
        frame_id=occupancy_grid.frame_id,
        ts=occupancy_grid.ts,
    )

    return gradient_grid


def voronoi_gradient(
    occupancy_grid: OccupancyGrid, obstacle_threshold: int = 50, max_distance: float = 2.0
) -> OccupancyGrid:
    """Create a Voronoi-based gradient OccupancyGrid for path planning.

    Unlike the regular gradient which can result in suboptimal paths in narrow
    corridors (where the center still has high cost), this method creates a cost
    map based on the Voronoi diagram of obstacles. Cells on Voronoi edges
    (equidistant from multiple obstacles) have minimum cost, encouraging paths
    that stay maximally far from all obstacles.

    For a corridor of width 10 cells:
    - Regular gradient: center cells might be 95 (still high cost)
    - Voronoi gradient: center cells are 0 (optimal path)

    The cost is interpolated based on relative position between the nearest
    obstacle and the nearest Voronoi edge:
    - At obstacle: cost = 100
    - At Voronoi edge: cost = 0
    - In between: cost = 99 * d_voronoi / (d_obstacle + d_voronoi)

    Args:
        obstacle_threshold: Cell values >= this are considered obstacles (default: 50)
        max_distance: Maximum distance in meters beyond which cost is 0 (default: 2.0)

    Returns:
        New OccupancyGrid with gradient values:
        - -1: Unknown cells (preserved as-is)
        - 0: On Voronoi edges (equidistant from obstacles) or far from obstacles
        - 1-99: Increasing cost closer to obstacles
        - 100: At obstacles
    """
    # Remember which cells are unknown
    unknown_mask = occupancy_grid.grid == CostValues.UNKNOWN

    # Create binary obstacle map
    obstacle_map = (occupancy_grid.grid >= obstacle_threshold).astype(np.float32)

    # Check if there are any obstacles
    if not np.any(obstacle_map):
        # No obstacles - everything is free
        gradient_data = np.zeros_like(occupancy_grid.grid, dtype=np.int8)
        gradient_data[unknown_mask] = CostValues.UNKNOWN
        return OccupancyGrid(
            grid=gradient_data,
            resolution=occupancy_grid.resolution,
            origin=occupancy_grid.origin,
            frame_id=occupancy_grid.frame_id,
            ts=occupancy_grid.ts,
        )

    # Label connected obstacle regions (clusters)
    # This groups all cells of the same wall/obstacle together
    obstacle_labels, num_obstacles = ndimage.label(obstacle_map)

    # If only one obstacle cluster, Voronoi edges don't make sense
    # Fall back to regular gradient behavior
    if num_obstacles <= 1:
        return gradient(occupancy_grid, obstacle_threshold, max_distance)

    # Compute distance transform with indices to nearest obstacle
    # indices[0][i,j], indices[1][i,j] = row,col of nearest obstacle to (i,j)
    distance_cells, indices = ndimage.distance_transform_edt(1 - obstacle_map, return_indices=True)

    # For each cell, find which obstacle cluster it belongs to (Voronoi region)
    # by looking up the label of its nearest obstacle cell
    nearest_obstacle_cluster = obstacle_labels[indices[0], indices[1]]

    # Find Voronoi edges: cells where neighbors belong to different obstacle clusters
    # Using max/min filters: an edge exists where max != min in the 3x3 neighborhood
    footprint = np.ones((3, 3), dtype=bool)
    local_max = ndimage.maximum_filter(
        nearest_obstacle_cluster, footprint=footprint, mode="nearest"
    )
    local_min = ndimage.minimum_filter(
        nearest_obstacle_cluster, footprint=footprint, mode="nearest"
    )
    voronoi_edges = local_max != local_min

    # Don't count obstacle cells as Voronoi edges
    voronoi_edges &= obstacle_map == 0

    # Compute distance to nearest Voronoi edge
    if not np.any(voronoi_edges):
        # No Voronoi edges found - fall back to regular gradient
        return gradient(occupancy_grid, obstacle_threshold, max_distance)

    voronoi_distance = ndimage.distance_transform_edt(~voronoi_edges)

    # Calculate cost based on position between obstacle and Voronoi edge
    # cost = 99 * d_voronoi / (d_obstacle + d_voronoi)
    # At Voronoi edge: d_voronoi = 0, cost = 0
    # Near obstacle: d_obstacle small, d_voronoi large, cost high
    total_distance = distance_cells + voronoi_distance
    with np.errstate(divide="ignore", invalid="ignore"):
        cost_ratio = np.where(total_distance > 0, voronoi_distance / total_distance, 0)

    gradient_values = cost_ratio * 99

    # Ensure obstacles are exactly 100
    gradient_values[obstacle_map > 0] = CostValues.OCCUPIED

    # Apply max_distance clipping - cells beyond max_distance from obstacles get cost 0
    max_distance_cells = max_distance / occupancy_grid.resolution
    gradient_values[distance_cells > max_distance_cells] = 0

    # Convert to int8
    gradient_data = gradient_values.astype(np.int8)

    # Preserve unknown cells
    gradient_data[unknown_mask] = CostValues.UNKNOWN

    return OccupancyGrid(
        grid=gradient_data,
        resolution=occupancy_grid.resolution,
        origin=occupancy_grid.origin,
        frame_id=occupancy_grid.frame_id,
        ts=occupancy_grid.ts,
    )
