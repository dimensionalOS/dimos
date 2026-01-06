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

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from scipy import ndimage  # type: ignore[import-untyped]

from dimos.msgs.geometry_msgs import Pose
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs import PointCloud2


def height_cost_occupancy(
    cloud: PointCloud2,
    resolution: float = 0.05,
    can_pass_under: float = 0.6,
    can_climb: float = 0.15,
    ignore_noise: float = 0.05,
    smoothing: float = 1.0,
    frame_id: str | None = None,
) -> OccupancyGrid:
    """Create a costmap based on terrain slope (rate of change of height).

    Costs are assigned based on the gradient magnitude of the terrain height.
    Steeper slopes get higher costs, with max_step height change mapping to cost 100.
    Cells without observations are marked unknown (-1).

    Args:
        cloud: PointCloud2 message containing 3D points
        resolution: Grid resolution in meters/cell (default: 0.05)
        can_pass_under: Max height to consider - points above are ignored (default: 0.6)
        can_climb: Height change in meters that maps to cost 100 (default: 0.15)
        smoothing: Gaussian smoothing sigma in cells for filling gaps (default: 1.0)
        frame_id: Reference frame for the grid (default: uses cloud's frame_id)

    Returns:
        OccupancyGrid with costs 0-100 based on terrain slope, -1 for unknown
    """
    points = cloud.as_numpy()
    ts = cloud.ts if hasattr(cloud, "ts") and cloud.ts is not None else 0.0

    if len(points) == 0:
        return OccupancyGrid(
            width=1, height=1, resolution=resolution, frame_id=frame_id or cloud.frame_id
        )

    # Find bounds of the point cloud in X-Y plane (use all points)
    min_x = np.min(points[:, 0])
    max_x = np.max(points[:, 0])
    min_y = np.min(points[:, 1])
    max_y = np.max(points[:, 1])

    # Add padding
    padding = 1.0
    min_x -= padding
    max_x += padding
    min_y -= padding
    max_y += padding

    # Calculate grid dimensions
    width = int(np.ceil((max_x - min_x) / resolution))
    height = int(np.ceil((max_y - min_y) / resolution))

    # Create origin pose
    origin = Pose()
    origin.position.x = min_x
    origin.position.y = min_y
    origin.position.z = 0.0
    origin.orientation.w = 1.0

    # Step 1: Build min and max height maps for each cell
    # Initialize with NaN to track which cells have observations
    min_height_map = np.full((height, width), np.nan, dtype=np.float32)
    max_height_map = np.full((height, width), np.nan, dtype=np.float32)

    # Convert point XY to grid indices
    grid_x = np.round((points[:, 0] - min_x) / resolution).astype(np.int32)
    grid_y = np.round((points[:, 1] - min_y) / resolution).astype(np.int32)

    # Clip to grid bounds
    grid_x = np.clip(grid_x, 0, width - 1)
    grid_y = np.clip(grid_y, 0, height - 1)

    # Use np.fmax/fmin.at which ignore NaN
    np.fmax.at(max_height_map, (grid_y, grid_x), points[:, 2])
    np.fmin.at(min_height_map, (grid_y, grid_x), points[:, 2])

    # Step 2: Determine effective height for each cell
    # If gap between min and max > can_pass_under, robot can pass under - use min (ground)
    # Otherwise use max (solid obstacle)
    height_gap = max_height_map - min_height_map
    height_map = np.where(height_gap > can_pass_under, min_height_map, max_height_map)

    # Track which cells have observations
    observed_mask = ~np.isnan(height_map)

    # Step 3: Apply smoothing to fill gaps while preserving unknown space
    if smoothing > 0 and np.any(observed_mask):
        # Use a weighted smoothing approach that only interpolates from known cells
        # Create a weight map (1 for observed, 0 for unknown)
        weights = observed_mask.astype(np.float32)
        height_map_filled = np.where(observed_mask, height_map, 0.0)

        # Smooth both height values and weights
        smoothed_heights = ndimage.gaussian_filter(height_map_filled, sigma=smoothing)
        smoothed_weights = ndimage.gaussian_filter(weights, sigma=smoothing)

        # Avoid division by zero (use np.divide with where to prevent warning)
        valid_smooth = smoothed_weights > 0.01
        height_map_smoothed = np.full_like(smoothed_heights, np.nan)
        np.divide(smoothed_heights, smoothed_weights, out=height_map_smoothed, where=valid_smooth)

        # Keep original values where we had observations, use smoothed elsewhere
        height_map = np.where(observed_mask, height_map, height_map_smoothed)

        # Update observed mask to include smoothed cells
        observed_mask = ~np.isnan(height_map)

    # Step 4: Calculate rate of change (gradient magnitude)
    # Use Sobel filters for gradient calculation
    if np.any(observed_mask):
        # Replace NaN with 0 for gradient calculation
        height_for_grad = np.where(observed_mask, height_map, 0.0)

        # Calculate gradients (Sobel gives gradient in pixels, scale by resolution)
        grad_x = ndimage.sobel(height_for_grad, axis=1) / (8.0 * resolution)
        grad_y = ndimage.sobel(height_for_grad, axis=0) / (8.0 * resolution)

        # Gradient magnitude = height change per meter
        gradient_magnitude = np.sqrt(grad_x**2 + grad_y**2)

        # Map gradient to cost: can_climb height change over one cell maps to cost 100
        # gradient_magnitude is in m/m, so multiply by resolution to get height change per cell
        height_change_per_cell = gradient_magnitude * resolution

        # Ignore height changes below noise threshold (lidar floor noise)
        height_change_per_cell = np.where(
            height_change_per_cell < ignore_noise, 0.0, height_change_per_cell
        )

        cost_float = (height_change_per_cell / can_climb) * 100.0
        cost_float = np.clip(cost_float, 0, 100)

        # Erode observed mask - only trust gradients where all neighbors are observed
        # This prevents false high costs at boundaries with unknown regions
        structure = ndimage.generate_binary_structure(2, 1)  # 4-connectivity
        valid_gradient_mask = ndimage.binary_erosion(observed_mask, structure=structure)

        # Convert to int8, marking cells without valid gradients as -1
        cost = np.where(valid_gradient_mask, cost_float.astype(np.int8), -1)
    else:
        cost = np.full((height, width), -1, dtype=np.int8)

    return OccupancyGrid(
        grid=cost,
        resolution=resolution,
        origin=origin,
        frame_id=frame_id or cloud.frame_id,
        ts=ts,
    )


def general_occupancy(
    cloud: PointCloud2,
    resolution: float = 0.05,
    min_height: float = 0.1,
    max_height: float = 2.0,
    frame_id: str | None = None,
    mark_free_radius: float = 0.4,
) -> OccupancyGrid:
    """Create an OccupancyGrid from a PointCloud2 message.

    Args:
        cloud: PointCloud2 message containing 3D points
        resolution: Grid resolution in meters/cell (default: 0.05)
        min_height: Minimum height threshold for including points (default: 0.1)
        max_height: Maximum height threshold for including points (default: 2.0)
        frame_id: Reference frame for the grid (default: uses cloud's frame_id)
        mark_free_radius: Radius in meters around obstacles to mark as free space (default: 0.0)
                            If 0, only immediate neighbors are marked free.
                            Set to preserve unknown areas for exploration.

    Returns:
        OccupancyGrid with occupied cells where points were projected
    """

    # Get points as numpy array
    points = cloud.as_numpy()

    if len(points) == 0:
        # Return empty grid
        return OccupancyGrid(
            width=1, height=1, resolution=resolution, frame_id=frame_id or cloud.frame_id
        )

    # Filter points by height for obstacles
    obstacle_mask = (points[:, 2] >= min_height) & (points[:, 2] <= max_height)
    obstacle_points = points[obstacle_mask]

    # Get points below min_height for marking as free space
    ground_mask = points[:, 2] < min_height
    ground_points = points[ground_mask]

    # Find bounds of the point cloud in X-Y plane (use all points)
    if len(points) > 0:
        min_x = np.min(points[:, 0])
        max_x = np.max(points[:, 0])
        min_y = np.min(points[:, 1])
        max_y = np.max(points[:, 1])
    else:
        # Return empty grid if no points at all
        return OccupancyGrid(
            width=1, height=1, resolution=resolution, frame_id=frame_id or cloud.frame_id
        )

    # Add some padding around the bounds
    padding = 1.0  # 1 meter padding
    min_x -= padding
    max_x += padding
    min_y -= padding
    max_y += padding

    # Calculate grid dimensions
    width = int(np.ceil((max_x - min_x) / resolution))
    height = int(np.ceil((max_y - min_y) / resolution))

    # Create origin pose (bottom-left corner of the grid)
    origin = Pose()
    origin.position.x = min_x
    origin.position.y = min_y
    origin.position.z = 0.0
    origin.orientation.w = 1.0  # No rotation

    # Initialize grid (all unknown)
    grid = np.full((height, width), -1, dtype=np.int8)

    # First, mark ground points as free space
    if len(ground_points) > 0:
        ground_x = ((ground_points[:, 0] - min_x) / resolution).astype(np.int32)
        ground_y = ((ground_points[:, 1] - min_y) / resolution).astype(np.int32)

        # Clip indices to grid bounds
        ground_x = np.clip(ground_x, 0, width - 1)
        ground_y = np.clip(ground_y, 0, height - 1)

        # Mark ground cells as free
        grid[ground_y, ground_x] = 0  # Free space

    # Then mark obstacle points (will override ground if at same location)
    if len(obstacle_points) > 0:
        obs_x = ((obstacle_points[:, 0] - min_x) / resolution).astype(np.int32)
        obs_y = ((obstacle_points[:, 1] - min_y) / resolution).astype(np.int32)

        # Clip indices to grid bounds
        obs_x = np.clip(obs_x, 0, width - 1)
        obs_y = np.clip(obs_y, 0, height - 1)

        # Mark cells as occupied
        grid[obs_y, obs_x] = 100  # Lethal obstacle

    # Apply mark_free_radius to expand free space areas
    if mark_free_radius > 0:
        # Expand existing free space areas by the specified radius
        # This will NOT expand from obstacles, only from free space

        free_mask = grid == 0  # Current free space
        free_radius_cells = int(np.ceil(mark_free_radius / resolution))

        # Create circular kernel
        y, x = np.ogrid[
            -free_radius_cells : free_radius_cells + 1,
            -free_radius_cells : free_radius_cells + 1,
        ]
        kernel = x**2 + y**2 <= free_radius_cells**2

        # Dilate free space areas
        expanded_free = ndimage.binary_dilation(free_mask, structure=kernel, iterations=1)

        # Mark expanded areas as free, but don't override obstacles
        grid[expanded_free & (grid != 100)] = 0

    # Create and return OccupancyGrid
    # Get timestamp from cloud if available
    ts = cloud.ts if hasattr(cloud, "ts") and cloud.ts is not None else 0.0

    occupancy_grid = OccupancyGrid(
        grid=grid,
        resolution=resolution,
        origin=origin,
        frame_id=frame_id or cloud.frame_id,
        ts=ts,
    )

    return occupancy_grid


def simple_occupancy(
    cloud: PointCloud2,
    resolution: float = 0.05,
    min_height: float = 0.1,
    max_height: float = 2.0,
    frame_id: str | None = None,
    closing_iterations: int = 1,
    closing_connectivity: int = 2,
) -> OccupancyGrid:
    points = cloud.as_numpy()

    if len(points) == 0:
        return OccupancyGrid(
            width=1, height=1, resolution=resolution, frame_id=frame_id or cloud.frame_id
        )

    # Filter points by height for obstacles
    obstacle_mask = (points[:, 2] >= min_height) & (points[:, 2] <= max_height)
    obstacle_points = points[obstacle_mask]

    # Get points below min_height for marking as free space
    ground_mask = points[:, 2] < min_height
    ground_points = points[ground_mask]

    # Find bounds of the point cloud in X-Y plane (use all points)
    if len(points) > 0:
        min_x = np.min(points[:, 0])
        max_x = np.max(points[:, 0])
        min_y = np.min(points[:, 1])
        max_y = np.max(points[:, 1])
    else:
        # Return empty grid if no points at all
        return OccupancyGrid(
            width=1, height=1, resolution=resolution, frame_id=frame_id or cloud.frame_id
        )

    # Add some padding around the bounds
    padding = 1.0  # 1 meter padding
    min_x -= padding
    max_x += padding
    min_y -= padding
    max_y += padding

    # Calculate grid dimensions
    width = int(np.ceil((max_x - min_x) / resolution))
    height = int(np.ceil((max_y - min_y) / resolution))

    # Create origin pose (bottom-left corner of the grid)
    origin = Pose()
    origin.position.x = min_x
    origin.position.y = min_y
    origin.position.z = 0.0
    origin.orientation.w = 1.0  # No rotation

    # Initialize grid (all unknown)
    grid = np.full((height, width), -1, dtype=np.int8)

    # First, mark ground points as free space
    if len(ground_points) > 0:
        ground_x = np.round((ground_points[:, 0] - min_x) / resolution).astype(np.int32)
        ground_y = np.round((ground_points[:, 1] - min_y) / resolution).astype(np.int32)

        # Clip indices to grid bounds
        ground_x = np.clip(ground_x, 0, width - 1)
        ground_y = np.clip(ground_y, 0, height - 1)

        # Mark ground cells as free
        grid[ground_y, ground_x] = 0  # Free space

    # Then mark obstacle points (will override ground if at same location)
    if len(obstacle_points) > 0:
        obs_x = np.round((obstacle_points[:, 0] - min_x) / resolution).astype(np.int32)
        obs_y = np.round((obstacle_points[:, 1] - min_y) / resolution).astype(np.int32)

        # Clip indices to grid bounds
        obs_x = np.clip(obs_x, 0, width - 1)
        obs_y = np.clip(obs_y, 0, height - 1)

        # Mark cells as occupied
        grid[obs_y, obs_x] = 100  # Lethal obstacle

    # Fill small gaps in occupied regions using morphological closing
    occupied_mask = grid == 100
    if np.any(occupied_mask) and closing_iterations > 0:
        # connectivity=1 gives 4-connectivity, connectivity=2 gives 8-connectivity
        structure = ndimage.generate_binary_structure(2, closing_connectivity)
        # Closing = dilation then erosion - fills small holes
        closed_mask = ndimage.binary_closing(
            occupied_mask, structure=structure, iterations=closing_iterations
        )
        # Fill gaps (both unknown and free space)
        grid[closed_mask] = 100

    # Create and return OccupancyGrid
    # Get timestamp from cloud if available
    ts = cloud.ts if hasattr(cloud, "ts") and cloud.ts is not None else 0.0

    occupancy_grid = OccupancyGrid(
        grid=grid,
        resolution=resolution,
        origin=origin,
        frame_id=frame_id or cloud.frame_id,
        ts=ts,
    )

    return occupancy_grid
