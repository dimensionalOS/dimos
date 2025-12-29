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

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any, Protocol, TypeVar

from numba import njit, prange  # type: ignore[import-untyped]
import numpy as np
from scipy import ndimage  # type: ignore[import-untyped]

from dimos.msgs.geometry_msgs import Pose
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid

if TYPE_CHECKING:
    from numpy.typing import NDArray


@njit(cache=True)  # type: ignore[untyped-decorator]
def _height_map_kernel(
    points: NDArray[np.floating[Any]],
    min_height_map: NDArray[np.floating[Any]],
    max_height_map: NDArray[np.floating[Any]],
    min_x: float,
    min_y: float,
    inv_res: float,
    width: int,
    height: int,
) -> None:
    """Build min/max height maps from points (faster than np.fmax/fmin.at)."""
    n = points.shape[0]
    for i in range(n):
        x = points[i, 0]
        y = points[i, 1]
        z = points[i, 2]

        gx = int((x - min_x) * inv_res + 0.5)
        gy = int((y - min_y) * inv_res + 0.5)

        if 0 <= gx < width and 0 <= gy < height:
            cur_min = min_height_map[gy, gx]
            cur_max = max_height_map[gy, gx]
            # NaN comparisons are always False, so first point sets the value
            if z < cur_min or cur_min != cur_min:  # cur_min != cur_min checks for NaN
                min_height_map[gy, gx] = z
            if z > cur_max or cur_max != cur_max:
                max_height_map[gy, gx] = z


@njit(cache=True, parallel=True)  # type: ignore[untyped-decorator]
def _simple_occupancy_kernel(
    points: NDArray[np.floating[Any]],
    grid: NDArray[np.signedinteger[Any]],
    min_x: float,
    min_y: float,
    inv_res: float,
    width: int,
    height: int,
    min_height: float,
    max_height: float,
) -> None:
    """Numba-accelerated kernel for simple_occupancy grid population."""
    n = points.shape[0]
    # Pass 1: Mark ground as free
    for i in prange(n):
        x = points[i, 0]
        y = points[i, 1]
        z = points[i, 2]
        if z < min_height:
            gx = int((x - min_x) * inv_res + 0.5)
            gy = int((y - min_y) * inv_res + 0.5)
            if 0 <= gx < width and 0 <= gy < height:
                grid[gy, gx] = 0

    # Pass 2: Mark obstacles (overwrites ground)
    for i in prange(n):
        x = points[i, 0]
        y = points[i, 1]
        z = points[i, 2]
        if min_height <= z <= max_height:
            gx = int((x - min_x) * inv_res + 0.5)
            gy = int((y - min_y) * inv_res + 0.5)
            if 0 <= gx < width and 0 <= gy < height:
                grid[gy, gx] = 100


if TYPE_CHECKING:
    from collections.abc import Callable

    from dimos.msgs.sensor_msgs import PointCloud2


@dataclass(frozen=True)
class OccupancyConfig:
    """Base config for all occupancy grid generators."""

    resolution: float = 0.05
    frame_id: str | None = None


ConfigT = TypeVar("ConfigT", bound=OccupancyConfig, covariant=True)


class OccupancyFn(Protocol[ConfigT]):
    """Protocol for pointcloud-to-occupancy conversion functions.

    Functions matching this protocol take a PointCloud2 and config kwargs,
    returning an OccupancyGrid. Call with: fn(cloud, resolution=0.1, ...)
    """

    @property
    def config_class(self) -> type[ConfigT]: ...

    def __call__(self, cloud: PointCloud2, **kwargs: Any) -> OccupancyGrid: ...


# Populated after function definitions below
OCCUPANCY_ALGOS: dict[str, Callable[..., OccupancyGrid]] = {}


@dataclass(frozen=True)
class HeightCostConfig(OccupancyConfig):
    """Config for height-cost based occupancy (terrain slope analysis)."""

    can_pass_under: float = 0.6
    can_climb: float = 0.15
    ignore_noise: float = 0.05
    smoothing: float = 1.0


def height_cost_occupancy(cloud: PointCloud2, **kwargs: Any) -> OccupancyGrid:
    """Create a costmap based on terrain slope (rate of change of height).

    Costs are assigned based on the gradient magnitude of the terrain height.
    Steeper slopes get higher costs, with max_step height change mapping to cost 100.
    Cells without observations are marked unknown (-1).

    Args:
        cloud: PointCloud2 message containing 3D points
        **kwargs: HeightCostConfig fields - resolution, can_pass_under, can_climb,
                  ignore_noise, smoothing, frame_id

    Returns:
        OccupancyGrid with costs 0-100 based on terrain slope, -1 for unknown
    """
    cfg = HeightCostConfig(**kwargs)
    points = cloud.as_numpy().astype(np.float64)  # Upcast to avoid float32 rounding
    ts = cloud.ts if hasattr(cloud, "ts") and cloud.ts is not None else 0.0

    if len(points) == 0:
        return OccupancyGrid(
            width=1,
            height=1,
            resolution=cfg.resolution,
            frame_id=cfg.frame_id or cloud.frame_id,
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
    width = int(np.ceil((max_x - min_x) / cfg.resolution))
    height = int(np.ceil((max_y - min_y) / cfg.resolution))

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

    # Use numba kernel (faster than np.fmax/fmin.at)
    _height_map_kernel(
        points,
        min_height_map,
        max_height_map,
        min_x,
        min_y,
        1.0 / cfg.resolution,
        width,
        height,
    )

    # Step 2: Determine effective height for each cell
    # If gap between min and max > can_pass_under, robot can pass under - use min (ground)
    # Otherwise use max (solid obstacle)
    height_gap = max_height_map - min_height_map
    height_map = np.where(height_gap > cfg.can_pass_under, min_height_map, max_height_map)

    # Track which cells have observations
    observed_mask = ~np.isnan(height_map)

    # Step 3: Apply smoothing to fill gaps while preserving unknown space
    if cfg.smoothing > 0 and np.any(observed_mask):
        # Use a weighted smoothing approach that only interpolates from known cells
        # Create a weight map (1 for observed, 0 for unknown)
        weights = observed_mask.astype(np.float32)
        height_map_filled = np.where(observed_mask, height_map, 0.0)

        # Smooth both height values and weights
        smoothed_heights = ndimage.gaussian_filter(height_map_filled, sigma=cfg.smoothing)
        smoothed_weights = ndimage.gaussian_filter(weights, sigma=cfg.smoothing)

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
        grad_x = ndimage.sobel(height_for_grad, axis=1) / (8.0 * cfg.resolution)
        grad_y = ndimage.sobel(height_for_grad, axis=0) / (8.0 * cfg.resolution)

        # Gradient magnitude = height change per meter
        gradient_magnitude = np.sqrt(grad_x**2 + grad_y**2)

        # Map gradient to cost: can_climb height change over one cell maps to cost 100
        # gradient_magnitude is in m/m, so multiply by resolution to get height change per cell
        height_change_per_cell = gradient_magnitude * cfg.resolution

        # Ignore height changes below noise threshold (lidar floor noise)
        height_change_per_cell = np.where(
            height_change_per_cell < cfg.ignore_noise, 0.0, height_change_per_cell
        )

        cost_float = (height_change_per_cell / cfg.can_climb) * 100.0
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
        resolution=cfg.resolution,
        origin=origin,
        frame_id=cfg.frame_id or cloud.frame_id,
        ts=ts,
    )


@dataclass(frozen=True)
class GeneralOccupancyConfig(OccupancyConfig):
    """Config for general obstacle-based occupancy."""

    min_height: float = 0.1
    max_height: float = 2.0
    mark_free_radius: float = 0.4


# can remove, just needs pulling out of unitree type/map.py
def general_occupancy(cloud: PointCloud2, **kwargs: Any) -> OccupancyGrid:
    """Create an OccupancyGrid from a PointCloud2 message.

    Args:
        cloud: PointCloud2 message containing 3D points
        **kwargs: GeneralOccupancyConfig fields - resolution, min_height, max_height,
                  frame_id, mark_free_radius

    Returns:
        OccupancyGrid with occupied cells where points were projected
    """
    cfg = GeneralOccupancyConfig(**kwargs)
    points = cloud.as_numpy().astype(np.float64)  # Upcast to avoid float32 rounding

    if len(points) == 0:
        return OccupancyGrid(
            width=1,
            height=1,
            resolution=cfg.resolution,
            frame_id=cfg.frame_id or cloud.frame_id,
        )

    # Filter points by height for obstacles
    obstacle_mask = (points[:, 2] >= cfg.min_height) & (points[:, 2] <= cfg.max_height)
    obstacle_points = points[obstacle_mask]

    # Get points below min_height for marking as free space
    ground_mask = points[:, 2] < cfg.min_height
    ground_points = points[ground_mask]

    # Find bounds of the point cloud in X-Y plane (use all points)
    min_x = np.min(points[:, 0])
    max_x = np.max(points[:, 0])
    min_y = np.min(points[:, 1])
    max_y = np.max(points[:, 1])

    # Add some padding around the bounds
    padding = 1.0  # 1 meter padding
    min_x -= padding
    max_x += padding
    min_y -= padding
    max_y += padding

    # Calculate grid dimensions
    width = int(np.ceil((max_x - min_x) / cfg.resolution))
    height = int(np.ceil((max_y - min_y) / cfg.resolution))

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
        ground_x = ((ground_points[:, 0] - min_x) / cfg.resolution).astype(np.int32)
        ground_y = ((ground_points[:, 1] - min_y) / cfg.resolution).astype(np.int32)

        # Clip indices to grid bounds
        ground_x = np.clip(ground_x, 0, width - 1)
        ground_y = np.clip(ground_y, 0, height - 1)

        # Mark ground cells as free
        grid[ground_y, ground_x] = 0  # Free space

    # Then mark obstacle points (will override ground if at same location)
    if len(obstacle_points) > 0:
        obs_x = ((obstacle_points[:, 0] - min_x) / cfg.resolution).astype(np.int32)
        obs_y = ((obstacle_points[:, 1] - min_y) / cfg.resolution).astype(np.int32)

        # Clip indices to grid bounds
        obs_x = np.clip(obs_x, 0, width - 1)
        obs_y = np.clip(obs_y, 0, height - 1)

        # Mark cells as occupied
        grid[obs_y, obs_x] = 100  # Lethal obstacle

    # Apply mark_free_radius to expand free space areas
    if cfg.mark_free_radius > 0:
        # Expand existing free space areas by the specified radius
        # This will NOT expand from obstacles, only from free space

        free_mask = grid == 0  # Current free space
        free_radius_cells = int(np.ceil(cfg.mark_free_radius / cfg.resolution))

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

    return OccupancyGrid(
        grid=grid,
        resolution=cfg.resolution,
        origin=origin,
        frame_id=cfg.frame_id or cloud.frame_id,
        ts=ts,
    )


@dataclass(frozen=True)
class SimpleOccupancyConfig(OccupancyConfig):
    """Config for simple occupancy with morphological closing."""

    min_height: float = 0.1
    max_height: float = 2.0
    closing_iterations: int = 1
    closing_connectivity: int = 2
    can_pass_under: float = 0.6
    can_climb: float = 0.15
    ignore_noise: float = 0.05
    smoothing: float = 1.0


def simple_occupancy(cloud: PointCloud2, **kwargs: Any) -> OccupancyGrid:
    """Create a simple occupancy grid with morphological closing.

    Args:
        cloud: PointCloud2 message containing 3D points
        **kwargs: SimpleOccupancyConfig fields - resolution, min_height, max_height,
                  frame_id, closing_iterations, closing_connectivity

    Returns:
        OccupancyGrid with occupied/free cells
    """
    cfg = SimpleOccupancyConfig(**kwargs)
    points = cloud.as_numpy().astype(np.float64)  # Upcast to avoid float32 rounding

    if len(points) == 0:
        return OccupancyGrid(
            width=1,
            height=1,
            resolution=cfg.resolution,
            frame_id=cfg.frame_id or cloud.frame_id,
        )

    # Find bounds of the point cloud in X-Y plane
    min_x = float(np.min(points[:, 0])) - 1.0
    max_x = float(np.max(points[:, 0])) + 1.0
    min_y = float(np.min(points[:, 1])) - 1.0
    max_y = float(np.max(points[:, 1])) + 1.0

    # Calculate grid dimensions
    width = int(np.ceil((max_x - min_x) / cfg.resolution))
    height = int(np.ceil((max_y - min_y) / cfg.resolution))

    # Create origin pose (bottom-left corner of the grid)
    origin = Pose()
    origin.position.x = min_x
    origin.position.y = min_y
    origin.position.z = 0.0
    origin.orientation.w = 1.0

    # Initialize grid (all unknown)
    grid = np.full((height, width), -1, dtype=np.int8)

    # Use numba kernel for fast grid population
    _simple_occupancy_kernel(
        points,
        grid,
        min_x,
        min_y,
        1.0 / cfg.resolution,
        width,
        height,
        cfg.min_height,
        cfg.max_height,
    )

    ts = cloud.ts if hasattr(cloud, "ts") and cloud.ts is not None else 0.0

    return OccupancyGrid(
        grid=grid,
        resolution=cfg.resolution,
        origin=origin,
        frame_id=cfg.frame_id or cloud.frame_id,
        ts=ts,
    )


# Populate algorithm registry
OCCUPANCY_ALGOS.update(
    {
        "height_cost": height_cost_occupancy,
        "general": general_occupancy,
        "simple": simple_occupancy,
    }
)
