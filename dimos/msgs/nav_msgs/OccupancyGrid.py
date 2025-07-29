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

import math
from enum import IntEnum
from typing import TYPE_CHECKING, BinaryIO, Optional

import numpy as np
from dimos_lcm.nav_msgs import MapMetaData
from dimos_lcm.nav_msgs import OccupancyGrid as LCMOccupancyGrid
from scipy import ndimage

from dimos.msgs.geometry_msgs import Pose, Vector3, VectorLike
from dimos.types.timestamped import Timestamped

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs import PointCloud2


class CostValues(IntEnum):
    """Standard cost values for occupancy grid cells.

    These values follow the ROS nav_msgs/OccupancyGrid convention:
    - 0: Free space
    - 1-99: Occupied space with varying cost levels
    - 100: Lethal obstacle (definitely occupied)
    - -1: Unknown space
    """

    UNKNOWN = -1  # Unknown space
    FREE = 0  # Free space
    OCCUPIED = 100  # Occupied/lethal space


class OccupancyGrid(Timestamped):
    """
    Convenience wrapper for nav_msgs/OccupancyGrid with numpy array support.
    """

    msg_name = "nav_msgs.OccupancyGrid"

    # Attributes
    ts: float
    frame_id: str
    info: MapMetaData
    grid: np.ndarray

    def __init__(
        self,
        grid: Optional[np.ndarray] = None,
        width: Optional[int] = None,
        height: Optional[int] = None,
        resolution: float = 0.05,
        origin: Optional[Pose] = None,
        frame_id: str = "world",
        ts: float = 0.0,
    ) -> None:
        """Initialize OccupancyGrid.

        Args:
            grid: 2D numpy array of int8 values (height x width)
            width: Width in cells (used if grid is None)
            height: Height in cells (used if grid is None)
            resolution: Grid resolution in meters/cell
            origin: Origin pose of the grid
            frame_id: Reference frame
            ts: Timestamp (defaults to current time if 0)
        """
        import time

        self.frame_id = frame_id
        self.ts = ts if ts != 0 else time.time()

        if grid is not None:
            # Initialize from numpy array
            if grid.ndim != 2:
                raise ValueError("Grid must be a 2D array")
            height, width = grid.shape
            self.info = MapMetaData(
                map_load_time=self._to_lcm_time(),
                resolution=resolution,
                width=width,
                height=height,
                origin=origin or Pose(),
            )
            self.grid = grid.astype(np.int8)
        elif width is not None and height is not None:
            # Initialize with dimensions
            self.info = MapMetaData(
                map_load_time=self._to_lcm_time(),
                resolution=resolution,
                width=width,
                height=height,
                origin=origin or Pose(),
            )
            self.grid = np.full((height, width), -1, dtype=np.int8)
        else:
            # Initialize empty
            self.info = MapMetaData(map_load_time=self._to_lcm_time())
            self.grid = np.array([], dtype=np.int8)

    def _to_lcm_time(self):
        """Convert timestamp to LCM Time."""
        from dimos_lcm.std_msgs import Time as LCMTime

        s = int(self.ts)
        return LCMTime(sec=s, nsec=int((self.ts - s) * 1_000_000_000))

    @property
    def width(self) -> int:
        """Width of the grid in cells."""
        return self.info.width

    @property
    def height(self) -> int:
        """Height of the grid in cells."""
        return self.info.height

    @property
    def resolution(self) -> float:
        """Grid resolution in meters/cell."""
        return self.info.resolution

    @property
    def origin(self) -> Pose:
        """Origin pose of the grid."""
        return self.info.origin

    @property
    def total_cells(self) -> int:
        """Total number of cells in the grid."""
        return self.width * self.height

    @property
    def occupied_cells(self) -> int:
        """Number of occupied cells (value >= 1)."""
        return int(np.sum(self.grid >= 1))

    @property
    def free_cells(self) -> int:
        """Number of free cells (value == 0)."""
        return int(np.sum(self.grid == 0))

    @property
    def unknown_cells(self) -> int:
        """Number of unknown cells (value == -1)."""
        return int(np.sum(self.grid == -1))

    @property
    def occupied_percent(self) -> float:
        """Percentage of cells that are occupied."""
        return (self.occupied_cells / self.total_cells * 100) if self.total_cells > 0 else 0.0

    @property
    def free_percent(self) -> float:
        """Percentage of cells that are free."""
        return (self.free_cells / self.total_cells * 100) if self.total_cells > 0 else 0.0

    @property
    def unknown_percent(self) -> float:
        """Percentage of cells that are unknown."""
        return (self.unknown_cells / self.total_cells * 100) if self.total_cells > 0 else 0.0

    def inflate(self, radius: float, cost_scaling_factor: float = 0.0) -> "OccupancyGrid":
        """Inflate obstacles by a given radius (vectorized).
        Args:
            radius: Inflation radius in meters
            cost_scaling_factor: Factor for decay (0.0 = no decay, binary inflation)
        Returns:
            New OccupancyGrid with inflated obstacles
        """
        # Convert radius to grid cells
        cell_radius = int(np.ceil(radius / self.resolution))

        # Get grid as numpy array
        grid_array = self.grid

        # Create square kernel for binary inflation
        kernel_size = 2 * cell_radius + 1
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)

        # Find occupied cells
        occupied_mask = grid_array >= CostValues.OCCUPIED

        if cost_scaling_factor == 0.0:
            # Binary inflation
            inflated = ndimage.binary_dilation(occupied_mask, structure=kernel)
            result_grid = grid_array.copy()
            result_grid[inflated] = CostValues.OCCUPIED
        else:
            # Distance-based inflation with decay
            # Create distance transform from occupied cells
            distance_field = ndimage.distance_transform_edt(~occupied_mask)

            # Apply exponential decay based on distance
            cost_field = CostValues.OCCUPIED * np.exp(-cost_scaling_factor * distance_field)

            # Combine with original grid, keeping higher values
            result_grid = np.maximum(grid_array, cost_field).astype(np.int8)

            # Ensure occupied cells remain at max value
            result_grid[occupied_mask] = CostValues.OCCUPIED

        # Create new OccupancyGrid with inflated data using numpy constructor
        return OccupancyGrid(
            grid=result_grid,
            resolution=self.resolution,
            origin=self.origin,
            frame_id=self.frame_id,
            ts=self.ts,
        )

    def world_to_grid(self, point: VectorLike) -> Vector3:
        """Convert world coordinates to grid coordinates.

        Args:
            point: A vector-like object containing X,Y coordinates

        Returns:
            Vector3 with grid coordinates
        """
        positionVector = Vector3(point)
        # Get origin position
        ox = self.origin.position.x
        oy = self.origin.position.y

        # Convert to grid coordinates (simplified, assuming no rotation)
        grid_x = (positionVector.x - ox) / self.resolution
        grid_y = (positionVector.y - oy) / self.resolution

        return Vector3(grid_x, grid_y, 0.0)

    def grid_to_world(self, grid_point: VectorLike) -> Vector3:
        """Convert grid coordinates to world coordinates.

        Args:
            grid_point: Vector-like object containing grid coordinates

        Returns:
            World position as Vector3
        """
        gridVector = Vector3(grid_point)
        # Get origin position
        ox = self.origin.position.x
        oy = self.origin.position.y

        # Convert to world (simplified, no rotation)
        x = ox + gridVector.x * self.resolution
        y = oy + gridVector.y * self.resolution

        return Vector3(x, y, 0.0)

    def __str__(self) -> str:
        """Create a concise string representation."""
        origin_pos = self.origin.position

        parts = [
            f"▦ OccupancyGrid[{self.frame_id}]",
            f"{self.width}x{self.height}",
            f"({self.width * self.resolution:.1f}x{self.height * self.resolution:.1f}m @",
            f"{1 / self.resolution:.0f}cm res)",
            f"Origin: ({origin_pos.x:.2f}, {origin_pos.y:.2f})",
            f"▣ {self.occupied_percent:.1f}%",
            f"□ {self.free_percent:.1f}%",
            f"◌ {self.unknown_percent:.1f}%",
        ]

        return " ".join(parts)

    def __repr__(self) -> str:
        """Create a detailed representation."""
        return (
            f"OccupancyGrid(width={self.width}, height={self.height}, "
            f"resolution={self.resolution}, frame_id='{self.frame_id}', "
            f"occupied={self.occupied_cells}, free={self.free_cells}, "
            f"unknown={self.unknown_cells})"
        )

    def lcm_encode(self) -> bytes:
        """Encode OccupancyGrid to LCM bytes."""
        # Create LCM message
        lcm_msg = LCMOccupancyGrid()

        # Build header on demand
        s = int(self.ts)
        lcm_msg.header.stamp.sec = s
        lcm_msg.header.stamp.nsec = int((self.ts - s) * 1_000_000_000)
        lcm_msg.header.frame_id = self.frame_id

        # Copy map metadata
        lcm_msg.info = self.info

        # Convert numpy array to flat data list
        if self.grid.size > 0:
            flat_data = self.grid.flatten()
            lcm_msg.data_length = len(flat_data)
            lcm_msg.data = flat_data.tolist()
        else:
            lcm_msg.data_length = 0
            lcm_msg.data = []

        return lcm_msg.lcm_encode()

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> "OccupancyGrid":
        """Decode LCM bytes to OccupancyGrid."""
        lcm_msg = LCMOccupancyGrid.lcm_decode(data)

        # Extract timestamp and frame_id from header
        ts = lcm_msg.header.stamp.sec + (lcm_msg.header.stamp.nsec / 1_000_000_000)
        frame_id = lcm_msg.header.frame_id

        # Extract grid data
        if lcm_msg.data and lcm_msg.info.width > 0 and lcm_msg.info.height > 0:
            grid = np.array(lcm_msg.data, dtype=np.int8).reshape(
                (lcm_msg.info.height, lcm_msg.info.width)
            )
        else:
            grid = np.array([], dtype=np.int8)

        # Create new instance
        instance = cls(
            grid=grid,
            resolution=lcm_msg.info.resolution,
            origin=lcm_msg.info.origin,
            frame_id=frame_id,
            ts=ts,
        )
        instance.info = lcm_msg.info
        return instance

    @classmethod
    def from_pointcloud(
        cls,
        cloud: "PointCloud2",
        resolution: float = 0.05,
        min_height: float = 0.1,
        max_height: float = 2.0,
        frame_id: Optional[str] = None,
        mark_free_radius: float = 0.0,
    ) -> "OccupancyGrid":
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
        # Import here to avoid circular dependency
        from dimos.msgs.sensor_msgs import PointCloud2

        # Get points as numpy array
        points = cloud.as_numpy()

        if len(points) == 0:
            # Return empty grid
            return cls(
                width=1, height=1, resolution=resolution, frame_id=frame_id or cloud.frame_id
            )

        # Filter points by height
        height_mask = (points[:, 2] >= min_height) & (points[:, 2] <= max_height)
        filtered_points = points[height_mask]

        if len(filtered_points) == 0:
            # No points in height range
            return cls(
                width=1, height=1, resolution=resolution, frame_id=frame_id or cloud.frame_id
            )

        # Find bounds of the point cloud in X-Y plane
        min_x = np.min(filtered_points[:, 0])
        max_x = np.max(filtered_points[:, 0])
        min_y = np.min(filtered_points[:, 1])
        max_y = np.max(filtered_points[:, 1])

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

        # Convert points to grid indices
        grid_x = ((filtered_points[:, 0] - min_x) / resolution).astype(np.int32)
        grid_y = ((filtered_points[:, 1] - min_y) / resolution).astype(np.int32)

        # Clip indices to grid bounds
        grid_x = np.clip(grid_x, 0, width - 1)
        grid_y = np.clip(grid_y, 0, height - 1)

        # Mark cells as occupied
        grid[grid_y, grid_x] = 100  # Lethal obstacle

        # Mark free space around obstacles based on mark_free_radius
        if mark_free_radius > 0:
            # Mark a specified radius around occupied cells as free
            from scipy.ndimage import binary_dilation

            occupied_mask = grid == 100
            free_radius_cells = int(np.ceil(mark_free_radius / resolution))

            # Create circular kernel
            y, x = np.ogrid[
                -free_radius_cells : free_radius_cells + 1,
                -free_radius_cells : free_radius_cells + 1,
            ]
            kernel = x**2 + y**2 <= free_radius_cells**2

            known_area = binary_dilation(occupied_mask, structure=kernel, iterations=1)
            # Mark non-occupied cells in the known area as free
            grid[known_area & (grid != 100)] = 0
        else:
            # Default: only mark immediate neighbors as free to preserve unknown
            from scipy.ndimage import binary_dilation

            occupied_mask = grid == 100
            # Use a small 3x3 kernel to only mark immediate neighbors
            structure = np.array([[1, 1, 1], [1, 1, 1], [1, 1, 1]])
            immediate_neighbors = binary_dilation(occupied_mask, structure=structure, iterations=1)

            # Mark only immediate neighbors as free (not the occupied cells themselves)
            grid[immediate_neighbors & (grid != 100)] = 0

        # Create and return OccupancyGrid
        # Get timestamp from cloud if available
        ts = cloud.ts if hasattr(cloud, "ts") and cloud.ts is not None else 0.0

        occupancy_grid = cls(
            grid=grid,
            resolution=resolution,
            origin=origin,
            frame_id=frame_id or cloud.frame_id,
            ts=ts,
        )

        return occupancy_grid

    def gradient(self, obstacle_threshold: int = 50, max_distance: float = 2.0) -> "OccupancyGrid":
        """Create a gradient OccupancyGrid for path planning.

        Creates a gradient where free space has value 0 and values increase near obstacles.
        This can be used as a cost map for path planning algorithms like A*.

        Args:
            obstacle_threshold: Cell values >= this are considered obstacles (default: 50)
            max_distance: Maximum distance to compute gradient in meters (default: 5.0)

        Returns:
            New OccupancyGrid with gradient values:
            - -1: Unknown cells far from obstacles (beyond max_distance)
            - 0: Free space far from obstacles
            - 1-99: Increasing cost as you approach obstacles
            - 100: At obstacles

        Note: Unknown cells within max_distance of obstacles will have gradient
        values assigned, allowing path planning through unknown areas.
        """

        # Remember which cells are unknown
        unknown_mask = self.grid == -1

        # Create a working grid where unknown cells are treated as free for distance calculation
        working_grid = self.grid.copy()
        working_grid[unknown_mask] = 0  # Treat unknown as free for gradient computation

        # Create binary obstacle map from working grid
        # Consider cells >= threshold as obstacles (1), everything else as free (0)
        obstacle_map = (working_grid >= obstacle_threshold).astype(np.float32)

        # Compute distance transform (distance to nearest obstacle in cells)
        distance_cells = ndimage.distance_transform_edt(1 - obstacle_map)

        # Convert to meters and clip to max distance
        distance_meters = np.clip(distance_cells * self.resolution, 0, max_distance)

        # Invert and scale to 0-100 range
        # Far from obstacles (max_distance) -> 0
        # At obstacles (0 distance) -> 100
        gradient_values = (1 - distance_meters / max_distance) * 100

        # Ensure obstacles are exactly 100
        gradient_values[obstacle_map > 0] = 100

        # Convert to int8 for OccupancyGrid
        gradient_data = gradient_values.astype(np.int8)

        # Only preserve unknown cells that are beyond max_distance from any obstacle
        # This allows gradient to spread through unknown areas near obstacles
        far_unknown_mask = unknown_mask & (distance_meters >= max_distance)
        gradient_data[far_unknown_mask] = -1

        # Create new OccupancyGrid with gradient
        gradient_grid = OccupancyGrid(
            grid=gradient_data,
            resolution=self.resolution,
            origin=self.origin,
            frame_id=self.frame_id,
            ts=self.ts,
        )

        return gradient_grid

    def filter_above(self, threshold: int) -> "OccupancyGrid":
        """Create a new OccupancyGrid with only values above threshold.

        Args:
            threshold: Keep cells with values > threshold

        Returns:
            New OccupancyGrid where:
            - Cells > threshold: kept as-is
            - Cells <= threshold: set to -1 (unknown)
            - Unknown cells (-1): preserved
        """
        new_grid = self.grid.copy()

        # Create mask for cells to filter (not unknown and <= threshold)
        filter_mask = (new_grid != -1) & (new_grid <= threshold)

        # Set filtered cells to unknown
        new_grid[filter_mask] = -1

        # Create new OccupancyGrid
        filtered = OccupancyGrid(
            new_grid,
            resolution=self.resolution,
            origin=self.origin,
            frame_id=self.frame_id,
            ts=self.ts,
        )

        return filtered

    def filter_below(self, threshold: int) -> "OccupancyGrid":
        """Create a new OccupancyGrid with only values below threshold.

        Args:
            threshold: Keep cells with values < threshold

        Returns:
            New OccupancyGrid where:
            - Cells < threshold: kept as-is
            - Cells >= threshold: set to -1 (unknown)
            - Unknown cells (-1): preserved
        """
        new_grid = self.grid.copy()

        # Create mask for cells to filter (not unknown and >= threshold)
        filter_mask = (new_grid != -1) & (new_grid >= threshold)

        # Set filtered cells to unknown
        new_grid[filter_mask] = -1

        # Create new OccupancyGrid
        filtered = OccupancyGrid(
            new_grid,
            resolution=self.resolution,
            origin=self.origin,
            frame_id=self.frame_id,
            ts=self.ts,
        )

        return filtered

    def max(self) -> "OccupancyGrid":
        """Create a new OccupancyGrid with all non-unknown cells set to maximum value.

        Returns:
            New OccupancyGrid where:
            - All non-unknown cells: set to CostValues.OCCUPIED (100)
            - Unknown cells: preserved as CostValues.UNKNOWN (-1)
        """
        new_grid = self.grid.copy()

        # Set all non-unknown cells to max
        new_grid[new_grid != CostValues.UNKNOWN] = CostValues.OCCUPIED

        # Create new OccupancyGrid
        maxed = OccupancyGrid(
            new_grid,
            resolution=self.resolution,
            origin=self.origin,
            frame_id=self.frame_id,
            ts=self.ts,
        )

        return maxed
