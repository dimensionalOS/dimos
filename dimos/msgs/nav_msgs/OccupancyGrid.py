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
from typing import TYPE_CHECKING, BinaryIO, Optional, Union

import numpy as np
from dimos_lcm.nav_msgs import MapMetaData as LCMMapMetaData
from dimos_lcm.nav_msgs import OccupancyGrid as LCMOccupancyGrid
from plum import dispatch

from dimos.msgs.geometry_msgs import Pose
from dimos.msgs.std_msgs import Header

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs import PointCloud2


class MapMetaData(LCMMapMetaData):
    """Convenience wrapper for MapMetaData with sensible defaults."""

    @dispatch
    def __init__(self) -> None:
        """Initialize with default values."""
        super().__init__()
        self.map_load_time = Header().stamp
        self.resolution = 0.05
        self.width = 0
        self.height = 0
        self.origin = Pose()

    @dispatch
    def __init__(self, resolution: float, width: int, height: int, origin: Pose) -> None:
        """Initialize with specified values."""
        super().__init__()
        self.map_load_time = Header().stamp
        self.resolution = resolution
        self.width = width
        self.height = height
        self.origin = origin


class OccupancyGrid(LCMOccupancyGrid):
    """
    Convenience wrapper for nav_msgs/OccupancyGrid with numpy array support.

    Cell values:
      - 0: Free space
      - 1-99: Occupied space (higher values = higher cost)
      - 100: Lethal obstacle
      - -1: Unknown
    """

    msg_name = "nav_msgs.OccupancyGrid"

    # Store the numpy array as a property
    _grid_array: Optional[np.ndarray] = None

    @dispatch
    def __init__(self) -> None:
        """Initialize an empty OccupancyGrid."""
        super().__init__()
        self.header = Header("world")  # Header takes frame_id as positional arg
        self.info = MapMetaData()
        self.data_length = 0
        self.data = []
        self._grid_array = np.array([], dtype=np.int8)

    @dispatch
    def __init__(
        self, width: int, height: int, resolution: float = 0.05, frame_id: str = "world"
    ) -> None:
        """Initialize with specified dimensions, all cells unknown (-1)."""
        super().__init__()
        self.header = Header(frame_id)  # Header takes frame_id as positional arg
        self.info = MapMetaData(resolution, width, height, Pose())
        self._grid_array = np.full((height, width), -1, dtype=np.int8)
        self._sync_data_from_array()

    @dispatch
    def __init__(
        self,
        grid: np.ndarray,
        resolution: float = 0.05,
        origin: Optional[Pose] = None,
        frame_id: str = "world",
    ) -> None:
        """Initialize from a numpy array.

        Args:
            grid: 2D numpy array of int8 values (height x width)
            resolution: Grid resolution in meters/cell
            origin: Origin pose of the grid (default: Pose at 0,0,0)
            frame_id: Reference frame (default: "world")
        """
        super().__init__()
        if grid.ndim != 2:
            raise ValueError("Grid must be a 2D array")

        height, width = grid.shape
        self.header = Header(frame_id)  # Header takes frame_id as positional arg
        self.info = MapMetaData(resolution, width, height, origin or Pose())
        self._grid_array = grid.astype(np.int8)
        self._sync_data_from_array()

    @dispatch
    def __init__(self, lcm_msg: LCMOccupancyGrid) -> None:
        """Initialize from an LCM OccupancyGrid message."""
        super().__init__()
        # Create Header from LCM header
        self.header = Header(lcm_msg.header)
        # Use the LCM info directly - it will have the right types
        self.info = lcm_msg.info
        self.data_length = lcm_msg.data_length
        self.data = lcm_msg.data
        self._sync_array_from_data()

    def _sync_data_from_array(self) -> None:
        """Sync the flat data list from the numpy array."""
        if self._grid_array is not None:
            flat_data = self._grid_array.flatten()
            self.data_length = len(flat_data)
            self.data = flat_data.tolist()

    def _sync_array_from_data(self) -> None:
        """Sync the numpy array from the flat data list."""
        if self.data and self.info.width > 0 and self.info.height > 0:
            self._grid_array = np.array(self.data, dtype=np.int8).reshape(
                (self.info.height, self.info.width)
            )
        else:
            self._grid_array = np.array([], dtype=np.int8)

    @property
    def grid(self) -> np.ndarray:
        """Get the grid as a 2D numpy array (height x width)."""
        if self._grid_array is None:
            self._sync_array_from_data()
        return self._grid_array

    @grid.setter
    def grid(self, value: np.ndarray) -> None:
        """Set the grid from a 2D numpy array."""
        if value.ndim != 2:
            raise ValueError("Grid must be a 2D array")
        self._grid_array = value.astype(np.int8)
        self.info.height, self.info.width = value.shape
        self._sync_data_from_array()

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

    def world_to_grid(self, x: float, y: float) -> tuple[int, int]:
        """Convert world coordinates to grid indices.

        Args:
            x: World X coordinate
            y: World Y coordinate

        Returns:
            (grid_x, grid_y) indices
        """
        # Get origin position and orientation
        ox = self.origin.position.x
        oy = self.origin.position.y

        # For now, assume no rotation (simplified)
        # TODO: Handle rotation from quaternion
        dx = x - ox
        dy = y - oy

        grid_x = int(dx / self.resolution)
        grid_y = int(dy / self.resolution)

        return grid_x, grid_y

    def grid_to_world(self, grid_x: int, grid_y: int) -> tuple[float, float]:
        """Convert grid indices to world coordinates.

        Args:
            grid_x: Grid X index
            grid_y: Grid Y index

        Returns:
            (x, y) world coordinates
        """
        # Get origin position
        ox = self.origin.position.x
        oy = self.origin.position.y

        # Convert to world (simplified, no rotation)
        x = ox + grid_x * self.resolution
        y = oy + grid_y * self.resolution

        return x, y

    def get_value(self, x: float, y: float) -> Optional[int]:
        """Get the value at world coordinates.

        Args:
            x: World X coordinate
            y: World Y coordinate

        Returns:
            Cell value or None if out of bounds
        """
        grid_x, grid_y = self.world_to_grid(x, y)

        if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
            return int(self.grid[grid_y, grid_x])
        return None

    def set_value(self, x: float, y: float, value: int) -> bool:
        """Set the value at world coordinates.

        Args:
            x: World X coordinate
            y: World Y coordinate
            value: Cell value to set

        Returns:
            True if successful, False if out of bounds
        """
        grid_x, grid_y = self.world_to_grid(x, y)

        if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
            self.grid[grid_y, grid_x] = value
            self._sync_data_from_array()
            return True
        return False

    def __str__(self) -> str:
        """Create a concise string representation."""
        origin_pos = self.origin.position

        parts = [
            f"▦ OccupancyGrid[{self.header.frame_id}]",
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
            f"resolution={self.resolution}, frame_id='{self.header.frame_id}', "
            f"occupied={self.occupied_cells}, free={self.free_cells}, "
            f"unknown={self.unknown_cells})"
        )

    def lcm_encode(self) -> bytes:
        """Encode OccupancyGrid to LCM bytes."""
        # Ensure data is synced from numpy array
        self._sync_data_from_array()

        # Create LCM message
        lcm_msg = LCMOccupancyGrid()

        # Copy header
        lcm_msg.header.stamp.sec = self.header.stamp.sec
        lcm_msg.header.stamp.nsec = self.header.stamp.nsec
        lcm_msg.header.frame_id = self.header.frame_id

        # Copy map metadata
        lcm_msg.info = self.info

        # Copy data
        lcm_msg.data_length = self.data_length
        lcm_msg.data = self.data

        return lcm_msg.lcm_encode()

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> "OccupancyGrid":
        """Decode LCM bytes to OccupancyGrid."""
        lcm_msg = LCMOccupancyGrid.lcm_decode(data)
        return cls(lcm_msg)

    @classmethod
    def from_pointcloud(
        cls,
        cloud: "PointCloud2",
        resolution: float = 0.05,
        min_height: float = 0.1,
        max_height: float = 2.0,
        inflate_radius: float = 0.0,
        frame_id: Optional[str] = None,
        mark_free_radius: float = 0.0,
    ) -> "OccupancyGrid":
        """Create an OccupancyGrid from a PointCloud2 message.

        Args:
            cloud: PointCloud2 message containing 3D points
            resolution: Grid resolution in meters/cell (default: 0.05)
            min_height: Minimum height threshold for including points (default: 0.1)
            max_height: Maximum height threshold for including points (default: 2.0)
            inflate_radius: Radius in meters to inflate obstacles (default: 0.0)
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
            return cls(1, 1, resolution, frame_id or cloud.frame_id)

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
        padding = max(1.0, inflate_radius * 2)  # At least 1 meter padding
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

        # Apply inflation if requested
        if inflate_radius > 0:
            # Calculate inflation radius in cells
            inflate_cells = int(np.ceil(inflate_radius / resolution))

            # Create a circular kernel for inflation
            y, x = np.ogrid[-inflate_cells : inflate_cells + 1, -inflate_cells : inflate_cells + 1]
            kernel = x**2 + y**2 <= inflate_cells**2

            # Find all occupied cells
            occupied_y, occupied_x = np.where(grid == 100)

            # Inflate around each occupied cell
            for oy, ox in zip(occupied_y, occupied_x):
                # Calculate kernel bounds
                y_min = max(0, oy - inflate_cells)
                y_max = min(height, oy + inflate_cells + 1)
                x_min = max(0, ox - inflate_cells)
                x_max = min(width, ox + inflate_cells + 1)

                # Calculate kernel slice
                kernel_y_min = max(0, inflate_cells - oy)
                kernel_y_max = kernel_y_min + (y_max - y_min)
                kernel_x_min = max(0, inflate_cells - ox)
                kernel_x_max = kernel_x_min + (x_max - x_min)

                # Apply inflation (only to free cells, not obstacles)
                mask = kernel[kernel_y_min:kernel_y_max, kernel_x_min:kernel_x_max]
                region = grid[y_min:y_max, x_min:x_max]

                # Set inflation cost (99) for free cells within radius
                inflated = (region == 0) & mask
                region[inflated] = 99

        # Create and return OccupancyGrid
        occupancy_grid = cls(grid, resolution, origin, frame_id or cloud.frame_id)

        # Update timestamp to match the cloud
        if hasattr(cloud, "ts") and cloud.ts is not None:
            occupancy_grid.header.stamp.sec = int(cloud.ts)
            occupancy_grid.header.stamp.nsec = int((cloud.ts - int(cloud.ts)) * 1e9)

        return occupancy_grid

    def gradient(self, obstacle_threshold: int = 50, max_distance: float = 5.0) -> "OccupancyGrid":
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
        from scipy.ndimage import distance_transform_edt

        # Remember which cells are unknown
        unknown_mask = self.grid == -1

        # Create a working grid where unknown cells are treated as free for distance calculation
        working_grid = self.grid.copy()
        working_grid[unknown_mask] = 0  # Treat unknown as free for gradient computation

        # Create binary obstacle map from working grid
        # Consider cells >= threshold as obstacles (1), everything else as free (0)
        obstacle_map = (working_grid >= obstacle_threshold).astype(np.float32)

        # Compute distance transform (distance to nearest obstacle in cells)
        distance_cells = distance_transform_edt(1 - obstacle_map)

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
            gradient_data,
            resolution=self.resolution,
            origin=self.origin,
            frame_id=self.header.frame_id,
        )

        # Copy timestamp
        gradient_grid.header.stamp = self.header.stamp

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
            new_grid, resolution=self.resolution, origin=self.origin, frame_id=self.header.frame_id
        )

        # Copy timestamp
        filtered.header.stamp = self.header.stamp

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
            new_grid, resolution=self.resolution, origin=self.origin, frame_id=self.header.frame_id
        )

        # Copy timestamp
        filtered.header.stamp = self.header.stamp

        return filtered

    def max(self) -> "OccupancyGrid":
        """Create a new OccupancyGrid with all non-unknown cells set to maximum value.

        Returns:
            New OccupancyGrid where:
            - All non-unknown cells: set to 100 (lethal obstacle)
            - Unknown cells (-1): preserved
        """
        new_grid = self.grid.copy()

        # Set all non-unknown cells to max
        new_grid[new_grid != -1] = 100

        # Create new OccupancyGrid
        maxed = OccupancyGrid(
            new_grid, resolution=self.resolution, origin=self.origin, frame_id=self.header.frame_id
        )

        # Copy timestamp
        maxed.header.stamp = self.header.stamp

        return maxed
