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

from typing import Optional, TypeAlias, TYPE_CHECKING, BinaryIO

import numpy as np
from dimos_lcm.nav_msgs import OccupancyGrid as LCMOccupancyGrid
from dimos_lcm.nav_msgs import MapMetaData as LCMMapMetaData
from plum import dispatch
from scipy import ndimage

from dimos.msgs.std_msgs import Header
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Vector3 import Vector3, VectorLike

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


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


class CostValues:
    """Standard cost values for occupancy grid cells."""

    FREE = 0  # Free space
    UNKNOWN = -1  # Unknown space
    OCCUPIED = 100  # Occupied/lethal space


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

    @classmethod
    def _get_field_type(cls, field_name):
        """Get field type for LCM decoding."""
        if field_name == "header":
            return Header
        elif field_name == "info":
            return MapMetaData
        return None

    @dispatch
    def __init__(self) -> None:
        """Initialize an empty occupancy grid."""
        super().__init__()
        self.header = Header("world")  # Header takes frame_id as positional arg
        self.info = MapMetaData()
        self.data = []
        self.data_length = 0
        self._grid_array = np.array([], dtype=np.int8)

    @dispatch
    def __init__(
        self, width: int, height: int, resolution: float = 0.05, frame_id: str = "world"
    ) -> None:
        """Initialize with specified dimensions, all cells unknown (-1)."""
        super().__init__()
        self.header = Header(frame_id)  # Header takes frame_id as positional arg
        self.info = MapMetaData(resolution, width, height, Pose())
        self._grid_array = np.full((height, width), CostValues.UNKNOWN, dtype=np.int8)
        self._sync_data_from_array()

    @dispatch
    def __init__(
        self,
        grid: np.ndarray,
        resolution: float = 0.05,
        origin: Optional[Pose] = None,
        frame_id: str = "world",
    ) -> None:
        """Initialize from numpy array.

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
    def width(self) -> int:
        """Grid width in cells."""
        return self.info.width

    @property
    def height(self) -> int:
        """Grid height in cells."""
        return self.info.height

    @property
    def resolution(self) -> float:
        """Grid resolution in meters per cell."""
        return self.info.resolution

    @property
    def origin(self) -> Pose:
        """Grid origin pose."""
        return self.info.origin

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

    def world_to_grid(self, x: float | VectorLike, y: Optional[float] = None) -> tuple[int, int]:
        """Convert world coordinates to grid indices.

        Args:
            x: World X coordinate or VectorLike object
            y: World Y coordinate (if x is not VectorLike)

        Returns:
            (grid_x, grid_y) indices
        """
        # Handle VectorLike input
        if y is None:
            vec = Vector3(x)
            x, y = vec.x, vec.y

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

    def grid_to_world(self, grid_x: int, grid_y: int) -> Vector3:
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

        return Vector3(x, y, 0.0)

    def get_value(self, x: float | VectorLike, y: Optional[float] = None) -> Optional[int]:
        """Get the value at world coordinates.

        Args:
            x: World X coordinate or VectorLike object
            y: World Y coordinate (if x is not VectorLike)

        Returns:
            Cell value or None if out of bounds
        """
        grid_x, grid_y = self.world_to_grid(x, y)

        if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
            return int(self.grid[grid_y, grid_x])
        return None

    def set_value(self, x: float | VectorLike, y: float | int, value: Optional[int] = None) -> bool:
        """Set the value at world coordinates.

        Args:
            x: World X coordinate or VectorLike object
            y: World Y coordinate or value (if x is VectorLike)
            value: Cell value to set (if y is coordinate)

        Returns:
            True if successful, False if out of bounds
        """
        # Handle overloaded arguments
        if value is None:
            # x is VectorLike, y is value
            grid_x, grid_y = self.world_to_grid(x)
            value = int(y)
        else:
            # x, y are coordinates, value is value
            grid_x, grid_y = self.world_to_grid(x, y)
            value = int(value)

        if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
            self.grid[grid_y, grid_x] = value
            self._sync_data_from_array()
            return True
        return False

    def is_occupied(
        self, x: float | VectorLike, y: Optional[float] = None, threshold: int = 50
    ) -> bool:
        """Check if a position is occupied.

        Args:
            x: World X coordinate or VectorLike object
            y: World Y coordinate (if x is not VectorLike)
            threshold: Cost threshold above which cell is occupied

        Returns:
            True if occupied or out of bounds
        """
        value = self.get_value(x, y)
        if value is None:
            return True  # Out of bounds is considered occupied
        return value >= threshold

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
        return OccupancyGrid(result_grid, self.resolution, self.origin, self.header.frame_id)

    @classmethod
    def from_pointcloud(
        cls,
        cloud: "PointCloud2",
        resolution: float = 0.05,
        min_height: float = 0.1,
        max_height: float = 2.0,
        inflate_radius: float = 0.0,
        frame_id: Optional[str] = None,
    ) -> "OccupancyGrid":
        """Create an OccupancyGrid from a PointCloud2 message.

        Args:
            cloud: PointCloud2 message containing 3D points
            resolution: Grid resolution in meters/cell (default: 0.05)
            min_height: Minimum height threshold for including points (default: 0.1)
            max_height: Maximum height threshold for including points (default: 2.0)
            inflate_radius: Radius in meters to inflate obstacles (default: 0.0)
            frame_id: Reference frame for the grid (default: uses cloud's frame_id)

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

        # Filter by height
        height_mask = (points[:, 2] >= min_height) & (points[:, 2] <= max_height)
        filtered_points = points[height_mask]

        if len(filtered_points) == 0:
            # No obstacles within height range
            return cls(1, 1, resolution, frame_id or cloud.frame_id)

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
        grid = np.full((height, width), CostValues.UNKNOWN, dtype=np.int8)

        # Convert points to grid indices
        grid_x = ((filtered_points[:, 0] - min_x) / resolution).astype(np.int32)
        grid_y = ((filtered_points[:, 1] - min_y) / resolution).astype(np.int32)

        # Clip indices to grid bounds
        grid_x = np.clip(grid_x, 0, width - 1)
        grid_y = np.clip(grid_y, 0, height - 1)

        # Mark cells as occupied
        grid[grid_y, grid_x] = CostValues.OCCUPIED  # Lethal obstacle

        # Create and return OccupancyGrid
        final_frame_id = frame_id if frame_id is not None else getattr(cloud, "frame_id", "world")
        occupancy_grid = cls(grid, resolution, origin, final_frame_id)

        # Update timestamp to match the cloud
        if hasattr(cloud, "ts") and cloud.ts is not None:
            occupancy_grid.header.stamp.sec = int(cloud.ts)
            occupancy_grid.header.stamp.nsec = int((cloud.ts - int(cloud.ts)) * 1e9)

        # Apply inflation if requested
        if inflate_radius > 0:
            occupancy_grid = occupancy_grid.inflate(inflate_radius)

        return occupancy_grid

    @property
    def total_cells(self) -> int:
        """Total number of cells."""
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
