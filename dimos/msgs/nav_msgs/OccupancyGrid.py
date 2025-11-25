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

from enum import IntEnum
import time
from typing import TYPE_CHECKING, BinaryIO

from dimos_lcm.nav_msgs import (  # type: ignore[import-untyped]
    MapMetaData,
    OccupancyGrid as LCMOccupancyGrid,
)
from dimos_lcm.std_msgs import Time as LCMTime  # type: ignore[import-untyped]
import numpy as np
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
    grid: np.ndarray  # type: ignore[type-arg]

    def __init__(
        self,
        grid: np.ndarray | None = None,  # type: ignore[type-arg]
        width: int | None = None,
        height: int | None = None,
        resolution: float = 0.05,
        origin: Pose | None = None,
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

        self.frame_id = frame_id
        self.ts = ts if ts != 0 else time.time()

        if grid is not None:
            # Initialize from numpy array
            if grid.ndim != 2:
                raise ValueError("Grid must be a 2D array")
            height, width = grid.shape
            self.info = MapMetaData(
                map_load_time=self._to_lcm_time(),  # type: ignore[no-untyped-call]
                resolution=resolution,
                width=width,
                height=height,
                origin=origin or Pose(),
            )
            self.grid = grid.astype(np.int8)
        elif width is not None and height is not None:
            # Initialize with dimensions
            self.info = MapMetaData(
                map_load_time=self._to_lcm_time(),  # type: ignore[no-untyped-call]
                resolution=resolution,
                width=width,
                height=height,
                origin=origin or Pose(),
            )
            self.grid = np.full((height, width), -1, dtype=np.int8)
        else:
            # Initialize empty
            self.info = MapMetaData(map_load_time=self._to_lcm_time())  # type: ignore[no-untyped-call]
            self.grid = np.array([], dtype=np.int8)

    def _to_lcm_time(self):  # type: ignore[no-untyped-def]
        """Convert timestamp to LCM Time."""

        s = int(self.ts)
        return LCMTime(sec=s, nsec=int((self.ts - s) * 1_000_000_000))

    @property
    def width(self) -> int:
        """Width of the grid in cells."""
        return self.info.width  # type: ignore[no-any-return]

    @property
    def height(self) -> int:
        """Height of the grid in cells."""
        return self.info.height  # type: ignore[no-any-return]

    @property
    def resolution(self) -> float:
        """Grid resolution in meters/cell."""
        return self.info.resolution  # type: ignore[no-any-return]

    @property
    def origin(self) -> Pose:
        """Origin pose of the grid."""
        return self.info.origin  # type: ignore[no-any-return]

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

    def inflate(self, radius: float) -> OccupancyGrid:
        """Inflate obstacles by a given radius (binary inflation).
        Args:
            radius: Inflation radius in meters
        Returns:
            New OccupancyGrid with inflated obstacles
        """
        # Convert radius to grid cells
        cell_radius = int(np.ceil(radius / self.resolution))

        # Get grid as numpy array
        grid_array = self.grid

        # Create circular kernel for binary inflation
        2 * cell_radius + 1
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

        return lcm_msg.lcm_encode()  # type: ignore[no-any-return]

    @classmethod
    def lcm_decode(cls, data: bytes | BinaryIO) -> OccupancyGrid:
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
            return cls(
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
            return cls(
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

        occupancy_grid = cls(
            grid=grid,
            resolution=resolution,
            origin=origin,
            frame_id=frame_id or cloud.frame_id,
            ts=ts,
        )

        return occupancy_grid

    def gradient(self, obstacle_threshold: int = 50, max_distance: float = 2.0) -> OccupancyGrid:
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
        unknown_mask = self.grid == CostValues.UNKNOWN

        # Create binary obstacle map
        # Consider cells >= threshold as obstacles (1), everything else as free (0)
        # Unknown cells are not considered obstacles for distance calculation
        obstacle_map = (self.grid >= obstacle_threshold).astype(np.float32)

        # Compute distance transform (distance to nearest obstacle in cells)
        # Unknown cells are treated as if they don't exist for distance calculation
        distance_cells = ndimage.distance_transform_edt(1 - obstacle_map)

        # Convert to meters and clip to max distance
        distance_meters = np.clip(distance_cells * self.resolution, 0, max_distance)  # type: ignore[operator]

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
            resolution=self.resolution,
            origin=self.origin,
            frame_id=self.frame_id,
            ts=self.ts,
        )

        return gradient_grid

    def filter_above(self, threshold: int) -> OccupancyGrid:
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

    def filter_below(self, threshold: int) -> OccupancyGrid:
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

    def max(self) -> OccupancyGrid:
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
