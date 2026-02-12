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

from __future__ import annotations

from enum import IntEnum
from functools import lru_cache
import time
from typing import TYPE_CHECKING, BinaryIO

from dimos_lcm.nav_msgs import (
    MapMetaData,
    OccupancyGrid as LCMOccupancyGrid,
)
from dimos_lcm.std_msgs import Time as LCMTime  # type: ignore[import-untyped]
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

from dimos.msgs.geometry_msgs import Pose, Vector3, VectorLike
from dimos.types.timestamped import Timestamped


@lru_cache(maxsize=16)
def _get_matplotlib_cmap(name: str):  # type: ignore[no-untyped-def]
    """Get a matplotlib colormap by name (cached for performance)."""
    return plt.get_cmap(name)


if TYPE_CHECKING:
    from pathlib import Path

    from numpy.typing import NDArray
    from rerun._baseclasses import Archetype


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
    grid: NDArray[np.int8]

    def __init__(
        self,
        grid: NDArray[np.int8] | None = None,
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

    @classmethod
    def from_path(cls, path: Path) -> OccupancyGrid:
        match path.suffix.lower():
            case ".npy":
                return cls(grid=np.load(path))
            case ".png":
                img = Image.open(path).convert("L")
                return cls(grid=np.array(img).astype(np.int8))
            case _:
                raise NotImplementedError(f"Unsupported file format: {path.suffix}")

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

    def copy(self) -> OccupancyGrid:
        """Create a deep copy of the OccupancyGrid.

        Returns:
            A new OccupancyGrid instance with copied data.
        """
        return OccupancyGrid(
            grid=self.grid.copy(),
            resolution=self.resolution,
            origin=self.origin,
            frame_id=self.frame_id,
            ts=self.ts,
        )

    def cell_value(self, world_position: Vector3) -> int:
        grid_position = self.world_to_grid(world_position)
        x = int(grid_position.x)
        y = int(grid_position.y)

        if not (0 <= x < self.width and 0 <= y < self.height):
            return CostValues.UNKNOWN

        return int(self.grid[y, x])

    def _generate_rgba_texture(
        self,
        colormap: str | None = None,
        opacity: float = 1.0,
        cost_range: tuple[int, int] | None = None,
        background: str | None = None,
    ) -> NDArray[np.uint8]:
        """Generate RGBA texture for the occupancy grid.

        Args:
            colormap: Optional matplotlib colormap name.
            opacity: Blend factor (0.0 to 1.0). Blends towards background color.
            cost_range: Optional (min, max) cost range. Cells outside range use background.
            background: Hex color for background (e.g. "#484981"). Default is black.

        Returns:
            RGBA numpy array of shape (height, width, 4).
            Note: NOT flipped - caller handles orientation.
        """
        # Parse background hex to RGB
        if background is not None:
            bg = background.lstrip("#")
            bg_rgb = np.array([int(bg[i : i + 2], 16) for i in (0, 2, 4)], dtype=np.float32)
        else:
            bg_rgb = np.array([0, 0, 0], dtype=np.float32)

        # Determine which cells are in range (if cost_range specified)
        if cost_range is not None:
            in_range_mask = (self.grid >= cost_range[0]) & (self.grid <= cost_range[1])
        else:
            in_range_mask = None

        if colormap is not None:
            cmap = _get_matplotlib_cmap(colormap)
            grid_float = self.grid.astype(np.float32)

            vis = np.zeros((self.height, self.width, 4), dtype=np.uint8)

            free_mask = self.grid == 0
            occupied_mask = self.grid > 0

            if np.any(free_mask):
                fg = np.array(cmap(0.0)[:3]) * 255
                blended = fg * opacity + bg_rgb * (1 - opacity)
                vis[free_mask, :3] = blended.astype(np.uint8)
                vis[free_mask, 3] = 255

            if np.any(occupied_mask):
                costs = grid_float[occupied_mask]
                cost_norm = 0.5 + (costs / 100) * 0.5
                fg = cmap(cost_norm)[:, :3] * 255
                blended = fg * opacity + bg_rgb * (1 - opacity)
                vis[occupied_mask, :3] = blended.astype(np.uint8)
                vis[occupied_mask, 3] = 255

            # Unknown cells: always black
            unknown_mask = self.grid == -1
            vis[unknown_mask, :3] = 0
            vis[unknown_mask, 3] = 255

            # Apply cost_range filter - set out-of-range cells to background
            if in_range_mask is not None:
                out_of_range = ~in_range_mask & (self.grid != -1)
                vis[out_of_range, :3] = bg_rgb.astype(np.uint8)
                vis[out_of_range, 3] = 255

            return vis

        # Default: Foxglove-style coloring
        vis = np.zeros((self.height, self.width, 4), dtype=np.uint8)

        free_mask = self.grid == 0
        occupied_mask = self.grid > 0

        # Free space: blue-purple #484981, blended with background
        fg_free = np.array([72, 73, 129], dtype=np.float32)
        blended_free = fg_free * opacity + bg_rgb * (1 - opacity)
        vis[free_mask, :3] = blended_free.astype(np.uint8)
        vis[free_mask, 3] = 255

        # Occupied: gradient from blue-purple to black, blended with background
        if np.any(occupied_mask):
            costs = self.grid[occupied_mask].astype(np.float32)
            factor = (1 - costs / 100).clip(0, 1)
            fg_occ = np.column_stack([72 * factor, 73 * factor, 129 * factor])
            blended_occ = fg_occ * opacity + bg_rgb * (1 - opacity)
            vis[occupied_mask, :3] = blended_occ.astype(np.uint8)
            vis[occupied_mask, 3] = 255

        # Unknown cells: always black
        unknown_mask = self.grid == -1
        vis[unknown_mask, :3] = 0
        vis[unknown_mask, 3] = 255

        # Apply cost_range filter - set out-of-range cells to background
        if in_range_mask is not None:
            out_of_range = ~in_range_mask & (self.grid != -1)
            vis[out_of_range, :3] = bg_rgb.astype(np.uint8)
            vis[out_of_range, 3] = 255

        return vis

    def to_rerun(
        self,
        colormap: str | None = None,
        z_offset: float = 0.01,
        opacity: float = 1.0,
        cost_range: tuple[int, int] | None = None,
        background: str | None = None,
    ) -> Archetype:
        """Convert to 3D textured mesh overlay on floor plane.

        Uses a single quad with the occupancy grid as a texture.
        Much more efficient than per-cell quads (4 vertices vs n_cells*4).
        """
        import rerun as rr

        if self.grid.size == 0:
            return rr.Mesh3D(vertex_positions=[])

        # Generate RGBA texture and flip to match world coordinates
        # Grid row 0 is at world y=origin (bottom), but texture row 0 is at UV v=0 (top)
        rgba = np.flipud(self._generate_rgba_texture(colormap, opacity, cost_range, background))

        # Single quad covering entire grid
        ox = self.origin.position.x
        oy = self.origin.position.y
        w = self.width * self.resolution
        h = self.height * self.resolution

        vertices = np.array(
            [
                [ox, oy, z_offset],  # 0: bottom-left (world)
                [ox + w, oy, z_offset],  # 1: bottom-right
                [ox + w, oy + h, z_offset],  # 2: top-right
                [ox, oy + h, z_offset],  # 3: top-left
            ],
            dtype=np.float32,
        )

        indices = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.uint32)

        # UV coords: Rerun uses top-left origin for textures
        # Grid row 0 is at world y=oy (bottom), row H-1 at y=oy+h (top)
        # Texture row 0 = grid row 0, so:
        #   world bottom (v0,v1) -> texture v=1 (bottom of texture)
        #   world top (v2,v3) -> texture v=0 (top of texture)
        texcoords = np.array(
            [
                [0.0, 1.0],  # v0: bottom-left world -> bottom-left tex
                [1.0, 1.0],  # v1: bottom-right world -> bottom-right tex
                [1.0, 0.0],  # v2: top-right world -> top-right tex
                [0.0, 0.0],  # v3: top-left world -> top-left tex
            ],
            dtype=np.float32,
        )

        return rr.Mesh3D(
            vertex_positions=vertices,
            triangle_indices=indices,
            vertex_texcoords=texcoords,
            albedo_texture=rgba,
        )
