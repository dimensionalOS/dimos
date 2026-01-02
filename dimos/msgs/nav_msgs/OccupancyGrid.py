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
from typing import TYPE_CHECKING, Any, BinaryIO

from dimos_lcm.nav_msgs import (
    MapMetaData,
    OccupancyGrid as LCMOccupancyGrid,
)
from dimos_lcm.std_msgs import Time as LCMTime  # type: ignore[import-untyped]
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image
import rerun as rr

from dimos.msgs.geometry_msgs import Pose, Vector3, VectorLike
from dimos.types.timestamped import Timestamped


@lru_cache(maxsize=16)
def _get_matplotlib_cmap(name: str):  # type: ignore[no-untyped-def]
    """Get a matplotlib colormap by name (cached for performance)."""
    return plt.get_cmap(name)


if TYPE_CHECKING:
    from pathlib import Path

    from numpy.typing import NDArray


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

    def to_rerun(  # type: ignore[no-untyped-def]
        self,
        colormap: str | None = None,
        mode: str = "image",
        z_offset: float = 0.01,
        **kwargs: Any,
    ):  # type: ignore[no-untyped-def]
        """Convert to Rerun visualization format.

        Args:
            colormap: Optional colormap name (e.g., "RdBu_r" for blue=free, red=occupied).
                     If None, uses grayscale for image mode or default colors for 3D modes.
            mode: Visualization mode:
                - "image": 2D grayscale/colored image (default)
                - "mesh": 3D textured plane overlay on floor
                - "points": 3D points for occupied cells only
            z_offset: Height offset for 3D modes (default 0.01m above floor)
            **kwargs: Additional args (ignored for compatibility)

        Returns:
            Rerun archetype for logging (rr.Image, rr.Mesh3D, or rr.Points3D)

        The visualization uses:
        - Free space (value 0): white/blue
        - Unknown space (value -1): gray/transparent
        - Occupied space (value > 0): black/red with gradient
        """
        if self.grid.size == 0:
            if mode == "image":
                return rr.Image(np.zeros((1, 1), dtype=np.uint8), color_model="L")
            elif mode == "mesh":
                return rr.Mesh3D(vertex_positions=[])
            else:
                return rr.Points3D([])

        if mode == "points":
            return self._to_rerun_points(colormap, z_offset)
        elif mode == "mesh":
            return self._to_rerun_mesh(colormap, z_offset)
        else:
            return self._to_rerun_image(colormap)

    def _to_rerun_image(self, colormap: str | None = None):  # type: ignore[no-untyped-def]
        """Convert to 2D image visualization."""
        # Use existing cached visualization functions for supported palettes
        if colormap in ("turbo", "rainbow"):
            from dimos.mapping.occupancy.visualizations import rainbow_image, turbo_image

            if colormap == "turbo":
                bgr_image = turbo_image(self.grid)
            else:
                bgr_image = rainbow_image(self.grid)

            # Convert BGR to RGB and flip for world coordinates
            rgb_image = np.flipud(bgr_image[:, :, ::-1])
            return rr.Image(rgb_image, color_model="RGB")

        if colormap is not None:
            # Use matplotlib colormap (cached for performance)
            cmap = _get_matplotlib_cmap(colormap)

            grid_float = self.grid.astype(np.float32)

            # Create RGBA image
            vis = np.zeros((self.height, self.width, 4), dtype=np.uint8)

            # Free space: low cost (blue in RdBu_r)
            free_mask = self.grid == 0
            # Occupied: high cost (red in RdBu_r)
            occupied_mask = self.grid > 0
            # Unknown: transparent gray
            unknown_mask = self.grid == -1

            # Map free to 0, costs to normalized value
            if np.any(free_mask):
                colors_free = (cmap(0.0)[:3] * np.array([255, 255, 255])).astype(np.uint8)
                vis[free_mask, :3] = colors_free
                vis[free_mask, 3] = 255

            if np.any(occupied_mask):
                # Normalize costs 1-100 to 0.5-1.0 range
                costs = grid_float[occupied_mask]
                cost_norm = 0.5 + (costs / 100) * 0.5
                colors_occ = (cmap(cost_norm)[:, :3] * 255).astype(np.uint8)
                vis[occupied_mask, :3] = colors_occ
                vis[occupied_mask, 3] = 255

            if np.any(unknown_mask):
                vis[unknown_mask] = [128, 128, 128, 100]  # Semi-transparent gray

            # Flip vertically to match world coordinates (y=0 at bottom)
            return rr.Image(np.flipud(vis), color_model="RGBA")

        # Grayscale visualization (no colormap)
        vis_gray = np.zeros((self.height, self.width), dtype=np.uint8)

        # Free space = white
        vis_gray[self.grid == 0] = 255

        # Unknown = gray
        vis_gray[self.grid == -1] = 128

        # Occupied (100) = black, costs (1-99) = gradient
        occupied_mask = self.grid > 0
        if np.any(occupied_mask):
            # Map 1-100 to 127-0 (darker = more occupied)
            costs = self.grid[occupied_mask].astype(np.float32)
            vis_gray[occupied_mask] = (127 * (1 - costs / 100)).astype(np.uint8)

        # Flip vertically to match world coordinates (y=0 at bottom)
        return rr.Image(np.flipud(vis_gray), color_model="L")

    def _to_rerun_points(self, colormap: str | None = None, z_offset: float = 0.01):  # type: ignore[no-untyped-def]
        """Convert to 3D points for occupied cells."""
        # Find occupied cells (cost > 0)
        occupied_mask = self.grid > 0
        if not np.any(occupied_mask):
            return rr.Points3D([])

        # Get grid coordinates of occupied cells
        gy, gx = np.where(occupied_mask)
        costs = self.grid[occupied_mask].astype(np.float32)

        # Convert to world coordinates
        ox = self.origin.position.x
        oy = self.origin.position.y
        wx = ox + (gx + 0.5) * self.resolution
        wy = oy + (gy + 0.5) * self.resolution
        wz = np.full_like(wx, z_offset)

        points = np.column_stack([wx, wy, wz])

        # Determine colors
        if colormap is not None:
            # Normalize costs to 0-1 range
            cost_norm = costs / 100.0
            cmap = _get_matplotlib_cmap(colormap)
            point_colors = (cmap(cost_norm)[:, :3] * 255).astype(np.uint8)
        else:
            # Default: red gradient based on cost
            intensity = (costs / 100.0 * 255).astype(np.uint8)
            point_colors = np.column_stack(
                [intensity, np.zeros_like(intensity), np.zeros_like(intensity)]
            )

        return rr.Points3D(
            positions=points,
            radii=self.resolution / 2,
            colors=point_colors,
        )

    def _to_rerun_mesh(self, colormap: str | None = None, z_offset: float = 0.01):  # type: ignore[no-untyped-def]
        """Convert to 3D mesh overlay on floor plane.

        Only renders known cells (free or occupied), skipping unknown cells.
        Uses per-vertex colors for proper alpha blending.
        Fully vectorized for performance (~100x faster than loop version).
        """
        # Only render known cells (not unknown = -1)
        known_mask = self.grid != -1
        if not np.any(known_mask):
            return rr.Mesh3D(vertex_positions=[])

        # Get grid coordinates of known cells
        gy, gx = np.where(known_mask)
        n_cells = len(gy)

        ox = self.origin.position.x
        oy = self.origin.position.y
        r = self.resolution

        # === VECTORIZED VERTEX GENERATION ===
        # World positions of cell corners (bottom-left of each cell)
        wx = ox + gx.astype(np.float32) * r
        wy = oy + gy.astype(np.float32) * r

        # Each cell has 4 vertices: (wx,wy), (wx+r,wy), (wx+r,wy+r), (wx,wy+r)
        # Shape: (n_cells, 4, 3)
        vertices = np.zeros((n_cells, 4, 3), dtype=np.float32)
        vertices[:, 0, 0] = wx
        vertices[:, 0, 1] = wy
        vertices[:, 0, 2] = z_offset
        vertices[:, 1, 0] = wx + r
        vertices[:, 1, 1] = wy
        vertices[:, 1, 2] = z_offset
        vertices[:, 2, 0] = wx + r
        vertices[:, 2, 1] = wy + r
        vertices[:, 2, 2] = z_offset
        vertices[:, 3, 0] = wx
        vertices[:, 3, 1] = wy + r
        vertices[:, 3, 2] = z_offset
        # Flatten to (n_cells*4, 3)
        flat_vertices = vertices.reshape(-1, 3)

        # === VECTORIZED INDEX GENERATION ===
        # Base vertex indices for each cell: [0, 4, 8, 12, ...]
        base_v = np.arange(n_cells, dtype=np.uint32) * 4
        # Two triangles per cell: (0,1,2) and (0,2,3) relative to base
        indices = np.zeros((n_cells, 2, 3), dtype=np.uint32)
        indices[:, 0, 0] = base_v
        indices[:, 0, 1] = base_v + 1
        indices[:, 0, 2] = base_v + 2
        indices[:, 1, 0] = base_v
        indices[:, 1, 1] = base_v + 2
        indices[:, 1, 2] = base_v + 3
        # Flatten to (n_cells*2, 3)
        flat_indices = indices.reshape(-1, 3)

        # === VECTORIZED COLOR GENERATION ===
        cell_values = self.grid[gy, gx]  # Get all cell values at once

        if colormap:
            cmap = _get_matplotlib_cmap(colormap)
            # Normalize costs: free(0) -> 0.0, cost(1-100) -> 0.5-1.0
            cost_norm = np.where(cell_values == 0, 0.0, 0.5 + (cell_values / 100) * 0.5)
            # Sample colormap for all cells at once (returns Nx4 RGBA float)
            rgba_float = cmap(cost_norm)[:, :3]  # Drop alpha, we set our own
            rgb = (rgba_float * 255).astype(np.uint8)
            # Alpha: 180 for free, 220 for occupied
            alpha = np.where(cell_values == 0, 180, 220).astype(np.uint8)
        else:
            # Default coloring: dark grey for free, black for occupied
            rgb = np.zeros((n_cells, 3), dtype=np.uint8)
            is_free = cell_values == 0
            # Free space: dark grey
            rgb[is_free] = [40, 40, 40]
            # Occupied: black to dark grey gradient (darker = more occupied)
            intensity = (40 * (1 - cell_values / 100)).astype(np.uint8)
            rgb[~is_free] = np.column_stack([intensity[~is_free]] * 3)
            alpha = np.where(is_free, 150, 200).astype(np.uint8)

        # Combine RGB and alpha into RGBA
        colors_per_cell = np.column_stack([rgb, alpha])  # (n_cells, 4)
        # Repeat each color 4 times (one per vertex)
        colors = np.repeat(colors_per_cell, 4, axis=0)  # (n_cells*4, 4)

        return rr.Mesh3D(
            vertex_positions=flat_vertices,
            triangle_indices=flat_indices,
            vertex_colors=colors,
        )
