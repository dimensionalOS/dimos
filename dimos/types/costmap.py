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
import base64
import pickle
import math
import numpy as np
from typing import Optional
from scipy import ndimage
from nav_msgs.msg import OccupancyGrid
from dimos.types.vector import Vector, VectorLike, x, y, to_vector
import open3d as o3d

DTYPE2STR = {
    np.float32: "f32",
    np.float64: "f64",
    np.int32: "i32",
    np.int8: "i8",
}

STR2DTYPE = {v: k for k, v in DTYPE2STR.items()}


def encode_ndarray(arr: np.ndarray, compress: bool = False):
    arr_c = np.ascontiguousarray(arr)
    payload = arr_c.tobytes()
    b64 = base64.b64encode(payload).decode("ascii")

    return {
        "type": "grid",
        "shape": arr_c.shape,
        "dtype": DTYPE2STR[arr_c.dtype.type],
        "data": b64,
    }


class Costmap:
    """Class to hold ROS OccupancyGrid data."""

    def __init__(
        self,
        grid: np.ndarray,
        origin: VectorLike,
        origin_theta: float = 0,
        resolution: float = 0.05,
    ):
        """Initialize Costmap with its core attributes."""
        self.grid = grid
        self.resolution = resolution
        self.origin = to_vector(origin).to_2d()
        self.origin_theta = origin_theta
        self.width = self.grid.shape[1]
        self.height = self.grid.shape[0]

    def serialize(self) -> tuple:
        """Serialize the Costmap instance to a tuple."""
        return {
            "type": "costmap",
            "grid": encode_ndarray(self.grid),
            "origin": self.origin.serialize(),
            "resolution": self.resolution,
            "origin_theta": self.origin_theta,
        }

    @classmethod
    def from_msg(cls, costmap_msg: OccupancyGrid) -> "Costmap":
        """Create a Costmap instance from a ROS OccupancyGrid message."""
        if costmap_msg is None:
            raise Exception("need costmap msg")

        # Extract info from the message
        width = costmap_msg.info.width
        height = costmap_msg.info.height
        resolution = costmap_msg.info.resolution

        # Get origin position as a vector-like object
        origin = (
            costmap_msg.info.origin.position.x,
            costmap_msg.info.origin.position.y,
        )

        # Calculate orientation from quaternion
        qx = costmap_msg.info.origin.orientation.x
        qy = costmap_msg.info.origin.orientation.y
        qz = costmap_msg.info.origin.orientation.z
        qw = costmap_msg.info.origin.orientation.w
        origin_theta = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

        # Convert to numpy array
        data = np.array(costmap_msg.data, dtype=np.int8)
        grid = data.reshape((height, width))

        return cls(
            grid=grid,
            resolution=resolution,
            origin=origin,
            origin_theta=origin_theta,
        )

    def save_pickle(self, pickle_path: str):
        """Save costmap to a pickle file.

        Args:
            pickle_path: Path to save the pickle file
        """
        with open(pickle_path, "wb") as f:
            pickle.dump(self, f)

    @classmethod
    def from_pickle(cls, pickle_path: str) -> "Costmap":
        """Load costmap from a pickle file containing a ROS OccupancyGrid message."""
        with open(pickle_path, "rb") as f:
            data = pickle.load(f)
            costmap = cls(*data)
        return costmap

    @classmethod
    def create_empty(
        cls, width: int = 100, height: int = 100, resolution: float = 0.1
    ) -> "Costmap":
        """Create an empty costmap with specified dimensions."""
        return cls(
            grid=np.zeros((height, width), dtype=np.int8),
            resolution=resolution,
            origin=(0.0, 0.0),
            origin_theta=0.0,
        )

    def world_to_grid(self, point: VectorLike) -> Vector:
        """Convert world coordinates to grid coordinates.

        Args:
            point: A vector-like object containing X,Y coordinates

        Returns:
            Tuple of (grid_x, grid_y) as integers
        """
        return (to_vector(point) - self.origin) / self.resolution

    def grid_to_world(self, grid_point: VectorLike) -> Vector:
        return to_vector(grid_point) * self.resolution + self.origin

    def is_occupied(self, point: VectorLike, threshold: int = 50) -> bool:
        """Check if a position in world coordinates is occupied.

        Args:
            point: Vector-like object containing X,Y coordinates
            threshold: Cost threshold above which a cell is considered occupied (0-100)

        Returns:
            True if position is occupied or out of bounds, False otherwise
        """
        grid_point = self.world_to_grid(point)
        grid_x, grid_y = int(grid_point.x), int(grid_point.y)
        if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
            # Consider unknown (-1) as unoccupied for navigation purposes
            value = self.grid[grid_y, grid_x]
            return value >= threshold
        return True  # Consider out-of-bounds as occupied

    def get_value(self, point: VectorLike) -> Optional[int]:
        point = self.world_to_grid(point)

        if 0 <= point.x < self.width and 0 <= point.y < self.height:
            return int(self.grid[point.y, point.x])
        return None

    def set_value(self, point: VectorLike, value: int = 0) -> bool:
        point = self.world_to_grid(point)

        if 0 <= point.x < self.width and 0 <= point.y < self.height:
            self.grid[point.y, point.x] = value
            return value
        return False

    def smudge(
        self,
        kernel_size: int = 7,
        iterations: int = 25,
        decay_factor: float = 0.9,
        threshold: int = 90,
        preserve_unknown: bool = False,
    ) -> "Costmap":
        """
        Creates a new costmap with expanded obstacles (smudged).

        Args:
            kernel_size: Size of the convolution kernel for dilation (must be odd)
            iterations: Number of dilation iterations
            decay_factor: Factor to reduce cost as distance increases (0.0-1.0)
            threshold: Minimum cost value to consider as an obstacle for expansion
            preserve_unknown: Whether to keep unknown (-1) cells as unknown

        Returns:
            A new Costmap instance with expanded obstacles
        """
        # Make sure kernel size is odd
        if kernel_size % 2 == 0:
            kernel_size += 1

        # Create a copy of the grid for processing
        grid_copy = self.grid.copy()

        # Create a mask of unknown cells if needed
        unknown_mask = None
        if preserve_unknown:
            unknown_mask = grid_copy == -1
            # Temporarily replace unknown cells with 0 for processing
            # This allows smudging to go over unknown areas
            grid_copy[unknown_mask] = 0

        # Create a mask of cells that are above the threshold
        obstacle_mask = grid_copy >= threshold

        # Create a binary map of obstacles
        binary_map = obstacle_mask.astype(np.uint8) * 100

        # Create a circular kernel for dilation (instead of square)
        y, x = np.ogrid[
            -kernel_size // 2 : kernel_size // 2 + 1,
            -kernel_size // 2 : kernel_size // 2 + 1,
        ]
        kernel = (x * x + y * y <= (kernel_size // 2) * (kernel_size // 2)).astype(np.uint8)

        # Create distance map using dilation
        # Each iteration adds one 'ring' of cells around obstacles
        dilated_map = binary_map.copy()

        # Store each layer of dilation with decreasing values
        layers = []

        # First layer is the original obstacle cells
        layers.append(binary_map.copy())

        for i in range(iterations):
            # Dilate the binary map
            dilated = ndimage.binary_dilation(
                dilated_map > 0, structure=kernel, iterations=1
            ).astype(np.uint8)

            # Calculate the new layer (cells that were just added in this iteration)
            new_layer = (dilated - (dilated_map > 0).astype(np.uint8)) * 100

            # Apply decay factor based on distance from obstacle
            new_layer = new_layer * (decay_factor ** (i + 1))

            # Add to layers list
            layers.append(new_layer)

            # Update dilated map for next iteration
            dilated_map = dilated * 100

        # Combine all layers to create a distance-based cost map
        smudged_map = np.zeros_like(grid_copy)
        for layer in layers:
            # For each cell, keep the maximum value across all layers
            smudged_map = np.maximum(smudged_map, layer)

        # Preserve original obstacles
        smudged_map[obstacle_mask] = grid_copy[obstacle_mask]

        # When preserve_unknown is true, restore all original unknown cells
        # This overlays unknown cells on top of the smudged map
        if preserve_unknown and unknown_mask is not None:
            smudged_map[unknown_mask] = -1

        # Ensure cost values are in valid range (0-100) except for unknown (-1)
        if preserve_unknown:
            valid_cells = ~unknown_mask
            smudged_map[valid_cells] = np.clip(smudged_map[valid_cells], 0, 100)
        else:
            smudged_map = np.clip(smudged_map, 0, 100)

        # Create a new costmap with the smudged grid
        return Costmap(
            grid=smudged_map.astype(np.int8),
            resolution=self.resolution,
            origin=self.origin,
            origin_theta=self.origin_theta,
        )

    @property
    def total_cells(self) -> int:
        return self.width * self.height

    @property
    def occupied_cells(self) -> int:
        return np.sum(self.grid >= 0.1)

    @property
    def unknown_cells(self) -> int:
        return np.sum(self.grid == -1)

    @property
    def free_cells(self) -> int:
        return self.total_cells - self.occupied_cells - self.unknown_cells

    @property
    def free_percent(self) -> float:
        return (self.free_cells / self.total_cells) * 100 if self.total_cells > 0 else 0.0

    @property
    def occupied_percent(self) -> float:
        return (self.occupied_cells / self.total_cells) * 100 if self.total_cells > 0 else 0.0

    @property
    def unknown_percent(self) -> float:
        return (self.unknown_cells / self.total_cells) * 100 if self.total_cells > 0 else 0.0

    def __str__(self) -> str:
        """
        Create a string representation of the Costmap.

        Returns:
            A formatted string with key costmap information
        """

        cell_info = [
            "▦ Costmap",
            f"{self.width}x{self.height}",
            f"({self.width * self.resolution:.1f}x{self.height * self.resolution:.1f}m @",
            f"{1 / self.resolution:.0f}cm res)",
            f"Origin: ({x(self.origin):.2f}, {y(self.origin):.2f})",
            f"▣ {self.occupied_percent:.1f}%",
            f"□ {self.free_percent:.1f}%",
            f"◌ {self.unknown_percent:.1f}%",
        ]

        return " ".join(cell_info)


def _inflate_lethal(costmap: np.ndarray, radius: int, lethal_val: int = 100) -> np.ndarray:
    """Return *costmap* with lethal cells dilated by *radius* grid steps (circular)."""
    if radius <= 0 or not np.any(costmap == lethal_val):
        return costmap

    mask = costmap == lethal_val
    dilated = mask.copy()
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx * dx + dy * dy > radius * radius or (dx == 0 and dy == 0):
                continue
            dilated |= np.roll(mask, shift=(dy, dx), axis=(0, 1))

    out = costmap.copy()
    out[dilated] = lethal_val
    return out


def pointcloud_to_costmap(
    pcd: o3d.geometry.PointCloud,
    *,
    resolution: float = 0.05,
    ground_z: float = 0.0,
    obs_min_height: float = 0.15,
    max_height: Optional[float] = 0.5,
    inflate_radius_m: Optional[float] = None,
    default_unknown: int = -1,
    cost_free: int = 0,
    cost_lethal: int = 100,
) -> tuple[np.ndarray, np.ndarray]:
    """Rasterise *pcd* into a 2-D int8 cost-map with optional obstacle inflation.

    Grid origin is **aligned** to the `resolution` lattice so that when
    `resolution == voxel_size` every voxel centroid lands squarely inside a cell
    (no alternating blank lines).
    """

    pts = np.asarray(pcd.points, dtype=np.float32)
    if pts.size == 0:
        return np.full((1, 1), default_unknown, np.int8), np.zeros(2, np.float32)

    # 0. Ceiling filter --------------------------------------------------------
    if max_height is not None:
        pts = pts[pts[:, 2] <= max_height]
        if pts.size == 0:
            return np.full((1, 1), default_unknown, np.int8), np.zeros(2, np.float32)

    # 1. Bounding box & aligned origin ---------------------------------------
    xy_min = pts[:, :2].min(axis=0)
    xy_max = pts[:, :2].max(axis=0)

    # Align origin to the resolution grid (anchor = 0,0)
    origin = np.floor(xy_min / resolution) * resolution

    # Grid dimensions (inclusive) -------------------------------------------
    Nx, Ny = (np.ceil((xy_max - origin) / resolution).astype(int) + 1).tolist()

    # 2. Bin points ------------------------------------------------------------
    idx_xy = np.floor((pts[:, :2] - origin) / resolution).astype(np.int32)
    np.clip(idx_xy[:, 0], 0, Nx - 1, out=idx_xy[:, 0])
    np.clip(idx_xy[:, 1], 0, Ny - 1, out=idx_xy[:, 1])

    lin = idx_xy[:, 1] * Nx + idx_xy[:, 0]
    z_max = np.full(Nx * Ny, -np.inf, np.float32)
    np.maximum.at(z_max, lin, pts[:, 2])
    z_max = z_max.reshape(Ny, Nx)

    # 3. Cost rules -----------------------------------------------------------
    costmap = np.full_like(z_max, default_unknown, np.int8)
    known = z_max != -np.inf
    costmap[known] = cost_free

    lethal = z_max >= (ground_z + obs_min_height)
    costmap[lethal] = cost_lethal

    # 4. Optional inflation ----------------------------------------------------
    if inflate_radius_m and inflate_radius_m > 0:
        cells = int(np.ceil(inflate_radius_m / resolution))
        costmap = _inflate_lethal(costmap, cells, lethal_val=cost_lethal)

    return costmap, origin.astype(np.float32)


if __name__ == "__main__":
    costmap = Costmap.from_pickle("costmapMsg.pickle")
    print(costmap)

    # Create a smudged version of the costmap for better planning
    smudged_costmap = costmap.smudge(
        kernel_size=10, iterations=10, threshold=80, preserve_unknown=False
    )

    print(costmap)
