import base64
import pickle
import numpy as np
from typing import Optional
from scipy import ndimage
from dimos.types.vector import Vector, VectorLike, x, y, to_vector
import open3d as o3d
from matplotlib import cm  # any matplotlib colormap

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
        resolution: float = 0.05,
    ):
        """Initialize Costmap with its core attributes."""
        self.grid = grid
        self.resolution = resolution
        self.origin = to_vector(origin).to_2d()
        self.width = self.grid.shape[1]
        self.height = self.grid.shape[0]

    def serialize(self) -> dict:
        """Serialize the Costmap instance to a dictionary."""
        return {
            "type": "costmap",
            "grid": encode_ndarray(self.grid),
            "origin": self.origin.serialize(),
            "resolution": self.resolution,
        }

    def save_pickle(self, pickle_path: str):
        """Save costmap to a pickle file.

        Args:
            pickle_path: Path to save the pickle file
        """
        with open(pickle_path, "wb") as f:
            pickle.dump(self, f)

    @classmethod
    def create_empty(
        cls, width: int = 100, height: int = 100, resolution: float = 0.1
    ) -> "Costmap":
        """Create an empty costmap with specified dimensions."""
        return cls(
            grid=np.zeros((height, width), dtype=np.int8),
            resolution=resolution,
            origin=(0.0, 0.0),
        )

    def world_to_grid(self, point: VectorLike) -> Vector:
        """Convert world coordinates to grid coordinates.

        Args:
            point: A vector-like object containing X,Y coordinates

        Returns:
            Vector containing grid_x and grid_y coordinates
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
        grid_pos = self.world_to_grid(point)

        if 0 <= grid_pos.x < self.width and 0 <= grid_pos.y < self.height:
            # Consider unknown (-1) as unoccupied for navigation purposes
            # Convert to int coordinates for grid indexing
            grid_y, grid_x = int(grid_pos.y), int(grid_pos.x)
            value = self.grid[grid_y, grid_x]
            return bool(value > 0 and value >= threshold)
        return True  # Consider out-of-bounds as occupied

    def get_value(self, point: VectorLike) -> Optional[int]:
        grid_pos = self.world_to_grid(point)

        if 0 <= grid_pos.x < self.width and 0 <= grid_pos.y < self.height:
            grid_y, grid_x = int(grid_pos.y), int(grid_pos.x)
            return int(self.grid[grid_y, grid_x])
        return None

    def set_value(self, point: VectorLike, value: int = 0) -> bool:
        grid_pos = self.world_to_grid(point)

        if 0 <= grid_pos.x < self.width and 0 <= grid_pos.y < self.height:
            grid_y, grid_x = int(grid_pos.y), int(grid_pos.x)
            self.grid[grid_y, grid_x] = value
            return True
        return False

    def smudge(
        self,
        kernel_size: int = 3,
        iterations: int = 20,
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
        if preserve_unknown and unknown_mask is not None:
            valid_cells = ~unknown_mask
            smudged_map[valid_cells] = np.clip(smudged_map[valid_cells], 0, 100)
        else:
            smudged_map = np.clip(smudged_map, 0, 100)

        # Create a new costmap with the smudged grid
        return Costmap(
            grid=smudged_map.astype(np.int8),
            resolution=self.resolution,
            origin=self.origin,
        )

    def __str__(self) -> str:
        """
        Create a string representation of the Costmap.

        Returns:
            A formatted string with key costmap information
        """
        # Calculate occupancy statistics
        total_cells = self.width * self.height
        occupied_cells = np.sum(self.grid >= 0.1)
        unknown_cells = np.sum(self.grid == -1)
        free_cells = total_cells - occupied_cells - unknown_cells

        # Calculate percentages
        occupied_percent = (occupied_cells / total_cells) * 100
        unknown_percent = (unknown_cells / total_cells) * 100
        free_percent = (free_cells / total_cells) * 100

        cell_info = [
            "▦ Costmap",
            f"{self.width}x{self.height}",
            f"({self.width * self.resolution:.1f}x{self.height * self.resolution:.1f}m @",
            f"{1 / self.resolution:.0f}cm res)",
            f"Origin: ({x(self.origin):.2f}, {y(self.origin):.2f})",
            f"▣ {occupied_percent:.1f}%",
            f"□ {free_percent:.1f}%",
            f"◌ {unknown_percent:.1f}%",
        ]

        return " ".join(cell_info)

    @property
    def o3d_geometry(self):
        return self.pointcloud

    @property
    def pointcloud(self, *, res: float = 0.25, origin=(0.0, 0.0), show_unknown: bool = False):
        """
        Visualise a 2-D costmap (int8, −1…100) as an Open3D PointCloud.

        • −1  → ‘unknown’  (optionally drawn as mid-grey, or skipped)
        • 0   → free
        • 1-99→ graduated cost (turbo colour-ramp)
        • 100 → lethal / obstacle (red end of ramp)

        Parameters
        ----------
        res : float
            Cell size in metres.
        origin : (float, float)
            World-space coord of costmap [row0,col0] centre.
        show_unknown : bool
            If true, draw unknown cells in grey; otherwise omit them.
        """
        cost = np.asarray(self.grid, dtype=np.int16)
        if cost.ndim != 2:
            raise ValueError("cost map must be 2-D (H×W)")

        H, W = cost.shape
        ys, xs = np.mgrid[0:H, 0:W]

        # ----------  flatten & mask  --------------------------------------------------
        xs = xs.ravel()
        ys = ys.ravel()
        vals = cost.ravel()

        unknown_mask = vals == -1
        if not show_unknown:
            keep = ~unknown_mask
            xs, ys, vals = xs[keep], ys[keep], vals[keep]

        # ----------  3-D points  ------------------------------------------------------
        xyz = np.column_stack(
            (
                (xs + 0.5) * res + origin[0],  # X
                (ys + 0.5) * res + origin[1],  # Y
                np.zeros_like(xs, dtype=np.float32),  # Z = 0
            )
        )

        # ----------  colours  ---------------------------------------------------------
        rgb = np.empty((len(vals), 3), dtype=np.float32)

        if show_unknown:
            # mid-grey for unknown
            rgb[unknown_mask[~unknown_mask if not show_unknown else slice(None)]] = (
                0.4,
                0.4,
                0.4,
            )

        # normalise valid costs: 0…100 → 0…1
        norm = np.clip(vals.astype(np.float32), 0, 100) / 100.0
        rgb_valid = cm.turbo(norm)[:, :3]  # type: ignore[attr-defined] # strip alpha
        rgb[:] = rgb_valid  # unknown already set if needed

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(xyz)
        pcd.colors = o3d.utility.Vector3dVector(rgb)

        return pcd
