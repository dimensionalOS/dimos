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

import pickle
import math
import numpy as np
from typing import Optional
from scipy import ndimage
from nav_msgs.msg import OccupancyGrid
from dimos.types.vector import Vector, VectorLike, x, y, to_vector


class Costmap:
    """Class to hold ROS OccupancyGrid data."""

    def __init__(
        self,
        grid: np.ndarray,
        origin_theta: float,
        origin: VectorLike,
        resolution: float = 0.05,
    ):
        """Initialize Costmap with its core attributes."""
        self.grid = grid
        self.resolution = resolution
        self.origin = to_vector(origin)
        self.origin_theta = origin_theta
        self.width = self.grid.shape[1]
        self.height = self.grid.shape[0]

    def serialize(self) -> tuple:
        """Serialize the Costmap instance to a tuple."""
        return pickle.dumps(self)

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
    def create_empty(cls, width: int = 100, height: int = 100, resolution: float = 0.1) -> "Costmap":
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
        grid_x, grid_y = self.world_to_grid(point)
        if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
            # Consider unknown (-1) as unoccupied for navigation purposes
            value = self.grid[grid_y, grid_x]
            return value > 0 and value >= threshold
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
        iterations: int = 10,
        decay_factor: float = 0.8,
        threshold: int = 80,
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
            dilated = ndimage.binary_dilation(dilated_map > 0, structure=kernel, iterations=1).astype(np.uint8)

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

        # When preserve_unknown is true, only restore unknown cells that haven't been smudged
        # This allows smudging to extend over unknown areas
        if preserve_unknown and unknown_mask is not None:
            # Only keep unknown value in cells that weren't affected by obstacle dilation
            unsmudged_unknown = unknown_mask & (smudged_map == 0)
            smudged_map[unsmudged_unknown] = -1

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

    def __str__(self) -> str:
        """
        Create a string representation of the Costmap.

        Returns:
            A formatted string with key costmap information
        """
        # Calculate occupancy statistics
        total_cells = self.width * self.height
        occupied_cells = np.sum(self.grid >= 50)
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


if __name__ == "__main__":
    costmap = Costmap.from_pickle("costmapMsg.pickle")
    print(costmap)

    # Create a smudged version of the costmap for better planning
    smudged_costmap = costmap.smudge(kernel_size=10, iterations=10, threshold=80, preserve_unknown=False)

    print(costmap)
