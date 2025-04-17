import pickle
import math
import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, Optional, Dict, Any, List, Union
from scipy import ndimage


class Costmap:
    """Class to hold costmap data from ROS OccupancyGrid messages."""

    def __init__(
        self,
        grid: np.ndarray,
        origin_x: float,
        origin_y: float,
        origin_theta: float,
        resolution: float = 0.05,
    ):
        """Initialize Costmap with its core attributes."""
        self.grid = grid
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.origin_theta = origin_theta
        self.width = self.grid.shape[1]
        self.height = self.grid.shape[0]

    @classmethod
    def from_msg(cls, costmap_msg) -> "Costmap":
        """Create a Costmap instance from a ROS OccupancyGrid message."""
        if costmap_msg is None:
            return cls(
                grid=np.zeros((100, 100), dtype=np.int8),
                resolution=0.1,
                origin_x=0.0,
                origin_y=0.0,
                origin_theta=0.0,
            )

        # Extract info from the message
        width = costmap_msg.info.width
        height = costmap_msg.info.height
        resolution = costmap_msg.info.resolution

        origin_x = costmap_msg.info.origin.position.x
        origin_y = costmap_msg.info.origin.position.y

        # Calculate orientation from quaternion
        qx = costmap_msg.info.origin.orientation.x
        qy = costmap_msg.info.origin.orientation.y
        qz = costmap_msg.info.origin.orientation.z
        qw = costmap_msg.info.origin.orientation.w
        origin_theta = math.atan2(
            2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)
        )

        # Convert to numpy array
        data = np.array(costmap_msg.data, dtype=np.int8)
        grid = data.reshape((height, width))

        return cls(
            grid=grid,
            resolution=resolution,
            origin_x=origin_x,
            origin_y=origin_y,
            origin_theta=origin_theta,
        )

    @classmethod
    def from_pickle(cls, pickle_path: str) -> "Costmap":
        """Load costmap from a pickle file containing a ROS OccupancyGrid message."""
        with open(pickle_path, "rb") as f:
            costmap_msg = pickle.load(f)
        return cls.from_msg(costmap_msg)

    @classmethod
    def create_empty(
        cls, width: int = 100, height: int = 100, resolution: float = 0.1
    ) -> "Costmap":
        """Create an empty costmap with specified dimensions."""
        return cls(
            grid=np.zeros((height, width), dtype=np.int8),
            resolution=resolution,
            origin_x=0.0,
            origin_y=0.0,
            origin_theta=0.0,
        )

    # TODO: this needs to use some generic vector ops, does dimos have those?
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to grid coordinates."""
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)
        return grid_x, grid_y

    # TODO: this needs to use some generic vector ops, does dimos have those?
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """Convert grid coordinates to world coordinates."""
        world_x = grid_x * self.resolution + self.origin_x
        world_y = grid_y * self.resolution + self.origin_y
        return world_x, world_y

    def is_occupied(self, x: float, y: float) -> bool:
        """Check if a position in world coordinates is occupied."""
        grid_x, grid_y = self.world_to_grid(x, y)
        if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
            return self.grid[grid_y, grid_x] > 0
        return True  # Consider out-of-bounds as occupied

    def get_value(self, x: float, y: float) -> Optional[int]:
        """Get the value at a position in world coordinates."""
        grid_x, grid_y = self.world_to_grid(x, y)
        if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
            return int(self.grid[grid_y, grid_x])
        return None

    def set_value(self, x: float, y: float, value: int) -> bool:
        """Set the value at a position in world coordinates."""
        grid_x, grid_y = self.world_to_grid(x, y)
        if 0 <= grid_x < self.width and 0 <= grid_y < self.height:
            self.grid[grid_y, grid_x] = value
            return True
        return False

    def smudge(
        self,
        kernel_size: int = 5,
        iterations: int = 10,
        decay_factor: float = 0.8,
        threshold: int = 1,
        preserve_unknown: bool = True,
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
        kernel = (x * x + y * y <= (kernel_size // 2) * (kernel_size // 2)).astype(
            np.uint8
        )

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

        # Restore unknown cells if needed
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
            origin_x=self.origin_x,
            origin_y=self.origin_y,
            origin_theta=self.origin_theta,
        )

    def plot(
        self,
        figsize=(10, 8),
        show_axes=True,
        # cmap="viridis",
        cmap="YlOrRd",
        title="Costmap Visualization",
        show=True,
        unknown_color="lightgray",
        show_grid=True,
        grid_interval=1.0,
        grid_color="black",
        grid_alpha=0.5,
        additional_points: Dict[str, Dict[str, Any]] = {},
    ) -> plt.Figure:
        fig, ax = plt.subplots(figsize=figsize)

        additional_points["origin"] = {
            "positions": [(0, 0)],  # Example position
            "color": "red",
            "marker": "x",
            "size": 100,
        }

        # Create the extent for proper world coordinate mapping
        extent = [
            self.origin_x,
            self.origin_x + self.width * self.resolution,
            self.origin_y,
            self.origin_y + self.height * self.resolution,
        ]

        # Create a masked array to handle unknown (-1) cells separately
        grid_copy = self.grid.copy()
        unknown_mask = grid_copy == -1

        # Create a custom colormap with a specific color for unknown cells
        norm = plt.Normalize(
            vmin=0, vmax=100
        )  # Adjust vmax based on your costmap values

        # Plot the known costs
        masked_grid = np.ma.array(grid_copy, mask=unknown_mask)
        im = ax.imshow(
            masked_grid,
            cmap=cmap,
            norm=norm,
            origin="lower",
            extent=extent,
            interpolation="none",
        )

        # Plot the unknown cells with a different color
        if np.any(unknown_mask):
            unknown_grid = np.ma.array(np.zeros_like(grid_copy), mask=~unknown_mask)
            ax.imshow(
                unknown_grid,
                cmap=plt.matplotlib.colors.ListedColormap([unknown_color]),
                origin="lower",
                extent=extent,
                interpolation="none",
            )

        # Add meter grid overlay
        if show_grid:
            # Calculate grid line positions for X and Y axes
            x_min, x_max = extent[0], extent[1]
            y_min, y_max = extent[2], extent[3]

            # Round to nearest grid_interval for cleaner display
            x_start = math.ceil(x_min / grid_interval) * grid_interval
            y_start = math.ceil(y_min / grid_interval) * grid_interval

            # Draw vertical grid lines (constant x-value)
            x_lines = np.arange(x_start, x_max, grid_interval)
            for x in x_lines:
                ax.axvline(
                    x=x,
                    color=grid_color,
                    linestyle="--",
                    linewidth=0.5,
                    alpha=grid_alpha,
                    zorder=1,
                )

            # Draw horizontal grid lines (constant y-value)
            y_lines = np.arange(y_start, y_max, grid_interval)
            for y in y_lines:
                ax.axhline(
                    y=y,
                    color=grid_color,
                    linestyle="--",
                    alpha=grid_alpha,
                    zorder=1,
                    linewidth=0.5,
                )

            # Add labeled tick marks at grid intervals
            ax.set_xticks(x_lines)
            ax.set_yticks(y_lines)

        # Add colorbar for known cells
        cbar = fig.colorbar(im, ax=ax)
        cbar.set_label("Cost Value (0-100)")

        # Plot additional points if provided
        if additional_points:
            for label, point_data in additional_points.items():
                positions = point_data.get("positions", [])
                color = point_data.get("color", "red")
                marker = point_data.get("marker", "o")
                size = point_data.get("size", 50)

                # Extract x and y coordinates
                x_coords = [pos[0] for pos in positions]
                y_coords = [pos[1] for pos in positions]

                ax.scatter(
                    x_coords, y_coords, c=color, marker=marker, s=size, label=label
                )

        # Set labels and title
        ax.set_xlabel("X (world coordinates)")
        ax.set_ylabel("Y (world coordinates)")
        ax.set_title(title)

        # Hide axes if requested
        if not show_axes:
            ax.set_axis_off()

        # Show the plot
        if show:
            plt.tight_layout()
            plt.show(block=False)

        return fig


if __name__ == "__main__":
    costmap = Costmap.from_pickle("costmapMsg.pickle")
    print(f"Costmap dimensions: {costmap.width}x{costmap.height}")
    print(f"Resolution: {costmap.resolution}")
    print(f"Origin: ({costmap.origin_x}, {costmap.origin_y}, {costmap.origin_theta})")

    # Plot the costmap with special handling for unknown (-1) cells and meter grid
    costmap.plot()
    costmap.smudge(kernel_size=5, iterations=10).plot()

    # block, wait for input and exit

    input("press any key")
