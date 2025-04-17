import matplotlib.pyplot as plt
import numpy as np
from typing import List, Tuple, Dict, Any, Union, Optional
from costmap import Costmap
from vector import Vector
from path import Path
from vectortypes import VectorLike, to_vector


class Drawer:
    """A class to draw costmaps, vectors, and paths together with styling options."""

    def __init__(self, figsize=(10, 8), title="Visualization", show_axes=True):
        """Initialize the drawer with figure settings.

        Args:
            figsize: Tuple of (width, height) for the figure
            title: Title for the plot
            show_axes: Whether to show axes in the plot
        """
        self.fig, self.ax = plt.subplots(figsize=figsize)
        self.ax.set_title(title)

        if not show_axes:
            self.ax.set_axis_off()
        else:
            self.ax.set_xlabel("X (world coordinates)")
            self.ax.set_ylabel("Y (world coordinates)")

        self.costmaps_drawn = 0
        self.paths_drawn = 0
        self.vectors_drawn = 0
        self.points_drawn = 0

        # Default styles
        self.default_styles = {
            "costmap": {
                "cmap": "YlOrRd",
                "show_grid": True,
                "grid_interval": 1.0,
                "grid_color": "black",
                "grid_alpha": 0.3,
                "unknown_color": "lightgray",
                "transparent_unknown": True,
                "transparent_empty": False,
            },
            "path": {
                "color": "#1f77b4",
                "marker": ".",
                "markersize": 0.5,
                "linewidth": 1,
                "linestyle": "dashed",
                "alpha": 1,
                "zorder": 10,
            },
            "vector": {
                "color": "red",
                "marker": "o",
                "markersize": 80,
                "alpha": 0.8,
                "zorder": 20,
                "show_text": False,
            },
            "point": {
                "color": "green",
                "marker": "o",
                "markersize": 8,
                "alpha": 0.8,
                "zorder": 30,
            },
        }

    def draw_costmap(self, costmap: Costmap, style: Dict[str, Any] = None) -> None:
        """Draw a costmap with optional styling.

        Args:
            costmap: The Costmap object to draw
            style: Dictionary of style options to override defaults
                   (cmap, show_grid, grid_interval, grid_color, grid_alpha, unknown_color)
        """
        style = style or {}

        # Merge default style with provided style
        merged_style = self.default_styles["costmap"].copy()
        merged_style.update(style)

        # Extract the extent for proper world coordinate mapping
        extent = [
            costmap.origin.x,
            costmap.origin.x + costmap.width * costmap.resolution,
            costmap.origin.y,
            costmap.origin.y + costmap.height * costmap.resolution,
        ]

        # Create a masked array to handle unknown (-1) cells and optionally empty (0) cells separately
        grid_copy = costmap.grid.copy()
        unknown_mask = grid_copy == -1

        # Additionally mask empty cells if requested
        if merged_style.get("transparent_empty", False):
            empty_mask = grid_copy == 0
            combined_mask = unknown_mask | empty_mask
        else:
            combined_mask = unknown_mask

        # Create a custom colormap with a specific color for unknown cells
        norm = plt.Normalize(vmin=0, vmax=100)

        # Plot the known costs
        masked_grid = np.ma.array(grid_copy, mask=combined_mask)
        im = self.ax.imshow(
            masked_grid,
            cmap=merged_style["cmap"],
            norm=norm,
            origin="lower",
            extent=extent,
            interpolation="none",
            alpha=merged_style.get("alpha", 1.0),
        )

        # Plot the unknown cells as transparent if requested or with a different color
        if np.any(unknown_mask):
            if merged_style.get("transparent_unknown", True):
                # Don't draw unknown cells (they will be transparent)
                pass
            else:
                # Draw unknown cells with the specified color
                unknown_grid = np.ma.array(np.zeros_like(grid_copy), mask=~unknown_mask)
                self.ax.imshow(
                    unknown_grid,
                    cmap=plt.matplotlib.colors.ListedColormap(
                        [merged_style["unknown_color"]]
                    ),
                    origin="lower",
                    extent=extent,
                    interpolation="none",
                )

        # Add meter grid overlay
        if merged_style["show_grid"]:
            # Calculate grid line positions for X and Y axes
            x_min, x_max = extent[0], extent[1]
            y_min, y_max = extent[2], extent[3]

            # Round to nearest grid_interval for cleaner display
            x_start = (
                np.ceil(x_min / merged_style["grid_interval"])
                * merged_style["grid_interval"]
            )
            y_start = (
                np.ceil(y_min / merged_style["grid_interval"])
                * merged_style["grid_interval"]
            )

            # Draw vertical grid lines (constant x-value)
            x_lines = np.arange(x_start, x_max, merged_style["grid_interval"])
            for x in x_lines:
                self.ax.axvline(
                    x=x,
                    color=merged_style["grid_color"],
                    linestyle="--",
                    linewidth=0.5,
                    alpha=merged_style["grid_alpha"],
                    zorder=1,
                )

            # Draw horizontal grid lines (constant y-value)
            y_lines = np.arange(y_start, y_max, merged_style["grid_interval"])
            for y in y_lines:
                self.ax.axhline(
                    y=y,
                    color=merged_style["grid_color"],
                    linestyle="--",
                    alpha=merged_style["grid_alpha"],
                    zorder=1,
                    linewidth=0.5,
                )

            # Add labeled tick marks at grid intervals
            self.ax.set_xticks(x_lines)
            self.ax.set_yticks(y_lines)

        # Add colorbar for known cells if this is the first costmap
        if self.costmaps_drawn == 0:
            cbar = self.fig.colorbar(im, ax=self.ax)
            cbar.set_label("Cost Value (0-100)")

        self.costmaps_drawn += 1

    def draw_path(
        self, path: Union[Path, List[VectorLike]], style: Dict[str, Any] = None
    ) -> None:
        """Draw a path with optional styling.

        Args:
            path: Path object or list of vector-like points
            style: Dictionary of style options to override defaults
                   (color, marker, markersize, linewidth, linestyle, alpha, zorder)
        """
        style = style or {}

        # Merge default style with provided style
        merged_style = self.default_styles["path"].copy()
        merged_style.update(style)

        # Handle different input types
        if isinstance(path, Path):
            points = [p for p in path]
        else:
            points = [to_vector(p) for p in path]

        if not points:
            return

        # Extract x and y coordinates
        x_coords = [p[0] for p in points]
        y_coords = [p[1] for p in points]

        # Plot path line
        self.ax.plot(
            x_coords,
            y_coords,
            color=merged_style["color"],
            linewidth=merged_style["linewidth"],
            linestyle=merged_style["linestyle"],
            alpha=merged_style["alpha"],
            zorder=merged_style["zorder"],
            marker=merged_style["marker"],
            markersize=merged_style["markersize"],
            label=(
                f"Path {self.paths_drawn}"
                if merged_style.get("show_label", False)
                else None
            ),
        )

        # Optionally highlight start and end points
        if merged_style.get("show_endpoints", False):
            self.ax.scatter(
                [x_coords[0]],
                [y_coords[0]],
                color=merged_style.get("start_color", "green"),
                marker=merged_style.get("start_marker", "o"),
                s=merged_style.get("endpoint_size", 100),
                zorder=merged_style["zorder"] + 1,
                label="Start" if merged_style.get("show_label", False) else None,
            )

            self.ax.scatter(
                [x_coords[-1]],
                [y_coords[-1]],
                color=merged_style.get("end_color", "red"),
                marker=merged_style.get("end_marker", "o"),
                s=merged_style.get("endpoint_size", 100),
                zorder=merged_style["zorder"] + 1,
                label="End" if merged_style.get("show_label", False) else None,
            )

        self.paths_drawn += 1

    def draw_vector(
        self,
        vec: Union[Vector, VectorLike],
        origin: VectorLike = None,
        style: Dict[str, Any] = None,
    ) -> None:
        """Draw a vector as a point.

        Args:
            vec: Vector object or vector-like points
            origin: Origin point for the vector (ignored by default) - kept for API compatibility
            style: Dictionary of style options to override defaults
                   (color, marker, markersize, alpha, zorder)
        """
        style = style or {}

        # Merge default style with provided style
        merged_style = self.default_styles["vector"].copy()
        merged_style.update(style)

        # Handle different input types
        vec = to_vector(vec)

        # Draw vector as a point
        self.ax.scatter(
            vec.x,
            vec.y,
            color=merged_style["color"],
            marker=merged_style.get("marker", "o"),
            s=merged_style.get("markersize", 80),
            alpha=merged_style["alpha"],
            zorder=merged_style["zorder"],
            label=(
                f"Vector {self.vectors_drawn}"
                if merged_style.get("show_label", False)
                else None
            ),
        )

        # Optionally draw a label
        if merged_style.get("show_label", False) or merged_style.get(
            "show_text", False
        ):
            self.ax.text(
                vec.x + merged_style.get("text_offset_x", 0.1),
                vec.y + merged_style.get("text_offset_y", 0.1),
                merged_style.get("text", f"({vec.x:.2f}, {vec.y:.2f})"),
                color=merged_style.get("text_color", merged_style["color"]),
                fontsize=merged_style.get("text_size", 10),
            )

        self.vectors_drawn += 1

    def draw_point(self, point: VectorLike, style: Dict[str, Any] = None) -> None:
        """Draw a point with optional styling.

        Args:
            point: The point to draw
            style: Dictionary of style options to override defaults
                   (color, marker, markersize, alpha, zorder)
        """
        style = style or {}

        # Merge default style with provided style
        merged_style = self.default_styles["point"].copy()
        merged_style.update(style)

        # Handle different input types
        point = to_vector(point)

        # Draw point
        self.ax.scatter(
            point.x,
            point.y,
            color=merged_style["color"],
            marker=merged_style["marker"],
            s=merged_style["markersize"],
            alpha=merged_style["alpha"],
            zorder=merged_style["zorder"],
            label=(
                f"Point {self.points_drawn}"
                if merged_style.get("show_label", False)
                else None
            ),
        )

        self.points_drawn += 1

    def add_legend(self, loc="best") -> None:
        """Add a legend to the plot."""
        handles, labels = self.ax.get_legend_handles_labels()
        if handles:
            self.ax.legend(loc=loc)

    def show(self, block=True) -> None:
        """Show the plot.

        Args:
            block: Whether to block execution when showing the plot
        """
        plt.tight_layout()
        plt.show(block=block)

    def save(self, filename: str, dpi: int = 300) -> None:
        """Save the plot to a file.

        Args:
            filename: The filename to save to
            dpi: Resolution in dots per inch
        """
        plt.tight_layout()
        plt.savefig(filename, dpi=dpi)


def draw(
    *items,
    figsize=(10, 8),
    title="Visualization",
    show_axes=True,
    show=True,
    save_path=None,
):
    """Create a visualization with multiple items.

    Args:
        *items: Individual objects (costmap, vector, path) or tuples of (item, style_dict)
        figsize: Figure size (width, height)
        title: Plot title
        show_axes: Whether to show axes
        show: Whether to show the plot
        save_path: File path to save the figure (optional)

    Example with styling:
        draw(
            (costmap, {"cmap": "viridis"}),
            (path, {"color": "red", "linewidth": 3}),
            (vector, {"color": "blue", "head_width": 0.2}),
            show=True
        )

    Example without styling (using defaults):
        draw(costmap, path, vector, title="Simple Visualization")
    """
    drawer = Drawer(figsize=figsize, title=title, show_axes=show_axes)

    for item in items:
        # Check if this is an (item, style) pair or just an item
        if isinstance(item, tuple) and len(item) == 2 and isinstance(item[1], dict):
            obj, style = item
        else:
            obj, style = item, None

        # Handle vector with origin special case
        if (
            isinstance(obj, tuple)
            and len(obj) == 2
            and (
                isinstance(obj[0], Vector)
                or (hasattr(obj[0], "__iter__") and len(obj[0]) >= 2)
            )
            and (
                isinstance(obj[1], Vector)
                or (hasattr(obj[1], "__iter__") and len(obj[1]) >= 2)
            )
            and style is None
        ):
            # This might be a (vector, origin) pair rather than a (obj, style) pair
            try:
                # Try to interpret as vector and origin
                vec = to_vector(obj[0])
                origin = to_vector(obj[1])
                drawer.draw_vector(vec, origin=origin)
                continue
            except:
                # If that fails, proceed with normal object type detection
                pass

        # Draw different types of objects
        if isinstance(obj, Costmap):
            drawer.draw_costmap(obj, style)
        elif isinstance(obj, Path):
            drawer.draw_path(obj, style)
        elif isinstance(obj, Vector):
            drawer.draw_vector(obj, style=style)
        elif (
            isinstance(obj, tuple)
            and len(obj) == 2
            and all(isinstance(o, (tuple, list, Vector)) for o in obj)
        ):
            # Assume it's a (vector, origin) pair
            drawer.draw_vector(obj[0], origin=obj[1], style=style)
        elif isinstance(obj, (list, tuple)) and all(
            isinstance(p, (tuple, list, Vector)) for p in obj
        ):
            # Assume it's a path represented as a list of points
            drawer.draw_path(obj, style)
        else:
            # Try to treat as a point
            try:
                drawer.draw_point(obj, style)
            except Exception as e:
                print(f"Could not draw object of type {type(obj)}: {e}")

    drawer.add_legend()

    if save_path:
        drawer.save(save_path)

    if show:
        drawer.show()

    return drawer.fig, drawer.ax


if __name__ == "__main__":
    from costmap import Costmap
    from astar import astar

    # Load the costmap
    costmap = Costmap.from_pickle("costmapMsg.pickle")

    # Create a smudged version of the costmap for better planning
    smudged_costmap = costmap.smudge(
        kernel_size=10, iterations=10, threshold=80, preserve_unknown=False
    )

    # Test different types of inputs for goal position
    start = Vector(0.0, 0.0)  # Define a single position
    goal = Vector(5.0, -7.0)  # Define a single position

    # Try each type of input
    path = astar(
        smudged_costmap,
        start=start,
        goal=goal,
        cost_threshold=50,
    )

    print(
        "draw:\n",
        "\n".join(map(lambda x: str(x), [start, goal, path, smudged_costmap])),
    )

    draw(
        (smudged_costmap, {"cmap": "turbo_r"}),
        (costmap, {"transparent_empty": True, "cmap": "Greys"}),
        path,
        (start, {"color": "green"}),
        goal,
    )
