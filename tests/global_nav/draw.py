import matplotlib.pyplot as plt
import numpy as np
from typing import List, Dict, Any, Union
from costmap import Costmap
from vector import Vector
from path import Path
from vectortypes import VectorLike, to_vector


class Drawer:
    """A class to draw costmaps, vectors, and paths together with styling options."""

    default_styles = {
        "costmap": {
            "cmap": "turbo_r",
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

    def __init__(
        self,
        figsize=(10, 8),
        title="Visualization",
        show_axes=True,
        show_colorbar=False,
        dark_mode=False,
        fig=None,
        ax=None,
        click_callback=None,
    ):
        """Initialize the drawer with figure settings.

        Args:
            figsize: Tuple of (width, height) for the figure
            title: Title for the plot
            show_axes: Whether to show axes in the plot
            show_colorbar: Whether to show colorbar for costmaps
            dark_mode: Whether to use dark background with light text
            fig: Existing figure to use (optional)
            ax: Existing axes to use (optional)
            click_callback: Custom callback function to be called with (x, y) coordinates when plot is clicked
        """
        if fig is not None and ax is not None:
            self.fig = fig
            self.ax = ax
            # Clear the axes but keep the figure
            self.ax.clear()
        else:
            self.fig, self.ax = plt.subplots(figsize=figsize)

        self.show_colorbar = show_colorbar
        self.dark_mode = dark_mode

        self.cid = self.fig.canvas.mpl_connect("button_press_event", self._on_click)

        self.click_callback = click_callback

        if dark_mode:
            bg_color = "black"
            text_color = "white"
        else:
            bg_color = "white"
            text_color = "black"

        self.fig.set_facecolor(bg_color)
        self.ax.set_facecolor(bg_color)
        self.ax.set_title(title, color=text_color)

        if not show_axes:
            self.ax.set_axis_off()
        else:
            self.ax.set_xlabel("X (world coordinates)", color=text_color)
            self.ax.set_ylabel("Y (world coordinates)", color=text_color)
            self.ax.tick_params(colors=text_color)
            self.ax.spines["bottom"].set_color(text_color)
            self.ax.spines["top"].set_color(text_color)
            self.ax.spines["left"].set_color(text_color)
            self.ax.spines["right"].set_color(text_color)

        self.reset_counters()

    def reset_counters(self):
        """Reset the counters for drawn objects."""
        self.costmaps_drawn = 0
        self.paths_drawn = 0
        self.vectors_drawn = 0
        self.points_drawn = 0

    def clear(self, title=None):
        """Clear the current axes for redrawing.

        Args:
            title: Optional new title for the plot
        """
        self.ax.clear()

        # Restore styling
        text_color = "white" if self.dark_mode else "black"

        if title:
            self.ax.set_title(title, color=text_color)

        # Restore axis styling
        if not hasattr(self.ax, "_axis_off") or not self.ax._axis_off:
            self.ax.set_xlabel("X (world coordinates)", color=text_color)
            self.ax.set_ylabel("Y (world coordinates)", color=text_color)
            self.ax.tick_params(colors=text_color)
            self.ax.spines["bottom"].set_color(text_color)
            self.ax.spines["top"].set_color(text_color)
            self.ax.spines["left"].set_color(text_color)
            self.ax.spines["right"].set_color(text_color)

        # Reset the counters
        self.reset_counters()

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
            text_color = "white" if self.dark_mode else "black"
            self.ax.tick_params(colors=text_color)

        # Add colorbar for known cells if this is the first costmap and show_colorbar is True
        if self.costmaps_drawn == 0 and self.show_colorbar:
            cbar = self.fig.colorbar(im, ax=self.ax)
            text_color = "white" if self.dark_mode else "black"
            cbar.set_label("Cost Value (0-100)", color=text_color)
            cbar.ax.yaxis.set_tick_params(color=text_color)
            cbar.outline.set_edgecolor(text_color)
            plt.setp(plt.getp(cbar.ax.axes, "yticklabels"), color=text_color)

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
            legend = self.ax.legend(loc=loc)
            if self.dark_mode:
                legend.get_frame().set_facecolor("black")
                for text in legend.get_texts():
                    text.set_color("white")
            else:
                legend.get_frame().set_facecolor("white")
                for text in legend.get_texts():
                    text.set_color("black")

    def _on_click(self, event):
        """Handle click events on the plot.

        Args:
            event: The matplotlib event object
        """
        if event.xdata is not None and event.ydata is not None:
            # Get world coordinates
            x, y = event.xdata, event.ydata

            # Print coordinates if no callback is provided
            if self.click_callback is None:
                print(f"Clicked at world coordinates: ({x:.3f}, {y:.3f})")
            else:
                # Call the user-provided callback with coordinates
                self.click_callback(x, y)

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

    def draw(self, *items, add_legend=False):
        """Draw multiple items with optional styling.

        Args:
            *items: Individual objects (costmap, vector, path) or tuples of (item, style_dict)
            add_legend: Whether to add a legend after drawing all items

        Returns:
            The figure and axes objects
        """
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
                    self.draw_vector(vec, origin=origin)
                    continue
                except:
                    # If that fails, proceed with normal object type detection
                    pass

            # Draw different types of objects
            if isinstance(obj, Costmap):
                self.draw_costmap(obj, style)
            elif isinstance(obj, Path):
                self.draw_path(obj, style)
            elif isinstance(obj, Vector):
                self.draw_vector(obj, style=style)
            elif (
                isinstance(obj, tuple)
                and len(obj) == 2
                and all(isinstance(o, (tuple, list, Vector)) for o in obj)
            ):
                # Assume it's a (vector, origin) pair
                self.draw_vector(obj[0], origin=obj[1], style=style)
            elif isinstance(obj, (list, tuple)) and all(
                isinstance(p, (tuple, list, Vector)) for p in obj
            ):
                # Assume it's a path represented as a list of points
                self.draw_path(obj, style)
            else:
                # Try to treat as a point
                try:
                    self.draw_point(obj, style)
                except Exception as e:
                    print(f"Could not draw object of type {type(obj)}: {e}")

        if add_legend:
            self.add_legend()

        return self.fig, self.ax


def draw(
    *items,
    legend=True,
    save_path=False,
    **kwargs,
):
    drawer = Drawer(
        **kwargs,
    )

    drawer.draw(*items)

    if legend:
        drawer.add_legend()

    if save_path:
        drawer.save(save_path)

    drawer.show()


if __name__ == "__main__":
    import time
    import random
    import matplotlib.pyplot as plt
    from costmap import Costmap
    from astar import astar

    # Load the costmap
    costmap = Costmap.from_pickle("costmapMsg.pickle")

    # Create a smudged version of the costmap for better planning
    smudged_costmap = costmap.smudge(
        kernel_size=10, iterations=10, threshold=80, preserve_unknown=False
    )

    # Start position
    start = Vector(0.0, 0.0)
    goal = Vector(4.0, -3.0)
    path = astar(
        smudged_costmap,
        start=start,
        goal=goal,
        cost_threshold=50,
    )

    draw(
        smudged_costmap,
        (costmap, {"transparent_empty": True, "transparent_unknown": True}),
        start,
        (goal, {"color": "green"}),
        path,
    )
