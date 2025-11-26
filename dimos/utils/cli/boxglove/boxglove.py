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

from typing import TYPE_CHECKING

import numpy as np
import reactivex.operators as ops
from rich.text import Text
from textual.app import App, ComposeResult
from textual.containers import Container
from textual.reactive import reactive
from textual.widgets import Footer, Static

from dimos import core
from dimos.msgs.nav_msgs import OccupancyGrid
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage

if TYPE_CHECKING:
    from reactivex.disposable import Disposable

    from dimos.msgs.nav_msgs import OccupancyGrid
    from dimos.utils.cli.boxglove.connection import Connection


blocks = "█▗▖▝▘"
shades = "█░░░░"
crosses = "┼┌┐└┘"
quadrant = "█▟▙▜▛"
triangles = "◼◢◣◥◤"  # 45-degree triangular blocks


alphabet = crosses

# Box drawing characters for smooth edges
top_left = alphabet[1]  # Quadrant lower right
top_right = alphabet[2]  # Quadrant lower left
bottom_left = alphabet[3]  # Quadrant upper right
bottom_right = alphabet[4]  # Quadrant upper left
full = alphabet[0]  # Full block


class OccupancyGridApp(App):  # type: ignore[type-arg]
    """A Textual app for visualizing OccupancyGrid data in real-time."""

    CSS = """
    Screen {
        layout: vertical;
        overflow: hidden;
    }

    #grid-container {
        width: 100%;
        height: 1fr;
        overflow: hidden;
        margin: 0;
        padding: 0;
    }

    #grid-display {
        width: 100%;
        height: 100%;
        margin: 0;
        padding: 0;
    }

    Footer {
        dock: bottom;
        height: 1;
    }
    """

    # Reactive properties
    grid_data: reactive[OccupancyGrid | None] = reactive(None)

    BINDINGS = [
        ("q", "quit", "Quit"),
        ("ctrl+c", "quit", "Quit"),
    ]

    def __init__(self, connection: Connection, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(*args, **kwargs)
        self.connection = connection
        self.subscription: Disposable | None = None
        self.grid_display: Static | None = None
        self.cached_grid: OccupancyGrid | None = None

    def compose(self) -> ComposeResult:
        """Create the app layout."""
        # Container for the grid (no scrolling since we scale to fit)
        with Container(id="grid-container"):
            self.grid_display = Static("", id="grid-display")
            yield self.grid_display

        yield Footer()

    def on_mount(self) -> None:
        """Subscribe to the connection when the app starts."""
        self.theme = "flexoki"

        # Subscribe to the OccupancyGrid stream
        def on_grid(grid: OccupancyGrid) -> None:
            self.grid_data = grid

        def on_error(error: Exception) -> None:
            self.notify(f"Error: {error}", severity="error")

        self.subscription = self.connection().subscribe(on_next=on_grid, on_error=on_error)  # type: ignore[assignment]

    async def on_unmount(self) -> None:
        """Clean up subscription when app closes."""
        if self.subscription:
            self.subscription.dispose()

    def watch_grid_data(self, grid: OccupancyGrid | None) -> None:
        """Update display when new grid data arrives."""
        if grid is None:
            return

        # Cache the grid for rerendering on terminal resize
        self.cached_grid = grid

        # Render the grid as ASCII art
        grid_text = self.render_grid(grid)
        self.grid_display.update(grid_text)  # type: ignore[union-attr]

    def on_resize(self, event) -> None:  # type: ignore[no-untyped-def]
        """Handle terminal resize events."""
        if self.cached_grid:
            # Re-render with new terminal dimensions
            grid_text = self.render_grid(self.cached_grid)
            self.grid_display.update(grid_text)  # type: ignore[union-attr]

    def render_grid(self, grid: OccupancyGrid) -> Text:
        """Render the OccupancyGrid as colored ASCII art, scaled to fit terminal."""
        text = Text()

        # Get the actual container dimensions
        container = self.query_one("#grid-container")
        content_width = container.content_size.width
        content_height = container.content_size.height

        # Each cell will be 2 chars wide to make square pixels
        terminal_width = max(1, content_width // 2)
        terminal_height = max(1, content_height)

        # Handle edge cases
        if grid.width == 0 or grid.height == 0:
            return text  # Return empty text for empty grid

        # Calculate scaling factors (as floats for smoother scaling)
        scale_x = grid.width / terminal_width
        scale_y = grid.height / terminal_height

        # Use the larger scale to ensure the grid fits
        scale_float = max(1.0, max(scale_x, scale_y))

        # For smoother resizing, we'll use fractional scaling
        # This means we might sample between grid cells
        render_width = min(int(grid.width / scale_float), terminal_width)
        render_height = min(int(grid.height / scale_float), terminal_height)

        # Store both integer and float scale for different uses
        int(np.ceil(scale_float))  # For legacy compatibility

        # Adjust render dimensions to use all available space
        # This reduces jumping by allowing fractional cell sizes
        actual_scale_x = grid.width / render_width if render_width > 0 else 1
        actual_scale_y = grid.height / render_height if render_height > 0 else 1

        # Function to get value with fractional scaling
        def get_cell_value(grid_data: np.ndarray, x: int, y: int) -> int:  # type: ignore[type-arg]
            # Use fractional coordinates for smoother scaling
            y_center = int((y + 0.5) * actual_scale_y)
            x_center = int((x + 0.5) * actual_scale_x)

            # Clamp to grid bounds
            y_center = max(0, min(y_center, grid.height - 1))
            x_center = max(0, min(x_center, grid.width - 1))

            # For now, just sample the center point
            # Could do area averaging for smoother results
            return grid_data[y_center, x_center]  # type: ignore[no-any-return]

        # Helper function to check if a cell is an obstacle
        def is_obstacle(grid_data: np.ndarray, x: int, y: int) -> bool:  # type: ignore[type-arg]
            if x < 0 or x >= render_width or y < 0 or y >= render_height:
                return False
            value = get_cell_value(grid_data, x, y)
            return value > 90  # Consider cells with >90% probability as obstacles

        # Character and color mapping with intelligent obstacle rendering
        def get_cell_char_and_style(grid_data: np.ndarray, x: int, y: int) -> tuple[str, str]:  # type: ignore[type-arg]
            value = get_cell_value(grid_data, x, y)
            norm_value = min(value, 100) / 100.0

            if norm_value > 0.9:
                # Check neighbors for intelligent character selection
                top = is_obstacle(grid_data, x, y + 1)
                bottom = is_obstacle(grid_data, x, y - 1)
                left = is_obstacle(grid_data, x - 1, y)
                right = is_obstacle(grid_data, x + 1, y)

                # Count neighbors
                neighbor_count = sum([top, bottom, left, right])

                # Select character based on neighbor configuration
                if neighbor_count == 4:
                    # All neighbors are obstacles - use full block
                    symbol = full + full
                elif neighbor_count == 3:
                    # Three neighbors - use full block (interior edge)
                    symbol = full + full
                elif neighbor_count == 2:
                    # Two neighbors - check configuration
                    if top and bottom:
                        symbol = full + full  # Vertical corridor
                    elif left and right:
                        symbol = full + full  # Horizontal corridor
                    elif top and left:
                        symbol = bottom_right + " "
                    elif top and right:
                        symbol = " " + bottom_left
                    elif bottom and left:
                        symbol = top_right + " "
                    elif bottom and right:
                        symbol = " " + top_left
                    else:
                        symbol = full + full
                elif neighbor_count == 1:
                    # One neighbor - point towards it
                    if top:
                        symbol = bottom_left + bottom_right
                    elif bottom:
                        symbol = top_left + top_right
                    elif left:
                        symbol = top_right + bottom_right
                    elif right:
                        symbol = top_left + bottom_left
                    else:
                        symbol = full + full
                else:
                    # No neighbors - isolated obstacle
                    symbol = full + full

                return symbol, None  # type: ignore[return-value]
            else:
                return "  ", None  # type: ignore[return-value]

        # Render the scaled grid row by row (flip Y axis for proper display)
        for y in range(render_height - 1, -1, -1):
            for x in range(render_width):
                char, style = get_cell_char_and_style(grid.grid, x, y)
                text.append(char, style=style)
            if y > 0:  # Add newline except for last row
                text.append("\n")

        # Could show scale info in footer status if needed

        return text


def main() -> None:
    """Run the OccupancyGrid visualizer with a connection."""
    # app = OccupancyGridApp(core.LCMTransport("/global_costmap", OccupancyGrid).observable)

    app = OccupancyGridApp(
        lambda: core.LCMTransport("/lidar", LidarMessage)  # type: ignore[no-untyped-call]
        .observable()
        .pipe(ops.map(lambda msg: msg.costmap()))  # type: ignore[attr-defined]
    )
    app.run()
    import time

    while True:
        time.sleep(1)


if __name__ == "__main__":
    main()
