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

"""
Utility functions for frontier exploration visualization and testing.
"""

import numpy as np
from PIL import Image, ImageDraw

from dimos.msgs.geometry_msgs import Vector3
from dimos.msgs.nav_msgs import CostValues, OccupancyGrid


def costmap_to_pil_image(costmap: OccupancyGrid, scale_factor: int = 2) -> Image.Image:
    """
    Convert costmap to PIL Image with ROS-style coloring and optional scaling.

    Args:
        costmap: Costmap to convert
        scale_factor: Factor to scale up the image for better visibility

    Returns:
        PIL Image with ROS-style colors
    """
    # Create image array (height, width, 3 for RGB)
    img_array = np.zeros((costmap.height, costmap.width, 3), dtype=np.uint8)

    # Apply ROS-style coloring based on costmap values
    for i in range(costmap.height):
        for j in range(costmap.width):
            value = costmap.grid[i, j]
            if value == CostValues.FREE:  # Free space = light grey
                img_array[i, j] = [205, 205, 205]
            elif value == CostValues.UNKNOWN:  # Unknown = dark gray
                img_array[i, j] = [128, 128, 128]
            elif value >= CostValues.OCCUPIED:  # Occupied/obstacles = black
                img_array[i, j] = [0, 0, 0]
            else:  # Any other values (low cost) = light grey
                img_array[i, j] = [205, 205, 205]

    # Flip vertically to match ROS convention (origin at bottom-left)
    img_array = np.flipud(img_array)

    # Create PIL image
    img = Image.fromarray(img_array, "RGB")

    # Scale up if requested
    if scale_factor > 1:
        new_size = (img.width * scale_factor, img.height * scale_factor)
        img = img.resize(new_size, Image.NEAREST)  # type: ignore[attr-defined]  # Use NEAREST to keep sharp pixels

    return img


def draw_frontiers_on_image(
    image: Image.Image,
    costmap: OccupancyGrid,
    frontiers: list[Vector3],
    scale_factor: int = 2,
    unfiltered_frontiers: list[Vector3] | None = None,
) -> Image.Image:
    """
    Draw frontier points on the costmap image.

    Args:
        image: PIL Image to draw on
        costmap: Original costmap for coordinate conversion
        frontiers: List of frontier centroids (top 5)
        scale_factor: Scaling factor used for the image
        unfiltered_frontiers: All unfiltered frontier results (light green)

    Returns:
        PIL Image with frontiers drawn
    """
    img_copy = image.copy()
    draw = ImageDraw.Draw(img_copy)

    def world_to_image_coords(world_pos: Vector3) -> tuple[int, int]:
        """Convert world coordinates to image pixel coordinates."""
        grid_pos = costmap.world_to_grid(world_pos)
        # Flip Y coordinate and apply scaling
        img_x = int(grid_pos.x * scale_factor)
        img_y = int((costmap.height - grid_pos.y) * scale_factor)  # Flip Y
        return img_x, img_y

    # Draw all unfiltered frontiers as light green circles
    if unfiltered_frontiers:
        for frontier in unfiltered_frontiers:
            x, y = world_to_image_coords(frontier)
            radius = 3 * scale_factor
            draw.ellipse(
                [x - radius, y - radius, x + radius, y + radius],
                fill=(144, 238, 144),
                outline=(144, 238, 144),
            )  # Light green

    # Draw top 5 frontiers as green circles
    for i, frontier in enumerate(frontiers[1:]):  # Skip the best one for now
        x, y = world_to_image_coords(frontier)
        radius = 4 * scale_factor
        draw.ellipse(
            [x - radius, y - radius, x + radius, y + radius],
            fill=(0, 255, 0),
            outline=(0, 128, 0),
            width=2,
        )  # Green

        # Add number label
        draw.text((x + radius + 2, y - radius), str(i + 2), fill=(0, 255, 0))

    # Draw best frontier as red circle
    if frontiers:
        best_frontier = frontiers[0]
        x, y = world_to_image_coords(best_frontier)
        radius = 6 * scale_factor
        draw.ellipse(
            [x - radius, y - radius, x + radius, y + radius],
            fill=(255, 0, 0),
            outline=(128, 0, 0),
            width=3,
        )  # Red

        # Add "BEST" label
        draw.text((x + radius + 2, y - radius), "BEST", fill=(255, 0, 0))

    return img_copy
