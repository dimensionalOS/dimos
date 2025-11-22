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
from PIL import Image

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
        img = img.resize(new_size, Image.NEAREST)  # Use NEAREST to keep sharp pixels

    return img
