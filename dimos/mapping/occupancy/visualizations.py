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

from functools import lru_cache
from typing import Literal, TypeAlias

import cv2
import numpy as np
from numpy.typing import NDArray

from dimos.msgs.nav_msgs import Path
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.sensor_msgs.image_impls.AbstractImage import ImageFormat

Palette: TypeAlias = Literal["rainbow", "turbo"]


def visualize_occupancy_grid(
    occupancy_grid: OccupancyGrid, palette: Palette, path: Path | None = None
) -> Image:
    match palette:
        case "rainbow":
            bgr_image = rainbow_image(occupancy_grid.grid)
        case "turbo":
            bgr_image = turbo_image(occupancy_grid.grid)
        case _:
            raise NotImplementedError()

    if path is not None and len(path.poses) > 0:
        _draw_path(occupancy_grid, bgr_image, path)

    return Image(
        data=bgr_image,
        format=ImageFormat.BGR,
        frame_id=occupancy_grid.frame_id,
        ts=occupancy_grid.ts,
    )


def _draw_path(occupancy_grid: OccupancyGrid, bgr_image: NDArray[np.uint8], path: Path) -> None:
    points = []
    for pose in path.poses:
        grid_coord = occupancy_grid.world_to_grid([pose.x, pose.y, pose.z])
        pixel_x = int(grid_coord.x)
        pixel_y = int(grid_coord.y)

        if 0 <= pixel_x < occupancy_grid.width and 0 <= pixel_y < occupancy_grid.height:
            points.append((pixel_x, pixel_y))

    if len(points) > 1:
        points_array = np.array(points, dtype=np.int32)
        cv2.polylines(bgr_image, [points_array], isClosed=False, color=(0, 0, 0), thickness=1)


def rainbow_image(grid: NDArray[np.int8]) -> NDArray[np.uint8]:
    """Convert the occupancy grid to a rainbow-colored Image.

    Color scheme:
    - -1 (unknown): black
    - 100 (occupied): magenta
    - 0-99: rainbow from blue (0) to red (99)

    Returns:
        Image with rainbow visualization of the occupancy grid
    """

    # Create a copy of the grid for visualization
    # Map values to 0-255 range for colormap
    height, width = grid.shape
    vis_grid = np.zeros((height, width), dtype=np.uint8)

    # Handle 0-99: map to colormap range
    gradient_mask = (grid >= 0) & (grid < 100)
    vis_grid[gradient_mask] = ((grid[gradient_mask] / 99.0) * 255).astype(np.uint8)

    # Apply JET colormap (blue to red) - returns BGR
    bgr_image = cv2.applyColorMap(vis_grid, cv2.COLORMAP_JET)

    unknown_mask = grid == -1
    bgr_image[unknown_mask] = [0, 0, 0]

    occupied_mask = grid == 100
    bgr_image[occupied_mask] = [255, 0, 255]

    return bgr_image.astype(np.uint8)


def turbo_image(grid: NDArray[np.int8]) -> NDArray[np.uint8]:
    """Convert the occupancy grid to a turbo-colored Image.

    Returns:
        Image with turbo visualization of the occupancy grid
    """
    color_lut = _turbo_lut()

    # Map grid values to lookup indices
    # Values: -1 -> 255, 0-100 -> 0-100, clipped to valid range
    lookup_indices = np.where(grid == -1, 255, np.clip(grid, 0, 100)).astype(np.uint8)

    # Create BGR image using lookup table (vectorized operation)
    return color_lut[lookup_indices]


def _interpolate_turbo(t: float) -> tuple[int, int, int]:
    """D3's interpolateTurbo colormap implementation.

    Based on Anton Mikhailov's Turbo colormap using polynomial approximations.

    Args:
        t: Value in [0, 1]

    Returns:
        RGB tuple (0-255 range)
    """
    t = max(0.0, min(1.0, t))

    r = 34.61 + t * (1172.33 - t * (10793.56 - t * (33300.12 - t * (38394.49 - t * 14825.05))))
    g = 23.31 + t * (557.33 + t * (1225.33 - t * (3574.96 - t * (1073.77 + t * 707.56))))
    b = 27.2 + t * (3211.1 - t * (15327.97 - t * (27814.0 - t * (22569.18 - t * 6838.66))))

    return (
        max(0, min(255, round(r))),
        max(0, min(255, round(g))),
        max(0, min(255, round(b))),
    )


@lru_cache(maxsize=1)
def _turbo_lut() -> NDArray[np.uint8]:
    # Pre-compute lookup table for all possible values (-1 to 100)
    color_lut = np.zeros((256, 3), dtype=np.uint8)

    for value in range(-1, 101):
        # Normalize to [0, 1] range based on domain [-1, 100]
        t = (value + 1) / 101.0

        if value == -1:
            rgb = (34, 24, 28)
        elif value == 100:
            rgb = (0, 0, 0)
        else:
            rgb = _interpolate_turbo(t * 2 - 1)

        # Map -1 to index 255, 0-100 to indices 0-100
        idx = 255 if value == -1 else value
        color_lut[idx] = [rgb[2], rgb[1], rgb[0]]

    return color_lut
