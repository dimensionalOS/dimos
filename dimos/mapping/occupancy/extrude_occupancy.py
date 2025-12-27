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


import numpy as np
from numpy.typing import NDArray

from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid

# Rectangle type: (x, y, width, height)
Rect = tuple[int, int, int, int]


def identify_convex_shapes(occupancy_grid: OccupancyGrid) -> list[Rect]:
    """Identify occupied zones and decompose them into convex rectangles.

    This function finds all occupied cells in the occupancy grid and
    decomposes them into axis-aligned rectangles suitable for MuJoCo
    collision geometry.

    Args:
        occupancy_grid: The input occupancy grid.
        output_path: Path to save the visualization image.

    Returns:
        List of rectangles as (x, y, width, height) tuples in grid coords.
    """
    grid = occupancy_grid.grid

    # Create binary mask of occupied cells (treat UNKNOWN as OCCUPIED)
    occupied_mask = ((grid == CostValues.OCCUPIED) | (grid == CostValues.UNKNOWN)).astype(
        np.uint8
    ) * 255

    return _decompose_to_rectangles(occupied_mask)


def _decompose_to_rectangles(mask: NDArray[np.uint8]) -> list[Rect]:
    """Decompose a binary mask into rectangles using greedy maximal rectangles.

    Iteratively finds and removes the largest rectangle until the mask is empty.

    Args:
        mask: Binary mask of the shape (255 for occupied, 0 for free).

    Returns:
        List of rectangles as (x, y, width, height) tuples.
    """
    rectangles: list[Rect] = []
    remaining = mask.copy()

    max_iterations = 10000  # Safety limit

    for _ in range(max_iterations):
        # Find the largest rectangle in the remaining mask
        rect = _find_largest_rectangle(remaining)

        if rect is None:
            break

        x_start, y_start, x_end, y_end = rect

        # Add rectangle to shapes
        # Store as (x, y, width, height)
        # x_end and y_end are exclusive (like Python slicing)
        rectangles.append((x_start, y_start, x_end - x_start, y_end - y_start))

        # Remove this rectangle from the mask
        remaining[y_start:y_end, x_start:x_end] = 0

    return rectangles


def _find_largest_rectangle(mask: NDArray[np.uint8]) -> tuple[int, int, int, int] | None:
    """Find the largest rectangle of 1s in a binary mask.

    Uses the histogram method for O(rows * cols) complexity.

    Args:
        mask: Binary mask (non-zero = occupied).

    Returns:
        (x_start, y_start, x_end, y_end) or None if no rectangle found.
        Coordinates are exclusive on the end (like Python slicing).
    """
    if not np.any(mask):
        return None

    rows, cols = mask.shape
    binary = (mask > 0).astype(np.int32)

    # Build histogram of heights for each row
    heights = np.zeros((rows, cols), dtype=np.int32)
    heights[0] = binary[0]
    for i in range(1, rows):
        heights[i] = np.where(binary[i] > 0, heights[i - 1] + 1, 0)

    best_area = 0
    best_rect: tuple[int, int, int, int] | None = None

    # For each row, find largest rectangle in histogram
    for row_idx in range(rows):
        hist = heights[row_idx]
        rect = _largest_rect_in_histogram(hist, row_idx)
        if rect is not None:
            x_start, y_start, x_end, y_end = rect
            area = (x_end - x_start) * (y_end - y_start)
            if area > best_area:
                best_area = area
                best_rect = rect

    return best_rect


def _largest_rect_in_histogram(
    hist: NDArray[np.int32], bottom_row: int
) -> tuple[int, int, int, int] | None:
    """Find largest rectangle in a histogram.

    Args:
        hist: Array of heights.
        bottom_row: The row index this histogram ends at.

    Returns:
        (x_start, y_start, x_end, y_end) or None.
    """
    n = len(hist)
    if n == 0:
        return None

    # Stack-based algorithm for largest rectangle in histogram
    stack: list[int] = []  # Stack of indices
    best_area = 0
    best_rect: tuple[int, int, int, int] | None = None

    for i in range(n + 1):
        h = hist[i] if i < n else 0

        while stack and hist[stack[-1]] > h:
            height = hist[stack.pop()]
            width_start = stack[-1] + 1 if stack else 0
            width_end = i
            area = height * (width_end - width_start)

            if area > best_area:
                best_area = area
                # Convert to rectangle coordinates
                y_start = bottom_row - height + 1
                y_end = bottom_row + 1
                best_rect = (width_start, y_start, width_end, y_end)

        stack.append(i)

    return best_rect


def generate_mujoco_scene(
    occupancy_grid: OccupancyGrid,
) -> str:
    """Generate a MuJoCo scene XML from an occupancy grid.

    Creates a scene with a flat floor and extruded boxes for each occupied
    region. All boxes are red and used for collision.

    Args:
        occupancy_grid: The input occupancy grid.

    Returns:
        Path to the generated XML file.
    """
    extrude_height = 0.5

    # Get rectangles from the occupancy grid
    rectangles = identify_convex_shapes(occupancy_grid)

    resolution = occupancy_grid.resolution
    origin_x = occupancy_grid.origin.position.x
    origin_y = occupancy_grid.origin.position.y

    # Build XML
    xml_lines = [
        '<?xml version="1.0" ?>',
        '<mujoco model="scene_occupancy">',
        '  <compiler angle="radian"/>',
        '  <visual><map znear="0.01" zfar="10000"/></visual>',
        "  <asset>",
        '    <material name="mat_red" rgba="0.8 0.2 0.2 1.0"/>',
        '    <material name="mat_floor" rgba="0.9 0.9 0.9 1.0"/>',
        '    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="1 1 1" rgb2="1 1 1" markrgb="0 0 0" width="300" height="300"/>',
        '    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0"/>',
        "  </asset>",
        "  <worldbody>",
        '    <light directional="true" pos="0 0 3" dir="0 0 -1"/>',
        '    <geom name="floor" size="0 0 0.01" type="plane" material="groundplane"/>',
    ]

    # Add each rectangle as a box geom
    for i, (gx, gy, gw, gh) in enumerate(rectangles):
        # Convert grid coordinates to world coordinates
        # Grid origin is top-left, world origin is at occupancy_grid.origin
        # gx, gy are in grid cells, need to convert to meters
        world_x = origin_x + (gx + gw / 2) * resolution
        world_y = origin_y + (gy + gh / 2) * resolution
        world_z = extrude_height / 2  # Center of the box

        # Box half-sizes
        half_x = (gw * resolution) / 2
        half_y = (gh * resolution) / 2
        half_z = extrude_height / 2

        xml_lines.append(
            f'    <geom name="wall_{i}" type="box" '
            f'size="{half_x:.4f} {half_y:.4f} {half_z:.4f}" '
            f'pos="{world_x:.4f} {world_y:.4f} {world_z:.4f}" '
            f'material="mat_red" contype="1" conaffinity="1"/>'
        )

    xml_lines.append("  </worldbody>")
    xml_lines.append('  <statistic center="1 -0.8 1.1" extent=".35"/>')
    xml_lines.append("</mujoco>\n")

    xml_content = "\n".join(xml_lines)

    return xml_content
