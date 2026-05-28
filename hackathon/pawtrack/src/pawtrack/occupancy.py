"""Reject subject candidates that land on a mapped wall (glass reflections).

A person reflected in a glass wall is detected by the VLM, and the lidar ranges
the glass *surface* the reflection sits on, so the two agree at the standoff and
the greeter walks over and waves at the reflection. The navigation costmap
already marks that glass wall as an obstacle, so a candidate whose floor
position lands on -- or within a small clearance of -- an occupied cell is
almost certainly a reflection or a badly ranged box; a real person to greet
stands in free space.

This module is pure (no DimOS import) so it can be unit tested with a plain
list-of-lists grid. The container adapts the DimOS ``OccupancyGrid`` into these
primitive arguments.
"""

from __future__ import annotations

import math
from collections.abc import Sequence

# nav_msgs OccupancyGrid cost convention: -1 unknown, 0 free, up to 100 lethal.
# A cell at or above this cost is treated as a wall for rejection. Tunable: raise
# toward 100 to reject only near-lethal cells (keeps subjects close to walls).
DEFAULT_COST_THRESHOLD = 50


def is_near_obstacle(
    grid: Sequence[Sequence[int]] | None,
    resolution: float,
    origin_xy: tuple[float, float],
    point_xy: tuple[float, float],
    *,
    clearance_m: float,
    cost_threshold: int = DEFAULT_COST_THRESHOLD,
) -> bool:
    """Whether world ``point_xy`` is on, or within ``clearance_m`` of, a wall.

    Fails open: returns False (keep the candidate) when there is no map yet, the
    resolution is invalid, the point is off the grid, or only unknown/free cells
    are nearby. The filter rejects only on a *known* obstacle, so a missing or
    partial map never suppresses greeting.

    Args:
        grid: Occupancy costs indexed ``grid[row_y][col_x]`` (the nav_msgs
            convention), or None when no costmap has arrived.
        resolution: Metres per cell.
        origin_xy: World coordinate of grid cell (0, 0).
        point_xy: World (x, y) to test.
        clearance_m: Reject if an occupied cell lies within this radius.
        cost_threshold: Minimum cell cost treated as an obstacle.

    Returns:
        True if a cell with cost >= ``cost_threshold`` lies within
        ``clearance_m`` of the point; False otherwise.
    """
    if grid is None or resolution <= 0.0:
        return False
    height = len(grid)
    if height == 0:
        return False
    width = len(grid[0])
    if width == 0:
        return False
    origin_x, origin_y = origin_xy
    center_x = int((point_xy[0] - origin_x) / resolution)
    center_y = int((point_xy[1] - origin_y) / resolution)
    radius_cells = max(0, math.ceil(clearance_m / resolution))
    for delta_y in range(-radius_cells, radius_cells + 1):
        for delta_x in range(-radius_cells, radius_cells + 1):
            if math.hypot(delta_x, delta_y) * resolution > clearance_m:
                continue
            cell_x = center_x + delta_x
            cell_y = center_y + delta_y
            if 0 <= cell_x < width and 0 <= cell_y < height:
                if grid[cell_y][cell_x] >= cost_threshold:
                    return True
    return False
