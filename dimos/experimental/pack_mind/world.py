# Copyright 2026 Dimensional Inc.
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

"""PACK MIND sim — world fabrication (SAR floor as an OccupancyGrid).

A collapsed-building floor: free space carved into rooms by walls with narrow
door gaps. Corridors/doors are kept wide enough to survive the router's safe-mask
erosion. Fully known map (no fog) — FrontierPatrolRouter sweeps unvisited *free*
cells, it does not need UNKNOWN.
"""

from __future__ import annotations

import numpy as np

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid

RES = 0.1  # meters per cell
DOOR = 8  # door gap width in cells (>> erosion structure so it stays traversable)


def pose_at(x: float, y: float) -> PoseStamped:
    """A PoseStamped at world (x, y)."""
    p = PoseStamped()
    p.position.x = x
    p.position.y = y
    return p


def make_maze_world(w: int = 60, h: int = 60, res: float = RES) -> OccupancyGrid:
    """Free interior, OCCUPIED border + 3 vertical dividers with alternating doors.

    The alternating door placement forces a serpentine route between the four
    columns — exactly where independent robots redundantly re-walk corridors and a
    shared coverage memory pays off.
    """
    grid = np.full((h, w), CostValues.FREE, dtype=np.int8)
    grid[0, :] = grid[-1, :] = CostValues.OCCUPIED
    grid[:, 0] = grid[:, -1] = CostValues.OCCUPIED

    # Vertical dividers at 1/4, 2/4, 3/4 width. Doors alternate top / bottom.
    for i, x in enumerate((w // 4, w // 2, 3 * w // 4)):
        grid[1:-1, x] = CostValues.OCCUPIED
        if i % 2 == 0:
            grid[1 : 1 + DOOR, x] = CostValues.FREE  # door at top
        else:
            grid[-1 - DOOR : -1, x] = CostValues.FREE  # door at bottom
    return OccupancyGrid(grid=grid, resolution=res)


def free_cells_world(world: OccupancyGrid) -> list[tuple[float, float]]:
    """World-coordinate centers of all FREE cells."""
    rows, cols = np.where(world.grid == CostValues.FREE)
    out: list[tuple[float, float]] = []
    for r, c in zip(rows.tolist(), cols.tolist()):
        wp = world.grid_to_world((int(c), int(r), 0))
        out.append((wp.x, wp.y))
    return out


def plant_survivors(
    world: OccupancyGrid, n: int, seed: int, margin_m: float = 0.5
) -> list[tuple[float, float]]:
    """Plant n survivors on free cells, spread across the map (min pairwise dist)."""
    rng = np.random.default_rng(seed)
    cells = free_cells_world(world)
    rng.shuffle(cells)
    chosen: list[tuple[float, float]] = []
    for c in cells:
        if all((c[0] - s[0]) ** 2 + (c[1] - s[1]) ** 2 >= margin_m**2 for s in chosen):
            chosen.append(c)
        if len(chosen) == n:
            break
    return chosen


def free_cell_count(world: OccupancyGrid) -> int:
    return int(np.count_nonzero(world.grid == CostValues.FREE))
