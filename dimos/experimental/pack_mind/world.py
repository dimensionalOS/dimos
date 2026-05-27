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
from PIL import Image
from scipy.ndimage import binary_dilation, binary_erosion, label

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.utils.data import get_data

RES = 0.1  # meters per cell
DOOR = 8  # door gap width in cells (>> erosion structure so it stays traversable)

# Pixel-value semantics of the DimOS SLAM occupancy PNGs (e.g.
# big_office_simple_occupancy.png). `OccupancyGrid.from_path` does NOT remap
# these to CostValues, so we interpret them here.
_PNG_FREE_VAL = 15  # mapped interior floor
_PNG_WALL_VAL = 105  # walls / obstacles (the 0 background is the building exterior)


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


def load_building_world(
    name: str = "big_office_simple_occupancy.png",
    downsample: int = 4,
    res_full: float = 0.05,
    close_iters: int = 1,
) -> OccupancyGrid:
    """Turn a real DimOS SLAM occupancy PNG into a fog-of-war search arena.

    The raw PNG is a *visualization* (pixel values 15=free, 105=wall, 0=exterior),
    not a CostValues grid, so we remap by value. We downsample to keep the sim
    fast and make the fixed sensor radius proportionate to the floor size.

    Structure comes from the *building footprint*, not interior pixels: the free
    interior is majority-pooled per block, morphologically closed to bridge SLAM
    speckle (which would otherwise sever real corridors), then reduced to its
    largest connected component. Everything outside the footprint is OCCUPIED, so
    the building's own concave outline — wings, corridors, dead-ends — is what
    blocks line-of-sight. That concavity is what makes shared-vs-independent fog
    diverge; the sparse interior wall dots are SLAM noise and are deliberately
    not carved back in (dilating them disconnects the floor).
    """
    raw = np.array(Image.open(get_data(name)).convert("L"))
    ds = max(1, downsample)
    h, w = raw.shape
    hc, wc = h // ds, w // ds
    raw = raw[: hc * ds, : wc * ds]

    free = (raw == _PNG_FREE_VAL).reshape(hc, ds, wc, ds).mean(axis=(1, 3)) > 0.5
    if close_iters > 0:  # bridge speckle gaps so corridors stay connected
        free = binary_dilation(free, iterations=close_iters)
        free = binary_erosion(free, iterations=close_iters, border_value=1)

    lab, n = label(free)  # keep the largest navigable floor, drop outliers
    if n > 1:
        sizes = np.bincount(lab.ravel())
        sizes[0] = 0
        free = lab == int(sizes.argmax())

    grid = np.full((hc, wc), CostValues.OCCUPIED, dtype=np.int8)
    grid[free] = CostValues.FREE
    return OccupancyGrid(grid=grid, resolution=res_full * ds, frame_id="world")


def spread_starts(
    world: OccupancyGrid, n: int, seed: int = 0, min_dist_m: float = 1.0
) -> list[tuple[float, float]]:
    """Pick n free-cell start positions, greedily maximizing pairwise spread.

    Replaces the hand-tuned _START_POOL when the arena geometry is data-driven."""
    rng = np.random.default_rng(seed)
    cells = free_cells_world(world)
    if not cells:
        raise ValueError("world has no FREE cells")
    rng.shuffle(cells)
    chosen: list[tuple[float, float]] = [cells[0]]
    for c in cells[1:]:
        if all((c[0] - s[0]) ** 2 + (c[1] - s[1]) ** 2 >= min_dist_m**2 for s in chosen):
            chosen.append(c)
        if len(chosen) == n:
            break
    # If the arena was too cramped to find n spread points, pad with any free cells.
    for c in cells:
        if len(chosen) >= n:
            break
        if c not in chosen:
            chosen.append(c)
    return chosen[:n]


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
