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

"""Two views of a point cloud for an agent, rendered from a pose it chooses.

``depth``: a perspective depth image from a pinhole camera at the pose, the
way the scene would look from there. ``occupancy``: the top-down map the
mapping stack already builds (:func:`dimos.mapping.pointclouds.occupancy.general_occupancy`)
over an absolute z band the caller states, optionally with a pose marked on
it, so a render from inside the space can be placed on the map of the whole
space. Every height is absolute z in the cloud's frame; nothing here
estimates a floor.

A build produces one form, set by ``constants.FORM``: images (PNGs written to
disk) or text (ASCII grids).

Conventions: x east, y north, z up. Yaw 0 looks along +x, positive yaw turns
toward +y (counter-clockwise from above). Pitch positive looks up.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import os
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np
from PIL import Image, ImageDraw

from dimos.constants import STATE_DIR
from dimos.mapping.pointclouds.occupancy import general_occupancy
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@dataclass(frozen=True)
class View:
    """A camera pose in the cloud's frame."""

    x: float
    y: float
    z: float
    yaw_deg: float = 0.0
    """Heading in degrees."""
    pitch_deg: float = 0.0
    """Tilt in degrees."""

    def axes(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Unit vectors forward, right, up in the cloud's frame."""
        yaw = math.radians(self.yaw_deg)
        pitch = math.radians(self.pitch_deg)
        forward = np.array(
            [math.cos(yaw) * math.cos(pitch), math.sin(yaw) * math.cos(pitch), math.sin(pitch)]
        )
        right = np.array([math.sin(yaw), -math.cos(yaw), 0.0])
        up = np.cross(right, forward)
        return forward, right, up


def as_view(view: View | tuple[float, ...] | dict[str, float]) -> View:
    if isinstance(view, View):
        return view
    if isinstance(view, dict):
        return View(**view)
    return View(*view)


# -- depth ----------------------------------------------------------------------


@dataclass(frozen=True)
class DepthRaster:
    depth: np.ndarray
    widths: tuple[float, float] | None
    point_ids: np.ndarray
    provenance: np.ndarray
    """0 missing, 1 projected return, 2 splat, 3 filled pixel."""
    projected_uv: np.ndarray


def depth_image(
    points: np.ndarray,
    view: View,
    *,
    fov_deg: float,
    size: tuple[int, int],
    max_depth: float | None,
    point_size_m: float | None,
) -> tuple[np.ndarray, tuple[float, float] | None]:
    raster = depth_raster(
        points, view, fov_deg=fov_deg, size=size, max_depth=max_depth, point_size_m=point_size_m
    )
    return raster.depth, raster.widths


def depth_raster(
    points: np.ndarray,
    view: View,
    *,
    fov_deg: float,
    size: tuple[int, int],
    max_depth: float | None,
    point_size_m: float | None,
    max_splat_px: int = 12,
) -> DepthRaster:
    """Depth along the view direction per pixel, ``inf`` where nothing was hit,
    shape (height, width), and the (smallest, largest) width in metres the
    returns were drawn at, None when nothing was in view. Each return is
    drawn as a square at its depth, as wide as the gap to its nearest
    neighbour in the cloud (or ``point_size_m`` for every return when given),
    so a dense cloud renders finely and a sparse one still closes into
    surfaces; the nearest return wins wherever squares overlap. A square
    reaches at most ``max_splat_px`` pixels from its return. Remaining
    one-pixel holes are closed from their nearest neighbour."""
    width, height = size
    if any(type(n) is not int or not 1 <= n <= 2048 for n in size) or width * height > 2097152:
        raise ValueError("depth size must be positive integers <=2048 and <=2097152 pixels")
    if (
        not np.isfinite([view.x, view.y, view.z, view.yaw_deg, view.pitch_deg, fov_deg]).all()
        or not 0 < fov_deg < 180
    ):
        raise ValueError("view must be finite and fov_deg must be between 0 and 180")
    if max_depth is not None and (not np.isfinite(max_depth) or max_depth <= 0.05):
        raise ValueError("max_depth must be finite and greater than 0.05")
    if point_size_m is not None and (not np.isfinite(point_size_m) or point_size_m < 0):
        raise ValueError("point_size_m must be finite and nonnegative")
    depth = np.full((height, width), np.inf, dtype=np.float32)
    ids = np.full((height, width), -1, dtype=np.int64)
    provenance = np.zeros((height, width), dtype=np.uint8)
    projected = np.full((len(points), 2), np.nan)
    if len(points) == 0:
        return DepthRaster(depth, None, ids, provenance, projected)
    forward, right, up = view.axes()
    rel = points - np.array([view.x, view.y, view.z], dtype=points.dtype)
    d = rel @ forward
    keep = d > 0.05
    if max_depth is not None:
        keep &= d <= max_depth
    rel, d = rel[keep], d[keep]
    source_ids = np.flatnonzero(keep)
    if len(d) == 0:
        return DepthRaster(depth, None, ids, provenance, projected)
    focal = (width / 2.0) / math.tan(math.radians(fov_deg) / 2.0)
    u = (rel @ right) / d * focal + width / 2.0
    v = -(rel @ up) / d * focal + height / 2.0
    projected[source_ids] = np.column_stack((u, v))
    col = np.floor(u).astype(np.int64)
    row = np.floor(v).astype(np.int64)
    width_m = point_spacing(rel) if point_size_m is None else np.full(len(d), point_size_m)
    radius = np.clip(np.round(focal * width_m / d / 2.0), 0, max_splat_px).astype(np.int64)
    # Positive float32 bit order equals numeric depth order; low bits break ties by return ID.
    keys = (d.astype(np.float32).view(np.uint32).astype(np.uint64) << 32) | source_ids.astype(
        np.uint64
    )
    missing = np.iinfo(np.uint64).max
    packed = np.full(width * height, missing, dtype=np.uint64)
    for r in np.unique(radius):
        group = radius == r
        gc, gr, gkeys = col[group], row[group], keys[group]
        for dy in range(-int(r), int(r) + 1):
            rows = gr + dy
            for dx in range(-int(r), int(r) + 1):
                cols = gc + dx
                inside = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
                np.minimum.at(packed, rows[inside] * width + cols[inside], gkeys[inside])
    nearest = packed.reshape(height, width)
    valid = nearest != missing
    depth[valid] = (nearest[valid] >> 32).astype(np.uint32).view(np.float32)
    ids[valid] = (nearest[valid] & np.uint64(0xFFFFFFFF)).astype(np.int64)
    yy, xx = np.indices(depth.shape)
    provenance[valid] = 2
    exact = (
        valid
        & (np.floor(projected[np.maximum(ids, 0), 0]) == xx)
        & (np.floor(projected[np.maximum(ids, 0), 1]) == yy)
    )
    provenance[exact] = 1
    padded = np.pad(depth, 1, constant_values=np.inf)
    padded_ids = np.pad(ids, 1, constant_values=-1)
    filled_depth = depth.copy()
    filled_ids = ids.copy()
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            neighbour = padded[1 + dy : 1 + dy + height, 1 + dx : 1 + dx + width]
            take = ~valid & (neighbour < filled_depth)
            filled_depth[take] = neighbour[take]
            filled_ids[take] = padded_ids[1 + dy : 1 + dy + height, 1 + dx : 1 + dx + width][take]
    provenance[~valid & np.isfinite(filled_depth)] = 3
    return DepthRaster(
        filled_depth,
        (round(float(width_m.min()), 3), round(float(width_m.max()), 3)),
        filled_ids,
        provenance,
        projected,
    )


def point_spacing(points: np.ndarray, *, min_m: float = 0.01, max_m: float = 0.5) -> np.ndarray:
    """Distance from each return to its nearest neighbour, clamped to
    [``min_m``, ``max_m``]: the width to draw it at so that neighbours just
    touch. A lone return has no neighbour and gets ``max_m``."""
    from scipy.spatial import cKDTree

    if len(points) < 2:
        return np.full(len(points), max_m)
    d, _ = cKDTree(points).query(points, k=2)
    spacing: np.ndarray = np.clip(d[:, 1], min_m, max_m)
    return spacing


def cloud_spacing(points: np.ndarray) -> float:
    """The typical gap between neighbouring returns: the median of
    :func:`point_spacing`. A cloud without returns is spaced like a lone return."""
    lone = np.zeros((1, 3), dtype=np.float32)
    return float(np.median(point_spacing(points if len(points) else lone)))


def _close_holes(depth: np.ndarray) -> np.ndarray:
    """A pixel with no return takes the nearest depth among its 8 neighbours
    when at least one has a return. One pass: seams close, open sky stays."""
    padded = np.pad(depth, 1, constant_values=np.inf)
    stacked = np.stack(
        [
            padded[1 + dy : 1 + dy + depth.shape[0], 1 + dx : 1 + dx + depth.shape[1]]
            for dy in (-1, 0, 1)
            for dx in (-1, 0, 1)
        ]
    )
    filled = stacked.min(axis=0)
    return np.where(np.isinf(depth), filled, depth)


def depth_ascii(
    depth: np.ndarray, *, max_depth: float | None, shape: tuple[int, int] = (64, 16)
) -> str:
    """The depth image as ``shape`` (columns, rows) of text: nearest return per
    block, digit 0..9 near..far over the colour range (see :func:`colour_range`)."""
    near, far = colour_range(depth, max_depth)
    cols, rows = shape
    height, width = depth.shape
    lines = []
    for r in range(rows):
        r0 = r * height // rows
        r1 = max((r + 1) * height // rows, r0 + 1)
        chars = []
        for c in range(cols):
            c0 = c * width // cols
            c1 = max((c + 1) * width // cols, c0 + 1)
            nearest = float(depth[r0:r1, c0:c1].min())
            if math.isinf(nearest):
                chars.append(".")
            else:
                chars.append(
                    str(min(9, int(9 * float(depth_fraction(np.array([nearest]), near, far)[0]))))
                )
        lines.append("".join(chars))
    return "\n".join(lines)


def _turbo() -> np.ndarray | None:
    """256 x 3 uint8 colour table, or None when matplotlib is unavailable."""
    try:
        from matplotlib import colormaps
    except ImportError:
        return None
    table = colormaps["turbo"](np.linspace(0.0, 1.0, 256))[:, :3]
    return (table * 255).astype(np.uint8)


def colour_range(depth: np.ndarray, max_depth: float | None) -> tuple[float, float]:
    """The depth span the colours cover: nearest to farthest return, so the
    palette always spans what is in view. (0, max_depth or 1) when nothing
    was hit."""
    hit = depth[np.isfinite(depth)]
    if len(hit) == 0:
        return 0.0, max_depth if max_depth is not None else 1.0
    near, far = float(hit.min()), float(hit.max())
    if far - near < 0.5:
        far = near + 0.5
    return round(near, 2), round(far, 2)


def depth_fraction(depth: np.ndarray, near: float, far: float) -> np.ndarray:
    """0 at ``near``, 1 at ``far``, on a log scale so that a metre of
    difference is visible up close and a far wall is still distinguishable
    from the floor in front of it."""
    near = max(near, 0.05)
    with np.errstate(divide="ignore", invalid="ignore"):
        f = np.log(np.maximum(depth, near) / near) / math.log(far / near)
    clipped: np.ndarray = np.clip(np.nan_to_num(f, nan=1.0, posinf=1.0), 0.0, 1.0)
    return clipped


def colour_scale(near: float, far: float, stops: int = 5) -> dict[str, Any]:
    """How to read depth off the image: the colour table's name, the colour at
    the near and far ends, evenly spaced stops with their depths, and the
    formula. ``f`` is a pixel colour's position along the table from the near
    colour (0) to the far colour (1)."""
    table = _turbo()
    near = max(near, 0.05)
    out: dict[str, Any] = {
        "scale": "matplotlib turbo, reversed" if table is not None else "grey, near = bright",
        "formula": "depth_m = near_m * (far_m / near_m) ** f; f = 0 at colour_near, 1 at colour_far, log spaced",
        "near_m": round(near, 2),
        "far_m": round(far, 2),
        "colour_none": [0, 0, 0],
        "none_means": "no return along that pixel's ray: open space, nothing within sensing range, or never observed; depth unknown, treat as infinity",
    }
    if table is not None:
        out["colour_near"] = [int(v) for v in table[255]]
        out["colour_far"] = [int(v) for v in table[0]]
        out["stops"] = []
        for i in range(stops):
            f = i / (stops - 1)
            out["stops"].append(
                {
                    "f": round(f, 2),
                    "rgb": [int(v) for v in table[round(255 * (1.0 - f))]],
                    "depth_m": round(near * (far / near) ** f, 2),
                }
            )
    return out


def depth_png(
    depth: np.ndarray, path: Path, *, max_depth: float | None
) -> tuple[Path, tuple[float, float]]:
    """Near is red through yellow and green to blue at the far end of the
    colour range; no return is black. Falls back to grey (near = bright)
    without matplotlib. Returns the path and the colour range in metres."""
    hit = np.isfinite(depth)
    near, far = colour_range(depth, max_depth)
    fraction = np.zeros(depth.shape, dtype=np.float32)
    fraction[hit] = depth_fraction(depth[hit], near, far)
    table = _turbo()
    if table is None:
        shade = np.zeros(depth.shape, dtype=np.uint8)
        shade[hit] = (255 * (1.0 - fraction[hit])).astype(np.uint8)
        image = Image.fromarray(shade, mode="L")
    else:
        index = (255 * (1.0 - fraction)).astype(np.uint8)
        rgb = table[index]
        rgb[~hit] = 0
        image = Image.fromarray(rgb, mode="RGB")
    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path)
    return path, (round(near, 2), round(far, 2))


# -- occupancy ------------------------------------------------------------------


def occupancy_grid(
    cloud: PointCloud2,
    points: np.ndarray,
    *,
    z_range: tuple[float, float],
    spacing: float,
    max_cells: int,
    free_radius: float,
    zoom: tuple[float, float, float] | None = None,
    heights: bool = False,
    _point_cells: dict[str, np.ndarray] | None = None,
) -> tuple[OccupancyGrid, np.ndarray | None]:
    """The mapping stack's general occupancy of the cloud over the absolute
    band ``z_range``: a return below it marks its cell free, inside it
    occupied, above it is ignored; free cells spread ``free_radius`` metres.
    The whole cloud by default; ``zoom`` (x, y, radius) crops a square. The
    cell is ``spacing`` (the typical gap between neighbouring returns) and
    grows when the map would need more than ``max_cells`` on a side. With
    ``heights``, also the highest return inside the band per cell (NaN where
    none), in the grid's row order."""

    def build(resolution: float) -> tuple[OccupancyGrid, np.ndarray | None]:
        grid = general_occupancy(
            cloud,
            resolution=resolution,
            min_height=z_range[0],
            max_height=z_range[1],
            mark_free_radius=free_radius,
        )
        top = cell_heights(grid, points, z_range) if heights else None
        indices = None
        if _point_cells is not None:
            indices = (
                (
                    points.astype(np.float64)[:, :2]
                    - [grid.origin.position.x, grid.origin.position.y]
                )
                / grid.resolution
            ).astype(np.int64)
            indices = np.clip(indices, [0, 0], [grid.width - 1, grid.height - 1])
        if zoom is not None:
            cropped = crop(grid, zoom[0], zoom[1], zoom[2])
            c0 = round((cropped.origin.position.x - grid.origin.position.x) / grid.resolution)
            r0 = round((cropped.origin.position.y - grid.origin.position.y) / grid.resolution)
            if top is not None:
                top = top[r0 : r0 + cropped.height, c0 : c0 + cropped.width]
            if indices is not None:
                indices -= [c0, r0]
            grid = cropped
        if _point_cells is not None and indices is not None:
            _point_cells["indices"] = indices
        return grid, top

    grid, top = build(spacing)
    side = max(grid.width, grid.height)
    if side > max_cells:
        # The builder pads the cloud's extent; size the cell from what it produced.
        grid, top = build(spacing * side / max_cells)
    return grid, top


def cell_heights(
    grid: OccupancyGrid, points: np.ndarray, z_range: tuple[float, float]
) -> np.ndarray:
    """The highest return inside ``z_range`` per cell of the uncropped
    ``grid``, in its row order; NaN where none landed. Cells are found the
    way the mapping stack's builder finds them, so every occupied cell has a
    height."""
    top = np.full(grid.grid.shape, -np.inf, dtype=np.float64)
    p = points.astype(np.float64)
    p = p[(p[:, 2] >= z_range[0]) & (p[:, 2] <= z_range[1])]
    if len(p):
        col = ((p[:, 0] - grid.origin.position.x) / grid.resolution).astype(np.int32)
        row = ((p[:, 1] - grid.origin.position.y) / grid.resolution).astype(np.int32)
        col = np.clip(col, 0, grid.width - 1)
        row = np.clip(row, 0, grid.height - 1)
        np.maximum.at(top, (row, col), p[:, 2])
    top[np.isinf(top)] = np.nan
    return top


def height_fraction(heights: np.ndarray, z_range: tuple[float, float]) -> np.ndarray:
    """0 at ``z_low``, 1 at ``z_high``, linear; NaN stays NaN."""
    lo, hi = z_range
    span = max(hi - lo, 1e-6)
    f: np.ndarray = np.clip((heights - lo) / span, 0.0, 1.0)
    return f


def height_scale(z_range: tuple[float, float], stops: int = 5) -> dict[str, Any]:
    """How to read a height off a colour='height' map: the colour at the low
    and high ends of the band, evenly spaced stops with their heights, and
    the formula. ``f`` is a pixel colour's position along the table from
    colour_low (0) to colour_high (1)."""
    table = _turbo()
    lo, hi = z_range
    out: dict[str, Any] = {
        "scale": "matplotlib turbo" if table is not None else "grey, high = bright",
        "formula": "z_m = z_low_m + f * (z_high_m - z_low_m); f = 0 at colour_low, 1 at colour_high, linear",
        "z_low_m": lo,
        "z_high_m": hi,
    }
    if table is not None:
        out["colour_low"] = [int(v) for v in table[0]]
        out["colour_high"] = [int(v) for v in table[255]]
        out["stops"] = [
            {
                "f": round(i / (stops - 1), 2),
                "rgb": [int(v) for v in table[round(255 * i / (stops - 1))]],
                "z_m": round(lo + (hi - lo) * i / (stops - 1), 2),
            }
            for i in range(stops)
        ]
    return out


def crop(grid: OccupancyGrid, x: float, y: float, radius: float) -> OccupancyGrid:
    """The part of ``grid`` within ``radius`` metres of (x, y), as its own grid."""
    g = grid.world_to_grid((x, y, 0.0))
    half = math.ceil(radius / grid.resolution)
    c0, c1 = max(0, math.floor(g.x) - half), min(grid.width, math.floor(g.x) + half + 1)
    r0, r1 = max(0, math.floor(g.y) - half), min(grid.height, math.floor(g.y) + half + 1)
    if c1 <= c0 or r1 <= r0:
        return grid
    origin = Pose()
    origin.position.x = grid.origin.position.x + c0 * grid.resolution
    origin.position.y = grid.origin.position.y + r0 * grid.resolution
    origin.orientation.w = 1.0
    return OccupancyGrid(
        grid=grid.grid[r0:r1, c0:c1].copy(),
        resolution=grid.resolution,
        origin=origin,
        frame_id=grid.frame_id,
        ts=grid.ts,
    )


def grid_north_up(grid: OccupancyGrid) -> np.ndarray:
    """Rows run north (top) to south, as on a map. The grid stores row 0 at
    the origin, the southern edge."""
    return np.flipud(grid.grid)


def cell_of(grid: OccupancyGrid, x: float, y: float) -> tuple[int, int]:
    """(column, row) of a world position in the north-up array; may fall
    outside the grid."""
    g = grid.world_to_grid((x, y, 0.0))
    col = math.floor(g.x)
    row = grid.height - 1 - math.floor(g.y)
    return col, row


def heading_char(yaw_deg: float) -> str:
    yaw = yaw_deg % 360.0
    if yaw < 45.0 or yaw >= 315.0:
        return ">"
    if yaw < 135.0:
        return "^"
    if yaw < 225.0:
        return "<"
    return "v"


def occupancy_ascii(
    grid: OccupancyGrid,
    *,
    mark: tuple[float, float, float] | None = None,
    heights: np.ndarray | None = None,
    z_range: tuple[float, float] | None = None,
    max_cols: int = 64,
) -> tuple[str, int]:
    """The map as text and the thinning step used. Blocks of ``step`` cells
    become one character: '#' if any is occupied (or, with ``heights`` and
    ``z_range``, a digit 0..9 for the highest return in the block), '.' if
    any is free, else '?'. ``mark`` (x, y, yaw_deg) is drawn as a heading
    arrow."""
    cells = grid_north_up(grid)
    rows, cols = cells.shape
    step = max(1, math.ceil(cols / max_cols))
    mark_col, mark_row = cell_of(grid, mark[0], mark[1]) if mark is not None else (-1, -1)
    digits = None
    if heights is not None and z_range is not None:
        digits = np.flipud(height_fraction(heights, z_range))
    lines = []
    for r in range(0, rows, step):
        chars = []
        for c in range(0, cols, step):
            block = cells[r : r + step, c : c + step]
            if mark is not None and r <= mark_row < r + step and c <= mark_col < c + step:
                chars.append(heading_char(mark[2]))
            elif (block == CostValues.OCCUPIED).any():
                if digits is None:
                    chars.append("#")
                else:
                    top = np.nanmax(digits[r : r + step, c : c + step])
                    chars.append(str(min(9, int(9 * float(top)))) if np.isfinite(top) else "#")
            elif (block == CostValues.FREE).any():
                chars.append(".")
            else:
                chars.append("?")
        lines.append("".join(chars))
    return "\n".join(lines), step


def occupancy_png(
    grid: OccupancyGrid,
    path: Path,
    *,
    mark: tuple[float, float, float] | None = None,
    heights: np.ndarray | None = None,
    z_range: tuple[float, float] | None = None,
    scale: int,
) -> Path:
    """White free, grey unseen, thin grey lines every metre. Occupied cells
    are black, or with ``heights`` and ``z_range`` coloured by their highest
    return from the low to the high end of the band. ``mark`` (x, y,
    yaw_deg) is drawn in red with a line along its heading. Each cell is
    ``scale`` pixels wide."""
    cells = grid_north_up(grid)
    rgb = np.full((*cells.shape, 3), 190, dtype=np.uint8)
    rgb[cells == CostValues.FREE] = 255
    rgb[cells == CostValues.OCCUPIED] = 0
    table = _turbo()
    if heights is not None and z_range is not None:
        fraction = np.flipud(height_fraction(heights, z_range))
        painted = (cells == CostValues.OCCUPIED) & np.isfinite(fraction)
        if table is not None:
            rgb[painted] = table[(255 * fraction[painted]).astype(np.uint8)]
        else:
            rgb[painted] = (255 * fraction[painted]).astype(np.uint8)[:, None]
    big = np.repeat(np.repeat(rgb, scale, axis=0), scale, axis=1)
    canvas = Image.fromarray(big, mode="RGB")
    draw = ImageDraw.Draw(canvas)
    metre = scale / grid.resolution
    x = 0.0
    while x < canvas.width:
        draw.line([(x, 0), (x, canvas.height)], fill=(150, 150, 150))
        x += metre
    y = float(canvas.height)
    while y > 0:
        draw.line([(0, y), (canvas.width, y)], fill=(150, 150, 150))
        y -= metre
    if mark is not None:
        col, row = cell_of(grid, mark[0], mark[1])
        cx, cy = (col + 0.5) * scale, (row + 0.5) * scale
        radius = max(3.0, scale * 1.5)
        yaw = math.radians(mark[2])
        tip = (cx + math.cos(yaw) * radius * 3, cy - math.sin(yaw) * radius * 3)
        draw.line([(cx, cy), tip], fill="red", width=max(1, scale // 2))
        draw.ellipse([cx - radius, cy - radius, cx + radius, cy + radius], fill="red")
    path.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(path)
    return path


# -- the encoding -----------------------------------------------------------------


def output_dir(out_dir: str | Path | None) -> Path:
    """Choose one absolute artifact directory with explicit caller precedence."""
    if out_dir is not None:
        selected = Path(out_dir)
    elif configured := os.environ.get("AGENT_ENCODE_DIR"):
        selected = Path(configured)
    elif run_dir := os.environ.get("DIMOS_RUN_LOG_DIR"):
        selected = Path(run_dir) / "agent_encode"
    else:
        selected = STATE_DIR / "agent_encode"
    return selected.expanduser().resolve()
