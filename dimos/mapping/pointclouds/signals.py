# Copyright 2025-2026 Dimensional Inc.
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

"""Pure-numpy signals derived from an (N, 3) lidar point cloud.

Stateless helpers that turn the raw array from
``LidarPointCloudClient.snapshot()`` into the quantitative products the
wishlist item-3 objectives need:

- **Measure the space** — ``extents`` (bounding box + room dimensions),
  ``ground_height``.
- **Reactive clearance / safety** — ``nearest_obstacle``, ``clearance``
  (range within an angular sector ahead), ``region_is_clear``.
- **Portable map artifact** — ``occupancy_grid`` (top-down (H, W)),
  ``height_map`` (2.5D max-z per cell).
- **Coverage feedback** — ``coverage_area`` (scanned floor area).

Shared building blocks: ``crop``, ``filter_height``, ``voxel_downsample``.

Every function takes a plain float array of shape (N, 3) in one common frame
(typically ``world``/``map``) and returns plain numpy / scalars — no LCM, no
open3d, no module graph — so they run in a script, a notebook, or a skill.
Functions whose answer is undefined on an empty cloud (``extents``,
``ground_height``) raise ``ValueError``; the "distance to nearest thing"
queries instead return ``inf`` (nothing nearby == infinitely clear), and the
grid/area products return empty/zero, so safety and coverage callers never
have to special-case an empty scan.
"""

from __future__ import annotations

from typing import NamedTuple

import numpy as np
from numpy.typing import NDArray

Points = NDArray[np.floating]
HeightBand = tuple[float, float]


def _as_xyz(pts: NDArray[np.floating]) -> NDArray[np.float64]:
    """Validate and coerce to an (N, 3) float64 array."""
    arr = np.asarray(pts, dtype=np.float64)
    if arr.ndim != 2 or arr.shape[1] != 3:
        raise ValueError(f"expected (N, 3) points, got shape {arr.shape}")
    return arr


# --------------------------------------------------------------------------- #
# Building blocks                                                             #
# --------------------------------------------------------------------------- #
def crop(pts: Points, lower: tuple[float, float, float], upper: tuple[float, float, float]) -> Points:
    """Keep only points inside the axis-aligned box [lower, upper] (inclusive)."""
    arr = _as_xyz(pts)
    lo = np.asarray(lower, dtype=np.float64)
    hi = np.asarray(upper, dtype=np.float64)
    mask = np.all((arr >= lo) & (arr <= hi), axis=1)
    return arr[mask]


def filter_height(pts: Points, z_min: float = -np.inf, z_max: float = np.inf) -> Points:
    """Keep only points whose z is within [z_min, z_max]."""
    arr = _as_xyz(pts)
    mask = (arr[:, 2] >= z_min) & (arr[:, 2] <= z_max)
    return arr[mask]


def voxel_downsample(pts: Points, voxel_size: float = 0.05) -> Points:
    """Collapse points into one representative per occupied voxel.

    Pure-numpy dedup for the duplicate points overlapping sweeps produce, so
    downstream work scales with occupied volume, not raw message count. Keeps
    the first point seen in each voxel (order-stable).
    """
    if voxel_size <= 0:
        raise ValueError("voxel_size must be > 0")
    arr = _as_xyz(pts)
    if len(arr) == 0:
        return arr
    keys = np.floor(arr / voxel_size).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return arr[np.sort(idx)]


# --------------------------------------------------------------------------- #
# Objective 1: measure the space                                              #
# --------------------------------------------------------------------------- #
class Extents(NamedTuple):
    """Axis-aligned bounds of a cloud. ``span`` is the room's (x, y, z) size."""

    min: NDArray[np.float64]  # (3,)
    max: NDArray[np.float64]  # (3,)
    span: NDArray[np.float64]  # (3,) = max - min
    center: NDArray[np.float64]  # (3,) = (min + max) / 2


def extents(pts: Points, percentiles: tuple[float, float] | None = None) -> Extents:
    """Bounding box, per-axis span (room dimensions), and center of the cloud.

    Args:
        percentiles: Optional (lo, hi) per-axis percentile bounds, e.g.
            ``(1.0, 99.0)``. None (default) = exact min/max. Real lidar
            accumulations carry stray long-range returns (reflections,
            through-glass hits) that inflate exact bounds wildly -- measured
            on the china-office session: 144x184x105m raw vs a ~44x76x10m
            building. Same idea as ``ground_height`` and
            ``SceneModel.from_rrd``'s z clipping.
    """
    arr = _as_xyz(pts)
    if len(arr) == 0:
        raise ValueError("cannot compute extents of an empty cloud")
    if percentiles is not None:
        lo, hi = percentiles
        mn = np.percentile(arr, lo, axis=0)
        mx = np.percentile(arr, hi, axis=0)
    else:
        mn = arr.min(axis=0)
        mx = arr.max(axis=0)
    return Extents(min=mn, max=mx, span=mx - mn, center=(mn + mx) / 2.0)


def ground_height(pts: Points, percentile: float = 2.0) -> float:
    """Estimate the floor's z as a low percentile of z (robust to a few strays)."""
    arr = _as_xyz(pts)
    if len(arr) == 0:
        raise ValueError("cannot estimate ground height of an empty cloud")
    return float(np.percentile(arr[:, 2], percentile))


# --------------------------------------------------------------------------- #
# Objective 2: reactive clearance / safety                                    #
# --------------------------------------------------------------------------- #
def _horizontal(arr: NDArray[np.float64], origin: tuple[float, float, float]):
    """Horizontal distance and bearing of each point relative to origin."""
    o = np.asarray(origin, dtype=np.float64)
    dx = arr[:, 0] - o[0]
    dy = arr[:, 1] - o[1]
    return np.hypot(dx, dy), np.arctan2(dy, dx)


def nearest_obstacle(
    pts: Points,
    origin: tuple[float, float, float] = (0.0, 0.0, 0.0),
    height_band: HeightBand | None = None,
) -> float:
    """Horizontal distance (m) from ``origin`` to the closest point.

    Pass ``height_band=(z_min, z_max)`` to ignore the floor/ceiling and only
    consider things at body height. Returns ``inf`` when nothing qualifies.
    """
    arr = _as_xyz(pts)
    if height_band is not None:
        arr = filter_height(arr, *height_band)
    if len(arr) == 0:
        return float("inf")
    dist, _ = _horizontal(arr, origin)
    return float(dist.min())


def clearance(
    pts: Points,
    heading: float,
    fov_deg: float = 60.0,
    origin: tuple[float, float, float] = (0.0, 0.0, 0.0),
    height_band: HeightBand | None = None,
) -> float:
    """Distance (m) to the closest point within a cone of ``fov_deg`` around
    ``heading`` (radians, world frame) as seen from ``origin``.

    The "how far can I go this way?" query. Returns ``inf`` when the cone is
    empty (clear).
    """
    arr = _as_xyz(pts)
    if height_band is not None:
        arr = filter_height(arr, *height_band)
    if len(arr) == 0:
        return float("inf")
    dist, bearing = _horizontal(arr, origin)
    # Signed angular difference wrapped to [-pi, pi].
    delta = np.angle(np.exp(1j * (bearing - heading)))
    in_cone = np.abs(delta) <= np.deg2rad(fov_deg) / 2.0
    if not in_cone.any():
        return float("inf")
    return float(dist[in_cone].min())


def region_is_clear(
    pts: Points,
    lower: tuple[float, float, float],
    upper: tuple[float, float, float],
) -> bool:
    """True when no point falls inside the axis-aligned box [lower, upper]."""
    arr = _as_xyz(pts)
    lo = np.asarray(lower, dtype=np.float64)
    hi = np.asarray(upper, dtype=np.float64)
    inside = np.all((arr >= lo) & (arr <= hi), axis=1)
    return not bool(inside.any())


# --------------------------------------------------------------------------- #
# Objective 3: portable map artifacts                                         #
# --------------------------------------------------------------------------- #
class OccupancyGrid2D(NamedTuple):
    """Top-down occupancy raster.

    ``grid[row, col]`` is True where a point fell in that cell. ``origin`` is
    the world (x, y) of the ``grid[0, 0]`` cell's lower corner; cell (r, c)
    covers world x in [origin_x + c*res, +res], y in [origin_y + r*res, +res].
    """

    grid: NDArray[np.bool_]  # (H, W)
    origin: NDArray[np.float64]  # (2,)
    resolution: float


def _grid_bounds(
    arr: NDArray[np.float64], bounds: tuple[float, float, float, float] | None
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    if bounds is not None:
        min_x, min_y, max_x, max_y = bounds
        return np.array([min_x, min_y]), np.array([max_x, max_y])
    if len(arr) == 0:
        raise ValueError("cannot infer grid bounds from an empty cloud; pass bounds=")
    return arr[:, :2].min(axis=0), arr[:, :2].max(axis=0)


def occupancy_grid(
    pts: Points,
    resolution: float = 0.1,
    height_band: HeightBand | None = None,
    bounds: tuple[float, float, float, float] | None = None,
) -> OccupancyGrid2D:
    """Rasterize the cloud into a top-down boolean occupancy grid.

    ``resolution`` is meters per cell. ``height_band`` restricts which z's
    count as occupancy (e.g. drop the floor). ``bounds`` fixes the extent
    (min_x, min_y, max_x, max_y); otherwise it's taken from the cloud.
    """
    if resolution <= 0:
        raise ValueError("resolution must be > 0")
    arr = _as_xyz(pts)
    if height_band is not None:
        arr = filter_height(arr, *height_band)
    lo, hi = _grid_bounds(arr, bounds)
    width = int(np.ceil((hi[0] - lo[0]) / resolution)) + 1
    height = int(np.ceil((hi[1] - lo[1]) / resolution)) + 1
    grid = np.zeros((height, width), dtype=np.bool_)
    if len(arr):
        cols = np.floor((arr[:, 0] - lo[0]) / resolution).astype(np.int64)
        rows = np.floor((arr[:, 1] - lo[1]) / resolution).astype(np.int64)
        keep = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
        grid[rows[keep], cols[keep]] = True
    return OccupancyGrid2D(grid=grid, origin=np.asarray(lo, dtype=np.float64), resolution=resolution)


class HeightMap(NamedTuple):
    """2.5D max-height raster; empty cells are NaN. Same geometry as OccupancyGrid2D."""

    grid: NDArray[np.float64]  # (H, W), NaN where no point
    origin: NDArray[np.float64]  # (2,)
    resolution: float


def height_map(
    pts: Points,
    resolution: float = 0.1,
    bounds: tuple[float, float, float, float] | None = None,
) -> HeightMap:
    """Max z per top-down cell (NaN where empty) — a simple 2.5D surface."""
    if resolution <= 0:
        raise ValueError("resolution must be > 0")
    arr = _as_xyz(pts)
    lo, hi = _grid_bounds(arr, bounds)
    width = int(np.ceil((hi[0] - lo[0]) / resolution)) + 1
    height = int(np.ceil((hi[1] - lo[1]) / resolution)) + 1
    grid = np.full((height, width), np.nan, dtype=np.float64)
    if len(arr):
        cols = np.floor((arr[:, 0] - lo[0]) / resolution).astype(np.int64)
        rows = np.floor((arr[:, 1] - lo[1]) / resolution).astype(np.int64)
        keep = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
        cr, rr, zr = cols[keep], rows[keep], arr[keep, 2]
        # np.maximum.at accumulates a per-cell max; seed with -inf then restore NaN.
        seeded = np.full((height, width), -np.inf, dtype=np.float64)
        np.maximum.at(seeded, (rr, cr), zr)
        filled = seeded != -np.inf
        grid[filled] = seeded[filled]
    return HeightMap(grid=grid, origin=np.asarray(lo, dtype=np.float64), resolution=resolution)


# --------------------------------------------------------------------------- #
# Objective 6: coverage feedback                                              #
# --------------------------------------------------------------------------- #
def coverage_area(
    pts: Points,
    resolution: float = 0.1,
    height_band: HeightBand | None = None,
) -> float:
    """Scanned floor area (m^2): occupied cells x cell area. 0.0 on an empty cloud."""
    arr = _as_xyz(pts)
    if height_band is not None:
        arr = filter_height(arr, *height_band)
    if len(arr) == 0:
        return 0.0
    grid = occupancy_grid(arr, resolution=resolution).grid
    return float(grid.sum()) * resolution * resolution
