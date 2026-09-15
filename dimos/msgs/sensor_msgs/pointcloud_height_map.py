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

"""Agent encoding of a point cloud: exact native bounds plus a top-down height map.

Everything is expressed in the cloud's own frame. Nothing here assumes a floor,
a robot, a sensor mount or a gravity direction; callers supply any such context
through the explicit options of :func:`encode_points`.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np
from numpy.typing import NDArray

GLYPHS = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"
"""Base-36 digits; one character per height-map cell."""

DEFAULT_CELLS = 48
MAX_CELLS = 120
FOOTPRINT_CELL_M = 0.2
LARGEST_GAPS = 4
SECTORS = 36

LEGEND = (
    "Coordinates are meters in frame_id, as stored, with no floor, robot or gravity "
    "assumption; ts is the timestamp. num_points counts stored returns of source_dtype "
    "precision; nonfinite_points of them are excluded. selection echoes the explicit "
    "filters applied (null = whole "
    "cloud); points_selected is how many finite returns pass them. bounds_m are the exact "
    "stored extrema per axis of the selected returns; distinct_coordinates counts distinct "
    "values per axis; largest_gaps_m lists the four widest intervals between consecutive "
    "distinct coordinates per axis as [low, high, width], widest first. centroid_xy_m is "
    "the mean XY; footprint_m2 counts occupied 0.2 m XY cells times 0.04. height_map is a "
    "top-down grid: cell_m is the cell size; x_centers_m and y_centers_m give the cell "
    "center coordinates. Row j of zmax_rows and zmin_rows lies at y_centers_m[j] (first "
    "row = largest y) and character i of a row at x_centers_m[i]. Each character is a "
    "base-36 digit k (0-9 then A-Z): the highest (zmax_rows) or lowest (zmin_rows) return "
    "z in that cell satisfies glyph_z_m[k] <= z < glyph_z_m[k] + z_step_m, where "
    "glyph_z_m[k] = z0_m + k*z_step_m. "
    "'.' means the cell holds no selected return; it does not establish free space. "
    "With a center, range_profile_m lists the horizontal distance from the center to the "
    "nearest selected return in each of 36 bearing sectors of 10 degrees, counterclockwise "
    "from +x in the XY plane (entry k spans 10k-5 to 10k+5 degrees), null when the sector "
    "holds none. agent_encode options: center=(x, y), radius=r (square half-width; omitted "
    "= whole cloud), z_range=(low, high), cell=m, cells=n (max columns/rows, default 48, "
    "limit 120), z_step=m."
)

_STEP_LADDER = (1.0, 2.0, 2.5, 5.0)


def nice_step(minimum: float) -> float:
    """Smallest 1, 2, 2.5 or 5 times a power of ten that is at least `minimum`."""
    if not math.isfinite(minimum) or minimum <= 0:
        raise ValueError("step must be positive and finite")
    exponent = math.floor(math.log10(minimum))
    for mantissa in _STEP_LADDER:
        step = float(f"{mantissa * 10.0**exponent:.3g}")
        if step >= minimum * (1 - 1e-12):
            return step
    return float(f"{10.0 ** (exponent + 1):.3g}")


def _clean(value: float) -> float:
    """Strip binary float noise from a product of round decimal steps."""
    return float(f"{value:.12g}")


def _decimals(value: float) -> int:
    """Decimal places needed to print `value` exactly enough for labels."""
    text = f"{value:.12g}"
    if "e" in text:
        mantissa, exponent = text.split("e")
        return max(0, len(mantissa.split(".")[1]) if "." in mantissa else 0) - int(exponent)
    return len(text.split(".")[1]) if "." in text else 0


def _axis_extrema(values: NDArray[np.float64]) -> dict[str, Any]:
    distinct = np.unique(values)
    gaps = np.diff(distinct)
    order = np.argsort(-gaps, kind="stable")[:LARGEST_GAPS]
    return {
        "bounds": [float(distinct[0]), float(distinct[-1])],
        "distinct": len(distinct),
        "gaps": [
            [float(distinct[i]), float(distinct[i + 1]), float(gaps[i])]
            for i in order
            if gaps[i] > 0
        ],
    }


def _range_profile(
    points: NDArray[np.float64], center: tuple[float, float], decimals: int
) -> list[float | None]:
    """Nearest selected return per bearing sector, measured from an explicit center."""
    offsets = points[:, :2] - np.array(center)
    distances = np.hypot(offsets[:, 0], offsets[:, 1])
    bearings = np.degrees(np.arctan2(offsets[:, 1], offsets[:, 0]))
    sectors = np.floor((bearings + 180 / SECTORS) / (360 / SECTORS)).astype(np.int64) % SECTORS
    nearest = np.full(SECTORS, np.inf)
    np.minimum.at(nearest, sectors, distances)
    return [round(float(r), decimals) if np.isfinite(r) else None for r in nearest]


def _check_range(values: NDArray[np.float64], cell: float) -> None:
    if not np.isfinite(values).all() or np.any(np.abs(values) / cell >= 2.0**52):
        raise ValueError("Point cloud coordinates exceed the encoder's numeric range")


def _height_map(
    points: NDArray[np.float64],
    window: tuple[NDArray[np.float64], NDArray[np.float64]],
    cell: float | None,
    cells: int,
    z_step: float | None,
) -> dict[str, Any]:
    """Grid the window (lower, upper XY corners) and quantize z per cell."""
    lower, upper = window
    span = float(max(upper[0] - lower[0], upper[1] - lower[1]))
    if cell is not None:
        # An explicit cell may exceed the default grid up to the hard limit;
        # beyond that it coarsens to the next round size and reports cell_m.
        cells = MAX_CELLS
    else:
        cell = nice_step(span / (cells - 1)) if span > 0 else 1.0
    while True:
        _check_range(np.concatenate([lower, upper]), cell)
        origin = np.floor(lower / cell) * cell
        shape = (np.floor((upper - origin) / cell) + 1).astype(np.int64)
        if shape.max() <= cells:
            break
        cell = nice_step(cell * (1 + 1e-9))
    columns, rows = int(shape[0]), int(shape[1])
    zmax = np.full((rows, columns), -np.inf)
    zmin = np.full((rows, columns), np.inf)
    if len(points):
        _check_range(points[:, 2], 1.0)
        ix = np.floor((points[:, 0] - origin[0]) / cell).astype(np.int64)
        iy = np.floor((points[:, 1] - origin[1]) / cell).astype(np.int64)
        np.maximum.at(zmax, (iy, ix), points[:, 2])
        np.minimum.at(zmin, (iy, ix), points[:, 2])
        z_low, z_high = float(points[:, 2].min()), float(points[:, 2].max())
    else:
        z_low, z_high = 0.0, 0.0
    if z_step is None:
        z_step = nice_step((z_high - z_low) / (len(GLYPHS) - 1)) if z_high > z_low else cell
    while True:
        z0 = _clean(math.floor(z_low / z_step) * z_step)
        if (z_high - z0) / z_step < len(GLYPHS):
            break
        z_step = nice_step(z_step * (1 + 1e-9))
    origin = np.array([_clean(float(v)) for v in origin])
    levels = int(np.floor((z_high - z0) / z_step)) + 1 if len(points) else 1
    occupied = np.isfinite(zmax)
    glyphs = np.array(list(GLYPHS))

    def render(values: NDArray[np.float64]) -> list[str]:
        chars = np.full(values.shape, ".", dtype="<U1")
        index = np.clip(np.floor((values[occupied] - z0) / z_step), 0, len(GLYPHS) - 1)
        chars[occupied] = glyphs[index.astype(np.int64)]
        return ["".join(chars[j]) for j in range(rows - 1, -1, -1)]

    decimals = _decimals(cell / 2)
    return {
        "cell_m": cell,
        "x_centers_m": [
            round(float(origin[0] + (i + 0.5) * cell), decimals) for i in range(columns)
        ],
        "y_centers_m": [
            round(float(origin[1] + (j + 0.5) * cell), decimals) for j in range(rows - 1, -1, -1)
        ],
        "z0_m": z0,
        "z_step_m": z_step,
        "glyph_z_m": [_clean(z0 + k * z_step) for k in range(levels)],
        "zmax_rows": render(zmax),
        "zmin_rows": render(zmin),
    }


def encode_points(
    stored: NDArray[Any],
    *,
    frame_id: str,
    ts: float | None,
    center: tuple[float, float] | None = None,
    radius: float | None = None,
    z_range: tuple[float, float] | None = None,
    cell: float | None = None,
    cells: int = DEFAULT_CELLS,
    z_step: float | None = None,
) -> dict[str, Any]:
    """Encode an (N, 3) array of stored coordinates. See :data:`LEGEND` for the fields."""
    if radius is not None and center is None:
        raise ValueError("radius needs a center")
    if radius is not None and not (math.isfinite(radius) and radius > 0):
        raise ValueError("radius must be positive and finite")
    cells = min(max(int(cells), 2), MAX_CELLS)
    if cell is not None and not (math.isfinite(cell) and cell > 0):
        raise ValueError("cell must be positive and finite")
    if z_step is not None and not (math.isfinite(z_step) and z_step > 0):
        raise ValueError("z_step must be positive and finite")
    finite = np.isfinite(stored).all(axis=1) if len(stored) else np.zeros(0, dtype=bool)
    points = stored[finite].astype(np.float64)
    selection: dict[str, Any] | None = None
    window: tuple[NDArray[np.float64], NDArray[np.float64]] | None = None
    if center is not None:
        cx, cy = float(center[0]), float(center[1])
        if not (math.isfinite(cx) and math.isfinite(cy)):
            raise ValueError("center must be finite")
        selection = {"center_xy": [cx, cy], "radius_m": None}
        if radius is not None:
            keep = (np.abs(points[:, 0] - cx) <= radius) & (np.abs(points[:, 1] - cy) <= radius)
            points = points[keep]
            window = (np.array([cx - radius, cy - radius]), np.array([cx + radius, cy + radius]))
            selection = {"center_xy": [cx, cy], "radius_m": float(radius), "shape": "square"}
    if z_range is not None:
        z_low, z_high = float(z_range[0]), float(z_range[1])
        if not z_low <= z_high:
            raise ValueError("z_range must be (low, high) with low <= high")
        points = points[(points[:, 2] >= z_low) & (points[:, 2] <= z_high)]
        selection = {**(selection or {}), "z_range_m": [z_low, z_high]}
    out: dict[str, Any] = {
        "frame_id": frame_id,
        "ts": None if ts is None else float(ts),
        "num_points": int(stored.shape[0]),
        "nonfinite_points": int((~finite).sum()),
        "source_dtype": str(stored.dtype),
        "selection": selection,
        "points_selected": len(points),
        "bounds_m": {axis: [] for axis in "xyz"},
        "distinct_coordinates": {axis: 0 for axis in "xyz"},
        "largest_gaps_m": {axis: [] for axis in "xyz"},
        "centroid_xy_m": [],
        "footprint_m2": 0.0,
        "height_map": None,
        "range_profile_m": None,
    }
    if len(points):
        lower, upper = points.min(axis=0), points.max(axis=0)
        with np.errstate(over="ignore"):
            spans = upper - lower
        if not np.isfinite(spans).all():
            raise ValueError("Point cloud extent exceeds the encoder's numeric range")
        for axis, name in enumerate("xyz"):
            extrema = _axis_extrema(points[:, axis])
            out["bounds_m"][name] = extrema["bounds"]
            out["distinct_coordinates"][name] = extrema["distinct"]
            out["largest_gaps_m"][name] = extrema["gaps"]
        # Summing offsets from the lower corner in sorted order keeps the mean
        # independent of return order and safe for very large coordinates.
        offsets = np.sort(points[:, :2] - lower[:2], axis=0)
        mean = lower[:2] + (offsets / len(points)).sum(axis=0)
        _check_range(points[:, :2], FOOTPRINT_CELL_M)
        occupied = np.unique(np.floor(points[:, :2] / FOOTPRINT_CELL_M), axis=0)
        out["footprint_m2"] = round(float(len(occupied)) * FOOTPRINT_CELL_M**2, 2)
        if window is None:
            window = (lower[:2].copy(), upper[:2].copy())
    if window is not None:
        out["height_map"] = _height_map(points, window, cell, cells, z_step)
        decimals = _decimals(out["height_map"]["cell_m"] / 2) + 1
        if len(points):
            out["centroid_xy_m"] = [round(float(v), decimals) for v in mean]
        if center is not None:
            out["range_profile_m"] = _range_profile(points, center, decimals)
    return out
