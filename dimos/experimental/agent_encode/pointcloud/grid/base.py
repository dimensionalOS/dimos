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

"""A top-down grid of cell values over the cloud's x/y."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import math
from pathlib import Path
from typing import TYPE_CHECKING, Literal

import numpy as np
from numpy.typing import NDArray
from scipy.spatial import cKDTree

from dimos.experimental.agent_encode.pointcloud.grid.lib import mask
from dimos.experimental.agent_encode.pointcloud.grid.lib.cells import overlap
from dimos.experimental.agent_encode.pointcloud.grid.regions import Regions, label
from dimos.experimental.agent_encode.pointcloud.image.lib.canvas import Drawable
from dimos.experimental.agent_encode.pointcloud.image.map import MapImage, grid_image

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@dataclass(frozen=True, eq=False)
class Grid:
    """Top-down cells over the cloud's x/y, world-aligned; values[row, col], row 0 is
    the south edge. A mask is a grid whose values are only 1, 0 or NaN."""

    origin: tuple[float, float]
    """World x, y of the south-west corner."""
    cell_m: float
    values: NDArray[np.float64]
    """(rows, columns); NaN is no data, never free."""
    cloud: PointCloud2 | None = None
    """The returns the cells were made from; None for grids computed from other grids."""

    def __post_init__(self) -> None:
        values = np.asarray(self.values, dtype=np.float64)
        if values.ndim != 2 or not values.size:
            raise ValueError("values must be a non-empty (rows, columns) array")
        if len(self.origin) != 2 or not np.isfinite(self.origin).all():
            raise ValueError("origin must be two finite coordinates")
        if not np.isfinite(self.cell_m) or self.cell_m <= 0:
            raise ValueError("cell_m must be positive and finite")
        object.__setattr__(self, "origin", (float(self.origin[0]), float(self.origin[1])))
        object.__setattr__(self, "values", values)

    @property
    def shape(self) -> tuple[int, int]:
        """(columns, rows)."""
        return self.values.shape[1], self.values.shape[0]

    def centres(self) -> NDArray[np.float64]:
        """World x, y of every cell centre, (rows, columns, 2)."""
        row, col = np.indices(self.values.shape)
        centres: NDArray[np.float64] = (
            np.stack((col, row), axis=-1) * self.cell_m + np.asarray(self.origin) + self.cell_m / 2
        )
        return centres

    def cell_of(self, xy: tuple[float, float]) -> tuple[int, int] | None:
        """(column, row) of the cell containing the point; None outside the grid."""
        col, row = np.floor((np.asarray(xy[:2], dtype=np.float64) - self.origin) / self.cell_m)
        if not (0 <= col < self.shape[0] and 0 <= row < self.shape[1]):
            return None
        return int(col), int(row)

    def at(self, xy: tuple[float, float]) -> float | None:
        """The containing cell's value; None outside the grid or where it has no data."""
        cell = self.cell_of(xy)
        if cell is None:
            return None
        value = float(self.values[cell[1], cell[0]])
        return value if math.isfinite(value) else None

    def near(self, xy: tuple[float, float], radius: float) -> tuple[float, float] | None:
        """(min, max) over the cells with data whose centre lies within ``radius`` of the
        point, and the cell containing it; None when there are none."""
        values = self.values[self.within(xy, radius)]
        values = values[np.isfinite(values)]
        if not len(values):
            return None
        return float(values.min()), float(values.max())

    def within(self, xy: tuple[float, float], radius: float) -> NDArray[np.bool_]:
        """Cells whose centre lies within ``radius`` of the point, and the cell containing
        it, as a (rows, columns) array."""
        if not math.isfinite(radius) or radius < 0:
            raise ValueError("radius must be finite and non-negative")
        near: NDArray[np.bool_] = (
            np.linalg.norm(self.centres() - np.asarray(xy[:2], dtype=np.float64), axis=-1) <= radius
        )
        cell = self.cell_of(xy)
        if cell is not None:
            near[cell[1], cell[0]] = True
        return near

    def window(
        self, area: tuple[tuple[float, float], tuple[float, float]], decimals: int = 3
    ) -> list[list[float | None]]:
        """Values of the cells whose centre lies in the closed area ((x0, y0), (x1, y1)),
        rows north to south; None where there is no data."""
        (x0, y0), (x1, y1) = area
        centres = self.centres()
        cols = np.flatnonzero((centres[0, :, 0] >= x0) & (centres[0, :, 0] <= x1))
        rows = np.flatnonzero((centres[:, 0, 1] >= y0) & (centres[:, 0, 1] <= y1))
        if len(cols) * len(rows) > 4096:
            raise ValueError(
                f"the area holds {len(cols)} x {len(rows)} cells, more than 4096; "
                "pass a smaller area"
            )
        block = np.round(self.values[np.ix_(rows[::-1], cols)], decimals)
        return [[float(v) if math.isfinite(v) else None for v in row] for row in block]

    def distance(self) -> Grid:
        """Metres from each cell centre to the nearest true cell centre of this mask; all
        NaN when no cell is true."""
        self._require_mask("distance()")
        targets = self.centres()[self.values == 1]
        distance = np.full(self.values.shape, np.nan)
        if len(targets):
            found, _ = cKDTree(targets).query(self.centres().reshape(-1, 2), workers=1)
            distance = found.reshape(self.values.shape)
        return self._derived(self.origin, distance)

    def regions(
        self, gap: int = 0, measure: Grid | None = None, connectivity: Literal[4, 8] = 8
    ) -> Regions:
        """Connected groups of this mask's true cells. ``gap`` links groups up to that many
        cells apart without adding the cells between; ``measure`` adds each region's
        statistics of another grid over the same cells."""
        self._require_mask("regions()")
        if type(gap) is not int or not 0 <= gap <= 4:
            raise ValueError("gap must be an integer from 0 to 4")
        if connectivity not in (4, 8):
            raise ValueError("connectivity must be 4 or 8")
        measured = None
        if measure is not None:
            if (
                measure.origin != self.origin
                or measure.cell_m != self.cell_m
                or measure.shape != self.shape
            ):
                raise ValueError(
                    "measure must cover the same cells as the mask: the same origin, "
                    "cell_m and shape"
                )
            measured = measure.values
        labels, regions = label(
            self.values, self.centres(), self.cell_m, connectivity, gap, measured
        )
        return Regions(regions, self._derived(self.origin, labels))

    def image(
        self,
        value_range: tuple[float, float] | None = None,
        draw: tuple[Drawable | PointCloud2, ...] = (),
        out_dir: Path | None = None,
    ) -> MapImage:
        """The grid as a top-down picture, coloured over ``value_range``."""
        return grid_image(self, value_range=value_range, draw=draw, out_dir=out_dir)

    def __gt__(self, other: float) -> Grid:
        return self._compare(other, np.greater)

    def __ge__(self, other: float) -> Grid:
        return self._compare(other, np.greater_equal)

    def __lt__(self, other: float) -> Grid:
        return self._compare(other, np.less)

    def __le__(self, other: float) -> Grid:
        return self._compare(other, np.less_equal)

    def __and__(self, other: Grid) -> Grid:
        return self._logic(other, "&", mask.both)

    def __or__(self, other: Grid) -> Grid:
        return self._logic(other, "|", mask.either)

    def __rand__(self, other: float) -> Grid:
        raise _precedence_error()

    def __ror__(self, other: float) -> Grid:
        raise _precedence_error()

    def __invert__(self) -> Grid:
        self._require_mask("~")
        return self._derived(self.origin, mask.negate(self.values))

    def __add__(self, other: Grid | float) -> Grid:
        return self._arithmetic(other, np.add)

    def __radd__(self, other: float) -> Grid:
        return self._derived(self.origin, other + self.values)

    def __sub__(self, other: Grid | float) -> Grid:
        return self._arithmetic(other, np.subtract)

    def __rsub__(self, other: float) -> Grid:
        return self._derived(self.origin, other - self.values)

    def __repr__(self) -> str:
        cols, rows = self.shape
        x0, y0 = self.origin
        finite = self.values[np.isfinite(self.values)]
        span = f"{finite.min():.4g}..{finite.max():.4g}" if len(finite) else "none"
        missing = 100 * (1 - len(finite) / self.values.size)
        return (
            f"Grid(cell {self.cell_m:g} m, {cols} x {rows} cells, "
            f"x {x0:.3f}..{x0 + cols * self.cell_m:.3f}, y {y0:.3f}..{y0 + rows * self.cell_m:.3f}, "
            f"values {span}, {missing:.0f}% no data)"
        )

    def _derived(self, origin: tuple[float, float], values: NDArray[np.float64]) -> Grid:
        return Grid(origin, self.cell_m, values)

    def _require_mask(self, operation: str) -> None:
        if not mask.is_mask(self.values):
            raise ValueError(f"{operation} requires a mask (a Grid of 1/0/no data), e.g. count > 0")

    def _compare(
        self, other: float, compare: Callable[[NDArray[np.float64], float], NDArray[np.bool_]]
    ) -> Grid:
        if isinstance(other, Grid) or not math.isfinite(other):
            raise TypeError("compare a Grid with a finite number, e.g. count > 0")
        finite = np.isfinite(self.values)
        return self._derived(self.origin, np.where(finite, compare(self.values, other), np.nan))

    def _pair(self, other: Grid) -> tuple[Grid, Grid]:
        """Both grids over the cells they share, at the finer of their cells."""
        cell = min(self.cell_m, other.cell_m)
        first, second = self._refined(cell), other._refined(cell)
        origin, mine, theirs = overlap(
            (first.origin, first.shape), (second.origin, second.shape), cell
        )
        return Grid(origin, cell, first.values[mine]), Grid(origin, cell, second.values[theirs])

    def _refined(self, cell_m: float) -> Grid:
        """This grid at ``cell_m``, each cell repeated over the finer cells inside it."""
        factor = self.cell_m / cell_m
        repeat = round(factor)
        if not math.isclose(factor, repeat, rel_tol=1e-6):
            raise ValueError(
                f"cells of {cell_m:g} m and {self.cell_m:g} m do not nest, so the grids cannot "
                "be combined; use cell sizes where one is a whole multiple of the other"
            )
        if repeat == 1:
            return self
        values = np.repeat(np.repeat(self.values, repeat, axis=0), repeat, axis=1)
        return Grid(self.origin, cell_m, values)

    def _logic(
        self,
        other: Grid,
        operation: str,
        combine: Callable[[NDArray[np.float64], NDArray[np.float64]], NDArray[np.float64]],
    ) -> Grid:
        if not isinstance(other, Grid):
            raise _precedence_error()
        self._require_mask(operation)
        other._require_mask(operation)
        first, second = self._pair(other)
        return Grid(first.origin, first.cell_m, combine(first.values, second.values))

    def _arithmetic(
        self,
        other: Grid | float,
        combine: Callable[[NDArray[np.float64], NDArray[np.float64] | float], NDArray[np.float64]],
    ) -> Grid:
        if isinstance(other, Grid):
            first, second = self._pair(other)
            return Grid(first.origin, first.cell_m, combine(first.values, second.values))
        return self._derived(self.origin, combine(self.values, float(other)))


def _precedence_error() -> TypeError:
    return TypeError(
        "& and | combine two masks, and they bind tighter than comparisons: "
        "write (grid > 1.1) & mask, not grid > 1.1 & mask"
    )
