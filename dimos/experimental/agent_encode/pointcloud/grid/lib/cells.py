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

"""World-aligned cell layouts, returns binned into them, and where two layouts overlap."""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import NDArray

from dimos.experimental.agent_encode.pointcloud import constants
from dimos.experimental.agent_encode.pointcloud.queries.lib.points import finite_points

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def aligned(
    area: tuple[tuple[float, float], tuple[float, float]], cell_m: float
) -> tuple[tuple[float, float], tuple[int, int]]:
    """Origin and (columns, rows) of the cells, aligned to multiples of ``cell_m``,
    covering every point of the closed ``((x0, y0), (x1, y1))`` area."""
    low = np.array(area[0], dtype=np.float64)
    high = np.array(area[1], dtype=np.float64)
    index = np.floor(low / cell_m)
    origin = np.round(index * cell_m, 6)
    origin = np.where(origin > low, np.round((index - 1) * cell_m, 6), origin)
    shape = np.floor((high - origin) / cell_m).astype(np.int64) + 1
    return (float(origin[0]), float(origin[1])), (int(shape[0]), int(shape[1]))


def cover(
    points_xy: NDArray[np.float32] | NDArray[np.float64], cell_m: float
) -> tuple[tuple[float, float], tuple[int, int], float, bool]:
    """Square XY cells covering every return, aligned to multiples of ``cell_m``:
    origin, (columns, rows), the cell and whether it grew.

    Alignment and cell size do not depend on the cloud's extent, so the same
    scene measures the same in any frame that holds it. The cell grows only when
    the grid would exceed ``constants.MAX_GRID_CELLS``, the encoder's memory guard.
    """
    points = np.asarray(points_xy, dtype=np.float64)
    low, high = points[:, :2].min(axis=0), points[:, :2].max(axis=0)
    area = ((float(low[0]), float(low[1])), (float(high[0]), float(high[1])))
    cell, limited = round(cell_m, 6), False
    while True:
        origin, shape = aligned(area, cell)
        if shape[0] * shape[1] <= constants.MAX_GRID_CELLS:
            break
        excess = shape[0] * shape[1] / constants.MAX_GRID_CELLS
        cell = float(np.ceil(cell * np.sqrt(excess) * 1000) / 1000)
        limited = True
    return origin, shape, cell, limited


def check_area(area: tuple[tuple[float, float], tuple[float, float]]) -> None:
    """Raise unless ``area`` is two finite corners, the first south-west of the second."""
    corners = np.asarray(area, dtype=np.float64)
    if corners.shape != (2, 2) or not np.isfinite(corners).all():
        raise ValueError("area must be ((x0, y0), (x1, y1)) with finite coordinates")
    if (corners[0] > corners[1]).any():
        raise ValueError("area must list the south-west corner first: x0 <= x1 and y0 <= y1")


@dataclass(frozen=True, eq=False)
class Binned:
    """A cloud's finite returns sorted into the cells of an aligned layout."""

    origin: tuple[float, float]
    """World x, y of the south-west corner."""
    shape: tuple[int, int]
    """(columns, rows)."""
    index: NDArray[np.int64]
    """Flat cell (row * columns + column) of each return inside the cells."""
    z: NDArray[np.float64]
    """The z of each of those returns."""

    def count(self) -> NDArray[np.int64]:
        """Returns per cell, flat."""
        counts: NDArray[np.int64] = np.bincount(self.index, minlength=self.shape[0] * self.shape[1])
        return counts

    def lowest(self) -> NDArray[np.float64]:
        """Lowest z per cell, flat; NaN where there is none."""
        low = np.full(self.shape[0] * self.shape[1], np.inf)
        np.minimum.at(low, self.index, self.z)
        low[~np.isfinite(low)] = np.nan
        return low

    def highest(self) -> NDArray[np.float64]:
        """Highest z per cell, flat; NaN where there is none."""
        high = np.full(self.shape[0] * self.shape[1], -np.inf)
        np.maximum.at(high, self.index, self.z)
        high[~np.isfinite(high)] = np.nan
        return high

    def any(self, keep: NDArray[np.bool_]) -> NDArray[np.bool_]:
        """Whether each cell holds a return that ``keep`` marks, flat."""
        hits: NDArray[np.bool_] = self.count_of(keep) > 0
        return hits

    def count_of(self, keep: NDArray[np.bool_]) -> NDArray[np.int64]:
        """Returns per cell that ``keep`` marks, flat."""
        counts: NDArray[np.int64] = np.bincount(
            self.index[keep], minlength=self.shape[0] * self.shape[1]
        )
        return counts

    def rows(self, flat: NDArray[np.float64]) -> NDArray[np.float64]:
        """A flat per-cell array as (rows, columns)."""
        return flat.reshape(self.shape[1], self.shape[0])


def bin_returns(
    cloud: PointCloud2, cell_m: float, area: tuple[tuple[float, float], tuple[float, float]] | None
) -> Binned:
    """The cloud's finite returns in cells of ``cell_m`` over ``area``, or over every
    return when it is None."""
    if not np.isfinite(cell_m) or cell_m <= 0:
        raise ValueError("cell_m must be positive and finite")
    points = finite_points(cloud).astype(np.float64)
    if area is None:
        if not len(points):
            raise ValueError("the cloud has no finite returns to cover; pass area=")
        low, high = points[:, :2].min(axis=0), points[:, :2].max(axis=0)
        area = ((float(low[0]), float(low[1])), (float(high[0]), float(high[1])))
    check_area(area)
    origin, shape = aligned(area, cell_m)
    if shape[0] * shape[1] > constants.MAX_GRID_CELLS:
        raise ValueError(
            f"{shape[0]} x {shape[1]} cells is more than {constants.MAX_GRID_CELLS}; "
            "pass a smaller area= or a larger cell_m"
        )
    ij = np.floor((points[:, :2] - np.asarray(origin)) / cell_m).astype(np.int64)
    inside = ((ij >= 0) & (ij < np.asarray(shape))).all(axis=-1)
    index = ij[inside, 1] * shape[0] + ij[inside, 0]
    return Binned(origin, shape, index, points[inside, 2])


def overlap(
    first: tuple[tuple[float, float], tuple[int, int]],
    second: tuple[tuple[float, float], tuple[int, int]],
    cell_m: float,
) -> tuple[tuple[float, float], tuple[slice, slice], tuple[slice, slice]]:
    """Where two layouts of ``cell_m`` cells, each (origin, (columns, rows)), overlap:
    its origin and each layout's (row, column) slices over it."""
    offset = (np.asarray(second[0]) - np.asarray(first[0])) / cell_m
    steps = np.round(offset)
    if not np.allclose(offset, steps, atol=1e-6):
        raise ValueError(
            f"grids at origins {first[0]} and {second[0]} are not aligned to the same "
            f"{cell_m} m cells; build both from a cloud with the same cell_m"
        )
    origin: list[float] = []
    spans: list[tuple[slice, slice]] = []
    for axis in (0, 1):
        k = int(steps[axis])
        start, stop = max(0, k), min(first[1][axis], k + second[1][axis])
        if start >= stop:
            raise ValueError("the two grids do not overlap")
        origin.append(first[0][axis] if start == 0 else second[0][axis])
        spans.append((slice(start, stop), slice(start - k, stop - k)))
    (first_x, second_x), (first_y, second_y) = spans
    return (origin[0], origin[1]), (first_y, first_x), (second_y, second_x)
