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

"""Connected groups of a mask's true cells."""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import math
from typing import TYPE_CHECKING, overload

import numpy as np
from numpy.typing import NDArray
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

from dimos.experimental.agent_encode.pointcloud.image.lib.canvas import Canvas, Drawable

if TYPE_CHECKING:
    # base.Grid builds Regions, so importing it at runtime would be circular
    from dimos.experimental.agent_encode.pointcloud.grid.base import Grid


@dataclass(frozen=True)
class Stats:
    """A measure grid over a region's cells with data, each cell weighted equally,
    quantiles by linear interpolation; all None when it has no such cell."""

    valid_cells: int
    min: float | None
    p10: float | None
    p50: float | None
    p90: float | None
    max: float | None


@dataclass(frozen=True)
class Region(Drawable):
    """One connected group of true cells; its geometry covers only those cells."""

    id: int
    """The region's value in ``Regions.labels``."""
    cell_count: int
    area_m2: float
    centroid: tuple[float, float]
    """Mean of the cell centres."""
    bounds: tuple[tuple[float, float], tuple[float, float]]
    """South-west and north-east corners of the cells."""
    stats: Stats | None
    """The ``measure`` grid over the region; None without one."""

    def draw(self, canvas: Canvas) -> None:
        (x0, y0), (x1, y1) = self.bounds
        z = canvas.z_extent[0]
        canvas.path(
            np.array(
                [(x0, y0, z), (x1, y0, z), (x1, y1, z), (x0, y1, z), (x0, y0, z)],
                dtype=np.float64,
            )
        )

    def __repr__(self) -> str:
        (x0, y0), (x1, y1) = self.bounds
        text = (
            f"Region {self.id}: {self.cell_count} cells, {self.area_m2:.3f} m2, "
            f"centroid ({self.centroid[0]:.3f}, {self.centroid[1]:.3f}), "
            f"x {x0:.3f}..{x1:.3f}, y {y0:.3f}..{y1:.3f}"
        )
        stats = self.stats
        if stats is None:
            return text
        if stats.valid_cells == 0:
            return f"{text}, no measured cells"
        quantiles = (stats.min, stats.p10, stats.p50, stats.p90, stats.max)
        return f"{text}, min/p10/p50/p90/max " + "/".join(
            f"{q:.3f}" for q in quantiles if q is not None
        )


@dataclass(frozen=True)
class Gap(Drawable):
    """The shortest distance between two regions and where it is."""

    distance_m: float
    """Edge to edge between their closest cells; 0 when the regions touch."""
    from_m: tuple[float, float]
    """x, y on the first region's edge."""
    to_m: tuple[float, float]
    """x, y on the second region's edge."""

    def draw(self, canvas: Canvas) -> None:
        z = canvas.z_extent[0]
        ends = np.array([(*self.from_m, z), (*self.to_m, z)], dtype=np.float64)
        canvas.path(ends)
        for end in ends:
            canvas.point(end, 3, filled=True)


@dataclass(frozen=True, eq=False)
class Regions(Sequence[Region]):
    """Every region of a mask, largest first (cell_count, then id)."""

    regions: tuple[Region, ...]
    labels: Grid
    """Region id per cell, 0 outside every region, NaN where the mask had no data."""

    @overload
    def __getitem__(self, index: int) -> Region: ...

    @overload
    def __getitem__(self, index: slice) -> tuple[Region, ...]: ...

    def __getitem__(self, index: int | slice) -> Region | tuple[Region, ...]:
        return self.regions[index]

    def __len__(self) -> int:
        return len(self.regions)

    def near(self, xy: tuple[float, float], radius: float = 0.0) -> list[Region]:
        """Regions with a cell whose centre lies within ``radius`` of the point, or that
        hold the cell containing it, largest first."""
        ids = self.labels.values[self.labels.within(xy, radius)]
        found = set(ids[np.isfinite(ids) & (ids > 0)].astype(np.int64).tolist())
        return [region for region in self.regions if region.id in found]

    def gap(self, first: Region, second: Region) -> Gap:
        """The shortest distance between two regions' cells, edge to edge."""
        if first.id == second.id:
            raise ValueError("gap needs two different regions")
        if not {first.id, second.id} <= {region.id for region in self.regions}:
            raise ValueError("both regions must come from this Regions")
        a, b = self._edge_centres(first.id), self._edge_centres(second.id)
        cell = self.labels.cell_m
        # An edge gap is at least the centre distance minus one cell diagonal, so the
        # closest pair lies within that much of the closest centres.
        closest, _ = cKDTree(b).query(a, workers=1)
        reach = float(closest.min()) + cell * math.sqrt(2)
        near = cKDTree(a).query_ball_tree(cKDTree(b), reach)
        i = np.repeat(np.arange(len(a)), [len(js) for js in near])
        j = np.concatenate([np.asarray(js, dtype=np.int64) for js in near])
        low, high = _facing(a[i], b[j], cell / 2)
        lengths = np.linalg.norm(high - low, axis=1)
        k = int(np.argmin(lengths))
        return Gap(
            round(float(lengths[k]), 3),
            (round(float(low[k, 0]), 3), round(float(low[k, 1]), 3)),
            (round(float(high[k, 0]), 3), round(float(high[k, 1]), 3)),
        )

    def _edge_centres(self, region_id: int) -> NDArray[np.float64]:
        """Centres of the region's cells that touch a cell outside it; the closest cell to
        anything outside is always one of these."""
        inside = self.labels.values == region_id
        interior = ndimage.binary_erosion(inside, np.ones((3, 3), dtype=bool), border_value=1)
        centres: NDArray[np.float64] = self.labels.centres()[inside & ~interior]
        return centres

    def __repr__(self) -> str:
        lines = [f"{len(self.regions)} regions"]
        lines += [repr(region) for region in self.regions[:10]]
        rest = self.regions[10:]
        if rest:
            lines.append(f"…and {len(rest)} more ({sum(r.cell_count for r in rest)} cells)")
        return "\n".join(lines)


def label(
    mask: NDArray[np.float64],
    centres: NDArray[np.float64],
    cell_m: float,
    connectivity: int,
    gap: int,
    measure: NDArray[np.float64] | None,
) -> tuple[NDArray[np.float64], tuple[Region, ...]]:
    """Region ids per cell (0 outside, NaN where the mask has no data) and every region,
    largest first. ``centres`` are the cells' world x, y, (rows, columns, 2)."""
    labels, count = ndimage.label(
        mask == 1, ndimage.generate_binary_structure(2, 1 if connectivity == 4 else 2)
    )
    if gap and count > 1:
        labels, count = _link_gaps(labels, count, gap, connectivity)
    sizes = np.bincount(labels.ravel(), minlength=count + 1)[1:]
    order = np.lexsort((np.arange(count), -sizes))
    found = ndimage.find_objects(labels)
    half = cell_m / 2
    regions = []
    for index in order:
        region_id, slices = int(index) + 1, found[index]
        inside = labels[slices] == region_id
        xy = centres[slices][inside]
        low, high = xy.min(0) - half, xy.max(0) + half
        regions.append(
            Region(
                region_id,
                int(inside.sum()),
                int(inside.sum()) * cell_m**2,
                (float(xy[:, 0].mean()), float(xy[:, 1].mean())),
                ((float(low[0]), float(low[1])), (float(high[0]), float(high[1]))),
                None if measure is None else _stats(measure[slices][inside]),
            )
        )
    result = labels.astype(np.float64)
    result[~np.isfinite(mask)] = np.nan
    return result, tuple(regions)


def _link_gaps(
    labels: NDArray[np.int32], count: int, gap: int, connectivity: int
) -> tuple[NDArray[np.int32], int]:
    radius = gap + 1
    rows, columns = labels.shape
    edges = []
    for dy in range(min(radius + 1, rows)):
        for dx in range(-min(radius, columns - 1), min(radius, columns - 1) + 1):
            if (dy == 0 and dx <= 0) or (connectivity == 4 and abs(dx) + dy > radius):
                continue
            first = labels[: rows - dy, max(-dx, 0) : min(columns, columns - dx)]
            second = labels[dy:, max(dx, 0) : min(columns, columns + dx)]
            linked = (first != 0) & (second != 0) & (first != second)
            if linked.any():
                edges.append(np.column_stack((first[linked] - 1, second[linked] - 1)))
    if not edges:
        return labels, count
    pairs = np.concatenate(edges)
    graph = coo_matrix(
        (np.ones(len(pairs), dtype=np.uint8), (pairs[:, 0], pairs[:, 1])),
        shape=(count, count),
    )
    group_count, groups = connected_components(graph, directed=False)
    # Canonical labels follow each group's first original cell in raster order.
    first_component = np.full(group_count, count)
    np.minimum.at(first_component, groups, np.arange(count))
    canonical = np.empty(group_count, dtype=np.int32)
    canonical[np.argsort(first_component)] = np.arange(1, group_count + 1)
    lookup = np.concatenate((np.zeros(1, dtype=np.int32), canonical[groups]))
    return lookup[labels], int(group_count)


def _stats(values: NDArray[np.float64]) -> Stats:
    finite = values[np.isfinite(values)]
    if not len(finite):
        return Stats(0, None, None, None, None, None)
    low, p10, p50, p90, high = np.quantile(finite, (0, 0.1, 0.5, 0.9, 1), method="linear").tolist()
    return Stats(len(finite), low, p10, p50, p90, high)


def _facing(
    a: NDArray[np.float64], b: NDArray[np.float64], half: float
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """The closest points of square cells centred at ``a`` and ``b``, pair by pair: along
    each axis, the facing edges, or the middle of the span both cells share."""
    a_low, a_high, b_low, b_high = a - half, a + half, b - half, b + half
    shared = (np.maximum(a_low, b_low) + np.minimum(a_high, b_high)) / 2
    on_a = np.where(a_high <= b_low, a_high, np.where(b_high <= a_low, a_low, shared))
    on_b = np.where(a_high <= b_low, b_low, np.where(b_high <= a_low, b_high, shared))
    return on_a, on_b
