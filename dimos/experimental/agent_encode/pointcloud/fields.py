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

"""Lazy geometric fields. Arrays stay inside the encoder until an output requests them."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Protocol

import numpy as np
from numpy.typing import NDArray
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

from dimos.experimental.agent_encode.pointcloud import constants
from dimos.experimental.agent_encode.pointcloud.runtime.context import (
    EncodeContext,
    Request,
    Result,
    Selection,
)


class Region3D(Protocol):
    """Anything that tests points for membership: a shape or a Band."""

    def contains(self, points: np.ndarray) -> np.ndarray: ...


@dataclass(frozen=True)
class Grid:
    """Fixed half-open cells, indexed [row, column].

    Resolution is never changed to satisfy an output budget.
    """

    origin: tuple[float, float]
    shape: tuple[int, int]
    """(columns, rows)."""
    cell_m: float
    plane: str = "xy"
    """Names two cloud axes, not gravity."""
    frame: str | None = None
    """None inherits the cloud frame."""

    def __post_init__(self) -> None:
        object.__setattr__(self, "origin", tuple(self.origin))
        object.__setattr__(self, "shape", tuple(self.shape))
        if self.plane not in ("xy", "xz", "yz"):
            raise ValueError("plane must be xy, xz or yz")
        if len(self.origin) != 2 or not np.isfinite(self.origin).all():
            raise ValueError("origin must contain two finite coordinates")
        if not np.isfinite(self.cell_m) or self.cell_m <= 0:
            raise ValueError("cell_m must be positive and finite")
        if len(self.shape) != 2 or any(type(n) is not int or n <= 0 for n in self.shape):
            raise ValueError("shape must contain two positive integers")
        if self.shape[0] * self.shape[1] > constants.MAX_GRID_CELLS:
            raise ValueError(
                f"grid exceeds {constants.MAX_GRID_CELLS} cells; request a smaller region"
            )

    @property
    def axes(self) -> tuple[int, int]:
        return "xyz".index(self.plane[0]), "xyz".index(self.plane[1])

    @property
    def normal(self) -> int:
        return next(i for i in range(3) if i not in self.axes)

    def check(self, ctx: EncodeContext) -> None:
        if self.frame is not None and self.frame != ctx.cloud.frame_id:
            raise ValueError("grid frame differs from cloud frame; transform explicitly")

    def centres(self) -> np.ndarray:
        row, col = np.indices((self.shape[1], self.shape[0]))
        centres: np.ndarray = (
            np.stack((col, row), axis=-1) * self.cell_m + np.asarray(self.origin) + self.cell_m / 2
        )
        return centres

    def indices(self, xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        ij = np.floor((xy - np.asarray(self.origin)) / self.cell_m).astype(np.int64)
        valid = ((ij >= 0) & (ij < np.asarray(self.shape))).all(axis=-1)
        return ij, valid


@dataclass(frozen=True)
class Select(Selection):
    """A lazy selection of returns: the include shapes, less the exclude shapes."""

    include: Region3D | tuple[Region3D, ...] = ()
    """Shapes or bands to intersect; none means all returns."""
    exclude: Region3D | tuple[Region3D, ...] = ()
    """Shapes or bands whose returns are then removed."""
    source: Selection | None = None

    def run(self, ctx: EncodeContext) -> np.ndarray:
        ctx = ctx.select(self.source)
        keep = np.ones(len(ctx.points), dtype=bool)
        for regions, invert in ((self.include, False), (self.exclude, True)):
            for region in regions if isinstance(regions, tuple) else (regions,):
                inside = region.contains(ctx.points)
                keep &= ~inside if invert else inside
        selected: np.ndarray = ctx.points[keep]
        return selected


@dataclass(frozen=True)
class Band:
    """A coordinate interval."""

    axis: str
    low: float | None = None
    high: float | None = None
    closed: tuple[bool, bool] = (True, False)
    """Whether the low and the high endpoint are included, each stated explicitly."""

    def __post_init__(self) -> None:
        if self.axis not in ("x", "y", "z") or len(self.closed) != 2:
            raise ValueError("Band needs an x/y/z axis and two endpoint flags")
        if any(v is not None and not np.isfinite(v) for v in (self.low, self.high)):
            raise ValueError("use None for an unbounded endpoint")
        if self.low is not None and self.high is not None and self.low > self.high:
            raise ValueError("Band low must not exceed high")

    def contains(self, points: np.ndarray) -> np.ndarray:
        values = points[:, "xyz".index(self.axis)]
        keep = np.ones(len(points), dtype=bool)
        if self.low is not None:
            keep &= values >= self.low if self.closed[0] else values > self.low
        if self.high is not None:
            keep &= values <= self.high if self.closed[1] else values < self.high
        return keep


@dataclass(frozen=True)
class FieldSummary(Result):
    """A computed field as a named output reports it; the cells stay internal."""

    grid: Grid
    channels: list[str]


@dataclass(frozen=True)
class FieldData(Result):
    """Internal computed fields."""

    grid: Grid
    values: dict[str, np.ndarray]
    """Non-finite numeric values mean missing evidence."""

    def scalar(self) -> np.ndarray:
        if len(self.values) != 1:
            raise ValueError("choose a field channel, for example height.min")
        return next(iter(self.values.values()))

    def summary(self) -> FieldSummary:
        return FieldSummary(self.grid, list(self.values))


@dataclass(frozen=True)
class Mask(FieldData):
    """Cells of 1, 0, or NaN for no data."""


@dataclass(frozen=True)
class Region:
    id: int
    cells: int
    centroid: tuple[float, float]
    bounds: tuple[tuple[float, float], tuple[float, float]]


@dataclass(frozen=True)
class MeasuredRegion(Region):
    """A region with equal-weight statistics of the ``values`` field over its finite
    cells; all None when it has none."""

    valid_cells: int
    min: float | None
    p10: float | None
    p50: float | None
    p90: float | None
    max: float | None


@dataclass(frozen=True)
class LabelsSummary(FieldSummary):
    region_count: int
    regions: list[Region]
    omitted_regions: int
    omitted_cells: int


@dataclass(frozen=True)
class Labels(FieldData):
    """Connected-component labels, 0 outside every region."""

    region_count: int
    regions: list[Region]
    """The listed regions, largest first when ``max_regions`` limits them."""
    omitted_regions: int
    omitted_cells: int

    def summary(self) -> LabelsSummary:
        return LabelsSummary(
            self.grid,
            list(self.values),
            self.region_count,
            self.regions,
            self.omitted_regions,
            self.omitted_cells,
        )


@dataclass(frozen=True)
class DistancesSummary(FieldSummary):
    target_count: int


@dataclass(frozen=True)
class Distances(FieldData):
    target_count: int
    """With none, every distance is missing: no targets is not free space."""

    def summary(self) -> DistancesSummary:
        return DistancesSummary(self.grid, list(self.values), self.target_count)


class FieldNode(Request[FieldData]):
    """A lazy field supporting arithmetic and three-state mask composition. Named
    directly, it reports its grid and channels; ``Components`` adds its region table."""

    def __sub__(self, other: FieldNode | float) -> Difference:
        return Difference(self, other)

    def __and__(self, other: FieldNode) -> And:
        return And(self, other)

    def __or__(self, other: FieldNode) -> Or:
        return Or(self, other)

    def __invert__(self) -> Not:
        return Not(self)


@dataclass(frozen=True)
class HeightField(FieldNode):
    """Count and min/max remaining-axis coordinates in each fixed grid cell.

    Lowest returns are not necessarily floor. Missing cells have count 0 and
    null extrema. No interpolation, floor fitting, or support threshold is applied.
    """

    grid: Grid
    source: Selection | None = None

    @property
    def min(self) -> Channel:
        return Channel(self, "min")

    @property
    def max(self) -> Channel:
        return Channel(self, "max")

    @property
    def count(self) -> Channel:
        return Channel(self, "count")

    def percentile(self, q: float, min_count: int = 4) -> Percentile:
        """Remaining-axis percentile per cell; insufficient support stays missing."""
        return Percentile(self, q, min_count)

    def run(self, ctx: EncodeContext) -> FieldData:
        self.grid.check(ctx)
        points = ctx.select(self.source).points.astype(np.float64)
        ij, inside = self.grid.indices(points[:, self.grid.axes])
        ij, points = ij[inside], points[inside]
        nx, ny = self.grid.shape
        index = ij[:, 1] * nx + ij[:, 0]
        count = np.bincount(index, minlength=nx * ny).reshape(ny, nx)
        low, high = np.full(nx * ny, np.inf), np.full(nx * ny, -np.inf)
        np.minimum.at(low, index, points[:, self.grid.normal])
        np.maximum.at(high, index, points[:, self.grid.normal])
        low[~np.isfinite(low)] = np.nan
        high[~np.isfinite(high)] = np.nan
        return FieldData(
            self.grid,
            {"count": count, "min": low.reshape(ny, nx), "max": high.reshape(ny, nx)},
        )


@dataclass(frozen=True)
class Percentile(FieldNode):
    """Per-cell return percentile using linear interpolation between sorted values.

    Like HeightField extrema, this measures a cloud axis, not an inferred floor.
    """

    source: HeightField
    q: float
    """In [0, 100]."""
    min_count: int
    """Cells with fewer selected returns are missing."""

    def run(self, ctx: EncodeContext) -> FieldData:
        if not np.isfinite(self.q) or not 0 <= self.q <= 100:
            raise ValueError("q must be finite and between 0 and 100")
        if type(self.min_count) is not int or self.min_count <= 0:
            raise ValueError("min_count must be a positive integer")
        height = ctx.evaluate(self.source)
        grid = height.grid
        count = height.values["count"].ravel()
        # Percentiles share sorting; extrema and counts never require it.
        ordered = ctx.evaluate(_SortedByCell(self.source))
        supported = count >= self.min_count
        starts = np.cumsum(count) - count
        position = (count[supported] - 1) * (self.q / 100)
        lower_index = np.floor(position).astype(np.int64)
        upper_index = np.ceil(position).astype(np.int64)
        lower = ordered[starts[supported] + lower_index]
        upper = ordered[starts[supported] + upper_index]
        values = np.full(count.shape, np.nan)
        values[supported] = lower + (upper - lower) * (position - lower_index)
        return FieldData(
            grid,
            {"percentile": values.reshape(grid.shape[1], grid.shape[0])},
        )


@dataclass(frozen=True)
class _SortedByCell:
    """A height field's selected coordinates, sorted by cell and then by value."""

    source: HeightField

    def run(self, ctx: EncodeContext) -> np.ndarray:
        grid = self.source.grid
        points = ctx.select(self.source.source).points.astype(np.float64)
        ij, inside = grid.indices(points[:, grid.axes])
        index = ij[inside, 1] * grid.shape[0] + ij[inside, 0]
        values = points[inside, grid.normal]
        ordered: np.ndarray = values[np.lexsort((values, index))]
        return ordered


@dataclass(frozen=True)
class Channel(FieldNode):
    source: FieldNode
    name: str

    def run(self, ctx: EncodeContext) -> FieldData:
        value = ctx.evaluate(self.source)
        return replace(value, values={self.name: value.values[self.name]})


@dataclass(frozen=True)
class DistanceField(FieldNode):
    """Cell-centre distances to selected projected returns or true mask-cell centres.

    Empty target sets produce null distances, never free space.
    """

    grid: Grid
    source: FieldNode | Selection | None = None
    """The targets. Those outside the grid still participate when this is a selection."""

    def run(self, ctx: EncodeContext) -> Distances:
        self.grid.check(ctx)
        if isinstance(self.source, FieldNode):
            mask = ctx.evaluate(self.source)
            if not isinstance(mask, Mask) or mask.grid != self.grid:
                raise ValueError("distance mask must be on the same grid")
            targets = self.grid.centres()[mask.scalar() == 1]
        else:
            targets = ctx.select(self.source).points[:, self.grid.axes]
        nx, ny = self.grid.shape
        distance = np.full((ny, nx), np.nan)
        if len(targets):
            values, _ = cKDTree(targets).query(self.grid.centres().reshape(-1, 2), workers=1)
            distance = values.reshape(ny, nx)
        return Distances(self.grid, {"distance_m": distance}, len(targets))


def _same_grid(left: FieldData, right: FieldData) -> None:
    if right.grid != left.grid:
        raise ValueError(
            "field grids differ; put one on the other's grid with Resample(field, grid)"
        )


def _masks(ctx: EncodeContext, left: FieldNode, right: FieldNode) -> tuple[Mask, Mask]:
    a, b = ctx.evaluate(left), ctx.evaluate(right)
    if not isinstance(a, Mask) or not isinstance(b, Mask):
        raise ValueError("& and | combine two masks")
    _same_grid(a, b)
    return a, b


@dataclass(frozen=True)
class Difference(FieldNode):
    """``left - right``: a field less another field or a number."""

    left: FieldNode
    right: FieldNode | float

    def run(self, ctx: EncodeContext) -> FieldData:
        left = ctx.evaluate(self.left)
        if isinstance(self.right, FieldNode):
            right = ctx.evaluate(self.right)
            _same_grid(left, right)
            return FieldData(left.grid, {"value": left.scalar() - right.scalar()})
        return FieldData(left.grid, {"value": left.scalar() - float(self.right)})


@dataclass(frozen=True)
class And(FieldNode):
    """``left & right``: 0 where either is 0, else missing where either is, else 1."""

    left: FieldNode
    right: FieldNode

    def run(self, ctx: EncodeContext) -> Mask:
        left, right = _masks(ctx, self.left, self.right)
        a, b = left.scalar(), right.scalar()
        missing = ~np.isfinite(a) | ~np.isfinite(b)
        out = np.where((a == 0) | (b == 0), 0, np.where(missing, np.nan, 1))
        return Mask(left.grid, {"value": out})


@dataclass(frozen=True)
class Or(FieldNode):
    """``left | right``: 1 where either is 1, else missing where either is, else 0."""

    left: FieldNode
    right: FieldNode

    def run(self, ctx: EncodeContext) -> Mask:
        left, right = _masks(ctx, self.left, self.right)
        a, b = left.scalar(), right.scalar()
        missing = ~np.isfinite(a) | ~np.isfinite(b)
        out = np.where((a == 1) | (b == 1), 1, np.where(missing, np.nan, 0))
        return Mask(left.grid, {"value": out})


@dataclass(frozen=True)
class Not(FieldNode):
    """``~mask``: 1 and 0 swap; missing stays missing."""

    source: FieldNode

    def run(self, ctx: EncodeContext) -> Mask:
        mask = ctx.evaluate(self.source)
        if not isinstance(mask, Mask):
            raise ValueError("~ requires a mask")
        a = mask.scalar()
        return Mask(mask.grid, {"value": np.where(np.isfinite(a), 1 - a, np.nan)})


@dataclass(frozen=True)
class Resample(FieldNode):
    """Put a field on another grid: each target cell takes the value of the source
    cell containing its centre. No interpolation; centres outside the source grid
    are null. Masks stay masks, so fields from different grids can be combined; labels
    keep their source's region table."""

    source: FieldNode
    grid: Grid

    def run(self, ctx: EncodeContext) -> FieldData:
        self.grid.check(ctx)
        data = ctx.evaluate(self.source)
        if data.grid.plane != self.grid.plane:
            raise ValueError("Resample needs grids on the same plane")
        ij, inside = data.grid.indices(self.grid.centres())
        values = {}
        for name, array in data.values.items():
            out = np.full(inside.shape, np.nan)
            out[inside] = array[ij[inside][:, 1], ij[inside][:, 0]]
            values[name] = out
        return replace(data, grid=self.grid, values=values)


@dataclass(frozen=True)
class Threshold(FieldNode):
    source: FieldNode
    comparison: str
    value: float

    def run(self, ctx: EncodeContext) -> Mask:
        functions = {
            ">": np.greater,
            ">=": np.greater_equal,
            "<": np.less,
            "<=": np.less_equal,
            "==": np.equal,
            "!=": np.not_equal,
        }
        if self.comparison not in functions or not np.isfinite(self.value):
            raise ValueError("use >, >=, <, <=, == or != and a finite threshold")
        data = ctx.evaluate(self.source)
        values = data.scalar()
        mask = np.where(np.isfinite(values), functions[self.comparison](values, self.value), np.nan)
        return Mask(data.grid, {"mask": mask})


@dataclass(frozen=True)
class Components(FieldNode):
    """Group true cells without adding evidence in gaps between them.

    Labels and region geometry include only the original true cells.
    """

    source: FieldNode
    connectivity: int = 8
    gap_cells: int = 0
    """Extends the linking radius to ``gap_cells + 1`` cells: Chebyshev distance for
    connectivity 8, Manhattan distance for connectivity 4. It links true cells across
    any cell between them, measured or not."""
    values: FieldNode | None = None
    """Adds each region's statistics of another field on the same grid: finite cells
    weighted equally, quantiles by linear interpolation."""
    max_regions: int | None = None
    """Keeps the largest regions in the table (cells, then id) and counts the rest;
    labels always cover every region."""

    def run(self, ctx: EncodeContext) -> Labels:
        if self.connectivity not in (4, 8):
            raise ValueError("connectivity must be 4 or 8")
        if type(self.gap_cells) is not int or not 0 <= self.gap_cells <= 4:
            raise ValueError("gap_cells must be an integer from 0 to 4")
        if self.max_regions is not None and (
            type(self.max_regions) is not int or self.max_regions < 1
        ):
            raise ValueError("max_regions must be a positive integer or None")
        data = ctx.evaluate(self.source)
        if not isinstance(data, Mask):
            raise ValueError("Components requires a mask")
        values = data.scalar()
        measured = None
        if self.values is not None:
            field = ctx.evaluate(self.values)
            _same_grid(data, field)
            measured = field.scalar().astype(np.float64)
        labels, count = ndimage.label(
            values == 1, ndimage.generate_binary_structure(2, 1 if self.connectivity == 4 else 2)
        )
        if self.gap_cells and count > 1:
            labels, count = self._link_gaps(labels, count)
        sizes = np.bincount(labels.ravel(), minlength=count + 1)[1:]
        order = np.arange(count)
        if self.max_regions is not None:
            order = np.lexsort((order, -sizes))[: self.max_regions]
        regions: list[Region] = []
        centres = data.grid.centres()
        found = ndimage.find_objects(labels)
        half = data.grid.cell_m / 2
        for index in order:
            label, slices = int(index) + 1, found[index]
            mask = labels[slices] == label
            xy = centres[slices][mask]
            low, high = xy.min(0) - half, xy.max(0) + half
            geometry = Region(
                label,
                int(mask.sum()),
                (float(xy[:, 0].mean()), float(xy[:, 1].mean())),
                ((float(low[0]), float(low[1])), (float(high[0]), float(high[1]))),
            )
            if measured is not None:
                geometry = _measured(geometry, measured[slices][mask])
            regions.append(geometry)
        result = labels.astype(float)
        result[~np.isfinite(values)] = np.nan
        return Labels(
            data.grid,
            {"label": result},
            region_count=int(count),
            regions=regions,
            omitted_regions=int(count - len(order)),
            omitted_cells=int(sizes.sum() - sizes[order].sum()),
        )

    def _link_gaps(self, labels: NDArray[np.int32], count: int) -> tuple[NDArray[np.int32], int]:
        radius = self.gap_cells + 1
        rows, columns = labels.shape
        edges = []
        for dy in range(min(radius + 1, rows)):
            for dx in range(-min(radius, columns - 1), min(radius, columns - 1) + 1):
                if (dy == 0 and dx <= 0) or (self.connectivity == 4 and abs(dx) + dy > radius):
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


def _measured(region: Region, values: NDArray[np.float64]) -> MeasuredRegion:
    finite = values[np.isfinite(values)]
    if not len(finite):
        return MeasuredRegion(
            **vars(region), valid_cells=0, min=None, p10=None, p50=None, p90=None, max=None
        )
    low, p10, p50, p90, high = np.quantile(finite, (0, 0.1, 0.5, 0.9, 1), method="linear").tolist()
    return MeasuredRegion(
        **vars(region), valid_cells=len(finite), min=low, p10=p10, p50=p50, p90=p90, max=high
    )
