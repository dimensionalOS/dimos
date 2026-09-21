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

from dataclasses import dataclass, field
from typing import Any

import numpy as np
from numpy.typing import NDArray
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree

from dimos.experimental.agent_encode.pointcloud import constants
from dimos.experimental.agent_encode.pointcloud.runtime.context import EncodeContext


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

    def describe(self, ctx: EncodeContext) -> dict[str, Any]:
        if self.frame is not None and self.frame != ctx.cloud.frame_id:
            raise ValueError("grid frame differs from cloud frame; transform explicitly")
        return {
            "frame": ctx.cloud.frame_id,
            "plane": self.plane,
            "origin": list(self.origin),
            "shape": list(self.shape),
            "cell_m": self.cell_m,
            "indexing": "[row,column]",
        }

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
class Select:
    """A lazy selection of returns: the include shapes, less the exclude shapes."""

    include: Any = ()
    """Shapes to intersect; none means all returns."""
    exclude: Any = ()
    """Shapes whose returns are then removed."""
    source: Any = None

    def run(self, ctx: EncodeContext) -> np.ndarray:
        ctx = ctx.select(self.source)
        keep = np.ones(len(ctx.points), dtype=bool)
        for shapes, invert in ((self.include, False), (self.exclude, True)):
            sequence = (shapes,) if hasattr(shapes, "contains") else shapes
            for shape in sequence:
                inside = shape.contains(ctx.points)
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


@dataclass
class FieldData:
    """Internal computed fields."""

    grid: Grid
    values: dict[str, np.ndarray]
    """Non-finite numeric values mean missing evidence."""
    metadata: dict[str, Any] = field(default_factory=dict)
    kind: str = "field"

    def scalar(self) -> np.ndarray:
        if len(self.values) != 1:
            raise ValueError("choose a field channel, for example height.min")
        return next(iter(self.values.values()))


class FieldNode:
    """A lazy field supporting arithmetic and three-state mask composition."""

    def run(self, ctx: EncodeContext) -> FieldData:
        raise NotImplementedError

    def __sub__(self, other: Any) -> Binary:
        return Binary(self, other, "sub")

    def __and__(self, other: Any) -> Binary:
        return Binary(self, other, "and")

    def __or__(self, other: Any) -> Binary:
        return Binary(self, other, "or")

    def __invert__(self) -> Binary:
        return Binary(self, None, "not")


@dataclass(frozen=True)
class HeightField(FieldNode):
    """Count and min/max remaining-axis coordinates in each fixed grid cell.

    Lowest returns are not necessarily floor. Missing cells have count 0 and
    null extrema. No interpolation, floor fitting, or support threshold is applied.
    """

    grid: Grid
    source: Any = None

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
        self.grid.describe(ctx)
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
            {
                "coordinate": "xyz"[self.grid.normal],
                "units": {"count": "returns", "min": "m", "max": "m"},
            },
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

    def __post_init__(self) -> None:
        if not np.isfinite(self.q) or not 0 <= self.q <= 100:
            raise ValueError("q must be finite and between 0 and 100")
        if type(self.min_count) is not int or self.min_count <= 0:
            raise ValueError("min_count must be a positive integer")

    def run(self, ctx: EncodeContext) -> FieldData:
        height = ctx.evaluate(self.source)
        grid = height.grid
        count = height.values["count"].ravel()
        # Percentiles share sorting; extrema and counts never require it.
        key = ("height_percentile_order", id(self.source), id(ctx.points))
        if key not in ctx.cache:
            points = ctx.select(self.source.source).points.astype(np.float64)
            ij, inside = grid.indices(points[:, grid.axes])
            index = ij[inside, 1] * grid.shape[0] + ij[inside, 0]
            values = points[inside, grid.normal]
            ctx.cache[key] = values[np.lexsort((values, index))]
        ordered = ctx.cache[key]
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
            {
                "coordinate": height.metadata["coordinate"],
                "units": {"percentile": "m"},
                "q": self.q,
                "min_count": self.min_count,
                "method": "linear",
            },
        )


@dataclass(frozen=True)
class Channel(FieldNode):
    source: FieldNode
    name: str

    def run(self, ctx: EncodeContext) -> FieldData:
        value = ctx.evaluate(self.source)
        return FieldData(
            value.grid, {self.name: value.values[self.name]}, value.metadata, value.kind
        )


@dataclass(frozen=True)
class DistanceField(FieldNode):
    """Cell-centre distances to selected projected returns or true mask-cell centres.

    Empty target sets produce null distances and status=no_targets, never free space.
    """

    grid: Grid
    source: Any = None
    """The targets. Those outside the grid still participate when this is a selection."""

    def run(self, ctx: EncodeContext) -> FieldData:
        self.grid.describe(ctx)
        metric = "projected_return_distance"
        if isinstance(self.source, FieldNode):
            mask = ctx.evaluate(self.source)
            if mask.kind != "mask" or mask.grid != self.grid:
                raise ValueError("distance mask must be on the same grid")
            targets = self.grid.centres()[mask.scalar() == 1]
            metric = "true_cell_centre_distance"
        else:
            targets = ctx.select(self.source).points[:, self.grid.axes]
        nx, ny = self.grid.shape
        distance = np.full((ny, nx), np.nan)
        if len(targets):
            values, _ = cKDTree(targets).query(self.grid.centres().reshape(-1, 2), workers=1)
            distance = values.reshape(ny, nx)
        return FieldData(
            self.grid,
            {"distance_m": distance},
            {
                "metric": metric,
                "units": {"distance_m": "m"},
                "status": "ok" if len(targets) else "no_targets",
            },
        )


@dataclass(frozen=True)
class Binary(FieldNode):
    left: FieldNode
    right: Any
    operation: str

    def run(self, ctx: EncodeContext) -> FieldData:
        left = ctx.evaluate(self.left)
        a = left.scalar()
        right = ctx.evaluate(self.right) if isinstance(self.right, FieldNode) else None
        if right is not None and right.grid != left.grid:
            raise ValueError(
                "field grids differ; put one on the other's grid with Resample(field, grid)"
            )
        b = right.scalar() if right is not None else float(self.right or 0)
        kind = "field"
        if self.operation == "not":
            if left.kind != "mask":
                raise ValueError("~ requires a mask")
            out = np.where(np.isfinite(a), 1 - a, np.nan)
            return FieldData(left.grid, {"value": out}, {"operation": "not"}, "mask")
        if self.operation == "sub":
            out = a - b
        else:
            if left.kind != "mask" or right is None or right.kind != "mask":
                raise ValueError("and/or require two masks")
            kind = "mask"
            missing = ~np.isfinite(a) | ~np.isfinite(b)
            if self.operation == "and":
                out = np.where((a == 0) | (b == 0), 0, np.where(missing, np.nan, 1))
            elif self.operation == "or":
                out = np.where((a == 1) | (b == 1), 1, np.where(missing, np.nan, 0))
            else:
                raise ValueError("unsupported binary operation")
        return FieldData(left.grid, {"value": out}, {"operation": self.operation}, kind)


@dataclass(frozen=True)
class Resample(FieldNode):
    """Put a field on another grid: each target cell takes the value of the source
    cell containing its centre. No interpolation; centres outside the source grid
    are null. Masks stay masks, so fields from different grids can be combined."""

    source: FieldNode
    grid: Grid

    def run(self, ctx: EncodeContext) -> FieldData:
        self.grid.describe(ctx)
        data = ctx.evaluate(self.source)
        if data.grid.plane != self.grid.plane:
            raise ValueError("Resample needs grids on the same plane")
        ij, inside = data.grid.indices(self.grid.centres())
        values = {}
        for name, array in data.values.items():
            out = np.full(inside.shape, np.nan)
            out[inside] = array[ij[inside][:, 1], ij[inside][:, 0]]
            values[name] = out
        return FieldData(
            self.grid,
            values,
            {
                **{k: v for k, v in data.metadata.items() if k != "regions"},
                "resampled_from": data.grid.describe(ctx),
            },
            data.kind,
        )


@dataclass(frozen=True)
class Threshold(FieldNode):
    source: FieldNode
    comparison: str
    value: float

    def run(self, ctx: EncodeContext) -> FieldData:
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
        return FieldData(
            data.grid, {"mask": mask}, {"comparison": self.comparison, "value": self.value}, "mask"
        )


def _statistics(values: NDArray[np.float64]) -> dict[str, Any]:
    """Equal-weight statistics of a region's finite cells; all null when it has none."""
    finite = values[np.isfinite(values)]
    names = ("min", "p10", "p50", "p90", "max")
    if not len(finite):
        return {"valid_cells": 0, **dict.fromkeys(names)}
    numbers = np.quantile(finite, (0, 0.1, 0.5, 0.9, 1), method="linear")
    return {"valid_cells": len(finite), **dict(zip(names, numbers.tolist(), strict=True))}


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

    def run(self, ctx: EncodeContext) -> FieldData:
        if self.connectivity not in (4, 8):
            raise ValueError("connectivity must be 4 or 8")
        if type(self.gap_cells) is not int or not 0 <= self.gap_cells <= 4:
            raise ValueError("gap_cells must be an integer from 0 to 4")
        if self.max_regions is not None and (
            type(self.max_regions) is not int or self.max_regions < 1
        ):
            raise ValueError("max_regions must be a positive integer or None")
        data = ctx.evaluate(self.source)
        if data.kind != "mask":
            raise ValueError("Components requires a mask")
        values = data.scalar()
        measured = None
        if self.values is not None:
            field = ctx.evaluate(self.values)
            if field.grid != data.grid:
                raise ValueError(
                    "field grids differ; put one on the other's grid with Resample(field, grid)"
                )
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
        regions = []
        centres = data.grid.centres()
        found = ndimage.find_objects(labels)
        for index in order:
            label, slices = int(index) + 1, found[index]
            mask = labels[slices] == label
            xy = centres[slices][mask]
            region = {
                "id": label,
                "cells": int(mask.sum()),
                "centroid": xy.mean(0).tolist(),
                "bounds": [
                    (xy.min(0) - data.grid.cell_m / 2).tolist(),
                    (xy.max(0) + data.grid.cell_m / 2).tolist(),
                ],
            }
            if measured is not None:
                region.update(_statistics(measured[slices][mask]))
            regions.append(region)
        omitted = {}
        if self.max_regions is not None:
            omitted = {
                "omitted_regions": int(count - len(order)),
                "omitted_cells": int(sizes.sum() - sizes[order].sum()),
            }
        result = labels.astype(float)
        result[~np.isfinite(values)] = np.nan
        return FieldData(
            data.grid,
            {"label": result},
            {
                "connectivity": self.connectivity,
                **({"gap_cells": self.gap_cells} if self.gap_cells else {}),
                "region_count": int(count),
                **omitted,
                "regions": regions,
            },
            "regions",
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
