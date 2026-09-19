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
from scipy import ndimage
from scipy.spatial import cKDTree

from dimos.experimental.agent_encode.pointcloud.runtime.context import EncodeContext


@dataclass(frozen=True)
class Grid:
    """Fixed half-open cells, indexed [row, column]; shape is (columns, rows).

    plane names two cloud axes, not gravity. None frame inherits the cloud frame.
    Resolution is never changed to satisfy an output budget.
    """

    origin: tuple[float, float]
    shape: tuple[int, int]
    cell_m: float
    plane: str = "xy"
    frame: str | None = None

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
        if self.shape[0] * self.shape[1] > 262144:
            raise ValueError("grid exceeds 262144 cells; request a smaller region")

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
        return (
            np.stack((col, row), axis=-1) * self.cell_m + np.asarray(self.origin) + self.cell_m / 2
        )

    def indices(self, xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        ij = np.floor((xy - np.asarray(self.origin)) / self.cell_m).astype(np.int64)
        valid = ((ij >= 0) & (ij < np.asarray(self.shape))).all(axis=-1)
        return ij, valid


@dataclass(frozen=True)
class Select:
    """Intersect include shapes, then remove exclude shapes; no includes means all returns."""

    include: Any = ()
    exclude: Any = ()
    source: Any = None

    def run(self, ctx: EncodeContext) -> np.ndarray:
        ctx = ctx.select(self.source)
        keep = np.ones(len(ctx.points), dtype=bool)
        for shapes, invert in ((self.include, False), (self.exclude, True)):
            sequence = (shapes,) if hasattr(shapes, "contains") else shapes
            for shape in sequence:
                inside = shape.contains(ctx.points)
                keep &= ~inside if invert else inside
        return ctx.points[keep]


@dataclass(frozen=True)
class Band:
    """A coordinate interval, with independently explicit endpoint inclusion."""

    axis: str
    low: float | None = None
    high: float | None = None
    closed: tuple[bool, bool] = (True, False)

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
    """Internal computed fields; non-finite numeric values mean missing evidence."""

    grid: Grid
    values: dict[str, np.ndarray]
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

    Targets outside the grid still participate when source is a selection.
    Empty target sets produce null distances and status=no_targets, never free space.
    """

    grid: Grid
    source: Any = None

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


@dataclass(frozen=True)
class Components(FieldNode):
    source: FieldNode
    connectivity: int = 8

    def run(self, ctx: EncodeContext) -> FieldData:
        if self.connectivity not in (4, 8):
            raise ValueError("connectivity must be 4 or 8")
        data = ctx.evaluate(self.source)
        if data.kind != "mask":
            raise ValueError("Components requires a mask")
        values = data.scalar()
        labels, count = ndimage.label(
            values == 1, ndimage.generate_binary_structure(2, 1 if self.connectivity == 4 else 2)
        )
        regions = []
        centres = data.grid.centres()
        for label, slices in enumerate(ndimage.find_objects(labels), 1):
            if slices is None:
                continue
            mask = labels[slices] == label
            xy = centres[slices][mask]
            regions.append(
                {
                    "id": label,
                    "cells": int(mask.sum()),
                    "centroid": xy.mean(0).tolist(),
                    "bounds": [
                        (xy.min(0) - data.grid.cell_m / 2).tolist(),
                        (xy.max(0) + data.grid.cell_m / 2).tolist(),
                    ],
                }
            )
        result = labels.astype(float)
        result[~np.isfinite(values)] = np.nan
        return FieldData(
            data.grid,
            {"label": result},
            {"connectivity": self.connectivity, "region_count": int(count), "regions": regions},
            "regions",
        )
