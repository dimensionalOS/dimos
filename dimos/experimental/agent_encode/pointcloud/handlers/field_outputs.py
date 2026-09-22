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

"""Bounded views of shared fields; output size never changes measurement resolution."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import numpy as np
from PIL import Image
from pydantic import JsonValue

from dimos.experimental.agent_encode.pointcloud.fields import (
    Channel,
    Distances,
    FieldData,
    FieldNode,
    Grid,
    HeightField,
    Labels,
    Mask,
    Percentile,
)
from dimos.experimental.agent_encode.pointcloud.handlers.lib.reference import reference
from dimos.experimental.agent_encode.pointcloud.handlers.lib.surface import (
    Contributors,
    GridSurface,
    Pickable,
)
from dimos.experimental.agent_encode.pointcloud.render import raster as render
from dimos.experimental.agent_encode.pointcloud.render.overlays import (
    DrawnOverlay,
    Overlay,
    draw_overlays,
    grid_pixel,
)
from dimos.experimental.agent_encode.pointcloud.runtime.context import (
    EncodeContext,
    Request,
    Result,
)


def numbers(values: np.ndarray | float, decimals: int | None = None) -> JsonValue:
    """JSON-compatible numbers with missing values represented by null.

    Whole-number arrays (counts, masks, labels) serialize as integers; others are
    rounded to ``decimals`` when given."""
    array = np.asarray(values, dtype=float)
    finite = np.isfinite(array)
    kept = array[finite]
    if np.array_equal(kept, np.round(kept)):
        kept = kept.astype(np.int64)
    elif decimals is not None:
        kept = np.round(kept, decimals)
    out = np.full(array.shape, None, dtype=object)
    out[finite] = kept.tolist()
    listed: JsonValue = out.tolist()
    return listed


def output_decimals(decimals: int | None, grid: Grid) -> int:
    """Caller's choice, else a thousandth of the cell size (0.1 m cells -> mm)."""
    if decimals is None:
        return max(0, int(np.ceil(-np.log10(grid.cell_m))) + 2)
    if type(decimals) is not int or not 0 <= decimals <= 9:
        raise ValueError("decimals must be an integer from 0 to 9")
    return decimals


def _channels(data: FieldData, fields: tuple[str, ...] | None) -> tuple[str, ...]:
    chosen = fields if fields is not None else tuple(data.values)
    if any(name not in data.values for name in chosen):
        raise ValueError(f"unknown field channel; channels are {list(data.values)}")
    return chosen


def _no_targets(data: FieldData) -> bool:
    return isinstance(data, Distances) and data.target_count == 0


def _cell_points(source: FieldNode, ctx: EncodeContext) -> np.ndarray | None:
    """The returns behind each cell of ``source``; None for derived values, which may
    depend on remote cells or returns."""
    if isinstance(source, HeightField):
        return ctx.select(source.source).points
    if isinstance(source, (Channel, Percentile)):
        return _cell_points(source.source, ctx)
    return None


@dataclass(frozen=True)
class PointSample:
    requested: tuple[float, float]
    status: Literal["ok", "outside_grid", "no_returns", "no_targets"]
    cell: tuple[int, int] | None
    """(column, row)."""
    centre: tuple[float, float] | None
    values: dict[str, JsonValue] | None


@dataclass(frozen=True)
class ValueRange:
    min: JsonValue
    max: JsonValue
    distinct: JsonValue
    """The distinct values of a mask or label field, at most 64; None for others."""


@dataclass(frozen=True)
class DiscSample:
    requested: tuple[float, float]
    status: Literal["ok", "outside_grid", "no_targets"]
    cells: int
    values: dict[str, ValueRange] | None


@dataclass(frozen=True)
class SampleResult(Result):
    grid: Grid
    decimals: int
    samples: list[PointSample] | list[DiscSample]


@dataclass(frozen=True)
class Sample(Request[SampleResult]):
    """Sample containing cells without interpolation."""

    source: FieldNode
    at: tuple[tuple[float, float], ...]
    """Coordinate pairs on the grid's plane."""
    fields: tuple[str, ...] | None = None
    decimals: int | None = None
    radius: float = 0.0
    """Above 0, summarises every cell whose centre lies within it of the point:
    min/max per channel, plus the distinct values for masks and component labels."""

    def run(self, ctx: EncodeContext) -> SampleResult:
        data = ctx.evaluate(self.source)
        xy = np.asarray(self.at, dtype=float)
        if xy.ndim != 2 or xy.shape[1] != 2 or not np.isfinite(xy).all():
            raise ValueError("at must be a list of finite coordinate pairs")
        if len(xy) > 4096:
            raise ValueError("sample exceeds 4096 locations; split the request")
        fields = _channels(data, self.fields)
        if not np.isfinite(self.radius) or self.radius < 0:
            raise ValueError("radius must be finite and non-negative")
        places = output_decimals(self.decimals, data.grid)
        if self.radius > 0:
            discs = [self._disc(data, point, fields, places) for point in xy]
            return SampleResult(data.grid, places, discs)
        ij, valid = data.grid.indices(xy)
        samples = []
        for point, cell, inside in zip(xy, ij, valid, strict=True):
            requested = (float(point[0]), float(point[1]))
            if not inside:
                samples.append(PointSample(requested, "outside_grid", None, None, None))
                continue
            col, row = int(cell[0]), int(cell[1])
            values = {name: numbers(data.values[name][row, col], places) for name in fields}
            status: Literal["ok", "no_returns", "no_targets"] = "ok"
            if _no_targets(data):
                status = "no_targets"
            elif all(v is None for v in values.values()):
                status = "no_returns"
            centre = np.round(
                np.asarray(data.grid.origin) + (cell + 0.5) * data.grid.cell_m, places
            )
            samples.append(
                PointSample(
                    requested, status, (col, row), (float(centre[0]), float(centre[1])), values
                )
            )
        return SampleResult(data.grid, places, samples)

    def _disc(
        self, data: FieldData, point: np.ndarray, fields: tuple[str, ...], places: int
    ) -> DiscSample:
        near = np.linalg.norm(data.grid.centres() - point, axis=-1) <= self.radius
        (cell,), (inside,) = data.grid.indices(point[None])
        if inside:
            near[cell[1], cell[0]] = True  # the containing cell, even when radius < cell_m
        requested = (float(point[0]), float(point[1]))
        if not near.any():
            return DiscSample(requested, "outside_grid", 0, None)
        values = {}
        for name in fields:
            cells = data.values[name][near].astype(float)
            cells = cells[np.isfinite(cells)]
            if not len(cells):
                values[name] = ValueRange(None, None, None)
                continue
            distinct = numbers(np.unique(cells)[:64]) if isinstance(data, (Mask, Labels)) else None
            values[name] = ValueRange(
                numbers(cells.min(), places), numbers(cells.max(), places), distinct
            )
        status: Literal["ok", "no_targets"] = "no_targets" if _no_targets(data) else "ok"
        return DiscSample(requested, status, int(near.sum()), values)


@dataclass(frozen=True)
class WindowResult(Result):
    grid: Grid
    decimals: int
    values: dict[str, JsonValue]
    """``values[channel][row][column]`` over the window."""


@dataclass(frozen=True)
class Window(Request[WindowResult]):
    """Exact cell slice, preserving the parent grid."""

    source: FieldNode
    cells: tuple[int, int, int, int]
    """(column, row, width, height)."""
    fields: tuple[str, ...] | None = None
    decimals: int | None = None

    def run(self, ctx: EncodeContext) -> WindowResult:
        data = ctx.evaluate(self.source)
        if len(self.cells) != 4 or any(type(v) is not int for v in self.cells):
            raise ValueError("cells must contain four integers")
        col, row, width, height = self.cells
        if (
            min(col, row) < 0
            or min(width, height) <= 0
            or col + width > data.grid.shape[0]
            or row + height > data.grid.shape[1]
        ):
            raise ValueError("window must lie inside the grid")
        places = output_decimals(self.decimals, data.grid)
        return WindowResult(
            data.grid,
            places,
            {
                name: numbers(data.values[name][row : row + height, col : col + width], places)
                for name in _channels(data, self.fields)
            },
        )


@dataclass(frozen=True)
class MapResult(Result):
    grid: Grid
    value_range: tuple[float, float]
    colour: render.ColourScale
    image: Path
    image_size: tuple[int, int]
    pixels_per_cell: int
    view_ref: dict[str, JsonValue]
    overlays: list[DrawnOverlay]


@dataclass(frozen=True)
class Map(Request[MapResult], Pickable):
    """Render a scalar field, mask, or labels without modifying the field grid.

    Pixel origin is top-left; increasing the grid's second axis goes up.
    """

    source: FieldNode
    value_range: tuple[float, float] | None = None
    overlays: tuple[Overlay, ...] = ()
    max_side: int = 1024
    """Longest image side in pixels. Images use nearest-neighbour enlargement only;
    oversized grids are rejected."""

    def pixels_per_cell(self, grid: Grid) -> int:
        if (
            type(self.max_side) is not int
            or not 1 <= self.max_side <= 2048
            or max(grid.shape) > self.max_side
        ):
            raise ValueError("map grid exceeds max_side (1..2048); request a smaller grid region")
        return max(1, self.max_side // max(grid.shape))

    def surface(self, ctx: EncodeContext) -> GridSurface:
        data = ctx.evaluate(self.source)
        values = data.scalar()
        points = _cell_points(self.source, ctx)
        contributors = None
        if points is not None:
            indices, _ = data.grid.indices(points.astype(np.float64)[:, data.grid.axes])
            contributors = Contributors.on(data.grid, points, indices)

        def cell_value(col: int, row: int) -> float | None:
            value = values[row, col]
            return float(value) if np.isfinite(value) else None

        return GridSurface(data.grid, self.pixels_per_cell(data.grid), contributors, cell_value)

    def run(self, ctx: EncodeContext) -> MapResult:
        data = ctx.evaluate(self.source)
        values = data.scalar()
        nx, ny = data.grid.shape
        scale = self.pixels_per_cell(data.grid)
        finite = np.isfinite(values)
        low, high = (
            self.value_range
            if self.value_range is not None
            else (
                (float(values[finite].min()), float(values[finite].max()))
                if finite.any()
                else (0.0, 1.0)
            )
        )
        if not np.isfinite([low, high]).all() or high < low:
            raise ValueError("value_range must be finite and ordered")
        denominator = high - low if high != low else 1.0
        fractions = np.where(finite, np.clip((values - low) / denominator, 0, 1), 0)
        _, table = render.colour_table()
        rgb = table[(fractions * 255).astype(int)].copy()
        rgb[~finite] = (96, 96, 96)
        picture = Image.fromarray(rgb[::-1].astype(np.uint8)).resize(
            (nx * scale, ny * scale), Image.Resampling.NEAREST
        )
        view_ref = reference(self, ctx)

        def project(point: np.ndarray) -> tuple[float, float]:
            return grid_pixel(point, data.grid.axes, data.grid.origin, data.grid.cell_m, ny, scale)

        with ctx.artifact("field.png") as (staging, path):
            picture.save(staging)
            overlays = draw_overlays(staging, self.overlays, ctx, project, view_ref=view_ref)
        return MapResult(
            data.grid,
            (low, high),
            render.colour_scale(low, high),
            path,
            (nx * scale, ny * scale),
            scale,
            view_ref,
            overlays,
        )
