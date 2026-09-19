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
from typing import Any

import numpy as np
from PIL import Image

from dimos.experimental.agent_encode.pointcloud.fields import FieldData, FieldNode
from dimos.experimental.agent_encode.pointcloud.handlers.lib.reference import reference
from dimos.experimental.agent_encode.pointcloud.render import raster as render
from dimos.experimental.agent_encode.pointcloud.render.overlays import draw_overlays, grid_pixel
from dimos.experimental.agent_encode.pointcloud.runtime.context import EncodeContext


def numbers(values: np.ndarray, decimals: int | None = None) -> Any:
    """JSON-compatible numbers with missing values represented by null.

    Whole-number arrays (counts, masks, labels) serialize as integers; others are
    rounded to ``decimals`` when given."""
    values = np.asarray(values, dtype=float)
    finite = np.isfinite(values)
    kept = values[finite]
    if np.array_equal(kept, np.round(kept)):
        kept = kept.astype(np.int64)
    elif decimals is not None:
        kept = np.round(kept, decimals)
    out = np.full(values.shape, None, dtype=object)
    out[finite] = kept.tolist()
    return out.tolist()


def output_decimals(decimals: int | None, grid: Any) -> int:
    """Caller's choice, else a thousandth of the cell size (0.1 m cells -> mm)."""
    if decimals is None:
        return max(0, int(np.ceil(-np.log10(grid.cell_m))) + 2)
    if type(decimals) is not int or not 0 <= decimals <= 9:
        raise ValueError("decimals must be an integer from 0 to 9")
    return decimals


def metadata(data: FieldData, ctx: EncodeContext) -> dict[str, Any]:
    return {
        "grid": data.grid.describe(ctx),
        "kind": data.kind,
        **{k: v for k, v in data.metadata.items() if k != "regions"},
    }


@dataclass(frozen=True)
class Sample:
    """Sample containing cells without interpolation; at uses the grid plane's coordinates.

    radius > 0 summarises every cell whose centre lies within radius of the point:
    min/max per channel, plus the distinct values for masks and component labels.
    """

    source: FieldNode
    at: Any
    fields: tuple[str, ...] | None = None
    decimals: int | None = None
    radius: float = 0.0

    def run(self, ctx: EncodeContext) -> dict[str, Any]:
        data = ctx.evaluate(self.source)
        xy = np.asarray(self.at, dtype=float)
        if xy.ndim != 2 or xy.shape[1] != 2 or not np.isfinite(xy).all():
            raise ValueError("at must be a list of finite coordinate pairs")
        if len(xy) > 4096:
            raise ValueError("sample exceeds 4096 locations; split the request")
        fields = self.fields if self.fields is not None else tuple(data.values)
        if any(name not in data.values for name in fields):
            raise ValueError(f"unknown field channel; channels are {list(data.values)}")
        if not np.isfinite(self.radius) or self.radius < 0:
            raise ValueError("radius must be finite and non-negative")
        places = output_decimals(self.decimals, data.grid)
        if self.radius > 0:
            samples = [self._disc(data, point, fields, places) for point in xy]
            return {
                **metadata(data, ctx),
                "handler": "Sample",
                "decimals": places,
                "radius": self.radius,
                "samples": samples,
            }
        ij, valid = data.grid.indices(xy)
        samples = []
        for point, cell, inside in zip(xy, ij, valid, strict=True):
            entry: dict[str, Any] = {"requested": point.tolist(), "status": "outside_grid"}
            if inside:
                col, row = cell
                values = {name: numbers(data.values[name][row, col], places) for name in fields}
                status = "no_targets" if data.metadata.get("status") == "no_targets" else "ok"
                if status == "ok" and all(v is None for v in values.values()):
                    status = "no_returns"
                entry.update(
                    status=status,
                    cell=cell.tolist(),
                    centre=np.round(
                        np.asarray(data.grid.origin) + (cell + 0.5) * data.grid.cell_m, places
                    ).tolist(),
                    values=values,
                )
            samples.append(entry)
        return {
            **metadata(data, ctx),
            "handler": "Sample",
            "decimals": places,
            "samples": samples,
        }

    def _disc(
        self, data: FieldData, point: np.ndarray, fields: tuple[str, ...], places: int
    ) -> dict[str, Any]:
        near = np.linalg.norm(data.grid.centres() - point, axis=-1) <= self.radius
        (cell,), (inside,) = data.grid.indices(point[None])
        if inside:
            near[cell[1], cell[0]] = True  # the containing cell, even when radius < cell_m
        entry: dict[str, Any] = {"requested": point.tolist(), "cells": int(near.sum())}
        if not near.any():
            return {**entry, "status": "outside_grid"}
        values: dict[str, Any] = {}
        for name in fields:
            cells = data.values[name][near].astype(float)
            cells = cells[np.isfinite(cells)]
            summary: dict[str, Any] = {"min": None, "max": None}
            if len(cells):
                summary = {"min": numbers(cells.min(), places), "max": numbers(cells.max(), places)}
                if data.kind in ("mask", "regions"):
                    summary["distinct"] = numbers(np.unique(cells)[:64])
            values[name] = summary
        status = "no_targets" if data.metadata.get("status") == "no_targets" else "ok"
        return {**entry, "status": status, "values": values}


@dataclass(frozen=True)
class Window:
    """Exact cell slice (column, row, width, height), preserving the parent grid."""

    source: FieldNode
    cells: tuple[int, int, int, int]
    fields: tuple[str, ...] | None = None
    decimals: int | None = None

    def run(self, ctx: EncodeContext) -> dict[str, Any]:
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
        fields = self.fields if self.fields is not None else tuple(data.values)
        if any(name not in data.values for name in fields):
            raise ValueError(f"unknown field channel; channels are {list(data.values)}")
        places = output_decimals(self.decimals, data.grid)
        return {
            **metadata(data, ctx),
            "handler": "Window",
            "cells": list(self.cells),
            "decimals": places,
            "values": {
                name: numbers(data.values[name][row : row + height, col : col + width], places)
                for name in fields
            },
        }


@dataclass(frozen=True)
class Map:
    """Render a scalar field, mask, or labels without modifying the field grid.

    Pixel origin is top-left; increasing the grid's second axis goes up.
    Images use nearest-neighbour enlargement only; oversized grids are rejected.
    """

    source: FieldNode
    value_range: tuple[float, float] | None = None
    overlays: tuple[Any, ...] = ()
    max_side: int = 1024

    def run(self, ctx: EncodeContext) -> dict[str, Any]:
        data = ctx.evaluate(self.source)
        values = data.scalar()
        nx, ny = data.grid.shape
        if (
            type(self.max_side) is not int
            or not 1 <= self.max_side <= 2048
            or max(nx, ny) > self.max_side
        ):
            raise ValueError("map grid exceeds max_side (1..2048); request a smaller grid region")
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
        out = {
            **metadata(data, ctx),
            "handler": "Map",
            "view_ref": reference(self, ctx),
            "channel": next(iter(data.values)),
            "value_range": [low, high],
            "missing_colour": [96, 96, 96],
            "display_aggregation": "none",
        }
        if render.FORM != "image":
            chars = np.where(finite, (fractions * 9).astype(int).astype(str), "?")
            out.update(
                ascii="\n".join("".join(row) for row in chars[::-1]),
                ascii_formula="value = low + digit / 9 * (high - low)",
                overlays_supported=False,
            )
            return out
        table = render._turbo()
        if table is None:
            table = np.repeat(np.arange(256, dtype=np.uint8)[:, None], 3, axis=1)
        rgb = table[(fractions * 255).astype(int)].copy()
        rgb[~finite] = (96, 96, 96)
        scale = max(1, self.max_side // max(nx, ny))
        picture = Image.fromarray(rgb[::-1].astype(np.uint8)).resize(
            (nx * scale, ny * scale), Image.Resampling.NEAREST
        )

        def project(point: np.ndarray) -> tuple[float, float]:
            return grid_pixel(point, data.grid.axes, data.grid.origin, data.grid.cell_m, ny, scale)

        with ctx.artifact("field.png") as (staging, path):
            picture.save(staging)
            overlays = draw_overlays(staging, self.overlays, ctx, project, view_ref=out["view_ref"])
        out.update(
            image=str(path),
            pixels_per_cell=scale,
            image_size=[nx * scale, ny * scale],
            pixel_transform={
                "origin": "top_left",
                "u": "(axis0-origin0)/cell_m*pixels_per_cell-0.5",
                "v": "(rows-(axis1-origin1)/cell_m)*pixels_per_cell-0.5",
            },
            colour_stops=[
                {"value": low + f * (high - low), "rgb": table[round(f * 255)].tolist()}
                for f in (0.0, 0.25, 0.5, 0.75, 1.0)
            ],
            overlays=overlays,
        )
        return out
