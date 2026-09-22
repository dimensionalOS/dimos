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

"""Portable visual queries. References contain bounded recipes, never point buffers or paths."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from pydantic import JsonValue

from dimos.experimental.agent_encode.pointcloud.handlers.lib.reference import (
    reference,
    resolve,
)
from dimos.experimental.agent_encode.pointcloud.handlers.lib.surface import Pickable, PickResult
from dimos.experimental.agent_encode.pointcloud.render.overlays import Canvas, Overlay
from dimos.experimental.agent_encode.pointcloud.runtime.context import EncodeContext


def _pixels(pick: Pick, width: int, height: int) -> tuple[np.ndarray, bool]:
    if sum(v is not None for v in (pick.uv, pick.rect, pick.polygon)) != 1:
        raise ValueError("choose exactly one of uv, rect, polygon")
    if type(pick.radius_px) is not int or not 0 <= pick.radius_px <= 64:
        raise ValueError("radius_px must be an integer in 0..64")
    if pick.uv is None and pick.radius_px:
        raise ValueError("radius_px applies only to uv")
    polygon = None
    if pick.uv is not None:
        if len(pick.uv) != 2 or any(type(v) is not int for v in pick.uv):
            raise ValueError("uv must contain integer native pixel indices")
        u, v = pick.uv
        if not (0 <= u < width and 0 <= v < height):
            return np.empty((0, 2), dtype=np.int64), True
        r = pick.radius_px
        x0, y0, x1, y1 = max(0, u - r), max(0, v - r), min(width, u + r + 1), min(height, v + r + 1)
    elif pick.rect is not None:
        if len(pick.rect) != 4 or any(type(v) is not int for v in pick.rect):
            raise ValueError("rect must contain integer (u,v,width,height)")
        x0, y0, w, h = pick.rect
        if min(w, h) <= 0:
            raise ValueError("rectangle dimensions must be positive")
        x1, y1 = x0 + w, y0 + h
    else:
        polygon = np.asarray(pick.polygon, dtype=float)
        if (
            polygon.ndim != 2
            or polygon.shape[1] != 2
            or not 3 <= len(polygon) <= 32
            or not np.isfinite(polygon).all()
        ):
            raise ValueError("polygon needs 3..32 finite native-pixel vertices")
        if np.any(polygon < 0) or np.any(polygon > [width, height]):
            raise ValueError("polygon must lie within image edges")
        x0, y0 = np.floor(polygon.min(0)).astype(int)
        x1, y1 = np.ceil(polygon.max(0)).astype(int)
    if x0 < 0 or y0 < 0 or x1 > width or y1 > height:
        raise ValueError("region must lie inside image edges")
    if (x1 - x0) * (y1 - y0) > 65536:
        raise ValueError("pick region exceeds 65536 pixels; use a smaller region")
    yy, xx = np.mgrid[y0:y1, x0:x1]
    pixels = np.column_stack((xx.ravel(), yy.ravel()))
    if polygon is not None:
        # Even-odd rule at pixel centres; boundary ties use this same deterministic rule.
        x, y = pixels[:, 0] + 0.5, pixels[:, 1] + 0.5
        inside = np.zeros(len(pixels), dtype=bool)
        for a, b in zip(polygon, np.roll(polygon, -1, axis=0), strict=True):
            if a[1] != b[1]:
                inside ^= ((a[1] > y) != (b[1] > y)) & (
                    x < (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0]
                )
        pixels = pixels[inside]
    return pixels, False


@dataclass(frozen=True)
class Pick(Overlay):
    """Lazy exact visual measurement of one region of a render, at most 65536 pixels.

    The reusable selection includes all selected returns.
    """

    view_ref: dict[str, JsonValue]
    uv: tuple[int, int] | None = None
    """A top-left native integer pixel index."""
    radius_px: int = 0
    """Selects a clipped square neighbourhood of ``uv``, at most 64."""
    rect: tuple[int, int, int, int] | None = None
    """(u, v, width, height), half-open."""
    polygon: tuple[tuple[float, float], ...] | None = None
    """3..32 vertices; it selects pixel centres by the even-odd rule."""
    max_items: int = 16
    """Reported hits or cells, at most 64."""

    @property
    def selection(self) -> PickSelection:
        return PickSelection(self)

    def _measure(self, ctx: EncodeContext) -> tuple[PickResult, np.ndarray]:
        if type(self.max_items) is not int or not 1 <= self.max_items <= 64:
            raise ValueError("max_items must be an integer in 1..64")
        node = resolve(self.view_ref, ctx)
        if not isinstance(node, Pickable):
            raise ValueError("view_ref must describe DepthView, Map, or OccupancyMap")
        surface = node.surface(ctx)
        pixels, outside = _pixels(self, *surface.size)
        selection_ref = reference(self.selection, ctx, "selection")
        if not len(pixels):
            empty = PickResult(
                "outside_image" if outside else "no_return", surface.size, 0, 0, None, selection_ref
            )
            return empty, ctx.points[:0]
        return surface.measure(pixels, self.max_items, selection_ref)

    def measured(self, ctx: EncodeContext) -> tuple[PickResult, np.ndarray]:
        """The result and the selected returns, measured once per call."""
        return ctx.root.evaluate(_PickMeasurement(self))

    def run(self, ctx: EncodeContext) -> PickResult:
        return self.measured(ctx)[0]

    def draw(self, canvas: Canvas) -> list[str]:
        result = canvas.ctx.evaluate(self)
        geometry = []
        # Native-pixel highlights are valid only on the exact referenced view recipe.
        if canvas.view_ref == self.view_ref:
            self._draw_region(canvas)
            geometry.append("picked_pixels")
        return [*geometry, *result.draw(canvas)]

    def _draw_region(self, canvas: Canvas) -> None:
        if self.uv is not None:
            (u, v), r = self.uv, self.radius_px
            canvas.draw.rectangle((u - r, v - r, u + r, v + r), outline=canvas.colour, width=1)
            canvas.draw.ellipse((u - 4, v - 4, u + 4, v + 4), outline=canvas.colour, width=1)
        elif self.rect is not None:
            u, v, w, h = self.rect
            canvas.draw.rectangle((u, v, u + w - 1, v + h - 1), outline=canvas.colour, width=2)
        else:
            assert self.polygon is not None
            vertices = [(float(x), float(y)) for x, y in self.polygon]
            canvas.draw.polygon(vertices, outline=canvas.colour, width=2)


@dataclass(frozen=True)
class PickSelection:
    pick: Pick

    def run(self, ctx: EncodeContext) -> np.ndarray:
        return self.pick.measured(ctx)[1]


@dataclass(frozen=True)
class SelectionRef:
    """Reopen a Pick result's JSON selection_ref against the exact original cloud."""

    selection_ref: dict[str, JsonValue]

    def run(self, ctx: EncodeContext) -> np.ndarray:
        node = resolve(self.selection_ref, ctx, "selection")
        if not isinstance(node, PickSelection):
            raise ValueError("selection_ref must describe a pick selection")
        return node.run(ctx.root)


@dataclass(frozen=True)
class _PickMeasurement:
    pick: Pick

    def run(self, ctx: EncodeContext) -> tuple[PickResult, np.ndarray]:
        return self.pick._measure(ctx)
