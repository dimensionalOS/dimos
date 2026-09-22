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

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Literal

import numpy as np
from pydantic import JsonValue

from dimos.experimental.agent_encode.pointcloud.handlers.lib.reference import reference
from dimos.experimental.agent_encode.pointcloud.handlers.lib.surface import (
    Pickable,
    PickResult,
    PickSurface,
    bounds,
)
from dimos.experimental.agent_encode.pointcloud.render import raster as render
from dimos.experimental.agent_encode.pointcloud.render.overlays import (
    Canvas,
    DrawnOverlay,
    Overlay,
    draw_overlays,
)
from dimos.experimental.agent_encode.pointcloud.runtime.context import (
    EncodeContext,
    Request,
    Result,
    Selection,
)


@dataclass(frozen=True)
class _DepthRaster:
    """The raster a DepthView draws, shared by every view with the same camera."""

    view: tuple[float, float, float, float, float]
    fov_deg: float
    size: tuple[int, int]
    max_depth: float | None
    point_size_m: float | None

    def run(self, ctx: EncodeContext) -> render.DepthRaster:
        return render.depth_raster(
            ctx.points,
            render.as_view(self.view),
            fov_deg=self.fov_deg,
            size=self.size,
            max_depth=self.max_depth,
            point_size_m=self.point_size_m,
        )


Provenance = Literal["no_return", "projected_return", "splat", "filled_pixel"]


@dataclass(frozen=True)
class DepthHit:
    point_id: int
    """The return's index among the finite returns of the view's source."""
    point_m: list[float]
    forward_depth_m: float
    projected_uv: list[float]
    pixel_provenance: list[Provenance]
    """How the return reached the selected pixels."""
    selected_pixel_count: int


@dataclass(frozen=True)
class DepthPick(PickResult):
    hits: list[DepthHit]
    hits_omitted: int
    depth_span_m: tuple[float, float] | None

    def draw(self, canvas: Canvas) -> list[str]:
        return [
            "picked_return"
            for hit in self.hits
            if canvas.point(np.asarray(hit.point_m), 3, filled=False)
        ]


@dataclass(frozen=True)
class DepthSurface(PickSurface):
    """A perspective render: each image pixel shows at most one return."""

    raster: render.DepthRaster
    points: np.ndarray
    """The returns the render drew, indexed by ``raster.point_ids``."""
    view: render.View

    @property
    def size(self) -> tuple[int, int]:
        height, width = self.raster.depth.shape
        return width, height

    def measure(
        self, pixels: np.ndarray, max_items: int, selection_ref: dict[str, JsonValue]
    ) -> tuple[DepthPick, np.ndarray]:
        raster = self.raster
        ids = raster.point_ids[pixels[:, 1], pixels[:, 0]]
        unique = np.unique(ids[ids >= 0])
        origin = np.array([self.view.x, self.view.y, self.view.z])
        depths = (self.points[unique] - origin) @ self.view.axes()[0]
        drawn_as: tuple[Provenance, ...] = (
            "no_return",
            "projected_return",
            "splat",
            "filled_pixel",
        )
        hits = []
        for point_id, depth in zip(unique[:max_items], depths[:max_items], strict=True):
            locations = pixels[ids == point_id]
            provenance = np.unique(raster.provenance[locations[:, 1], locations[:, 0]])
            hits.append(
                DepthHit(
                    int(point_id),
                    self.points[point_id].tolist(),
                    float(depth),
                    raster.projected_uv[point_id].tolist(),
                    [drawn_as[int(p)] for p in provenance],
                    len(locations),
                )
            )
        selected = self.points[unique]
        result = DepthPick(
            "hit" if len(unique) == 1 else ("ambiguous" if len(unique) else "no_return"),
            self.size,
            len(pixels),
            len(unique),
            bounds(selected),
            selection_ref,
            hits,
            max(0, len(unique) - max_items),
            (float(depths.min()), float(depths.max())) if len(unique) else None,
        )
        return result, selected


@dataclass(frozen=True)
class DepthViewResult(Result):
    image: Path
    view_ref: dict[str, JsonValue]
    covered_pixels: int
    """Pixels with a return."""
    point_size_m: tuple[float, float] | None
    """The smallest and largest width drawn."""
    colour: render.ColourScale
    """Depths from near to far on a log scale; black is no return."""
    overlays: list[DrawnOverlay]


@dataclass(frozen=True)
class DepthView(Request[DepthViewResult], Pickable):
    """A perspective depth render of the cloud as an image."""

    view: tuple[float, float, float, float, float]
    """x, y, z, yaw_deg, pitch_deg."""
    fov_deg: float = 90.0
    size: tuple[int, int] = (768, 480)
    """Width, height in pixels; large enough for a VLM to read detail."""
    max_depth: float | None = None
    """No cap: every return in front of the view is drawn."""
    point_size_m: float | None = None
    """One width for every return. None draws each as wide as the gap to its nearest
    neighbour, so surfaces close at the cloud's own resolution."""
    source: Selection | None = None
    overlays: tuple[Overlay, ...] = ()

    def raster(self, ctx: EncodeContext) -> tuple[render.DepthRaster, EncodeContext]:
        """The drawn raster and the context of the returns it drew."""
        selected = ctx.select(self.source)
        camera = _DepthRaster(self.view, self.fov_deg, self.size, self.max_depth, self.point_size_m)
        return selected.evaluate(camera), selected

    def surface(self, ctx: EncodeContext) -> DepthSurface:
        raster, selected = self.raster(ctx)
        return DepthSurface(raster, selected.points, render.as_view(self.view))

    def run(self, ctx: EncodeContext) -> DepthViewResult:
        original_ctx = ctx
        raster, ctx = self.raster(ctx)
        pose = render.as_view(self.view)
        near, far = render.colour_range(raster.depth, self.max_depth)
        view_ref = reference(self, original_ctx)
        overlays: list[DrawnOverlay] = []
        with ctx.artifact("depth.png") as (staging, path):
            render.depth_png(raster.depth, staging, max_depth=self.max_depth)
            if self.overlays:
                forward, right, up = pose.axes()
                origin = np.array([pose.x, pose.y, pose.z])
                focal = (self.size[0] / 2) / math.tan(math.radians(self.fov_deg) / 2)

                def project(point: np.ndarray) -> tuple[float, float] | None:
                    relative = point - origin
                    distance = float(relative @ forward)
                    if distance <= 0.05 or (
                        self.max_depth is not None and distance > self.max_depth
                    ):
                        return None
                    return (
                        float(relative @ right) / distance * focal + self.size[0] / 2,
                        -float(relative @ up) / distance * focal + self.size[1] / 2,
                    )

                overlays = draw_overlays(
                    staging, self.overlays, original_ctx, project, view_ref=view_ref
                )
        return DepthViewResult(
            path,
            view_ref,
            int(np.isfinite(raster.depth).sum()),
            raster.widths,
            render.colour_scale(max(near, 0.05), far, log=True, reverse=True),
            overlays,
        )
