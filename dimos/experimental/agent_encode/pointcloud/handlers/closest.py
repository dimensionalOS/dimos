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

import numpy as np

from dimos.experimental.agent_encode.pointcloud.render.overlays import Canvas, Overlay
from dimos.experimental.agent_encode.pointcloud.runtime.context import EncodeContext, Node
from dimos.experimental.agent_encode.pointcloud.shapes.base import Shape


@dataclass(frozen=True)
class ClosestResult:
    """Both None when no return qualifies."""

    distance_m: float | None
    """From the shape's surface to the nearest return; 0 when a return is inside."""
    point_m: tuple[float, float, float] | None


@dataclass(frozen=True)
class Closest(Overlay):
    """The return nearest to a shape's surface, and how far away it is."""

    shape: Shape
    """A Cylinder only considers returns inside its z_range and measures horizontally."""
    source: Node[np.ndarray] | None = None

    def draw(self, canvas: Canvas) -> list[str]:
        geometry = self.shape.draw(canvas)
        point = canvas.ctx.evaluate(self).point_m
        if point is not None:
            if canvas.point(np.asarray(point), 4, filled=True):
                geometry.append("point")
            canvas.path(np.stack((self.shape.anchor(canvas.z_extent), point)))
            geometry.append("nearest_segment")
        return geometry

    def run(self, ctx: EncodeContext) -> ClosestResult:
        ctx = ctx.select(self.source)
        if len(ctx.points) == 0:
            return ClosestResult(None, None)
        d = self.shape.distance(ctx.points)
        i = int(np.argmin(d))
        if not np.isfinite(d[i]):
            return ClosestResult(None, None)
        x, y, z = (round(float(v), 3) for v in ctx.points[i])
        return ClosestResult(round(float(d[i]), 3), (x, y, z))
