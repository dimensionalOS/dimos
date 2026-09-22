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
from functools import cached_property
import math

import numpy as np

from dimos.experimental.agent_encode.pointcloud.render.overlays import Canvas, Overlay
from dimos.experimental.agent_encode.pointcloud.runtime.context import EncodeContext, Node
from dimos.experimental.agent_encode.pointcloud.shapes.base import Shape


@dataclass(frozen=True)
class SweepResult:
    hit: bool
    distance_m: float | None
    """How far the shape travelled before first touching a return; None when it reached
    max_distance untouched."""
    point_m: tuple[float, float, float] | None
    """The first return touched."""
    direction: list[float]
    """The unit direction of travel."""
    start_inside: int
    """Returns already inside the shape before it moved; any gives distance_m 0, so move
    the shape to sweep past them."""
    samples_checked: int


@dataclass(frozen=True)
class Sweep(Overlay):
    """Sample a shape's motion along a heading or a 3D direction, every ``step_m`` from
    the start to the endpoint; contacts between samples can be missed."""

    shape: Shape
    direction_deg: float | None = None
    """Horizontal heading, degrees from +x toward +y."""
    max_distance: float = 1.0
    step_m: float = 0.05
    source: Node[np.ndarray] | None = None
    direction: tuple[float, float, float] | None = None
    """A 3D heading (dx, dy, dz) in place of ``direction_deg``; it is normalized."""

    @cached_property
    def unit_direction(self) -> np.ndarray:
        if self.direction_deg is not None:
            heading = math.radians(self.direction_deg)
            return np.array([math.cos(heading), math.sin(heading), 0.0])
        direction = np.asarray(self.direction, dtype=np.float64)
        unit: np.ndarray = direction / np.linalg.norm(direction)
        return unit

    def draw(self, canvas: Canvas) -> list[str]:
        geometry = self.shape.draw(canvas)
        result = canvas.ctx.evaluate(self)
        if result.point_m is not None and canvas.point(np.asarray(result.point_m), 4, filled=True):
            geometry.append("point")
        start = self.shape.anchor(canvas.z_extent)
        travel = result.distance_m if result.distance_m is not None else self.max_distance
        canvas.path(np.stack((start, start + self.unit_direction * travel)))
        return [*geometry, "sweep_segment"]

    def run(self, ctx: EncodeContext) -> SweepResult:
        if not math.isfinite(self.step_m) or self.step_m <= 0:
            raise ValueError("step_m must be finite and positive")
        if not math.isfinite(self.max_distance) or self.max_distance < 0:
            raise ValueError("max_distance must be finite and non-negative")
        if self.max_distance / self.step_m > 4096:
            raise ValueError("Sweep takes at most 4096 steps; raise step_m or shorten max_distance")
        if (self.direction is None) == (self.direction_deg is None):
            raise ValueError("Sweep takes exactly one of direction or direction_deg")
        if self.direction is not None:
            if len(self.direction) != 3 or not np.isfinite(self.direction).all():
                raise ValueError("direction must contain three finite values")
            if not np.any(self.direction):
                raise ValueError("direction must have a nonzero length")
        if self.direction_deg is not None and not math.isfinite(self.direction_deg):
            raise ValueError("direction_deg must be finite")
        chord = self.shape.chord(self.unit_direction)
        if chord < self.step_m:
            raise ValueError(
                f"Sweep shape is {chord:g} m deep along its travel but steps {self.step_m:g} m, "
                "so returns between steps are skipped; sweep a body-sized shape, or use "
                "Closest for a point"
            )
        ctx = ctx.select(self.source)
        direction = self.unit_direction
        points = ctx.points
        start_inside = int(self.shape.contains(points).sum())
        steps = math.ceil(self.max_distance / self.step_m)
        for k in range(steps + 1):
            t = min(k * self.step_m, self.max_distance)
            # Inverse translation supports vertical motion for every existing shape.
            inside = self.shape.contains(points - t * direction)
            if inside.any():
                hits = points[inside]
                # The first return reached: the one furthest back along the travel direction.
                x, y, z = (round(float(v), 3) for v in hits[int(np.argmin(hits @ direction))])
                return SweepResult(
                    True, round(t, 3), (x, y, z), direction.tolist(), start_inside, k + 1
                )
        return SweepResult(False, None, None, direction.tolist(), start_inside, steps + 1)
