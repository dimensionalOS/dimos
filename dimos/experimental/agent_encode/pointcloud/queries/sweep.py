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
from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import NDArray

from dimos.experimental.agent_encode.pointcloud.image.lib.canvas import Canvas, Drawable
from dimos.experimental.agent_encode.pointcloud.queries.base import Query
from dimos.experimental.agent_encode.pointcloud.queries.lib.points import finite_points
from dimos.experimental.agent_encode.pointcloud.shapes.base import Shape

if TYPE_CHECKING:
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@dataclass(frozen=True)
class SweepResult(Drawable):
    """How far a shape moved before first touching a return."""

    shape: Shape
    direction: tuple[float, float, float]
    """The unit direction of travel."""
    distance_m: float | None
    """How far the shape travelled before first touching a return; None when it reached
    max_m untouched."""
    point_m: tuple[float, float, float] | None
    """The first return touched."""
    start_inside: int
    """Returns already inside the shape before it moved; any gives distance_m 0, so move
    the shape to sweep past them."""
    max_m: float

    def draw(self, canvas: Canvas) -> None:
        self.shape.draw(canvas)
        start = self.shape.anchor(canvas.z_extent)
        travel = self.distance_m if self.distance_m is not None else self.max_m
        end = start + np.asarray(self.direction, dtype=np.float64) * travel
        canvas.path(np.stack((start, end)))
        if self.point_m is not None:
            canvas.point(np.asarray(self.point_m, dtype=np.float64), 4, filled=True)


@dataclass(frozen=True)
class Sweep(Query[SweepResult]):
    """Move a shape along a heading or a 3D direction, testing it every ``step_m`` from the
    start to ``max_m``; contacts between samples can be missed."""

    shape: Shape
    heading_deg: float | None = None
    """Horizontal heading, degrees from +x toward +y."""
    direction: tuple[float, float, float] | None = None
    """A 3D direction (dx, dy, dz) in place of ``heading_deg``; it is normalised."""
    max_m: float = 1.0
    """How far to move."""
    step_m: float = 0.05
    """Distance between tested positions."""

    def __post_init__(self) -> None:
        if not math.isfinite(self.step_m) or self.step_m <= 0:
            raise ValueError("step_m must be finite and positive")
        if not math.isfinite(self.max_m) or self.max_m < 0:
            raise ValueError("max_m must be finite and non-negative")
        if self.max_m / self.step_m > 4096:
            raise ValueError("Sweep takes at most 4096 steps; raise step_m or shorten max_m")
        if (self.direction is None) == (self.heading_deg is None):
            raise ValueError("Sweep takes exactly one of heading_deg or direction")
        if self.direction is not None:
            if len(self.direction) != 3 or not np.isfinite(self.direction).all():
                raise ValueError("direction must contain three finite values")
            if not np.any(self.direction):
                raise ValueError("direction must have a nonzero length")
        if self.heading_deg is not None and not math.isfinite(self.heading_deg):
            raise ValueError("heading_deg must be finite")
        chord = self.shape.chord(self._unit())
        if chord < self.step_m:
            raise ValueError(
                f"Sweep shape is {chord:g} m deep along its travel but steps {self.step_m:g} m, "
                "so returns between steps are skipped; sweep a body-sized shape, or use "
                "Nearest for a point"
            )

    def _unit(self) -> NDArray[np.float64]:
        if self.heading_deg is not None:
            heading = math.radians(self.heading_deg)
            return np.array([math.cos(heading), math.sin(heading), 0.0])
        direction = np.asarray(self.direction, dtype=np.float64)
        unit: NDArray[np.float64] = direction / np.linalg.norm(direction)
        return unit

    def run(self, cloud: PointCloud2) -> SweepResult:
        direction = self._unit()
        dx, dy, dz = (round(float(v), 3) for v in direction)
        points = finite_points(cloud)
        start_inside = int(self.shape.contains(points).sum())
        steps = math.ceil(self.max_m / self.step_m)
        for k in range(steps + 1):
            t = min(k * self.step_m, self.max_m)
            # Moving the returns back by t is moving the shape forward by t.
            inside = self.shape.contains(points - t * direction)
            if inside.any():
                hits = points[inside]
                # The first return reached: the one furthest back along the travel direction.
                x, y, z = (round(float(v), 3) for v in hits[int(np.argmin(hits @ direction))])
                return SweepResult(
                    self.shape, (dx, dy, dz), round(t, 3), (x, y, z), start_inside, self.max_m
                )
        return SweepResult(self.shape, (dx, dy, dz), None, None, start_inside, self.max_m)
