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
from typing import Any

import numpy as np

from dimos.experimental.agent_encode.pointcloud.runtime.context import EncodeContext
from dimos.experimental.agent_encode.pointcloud.shapes.box import Box
from dimos.experimental.agent_encode.pointcloud.shapes.cylinder import Cylinder
from dimos.experimental.agent_encode.pointcloud.shapes.sphere import Sphere

Shape = Box | Cylinder | Sphere


@dataclass(frozen=True)
class Sweep:
    """Sample a shape's motion along a heading or a 3D direction.

    Samples include the start and endpoint; contacts between samples can be missed.
    The result gives hit (bool), distance_m the shape travelled before first touching
    a return (null when it reaches max_distance untouched), and point_m [x, y, z] of
    that first return. start_inside counts the returns already inside the shape
    before it moves; any gives distance_m 0, so shrink or move the shape to sweep
    past them.
    """

    shape: Shape
    direction_deg: float | None = None
    """Horizontal heading, degrees from +x toward +y."""
    max_distance: float = 1.0
    step_m: float = 0.05
    source: Any = None
    direction: tuple[float, float, float] | None = None
    """A 3D heading (dx, dy, dz) in place of ``direction_deg``; it is normalized."""

    def run(self, ctx: EncodeContext) -> dict[str, Any]:
        ctx = ctx.select(self.source)
        if (
            not math.isfinite(self.step_m)
            or not math.isfinite(self.max_distance)
            or self.step_m <= 0
            or self.max_distance < 0
        ):
            raise ValueError(
                "Sweep needs a finite positive step_m and finite non-negative max_distance"
            )
        if self.direction is not None:
            if self.direction_deg is not None:
                raise ValueError("Sweep accepts either direction or direction_deg, not both")
            direction = np.asarray(self.direction, dtype=np.float64)
            if direction.shape != (3,) or not np.isfinite(direction).all():
                raise ValueError("direction must contain three finite values")
            norm = float(np.linalg.norm(direction))
            if not math.isfinite(norm) or norm == 0:
                raise ValueError("direction must have a finite nonzero length")
            direction = direction / norm
        else:
            if self.direction_deg is None or not math.isfinite(self.direction_deg):
                raise ValueError("Sweep requires a finite direction_deg or a 3D direction")
            heading = math.radians(self.direction_deg)
            direction = np.array([math.cos(heading), math.sin(heading), 0.0])
        thinnest = _thinnest(self.shape)
        if thinnest < self.step_m:
            raise ValueError(
                f"Sweep shape is {thinnest:g} m thin but steps {self.step_m:g} m, so returns "
                "between steps are skipped; sweep a body-sized shape, or use Closest for a point"
            )
        out: dict[str, Any] = {
            "handler": "Sweep",
            **self.shape.describe(),
            "direction_deg": self.direction_deg,
            "max_distance": self.max_distance,
            "direction": direction.tolist(),
            "step_m": self.step_m,
            "method": "sampled",
            "precision_m": self.step_m,
            "sampling_note": "Contacts between sampled positions can be missed.",
        }
        points = ctx.points
        out["start_inside"] = int(self.shape.contains(points).sum())
        steps = math.ceil(self.max_distance / self.step_m)
        for k in range(steps + 1):
            t = min(k * self.step_m, self.max_distance)
            # Inverse translation supports vertical motion for every existing shape.
            inside = self.shape.contains(points - t * direction)
            if inside.any():
                hits = points[inside]
                # The first return reached: the one furthest back along the travel direction.
                along = hits @ direction
                first = hits[int(np.argmin(along))]
                out.update(
                    hit=True,
                    distance_m=round(t, 3),
                    point_m=[round(float(v), 3) for v in first],
                    status="sampled_hit",
                    samples_checked=k + 1,
                )
                return out
        out.update(
            hit=False,
            distance_m=None,
            point_m=None,
            status="no_sampled_hit",
            samples_checked=steps + 1,
        )
        return out


def _thinnest(shape: Any) -> float:
    """Smallest extent of the swept shape: returns thinner than a step can slip past."""
    if hasattr(shape, "size"):
        return float(min(shape.size))
    return 2.0 * float(shape.radius)
