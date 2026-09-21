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
from typing import Any

import numpy as np

from dimos.experimental.agent_encode.pointcloud.runtime.context import EncodeContext
from dimos.experimental.agent_encode.pointcloud.shapes.box import Box
from dimos.experimental.agent_encode.pointcloud.shapes.cylinder import Cylinder
from dimos.experimental.agent_encode.pointcloud.shapes.sphere import Sphere

Shape = Box | Cylinder | Sphere


@dataclass(frozen=True)
class Closest:
    """The return nearest to a shape's surface, and how far away it is.

    The result gives distance_m from the surface to that return (0 when a return is
    inside) and its point_m [x, y, z]; both are null when no return qualifies.
    """

    shape: Shape
    """A Cylinder only considers returns inside its z_range and measures horizontally."""
    source: Any = None

    def run(self, ctx: EncodeContext) -> dict[str, Any]:
        ctx = ctx.select(self.source)
        out: dict[str, Any] = {"handler": "Closest", **self.shape.describe()}
        if len(ctx.points) == 0:
            out.update(distance_m=None, point_m=None)
            return out
        d = self.shape.distance(ctx.points)
        i = int(np.argmin(d))
        if not np.isfinite(d[i]):
            out.update(distance_m=None, point_m=None)
            return out
        out["distance_m"] = round(float(d[i]), 3)
        out["point_m"] = [round(float(v), 3) for v in ctx.points[i]]
        return out
