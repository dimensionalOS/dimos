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
from typing import Any

from dimos.experimental.agent_encode.pointcloud.fields import Select
from dimos.experimental.agent_encode.pointcloud.runtime.context import EncodeContext
from dimos.experimental.agent_encode.pointcloud.shapes.box import Box
from dimos.experimental.agent_encode.pointcloud.shapes.cylinder import Cylinder
from dimos.experimental.agent_encode.pointcloud.shapes.sphere import Sphere

Shape = Box | Cylinder | Sphere


@dataclass(frozen=True)
class Overlap:
    """Are there returns inside a shape, and where do they lie.

    The result gives the count of returns inside and, when any, bounds_m as
    [[x_min, y_min, z_min], [x_max, y_max, z_max]] of those returns. A count of 0
    says nothing was returned from inside the shape; it does not prove the volume
    is empty.
    """

    shape: Shape
    source: Any = None

    @cached_property
    def selection(self) -> Select:
        """The returns counted by this query, reusable as another node's source."""
        return Select(include=self.shape, source=self.source)

    def run(self, ctx: EncodeContext) -> dict[str, Any]:
        ctx = ctx.select(self.source)
        inside = ctx.points[self.shape.contains(ctx.points)]
        out: dict[str, Any] = {
            "handler": "Overlap",
            **self.shape.describe(),
            "count": len(inside),
        }
        if len(inside):
            out["bounds_m"] = [
                [round(float(v), 3) for v in inside.min(axis=0)],
                [round(float(v), 3) for v in inside.max(axis=0)],
            ]
        else:
            out["bounds_m"] = None
        return out
