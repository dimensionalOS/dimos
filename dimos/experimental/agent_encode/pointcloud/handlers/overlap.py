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

import numpy as np

from dimos.experimental.agent_encode.pointcloud.fields import Select
from dimos.experimental.agent_encode.pointcloud.render.overlays import Canvas, Overlay
from dimos.experimental.agent_encode.pointcloud.runtime.context import EncodeContext, Node
from dimos.experimental.agent_encode.pointcloud.shapes.base import Shape


@dataclass(frozen=True)
class OverlapResult:
    count: int
    """Returns inside. 0 says nothing was returned from inside the shape; it does not
    prove the volume is empty."""
    bounds_m: tuple[list[float], list[float]] | None
    """Lowest and highest corner of the returns inside."""


@dataclass(frozen=True)
class Overlap(Overlay):
    """Are there returns inside a shape, and where do they lie."""

    shape: Shape
    source: Node[np.ndarray] | None = None

    @cached_property
    def selection(self) -> Select:
        """The returns counted by this query, reusable as another node's source."""
        return Select(include=self.shape, source=self.source)

    def draw(self, canvas: Canvas) -> list[str]:
        return self.shape.draw(canvas)

    def run(self, ctx: EncodeContext) -> OverlapResult:
        ctx = ctx.select(self.source)
        inside = ctx.points[self.shape.contains(ctx.points)]
        if not len(inside):
            return OverlapResult(0, None)
        low, high = np.round(inside.min(axis=0), 3), np.round(inside.max(axis=0), 3)
        return OverlapResult(len(inside), (low.tolist(), high.tolist()))
