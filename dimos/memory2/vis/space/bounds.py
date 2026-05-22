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

"""World-space bounding box accumulator shared by Space renderers.

Both `svg.py` and `raster.py` walk `space.elements`, growing this Bounds
with every drawable point, then map the resulting box into their target
canvas. Y semantics: world Y-up; canvases flip to Y-down at render time
(`_y(wy) = -wy` in SVG, `py = (ymax - wy) * res_px` in raster).
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Bounds:
    """Accumulates world-space bounding box during rendering."""

    xmin: float = float("inf")
    xmax: float = float("-inf")
    ymin: float = float("inf")
    ymax: float = float("-inf")

    def include(self, x: float, y: float) -> None:
        self.xmin = min(self.xmin, x)
        self.xmax = max(self.xmax, x)
        self.ymin = min(self.ymin, y)
        self.ymax = max(self.ymax, y)

    @property
    def empty(self) -> bool:
        return self.xmin > self.xmax

    @property
    def width(self) -> float:
        return max(self.xmax - self.xmin, 1.0)

    @property
    def height(self) -> float:
        return max(self.ymax - self.ymin, 1.0)
