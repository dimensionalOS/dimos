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

"""Structural twin: ``Stateless[Shape.In, Shape.Out]`` accepts anything step-shaped.

Inherit to share bundles and get def-site checking; use the alias to accept
"anything shape-shaped", MRO membership optional. Origin: opt2_shapes.py +
sketch3 §4 (CostMapperLike).

Static-typing fixture — never imported at runtime.
"""

from __future__ import annotations

from typing import TypeAlias

from dimos.pure.typing import EngineSurface, Stateless


class Shape(EngineSurface):
    class In:
        ts: float

    class Out:
        ts: float
        cost: float

    def step(self, i: In) -> Out:
        raise NotImplementedError


ShapeLike: TypeAlias = Stateless[Shape.In, Shape.Out]


class Inheriting(Shape):
    def step(self, i: Shape.In) -> Shape.Out:
        raise NotImplementedError


class Unrelated(EngineSurface):
    """Same step shape, no MRO relation to Shape."""

    def step(self, i: Shape.In) -> Shape.Out:
        raise NotImplementedError


class Different(EngineSurface):
    """Nominally different In bundle — not shape-shaped."""

    class In:
        ts: float

    def step(self, i: In) -> Shape.Out:
        raise NotImplementedError


def deploy(m: ShapeLike) -> None:
    pass


deploy(Inheriting())
deploy(Unrelated())
deploy(Different())  # E[arg-type]: incompatible type
