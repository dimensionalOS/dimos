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

"""Shape conformance is checked at the def — before any wiring exists.

A shape is a module whose step raises; implementations inherit the bundles
and override step. Wrong return or narrowed input is an [override] error.

Static-typing fixture — never imported at runtime. Origin: opt2_shapes.py.
"""

from __future__ import annotations

from dimos.pure.typing import EngineSurface


class Image:
    brightness: float


class Shape(EngineSurface):
    class In:
        ts: float
        image: Image

    class Out:
        ts: float
        located: str

    def step(self, i: In) -> Out:
        raise NotImplementedError


class Conforming(Shape):
    def step(self, i: Shape.In) -> Shape.Out:
        raise NotImplementedError


reveal_type(next(Conforming().over()))  # R: Shape.Out


class RicherIn(Shape.In):
    depth: float


class WrongReturn(Shape):
    def step(self, i: Shape.In) -> str:  # E[override]: Return type
        raise NotImplementedError


class NarrowedIn(Shape):
    def step(self, i: RicherIn) -> Shape.Out:  # E[override]: incompatible with supertype
        raise NotImplementedError
