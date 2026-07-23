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

"""Protocol data params are positional-only: implementations keep name freedom.

Without ``/`` in the protocols, renaming ``i`` to ``frame`` would silently
break structural conformance and surface as a distant invalid-self error at
over(). The flip side, also pinned: keyword-calling step through a
protocol-typed reference is not a supported surface.

Static-typing fixture — never imported at runtime. (T3 coordination, 2a.)
"""

from __future__ import annotations

from dimos.pure.typing import EngineSurface, Stateless


class Image:
    brightness: float


class Renamed(EngineSurface):
    class In:
        ts: float
        image: Image

    class Out:
        ts: float
        located: str

    def step(self, frame: In) -> Out:
        raise NotImplementedError


reveal_type(next(Renamed().over()))  # R: Renamed.Out


class RenamedMealy(EngineSurface):
    class In:
        ts: float

    class Out:
        ts: float

    class State:
        pass

    def step(self, acc: State, row: In) -> tuple[State, Out | None]:
        raise NotImplementedError


reveal_type(next(RenamedMealy().over()))  # R: RenamedMealy.Out


def keyword_caller(m: Stateless[Renamed.In, Renamed.Out]) -> None:
    m.step(i=Renamed.In())  # E[call-arg]: Unexpected keyword argument
