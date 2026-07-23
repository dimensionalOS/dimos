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

"""Mealy step ``(State, In) -> tuple[State, Out | None]``: TOut solves to Out.

Static-typing fixture — never imported at runtime. Origin: opt2_selfproto.py
(Cycler) + opt2_variants.py (MealyMod).
"""

from __future__ import annotations

from dimos.pure.typing import EngineSurface


class Mapper(EngineSurface):
    class In:
        ts: float

    class Out:
        ts: float
        n: int

    class State:
        count: int = 0

    def step(self, state: State, i: In) -> tuple[State, Out | None]:
        raise NotImplementedError


reveal_type(next(Mapper().over()))  # R: Mapper.Out
reveal_type(next(Mapper().over()).n)  # R: builtins.int


class AlwaysEmits(EngineSurface):
    """``tuple[State, Out]`` without None also matches (tuple covariance)."""

    class In:
        ts: float

    class Out:
        ts: float

    class State:
        pass

    def step(self, state: State, i: In) -> tuple[State, Out]:
        raise NotImplementedError


reveal_type(next(AlwaysEmits().over()))  # R: AlwaysEmits.Out
