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

"""Where a malformed module surfaces: at the engine call site, as a self error.

The priced cost of no class generics (sketch banner): a bad step is not a
mypy error at the class statement — it fails at .over(). T3's
__init_subclass__ is the runtime backstop with the good message.

Static-typing fixture — never imported at runtime.
"""

from __future__ import annotations

from dimos.pure.typing import EngineSurface


class A:
    ts: float


class NoStep(EngineSurface):
    pass


NoStep().over()  # E[misc]: Invalid self argument


class BadArity(EngineSurface):
    """Four data params: matches no shape protocol."""

    def step(self, a: A, b: A, c: A, d: A) -> None:
        raise NotImplementedError


BadArity().over()  # E[misc]: Invalid self argument


class StepAndFold(EngineSurface):
    """Statically, step wins (first matching overload); T3 rejects at import."""

    class In:
        ts: float

    class Out:
        ts: float

    def step(self, i: In) -> Out:
        raise NotImplementedError

    def fold(self, rows: object) -> object:
        raise NotImplementedError


reveal_type(next(StepAndFold().over()))  # R: StepAndFold.Out
