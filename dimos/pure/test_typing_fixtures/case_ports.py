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

"""m.i / m.o: per-module typed port views via protocol-overloaded __get__.

The bundle parameter is exact per module (zero annotations on the module);
per-FIELD access is the documented Any hole — names and value types are
checked at runtime by the rim, not statically (spec §ports).

Static-typing fixture — never imported at runtime.
"""

from __future__ import annotations

from collections.abc import Iterator

from dimos.pure.typing import EngineSurface


class Image:
    brightness: float


class Tagger(EngineSurface):
    class In:
        ts: float
        image: Image

    class Out:
        ts: float
        located: str

    def step(self, i: In) -> Out:
        raise NotImplementedError


m = Tagger()
reveal_type(m.i)  # R: dimos.pure.typing.InPorts[Tagger.In]
reveal_type(m.o)  # R: dimos.pure.typing.OutPorts[Tagger.Out]
reveal_type(m.i.image)  # R: dimos.pure.typing.InPort[Any]
reveal_type(m.o.located)  # R: dimos.pure.typing.OutPort[Any]

# the sketch's deployment lines typecheck:
m.i.image.transport = object()
m.i.image.source = object()
m.o.located.transport = object()
frames = m.o.located.frames
reveal_type(frames)  # R: tuple[builtins.str, builtins.str]
sub = m.o.located.subscribe(lambda row: None)

# descriptor-provided member is read-only (spelled @property):
m.o.located.frames = ("a", "b")  # E[misc]: read-only

# documented hole: field names are not statically checked (rim validates):
typo = m.i.imagee
reveal_type(typo)  # R: dimos.pure.typing.InPort[Any]

# class access resolves to the accessor itself:
reveal_type(Tagger.i)  # R: dimos.pure.typing._InAccessor


class AsyncMod(EngineSurface):
    class In:
        ts: float

    class Out:
        ts: float

    async def step(self, i: In) -> Out:
        raise NotImplementedError


# the order canary one level down: if Stateless preceded AsyncStateless in
# _OutAccessor, this would reveal OutPorts[Coroutine[...]]:
reveal_type(AsyncMod().o)  # R: dimos.pure.typing.OutPorts[AsyncMod.Out]
reveal_type(AsyncMod().i)  # R: dimos.pure.typing.InPorts[AsyncMod.In]


class StatefulMod(EngineSurface):
    class In:
        ts: float

    class Out:
        ts: float

    class State:
        pass

    def step(self, state: State, i: In) -> tuple[State, Out | None]:
        raise NotImplementedError


reveal_type(StatefulMod().i)  # R: dimos.pure.typing.InPorts[StatefulMod.In]
reveal_type(StatefulMod().o)  # R: dimos.pure.typing.OutPorts[StatefulMod.Out]


class FoldMod(EngineSurface):
    class In:
        ts: float

    class Out:
        ts: float

    def fold(self, rows: Iterator[In]) -> Iterator[Out]:
        raise NotImplementedError


reveal_type(FoldMod().i)  # R: dimos.pure.typing.InPorts[FoldMod.In]
reveal_type(FoldMod().o)  # R: dimos.pure.typing.OutPorts[FoldMod.Out]
