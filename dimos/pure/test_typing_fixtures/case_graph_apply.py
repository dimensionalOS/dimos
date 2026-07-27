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

"""Symbolic application typing (T13 §3): m(x=port) -> member Out; graphs via Wires.

Static-typing fixture — never imported at runtime; checked by
test_typing_static.py. Pins the two-plane doctrine: data-plane payload types
ride the bundle annotations; Port appears statically only at feedback().
"""

from __future__ import annotations

from dimos.pure.graph import Port, PureGraph, feedback
from dimos.pure.module import PureModule


class Cloud:
    pass


class Grid:
    pass


class Mapper(PureModule):
    class In:
        ts: float
        scan: Cloud

    class Out:
        ts: float
        grid: Grid

    def step(self, i: In) -> Out:
        raise NotImplementedError


class Smoother(PureModule):
    class In:
        ts: float
        grid: Grid
        prev: Grid

    class Out:
        ts: float
        grid: Grid

    class State:
        pass

    def step(self, state: State, i: In) -> tuple[State, Out]:
        raise NotImplementedError


# ── module application: keyword ports return the member's Out, typed ─────────
m = Mapper()
o = m(scan=Cloud())
reveal_type(o)  # R: Mapper.Out
reveal_type(o.grid)  # R: Grid

# Mealy members infer through the same overload stack.
s = Smoother()(grid=o.grid, prev=o.grid)
reveal_type(s.grid)  # R: Grid

# ── the transformer face survives the overload split (positional iterator) ───
reveal_type(m(iter([])))  # R: typing.Iterator[Any]


# ── graph application + entry points type through the Wires protocol ─────────
class Nav(PureGraph):
    voxel: float = 0.1

    class In:
        ts: float
        lidar: Cloud

    class Out:
        ts: float
        grid: Grid

    def wire(self, i: In) -> Out:
        inner = Mapper()(scan=i.lidar)
        reveal_type(i.lidar)  # R: Cloud
        reveal_type(inner.grid)  # R: Grid
        raise NotImplementedError


g = Nav(voxel=0.2)
res = g(lidar=Cloud())
reveal_type(res)  # R: Nav.Out
reveal_type(g.over())  # R: dimos.pure.graph.GraphRun[Nav.Out]

# ── feedback: the one place Port appears statically; close() is payload-typed ─
prev: Port[Grid] = feedback()
prev.close(Grid())
prev.close(Cloud())  # E[arg-type]: expected "Grid"
