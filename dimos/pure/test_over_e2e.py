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

"""End-to-end ``over()``: canonical modules through the real align + driver stack.

Integration seam tests (orchestrator-owned): every shape runs pm-surface-only,
over synthetic stamped streams — the composition no single task's suite covers.
"""

from __future__ import annotations

from collections.abc import Iterator
import dataclasses
from typing import NamedTuple

import pytest

from dimos import pure as pm


@dataclasses.dataclass(frozen=True)
class Sample:
    ts: float
    v: float


def _stream(*pairs: tuple[float, float]) -> list[Sample]:
    return [Sample(ts=t, v=v) for t, v in pairs]


class Doubler(pm.PureModule):
    class In(pm.In):
        x: Sample = pm.tick()

    class Out(pm.Out):
        y: float = pm.contract(min_hz=1)

    def step(self, i: In) -> Out:
        return Doubler.Out(y=i.x.v * 2)


class Gate(pm.PureModule):
    threshold: float = 2.0

    class In(pm.In):
        x: Sample = pm.tick()

    class Out(pm.Out):
        y: float = 0.0

    def step(self, i: In) -> Out | None:
        return Gate.Out(y=i.x.v) if i.x.v >= self.threshold else None


class Scaled(pm.PureModule):
    class In(pm.In):
        x: Sample = pm.tick()
        k: Sample = pm.latest()

    class Out(pm.Out):
        y: float = 0.0

    def step(self, i: In) -> Out:
        return Scaled.Out(y=i.x.v * i.k.v)


class Summer(pm.PureModule):
    class In(pm.In):
        x: Sample = pm.tick()

    class Out(pm.Out):
        total: float = 0.0

    class State(NamedTuple):
        total: float = 0.0

    def step(self, s: State, i: In) -> tuple[State, Out]:
        t = s.total + i.x.v
        return Summer.State(total=t), Summer.Out(total=t)


class AsyncEcho(pm.PureModule):
    max_inflight: int = 2

    class In(pm.In):
        x: Sample = pm.tick()

    class Out(pm.Out):
        y: float = 0.0

    async def step(self, i: In) -> Out:
        return AsyncEcho.Out(y=i.x.v)


class PairBatcher(pm.PureModule):
    class In(pm.In):
        x: Sample = pm.tick()

    class Out(pm.Out):
        total: float = 0.0

    def fold(self, rows: Iterator[In]) -> Iterator[Out]:
        buf: list[PairBatcher.In] = []
        for r in rows:
            buf.append(r)
            if len(buf) == 2:
                yield PairBatcher.Out(ts=buf[-1].ts, total=sum(b.x.v for b in buf))
                buf = []


def test_stateless_stamps_tick_ts() -> None:
    rows = list(Doubler().over(x=_stream((1.0, 1.0), (2.0, 3.0))))
    assert [r.y for r in rows] == [2.0, 6.0]
    assert [r.ts for r in rows] == [1.0, 2.0]


def test_stateless_none_skips_and_config_changes_behavior() -> None:
    xs = _stream((1.0, 1.0), (2.0, 3.0), (3.0, 2.0))
    assert [r.y for r in Gate().over(x=xs)] == [3.0, 2.0]
    assert [r.y for r in Gate(threshold=2.5).over(x=xs)] == [3.0]


def test_latest_port_resolves_at_tick() -> None:
    rows = list(Scaled().over(x=_stream((1.0, 2.0), (2.0, 3.0)), k=_stream((0.5, 10.0))))
    assert [r.y for r in rows] == [20.0, 30.0]


def test_mealy_threads_state_from_default() -> None:
    rows = list(Summer().over(x=_stream((1.0, 1.0), (2.0, 2.0), (3.0, 3.0))))
    assert [r.total for r in rows] == [1.0, 3.0, 6.0]
    assert [r.ts for r in rows] == [1.0, 2.0, 3.0]


def test_async_emits_in_tick_order() -> None:
    rows = list(AsyncEcho().over(x=_stream((1.0, 1.0), (2.0, 2.0), (3.0, 3.0))))
    assert [r.y for r in rows] == [1.0, 2.0, 3.0]
    assert [r.ts for r in rows] == [1.0, 2.0, 3.0]


def test_fold_self_stamps() -> None:
    rows = list(PairBatcher().over(x=_stream((1.0, 1.0), (2.0, 2.0), (3.0, 3.0), (4.0, 4.0))))
    assert [(r.ts, r.total) for r in rows] == [(2.0, 3.0), (4.0, 7.0)]


def test_over_reinvokable_and_unknown_stream_eager() -> None:
    m = Doubler()
    xs = _stream((1.0, 1.0))
    assert [r.y for r in m.over(x=xs)] == [2.0]
    assert [r.y for r in m.over(x=xs)] == [2.0]
    with pytest.raises(pm.PureModuleRunError):
        m.over(nope=[])
