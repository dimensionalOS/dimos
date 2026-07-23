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

"""Unit tests for T6 drivers (spec: dimos/pure/tasks/t6-drivers.md §14).

Written against the spec, module-skipped until the implementation lands.
Fixtures are plain classes plus real pm.In/pm.Out bundles — deliberately no
PureModule import anywhere: drivers drive anything step-shaped (spec §1).
Async tests choreograph completion order with asyncio.Event between steps —
no sleeps, no wall clock. run_over tests stub dimos.pure.align in
sys.modules, pinning the assumed T5 contract (spec §3 A1) executably.
"""

from __future__ import annotations

import asyncio
import sys
import types
from typing import NamedTuple

import pytest

from dimos.pure import pm
from dimos.pure.drivers import (
    PureModuleRunError,
    RunHooks,
    RunRule,
    StepError,
    drive_async,
    drive_fold,
    drive_mealy,
    drive_stateless,
    run_over,
)
from dimos.pure.rows import Progress, contract, tick
from dimos.pure.stepspec import StepKind, StepSpec

# ── shared fixtures: real bundles, plain (non-PureModule) step carriers ──────


class Ping(pm.In):
    x: float = tick()


class Pong(pm.Out):
    y: float = contract(min_hz=1)


def _rows(*ts: float) -> list[Ping]:
    return [Ping(ts=t, x=t) for t in ts]


def _spec(kind: StepKind, state_type: type | None = None, skips: bool = False) -> StepSpec:
    return StepSpec(
        kind=kind, in_type=Ping, out_type=Pong, state_type=state_type, skips=skips, owner=object
    )


class EchoStep:  # stateless, skips=False
    def step(self, i: Ping) -> Pong:
        return Pong(y=i.x)


class EvenStep:  # stateless, skips=True — emits even ticks only
    def step(self, i: Ping) -> Pong | None:
        return Pong(y=i.x) if int(i.x) % 2 == 0 else None


class StampingStep:  # doctrinally confused: sets ts explicitly
    def step(self, i: Ping) -> Pong:
        return Pong(ts=999.0, y=i.x)


class LyingStep:  # annotated never-skip, returns None anyway
    def step(self, i: Ping) -> Pong:
        return None  # type: ignore[return-value]


class BoomStep:
    def step(self, i: Ping) -> Pong:
        raise ValueError("boom")


class EveryOther:  # Mealy, skips=True — emits every second tick
    class State(NamedTuple):
        n: int = 0

    def step(self, state: State, i: Ping) -> tuple[State, Pong | None]:
        nxt = EveryOther.State(n=state.n + 1)
        return nxt, (Pong(y=i.x) if nxt.n % 2 == 0 else None)


class Recorder:  # Mealy, records every state it was handed
    class State(NamedTuple):
        n: int = 0

    def __init__(self) -> None:
        self.seen: list[int] = []

    def step(self, state: State, i: Ping) -> tuple[State, Pong]:
        self.seen.append(state.n)
        return Recorder.State(n=state.n + 1), Pong(y=float(state.n))


class Batcher2:  # fold: batches of 2, stamps at the batch's last row
    def fold(self, rows_it):
        batch: list[float] = []
        for r in rows_it:
            batch.append(r.x)
            if len(batch) == 2:
                yield Pong(ts=r.ts, y=sum(batch))
                batch.clear()


def _stub_align(monkeypatch, rows_ret):
    """Install a fake dimos.pure.align; returns the recorded (in_type, streams) calls."""
    calls: list[tuple[type, dict]] = []

    def align(in_type, streams, *, tf=None, on_decision=None, progress=False):
        calls.append((in_type, dict(streams)))
        return iter(rows_ret)

    mod = types.ModuleType("dimos.pure.align")
    mod.align = align  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "dimos.pure.align", mod)
    return calls


# ── stateless (spec §5.1) ────────────────────────────────────────────────────


def test_stateless_emits_and_stamps():
    hooks = RunHooks()
    outs = list(drive_stateless(EchoStep(), iter(_rows(1.0, 2.0)), skips=False, hooks=hooks))
    assert [o.y for o in outs] == [1.0, 2.0]
    assert [o.ts for o in outs] == [1.0, 2.0]  # engine-stamped with tick ts, never UNSTAMPED
    assert (hooks.ticks, hooks.emits, hooks.skips, hooks.drops) == (2, 2, 0, 0)


def test_stateless_skip_counting():
    hooks = RunHooks()
    outs = list(
        drive_stateless(EvenStep(), iter(_rows(1.0, 2.0, 3.0, 4.0)), skips=True, hooks=hooks)
    )
    assert [o.ts for o in outs] == [2.0, 4.0]
    assert (hooks.ticks, hooks.emits, hooks.skips) == (4, 2, 2)


def test_stateless_ts_overwrite():
    # T1 D11: user-set ts on a step Out row is overwritten, never an error.
    outs = list(drive_stateless(StampingStep(), iter(_rows(5.0)), skips=False, hooks=RunHooks()))
    assert outs[0].ts == 5.0


def test_stateless_none_no_skip_raises():
    # T3 §15 Q4 settled (spec §5.3): annotation-contradicting None is an error.
    hooks = RunHooks()
    it = drive_stateless(LyingStep(), iter(_rows(1.0)), skips=False, hooks=hooks)
    with pytest.raises(PureModuleRunError) as ei:
        next(it)
    assert ei.value.rule is RunRule.STEP_NONE_NO_SKIP
    assert "step-none-no-skip" in str(ei.value)
    assert hooks.errors == 1


def test_stateless_lazy_pull():
    pulled: list[float] = []

    def src():
        for r in _rows(1.0, 2.0, 3.0):
            pulled.append(r.ts)
            yield r

    it = drive_stateless(EchoStep(), src(), skips=False, hooks=RunHooks())
    assert pulled == []  # nothing before first next()
    next(it)
    assert pulled == [1.0]  # one row per tick, no prefetch


def test_stateless_step_error_wrapped():
    hooks = RunHooks()
    it = drive_stateless(BoomStep(), iter(_rows(7.0)), skips=False, hooks=hooks)
    with pytest.raises(StepError) as ei:
        next(it)
    assert isinstance(ei.value.__cause__, ValueError)  # original chained, uncloaked
    assert "7.0" in str(ei.value)  # tick coordinates in the message
    assert hooks.errors == 1
    assert hooks.last_error is ei.value.__cause__  # hook keeps the raw original


# ── mealy (spec §5.2) ────────────────────────────────────────────────────────


def test_mealy_threads_state_from_default():
    m = Recorder()
    outs = list(
        drive_mealy(m, iter(_rows(1.0, 2.0, 3.0)), Recorder.State(), skips=False, hooks=RunHooks())
    )
    assert m.seen == [0, 1, 2]  # started from State(), threaded linearly
    assert [o.y for o in outs] == [0.0, 1.0, 2.0]
    assert [o.ts for o in outs] == [1.0, 2.0, 3.0]


def test_mealy_skip_and_emit():
    hooks = RunHooks()
    outs = list(
        drive_mealy(
            EveryOther(),
            iter(_rows(1.0, 2.0, 3.0, 4.0)),
            EveryOther.State(),
            skips=True,
            hooks=hooks,
        )
    )
    assert [o.ts for o in outs] == [2.0, 4.0]
    assert (hooks.ticks, hooks.emits, hooks.skips) == (4, 2, 2)


def test_mealy_fresh_state_per_run():
    m = EveryOther()

    def run() -> list[float]:
        it = drive_mealy(
            m, iter(_rows(1.0, 2.0, 3.0, 4.0)), EveryOther.State(), skips=True, hooks=RunHooks()
        )
        return [o.ts for o in it]

    assert run() == run() == [2.0, 4.0]  # no state bleed across runs


# ── mealy finish() tail flush (ACCEPTED design) ──────────────────────────────


class Accumulator:  # Mealy + finish: per-tick running sum, finish flushes the scan count
    class State(NamedTuple):
        total: float = 0.0
        n: int = 0

    def __init__(self) -> None:
        self.finished: list[int] = []

    def step(self, state: State, i: Ping) -> tuple[State, Pong]:
        nxt = Accumulator.State(total=state.total + i.x, n=state.n + 1)
        return nxt, Pong(y=nxt.total)

    def finish(self, state: State) -> Pong | None:
        self.finished.append(state.n)  # observe that finish ran, exactly once
        return Pong(y=float(state.n))


class SilentFinisher:  # Mealy whose finish returns None (nothing to flush)
    class State(NamedTuple):
        n: int = 0

    def step(self, state: State, i: Ping) -> tuple[State, Pong]:
        return SilentFinisher.State(n=state.n + 1), Pong(y=i.x)

    def finish(self, state: State) -> Pong | None:
        return None


def test_mealy_finish_flushes_at_exhaustion():
    m = Accumulator()
    hooks = RunHooks()
    outs = list(
        drive_mealy(
            m,
            iter(_rows(1.0, 2.0, 3.0)),
            Accumulator.State(),
            skips=False,
            hooks=hooks,
            finish=True,
        )
    )
    assert [o.y for o in outs] == [1.0, 3.0, 6.0, 3.0]  # 3 step sums + finish count(3)
    assert outs[-1].ts == 3.0  # tail stamped from the LAST consumed tick's ts
    assert m.finished == [3]  # finish fired exactly once, at exhaustion
    assert (hooks.ticks, hooks.emits) == (3, 4)  # 3 ticks; the tail counts as an emit


def test_mealy_finish_none_emits_nothing():
    hooks = RunHooks()
    outs = list(
        drive_mealy(
            SilentFinisher(),
            iter(_rows(1.0, 2.0)),
            SilentFinisher.State(),
            skips=False,
            hooks=hooks,
            finish=True,
        )
    )
    assert [o.ts for o in outs] == [1.0, 2.0]  # no tail row
    assert hooks.emits == 2


def test_mealy_finish_not_fired_on_early_close():
    m = Accumulator()
    it = drive_mealy(
        m,
        iter(_rows(1.0, 2.0, 3.0)),
        Accumulator.State(),
        skips=False,
        hooks=RunHooks(),
        finish=True,
    )
    next(it)  # one step
    it.close()  # consumer breaks out early — an abort, not a stream end
    assert m.finished == []  # finish never ran


def test_mealy_finish_skipped_on_empty_stream():
    m = Accumulator()
    outs = list(
        drive_mealy(m, iter([]), Accumulator.State(), skips=False, hooks=RunHooks(), finish=True)
    )
    assert outs == [] and m.finished == []  # no tick consumed → no ts to stamp, no finish


def test_mealy_finish_flag_off_never_calls_hook():
    m = Accumulator()
    outs = list(
        drive_mealy(m, iter(_rows(1.0)), Accumulator.State(), skips=False, hooks=RunHooks())
    )
    assert m.finished == [] and len(outs) == 1  # finish=False (default): hook untouched


# ── progress markers (graph interior edges) ─────────────────────────────────


def test_stateless_progress_yields_marker_on_skip():
    hooks = RunHooks()
    outs = list(
        drive_stateless(
            EvenStep(), iter(_rows(1.0, 2.0, 3.0)), skips=True, hooks=hooks, progress=True
        )
    )
    assert [type(o) for o in outs] == [Progress, Pong, Progress]
    assert [o.ts for o in outs] == [1.0, 2.0, 3.0]
    assert (hooks.ticks, hooks.emits, hooks.skips) == (3, 1, 2)  # markers are not emits


def test_mealy_progress_yields_marker_on_skip():
    outs = list(
        drive_mealy(
            EveryOther(),
            iter(_rows(1.0, 2.0)),
            EveryOther.State(),
            skips=True,
            hooks=RunHooks(),
            progress=True,
        )
    )
    assert [type(o) for o in outs] == [Progress, Pong]
    assert outs[0].ts == 1.0 and outs[1].ts == 2.0


def test_drivers_pass_aligner_markers_through():
    # an aligner-dropped tick arrives as a marker: passed through, never stepped
    rows: list = [Progress(0.5), *_rows(1.0)]
    outs = list(drive_stateless(EchoStep(), iter(rows), skips=False, hooks=RunHooks()))
    assert isinstance(outs[0], Progress) and outs[0].ts == 0.5
    assert isinstance(outs[1], Pong) and outs[1].ts == 1.0


# ── fold (spec §7) ───────────────────────────────────────────────────────────


def test_fold_self_stamp_valid():
    hooks = RunHooks()
    outs = list(drive_fold(Batcher2(), iter(_rows(1.0, 2.0, 3.0, 4.0, 5.0)), hooks=hooks))
    assert [(o.ts, o.y) for o in outs] == [(2.0, 3.0), (4.0, 7.0)]  # self-stamped, untouched
    assert (hooks.ticks, hooks.emits, hooks.skips) == (5, 2, 0)  # ticks = rows the fold pulled


def test_fold_unstamped_caught():
    class ForgetsStamp:
        def fold(self, rows_it):
            for r in rows_it:
                yield Pong(y=r.x)  # ts stays UNSTAMPED

    it = drive_fold(ForgetsStamp(), iter(_rows(1.0)), hooks=RunHooks())
    with pytest.raises(PureModuleRunError) as ei:
        next(it)
    assert ei.value.rule is RunRule.FOLD_UNSTAMPED


def test_fold_nonmonotonic_equal_ts():
    class EqualStamper:
        def fold(self, rows_it):
            for r in rows_it:
                yield Pong(ts=5.0, y=r.x)  # second yield repeats ts=5.0

    it = drive_fold(EqualStamper(), iter(_rows(5.0, 6.0)), hooks=RunHooks())
    assert next(it).ts == 5.0
    with pytest.raises(PureModuleRunError) as ei:
        next(it)
    assert ei.value.rule is RunRule.FOLD_NONMONOTONIC  # strict: equal ts is a violation


def test_fold_nonmonotonic_regression():
    class RegressingStamper:
        def fold(self, rows_it):
            it2 = iter(rows_it)
            r1 = next(it2)
            next(it2)
            yield Pong(ts=r1.ts, y=r1.x)
            yield Pong(ts=r1.ts - 1.0, y=0.0)

    it = drive_fold(RegressingStamper(), iter(_rows(6.0, 7.0)), hooks=RunHooks())
    assert next(it).ts == 6.0
    with pytest.raises(PureModuleRunError) as ei:
        next(it)
    assert ei.value.rule is RunRule.FOLD_NONMONOTONIC


def test_fold_future_ts():
    class FutureStamper:
        def fold(self, rows_it):
            for r in rows_it:
                yield Pong(ts=r.ts + 100.0, y=r.x)  # ahead of newest consumed input

    it = drive_fold(FutureStamper(), iter(_rows(3.0)), hooks=RunHooks())
    with pytest.raises(PureModuleRunError) as ei:
        next(it)
    assert ei.value.rule is RunRule.FOLD_FUTURE_TS


def test_fold_yield_before_any_input():
    class EagerYielder:
        def fold(self, rows_it):
            yield Pong(ts=0.0, y=0.0)  # no input consumed: no time authority

    it = drive_fold(EagerYielder(), iter(_rows(1.0)), hooks=RunHooks())
    with pytest.raises(PureModuleRunError) as ei:
        next(it)
    assert ei.value.rule is RunRule.FOLD_FUTURE_TS


def test_fold_yield_none():
    class NoneYielder:
        def fold(self, rows_it):
            next(iter(rows_it))
            yield None

    it = drive_fold(NoneYielder(), iter(_rows(1.0)), hooks=RunHooks())
    with pytest.raises(PureModuleRunError) as ei:
        next(it)
    assert ei.value.rule is RunRule.FOLD_YIELDED_NONE


def test_fold_lazy_single_pass():
    pulled: list[float] = []

    def src():
        for r in _rows(1.0, 2.0, 3.0, 4.0):
            pulled.append(r.ts)
            yield r

    it = drive_fold(Batcher2(), src(), hooks=RunHooks())
    assert pulled == []  # driver prefetches nothing
    out = next(it)
    assert out.ts == 2.0
    assert pulled == [1.0, 2.0]  # exactly the fold's own pulls


def test_fold_teardown_on_break():
    closed: list[bool] = []
    torn: list[bool] = []

    class Flushy:
        def fold(self, rows_it):
            try:
                for r in rows_it:
                    yield Pong(ts=r.ts, y=r.x)
            finally:
                closed.append(True)

    hooks = RunHooks(teardown=lambda: torn.append(True))
    it = drive_fold(Flushy(), iter(_rows(1.0, 2.0, 3.0)), hooks=hooks)
    next(it)
    it.close()  # consumer break
    assert closed == [True]  # GeneratorExit delivered into the fold generator
    assert torn == [True]  # seam ran exactly once


def test_fold_tail_flush_after_exhaustion():
    class TailFlusher:
        def fold(self, rows_it):
            last, total = None, 0.0
            for r in rows_it:
                last, total = r, total + r.x
            if last is not None:
                yield Pong(ts=last.ts, y=total)  # post-exhaustion flush: legal

    outs = list(drive_fold(TailFlusher(), iter(_rows(1.0, 2.0)), hooks=RunHooks()))
    assert [(o.ts, o.y) for o in outs] == [(2.0, 3.0)]


def test_fold_step_error_wrapped():
    class BoomFold:
        def fold(self, rows_it):
            next(iter(rows_it))
            raise ValueError("mid-fold boom")
            yield  # pragma: no cover — makes this a generator function

    it = drive_fold(BoomFold(), iter(_rows(1.0)), hooks=RunHooks())
    with pytest.raises(StepError) as ei:
        next(it)
    assert isinstance(ei.value.__cause__, ValueError)


# ── async (spec §6): event-choreographed, no sleeps, no wall clock ───────────


def test_async_tick_order_emission():
    order: list[float] = []
    ev1 = asyncio.Event()

    class Chor:
        async def step(self, i: Ping) -> Pong:
            if i.x == 1.0:
                await ev1.wait()  # tick 1 completes only after tick 2 ran
            else:
                ev1.set()
            order.append(i.x)
            return Pong(y=i.x)

    hooks = RunHooks()
    outs = list(
        drive_async(Chor(), iter(_rows(1.0, 2.0)), max_inflight=2, skips=False, hooks=hooks)
    )
    assert order == [2.0, 1.0]  # completion order: reversed
    assert [o.ts for o in outs] == [1.0, 2.0]  # emission order: tick order
    assert (hooks.ticks, hooks.emits) == (2, 2)


def test_async_inflight_bound():
    cur = peak = 0
    release = asyncio.Event()

    class Gauge:
        async def step(self, i: Ping) -> Pong:
            nonlocal cur, peak
            cur += 1
            peak = max(peak, cur)
            if i.x == 2.0:
                release.set()
            await release.wait()
            cur -= 1
            return Pong(y=i.x)

    outs = list(
        drive_async(
            Gauge(), iter(_rows(1.0, 2.0, 3.0, 4.0)), max_inflight=2, skips=False, hooks=RunHooks()
        )
    )
    assert len(outs) == 4
    assert peak == 2  # window saturates at, and never exceeds, max_inflight


def test_async_drop_on_exception():
    class Flaky:
        async def step(self, i: Ping) -> Pong:
            if i.x == 2.0:
                raise ValueError("transient")
            return Pong(y=i.x)

    hooks = RunHooks()
    outs = list(
        drive_async(Flaky(), iter(_rows(1.0, 2.0, 3.0)), max_inflight=2, skips=False, hooks=hooks)
    )
    assert [o.ts for o in outs] == [1.0, 3.0]  # dropped tick leaves no gap-stall
    assert hooks.drops == 1
    assert isinstance(hooks.last_error, ValueError)
    assert (hooks.ticks, hooks.emits, hooks.errors) == (3, 2, 0)  # a drop is not an error


def test_async_none_skip():
    class AsyncEven:
        async def step(self, i: Ping) -> Pong | None:
            return Pong(y=i.x) if int(i.x) % 2 == 0 else None

    hooks = RunHooks()
    outs = list(
        drive_async(
            AsyncEven(), iter(_rows(1.0, 2.0, 3.0)), max_inflight=2, skips=True, hooks=hooks
        )
    )
    assert [o.ts for o in outs] == [2.0]
    assert (hooks.ticks, hooks.emits, hooks.skips, hooks.drops) == (3, 1, 2, 0)


def test_async_none_no_skip_propagates():
    # Engine-detected violation propagates even in async — never counted as a drop.
    class AsyncLiar:
        async def step(self, i: Ping) -> Pong:
            return None  # type: ignore[return-value]

    hooks = RunHooks()
    it = drive_async(AsyncLiar(), iter(_rows(1.0)), max_inflight=2, skips=False, hooks=hooks)
    with pytest.raises(PureModuleRunError) as ei:
        next(it)
    assert ei.value.rule is RunRule.STEP_NONE_NO_SKIP
    assert hooks.drops == 0
    assert hooks.errors == 1


def test_async_drain_emits_completed():
    ev = asyncio.Event()

    class TailHeavy:
        async def step(self, i: Ping) -> Pong:
            if i.x < 3.0:
                await ev.wait()  # 1 and 2 block until 3 runs
            else:
                ev.set()
            return Pong(y=i.x)

    outs = list(
        drive_async(
            TailHeavy(), iter(_rows(1.0, 2.0, 3.0)), max_inflight=3, skips=False, hooks=RunHooks()
        )
    )
    assert [o.ts for o in outs] == [1.0, 2.0, 3.0]  # exhaustion drains fully, in order


def test_async_break_cancels_inflight():
    started: list[float] = []
    done: list[float] = []
    cancelled: list[float] = []
    torn: list[bool] = []
    ev = asyncio.Event()  # never set: blockers park forever

    class Blocky:
        async def step(self, i: Ping) -> Pong:
            started.append(i.x)
            if i.x != 1.0:
                try:
                    await ev.wait()
                except asyncio.CancelledError:
                    cancelled.append(i.x)
                    raise
            done.append(i.x)
            return Pong(y=i.x)

    hooks = RunHooks(teardown=lambda: torn.append(True))
    it = drive_async(Blocky(), iter(_rows(1.0, 2.0, 3.0)), max_inflight=3, skips=False, hooks=hooks)
    assert next(it).ts == 1.0
    it.close()  # consumer break -> CLOSING: cancel + reap the window
    assert started == [1.0, 2.0, 3.0]  # every admitted tick started
    assert done == [1.0]  # only the emitted tick completed
    assert sorted(cancelled) == [2.0, 3.0]  # in-flight reaped, none leaked
    assert torn == [True]  # seam exactly once


def test_async_ateardown_runs_on_run_loop():
    flags: list[bool] = []

    async def adispose() -> None:
        flags.append(True)  # spec §8.4: awaited inside the run loop, before close

    class Instant:
        async def step(self, i: Ping) -> Pong:
            return Pong(y=i.x)

    hooks = RunHooks(ateardown=adispose)
    list(drive_async(Instant(), iter(_rows(1.0)), max_inflight=1, skips=False, hooks=hooks))
    assert flags == [True]


# ── teardown (spec §8) ───────────────────────────────────────────────────────


def test_baseexception_propagates_raw_stateless():
    # §10.2/§8.3 row 6: BaseException is never wrapped or counted; teardown runs once.
    class Interrupted:
        def step(self, i: Ping) -> Pong:
            raise KeyboardInterrupt

    torn: list[bool] = []
    hooks = RunHooks(teardown=lambda: torn.append(True))
    it = drive_stateless(Interrupted(), iter(_rows(1.0)), skips=False, hooks=hooks)
    with pytest.raises(KeyboardInterrupt):
        next(it)
    assert torn == [True]
    assert (hooks.errors, hooks.last_error) == (0, None)  # raw unwind, uncounted


def test_async_baseexception_propagates_raw():
    # §6.4: BaseException from a step task propagates raw — never a counted drop;
    # CLOSING still reaps the in-flight sibling on the way out.
    ev = asyncio.Event()  # never set: the sibling parks until cancelled
    cancelled: list[float] = []

    class Interrupted:
        async def step(self, i: Ping) -> Pong:
            if i.x == 1.0:
                raise KeyboardInterrupt
            try:
                await ev.wait()
            except asyncio.CancelledError:
                cancelled.append(i.x)
                raise
            return Pong(y=i.x)

    hooks = RunHooks()
    it = drive_async(Interrupted(), iter(_rows(1.0, 2.0)), max_inflight=2, skips=False, hooks=hooks)
    with pytest.raises(KeyboardInterrupt):
        next(it)
    assert cancelled == [2.0]  # §6.3 CLOSING reaped the sibling, none leaked
    assert (hooks.drops, hooks.errors) == (0, 0)  # BaseException: never counted (§6.4)


def test_teardown_on_break_stateless():
    torn: list[bool] = []
    hooks = RunHooks(teardown=lambda: torn.append(True))
    it = drive_stateless(EchoStep(), iter(_rows(1.0, 2.0, 3.0)), skips=False, hooks=hooks)
    next(it)
    it.close()
    assert torn == [True]


def test_teardown_exactly_once_on_error():
    torn: list[bool] = []
    hooks = RunHooks(teardown=lambda: torn.append(True))
    it = drive_stateless(BoomStep(), iter(_rows(1.0)), skips=False, hooks=hooks)
    with pytest.raises(StepError):
        next(it)
    assert torn == [True]
    it.close()  # closing a finished generator must not re-run the seam
    assert torn == [True]


def test_teardown_closes_rows():
    rows_closed: list[bool] = []

    def src():
        try:
            yield from _rows(1.0, 2.0)
        finally:
            rows_closed.append(True)

    it = drive_stateless(EchoStep(), src(), skips=False, hooks=RunHooks())
    next(it)
    it.close()
    assert rows_closed == [True]  # spec §8.2: drivers own and close rows


# ── run_over (spec §4): eager validation + composition over stubbed align ────


def test_run_over_unknown_stream_eager():
    with pytest.raises(PureModuleRunError) as ei:
        run_over(EchoStep(), _spec(StepKind.STATELESS), {"nope": []})  # raises at call
    assert ei.value.rule is RunRule.UNKNOWN_STREAM
    assert "nope" in str(ei.value)
    assert "x" in str(ei.value)  # names the valid In fields


def test_run_over_bad_max_inflight_eager():
    class BadWindow:
        max_inflight = 0

        async def step(self, i: Ping) -> Pong:
            return Pong(y=i.x)

    with pytest.raises(PureModuleRunError) as ei:
        run_over(BadWindow(), _spec(StepKind.ASYNC_STATELESS), {"x": []})
    assert ei.value.rule is RunRule.BAD_MAX_INFLIGHT


def test_run_over_inside_loop_eager():
    # §6.2: sync over() cannot be driven from inside a running loop — eager guard.
    class AsyncEcho:
        async def step(self, i: Ping) -> Pong:
            return Pong(y=i.x)

    async def call() -> PureModuleRunError:
        with pytest.raises(PureModuleRunError) as ei:
            run_over(AsyncEcho(), _spec(StepKind.ASYNC_STATELESS), {"x": []})
        return ei.value

    err = asyncio.run(call())
    assert err.rule is RunRule.INSIDE_LOOP
    assert "run-inside-loop" in str(err)


def test_run_over_composes_align(monkeypatch):
    calls = _stub_align(monkeypatch, _rows(1.0, 2.0))
    hooks = RunHooks()
    streams = {"x": [object()]}
    outs = list(run_over(EchoStep(), _spec(StepKind.STATELESS), streams, hooks=hooks))
    assert calls == [(Ping, {"x": streams["x"]})]  # A1: align(in_type, streams)
    assert [o.ts for o in outs] == [1.0, 2.0]
    assert (hooks.ticks, hooks.emits) == (2, 2)


def test_run_over_mealy_starts_from_default_state(monkeypatch):
    _stub_align(monkeypatch, _rows(1.0, 2.0))
    m = Recorder()
    spec = _spec(StepKind.MEALY, state_type=Recorder.State)
    outs = list(run_over(m, spec, {"x": []}))
    assert m.seen == [0, 1]  # D16: run started from spec.state_type()
    assert [o.y for o in outs] == [0.0, 1.0]


def test_run_over_async_serial_default(monkeypatch):
    # D6: no max_inflight field -> DEFAULT_MAX_INFLIGHT = 1 (serial).
    _stub_align(monkeypatch, _rows(1.0, 2.0, 3.0))
    cur = peak = 0

    class NoField:
        async def step(self, i: Ping) -> Pong:
            nonlocal cur, peak
            cur += 1
            peak = max(peak, cur)
            await asyncio.sleep(0)  # pure reschedule point — not wall clock
            cur -= 1
            return Pong(y=i.x)

    outs = list(run_over(NoField(), _spec(StepKind.ASYNC_STATELESS), {"x": []}))
    assert len(outs) == 3
    assert peak == 1


def test_run_over_async_max_inflight_from_module_field(monkeypatch):
    _stub_align(monkeypatch, _rows(1.0, 2.0, 3.0))
    cur = peak = 0

    class Wide:
        max_inflight = 3  # the sketch's Captioner pattern: a config field

        async def step(self, i: Ping) -> Pong:
            nonlocal cur, peak
            cur += 1
            peak = max(peak, cur)
            await asyncio.sleep(0)  # pure reschedule point — not wall clock
            cur -= 1
            return Pong(y=i.x)

    outs = list(run_over(Wide(), _spec(StepKind.ASYNC_STATELESS), {"x": []}))
    assert len(outs) == 3
    assert peak == 3


# ── the memory2 boundary (index reconciliation; t5-align.md §11.1) ───────────
# A memory2 Stream yields Observation envelopes (envelope carries ts; payload
# behind .data). run_over unwraps to payloads at the over() boundary so row
# fields receive payloads (T5 D2) and the payload ts stays the ONE authority.


class _FakeObservation:
    """Shape-match of memory2's Observation: envelope ts + data payload."""

    def __init__(self, ts: float, data: object) -> None:
        self.ts = ts
        self.data = data


def _install_fake_memory2_stream(monkeypatch) -> type:
    """A sys.modules stub matching dimos.memory2.stream's Stream shape."""

    class Stream:
        def __init__(self, observations) -> None:
            self._observations = list(observations)

        def __iter__(self):
            return iter(self._observations)

    mod = types.ModuleType("dimos.memory2.stream")
    mod.Stream = Stream  # type: ignore[attr-defined]
    monkeypatch.setitem(sys.modules, "dimos.memory2.stream", mod)
    return Stream


def test_run_over_unwraps_memory2_stream(monkeypatch):
    stream_cls = _install_fake_memory2_stream(monkeypatch)
    calls = _stub_align(monkeypatch, _rows(1.0))
    payloads = _rows(10.0, 11.0)  # stamped payloads: ts survives the unwrap
    stream = stream_cls(_FakeObservation(ts=p.ts, data=p) for p in payloads)
    list(run_over(EchoStep(), _spec(StepKind.STATELESS), {"x": stream}))
    ((_, streams_seen),) = calls
    coerced = streams_seen["x"]
    assert coerced is not stream  # recognized and adapted, not passed verbatim
    items = list(coerced)
    assert [a is b for a, b in zip(items, payloads, strict=False)] == [
        True,
        True,
    ]  # payloads, unwrapped
    assert [i.ts for i in items] == [10.0, 11.0]  # payload ts intact — never lost
    assert list(coerced) == payloads  # re-iterable, like the Stream it wraps


def test_run_over_memory2_absent_is_passthrough(monkeypatch):
    # Recognition is sys.modules-based: without memory2 imported, nothing is
    # imported and every value passes verbatim (the plain-iterable contract).
    monkeypatch.delitem(sys.modules, "dimos.memory2.stream", raising=False)
    calls = _stub_align(monkeypatch, _rows(1.0))
    stream = [_rows(1.0)[0]]
    list(run_over(EchoStep(), _spec(StepKind.STATELESS), {"x": stream}))
    assert calls[0][1]["x"] is stream
    assert "dimos.memory2.stream" not in sys.modules  # coercion imported nothing


def test_run_over_unwraps_real_memory2_stream(monkeypatch):
    memory = pytest.importorskip("dimos.memory2.store.memory")
    calls = _stub_align(monkeypatch, _rows(1.0))
    with memory.MemoryStore() as store:
        stream = store.stream("x", Ping)
        for row in _rows(10.0, 11.0):
            stream.append(row, ts=row.ts)
        list(run_over(EchoStep(), _spec(StepKind.STATELESS), {"x": stream}))
        ((_, streams_seen),) = calls
        items = list(streams_seen["x"])
        assert all(isinstance(i, Ping) for i in items)  # payloads, not Observations
        assert [i.ts for i in items] == [10.0, 11.0]  # payload ts intact after unwrap


def test_hooks_invariant_step_kinds():
    # spec §9: at clean exhaustion, ticks == emits + skips + drops (step kinds).
    hooks = RunHooks()
    list(drive_stateless(EvenStep(), iter(_rows(1.0, 2.0, 3.0)), skips=True, hooks=hooks))
    assert hooks.ticks == hooks.emits + hooks.skips + hooks.drops

    class Flaky:
        async def step(self, i: Ping) -> Pong | None:
            if i.x == 2.0:
                raise ValueError("transient")
            return Pong(y=i.x) if i.x != 3.0 else None

    ahooks = RunHooks()
    list(
        drive_async(
            Flaky(), iter(_rows(1.0, 2.0, 3.0, 4.0)), max_inflight=2, skips=True, hooks=ahooks
        )
    )
    assert ahooks.ticks == ahooks.emits + ahooks.skips + ahooks.drops == 4
