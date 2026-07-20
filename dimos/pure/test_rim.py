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

"""T8a rim-core tests (spec: tasks/t8-rim.md §13.2, TR- cases).

Real bodies behind a module-level skip; the T8a implementation wave deletes
the ``pytestmark`` line below. FakeWire is the in-memory transport double
(spec §13.1): Publishable + Subscribable, with a foreign-thread delivery
helper simulating arbitrary transport dispatcher threads.
"""

from __future__ import annotations

from collections.abc import Callable, Iterator
import dataclasses
from pathlib import Path
import sys
import threading
import time
import types
from typing import Any, NamedTuple

import pytest

from dimos import pure as pm
from dimos.pure import rim
from dimos.pure.resources import ResourceError
from dimos.pure.rim import RimError, RimRule

WAIT_S = 5.0  # test-side wall clock only; the rim's data path reads no clocks


# ── payloads and wires (spec §13.1) ──────────────────────────────────────────


@dataclasses.dataclass(frozen=True)
class Ping:
    ts: float
    v: float


@dataclasses.dataclass(frozen=True)
class Bare:
    """Deliberately unstamped payload for the egress currency check."""

    v: float


@dataclasses.dataclass(frozen=True)
class Pos:
    """Interpolatable payload — exercises T5 interpolate through the live rings (G7)."""

    ts: float
    v: float

    def interpolate(self, other: Pos, alpha: float) -> Pos:
        return Pos(ts=self.ts + (other.ts - self.ts) * alpha, v=self.v + (other.v - self.v) * alpha)


class FakeWire:
    """In-memory Publishable + Subscribable; delivery inline or from a thread."""

    def __init__(self) -> None:
        self.sent: list[Any] = []
        self._subs: list[Callable[[Any], Any]] = []
        self._threads: list[threading.Thread] = []
        self._lock = threading.Lock()

    def publish(self, msg: Any) -> None:
        with self._lock:
            self.sent.append(msg)

    def subscribe(self, callback: Callable[[Any], Any]) -> Callable[[], None]:
        self._subs.append(callback)

        def unsubscribe() -> None:
            if callback in self._subs:
                self._subs.remove(callback)

        return unsubscribe

    @property
    def n_subs(self) -> int:
        return len(self._subs)

    def deliver(self, msg: Any, *, on_thread: bool = False) -> None:
        if not on_thread:
            for cb in list(self._subs):
                cb(msg)
            return
        t = threading.Thread(target=self.deliver, args=(msg,), daemon=True)
        self._threads.append(t)
        t.start()

    def join(self) -> None:
        for t in self._threads:
            t.join(WAIT_S)


def wait_for(cond: Callable[[], bool], timeout: float = WAIT_S) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if cond():
            return
        time.sleep(0.005)
    raise AssertionError("condition not reached before timeout")


# ── module zoo (defined at import: T1-T3 validate them for free) ─────────────

STEP_THREADS: list[int] = []  # test-only observability; cleared per test


class Echo(pm.PureModule):
    class In(pm.In):
        ping: Ping = pm.tick()

    class Out(pm.Out):
        pong: Ping = pm.contract(min_hz=1)

    def step(self, i: In) -> Out:
        STEP_THREADS.append(threading.get_ident())
        return Echo.Out(pong=Ping(ts=i.ts, v=i.ping.v * 2))


class Probe:
    """Order-recording disposable resource."""

    def __init__(self, log: list[str], name: str) -> None:
        self.log = log
        self.name = name
        self.log.append(f"create:{name}")

    def dispose(self) -> None:
        self.log.append(f"dispose:{self.name}")


RESOURCE_LOG: list[str] = []


class Stateful(pm.PureModule):
    class In(pm.In):
        ping: Ping = pm.tick()

    class Out(pm.Out):
        total: float = pm.contract(min_hz=1)

    class State(NamedTuple):
        acc: float = 0.0

    @pm.resource
    def a(self) -> Probe:
        return Probe(RESOURCE_LOG, "a")

    @pm.resource
    def b(self) -> Probe:
        return Probe(RESOURCE_LOG, "b")

    def step(self, state: State, i: In) -> tuple[State, Out | None]:
        state = state._replace(acc=state.acc + i.ping.v)
        return state, Stateful.Out(total=state.acc)


class AsyncEcho(pm.PureModule):
    max_inflight: int = 2

    class In(pm.In):
        ping: Ping = pm.tick()

    class Out(pm.Out):
        pong: Ping = pm.contract(min_hz=1)

    async def step(self, i: In) -> Out:
        return AsyncEcho.Out(pong=Ping(ts=i.ts, v=i.ping.v + 100))


class FoldEcho(pm.PureModule):
    def fold(self, rows: Iterator[Echo.In]) -> Iterator[Echo.Out]:
        for r in rows:
            yield Echo.Out(ts=r.ts, pong=Ping(ts=r.ts, v=r.ping.v))


class TwoIn(pm.PureModule):
    class In(pm.In):
        a: Ping = pm.tick()
        b: Ping = pm.latest()  # required secondary: holds ticks when silent

    class Out(pm.Out):
        both: float = pm.contract(min_hz=1)

    def step(self, i: In) -> Out:
        return TwoIn.Out(both=i.a.v + i.b.v)


class Sparse(pm.PureModule):
    class In(pm.In):
        ping: Ping = pm.tick()

    class Out(pm.Out):
        always: Ping = pm.contract(min_hz=1)
        rare: Ping | None = None

    def step(self, i: In) -> Out:
        rare = Ping(ts=i.ts, v=-1.0) if i.ping.v > 10 else None
        return Sparse.Out(always=Ping(ts=i.ts, v=i.ping.v), rare=rare)


class BareOut(pm.PureModule):
    class In(pm.In):
        ping: Ping = pm.tick()

    class Out(pm.Out):
        naked: Bare = pm.contract(min_hz=1)

    def step(self, i: In) -> Out:
        return BareOut.Out(naked=Bare(v=i.ping.v))


class Located(pm.PureModule):
    """Two-input: tick img + interpolated pos — the live-interpolation marshal (G7)."""

    class In(pm.In):
        img: Ping = pm.tick()
        pos: Pos = pm.interpolate()

    class Out(pm.Out):
        at: Pos = pm.contract(min_hz=1)

    def step(self, i: In) -> Out:
        return Located.Out(at=i.pos)


class FailWarmup(pm.PureModule):
    """One good resource then a failing factory — warmup unwinds the prefix (G3)."""

    class In(pm.In):
        ping: Ping = pm.tick()

    class Out(pm.Out):
        pong: Ping = pm.contract(min_hz=1)

    @pm.resource
    def a(self) -> Probe:
        return Probe(RESOURCE_LOG, "a")

    @pm.resource
    def bad(self) -> Probe:
        raise RuntimeError("boom")

    def step(self, i: In) -> Out:
        return FailWarmup.Out(pong=i.ping)


@pytest.fixture(autouse=True)
def _clear_logs() -> Iterator[None]:
    STEP_THREADS.clear()
    RESOURCE_LOG.clear()
    yield


@pytest.fixture
def live_echo() -> Iterator[tuple[Echo, FakeWire, FakeWire]]:
    """A started Echo wired FakeWire→FakeWire; always stopped (thread hygiene).

    Lossless ingress (capacity=None): these tests assert zero-loss plumbing
    (marshal order, subscriber fan-out) independent of the default KeepLast ring
    — the default-1 drop behavior has its own test (test_backpressure_keeplast_default)."""
    m = Echo()
    win, wout = FakeWire(), FakeWire()
    m.i.ping.capacity = None
    m.i.ping.transport = win
    m.o.pong.transport = wout
    rim.start_module(m)
    try:
        yield m, win, wout
    finally:
        rim.stop_module(m)
        win.join()


# ── ports and views ──────────────────────────────────────────────────────────


class TestPorts:
    def test_port_identity(self) -> None:
        m = Echo()
        assert m.i.ping is m.i.ping
        assert m.o.pong is m.o.pong
        assert rim.ports_of(m).i is rim.ports_of(m).i

    def test_unknown_port_name(self) -> None:
        m = Echo()
        with pytest.raises(AttributeError, match=r"ping.*\[rim-unknown-port\]"):
            _ = m.i.pingg
        with pytest.raises(AttributeError, match=r"\[rim-unknown-port\]"):
            _ = m.o.nope
        # getattr with default still works (AttributeError subclass contract)
        assert getattr(m.i, "pingg", None) is None

    def test_views_all_shapes(self) -> None:
        # all four step shapes get runtime views (fold reuses Echo's bundles)
        assert Echo().o.pong is not None
        assert Stateful().o.total is not None
        assert AsyncEcho().o.pong is not None
        assert FoldEcho().i.ping is not None and FoldEcho().o.pong is not None

    def test_mutual_exclusion(self) -> None:
        m = Echo()
        m.i.ping.transport = FakeWire()
        with pytest.raises(RimError, match=r"\[rim-port-conflict\]") as ei:
            m.i.ping.source = [Ping(1.0, 1.0)]
        assert ei.value.rim_rule is RimRule.PORT_CONFLICT
        m2 = Echo()
        m2.i.ping.source = [Ping(1.0, 1.0)]
        with pytest.raises(RimError, match=r"\[rim-port-conflict\]"):
            m2.i.ping.transport = FakeWire()

    def test_unbind_via_none(self) -> None:
        m = Echo()
        m.i.ping.transport = FakeWire()
        m.i.ping.transport = None
        m.i.ping.source = [Ping(1.0, 1.0)]  # now legal

    def test_live_rebind_rejected(self, live_echo: tuple[Echo, FakeWire, FakeWire]) -> None:
        m, _, _ = live_echo
        with pytest.raises(RimError, match=r"\[rim-live-rebind\]"):
            m.i.ping.transport = FakeWire()
        with pytest.raises(RimError, match=r"\[rim-live-rebind\]"):
            m.o.pong.transport = FakeWire()

    def test_not_subscribable(self) -> None:
        m = Echo()
        with pytest.raises(RimError, match=r"\[rim-not-subscribable\]"):
            m.i.ping.transport = object()

    def test_not_publishable(self) -> None:
        m = Echo()
        with pytest.raises(RimError, match=r"\[rim-not-publishable\]"):
            m.o.pong.transport = object()

    def test_frames_raises_until_t11(self) -> None:
        m = Echo()
        with pytest.raises(Exception):
            _ = m.o.pong.frames


# ── marshal, ordering, backpressure ──────────────────────────────────────────


class TestIngress:
    def test_marshal_single_thread(self, live_echo: tuple[Echo, FakeWire, FakeWire]) -> None:
        m, win, wout = live_echo
        for i in range(1, 6):
            win.deliver(Ping(ts=float(i), v=float(i)), on_thread=True)
            win.join()  # sequence deliveries deterministically, threads still foreign
        wait_for(lambda: len(wout.sent) == 5)
        assert len(set(STEP_THREADS)) == 1, "all steps on ONE session thread"
        assert STEP_THREADS[0] != threading.get_ident()

    def test_row_order_determinism(self, live_echo: tuple[Echo, FakeWire, FakeWire]) -> None:
        m, win, wout = live_echo
        for i in range(1, 8):
            win.deliver(Ping(ts=float(i), v=float(i)))
        wait_for(lambda: len(wout.sent) == 7)
        assert [p.ts for p in wout.sent] == [float(i) for i in range(1, 8)]
        assert [p.v for p in wout.sent] == [float(i) * 2 for i in range(1, 8)]

    def test_backpressure_keeplast_default(self) -> None:
        m = Echo()
        win, wout = FakeWire(), FakeWire()
        m.i.ping.transport = win  # capacity defaults to 1 (KeepLast, AD1)
        m.o.pong.transport = wout
        rim.start_module(m)
        try:
            # deliver a burst; drop-oldest keeps the newest, drops accounted:
            for i in range(1, 51):
                win.deliver(Ping(ts=float(i), v=float(i)))
            wait_for(lambda: rim.stats(m).ingress["ping"].received == 50)
            wait_for(
                lambda: rim.stats(m).hooks.ticks + rim.stats(m).ingress["ping"].dropped_overflow
                >= 50
            )
            s = rim.stats(m)
            assert s.ingress["ping"].received == 50
            assert s.ingress["ping"].dropped_overflow > 0  # counted, never warned
            # newest survives: the last published row is ts=50
            wait_for(lambda: bool(wout.sent) and wout.sent[-1].ts == 50.0)
        finally:
            rim.stop_module(m)

    def test_capacity_validation(self) -> None:
        m = Echo()
        for bad in (0, -1, True, 1.5):
            with pytest.raises(RimError, match=r"\[rim-bad-capacity\]") as ei:
                m.i.ping.capacity = bad  # type: ignore[assignment]
            assert ei.value.rim_rule is RimRule.BAD_CAPACITY
        m.i.ping.capacity = 4  # int >= 1 ok
        m.i.ping.capacity = None  # unbounded ok

    def test_capacity_unbounded_lossless(self) -> None:
        m = Echo()
        win, wout = FakeWire(), FakeWire()
        m.i.ping.capacity = None  # lossless: recorder-style, no drops (AD1)
        m.i.ping.transport = win
        m.o.pong.transport = wout
        rim.start_module(m)
        try:
            for i in range(1, 11):
                win.deliver(Ping(ts=float(i), v=float(i)))
            wait_for(lambda: len(wout.sent) == 10)
            assert rim.stats(m).ingress["ping"].dropped_overflow == 0
        finally:
            rim.stop_module(m)

    def test_live_interpolation_marshal(self) -> None:
        """Two foreign threads feed img+pos; T5 interpolate runs through the rings (G7)."""
        m = Located()
        wimg, wpos, wout = FakeWire(), FakeWire(), FakeWire()
        m.i.img.capacity = None
        m.i.pos.capacity = None
        m.i.img.transport = wimg
        m.i.pos.transport = wpos
        m.o.at.transport = wout
        rim.start_module(m)
        try:
            wpos.deliver(Pos(ts=0.0, v=0.0), on_thread=True)
            wimg.deliver(Ping(ts=1.0, v=0.0), on_thread=True)
            wpos.deliver(Pos(ts=2.0, v=2.0), on_thread=True)  # brackets tick 1.0
            wimg.join()
            wpos.join()
            wait_for(lambda: len(wout.sent) == 1)
            assert wout.sent[0].v == pytest.approx(1.0)  # midpoint interpolation
        finally:
            rim.stop_module(m)

    def test_hold_semantics_silent_secondary(self) -> None:
        m = TwoIn()
        wa, wb, wout = FakeWire(), FakeWire(), FakeWire()
        m.i.a.capacity = None  # lossless: assert the hold, not the ring
        m.i.b.capacity = None  # b's 0.5/5.0 burst must both land to bracket tick 1.0
        m.i.a.transport = wa
        m.i.b.transport = wb
        m.o.both.transport = wout
        rim.start_module(m)
        try:
            wa.deliver(Ping(ts=1.0, v=1.0))
            # required secondary silent → the tick HOLDS (spec §5): no rows,
            # held gauge shows the parked tick.
            wait_for(lambda: rim.stats(m).held_tick_ts == 1.0)
            assert wout.sent == []
            wb.deliver(Ping(ts=0.5, v=10.0))
            wb.deliver(Ping(ts=5.0, v=99.0))  # frontier passes 1.0 → tick fires
            wait_for(lambda: len(wout.sent) == 1)
            assert wout.sent[0] == 11.0  # latest at tick 1.0 is b@0.5
        finally:
            rim.stop_module(m)


# ── lifecycle ────────────────────────────────────────────────────────────────


class TestLifecycle:
    def test_warmup_creates_sync_resources(self) -> None:
        m = Stateful()
        m.i.ping.source = [Ping(1.0, 1.0)]
        assert RESOURCE_LOG == []
        rim.warmup_module(m)
        try:
            assert RESOURCE_LOG == ["create:a", "create:b"]
        finally:
            rim.stop_module(m)
        assert RESOURCE_LOG == ["create:a", "create:b", "dispose:b", "dispose:a"]

    def test_warmup_failure_unwinds_prefix(self) -> None:
        """A failing factory unwinds already-created resources in reverse (G3, spec §7.2)."""
        m = FailWarmup()
        m.i.ping.source = [Ping(1.0, 1.0)]
        with pytest.raises(ResourceError, match=r"\[resource-warmup-error\]"):
            rim.warmup_module(m)
        assert RESOURCE_LOG == ["create:a", "dispose:a"]  # 'a' created then unwound
        # no live session was left behind:
        assert rim.stats(m).state in ("new", "stopped")

    def test_concurrent_stop_joins_latch(self) -> None:
        """Second stop from another thread JOINS the latch, not returns early (G4, P15/P16)."""
        m = Stateful()
        win, wout = FakeWire(), FakeWire()
        m.i.ping.transport = win
        m.o.total.transport = wout
        rim.start_module(m)
        for i in range(1, 4):
            win.deliver(Ping(ts=float(i), v=1.0))
        barrier = threading.Barrier(2)

        def racer() -> None:
            barrier.wait()
            rim.stop_module(m)

        t = threading.Thread(target=racer)
        t.start()
        barrier.wait()
        rim.stop_module(m)
        t.join(WAIT_S)
        # both callers observe a fully-drained, disposed session (not an early return):
        assert not t.is_alive()
        assert rim.stats(m).state == "stopped"
        assert RESOURCE_LOG[-2:] == ["dispose:b", "dispose:a"]

    def test_flow_end_to_end(self, live_echo: tuple[Echo, FakeWire, FakeWire]) -> None:
        m, win, wout = live_echo
        got: list[Ping] = []
        unsub = m.o.pong.subscribe(got.append)
        win.deliver(Ping(ts=1.0, v=3.0))
        wait_for(lambda: len(wout.sent) == 1 and len(got) == 1)
        assert wout.sent[0].v == 6.0 and got[0].v == 6.0
        unsub()
        win.deliver(Ping(ts=2.0, v=4.0))
        wait_for(lambda: len(wout.sent) == 2)
        assert len(got) == 1  # unsubscribed

    def test_stop_drains_then_disposes(self) -> None:
        m = Stateful()
        win, wout = FakeWire(), FakeWire()
        m.i.ping.capacity = None  # lossless: assert the drain processes all 3
        m.i.ping.transport = win
        m.o.total.transport = wout
        rim.start_module(m)
        for i in range(1, 4):
            win.deliver(Ping(ts=float(i), v=1.0))
        rim.stop_module(m)
        # every delivery accepted before stop was processed (drain), THEN
        # resources disposed in reverse:
        assert len(wout.sent) == 3
        assert RESOURCE_LOG[-2:] == ["dispose:b", "dispose:a"]

    def test_stop_idempotent(self, live_echo: tuple[Echo, FakeWire, FakeWire]) -> None:
        m, _, _ = live_echo
        rim.stop_module(m)
        rim.stop_module(m)  # second call returns immediately (spec §7.3 #1)

    def test_stop_unsubscribes_ingestion(self, live_echo: tuple[Echo, FakeWire, FakeWire]) -> None:
        m, win, _ = live_echo
        assert win.n_subs == 1
        rim.stop_module(m)
        assert win.n_subs == 0

    def test_stop_before_start_disposes_warmup(self) -> None:
        m = Stateful()
        m.i.ping.source = [Ping(1.0, 1.0)]
        rim.warmup_module(m)
        rim.stop_module(m)  # no session thread ever existed (spec §7.3 #8)
        assert RESOURCE_LOG == ["create:a", "create:b", "dispose:b", "dispose:a"]

    def test_restart_fresh_session(self) -> None:
        m = Stateful()
        win, wout = FakeWire(), FakeWire()
        m.i.ping.transport = win
        m.o.total.transport = wout
        rim.start_module(m)
        win.deliver(Ping(ts=1.0, v=5.0))
        wait_for(lambda: len(wout.sent) == 1)
        rim.stop_module(m)
        rim.start_module(m)  # fresh hooks/resources/queues; bindings persist (D8)
        try:
            win.deliver(Ping(ts=2.0, v=1.0))
            wait_for(lambda: len(wout.sent) == 2)
            assert wout.sent[-1] == 1.0  # fresh State: acc restarted, not 6.0
            assert RESOURCE_LOG.count("create:a") == 2
            assert rim.stats(m).hooks.ticks == 1  # fresh counters
        finally:
            rim.stop_module(m)

    def test_already_live(self, live_echo: tuple[Echo, FakeWire, FakeWire]) -> None:
        m, _, _ = live_echo
        with pytest.raises(RimError, match=r"\[rim-already-live\]"):
            rim.start_module(m)

    def test_start_auto_warms(self) -> None:
        m = Stateful()
        m.i.ping.source = [Ping(1.0, 1.0)]
        rim.start_module(m)  # D4: no explicit warmup()
        try:
            assert "create:a" in RESOURCE_LOG
        finally:
            rim.stop_module(m)

    def test_lifecycle_via_module_methods(self) -> None:
        """Seam S4: m.warmup()/m.start()/m.stop() delegate to the rim."""
        m = Echo()
        win, wout = FakeWire(), FakeWire()
        m.i.ping.transport = win
        m.o.pong.transport = wout
        m.warmup()
        m.start()
        try:
            win.deliver(Ping(ts=1.0, v=1.0))
            wait_for(lambda: len(wout.sent) == 1)
        finally:
            m.stop()
        assert win.n_subs == 0

    def test_missing_tick_wiring_fails_at_start(self) -> None:
        m = Echo()  # nothing bound
        with pytest.raises(pm.AlignmentError, match=r"\[align-missing-tick-stream\]"):
            rim.start_module(m)


# ── egress ───────────────────────────────────────────────────────────────────


class TestEgress:
    def test_sparse_none_not_published(self) -> None:
        m = Sparse()
        win, walways, wrare = FakeWire(), FakeWire(), FakeWire()
        m.i.ping.capacity = None  # lossless: both ticks must land to check sparse egress
        m.i.ping.transport = win
        m.o.always.transport = walways
        m.o.rare.transport = wrare
        rim.start_module(m)
        try:
            win.deliver(Ping(ts=1.0, v=1.0))
            win.deliver(Ping(ts=2.0, v=99.0))
            wait_for(lambda: len(walways.sent) == 2)
            assert len(wrare.sent) == 1  # only the v>10 tick published rare
        finally:
            rim.stop_module(m)

    def test_bare_value_currency(self, live_echo: tuple[Echo, FakeWire, FakeWire]) -> None:
        m, win, wout = live_echo
        win.deliver(Ping(ts=1.0, v=1.0))
        wait_for(lambda: len(wout.sent) == 1)
        assert isinstance(wout.sent[0], Ping)  # the field VALUE, no envelope (P31)

    def test_unstamped_out_error(self) -> None:
        m = BareOut()
        win, wout = FakeWire(), FakeWire()
        m.i.ping.transport = win
        m.o.naked.transport = wout
        rim.start_module(m)
        try:
            win.deliver(Ping(ts=1.0, v=1.0))
            wait_for(lambda: rim.stats(m).error is not None)
            err = rim.stats(m).error
            assert isinstance(err, RimError) and err.rim_rule is RimRule.UNSTAMPED_OUT
            assert "naked" in str(err)
            assert wout.sent == []
        finally:
            rim.stop_module(m)

    def test_local_only_port_skips_stamp_check(self) -> None:
        m = BareOut()
        win = FakeWire()
        m.i.ping.transport = win
        got: list[Bare] = []
        m.o.naked.subscribe(got.append)  # no transport bound: plain values OK
        rim.start_module(m)
        try:
            win.deliver(Ping(ts=1.0, v=7.0))
            wait_for(lambda: len(got) == 1)
            assert got[0].v == 7.0
        finally:
            rim.stop_module(m)

    def test_subscriber_exception_counted(self, live_echo: tuple[Echo, FakeWire, FakeWire]) -> None:
        m, win, wout = live_echo

        def boom(_: Ping) -> None:
            raise ValueError("dashboard broke")

        m.o.pong.subscribe(boom)
        win.deliver(Ping(ts=1.0, v=1.0))
        win.deliver(Ping(ts=2.0, v=2.0))
        wait_for(lambda: len(wout.sent) == 2)  # data plane survives (spec §6.3)
        assert rim.stats(m).egress_errors == 2


# ── async modules ────────────────────────────────────────────────────────────


class TestAsync:
    def test_async_module_live(self) -> None:
        m = AsyncEcho()
        win, wout = FakeWire(), FakeWire()
        m.i.ping.capacity = None  # lossless: assert all 5 flow through the live async window
        m.i.ping.transport = win
        m.o.pong.transport = wout
        rim.start_module(m)
        try:
            for i in range(1, 6):
                win.deliver(Ping(ts=float(i), v=float(i)))
            wait_for(lambda: len(wout.sent) == 5)
            assert [p.v for p in wout.sent] == [101.0, 102.0, 103.0, 104.0, 105.0]
        finally:
            rim.stop_module(m)

    def test_async_bad_max_inflight_at_start(self) -> None:
        class BadInflight(pm.PureModule):
            max_inflight: int = 0

            class In(pm.In):
                ping: Ping = pm.tick()

            class Out(pm.Out):
                pong: Ping = pm.contract(min_hz=1)

            async def step(self, i: In) -> Out:
                return BadInflight.Out(pong=i.ping)

        m = BadInflight()
        m.i.ping.transport = FakeWire()
        m.o.pong.transport = FakeWire()
        with pytest.raises(pm.PureModuleRunError, match=r"\[run-bad-max-inflight\]"):
            rim.start_module(m)  # eagerly, on the caller thread (spec §6.1.3)


# ── sources, isolation, stats, transformer ───────────────────────────────────


class TestSourcesAndInterop:
    def test_iterable_source_pump(self) -> None:
        m = Echo()
        wout = FakeWire()
        m.i.ping.capacity = None  # lossless: every pumped item processed (D9/AD1)
        m.i.ping.source = [Ping(ts=float(i), v=float(i)) for i in range(1, 4)]
        m.o.pong.transport = wout
        rim.start_module(m)
        try:
            wait_for(lambda: len(wout.sent) == 3)  # finite source drains fully (D9)
        finally:
            rim.stop_module(m)

    def test_out_port_as_source(self) -> None:
        """m2.i.x.source = m1.o.y — in-process pure composition (spec §4.4)."""
        m1, m2 = Echo(), Echo()
        win, wout = FakeWire(), FakeWire()
        m1.i.ping.transport = win
        m2.i.ping.source = m1.o.pong
        m2.o.pong.transport = wout
        rim.start_module(m2)
        rim.start_module(m1)
        try:
            win.deliver(Ping(ts=1.0, v=1.0))
            wait_for(lambda: len(wout.sent) == 1)
            assert wout.sent[0].v == 4.0  # doubled twice
        finally:
            rim.stop_module(m1)
            rim.stop_module(m2)

    def test_over_isolation(self) -> None:
        """Bound wires observe NOTHING under over() (spec §4.3, structural)."""
        m = Echo()
        win, wout = FakeWire(), FakeWire()
        m.i.ping.transport = win
        m.o.pong.transport = wout
        rows = list(m.over(ping=[Ping(ts=1.0, v=1.0), Ping(ts=2.0, v=2.0)]))
        assert [r.pong.v for r in rows] == [2.0, 4.0]
        assert wout.sent == [] and win.n_subs == 0

    def test_over_during_live_with_resources_rejected(self) -> None:
        """D19 (G6): a resource-bearing module rejects over() while live — T7's guard."""
        m = Stateful()  # has @resource a, b
        win, wout = FakeWire(), FakeWire()
        m.i.ping.transport = win
        m.o.total.transport = wout
        rim.start_module(m)
        try:
            with pytest.raises(ResourceError, match=r"\[resource-concurrent-run\]"):
                next(iter(m.over(ping=[Ping(ts=1.0, v=1.0)])))
        finally:
            rim.stop_module(m)

    def test_over_during_live_resource_free_ok(self) -> None:
        """D19 (G6): a resource-free module may over() concurrently — harmless, disjoint."""
        m = Echo()  # no resources
        win, wout = FakeWire(), FakeWire()
        m.i.ping.transport = win
        m.o.pong.transport = wout
        rim.start_module(m)
        try:
            rows = list(m.over(ping=[Ping(ts=1.0, v=3.0)]))
            assert rows[0].pong.v == 6.0
            assert wout.sent == []  # replay never rebroadcasts on the live wire
        finally:
            rim.stop_module(m)

    def test_stats_surface(self, live_echo: tuple[Echo, FakeWire, FakeWire]) -> None:
        m, win, wout = live_echo
        win.deliver(Ping(ts=1.0, v=1.0))
        wait_for(lambda: len(wout.sent) == 1)
        s = rim.stats(m)
        assert s.state == "running"
        assert s.hooks.ticks == 1 and s.hooks.emits == 1
        assert s.align is not None and s.align.rows_emitted == 1
        assert s.ingress["ping"].received == 1
        assert s.published["pong"] == 1
        assert s.error is None

    def test_rim_never_imports_core(self) -> None:
        src = Path(rim.__file__).read_text()
        for forbidden in ("dimos.core", "dimos.protocol", "dimos.memory2"):
            assert forbidden not in src, f"rim core must not import {forbidden}"


@dataclasses.dataclass
class FakeObs:
    """Duck memory2 Observation: ts + data + derive (spec §10)."""

    ts: float
    data: Any
    tag: str = ""

    def derive(self, *, data: Any, **overrides: Any) -> FakeObs:
        return FakeObs(
            ts=overrides.get("ts", self.ts), data=data, tag=overrides.get("tag", self.tag)
        )


class TestTransformer:
    def test_equivalence_with_over(self) -> None:
        pings = [Ping(ts=float(i), v=float(i)) for i in range(1, 5)]
        expected = [r.pong.v for r in Echo().over(ping=pings)]
        obs_in = [FakeObs(ts=p.ts, data=p, tag="keep") for p in pings]
        obs_out = list(rim.transformer(Echo())(iter(obs_in)))
        assert [o.data.pong.v for o in obs_out] == expected
        assert [o.ts for o in obs_out] == [p.ts for p in pings]  # ts = row ts (D11)
        assert all(o.tag == "keep" for o in obs_out)  # envelope fields carry over

    def test_multi_input_raises(self) -> None:
        with pytest.raises(RimError, match=r"\[rim-transform-shape\]") as ei:
            rim.transformer(TwoIn())
        assert ei.value.rim_rule is RimRule.TRANSFORM_SHAPE


class _FakeStream:
    """sys.modules stub matching dimos.memory2.stream's Stream shape (subscribable)."""

    def __init__(self, items: list[Any]) -> None:
        self._items = list(items)

    def subscribe(self, on_next: Callable[[Any], Any]) -> Any:
        for item in self._items:
            on_next(item)

        class _Disp:
            def dispose(self) -> None:
                pass

        return _Disp()


class TestSources:
    def test_observation_unwrap(self, monkeypatch: pytest.MonkeyPatch) -> None:
        """A memory2-shaped source is unwrapped to obs.data at ingestion (G7, spec §10)."""
        mod = types.ModuleType("dimos.memory2.stream")
        mod.Stream = _FakeStream  # type: ignore[attr-defined]
        mod.Observation = FakeObs  # type: ignore[attr-defined]
        monkeypatch.setitem(sys.modules, "dimos.memory2.stream", mod)
        payloads = [Ping(ts=float(i), v=float(i)) for i in range(1, 3)]
        m = Echo()
        wout = FakeWire()
        m.i.ping.capacity = None
        m.i.ping.source = _FakeStream([FakeObs(ts=p.ts, data=p) for p in payloads])
        m.o.pong.transport = wout
        rim.start_module(m)
        try:
            wait_for(lambda: len(wout.sent) == 2)
            assert [p.v for p in wout.sent] == [2.0, 4.0]  # payloads unwrapped, doubled
        finally:
            rim.stop_module(m)
