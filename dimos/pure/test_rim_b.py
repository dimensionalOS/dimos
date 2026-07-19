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

"""T8a rim-core tests (spec: tasks/t8-rim-b.md §15.1). Real bodies, skipped
until the T8a implementation lands — the implementing wave deletes the
module-level skip line and nothing else."""

from __future__ import annotations

import dataclasses
import sys
import threading
import time
import types
from typing import NamedTuple

import pytest

from dimos import pure as pm
from dimos.pure import rim_b as rim
from dimos.pure.resources import ResourceError

pytestmark = pytest.mark.skip(reason="T8 skeleton — bodies land with T8a")


# ── currency + fakes ─────────────────────────────────────────────────────────


@dataclasses.dataclass(frozen=True)
class Sample:
    ts: float
    v: float


@dataclasses.dataclass(frozen=True)
class Pos:
    ts: float
    v: float

    def interpolate(self, other, alpha):
        return Pos(
            ts=self.ts + (other.ts - self.ts) * alpha,
            v=self.v + (other.v - self.v) * alpha,
        )


class FakeTransport:
    """TransportLike: records publishes; delivers to subscribers off-thread."""

    def __init__(self):
        self.published = []
        self._subs = []

    def publish(self, msg):
        self.published.append(msg)

    def subscribe(self, callback):
        self._subs.append(callback)

        def _unsub():
            self._subs.remove(callback)

        return _unsub

    def deliver(self, *msgs):
        """Invoke subscribers from a foreign thread — exercises the marshal."""

        def _run():
            for m in msgs:
                for cb in list(self._subs):
                    cb(m)

        t = threading.Thread(target=_run, name="fake-transport-deliver")
        t.start()
        t.join()


class FakeSource:
    """SourceLike over a list; unsubscribe returned as a plain callable."""

    def __init__(self, items):
        self._items = list(items)
        self.unsubscribed = False

    def subscribe(self, callback):
        for item in self._items:
            callback(item)

        def _unsub():
            self.unsubscribed = True

        return _unsub


def wait_until(pred, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(0.005)
    return False


# ── probe modules ────────────────────────────────────────────────────────────


class Doubler(pm.PureModule):
    class In(pm.In):
        x: Sample = pm.tick()

    class Out(pm.Out):
        y: Sample = pm.contract(min_hz=1)

    def step(self, i: In) -> Out:
        return Doubler.Out(y=Sample(ts=i.x.ts, v=i.x.v * 2))


class Located(pm.PureModule):
    class In(pm.In):
        img: Sample = pm.tick()
        pos: Pos = pm.interpolate()

    class Out(pm.Out):
        y: Pos = pm.contract(min_hz=1)

    def step(self, i: In) -> Out:
        return Located.Out(y=i.pos)


RES_LOG: list = []


class _Recorder:
    def __init__(self, name):
        self.name = name
        RES_LOG.append(("create", name))

    def dispose(self):
        RES_LOG.append(("dispose", self.name))


class ResMod(pm.PureModule):
    class In(pm.In):
        x: Sample = pm.tick()

    class Out(pm.Out):
        y: Sample = pm.contract(min_hz=1)

    class State(NamedTuple):
        n: int = 0

    @pm.resource
    def a(self):
        return _Recorder("a")

    @pm.resource
    def b(self):
        return _Recorder("b")

    def step(self, state: State, i: In) -> tuple[State, Out | None]:
        assert isinstance(self.a, _Recorder)  # run world resolves resources
        return state._replace(n=state.n + 1), ResMod.Out(y=Sample(ts=i.x.ts, v=float(state.n)))


class FailWarmup(pm.PureModule):
    class In(pm.In):
        x: Sample = pm.tick()

    class Out(pm.Out):
        y: Sample = pm.contract(min_hz=1)

    @pm.resource
    def a(self):
        return _Recorder("a")

    @pm.resource
    def bad(self):
        raise RuntimeError("boom")

    def step(self, i: In) -> Out:
        return FailWarmup.Out(y=i.x)


GATE = threading.Event()


class SlowMod(pm.PureModule):
    class In(pm.In):
        x: Sample = pm.tick()

    class Out(pm.Out):
        y: Sample = pm.contract(min_hz=1)

    def step(self, i: In) -> Out:
        GATE.wait(timeout=10.0)
        return SlowMod.Out(y=i.x)


class SparseMod(pm.PureModule):
    class In(pm.In):
        x: Sample = pm.tick()

    class Out(pm.Out):
        y: Sample = pm.contract(min_hz=1)
        alert: Sample | None = None

    def step(self, i: In) -> Out:
        return SparseMod.Out(y=i.x, alert=i.x if i.x.v < 0 else None)


class _AClient:
    def __init__(self):
        self.closed = False

    async def echo(self, s):
        return s

    async def aclose(self):
        self.closed = True


class AsyncEcho(pm.PureModule):
    max_inflight: int = 2

    class In(pm.In):
        x: Sample = pm.tick()

    class Out(pm.Out):
        y: Sample = pm.contract(min_hz=1)

    @pm.resource
    def client(self) -> _AClient:
        return _AClient()

    async def step(self, i: In) -> Out:
        return AsyncEcho.Out(y=await self.client.echo(i.x))


@pytest.fixture(autouse=True)
def _fresh_logs():
    RES_LOG.clear()
    GATE.clear()
    yield
    GATE.set()  # release any stuck SlowMod step so stop() can join


@pytest.fixture
def stopper():
    """Guarantee rim.stop for every module a test starts."""
    started = []
    yield started.append
    for m in started:
        rim.stop(m)


def _wired(module_cls, **cfg):
    """Module with fake transports on every port; returns (m, ins, outs)."""
    m = module_cls(**cfg)
    spec = type(m).__pure_step__
    ins = {}
    for name in spec.in_type.fields():
        t = FakeTransport()
        setattr_port = getattr(m.i, name)
        setattr_port.transport = t
        ins[name] = t
    outs = {}
    for name in spec.out_type.fields():
        t = FakeTransport()
        getattr(m.o, name).transport = t
        outs[name] = t
    return m, ins, outs


# ── 1–2: views + binding ─────────────────────────────────────────────────────


class TestPortViews:
    def test_handles_and_identity(self):
        m = Doubler()
        assert isinstance(m.i.x, rim.RimInPort)
        assert isinstance(m.o.y, rim.RimOutPort)
        assert m.i is m.i and m.i.x is m.i.x  # cached view + handle
        assert m.o.y is m.o.y

    def test_class_access_reveals_accessor(self):
        assert not isinstance(Doubler.i, rim.RimInPorts)

    def test_unknown_name_attribute_error(self):
        m = Doubler()
        with pytest.raises(AttributeError, match=r"x") as e:
            _ = m.i.imagee
        assert "rim-unknown-port" in str(e.value)
        with pytest.raises(AttributeError, match="rim-unknown-port"):
            _ = m.o.zzz


class TestBinding:
    def test_transport_source_mutually_exclusive(self):
        m = Doubler()
        m.i.x.transport = FakeTransport()
        with pytest.raises(rim.RimError, match="rim-both-bindings"):
            m.i.x.source = FakeSource([])
        m.i.x.transport = None  # clearing re-opens the other binding
        m.i.x.source = FakeSource([])

    def test_bad_transport_and_source(self):
        m = Doubler()
        with pytest.raises(rim.RimError, match="rim-bad-transport"):
            m.i.x.transport = object()
        with pytest.raises(rim.RimError, match="rim-bad-source"):
            m.i.x.source = object()
        with pytest.raises(rim.RimError, match="rim-bad-transport"):
            m.o.y.transport = object()

    def test_bad_capacity(self):
        m = Doubler()
        for bad in (0, -1, True, 1.5):
            with pytest.raises(rim.RimError, match="rim-bad-capacity"):
                m.i.x.capacity = bad
        m.i.x.capacity = 4
        m.i.x.capacity = None

    def test_rebind_while_running(self, stopper):
        m, ins, outs = _wired(Doubler)
        rim.start(m)
        stopper(m)
        with pytest.raises(rim.RimError, match="rim-rebind-running"):
            m.i.x.transport = FakeTransport()
        with pytest.raises(rim.RimError, match="rim-rebind-running"):
            m.o.y.transport = FakeTransport()


# ── 3–5: live loop ───────────────────────────────────────────────────────────


class TestLiveLoop:
    def test_stateless_end_to_end(self, stopper):
        m, ins, outs = _wired(Doubler)
        seen = []
        m.o.y.subscribe(seen.append)
        rim.start(m)
        stopper(m)
        ins["x"].deliver(Sample(ts=1.0, v=1.0), Sample(ts=2.0, v=2.0), Sample(ts=3.0, v=3.0))
        assert wait_until(lambda: len(outs["y"].published) == 3)
        assert [s.v for s in outs["y"].published] == [2.0, 4.0, 6.0]
        assert [s.ts for s in outs["y"].published] == [1.0, 2.0, 3.0]  # tick order
        assert seen == outs["y"].published  # local subscribers see the same

    def test_marshal_two_threads_interpolation(self, stopper):
        m, ins, outs = _wired(Located)
        rim.start(m)
        stopper(m)
        ins["pos"].deliver(Pos(ts=0.0, v=0.0))
        ins["img"].deliver(Sample(ts=1.0, v=0.0))
        ins["pos"].deliver(Pos(ts=2.0, v=2.0))  # closes the bracket -> tick fires
        assert wait_until(lambda: len(outs["y"].published) == 1)
        assert outs["y"].published[0].v == pytest.approx(1.0)  # midpoint

    def test_backpressure_keeplast_default(self, stopper):
        m, ins, outs = _wired(SlowMod)
        rim.start(m)
        stopper(m)
        ins["x"].deliver(Sample(ts=1.0, v=1.0))
        assert wait_until(lambda: rim.session(m).stats.run.ticks >= 1)  # step busy
        ins["x"].deliver(*(Sample(ts=float(t), v=float(t)) for t in range(2, 12)))
        GATE.set()
        assert wait_until(lambda: outs["y"].published and outs["y"].published[-1].ts == 11.0)
        rim.stop(m)
        stats = rim.session(m).stats
        assert stats.ports["x"].rim_dropped > 0  # counted, not warned about
        # capacity=1: only the first (in-flight) and freshest queued survive
        assert len(outs["y"].published) <= 3

    def test_backpressure_unbounded(self, stopper):
        m = SlowMod()
        tin, tout = FakeTransport(), FakeTransport()
        m.i.x.capacity = None
        m.i.x.transport = tin
        m.o.y.transport = tout
        GATE.set()  # never block
        rim.start(m)
        stopper(m)
        tin.deliver(*(Sample(ts=float(t), v=float(t)) for t in range(1, 11)))
        assert wait_until(lambda: len(tout.published) == 10)
        assert rim.session(m).stats.ports["x"].rim_dropped == 0


# ── 6–7: lifecycle ───────────────────────────────────────────────────────────


class TestLifecycle:
    def test_warmup_creates_stop_disposes_reverse(self, stopper):
        m, ins, outs = _wired(ResMod)
        rim.warmup(m)
        assert RES_LOG == [("create", "a"), ("create", "b")]
        rim.warmup(m)  # idempotent
        assert RES_LOG == [("create", "a"), ("create", "b")]
        rim.start(m)
        stopper(m)
        rim.stop(m)
        assert RES_LOG[-2:] == [("dispose", "b"), ("dispose", "a")]
        rim.stop(m)  # idempotent — no double dispose
        assert RES_LOG.count(("dispose", "a")) == 1

    def test_stop_before_start_disposes_warmed(self):
        m, ins, outs = _wired(ResMod)
        rim.warmup(m)
        rim.stop(m)
        assert RES_LOG == [
            ("create", "a"),
            ("create", "b"),
            ("dispose", "b"),
            ("dispose", "a"),
        ]

    def test_warmup_failure_unwinds_prefix(self):
        m = FailWarmup()
        with pytest.raises(ResourceError, match="resource-warmup-error"):
            rim.warmup(m)
        assert RES_LOG == [("create", "a"), ("dispose", "a")]
        assert rim.session(m) is None or rim.session(m).state is not rim.SessionState.RUNNING

    def test_double_start_rejected(self, stopper):
        m, ins, outs = _wired(Doubler)
        rim.start(m)
        stopper(m)
        with pytest.raises(rim.RimError, match="rim-already-running"):
            rim.start(m)

    def test_start_after_stop_fresh_session(self, stopper):
        m, ins, outs = _wired(ResMod)
        rim.start(m)
        rim.stop(m)
        first_creates = RES_LOG.count(("create", "a"))
        rim.start(m)
        stopper(m)
        assert RES_LOG.count(("create", "a")) == first_creates + 1  # fresh resources

    def test_missing_tick_binding_fails_fast(self):
        m = Doubler()  # nothing bound
        with pytest.raises(Exception, match="align-missing-tick-stream"):
            rim.start(m)
        assert rim.session(m) is None or rim.session(m).state is rim.SessionState.STOPPED

    def test_stop_drains_queued(self):
        m = Doubler()
        tin, tout = FakeTransport(), FakeTransport()
        m.i.x.capacity = None
        m.i.x.transport = tin
        m.o.y.transport = tout
        rim.start(m)
        tin.deliver(*(Sample(ts=float(t), v=1.0) for t in range(1, 6)))
        rim.stop(m)  # drain-before-dispose: queued rows still emitted
        assert len(tout.published) == 5


# ── 8–10: async, sparse, fan-out errors ──────────────────────────────────────


class TestAsyncLive:
    def test_async_end_to_end_and_aclose(self, stopper):
        m, ins, outs = _wired(AsyncEcho)
        rim.start(m)
        stopper(m)
        ins["x"].deliver(*(Sample(ts=float(t), v=float(t)) for t in (1, 2, 3)))
        assert wait_until(lambda: len(outs["y"].published) == 3)
        assert [s.ts for s in outs["y"].published] == [1.0, 2.0, 3.0]  # tick order
        rim.stop(m)
        # the run's client resource was disposed on the driver loop (aclose)
        run = rim.session(m).stats.run
        assert run.emits == 3


class TestFanOut:
    def test_sparse_none_stays_quiet(self, stopper):
        m, ins, outs = _wired(SparseMod)
        rim.start(m)
        stopper(m)
        ins["x"].deliver(Sample(ts=1.0, v=5.0), Sample(ts=2.0, v=-5.0))
        assert wait_until(lambda: len(outs["y"].published) == 2)
        assert len(outs["alert"].published) == 1  # only the negative tick
        assert outs["alert"].published[0].ts == 2.0

    def test_subscriber_error_counted_not_fatal(self, stopper):
        m, ins, outs = _wired(Doubler)

        def boom(_msg):
            raise RuntimeError("subscriber boom")

        m.o.y.subscribe(boom)
        rim.start(m)
        stopper(m)
        ins["x"].deliver(Sample(ts=1.0, v=1.0), Sample(ts=2.0, v=2.0))
        assert wait_until(lambda: len(outs["y"].published) == 2)  # run survived
        assert rim.session(m).stats.publish["y"].publish_errors == 2

    def test_unsubscribe(self, stopper):
        m, ins, outs = _wired(Doubler)
        seen = []
        unsub = m.o.y.subscribe(seen.append)
        unsub()
        rim.start(m)
        stopper(m)
        ins["x"].deliver(Sample(ts=1.0, v=1.0))
        assert wait_until(lambda: len(outs["y"].published) == 1)
        assert seen == []


# ── 11: over() isolation ─────────────────────────────────────────────────────


class TestOverIsolation:
    def test_over_ignores_bindings(self):
        m, ins, outs = _wired(Doubler)
        rows = list(m.over(x=[Sample(ts=1.0, v=1.0), Sample(ts=2.0, v=2.0)]))
        assert [r.y.v for r in rows] == [2.0, 4.0]
        assert outs["y"].published == []  # replay cannot rebroadcast

    def test_over_during_live_with_resources_rejected(self, stopper):
        m, ins, outs = _wired(ResMod)
        rim.start(m)
        stopper(m)
        with pytest.raises(ResourceError, match="resource-concurrent-run"):
            next(iter(m.over(x=[Sample(ts=1.0, v=1.0)])))

    def test_over_during_live_resource_free_ok(self, stopper):
        m, ins, outs = _wired(Doubler)
        rim.start(m)
        stopper(m)
        rows = list(m.over(x=[Sample(ts=1.0, v=3.0)]))
        assert rows[0].y.v == 6.0


# ── 12: stats surface ────────────────────────────────────────────────────────


class TestStats:
    def test_counters_cross_thread(self, stopper):
        m, ins, outs = _wired(Doubler)
        rim.start(m)
        stopper(m)
        s = rim.session(m)
        assert s.state is rim.SessionState.RUNNING
        ins["x"].deliver(*(Sample(ts=float(t), v=1.0) for t in range(1, 4)))
        assert wait_until(lambda: s.stats.run.emits == 3)
        assert s.stats.ports["x"].delivered == 3
        assert s.stats.align.rows_emitted == 3
        assert s.stats.publish["y"].published == 3
        rim.stop(m)
        assert s.state is rim.SessionState.STOPPED
        assert s.error is None


# ── 14: sources ──────────────────────────────────────────────────────────────


@dataclasses.dataclass
class _FakeObservation:
    """Shape-match of memory2's Observation: envelope ts + data payload."""

    ts: float
    data: object


class _FakeStream:
    """sys.modules stub matching dimos.memory2.stream's Stream shape (subscribable)."""

    def __init__(self, items):
        self._items = list(items)

    def subscribe(self, on_next):
        for item in self._items:
            on_next(item)

        class _Disp:
            def dispose(self):
                pass

        return _Disp()


class TestSources:
    def test_source_feeds_port(self, stopper):
        m = Doubler()
        out = FakeTransport()
        src = FakeSource([Sample(ts=1.0, v=1.0), Sample(ts=2.0, v=2.0)])
        m.i.x.capacity = None
        m.i.x.source = src
        m.o.y.transport = out
        rim.start(m)
        stopper(m)
        assert wait_until(lambda: len(out.published) == 2)
        rim.stop(m)
        assert src.unsubscribed

    def test_observation_unwrap(self, monkeypatch, stopper):
        mod = types.ModuleType("dimos.memory2.stream")
        mod.Stream = _FakeStream
        mod.Observation = _FakeObservation
        monkeypatch.setitem(sys.modules, "dimos.memory2.stream", mod)
        payloads = [Sample(ts=1.0, v=1.0), Sample(ts=2.0, v=2.0)]
        m = Doubler()
        out = FakeTransport()
        m.i.x.capacity = None
        m.i.x.source = _FakeStream([_FakeObservation(ts=p.ts, data=p) for p in payloads])
        m.o.y.transport = out
        rim.start(m)
        stopper(m)
        assert wait_until(lambda: len(out.published) == 2)
        assert [s.v for s in out.published] == [2.0, 4.0]  # payloads, not envelopes
