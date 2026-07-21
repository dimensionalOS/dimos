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

"""T15 debug-recorder tests (spec: dimos/pure/tasks/t15-debug.md).

The names + docstrings pin the acceptance bar; the module-level fixtures use
landed machinery only (bundle definitions, module construction) so tests need
no robot and no graph — replay re-drives step over the recorded In rows.
"""

from pathlib import Path

import pytest

from dimos import pure as pm
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.pure import debugrec, drivers

# ── fixture modules (landed machinery only) ──────────────────────────────────


class Doubler(pm.PureModule):
    """Stateless: emits 2 * v every tick (deterministic replay subject)."""

    class In(pm.In):
        v: float = pm.tick()

    class Out(pm.Out):
        double: float | None = None

    def step(self, i: In) -> Out:
        return Doubler.Out(double=2.0 * i.v)


class Tripler(pm.PureModule):
    """Stateless variant: the modified module for the behavioral-diff replay bar."""

    class In(pm.In):
        v: float = pm.tick()

    class Out(pm.Out):
        double: float | None = None

    def step(self, i: In) -> Out:
        return Tripler.Out(double=3.0 * i.v)


class Counter(pm.PureModule):
    """Mealy: running count + sum — State snapshots feed the from_seq replay bar."""

    class State(pm.State):
        n: int = 0
        total: float = 0.0

    class In(pm.In):
        v: float = pm.tick()

    class Out(pm.Out):
        total: float | None = None

    def step(self, s: State, i: In) -> "tuple[Counter.State, Counter.Out]":
        s = s.replace(n=s.n + 1, total=s.total + i.v)
        return s, Counter.Out(total=s.total)


class Mapper(pm.PureModule):
    """Required-tf module: with no tf transforms, every tick drops tf-unresolvable:pose."""

    frame_id: str = "world"
    body_frame: str = "robot"

    class In(pm.In):
        scan: float = pm.tick()
        pose: Transform = pm.tf("{frame_id}", "{body_frame}")

    class Out(pm.Out):
        cloud: float | None = pm.contract(min_hz=1.0)

    def step(self, i: In) -> Out:
        return Mapper.Out(cloud=1.0)


class S:
    """Minimal stamped payload for hand-built streams."""

    def __init__(self, ts: float, v: float = 0.0) -> None:
        self.ts = ts
        self.v = v


class V:
    """A stamped, picklable numeric — the tick payload lands whole in the In field.

    A plain object (not a ``float`` subclass, so no ``__new__``): the arithmetic
    the fixture steps use (``2.0 * i.v``, ``total + i.v``) dispatches through the
    reflected operators, and ``float(i.v)`` reads the value.
    """

    def __init__(self, ts: float, val: float) -> None:
        self.ts = float(ts)
        self.val = float(val)

    def __rmul__(self, other: float) -> float:
        return other * self.val

    def __radd__(self, other: float) -> float:
        return other + self.val

    def __float__(self) -> float:
        return self.val


def _floats(n: int) -> list[V]:
    """A ``v`` tick stream of stamped floats, ts == value == index."""
    return [V(float(t), float(t)) for t in range(n)]


# ── env / toggle grammar ─────────────────────────────────────────────────────


def test_env_grammar_global_toggle() -> None:
    """DIMOS_PURE_DEBUG=1 parses to one catch-all decisions-only rule; "0"/"" parse empty."""
    assert debugrec.parse_debug("1") == (("", debugrec.Debug()),)
    assert debugrec.parse_debug("0") == ()
    assert debugrec.parse_debug("") == ()


def test_env_grammar_per_module_rules() -> None:
    """The spec line 'nav_stack/voxel_mapper2:rows,thin=5;planner:rows' parses per-path,
    with '/' normalized to the T13 '.' member-path separator."""
    cfg = debugrec.parse_debug("nav_stack/voxel_mapper2:rows,thin=5;planner:rows")
    assert debugrec.resolve_debug(cfg, "nav_stack.voxel_mapper2") == debugrec.Debug(
        rows=True, thin=5
    )
    assert debugrec.resolve_debug(cfg, "planner") == debugrec.Debug(rows=True)
    assert debugrec.resolve_debug(cfg, "unlisted") is None


def test_env_grammar_rejects_bad_token() -> None:
    """An unknown layer/option raises DebugError naming the token [debug-grammar]."""
    with pytest.raises(debugrec.DebugError, match=r"\[debug-grammar\]"):
        debugrec.parse_debug("planner:rowz")


def test_resolve_most_specific_prefix_wins() -> None:
    """over(debug=)/env layering: the longest matching path prefix supplies the rule."""
    cfg = (
        ("", debugrec.Debug()),
        ("nav_stack", debugrec.Debug(rows=True)),
        ("nav_stack.voxel_mapper2", debugrec.Debug(rows=True, thin=5)),
    )
    assert debugrec.resolve_debug(cfg, "nav_stack.voxel_mapper2") == debugrec.Debug(
        rows=True, thin=5
    )
    assert debugrec.resolve_debug(cfg, "nav_stack.planner") == debugrec.Debug(rows=True)
    assert debugrec.resolve_debug(cfg, "elsewhere") == debugrec.Debug()


def test_debug_thin_validated() -> None:
    """Debug(thin=0) is rejected at construction (landed: frozen-dataclass validation)."""
    with pytest.raises(ValueError, match=r"\[debug-grammar\]"):
        debugrec.Debug(thin=0)


# ── capture: decisions layer ─────────────────────────────────────────────────


def test_over_debug_true_records_decisions(tmp_path: Path) -> None:
    """m.over(..., debug=True) writes one TickDecision per tick attempt to
    <db>/<path>/decisions, keyed by (seq, tick_ts) — no wall clock anywhere."""
    db = tmp_path / "debug.db"
    out = list(Doubler().over(v=_floats(5), debug=pm.Debug(db=db)))
    assert [o.double for o in out] == [0.0, 2.0, 4.0, 6.0, 8.0]

    with debugrec.load(db) as run:
        assert run.modules() == ["doubler"]
        ticks = list(run.module("doubler").ticks())
    assert [t.seq for t in ticks] == [0, 1, 2, 3, 4]
    assert all(t.fired and t.emitted for t in ticks)
    # data-time keys only: tick_ts is the payload ts (== seq here), never epoch wall clock.
    assert [t.tick_ts for t in ticks] == [0.0, 1.0, 2.0, 3.0, 4.0]
    assert all(t.step_ms is not None and t.step_ms >= 0.0 for t in ticks)


def test_zero_rows_incident_diagnosable_from_summary(tmp_path: Path) -> None:
    """THE acceptance bar: a required-tf module whose tf never resolves runs to
    0 rows; run.summary() alone names the dropped field and reason
    (tf-unresolvable:pose, N/N) — no monkeypatching, no source diving."""
    db = tmp_path / "debug.db"
    out = list(Mapper().over(scan=_floats(6), tf=[], debug=pm.Debug(db=db)))
    assert out == []  # every tick dropped: 0 rows

    with debugrec.load(db) as run:
        summary = run.summary()
        md = run.module("mapper")
        drops = list(md.drops())
    assert "tf-unresolvable:pose" in summary
    assert "6/6" in summary
    assert len(drops) == 6
    assert all(t.drop_reason == "tf-unresolvable:pose" and not t.fired for t in drops)


def test_drops_filter_by_reason_prefix(tmp_path: Path) -> None:
    """mod.drops(reason='tf-unresolvable') matches 'tf-unresolvable:pose' records."""
    db = tmp_path / "debug.db"
    list(Mapper().over(scan=_floats(4), tf=[], debug=pm.Debug(db=db)))
    with debugrec.load(db) as run:
        md = run.module("mapper")
        assert len(list(md.drops(reason="tf-unresolvable"))) == 4
        assert list(md.drops(reason="missing-required")) == []


def test_disabled_capture_wires_nothing(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    """debug off (no arg, no env) ⇒ session_for returns None: on_decision stays None
    on the aligner and hooks carry no debug callables — zero tick-path cost."""
    monkeypatch.delenv("DIMOS_PURE_DEBUG", raising=False)
    m = Doubler()
    assert debugrec.session_for(m, debug=False) is None
    assert debugrec.session_for(m, debug=None) is None
    # A disabled run writes no db and still produces rows.
    db = tmp_path / "debug.db"
    out = list(m.over(v=_floats(3), debug=Path(db) and False))
    assert [o.double for o in out] == [0.0, 2.0, 4.0]
    assert not db.exists()


# ── capture: ring + threading contract ───────────────────────────────────────


def test_ring_overflow_drops_oldest_without_blocking() -> None:
    """Pushing capacity+k events keeps the newest `capacity`, sets dropped == k,
    and push never blocks on a full ring (drop-oldest, doctrine #1)."""
    ring = debugrec.DebugRing(capacity=4)

    def ev(seq: int) -> debugrec.DebugEvent:
        return debugrec.DebugEvent(
            debugrec.DebugEventKind.DECISION, "m", seq, float(seq), None, seq
        )

    for seq in range(10):  # 6 over capacity — never blocks
        ring.push(ev(seq))
    assert len(ring) == 4
    assert ring.dropped == 6
    drained = ring.drain()
    assert [e.seq for e in drained] == [6, 7, 8, 9]  # oldest evicted, newest kept


def test_over_inline_drain_paces_on_ticks_not_emits(tmp_path: Path) -> None:
    """Doctrine #2: over() drains inline every N tick ATTEMPTS, independent of emit
    rate. A slow-emitting run (attempts >> ring capacity, ~0 emits) must never drop
    a ring event — regression: the drain was keyed to out rows, so an all-drop run
    overflowed the ring and lost decisions (field report, pgo_voxel_mapper 4096/4460)."""
    n = debugrec.DEFAULT_RING_CAPACITY + 500  # exceed the ring: only inline draining saves it
    writer = debugrec.DebugWriter.open(tmp_path / "debug.db")
    session = debugrec.DebugSession("slow", debugrec.Debug(), writer, live=False)
    writer.register(session)
    try:
        for i in range(n):  # every tick drops (no fire, no emit) — tap_out_rows never runs
            session.on_decision(float(i), False, "tf-unresolvable:pose", {}, 0)
        session.close()
        assert session.ring.dropped == 0  # inline attempt-paced drains kept the ring bounded
    finally:
        writer.close()
    with debugrec.load(tmp_path / "debug.db") as run:
        assert len(list(run.module("slow").drops())) == n  # every attempt persisted, none lost


def test_tick_path_never_touches_store(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    """During over(debug=...) iteration, mem2 append happens only at the inline
    drain boundary (every OVER_DRAIN_EVERY ticks) and teardown — never inside
    on_decision/on_step/tap_* (asserted by instrumenting the writer)."""
    writes: list[int] = []
    orig = debugrec.DebugWriter.write

    def spy(self: debugrec.DebugWriter, events: list[debugrec.DebugEvent]) -> None:
        writes.append(len(events))
        orig(self, events)

    monkeypatch.setattr(debugrec.DebugWriter, "write", spy)
    db = tmp_path / "debug.db"
    gen = Doubler().over(v=_floats(5), debug=pm.Debug(rows=True, db=db))
    for _ in gen:
        # 5 ticks < OVER_DRAIN_EVERY (256): nothing reaches the store mid-iteration.
        assert writes == []
    # Teardown (generator exhaustion → tap_out_rows finally → close) drains once.
    assert writes and sum(writes) > 0


# ── capture: rows + state layers ─────────────────────────────────────────────


def test_rows_layer_captures_what_step_saw(tmp_path: Path) -> None:
    """With Debug(rows=True), in_rows() returns the post-alignment In rows and
    out_rows() the engine-stamped Out rows, seq-correlated with decisions."""
    db = tmp_path / "debug.db"
    list(Doubler().over(v=_floats(4), debug=pm.Debug(rows=True, db=db)))
    with debugrec.load(db) as run:
        md = run.module("doubler")
        in_rows = list(md.in_rows())
        out_rows = list(md.out_rows())
        seqs = [t.seq for t in md.ticks()]
    assert [r.seq for r in in_rows] == seqs == [0, 1, 2, 3]
    assert [float(r.row.v) for r in in_rows] == [0.0, 1.0, 2.0, 3.0]
    assert [r.row.double for r in out_rows] == [0.0, 2.0, 4.0, 6.0]
    assert [r.tick_ts for r in out_rows] == [0.0, 1.0, 2.0, 3.0]  # engine-stamped tick ts


def test_rows_thinning(tmp_path: Path) -> None:
    """Debug(rows=True, thin=5) keeps every 5th row while decisions stay complete."""
    db = tmp_path / "debug.db"
    list(Doubler().over(v=_floats(12), debug=pm.Debug(rows=True, thin=5, db=db)))
    with debugrec.load(db) as run:
        md = run.module("doubler")
        assert [r.seq for r in md.in_rows()] == [0, 5, 10]
        assert len(list(md.ticks())) == 12  # decisions stay complete


# ── replay + fixtures ────────────────────────────────────────────────────────


def test_replay_reproduces_recorded_out_stream(tmp_path: Path) -> None:
    """mod.replay() rebuilds the module from the config record, re-drives step
    over the recorded In rows, and reproduces the recorded out stream exactly
    (values and tick ts)."""
    db = tmp_path / "debug.db"
    recorded = list(Doubler().over(v=_floats(5), debug=pm.Debug(rows=True, db=db)))
    with debugrec.load(db) as run:
        replayed = list(run.module("doubler").replay())
    assert [(o.ts, o.double) for o in replayed] == [(r.ts, r.double) for r in recorded]


def test_replay_accepts_modified_module(tmp_path: Path) -> None:
    """mod.replay(Doubler-variant) drives the caller's instance over the same
    recorded inputs — the behavioral-diff loop from the spec."""
    db = tmp_path / "debug.db"
    list(Doubler().over(v=_floats(4), debug=pm.Debug(rows=True, db=db)))
    with debugrec.load(db) as run:
        replayed = list(run.module("doubler").replay(Tripler()))
    assert [o.double for o in replayed] == [0.0, 3.0, 6.0, 9.0]


def test_replay_from_seq_seeds_recorded_state(tmp_path: Path) -> None:
    """For a Mealy module, replay(from_seq=N) seeds the newest State snapshot at
    seq <= N and continues mid-run identically to the original tail; without a
    state layer it raises [debug-no-state]."""
    db = tmp_path / "debug.db"
    recorded = list(Counter().over(v=_floats(6), debug=pm.Debug(rows=True, state=True, db=db)))
    with debugrec.load(db) as run:
        tail = list(run.module("counter").replay(from_seq=3))
    # ticks 4,5 (seq 3's state seeds the continuation): totals 1+2+3+4=10, +5=15.
    assert [o.total for o in tail] == [r.total for r in recorded[4:]]

    db2 = tmp_path / "nostate.db"
    list(Counter().over(v=_floats(6), debug=pm.Debug(rows=True, state=False, db=db2)))
    with debugrec.load(db2) as run:
        with pytest.raises(debugrec.DebugError, match=r"\[debug-no-state\]"):
            list(run.module("counter").replay(from_seq=3))


def test_fixture_export_is_runnable(tmp_path: Path) -> None:
    """mod.fixture(seq, path=...) writes a self-contained pytest file that passes
    under pytest with no reference to the source debug.db."""
    import subprocess
    import sys

    db = tmp_path / "debug.db"
    list(Doubler().over(v=_floats(5), debug=pm.Debug(rows=True, db=db)))
    with debugrec.load(db) as run:
        fixture_path = run.module("doubler").fixture(seq=3, path=tmp_path / "test_regression_3.py")
    db.unlink()  # the fixture must not depend on the source db
    result = subprocess.run(
        [sys.executable, "-m", "pytest", fixture_path, "-q"],
        cwd=tmp_path,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr


# ── end-of-run report ────────────────────────────────────────────────────────


def test_silent_run_warning_fires_without_debug_enabled(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """ticks > 0 and rows == 0 at teardown logs the [debug-zero-rows] warning
    naming the top dropped field — unconditionally, debug capture off included."""
    # Unit: the copy names the top dropped field; a healthy run stays silent.
    msg = debugrec.silent_run_warning("M", ticks=6, rows=0, drops_by_field={"pose": 6})
    assert msg is not None and "[debug-zero-rows]" in msg and "pose" in msg
    assert debugrec.silent_run_warning("M", ticks=6, rows=6, drops_by_field={}) is None

    # Integration: a real zero-rows run warns even with NO debug capture wired.
    seen: list[str] = []

    class FakeLog:
        def warning(self, message: str, *a: object, **k: object) -> None:
            seen.append(message)

        def __getattr__(self, name: str) -> object:
            return lambda *a, **k: None

    monkeypatch.setattr(drivers, "_LOG", FakeLog())
    out = list(Mapper().over(scan=_floats(3), tf=[]))
    assert out == []
    assert any("[debug-zero-rows]" in s for s in seen)
