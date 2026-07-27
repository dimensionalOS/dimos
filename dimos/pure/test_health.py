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

"""T9 health tests (spec: dimos/pure/tasks/t9-health.md, Implementation contract).

Names + docstrings pin the acceptance bar. Impl-dependent tests are
skip-gated ``"T9 impl pending"``; structure tests (flat wire row, entity
paths, stats accessors) run against the skeleton today. No robot, no graph:
the pacer's seams are callables/protocols, so every liveness scenario is a
fake snapshot + an injected clock.
"""

import dataclasses
import math
import typing
from typing import Any

import pytest

from dimos.pure import health
from dimos.pure.debugrec import TickDecision
from dimos.pure.health import (
    Health,
    HealthCounters,
    HealthPacer,
    HealthRecord,
    HealthStats,
)


def _impl(fn: Any) -> Any:
    """T9 landed: the impl-pending gate is now a pass-through."""
    return fn


# ── fakes (the pacer's seams are plain callables + protocol attrs) ───────────


@dataclasses.dataclass
class FakeSnapshot:
    """Structural ``SessionSnapshot``: the rim seam, hand-driven."""

    frontier_ts: float = float("-inf")
    inflight: int = 0
    rows_emitted: int = 0
    published: dict[str, int] = dataclasses.field(default_factory=dict)
    error: BaseException | None = None


class FakeDecisions:
    """Structural ``DecisionSource``: a pre-loaded batch per drain."""

    def __init__(self) -> None:
        self.batches: list[list[TickDecision]] = []
        self.dropped_count = 0

    def drain_into(self, observe: Any) -> int:
        batch = self.batches.pop(0) if self.batches else []
        for d in batch:
            observe(d)
        return len(batch)

    @property
    def dropped(self) -> int:
        return self.dropped_count


def _clock(times: list[float]) -> Any:
    """A wall clock that serves ``times`` then keeps returning the last one."""
    it = iter(times)
    last = [times[0]]

    def now() -> float:
        for t in it:
            last[0] = t
            return t
        return last[0]

    return now


def _fired(seq: int, ts: float, *, step_ms: float = 1.0, emitted: bool = True) -> TickDecision:
    return TickDecision(seq, ts, True, None, {}, step_ms, emitted, 0)


def _dropped(seq: int, ts: float, reason: str) -> TickDecision:
    return TickDecision(seq, ts, False, reason, {}, None, False, 0)


# ── the row shape (runs against the skeleton today) ──────────────────────────


def test_flat_wire_row_contains_no_containers() -> None:
    """Q2: the wire/rerun projection is flat primitives + short strings ONLY —
    no dict/list/set/tuple field anywhere on ``Health`` (the full drops dict
    lives on ``HealthRecord`` in storage)."""
    for name, hint in typing.get_type_hints(Health).items():
        origin = typing.get_origin(hint) or hint
        assert origin not in (dict, list, set, tuple), f"Health.{name} is a container: {hint}"


def test_storage_record_carries_the_full_drops_dict() -> None:
    """Q2's other half: ``HealthRecord`` = wire row + full ``drops_by_reason``."""
    hints = typing.get_type_hints(HealthRecord)
    assert hints["health"] is Health
    assert typing.get_origin(hints["drops_by_reason"]) is dict


def test_lag_is_ts_minus_frontier() -> None:
    """The two-clock signature: ``lag = ts - frontier_ts`` (inf before first tick)."""
    row = Health(
        ts=100.0,
        path="m",
        frontier_ts=97.5,
        seq=0,
        ticks_fired=0,
        ticks_dropped=0,
        rows_emitted=0,
        top_drop_reason="",
        debug_dropped=0,
        out_hz=0.0,
        step_ms_p50=math.nan,
        step_ms_p95=math.nan,
        inflight=0,
        contract_violation=None,
        resource_error=None,
    )
    assert row.lag == 2.5
    assert math.isinf(row._replace(frontier_ts=float("-inf")).lag)


def test_rerun_entity_path_convention() -> None:
    """number_problem.md: scalars land under ``plots/health/<path>/<field>``."""
    assert health.rerun_entity("nav.voxel_mapper2", "out_hz") == (
        "plots/health/nav.voxel_mapper2/out_hz"
    )


def test_health_stream_name() -> None:
    """Health rows live next to T15 decisions: the ``<path>.health`` stream."""
    assert health.health_stream("nav.voxel_mapper2") == "nav.voxel_mapper2.health"


def test_stats_drop_accessors() -> None:
    """``drops_total``/``top_drop_reason`` project the reason dict (flat row feed)."""
    stats = HealthStats(
        5, 3, 4, {"tf-unresolvable:pose": 2, "missing-required:goal": 1}, None, None, None
    )
    assert stats.drops_total == 3
    assert stats.top_drop_reason == "tf-unresolvable:pose"
    empty = HealthStats(0, 0, 0, {}, None, None, None)
    assert empty.drops_total == 0
    assert empty.top_drop_reason == ""


def test_pacer_rejects_negative_hz() -> None:
    """``health_hz`` is a cadence: negative is a construction-time error."""
    with pytest.raises(ValueError):
        HealthPacer("m", snapshot=FakeSnapshot, health_hz=-1.0)


# ── the aggregator (shared layer; by-construction guarantee) ─────────────────


@_impl
def test_counters_fold_decisions() -> None:
    """observe(): fired/dropped/emitted counts + reason dict + step durations."""
    c = HealthCounters()
    c.observe(_fired(0, 1.0, step_ms=2.0))
    c.observe(_fired(1, 2.0, step_ms=4.0, emitted=False))
    c.observe(_dropped(2, 3.0, "tf-unresolvable:pose"))
    c.observe(_dropped(3, 4.0, "tf-unresolvable:pose"))
    s = c.snapshot()
    assert (s.ticks_fired, s.ticks_dropped, s.rows_emitted) == (2, 2, 1)
    assert s.drops_by_reason == {"tf-unresolvable:pose": 2}
    assert s.step_ms_max == 4.0


@_impl
def test_posthoc_fold_equals_live_fold() -> None:
    """THE by-construction guarantee: the same decision sequence folded in
    pacer-sized batches (live) and all at once (post-hoc summary) yields
    identical ``HealthStats``."""
    seq = [
        _fired(0, 1.0, step_ms=1.0),
        _dropped(1, 2.0, "missing-required:goal"),
        _fired(2, 3.0, step_ms=3.0, emitted=False),
        _dropped(3, 4.0, "tf-unresolvable:pose"),
        _fired(4, 5.0, step_ms=2.0),
    ]
    live, posthoc = HealthCounters(), HealthCounters()
    for batch in (seq[:2], seq[2:4], seq[4:]):  # live: three pacer drains
        for d in batch:
            live.observe(d)
    for d in seq:  # post-hoc: one fold over the recorded stream
        posthoc.observe(d)
    assert live.snapshot() == posthoc.snapshot()


@_impl
def test_step_ring_is_a_recent_window() -> None:
    """Q4: p50/p95 cover the last ``STEP_RING_SIZE`` durations — old samples evict."""
    c = HealthCounters()
    for i in range(health.STEP_RING_SIZE):
        c.observe(_fired(i, float(i), step_ms=1.0))
    for i in range(health.STEP_RING_SIZE):
        c.observe(_fired(1000 + i, 1000.0 + i, step_ms=9.0))
    s = c.snapshot()
    assert s.step_ms_p50 == 9.0  # the 1.0 ms era has fully evicted
    assert s.step_ms_p95 == 9.0


# ── contracts (grace + violation copy) ───────────────────────────────────────


@_impl
def test_contract_grace_period() -> None:
    """No judgment before ``2/min_hz`` s — a 0.25 Hz contract is not violated 1 s in."""
    assert (
        health.evaluate_contract("map", 0.25, now=101.0, started=100.0, last_emit=100.0) is None
    )  # 1 s in: inside the 8 s grace
    assert (
        health.evaluate_contract("map", 0.25, now=107.9, started=100.0, last_emit=100.0) is None
    )  # still inside grace
    assert (
        health.evaluate_contract("map", 0.25, now=109.0, started=100.0, last_emit=100.0) is not None
    )  # grace over, gap 9 s > 4 s period


@_impl
def test_contract_violation_message_names_field_and_rates() -> None:
    """The doc's copy shape: ``"<field>: <observed> Hz < min_hz <min_hz>"``."""
    msg = health.evaluate_contract("global_map", 0.25, now=109.0, started=100.0, last_emit=100.0)
    assert msg is not None
    assert msg.startswith("global_map: ")
    assert "min_hz 0.25" in msg


@_impl
def test_contract_honored_is_none_after_grace() -> None:
    """A field emitting within its period judges None forever after."""
    assert health.evaluate_contract("map", 1.0, now=100.0, started=10.0, last_emit=99.5) is None


# ── the pacer (liveness semantics) ───────────────────────────────────────────


@_impl
def test_stalled_module_still_healths() -> None:
    """The stall signature: rows keep coming, ``ts`` advances, ``frontier_ts``
    freezes, ``lag`` grows — the pacer never depends on ticks."""
    snap = FakeSnapshot(frontier_ts=50.0, rows_emitted=7)
    rows: list[Health] = []
    pacer = HealthPacer(
        "m", snapshot=lambda: snap, publish=rows.append, clock=_clock([100.0, 101.0, 102.0])
    )
    first = pacer.sample()
    second = pacer.sample()
    assert [first, second] == rows
    assert second.ts > first.ts
    assert first.frontier_ts == second.frontier_ts == 50.0
    assert second.lag > first.lag
    assert second.seq == first.seq + 1
    assert pacer.latest is second


@_impl
def test_out_hz_is_a_wall_window_rate() -> None:
    """Rates are clock projections (taxonomy): Δemits over Δwall, 0.0 on row one."""
    snap = FakeSnapshot(frontier_ts=1.0)
    pacer = HealthPacer("m", snapshot=lambda: snap, clock=_clock([100.0, 102.0]))
    assert pacer.sample().out_hz == 0.0
    snap.rows_emitted = 10
    assert pacer.sample().out_hz == pytest.approx(5.0)  # 10 emits / 2 s


@_impl
def test_dead_session_reports_resource_error() -> None:
    """A dead-but-not-stopped session still healths, with ``resource_error`` set."""
    snap = FakeSnapshot(error=RuntimeError("cuda fell over"))
    pacer = HealthPacer("m", snapshot=lambda: snap, clock=_clock([100.0]))
    row = pacer.sample()
    assert row.resource_error is not None and "cuda fell over" in row.resource_error


@_impl
def test_pacer_drains_ring_through_the_counter_fold() -> None:
    """One thread, two jobs: sample() drains the T15 ring INTO the counters
    (drained decisions are the counters' only live feed) and surfaces the
    ring's overflow count as ``debug_dropped``."""
    decisions = FakeDecisions()
    decisions.batches = [[_fired(0, 1.0), _dropped(1, 2.0, "missing-required:goal")]]
    decisions.dropped_count = 3
    snap = FakeSnapshot(frontier_ts=2.0)
    pacer = HealthPacer("m", snapshot=lambda: snap, decisions=decisions, clock=_clock([100.0]))
    row = pacer.sample()
    assert (row.ticks_fired, row.ticks_dropped) == (1, 1)
    assert row.top_drop_reason == "missing-required:goal"
    assert row.debug_dropped == 3


@_impl
def test_flattened_projection_to_rerun_no_dict_entities() -> None:
    """to_rerun(): scalar entities under ``plots/health/<path>/<field>`` only —
    no per-reason series, no dict-valued components (unbounded-cardinality rule)."""
    snap = FakeSnapshot(frontier_ts=1.0)
    pacer = HealthPacer("nav.vm", snapshot=lambda: snap, clock=_clock([100.0]))
    pairs = pacer.sample().to_rerun()
    assert pairs, "to_rerun returned nothing"
    for entity, _component in pairs:
        assert entity.startswith("plots/health/nav.vm/")
        assert ":" not in entity  # drop-reason strings never mint entities


# ── storage (health next to decisions, same run db) ──────────────────────────


@_impl
def test_health_rows_land_in_run_db_next_to_decisions(tmp_path: Any) -> None:
    """The pacer records ``HealthRecord`` to ``<path>.health`` in the SAME
    debug.db (same writer), loadable next to the decisions stream."""
    from dimos.pure import debugrec

    writer: Any = debugrec.DebugWriter.open(tmp_path / "debug.db")
    try:
        recorded: list[tuple[HealthRecord, float]] = []

        def record(rec: HealthRecord, ts: float) -> None:
            recorded.append((rec, ts))
            writer.append_row(health.health_stream("m"), rec, ts)  # contract H7 seam

        snap = FakeSnapshot(frontier_ts=42.0)
        pacer = HealthPacer("m", snapshot=lambda: snap, record=record, clock=_clock([100.0]))
        row = pacer.sample()
        assert recorded and recorded[0][0].health is row
    finally:
        writer.close()

    from dimos.memory2.store.sqlite import SqliteStore

    store = SqliteStore(path=str(tmp_path / "debug.db"), must_exist=True)
    try:
        assert health.health_stream("m") in store.list_streams()
        obs = list(store.stream(health.health_stream("m"), HealthRecord))
        assert len(obs) == 1
        assert obs[0].data.health.frontier_ts == 42.0
        assert obs[0].ts == 42.0  # storage ts = frontier_ts: joins on data time (H7)
    finally:
        store.stop()


# ── the type swap (legacy bridge mechanism unchanged, payload swapped) ───────


@_impl
def test_placeholder_type_swap_keeps_autoconnect_key_legible() -> None:
    """T9 swaps the TYPE, not the mechanism: the shared topic keys on
    ``("health", Health)`` — a single named class, never ``("health", object)``."""
    from dimos.pure import legacy

    assert legacy._HEALTH_PAYLOAD is Health
    assert not hasattr(legacy, "HealthPlaceholder")  # fully replaced, not aliased
    key = (legacy._HEALTH_STREAM, legacy._HEALTH_PAYLOAD)
    assert key == ("health", Health)
    assert key[1].__name__ == "Health"  # the registry stays legible
