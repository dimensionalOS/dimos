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

"""Health: live operational liveness for pure modules (T9).

Spec: ``dimos/pure/tasks/t9-health.md`` (incl. its Implementation contract).
Two clocks per row: ``ts`` is the wall emission stamp (the ONLY sanctioned wall
clock — it lives in the pacer and nowhere else), ``frontier_ts`` is the
module's data-time watermark; joins against data happen on ``frontier_ts``,
never on the wall stamp. Counts and durations aggregate in ``HealthCounters``
(shared with T15's post-hoc ``ModuleDebug.summary()`` — the by-construction
guarantee that a debug capture summarizes to what health reported live);
rates are per-consumer clock projections and are computed HERE against wall
time only. The pacer is the ONE per-session off-tick-path thread: it emits
the health row and drains the T15 debug ring (T15's interim pacer retires
into it). Off under ``over()``/``--pure`` — eval-mode performance is a
post-hoc query over T15 records, not fake wall-clock rows.
"""

from __future__ import annotations

import collections
from collections.abc import Callable, Mapping
import math
import threading
import time
from typing import Any, Final, NamedTuple, Protocol

from dimos.pure.debugrec import TickDecision
from dimos.utils.logging_config import setup_logger

_LOG: Final = setup_logger()  # reachable in forkserver workers (bare getLogger is swallowed)

DEFAULT_HEALTH_HZ: Final[float] = 1.0
"""Health row cadence; the ``PureModuleConfig.health_hz`` default (Q3 resolution)."""

STEP_RING_SIZE: Final[int] = 256
"""Step-duration ring backing p50/p95 — bounded memory, honest recent window (Q4)."""

CONTRACT_GRACE_PERIODS: Final[float] = 2.0
"""Startup grace = ``CONTRACT_GRACE_PERIODS / min_hz`` seconds before the first
contract judgment (a 0.25 Hz contract cannot be violated 1 s in)."""

DRAIN_ONLY_PERIOD_S: Final[float] = 0.5
"""Pacer wake period when ``health_hz == 0`` but a debug ring still needs
draining (the retired T15 interim pacer's cadence, inherited)."""


def health_stream(path: str) -> str:
    """Run-db stream name for one module's health rows: ``<path>.health``."""
    return f"{path}.health"


def rerun_entity(path: str, field: str) -> str:
    """Rerun entity path for one flattened health scalar (number_problem.md)."""
    return f"plots/health/{path}/{field}"


def _nearest_rank(sorted_vals: list[float], pct: float) -> float:
    """Nearest-rank percentile of a pre-sorted list (matches the retired debugrec._pct)."""
    k = max(0, min(len(sorted_vals) - 1, round(pct / 100.0 * (len(sorted_vals) - 1))))
    return sorted_vals[k]


# ── the shared aggregate (clock-free: counts + durations only) ────────────────


class HealthStats(NamedTuple):
    """Clock-free aggregate of tick-decision events (the T9/T15 shared layer).

    Counts and durations only — a duration is not a timestamp. Rates
    (``out_hz``, data-time cadence) are per-consumer clock projections and are
    deliberately ABSENT: the live pacer computes them against wall time, the
    post-hoc reader against data time.
    """

    ticks_fired: int
    ticks_dropped: int
    rows_emitted: int
    drops_by_reason: dict[str, int]  # 'tf-unresolvable:pose' → n (full fidelity)
    step_ms_p50: float | None  # None when no fired tick carried a duration
    step_ms_p95: float | None  # p50/p95: nearest-rank over the recent ring window
    step_ms_max: float | None  # all-time running max (monotone, ring-independent)

    @property
    def drops_total(self) -> int:
        """Sum over ``drops_by_reason`` — equals ``ticks_dropped`` by construction."""
        return sum(self.drops_by_reason.values())

    @property
    def top_drop_reason(self) -> str:
        """The most frequent drop reason (``""`` when nothing dropped)."""
        if not self.drops_by_reason:
            return ""
        return max(self.drops_by_reason.items(), key=lambda kv: kv[1])[0]


class HealthCounters:
    """Incremental ``TickDecision`` aggregator: ``observe`` folds, ``snapshot`` projects.

    ONE definition of every counter, shared by the live pacer and T15's
    post-hoc ``ModuleDebug.summary()`` — folding the same decision sequence
    yields the same ``HealthStats`` by construction.

    Thread ownership: ``observe`` and ``snapshot`` run on the drain caller
    ONLY (live: the pacer thread, as it drains the T15 ring; post-hoc: the
    reader's caller). Never called on the tick path — the tick path pushes
    events into the T15 ring and nothing else. Plain attribute reads from
    other threads are lock-free and acceptably stale (ints/floats under the
    GIL); ``snapshot`` (which copies the reason dict) is drain-caller-only.
    """

    def __init__(self, ring_size: int = STEP_RING_SIZE) -> None:
        """Empty aggregate; ``ring_size`` bounds the step-duration window (Q4)."""
        self.ticks_fired = 0
        self.ticks_dropped = 0
        self.rows_emitted = 0
        self.drops_by_reason: dict[str, int] = {}
        self._step_ring: collections.deque[float] = collections.deque(maxlen=ring_size)
        self._step_max: float | None = None  # running all-time max (monotone)

    def observe(self, decision: TickDecision) -> None:
        """Fold one tick attempt: fired/dropped counts, drop reason, step duration.

        Not fired → ``ticks_dropped`` + ``drops_by_reason[decision.drop_reason]``
        (``"unknown"`` when None). Fired → ``ticks_fired``; ``emitted`` →
        ``rows_emitted``; a non-None ``step_ms`` enters the ring and advances
        the running max.
        """
        if not decision.fired:
            self.ticks_dropped += 1
            reason = decision.drop_reason or "unknown"
            self.drops_by_reason[reason] = self.drops_by_reason.get(reason, 0) + 1
            return
        self.ticks_fired += 1
        if decision.emitted:
            self.rows_emitted += 1
        if decision.step_ms is not None:
            self._step_ring.append(decision.step_ms)
            if self._step_max is None or decision.step_ms > self._step_max:
                self._step_max = decision.step_ms

    def snapshot(self) -> HealthStats:
        """Project the current aggregate: dict copied, percentiles over the ring.

        p50/p95 are nearest-rank over the (up to ``ring_size``) most recent
        step durations — an honest recent window, in BOTH the live and the
        post-hoc fold (the guarantee demands one definition). ``step_ms_max``
        is the all-time running max. All three ``None`` when no durations were
        observed. Drain-caller-only.
        """
        ring = sorted(self._step_ring)
        p50 = _nearest_rank(ring, 50.0) if ring else None
        p95 = _nearest_rank(ring, 95.0) if ring else None
        return HealthStats(
            ticks_fired=self.ticks_fired,
            ticks_dropped=self.ticks_dropped,
            rows_emitted=self.rows_emitted,
            drops_by_reason=dict(self.drops_by_reason),
            step_ms_p50=p50,
            step_ms_p95=p95,
            step_ms_max=self._step_max,
        )


# ── the row (Q2 split: flat wire row here; dict rides HealthRecord in storage) ─


class Health(NamedTuple):
    """THE global health row: one flat wire-safe shape for every pure module.

    Replaces ``legacy.HealthPlaceholder`` — autoconnect keys the shared topic
    on ``("health", Health)``. Flat primitives + short strings ONLY (no
    containers): this row IS the wire payload, the ``to_rerun`` subject, and
    the LCM msg shape if the topic ever needs one (Q2 resolution — the full
    ``drops_by_reason`` dict stays in storage via ``HealthRecord``).

    Two clocks: ``ts`` (wall, emission stamp — the only wall clock in the
    system) and ``frontier_ts`` (data watermark). ``ts`` advancing while
    ``frontier_ts`` freezes IS the stall signature; joins happen on
    ``frontier_ts`` only. As a module input, health must be declared
    ``pm.latest(control=True)`` (a wall-stamped row sits permanently in the
    future of replayed data time).
    """

    ts: float  # wall clock at emission (the ONLY wall stamp)
    path: str  # module path (T13 dots: "nav.voxel_mapper2")
    frontier_ts: float  # data-time watermark (-inf before the first tick)
    seq: int  # health row counter (a gap = dropped health rows)
    # cumulative counters (consumers diff; robust to missed rows)
    ticks_fired: int
    ticks_dropped: int
    rows_emitted: int
    top_drop_reason: str  # "" when nothing dropped (flat stand-in for the dict)
    debug_dropped: int  # T15 ring overflow counter (0 when capture is off)
    # windowed conveniences (since the previous health row; wall window)
    out_hz: float
    step_ms_p50: float  # nan when no samples (wire wants fixed-width scalars)
    step_ms_p95: float
    inflight: int  # rim in-queue depth at the sampling instant (Q6)
    # sparse — None when healthy
    contract_violation: str | None  # "global_map: 0.11 Hz < min_hz 0.25"
    resource_error: str | None  # from T7 warmup/dispose failures / session death

    @property
    def lag(self) -> float:
        """``ts - frontier_ts``: how far behind data time the module is (wall s)."""
        return self.ts - self.frontier_ts

    def to_rerun(self) -> list[tuple[str, Any]]:
        """Flattened projection as ``(entity_path, archetype)`` pairs.

        Numeric fields (incl. ``lag``) → ``rr.Scalars`` under
        ``rerun_entity(path, field)`` = ``plots/health/<path>/<field>``
        (nan-valued fields skipped); ``top_drop_reason`` and the sparse
        violation/error strings → text entries at the same root. ``rerun``
        imports lazily — health.py stays importable without viz extras.
        Per-reason drop series are post-hoc plots from the STORED dict,
        never live entities (unbounded-cardinality rule, Q2).
        """
        import rerun as rr  # lazy: health.py stays importable without viz extras

        pairs: list[tuple[str, Any]] = []
        numeric: list[tuple[str, float]] = [
            ("frontier_ts", self.frontier_ts),
            ("lag", self.lag),
            ("seq", float(self.seq)),
            ("ticks_fired", float(self.ticks_fired)),
            ("ticks_dropped", float(self.ticks_dropped)),
            ("rows_emitted", float(self.rows_emitted)),
            ("debug_dropped", float(self.debug_dropped)),
            ("out_hz", self.out_hz),
            ("step_ms_p50", self.step_ms_p50),
            ("step_ms_p95", self.step_ms_p95),
            ("inflight", float(self.inflight)),
        ]
        for field, val in numeric:
            if math.isfinite(val):  # skip nan percentiles + -inf frontier/lag
                pairs.append((rerun_entity(self.path, field), rr.Scalars(val)))
        for field, text in (
            ("top_drop_reason", self.top_drop_reason),
            ("contract_violation", self.contract_violation),
            ("resource_error", self.resource_error),
        ):
            if text:
                pairs.append((rerun_entity(self.path, field), rr.TextLog(text)))
        return pairs


class HealthRecord(NamedTuple):
    """Storage row for the run db: the wire row + the full drops dict (Q2).

    Appended by the pacer to the ``health_stream(path)`` stream of the SAME
    run db T15 uses (pickle codec, explicit ``ts=``) — investigating health is
    the debug-investigation motion: one load, health next to decisions,
    joined on ``frontier_ts``.
    """

    health: Health
    drops_by_reason: dict[str, int]


# ── pacer input seams (protocols keep health.py rim-free; no import cycles) ───


class SessionSnapshot(Protocol):
    """What the pacer reads each wake — lock-free, acceptably stale (rim contract).

    ``rim.RimStats`` satisfies this structurally once it gains ``frontier_ts``
    and ``inflight`` (Implementation contract H4); tests substitute plain
    fakes. Read-only by protocol (dataclass fields satisfy structurally).
    Nothing in a snapshot reads a clock.
    """

    @property
    def frontier_ts(self) -> float:
        """Aligner tick-port data watermark (-inf before the first tick)."""
        ...

    @property
    def inflight(self) -> int:
        """Queued items across the session's in-port queues (Q6: ring depth)."""
        ...

    @property
    def rows_emitted(self) -> int:
        """Driver emit count (``RunHooks.emits``) — the out_hz numerator."""
        ...

    @property
    def published(self) -> Mapping[str, int]:
        """Per Out field cumulative publish counts (contract cadence feed)."""
        ...

    @property
    def error(self) -> BaseException | None:
        """What killed the session, if anything (→ ``resource_error``)."""
        ...


class DecisionSource(Protocol):
    """The T15 ring, drained through the counter fold (contract H3).

    ``debugrec.DebugSession`` satisfies this once it gains ``drain_into``
    (drains the ring, calls ``observe`` per decision payload, forwards the
    whole batch to the writer when one exists) and ``dropped``.
    """

    def drain_into(self, observe: Callable[[TickDecision], None]) -> int:
        """Drain queued events; fold decisions through ``observe``; persist; count."""
        ...

    @property
    def dropped(self) -> int:
        """Ring events evicted by overflow since session start (monotonic)."""
        ...


def evaluate_contract(
    field: str, min_hz: float, *, now: float, started: float, last_emit: float
) -> str | None:
    """One Out contract's wall-window judgment; None while honored or in grace.

    All arguments are wall-clock seconds from the pacer's clock. Rule:

    - grace: None while ``now - started < CONTRACT_GRACE_PERIODS / min_hz``;
    - after grace: violation iff ``now - last_emit > 1 / min_hz``, where
      ``last_emit`` is the wall instant the pacer last observed the field's
      publish count increase (initialized to ``started``);
    - the message reports the observed rate as ``1 / (now - last_emit)``:
      ``f"{field}: {observed:.2f} Hz < min_hz {min_hz}"``.

    Pure and clock-free by signature — unit-testable without threads.
    """
    if now - started < CONTRACT_GRACE_PERIODS / min_hz:
        return None  # still inside the startup grace window
    if now - last_emit > 1.0 / min_hz:
        observed = 1.0 / (now - last_emit) if now > last_emit else float("inf")
        return f"{field}: {observed:.2f} Hz < min_hz {min_hz}"
    return None


# ── the pacer (the ONE per-session off-tick-path thread; wall clock lives here) ─


class HealthPacer:
    """Per-live-session daemon thread: emit the health row, drain the debug ring.

    Owned by the rim (started at session start, stopped+joined at teardown —
    never by ``over()``: health is live-only). Each wake (``1/health_hz`` s)
    runs ``sample()``: drain the T15 ring through the counter fold, read the
    session snapshot lock-free, evaluate contracts, assemble + publish +
    record one ``Health`` row. The pacer never depends on ticks — a stalled
    (or dead-but-not-stopped) module still healths; that is the point.
    Zero tick-path cost: nothing here is ever called from the tick path, and
    nothing the pacer reads takes a tick-path lock.

    ``health_hz == 0`` is drain-only mode: no rows, but the ring still drains
    every ``DRAIN_ONLY_PERIOD_S`` (the retired T15 interim pacer's job).
    A sampling fault logs and continues — health must be the last thing to
    die, and one bad wake must not silence the next.
    """

    path: str

    def __init__(
        self,
        path: str,
        *,
        snapshot: Callable[[], SessionSnapshot],
        health_hz: float = DEFAULT_HEALTH_HZ,
        contracts: Mapping[str, float] | None = None,  # Out field → min_hz
        decisions: DecisionSource | None = None,  # None: no debug ring (counters stay zero)
        publish: Callable[[Health], None] | None = None,  # wire egress (legacy health stream)
        record: Callable[[HealthRecord, float], None] | None = None,  # run-db append (ts arg)
        counters: HealthCounters | None = None,  # injectable for tests; fresh by default
        clock: Callable[[], float] = time.time,  # THE wall-clock exemption; injectable
    ) -> None:
        """Bind seams; validates ``health_hz >= 0``; inert until ``start``."""
        if health_hz < 0:
            raise ValueError(f"HealthPacer(health_hz={health_hz}): must be >= 0")
        self.path = path
        self._snapshot = snapshot
        self._hz = health_hz
        self._contracts: dict[str, float] = dict(contracts or {})
        self._decisions = decisions
        self._publish = publish
        self._record = record
        self._counters = counters if counters is not None else HealthCounters()
        self._clock = clock
        self._seq = 0
        self._latest: Health | None = None
        self._started_wall: float | None = None  # set once by start(); grace baseline
        self._last_emit_wall: dict[str, float] = {}  # per contract field
        self._prev_published: dict[str, int] = {}
        self._prev_rows = 0  # out_hz window numerator baseline
        self._prev_wall: float | None = None  # out_hz window denominator baseline
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def latest(self) -> Health | None:
        """The most recently assembled row (lock-free peek; None before the first)."""
        return self._latest

    def start(self) -> None:
        """Spawn the daemon thread (``pure-health:<path>``); idempotent.

        Stamps ``started`` (wall) as the contract grace baseline and the
        first out_hz window edge, then wakes every ``1/health_hz`` s
        (``DRAIN_ONLY_PERIOD_S`` when ``health_hz == 0``) calling ``sample``
        (drain-only skips row assembly), guarding each wake with a
        log-and-continue fault handler.
        """
        if self._thread is not None:
            return  # idempotent
        started = self._clock()
        self._started_wall = started
        self._prev_wall = started  # first out_hz window opens at start
        self._stop.clear()
        drain_only = self._hz <= 0
        period = DRAIN_ONLY_PERIOD_S if drain_only else 1.0 / self._hz

        def _run() -> None:
            while not self._stop.wait(period):
                try:
                    if drain_only:
                        self._drain_only()
                    else:
                        self.sample()
                except BaseException as exc:  # health is the last thing allowed to die
                    _LOG.warning("health pacer wake failed for %s: %r", self.path, exc)

        self._thread = threading.Thread(target=_run, name=f"pure-health:{self.path}", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Signal + join the thread (bounded by one period); idempotent.

        Runs one final ``sample()`` best-effort before returning so teardown
        flushes the last window; never raises into the caller's teardown.
        """
        self._stop.set()
        thread = self._thread
        if thread is not None:
            period = DRAIN_ONLY_PERIOD_S if self._hz <= 0 else 1.0 / self._hz
            thread.join(timeout=period * 4)
            self._thread = None
        try:
            if self._hz <= 0:
                self._drain_only()
            else:
                self.sample()
        except BaseException as exc:  # teardown must never raise
            _LOG.warning("health pacer final sample failed for %s: %r", self.path, exc)

    def _drain_only(self) -> None:
        """Drain-only wake (``health_hz == 0``): move the ring through the fold, no row."""
        if self._decisions is not None:
            self._decisions.drain_into(self._counters.observe)

    def sample(self) -> Health:
        """One wake's work, sans sleep — THE unit-testable heart.

        1. ``decisions.drain_into(counters.observe)`` when a ring is wired
           (T15 persistence rides the same drain).
        2. ``snapshot()`` once; ``stats = counters.snapshot()``.
        3. out_hz = Δ``snapshot.rows_emitted`` / Δwall since the previous row
           (0.0 on the first row); contract fields whose ``published`` count
           advanced refresh ``last_emit_wall``; violations via
           ``evaluate_contract``, joined with ``"; "`` (None when all honored).
        4. Assemble ``Health`` (ts = ``clock()``, seq++, frontier/inflight
           from the snapshot, ``resource_error = repr(snapshot.error)`` when
           set, ``debug_dropped = decisions.dropped`` when wired, None→nan
           percentile mapping); publish when wired; record
           ``HealthRecord(row, stats.drops_by_reason)`` when wired (storage
           ts per contract H7); retain as ``latest``; return it.
        """
        # 1. drain the T15 ring THROUGH the counter fold (the counters' live feed)
        if self._decisions is not None:
            self._decisions.drain_into(self._counters.observe)
        # 2. one snapshot each (session state + folded counters)
        snap = self._snapshot()
        stats = self._counters.snapshot()
        now = self._clock()
        # 3a. out_hz — a wall-window rate over the driver emit count (per-consumer clock)
        if self._prev_wall is None:
            out_hz = 0.0
        else:
            dt = now - self._prev_wall
            out_hz = (snap.rows_emitted - self._prev_rows) / dt if dt > 0 else 0.0
        self._prev_wall = now
        self._prev_rows = snap.rows_emitted
        # 3b. contracts — refresh last_emit for fields whose publish count advanced
        started = self._started_wall if self._started_wall is not None else now
        published = snap.published
        violations: list[str] = []
        for field, min_hz in self._contracts.items():
            last_emit = self._last_emit_wall.setdefault(field, started)
            cur = published.get(field, 0)
            if cur > self._prev_published.get(field, 0):
                last_emit = now
                self._last_emit_wall[field] = now
            self._prev_published[field] = cur
            v = evaluate_contract(field, min_hz, now=now, started=started, last_emit=last_emit)
            if v is not None:
                violations.append(v)
        contract_violation = "; ".join(violations) if violations else None
        # 4. assemble, publish, record, retain
        seq = self._seq
        self._seq += 1
        row = Health(
            ts=now,
            path=self.path,
            frontier_ts=snap.frontier_ts,
            seq=seq,
            ticks_fired=stats.ticks_fired,
            ticks_dropped=stats.ticks_dropped,
            rows_emitted=stats.rows_emitted,
            top_drop_reason=stats.top_drop_reason,
            debug_dropped=self._decisions.dropped if self._decisions is not None else 0,
            out_hz=out_hz,
            step_ms_p50=stats.step_ms_p50 if stats.step_ms_p50 is not None else math.nan,
            step_ms_p95=stats.step_ms_p95 if stats.step_ms_p95 is not None else math.nan,
            inflight=snap.inflight,
            contract_violation=contract_violation,
            resource_error=repr(snap.error) if snap.error is not None else None,
        )
        if self._publish is not None:
            self._publish(row)
        if self._record is not None:
            record_ts = max(snap.frontier_ts, 0.0) if math.isfinite(snap.frontier_ts) else 0.0
            self._record(HealthRecord(row, stats.drops_by_reason), record_ts)
        self._latest = row
        return row
