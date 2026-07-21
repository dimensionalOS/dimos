# T9 — Health (live liveness for pure modules)

Status: spec-ready — typed skeleton + implementation contract below
(2026-07-21). Owner: unassigned.
Prereq context: index.md §T9 charter; T5/T6/T8/T11 engine counters
(`AlignStats`/`PortStats`/`TfStats`); T8 legacy bridge health mechanism
(`legacy.py`: `HealthPlaceholder`, `_HEALTH_STREAM` — T9 swaps the TYPE, not
the mechanism); T15 (`tasks/t15-debug.md`: shares the step-timing hook and the
per-session pacer thread — T15's interim pacer is retired INTO T9's).

## Scope (narrowed 2026-07-21)

Health owns **live operational liveness only**: "is this module alive,
keeping up, honoring contracts — NOW." It is NOT a performance-measurement
tool:

- Under `over()`/`--pure` (faster-than-realtime) health is OFF. Eval-mode
  performance is a post-hoc query over T15 records: step_ms distribution
  (wall durations, valid in both modes) + contract cadence in DATA time
  (emit-ts diffs, replay-speed-invariant). A 1 Hz wall sampler of a 14×
  replay answers a question that doesn't exist.
- Consequence: T9 needs no replay mode, no data-time pacer, no eval story.
  One whole class of complexity deleted before implementation.

## Settled doctrine

1. **ONE global stamped `Health` row type** for all modules (index charter);
   autoconnect keys the shared topic on `("health", Health)`, so the type is
   a single named class in `dimos/pure/health.py` replacing
   `legacy.HealthPlaceholder`.
2. **Wall clock is legal HERE ONLY** — and only as the row *stamp* and the
   pacer. The pacer thread reads monotonic counters; zero locks, zero work on
   the tick path.
3. **A stalled module still healths** — the pacer never depends on ticks.
4. **Two clocks per row (2026-07-21):** `ts` = wall emission clock;
   `frontier_ts` = the module's data-time watermark (aligner `last_ts`,
   already tracked). `ts − frontier_ts` = lag. Row `ts` advancing while
   `frontier_ts` freezes IS the stall signature. Any join of health against
   data happens on `frontier_ts`, NEVER on the wall stamp.
5. **Health as a module input** (a future HealthMonitor): must be declared
   `pm.latest(control=True)` or joined on `frontier_ts` — a wall-stamped row
   sits permanently in the future of replayed data time (the rerun-click
   pathology; control=True exists for exactly this).
6. **Shared machinery with T15:** the step-timing wrap in `run_over` (T15
   contract) feeds both `step_ms` percentiles here and per-tick `step_ms`
   there; the per-session off-tick-path thread is ONE thread with two jobs
   (emit health row, drain debug ring).

## Layering with T15: share the aggregator, not the row (Ivan, 2026-07-21)

Most Health fields are projections of the same per-tick decision events T15
records. The shared layer is ONE aggregation class in `health.py`:

```
TickDecision events → HealthCounters.observe(d)   # incremental
                      HealthCounters.snapshot() → HealthStats
live:     pacer wraps → Health(ts=wall, frontier_ts, inflight, …, stats)
post-hoc: T15 ModuleDebug.summary() folds recorded decisions through the
          SAME HealthCounters
```

Guarantee by construction (and acceptance-testable): the post-hoc summary of
a debug capture equals what health would have reported live.

Field taxonomy that makes the split principled:
- **Counts + durations are clock-free → in the shared `HealthStats`**
  (ticks_fired, ticks_dropped, rows_emitted, drops_by_reason, step_ms
  percentiles — a duration is not a timestamp).
- **Rates are clock projections → per-consumer, NOT shared**: live `out_hz`
  is per wall second; post-hoc cadence is per data second (wall arrival is
  deliberately absent from records). Each consumer computes its rate against
  its own clock.
- **Liveness extras are health-only** (not derivable from decision events):
  frontier_ts lag, inflight (rim ring depth), resource_error (T7 lifecycle),
  debug_dropped (ring overflow).

Sequencing: T15 is implemented first with a self-contained `summary()`; T9's
implementation contract includes extracting `HealthCounters` into `health.py`
and refactoring `ModuleDebug.summary()` onto it (one definition of every
counter, forever after).

## The row (proposal — field-level review wanted)

```python
class Health(NamedTuple):        # or frozen dataclass; wire-safe, pickle codec
    ts: float                    # wall clock at emission (the ONLY wall stamp)
    path: str                    # module path (T13 dots: "nav.voxel_mapper2")
    frontier_ts: float           # data-time watermark (-inf before first tick)
    seq: int                     # health row counter (gap = dropped rows)
    # cumulative counters (consumers diff; robust to missed rows)
    ticks_fired: int
    ticks_dropped: int
    rows_emitted: int
    drops_by_reason: dict[str, int]   # 'tf-unresolvable:pose' → n
    debug_dropped: int           # T15 ring overflow counter (0 when off)
    # windowed conveniences (since previous health row)
    out_hz: float                # emits/sec, wall window
    step_ms_p50: float | None
    step_ms_p95: float | None
    inflight: int                # rim ring depth at sampling instant
    # sparse — None when healthy
    contract_violation: str | None    # "global_map: 0.11 Hz < min_hz 0.25"
    resource_error: str | None        # from T7 warmup/dispose failures
```

## Mechanism

- **Emission**: the T8 rim's synthesized `health` stream (`m.health`,
  `legacy._HEALTH_STREAM` topic) — mechanism unchanged, payload type swapped.
  Dogfoods T8 port machinery; per-namespace merged topic as today.
- **Pacer**: per-session daemon thread, `health_hz` (default 1 Hz). Reads
  engine counters lock-free (momentarily-stale reads are fine — drivers.py
  already documents this contract), assembles the row, publishes, then drains
  the T15 ring. Started by the rim at session start, joined at teardown.
- **contract_ok evaluation**: per Out field with `pm.contract(min_hz=...)`,
  wall-window cadence vs min_hz, with a startup grace of `2/min_hz` seconds
  before first judgment (a 0.25 Hz contract cannot be violated 1 s in).
- **Recording**: the pacer appends every Health row to the run db
  (`<path>.health` stream, same store + writer thread as T15; explicit
  `ts=` on append). Health investigation is the debug-investigation motion:
  one `pm.debug_load`, health next to decisions, joined on `frontier_ts`.
- **Viz tie-in** (`number_problem.md`): `Health.to_rerun()` → the FLATTENED
  projection (see Q2 resolution) as `rr.Scalars` under
  `plots/health/<path>/<field>` — dashboards for free through the same
  scalar path; per-reason drop series are post-hoc plots from the stored
  dict, never live entities.

## QUESTIONS (Ivan triage; implementer proceeds on DEFAULT)

QUESTION 1: Counter semantics — cumulative-only, or cumulative + windowed conveniences?
DEFAULT: both, as in the row above (cumulative for robustness, out_hz/p95 windowed for dashboards).
OPTIONS: a) both (default) — slightly fatter row; b) cumulative only — consumers diff, leaner but every dashboard reimplements rates; c) windowed only — fragile across dropped health rows.

QUESTION 2 — RESOLVED (Ivan, 2026-07-21): split system-of-record from projection.
- **Storage**: health rows are recorded into the SAME run db T15 uses
  (stream `<path>.health`, same writer thread, pickle codec) with the FULL
  `drops_by_reason` dict — full fidelity where pickle is legal. ~200 B at
  1 Hz: always-on live is noise. Investigating health = the same
  `pm.debug_load` motion as debug records; join on `frontier_ts`.
- **Wire topic + `to_rerun()`**: flattened projection ONLY — counts,
  `drops_total`, `top_drop_reason: str`, `lag`, `out_hz`, `step_ms_p95`,
  `inflight`, sparse violation strings. All flat primitives/short strings:
  renders as per-field `rr.Scalars` (+ reason as text), and IS the LCM msg
  shape if the topic ever needs one (diagnostic_msgs KeyValue lesson).
- Rationale: reason strings are unbounded-cardinality — fine as offline plots
  from the stored dict, wrong as dynamically-minted live scalar entities.
- Open sub-point for impl: whether the live topic can carry a pickled plain
  row depends on rim egress for non-LCM payloads (zenoh pickles, LCM
  doesn't). DEFAULT: plain row + existing egress; if LCM-only, the flattened
  projection above becomes the LCM msg with no design change.

QUESTION 3: `health_hz` config — where does the knob live?
DEFAULT: a `health_hz: float = 1.0` field on PureModuleConfig (engine-level, every module inherits; per-member override falls out of per-instance config).
OPTIONS: a) config field (default); b) global_config only — no per-member override, simpler; c) rim/session parameter — invisible to module authors but no blueprint-time override.

QUESTION 4: step_ms percentile window — what backs p50/p95?
DEFAULT: fixed-size ring of the last 256 step durations per session (bounded memory, honest percentiles over the recent window).
OPTIONS: a) 256-ring (default); b) since-last-row list — exact per window but unbounded under fast ticks; c) t-digest — precise, a dependency for little gain at 1 Hz.

QUESTION 5: does T9 also own the WATCHDOG (alarm when a member's health stalls or violates), or emit-only?
DEFAULT: emit-only; a HealthMonitor consumer module (health as latest(control=True) input) is a separate follow-up task.
OPTIONS: a) emit-only (default); b) coordinator-side stall detection now — more moving parts inside T9.

QUESTION 6: `inflight` source — rim ring depth only, or also async-stateless in-flight step count?
DEFAULT: rim ring depth; async in-flight count added when an async module actually needs it.
OPTIONS: a) ring depth (default); b) both now — touches the async driver path for a currently-unused number.

## Triage resolutions (Ivan, 2026-07-21)

- **Q1**: resolved by the aggregator taxonomy — counts + durations live in the
  shared `HealthStats`; rates are per-consumer clock projections (live wall
  `out_hz` computed by the pacer; data-time cadence computed post-hoc).
- **Q2**: resolved above (storage/projection split; full dict to the run db,
  flattened projection to wire + rerun).
- **Q3**: default — `health_hz: float = 1.0` on PureModuleConfig (per-member
  override via per-instance config).
- **Q4**: default — 256-entry step-duration ring backs p50/p95.
- **Q5**: default — emit-only; HealthMonitor consumer is a separate follow-up
  task (health-as-input doctrine: latest(control=True)).
- **Q6**: default — inflight = rim ring depth; async in-flight count deferred
  until an async module needs it.

---

# Implementation contract (typed spec — 2026-07-21)

Normative skeleton: **`dimos/pure/health.py`** (mypy --strict clean; bodies
`NotImplementedError`, exact signatures binding; load-bearing one-liners
implemented). Acceptance tests, skip-gated `"T9 impl pending"`:
**`dimos/pure/test_health.py`**. T15 LANDED on this branch while this
contract was written; the seam diffs below are SPEC'D, NOT APPLIED —
`align.py`, `rim.py`, `drivers.py`, `debugrec.py`, `test_debugrec.py`,
`legacy.py`, `config.py`, `__init__.py` are untouched by T9's spec commit.
All anchors are symbolic (function/class + a short quote); quote matches,
never line numbers, are authoritative — T15's tree is fresh and still
settling. Every anchor: provisional, verify at impl.

## H1. File + layering

`health.py` (charter-fixed name; unlike debug-vs-debugrec there is no
`pm.health` callable to shadow — `health` is a stream/attr name only).
Module-scope imports: stdlib + `debugrec.TickDecision` + `setup_logger`.
health.py imports NO rim, NO legacy, NO mem2 — the rim edge is structural
(`SessionSnapshot`/`DecisionSource` protocols), storage arrives as a
`record` callable the rim builds. Edges: health → debugrec (the event
vocabulary); debugrec → health ONLY lazily inside `ModuleDebug.summary()`
(H3 — no cycle); legacy → health (H6 type swap); rim → health lazily (the
tfbuffer pattern). Wall-clock audit: the only wall clock in the whole
feature is `HealthPacer`'s `clock` default (`time.time`); durations arrive
already-monotonic from T15's `perf_counter` bracket; `evaluate_contract` is
clock-free by signature.

## H2. Row spelling — DECIDED: `Health` is the flat wire row; `HealthRecord` is storage

Doctrine 1 says autoconnect keys on `("health", Health)`; Q2 says the wire
carries the flattened projection only. Both hold iff **`Health` IS the
flattened projection**: a flat NamedTuple (primitives + short strings, no
containers — pinned by `test_flat_wire_row_contains_no_containers`), the
wire payload, the `to_rerun()` subject, and the LCM msg shape if the topic
ever needs one. The full `drops_by_reason` dict rides
`HealthRecord(health, drops_by_reason)` in the run db ONLY. Rejected
alternative — `Health` embedding `HealthStats` — would force a second wire
type and re-key autoconnect on the projection, or put a dict on the wire;
counter definitions already live once in `HealthStats`, so the flat row
copies VALUES (at 1 Hz, in the pacer), not definitions. `drops_total` was
dropped from the row (`== ticks_dropped` by construction; kept as a
`HealthStats` accessor); `top_drop_reason: str` (`""` when clean) is the
flat stand-in for the dict. `lag` is a property (`ts − frontier_ts`),
recomputed, not stored — it becomes a real field only in an LCM mapping.

## H3. Shared aggregator + `ModuleDebug.summary()` refactor (debugrec.py, post-T15)

`HealthCounters.observe/snapshot` is the ONE definition of every count and
duration stat. Percentiles: nearest-rank over a `STEP_RING_SIZE`-deep
recent window in BOTH folds (the by-construction guarantee demands one
definition — post-hoc summary percentiles change semantics from all-run to
last-256-window, deliberately); `step_ms_max` is the all-time running max
(monotone, preserves summary's current max semantics).

Refactor (T9's wave, now that T15 landed): in `ModuleDebug.summary()`
(anchor: `step_ms = sorted(t.step_ms for t in fired if t.step_ms is not None)`
and the `by_reason` accumulation loop above it), replace the hand-rolled
reason/percentile accumulation with

```python
from dimos.pure.health import HealthCounters  # lazy: keeps debugrec module-scope stdlib-only
counters = HealthCounters()
for t in ticks:
    counters.observe(t)
stats = counters.snapshot()
```

and format from `stats` (drops by reason from `stats.drops_by_reason`,
p50/p95/max from the snapshot). `by_field` derivation (reason-string
parsing) and `_contract_cadence` stay local to debugrec — data-time cadence
is a per-consumer rate projection, exactly what the taxonomy keeps out of
the shared layer. `debugrec._pct` dies; nearest-rank lives inside
`HealthCounters.snapshot` (private helper in health.py).

## H4. The pacer — one thread, two jobs; T15 interim-pacer retirement

**Ownership**: the rim. `_LiveSession` gains `self._health: Any = None`
(HealthPacer | None); built + started in `start()` after the T15 debug
wiring block (anchor: `self._debug = debugrec.session_for(`) and the
session thread spawn; stopped + joined in `stop()` after
`thread.join(STOP_JOIN_TIMEOUT)` and in `_unwind_start`. Deliberately NOT
stopped in `_run`'s finally: a dead-but-not-stopped session keeps emitting
rows with `resource_error` set (the flight recorder outlives the crash;
doctrine 3). `over()`/`run_over` never construct a pacer — health is off
under eval by construction, nothing to gate.

**The second job**: `sample()` step 1 drains the T15 ring THROUGH the
counter fold. debugrec additions (typed against `health.DecisionSource`):

```python
class DebugSession:
    def drain_into(self, observe: Callable[[TickDecision], None]) -> int:
        """Drain queued events; fold each TickDecision payload through observe;
        forward the whole batch to the writer (when one exists); return count."""
    @property
    def dropped(self) -> int:
        """Ring overflow counter (delegates to self.ring.dropped)."""
    @property
    def writer(self) -> DebugWriter:
        """The session's writer (H7 record closure needs it)."""
```

ADJUSTMENT to t15 §C8, flagged: C8 said the T9 pacer "invokes
`DebugWriter.drain()` each wake". Writer-level drain hides the decision
payloads from the counter fold, which the by-construction guarantee needs —
the integration point moves one level down, to `DebugSession.drain_into`.
`DebugWriter.drain()` survives for teardown/multi-session use;
`DebugSession.drain()` survives for the over() inline path.

**Interim-pacer retirement** (debugrec.py):

1. In `session_for`, delete
   ```python
       if live:
           writer.start_pacer()
   ```
2. In `DebugWriter`, delete `start_pacer`, the `_pacer`/`_pacer_stop`
   fields, and the pacer stop/join block at the top of `close()`; delete
   `DEFAULT_PACER_PERIOD_S` (constant + `__all__` + docstring mention) —
   its 0.5 s cadence survives as `health.DRAIN_ONLY_PERIOD_S`.
3. `test_debugrec.py`: update any pin of `start_pacer`/
   `DEFAULT_PACER_PERIOD_S` to the rim-owned pacer (verify at impl —
   T15's tests are green as landed).
4. Drain-only mode closes the gap retirement opens: when a live session has
   a debug session but `health_hz == 0`, the rim still constructs + starts
   the pacer (no row assembly; ring drained every `DRAIN_ONLY_PERIOD_S`).
   The ring is never left undrained-until-close.

**Fault policy**: a wake fault (including a raising `publish` callable)
logs at WARNING and continues — health is the last thing allowed to die,
and one bad wake must not silence the next. No disable latch (contrast
`DebugSession._fault`: capture may degrade, liveness must not).

## H5. Rim + align seams

**`Aligner.frontier_ts`** (align.py, next to the existing
`held_tick_ts` property — anchor `def held_tick_ts`):

```python
    @property
    def frontier_ts(self) -> float:
        """Tick-port data watermark (-inf before the first accepted tick);
        lock-free, acceptably stale (same contract as stats)."""
        return self._tick.last_ts
```

**`RimStats`** (rim.py, anchor `class RimStats`) gains two fields + one
derived property, after which it satisfies `health.SessionSnapshot`
structurally (pinned by a fake in test_health.py, and at impl by a direct
`session.stats()`-into-pacer test):

```python
    frontier_ts: float   # aligner.frontier_ts; -inf when no aligner yet
    inflight: int        # sum(len(q.items) for q in session queues) — benign race
    @property
    def rows_emitted(self) -> int:
        """Driver emit count (hooks.emits) — the out_hz numerator."""
        return self.hooks.emits
```

Filled in BOTH constructors — `_LiveSession.stats()` (anchor
`return RimStats(`) and module-level `stats()`.

**Pacer construction** (in `_LiveSession.start()`, after the debug block):

```python
from dimos.pure import health as _health  # lazy: sanctioned engine edge
hz = float(getattr(self._module.config, "health_hz", _health.DEFAULT_HEALTH_HZ))
contracts = {
    name: fs.min_hz
    for name, fs in self._spec.out_type.fields().items()
    if isinstance(fs, ContractSpec)
}
self._health = _health.HealthPacer(
    path,                       # same identifier the debug session uses
    snapshot=self.stats,
    health_hz=hz,
    contracts=contracts,
    decisions=self._debug,      # DecisionSource once H4 lands; None when capture off
    publish=self._health_publish,
    record=_record,             # H7 closure; None when no writer
)
self._health.start()
```

`path` = `debugrec.default_path(module)` for a lone `start_module`; the
legacy adapter's coordinator module name when deploying (identical to the
debug session's path — one identifier per member everywhere).
`TfOutSpec.min_hz` is OUT of live contract scope (tf_out publishes onto the
TF rail, not `_published`; its cadence stays a post-hoc data-time query) —
note in the pacer docstring at impl.

**Publish threading**: `rim.start_module(module)` (anchor
`def start_module`) gains `health_publish: Callable[[Any], None] | None = None`,
stored onto the session. The legacy actor's `start` (legacy.py, anchor
`rim.start_module(self._pure)`) becomes

```python
        rim.start_module(self._pure, health_publish=getattr(self, _HEALTH_STREAM).publish)
```

— the same legacy `Out` stream object the rim already publishes through as
a port transport (`transport.publish(value)` in `_emit`), so `.publish`
exists; exact spelling verify at impl. Bare rim sessions (no legacy
adapter) run with `publish=None`: rows still record to the run db (Q7
below). Wire payload is the plain pickled `Health` row over existing
egress (Q2 sub-point default); an LCM-only backend maps the same flat
fields to a msg def with no design change.

## H6. The type swap (legacy.py)

Current text (the mechanism T9 keeps, the type it replaces):

```python
class HealthPlaceholder:
    """Named placeholder health payload; T9 swaps the TYPE, not the mechanism.
    ..."""

_HEALTH_PAYLOAD: type = HealthPlaceholder
"""Health stream payload type; T9 replaces with ``dimos.pure.health.Health`` (spec §12)."""
```

becomes

```python
from dimos.pure.health import Health

_HEALTH_PAYLOAD: type = Health
"""Health stream payload type (T9): the ONE global flat health row."""
```

with `class HealthPlaceholder` DELETED (not aliased) and
`"HealthPlaceholder"` removed from `legacy.__all__`. The annotation
synthesis line — `annotations[_HEALTH_STREAM] = _stream_annotation(Out, _HEALTH_PAYLOAD)`
— is untouched: annotation, stream ref, autoconnect merge, adapter all
unchanged; the shared topic key becomes `("health", Health)` (pinned by
`test_placeholder_type_swap_keeps_autoconnect_key_legible`). Import
direction is legal: legacy already imports `dimos.pure.rim`; health.py
imports neither legacy nor rim. `test_legacy.py`'s health checks are
name-based (`"health"` in refs/registry) and survive; grep for any direct
`HealthPlaceholder` reference at impl. t8-rim.md §12/P21 references stay as
history.

## H7. Storage — health next to decisions, same run db

The pacer records `HealthRecord` to stream **`health_stream(path)`** =
`<path>.health` (dotted names are legal post-T15's mem2 relaxation) in the
SAME debug.db, via a new writer method:

```python
class DebugWriter:
    def append_row(self, stream: str, payload: Any, ts: float) -> None:
        """Append one non-ring row (drain callers only; short txn, same lock
        as write; store.stream(stream, type(payload)).append(payload, ts=ts))."""
```

rim builds `_record = lambda rec, ts: self._debug.writer.append_row(
_health.health_stream(path), rec, ts)` when a debug session exists, else
`record=None` (no run dir ⇒ no health persistence; live topic + `latest`
still serve). Storage obs ts = **`frontier_ts`**, clamped
`max(frontier_ts, 0.0)` while non-finite — range queries then line up with
the decisions stream (both in data time; the whole point of joining on
`frontier_ts`); the row FIELD keeps the honest `-inf` for the pre-first-tick
prefix (Q4 below). Codec: pickle via `codec_for` fallback, like every T15
record. Reader: `ModuleDebug` gains
`health() -> Iterator[HealthRecord]` reading `health_stream(self.path)`
directly, and `"health"` joins `_LAYER_NAME`/`_LAYER_SUFFIXES` so
`modules()` discovers a path that recorded only health.

## H8. Config knob — `health_hz` on `PureModuleConfig` (config.py)

Anchor `class PureModuleConfig(BaseConfig):` — the root of every
synthesized model gains

```python
    health_hz: float = Field(default=1.0, ge=0.0)
    """Live health row cadence, Hz (T9); 0 disables rows (ring drain continues)."""
```

(`from pydantic import Field` already adjacent). `RESERVED_CONFIG_FIELDS`
gains `"health_hz"` so a module body redeclaring it gets a
`ConfigFieldError` instead of a silent override. Mechanics verified:
synthesized per-module models inherit base fields via `create_model
__base__`; per-member override = constructor kwarg (`VoxelMapper2(...,
health_hz=2.0)`) / per-instance config, exactly Q3's resolution; legacy
`_actor_config_model` copies `model_fields` so ActorConfig carries it, and
`health_hz` is not in `_RESERVED_CONFIG`. SIDE EFFECT, flagged as Q6:
`model_dump()` — THE canonical config serialization ("module identity =
class + config", memo keys, T10 checkpoints) — now varies with an
operationally-inert knob.

## H9. pm surface + index

`dimos/pure/__init__.py` (T15 also edited this file — merge, don't clobber):
add `Health` to the export block (`from dimos.pure.health import Health`)
and to the `test_pm_surface` pin. ONLY `Health` — it is what users declare
(`pm.latest(control=True)` health-monitor inputs) and what autoconnect
keys; `HealthStats`/`HealthCounters`/`HealthPacer`/`HealthRecord` stay
module-qualified (`from dimos.pure import health`) as engine/post-hoc
tooling. index.md §T9 row flips to implementing/done as usual.

## H10. Acceptance mapping (`dimos/pure/test_health.py`)

- `test_stalled_module_still_healths` — doctrine 3: ts advances, frontier
  freezes, lag grows, seq counts; the pacer needs no ticks.
- `test_posthoc_fold_equals_live_fold` — THE by-construction guarantee:
  batched (live) vs single (post-hoc) fold, identical `HealthStats`.
- `test_pacer_drains_ring_through_the_counter_fold` — one thread, two jobs;
  `debug_dropped` surfaces ring overflow.
- `test_contract_grace_period` / `..._message_names_field_and_rates` /
  `..._honored_is_none_after_grace` — `evaluate_contract`, pure.
- `test_flat_wire_row_contains_no_containers` /
  `test_flattened_projection_to_rerun_no_dict_entities` — Q2 flatness, no
  reason-string entities.
- `test_health_rows_land_in_run_db_next_to_decisions` — H7 storage, obs ts
  = frontier_ts.
- `test_placeholder_type_swap_keeps_autoconnect_key_legible` — H6.
- Structural tests (row shape, `lag`, entity paths, stream name, stats
  accessors, `health_hz >= 0`) run green against the skeleton today.

---

# Questions for Ivan (T9 spec round)

The implementer proceeds on DEFAULT unless overridden — never stalls.

QUESTION 1: row spelling — `Health` as the flat wire row + `HealthRecord` storage pair (H2), or one embedded-stats row + a separate wire projection type?
DEFAULT: flat `Health` + `HealthRecord` — keeps doctrine 1 verbatim (autoconnect keys `("health", Health)`), keeps dicts off the wire, one row type everywhere a consumer looks.
OPTIONS: a) flat + record pair (default); b) `Health(stats=HealthStats, ...)` embedded — single counter spelling, but the wire then needs a second projection type and autoconnect re-keys on it; c) flat Health WITH the dict field, pickled whole onto the wire — one type total, violates the Q2 flatness resolution.

QUESTION 2: `frontier_ts` source — new `Aligner.frontier_ts` property = tick-port `last_ts`?
DEFAULT: tick-port watermark (H5) — ticks drive the module's output frontier; lock-free float read, same staleness contract as `stats`.
OPTIONS: a) tick-port last_ts (default); b) max across all port watermarks — leads the tick on fast secondaries, overstates progress; c) `held_tick_ts` when holding else tick last_ts — mixes two meanings into one field.

QUESTION 3: the counters' live feed when no run-log dir exists (T15 Q4 leaves live capture OFF then) — health still needs decision events.
DEFAULT: when health is on (hz > 0), the rim requests a decisions-only session regardless (`session_for` gains a `require: bool` — or the rim passes `debug=True`); `DebugSession` writer handling unchanged where a db resolves (T15's chain always resolves one: `./logs` fallback), so in practice this is "live decisions default ON whenever health is on", a one-line widening of T15 Q4.
OPTIONS: a) widen the live default (default); b) writer-optional DebugSession (`writer: DebugWriter | None`) for a ring-without-db mode — more surface, saves a stray debug.db; c) counters stay zero when capture is off (health rows carry liveness fields only) — breaks the by-construction guarantee exactly where it's most needed.

QUESTION 4: run-db obs ts for health rows — data time or wall?
DEFAULT: `ts=max(frontier_ts, 0.0)` (H7) — health range-queries line up with decision/data streams in data time; the row field keeps the honest `-inf`.
OPTIONS: a) frontier_ts clamped (default); b) wall `ts` — monotone obs ts, but health lands decades away from replayed data time in every range query; c) skip recording until the first tick — loses the pre-first-tick stall prefix, the most diagnostic rows of all.

QUESTION 5: the event vocabulary — health.py imports `TickDecision` from debugrec at module scope (H1), debugrec lazy-imports health only inside `summary()`.
DEFAULT: as spec'd — one event type, no cycle, debugrec module scope stays stdlib-only.
OPTIONS: a) import TickDecision (default); b) builtins-only structural event protocol in health.py — no import edge at all, but two spellings of one event and mypy can't pin the fold's input; c) move TickDecision into health.py — inverts a frozen T15 contract for aesthetics.

QUESTION 6: `health_hz` lands in `model_dump()` — module identity / memo keys / T10 checkpoint dumps now vary with an operationally-inert knob (H8 side effect; Q3's resolution predates this observation).
DEFAULT: proceed per the Q3 resolution (plain field) — dumps are honest about the full config; memoizers that care can drop it.
OPTIONS: a) plain field (default); b) `Field(exclude=True)` — identity-clean, but checkpoints/sweeps silently lose the knob; c) move the knob to global_config only — no per-member override, walks back the resolution.

QUESTION 7: wire egress for bare-rim live sessions (no legacy adapter) — `publish=None` for now?
DEFAULT: yes — the legacy adapter is the only live deployment face today; bare `start_module` health still records to the run db and serves `pacer.latest`.
OPTIONS: a) publish via the adapter only (default); b) synthesize a rim-native `m.health` out port now — dogfoods T8 ports, but invents a port outside the Out bundle for a consumer that doesn't exist yet; c) a module-level health callback registry — global state, no.

## Architect-round triage (Ivan, 2026-07-21): ALL SEVEN AT DEFAULT

Q1 flat Health + HealthRecord pair; Q2 new `Aligner.frontier_ts` property;
Q3 decisions-only session ON whenever health is on (widens T15 Q4);
Q4 run-db obs ts = `max(frontier_ts, 0.0)`; Q5 health imports TickDecision,
debugrec lazy-imports health in summary(); Q6 proceed — `health_hz` enters
`model_dump()` (identity variance with an inert knob accepted, revisit at
T10 if it bites); Q7 bare-rim sessions publish nowhere. Implementation is GO.
