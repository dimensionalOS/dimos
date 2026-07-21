# T15 — Debug recorder (per-module flight recorder + replay)

Status: spec-ready — typed skeleton + implementation contract below
(2026-07-21). Owner: unassigned.
Prereq context: T5 aligner stats, T6 drivers, T8 rim session, T9 health (shares
its thread and counters), T10 checkpoint (config-dump kinship).
Companion notes: `dimos/pure/api_improvements.md` (the incidents that motivate this).

## The incidents that motivate this

2026-07-21, building `PGOVoxelMapper`: a required-tf run emitted **0 rows with
0 warnings** three separate ways (pre-stream tick draining the tf hold; drops
counted in `stats.drops_by_field` that nothing reads; a silent CPU device
fallback). Diagnosis required monkeypatching `step` and `VoxelGrid.__init__`
to learn *whether ticks fired at all*. Every question had the same shape:

> for module M: what arrived, what did each tick resolve to, did step run,
> how long did it take, what came out — and if nothing, why?

All of that is knowable at one chokepoint per module (aligner + step call
site). Nothing about it requires module cooperation.

## Relationship to T9 (health)

Same counters, same off-tick-path consumer thread, different cadence and
consumer:

| | T9 health | T15 debug recorder |
|---|---|---|
| payload | 1 Hz summary row (rates, contract_ok, p95) | per-tick event records (+ optional payloads) |
| transport | rim `health` stream / topic | mem2 sqlite in the run-log dir |
| consumer | dashboards, autoconnect, watchdogs | forensics, replay, fixtures, CLI report |
| default | on (live), off under `over()` | decisions layer on; payload layers opt-in |

T9 lands unchanged. T15's writer is the SAME per-session thread as T9's pacer
(one off-path thread per module session, two jobs). Persisting health topics
long-term stays a plain recorder module subscribing `health` — no special path.

## Doctrine

1. **No mem2 on the tick path.** The tick path only appends a small plain-data
   record to a bounded per-session ring. The pacer thread drains ring → sqlite.
   Ring full ⇒ drop-oldest + increment a `debug_dropped` counter — the recorder
   must never block or slow a tick, ever.
2. **Under `over()` there is no thread** (T6 is sync): drain inline every N
   ticks and at teardown. Offline runs are bounded; this costs nothing.
3. **Wall clock stays out of records' keys.** Decision records are keyed by
   data-time `tick_ts` (+ a monotonically increasing `seq`). Durations
   (`step_ms`, `hold_ms`) come from the monotonic clock — durations are not
   timestamps.
4. **Capture what step SAW, not what the wire carried.** The rows layer records
   post-alignment In rows — that is what makes replay deterministic. Raw
   pre-alignment capture is a separate, rarely-needed layer (replaying the
   source recording re-derives it).

## Capture layers

| layer | contents | cost | default |
|---|---|---|---|
| `decisions` | one `TickDecision` per tick attempt | ~100 B/tick | ON when debug enabled |
| `rows` | the In row step received + the Out row it returned | payload-sized (clouds!) | opt-in, per-module filter, `thin=N` |
| `state` | Mealy State snapshot at every emit (or every N ticks) | tiny (State is plain data) | ON with `rows` |
| `raw` | pre-alignment port messages | largest | opt-in, rare |

Plus one `config` record per run: module class qualname + `config.model_dump()`
+ schema version (T10 kinship) — enough to reconstruct the module for replay.

`TickDecision` (plain data):

```python
class TickDecision(NamedTuple):
    seq: int                 # tick attempt counter, 0-based
    tick_ts: float           # data time of the trigger message
    fired: bool              # False = dropped before step
    drop_reason: str | None  # 'missing-required:pose' | 'tf-unresolvable:pose'
                             # | 'nonmonotonic:<port>' | 'future-ts:<port>' | ...
    ports: dict[str, float]  # per secondary/tf field: resolved sample ts (nan = default)
    step_ms: float | None    # monotonic duration of step (None if not fired)
    emitted: bool            # step returned an Out row (vs None skip)
    hold_pulls: int          # tf items pulled while holding this tick
```

## Toggle & API

Capture (three equivalent spellings; most specific wins):

```python
m.over(scan=..., debug=True)                          # decisions only
m.over(scan=..., debug=pm.Debug(rows=True, thin=5))   # + thinned payloads
graph.over(...).debug(...)                            # all members, path-prefixed
DIMOS_PURE_DEBUG=1                                    # global, decisions
DIMOS_PURE_DEBUG="nav_stack/voxel_mapper2:rows,thin=5;planner:rows"
```

Live: a coordinator/global-config flag with the same grammar; the rim reads it
at session start (T8) and hands the ring to the T9 pacer thread.

Storage: `DIMOS_RUN_LOG_DIR/debug.db` — ONE mem2 store per run, streams
namespaced by module path: `<module_path>/decisions`, `.../in`, `.../out`,
`.../state`, `.../raw/<port>`, `.../config`. One store so the whole run loads
as one object; multi-process live writers go through sqlite WAL (debug write
rates are low — decisions are ~100 B at tick rate, rows are thinned), with a
busy timeout; if WAL contention ever shows up in practice, fall back to
per-process staging dbs merged at teardown — an implementation detail behind
`debug.load()`, never a layout the user sees.

Read + replay:

```python
from dimos.pure import debug

run = debug.load(run_dir)          # or debug.latest()
run.summary()                      # aggregate end-of-run report (CLI prints this)

mod = run.module("nav_stack/voxel_mapper2")
mod.summary()                      # ticks fired/dropped/emitted, drops by reason
mod.ticks()                        # Iterator[TickDecision]
mod.drops(reason="tf-unresolvable")
mod.in_rows(); mod.out_rows()      # rows layer (empty unless captured)

mod.replay()                       # rebuild module from `config`, re-drive step
mod.replay(VoxelMapper2(voxel_size=0.2))   # modified module over recorded inputs
mod.fixture(seq=4832, path="test_regression_4832.py")  # pytest fixture export
```

`replay()` is the payoff purity buys: recorded In rows re-drive `step`
deterministically — REPL, debugger, or changed code — with no robot and no
graph. `fixture()` turns any recorded tick into a committed regression test.

CLI is a thin shell over the reader:

```
dimos pure debug <run_dir>                      # aggregate summary table
dimos pure debug <run_dir> -m planner --drops   # drops by reason for one module
dimos pure debug <run_dir> -m planner --tick 4832   # dump decision (+ row if captured)
```

## Plug points (the seams)

- **T5 `Aligner`**: gains an optional `on_decision` callback (None ⇒ zero
  cost). `_fire` and the pull loop already know every drop reason; today they
  only bump counters.
- **T6 `run_over`**: wraps the step call (step_ms, emitted), owns inline
  draining for `over()`.
- **T8 rim**: allocates the ring at session start, hands it to the pacer,
  flushes at teardown (resource-style lifecycle).
- **T9 pacer thread**: drains the ring each wake; unchanged health emission.

## Hot-reload kinship (the `state` layer)

Hot reload (planned) restarts a module with new code and hands it its prior
state. For Mealy modules the handoff artifact is the **State snapshot at a
tick boundary** — NOT the last input (inputs keep flowing live; State is the
plain-data thing the doctrine made checkpointable). That artifact is exactly
the `state` layer above, so one mechanism serves three consumers:

- **debug replay**: `mod.replay(from_seq=N)` starts mid-run from the recorded
  State instead of stepping from tick 0;
- **hot reload**: restore latest State + resume — same snapshot, same
  tick-boundary rule as T10;
- **T10 checkpoint**: `{schema_version, qualname, config dump, State, cursor}`
  is the `config` record + a `state` record; T15 stores the same shape.

Resources are NOT in the handoff — the reloaded module rebuilds them cold
(the VoxelMapper2 docstring tradeoff; PGO's keyframe-only map is
reload-friendly precisely because the grid is derivable from pgo state, while
gtsam history is not). A code change that alters the State schema hits T10's
`schema_version` gate: explicit migration or cold start, never silent
field-dropping.

## End-of-run report (the free 80%)

With only the decisions layer, `run.summary()` / the CLI table is the
"end-of-run alignment report" from `api_improvements.md` — rows vs ticks per
module, drops by field AND reason, tf coverage. `over()` and the rim SHOULD
log a one-line WARNING at teardown whenever `ticks > 0 and rows == 0`,
regardless of whether debug capture was enabled.

## Watch out for

- Ring overflow policy is drop-oldest + counter — NEVER backpressure the tick
  path. A recorder that slows the system lies about the system.
- Rows-layer copies of pointclouds can double run I/O — `thin=N` and
  per-module filters are load-bearing, not niceties.
- decisions must stay cheap enough to leave on: no dict churn per tick when
  disabled (`on_decision=None` short-circuit), preallocated record building
  when enabled.
- One shared debug.db across forkserver workers means cross-process sqlite
  writes: WAL + busy_timeout, short transactions from the drain thread only.
  Never share a connection handle across processes.
- `config` record needs `schema_version` from day one (T10's lesson).
- Replay of resource-holding modules builds FRESH resources (grid, gtsam) —
  replay determinism covers step logic, not resource-internal nondeterminism
  (CUDA reductions may differ at float epsilon; document, don't promise
  bit-equality).

## Acceptance bar

The `PGOVoxelMapper` zero-rows incident, re-run with `DIMOS_PURE_DEBUG=1`,
is diagnosable from the CLI alone: the summary names the dropped field and
reason (`tf-unresolvable:pose`, 598/600), and `--tick 0` shows the tf frontier
past the tick with zero port resolutions. Then: capture `rows` on
`voxel_mapper2` over a 600-scan slice, `mod.replay()` reproduces the recorded
`out` stream, and `mod.fixture()` emits a runnable pytest file.

---

# Implementation contract (typed spec — 2026-07-21)

Normative skeleton: **`dimos/pure/debugrec.py`** (mypy --strict clean; bodies
`NotImplementedError`, exact signatures binding). Acceptance tests, skip-gated
`"T15 impl pending"`: **`dimos/pure/test_debugrec.py`**. The design above is
settled; this section fixes the seams, the storage layout, and the threading
contract. Line anchors are as of `feat/ivan/pure-modules` @ 2026-07-21 — quote
matches, not line numbers, are authoritative if the files drift.

The seam diffs below are SPEC'D, NOT APPLIED — `align.py`, `drivers.py`,
`typing.py`, `graph.py`, `rim.py` are untouched on this branch; the
implementer applies them exactly as written.

## C1. File name: `debugrec.py`, not `debug.py`

`debug.py` shadows nothing today, but the *name* `debug` is the feature's API
spelling — the `over(debug=)` kwarg, the `DIMOS_PURE_DEBUG` grammar, and (if
Q3 lands that way) a `pm.Debug` export — and the pm surface re-exports names,
never submodules (house rule, `dimos/pure/__init__.py` docstring). A
`dimos.pure.debug` submodule would bind the module object onto the package at
first import and make `from dimos.pure import debug` return a module where
every sibling name is a class/function. Same shape as tfbuffer-not-tf
(t11-tf.md §18.1). Reader spelling becomes:

```python
from dimos.pure import debugrec
run = debugrec.load(run_dir)
```

Layering: `debugrec.py` is engine-level but keeps module scope light — stdlib
+ `dimos.pure.typing` protocols + `setup_logger` only. memory2 is imported
LAZILY inside `DebugWriter`/`DebugRun` bodies; `align.py`/`drivers.py` never
import debugrec at module scope (they receive callables / do lazy imports),
so no cycles and the zero-engine pm-surface import test stays intact.

## C2. Storage layout — with one forced adjustment

**Discovery (adjusts the spec's storage sentence):**
`dimos/memory2/utils/validation.py::validate_identifier` enforces
`^[A-Za-z_][A-Za-z0-9_]*$` on stream names — `<module_path>/decisions` is not
a legal physical mem2 stream name (no `/`, no `.`). The logical namespacing
stays the API; physically:

- The writer sanitizes each logical name to an identifier (chars outside
  `[A-Za-z0-9_]` → `_`, `d_` prefix, numeric suffix on collision) and appends
  one `StreamMeta` row per created stream to a **`debug_meta`** manifest
  stream in the same db.
- The reader resolves logical → physical **only** via the manifest; the
  encoding is never re-derived and never user-visible (the spec's
  "implementation detail behind `debug.load()`" clause covers this).

Streams per module path `P` (logical names):

| logical | payload | obs ts |
|---|---|---|
| `P/decisions` | `TickDecision` | `tick_ts` |
| `P/in`, `P/out` | `RowRecord` (seq, tick_ts, row) | `tick_ts` |
| `P/state` | `StateRecord` (seq, tick_ts, state) | `tick_ts` |
| `P/raw/<port>` | the message itself | msg `ts` |
| `P/config` | `ModuleConfigRecord` (exactly one) | 0.0 |

Rules:

- `Stream.append(payload, ts=...)` MUST always pass `ts` explicitly — its
  default is `time.time()` (wall clock), which doctrine #3 bans from records.
- Codec: mem2's `codec_for` fallback → `PickleCodec` for the record
  NamedTuples and bundle rows (Q2). Raw streams may auto-select the lcm codec
  when the payload type carries `lcm_encode`/`lcm_decode` — fine either way.
- WAL: mem2's `open_sqlite_connection` already sets
  `journal_mode=WAL` + `synchronous=NORMAL`; stdlib sqlite3's default 5 s
  busy timeout covers debug-rate contention (Q8). Connections are never
  shared across processes — each process's `DebugWriter.open` builds its own
  `SqliteStore` on the same file.
- `ModuleConfigRecord.schema_version = DEBUG_SCHEMA_VERSION = 1` from day one
  (T10's lesson).

## C3. Threading contract

| context | may do | must never do |
|---|---|---|
| tick path (run thread): `Aligner` hook, `RunHooks.on_step/on_state`, `tap_*` | build plain records, `DebugRing.push` (O(1), short mutex) | touch mem2, allocate per-tick when disabled (`None` short-circuit), block |
| drain caller — live: T9 pacer (interim: `DebugWriter.start_pacer` daemon); over(): the run thread at the `OVER_DRAIN_EVERY` boundary inside `tap_out_rows` and at teardown | `ring.drain()`, mem2 appends, short transactions | hold the ring mutex across I/O (pop batch first, then write) |
| teardown (`DebugSession.close`) | final drain, flush ring `dropped` counter (logged + a tag on the decisions stream), unregister | raise into the run's teardown (log instead) |

Ring overflow is drop-oldest + `dropped` counter — never backpressure
(§Watch out for). `seq` is the per-session 0-based tick-attempt counter,
assigned in `DebugSession.on_decision`; a fired decision is held pending and
completed by `on_step` (same thread, same tick — the aligner fires, then the
driver steps, before the next `__next__`), so no cross-thread handoff exists
on the tick path.

## C4. Seam — T5 `Aligner.on_decision` (`dimos/pure/align.py`)

**Anchor 1 — `Aligner.__init__` signature (align.py:326):**

```python
    def __init__(
        self,
        in_type: type[InT],
        streams: Mapping[str, Iterable[Stamped]],
        *,
        tf: TfContext | None = None,
    ) -> None:
```

gains `on_decision: Callable[[float, bool, str | None, dict[str, float], int], None] | None = None`
(the `DecisionHook` shape — builtins only, so align.py imports nothing new),
stored next to `self._held_tick_ts = None` (align.py:414) as
`self._on_decision = on_decision`. The `align()` wrapper (align.py:633–640)
threads the same keyword.

**Anchor 2 — required-secondary drop in `_fire` (align.py:479–483):**

```python
        if missing_required:
            stats.ticks_dropped += 1
            for field_name in missing_required:
                stats.drops_by_field[field_name] = stats.drops_by_field.get(field_name, 0) + 1
            return None
```

before `return None`, when the hook is set:
`hook(tick_ts, False, "missing-required:" + ",".join(missing_required), ports, 0)`.

**Anchor 3 — the tf hold + drop (align.py:496–516):** the hold loop

```python
            while (
                unresolved
                and ctx is not None
                and ctx.frontier <= tick_ts + ctx.buffer.horizon
                and ctx.pull()
            ):
```

counts iterations into a local `hold_pulls` (only when the hook is set —
zero work otherwise); the unresolved-drop branch (align.py:506–516) calls
`hook(tick_ts, False, "tf-unresolvable:" + ",".join(sorted(unresolved)), ports, hold_pulls)`
alongside its existing `_DBG.warning`.

**Anchor 4 — the emit path (align.py:522–525):**

```python
        resolved[self._tick.name] = tick_item
        stats.rows_emitted += 1
        return self._in_type(ts=tick_ts, **resolved)
```

before `return`: `hook(tick_ts, True, None, ports, hold_pulls)`. `step_ms`
and `emitted` are NOT the aligner's to know — the session holds the fired
decision pending until the driver's `on_step` completes it (C3).

**`ports` dict** (built only when the hook is set — no per-tick dict churn
when disabled, §Watch out for): for each secondary, the resolved sample's ts
(`p.head_ts`/`p.newest_ts` for latest exact/committed hits, `tick_ts` for
interpolated brackets and exact interp hits), `float("nan")` when the default
was used; for each tf field, `tick_ts` when resolved, `nan` on default.

**Anchor 5 — tick-port ingestion drops in `__next__` (align.py:575–586):**

```python
                    if math.isfinite(p.last_ts) and ts > p.last_ts + _FUTURE_TS_OUTLIER_S:
                        ...
                        continue
                    if ts <= p.last_ts:
                        p.stats.dropped_nonmonotonic += 1
                        continue
```

when `p is tick` and the hook is set, these are dropped tick ATTEMPTS
(a lost trigger = a tick that never was): call
`hook(ts, False, f"future-ts:{p.name}", {}, 0)` /
`hook(ts, False, f"nonmonotonic:{p.name}", {}, 0)` respectively. Secondary-
port sample drops stay `PortStats` only (they drop samples, not ticks).

## C5. Seam — T6 `RunHooks` + `run_over` (`dimos/pure/drivers.py`)

**Anchor 1 — `RunHooks` (drivers.py:103–120)** gains two optional callables,
same seam style as the T7 warmup/teardown fields:

```python
    on_step: Callable[[float, float, bool], None] | None = None   # (tick_ts, step_ms, emitted)
    on_state: Callable[[float, Any], None] | None = None          # (tick_ts, new_state) after an emit
```

**Anchor 2 — sync step timing.** In `_stateless_rows` (drivers.py:361–374)
and `_mealy_rows` (drivers.py:401–414), around

```python
        try:
            out = module.step(row)
```

when `hooks.on_step is not None`, bracket the call with
`time.perf_counter()` (monotonic; durations only — doctrine #3) and after the
None/skip decision call `hooks.on_step(row.ts, step_ms, out is not None)`.
`_mealy_rows` additionally calls `hooks.on_state(row.ts, state)` after a
non-None emit when `hooks.on_state` is set (the state layer's tick-boundary
snapshot — T10/hot-reload kinship).

**Anchor 3 — async step timing.** In `_drive_async_rows` (drivers.py:486):

```python
                task = asyncio.ensure_future(module.step(row))  # Awaitable, not Coroutine
```

when `hooks.on_step` is set, wrap in a timing coroutine measuring start →
completion inside the task (window wait excluded, overlap-safe); the hook
fires at the AWAIT_HEAD emit/skip decision (drivers.py:509–516). A raised
step (D5 dropped tick) fires `hooks.on_step(ts, step_ms, False)` too — the
decision record is how the drop stays visible. Fold: no per-step timing
(`step_ms` stays None; decisions come from the aligner alone).

**Anchor 4 — `run_over` (drivers.py:175–182):**

```python
def run_over(
    module: Any,
    spec: StepSpec,
    streams: Mapping[str, Streamable],
    *,
    tf: Any | None = None,
    hooks: RunHooks | None = None,
) -> Iterator[Any]:
```

gains `debug: Any | None = None` — a built `debugrec.DebugSession` (runtime-
untyped like `module`/`tf`; static types live on `over()`). When not None:

1. `debug.record_config(module)` once, before any pull.
2. The aligner is built with `on_decision=debug.on_decision`
   (anchor: `rows = align(spec.in_type, {...}, tf=ctx)`, drivers.py:209),
   and when the rows layer is on, `rows = debug.tap_in_rows(rows)` — the
   post-alignment capture point (doctrine #4).
3. `hooks.on_step = debug.on_step`; `hooks.on_state = debug.on_state` when
   the state layer is on (Mealy only).
4. The returned iterator is wrapped `debug.tap_out_rows(driver)` — captures
   stamped Out rows (rows layer), drains inline every `OVER_DRAIN_EVERY`
   ticks, and closes the session in its `finally` (doctrine #2: no thread
   under over()).

**Anchor 5 — the unconditional teardown warning.** `_finalized`
(drivers.py:323–334):

```python
    try:
        hooks.warmup()
        yield from inner
    finally:
        try:
            close = getattr(rows, "close", None)
            if close is not None:
                close()
        finally:
            hooks.teardown()
```

after `hooks.teardown()`: lazy-import debugrec,
`msg = silent_run_warning(target, hooks.ticks, hooks.emits, drops_by_field)`
with `drops_by_field` from `getattr(rows, "stats", None)` (the Aligner is
`rows` here) and `_LOG.warning(msg)` when not None. This runs with or
without debug capture (§End-of-run report) — the one log line that would
have turned the 45-minute bisect into a read.

## C6. Seam — `over(debug=)` (`dimos/pure/typing.py`)

The four `over` overloads + body (typing.py:235–256) each gain
`debug: bool | str | Debug | None = None` (TYPE_CHECKING import of `Debug`
from `dimos.pure.debugrec`, next to the `TfSource` pattern). Body:

```python
    def over(self: Any, *, tf: Any = None, debug: Any = None, **streams: Streamable) -> Iterator[Any]:
        from dimos.pure.debugrec import session_for   # lazy: sanctioned edge
        ...
        return run_over(self, step_spec(type(self)), streams, tf=tf,
                        debug=session_for(self, debug=debug))
```

`session_for` returns None when capture is off (arg False, or arg None with
no env toggle) — run_over then wires NOTHING (zero tick-path cost, the
pinned Tagger floor is untouched).

## C7. Seam — `PureGraph` runs (`dimos/pure/graph.py`)

Spelling per §Toggle & API: `graph.over(...).debug(...)` — a `GraphRun`
method (graph.py:1228), returning `self`, storing the coerced `DebugConfig`.
Both member-driver call sites —

```python
            gen = run_over(module, spec, streams, tf=tfs.get(path), hooks=RunHooks())
```

(`_RunInstance._build`, graph.py:1077; `_TerminalRun._build`, graph.py:1166)
— resolve `resolve_debug(config, path)` per member (paths are the T13
member paths, dot-separated, `named()`/snake_case — C9), build sessions via
`session_for(module, path=path, debug=...)`, ONE shared `DebugWriter` per
run, and pass `debug=` into `run_over`. Fresh `EdgeStream` iterations are
fresh runs (T13 §0.2) — each gets fresh sessions; seq restarts with the run.

## C8. Seam — T8 rim session + T9 pacer (`dimos/pure/rim.py`)

**Session allocation** — `_LiveSession.start()` (rim.py:671), before the
aligner is built:

```python
            self._aligner = align(self._spec.in_type, feeds, tf=self._tf_ctx)
```

(rim.py:689) becomes `align(..., on_decision=...)` when
`self._debug = debugrec.session_for(module, path=..., live=True)` returned a
session (lazy import, the tfbuffer pattern); `hooks.on_step`/`on_state` are
set before `_dispatch` (rim.py:692), and the driver is wrapped
`self._debug.tap_out_rows(self._driver)` next to the `tf_out_tap` wrap
(rim.py:693–696). Live `path` defaults to `default_path(module)`
(snake_case class name); the T8 legacy adapter passes its coordinator module
name instead when deploying (same identifier autoconnect sees).

**Drain thread** — the rim starts NO thread of its own. Session registration
calls `writer.start_pacer()` (interim daemon, Q6) until T9 lands; the T9
integration point is one call — the pacer thread invokes
`DebugWriter.drain()` each wake (same thread, two jobs, per §Relationship to
T9) and `start_pacer` is then never invoked. Nothing here depends on T9
landing first.

**Teardown flush + warning** — `_run`'s `finally` (rim.py:831–850), after
the driver close and tf release, before `self._seal_ingestion()`:
`self._debug.close()` (final drain — flushes the ring even on session death)
and the same unconditional `silent_run_warning` using
`self._hooks.ticks/emits` + `self._aligner.stats.drops_by_field` —
regardless of whether `self._debug` exists. `stop()` needs no extra step:
step 4–5 already run `_run`'s finally on the session thread.

## C9. Path, seq, and grammar decisions

- **Module path** = the T13 member path exactly as the graph mints it:
  dot-separated, `named()` overrides, snake_case defaults, `_2` dedup
  (graph.py `_member_name`). A lone `m.over()` uses
  `default_path(m)` = `snake_case(type(m).__name__)`. The doc's `/`
  spellings above remain valid input: env grammar and CLI accept `/` and
  normalize to `.` (Q11).
- **seq** is per-session and 0-based, counting tick ATTEMPTS: every `_fire`
  outcome plus tick-port ingestion drops (C4 anchor 5). `ticks()` is
  therefore a superset of `AlignStats.ticks_fired`.
- **No wall clock in record keys** (doctrine #3): keys are `(seq, tick_ts)`;
  `step_ms` from `time.perf_counter()`; the only wall clock in the whole
  feature is the interim drain pacer's sleep (the T9 exemption).

## C10. CLI

`dimos pure debug <run_dir>` / `-m <path> --drops` / `--tick N` — a thin
typer shell over `DebugRun.summary()` / `ModuleDebug.summary()/drops()/
ticks()`, no logic of its own. Placement Q10 (default: `dimos/pure/cli.py`
sub-app registered in `dimos/robot/cli/dimos.py`).

## C11. Acceptance mapping

`dimos/pure/test_debugrec.py` pins the bar; headline tests:

- `test_zero_rows_incident_diagnosable_from_summary` — THE incident, CLI/
  summary-only diagnosis (spec §Acceptance bar).
- `test_replay_reproduces_recorded_out_stream`,
  `test_replay_from_seq_seeds_recorded_state` — replay + state-layer seeding.
- `test_ring_overflow_drops_oldest_without_blocking`,
  `test_tick_path_never_touches_store` — doctrine #1.
- `test_env_grammar_*`, `test_resolve_most_specific_prefix_wins` — toggles.
- `test_silent_run_warning_fires_without_debug_enabled` — the free 80%.
- `test_fixture_export_is_runnable` — fixture()'s committed-regression bar.

---

# Questions for Ivan

The implementer proceeds on DEFAULT unless overridden — never stalls.

QUESTION 1: mem2 stream names can't hold `/` or `.` (`validate_identifier`) — sanitize with a manifest, or relax memory2?
DEFAULT: sanitize to safe identifiers + a `debug_meta` manifest stream; reader maps only via the manifest (zero memory2 changes, encoding invisible behind `load()`).
OPTIONS: a) sanitize + manifest — no cross-package touch, raw `sqlite3` sessions see mangled names; b) relax `validate_identifier` + quote identifiers throughout memory2 — pretty everywhere, touches every mem2 SQL path for a debug feature; c) flat tag-based layout (one stream per layer, path as a tag) — fewer tables, loses per-module streams and mem2 stream tooling symmetry.

QUESTION 2: storage codec for `TickDecision`/`RowRecord`/`StateRecord`?
DEFAULT: pickle (mem2 `codec_for` fallback — NamedTuples and bundle rows store whole; `schema_version` gates migrations).
OPTIONS: a) pickle — zero work, Python-only readers; b) a dedicated lcm msg for TickDecision — cross-language + plot tooling on decisions, new msg def while rows stay pickled anyway; c) new JSON codec — greppable db, slow and wrong for row payloads.

QUESTION 3: surface spelling — what does pm export?
DEFAULT: `pm.Debug` only (the `over(debug=)` value); the reader stays module-qualified `from dimos.pure import debugrec`.
OPTIONS: a) `pm.Debug` only; b) also `pm.debug_load`/`pm.debug_latest` aliases — flatter REPL, two names for one thing; c) name the file `debug.py` and bless `from dimos.pure import debug` — matches the doc's sketch verbatim, breaks the no-submodule-re-exports house rule.

QUESTION 4: decisions capture default when nothing is toggled?
DEFAULT: `over()` = off (tests/evals write nothing); live rim sessions = decisions-only ON whenever `DIMOS_RUN_LOG_DIR` is set (the coordinator black-box promise in §Relationship to T9).
OPTIONS: a) split default as above; b) off everywhere until opted in — simplest, forfeits the flight recorder; c) on everywhere including over() — a debug.db appears in every pytest tmp cwd.

QUESTION 5: ring capacity default?
DEFAULT: 4096 events per session (decisions-only ≈ hundreds of KB; > 30 s of headroom at 100 Hz against a 0.5 s drain).
OPTIONS: a) 4096; b) 1024 — tighter, risks drops the moment the rows layer buffers a burst of clouds; c) derived (drain period × expect_hz × margin) — adaptive, more machinery than the problem merits.

QUESTION 6: live-mode drain before T9 lands?
DEFAULT: interim writer-owned daemon pacer (`start_pacer`, 0.5 s), started on first live session registration; T9's pacer takes over by simply calling `DebugWriter.drain()` and never starting the interim.
OPTIONS: a) interim pacer thread; b) no live capture until T9 — cleaner, delays the black box for no engine benefit; c) drain on the session thread every N ticks — puts sqlite on the live tick path, violates doctrine #1.

QUESTION 7: over() debug.db location when there is no run dir?
DEFAULT: `Debug.db` override → `session_for(run_dir=)` → `$DIMOS_RUN_LOG_DIR` → `./debug.db`.
OPTIONS: a) that chain; b) temp dir + printed path — no cwd pollution, artifacts get lost; c) refuse without an explicit path — predictable, hostile in the REPL.

QUESTION 8: WAL / busy-timeout ownership?
DEFAULT: none — mem2's connections are already WAL + synchronous=NORMAL and stdlib sqlite3 defaults to a 5 s busy timeout; the spec's staging-db fallback stays reserved for observed contention.
OPTIONS: a) nothing now; b) writer issues `PRAGMA busy_timeout=10000` on its connections — cheap insurance, papering over a rate problem if it ever triggers; c) per-process staging dbs merged at teardown now — complexity before evidence.

QUESTION 9: fixture file shape?
DEFAULT: one generated test `.py` + one `.pickle` sidecar (`{config, in_row, state?, out_row}`); the test rebuilds the module from config, steps the recorded In row, asserts the recorded Out — self-contained, committable.
OPTIONS: a) py + sidecar; b) everything inline (base64 in the .py) — single file, unreviewable diffs for cloud-sized rows; c) reference the source debug.db by path — smallest, breaks the moment the run dir is cleaned.

QUESTION 10: CLI placement?
DEFAULT: new `dimos/pure/cli.py` typer sub-app; one `main.add_typer(pure_app, name="pure")` line in `dimos/robot/cli/dimos.py`.
OPTIONS: a) sub-app file; b) inline commands in `dimos.py` — no new file, grows an already-long module; c) separate console script — bypasses the `dimos` entry point users already know.

QUESTION 11: module-path separator in env grammar / CLI?
DEFAULT: canonical is T13's dot path (`nav_stack.voxel_mapper2`); `/` accepted everywhere and normalized to `.` — this doc's own examples keep working.
OPTIONS: a) both, normalize to dots; b) dots only — rejects the examples above; c) slashes only — diverges from T13 edge addressing (`<member_path>.<field>`).

## Triage resolutions (Ivan, 2026-07-21)

- **Q1 + Q11 (merged): relax memory2.** `validate_identifier` additionally
  allows `.` in stream names; dots are the canonical path separator
  everywhere (`/` accepted on input, normalized to dots). The
  sanitize-plus-manifest design DIES; the reader enumerates modules/layers by
  prefix-splitting `list_streams()`. Implementer MUST verify sqlite
  table-name quoting end-to-end (base tables + derived `_rtree`/`_blob`/vec
  tables + blobstore filenames) — a dotted name in an unquoted identifier is
  a SQL break.
- **Q2**: default — pickle via `codec_for` fallback.
- **Q3**: option (a) — export `pm.Debug`, `pm.debug_load`, `pm.debug_latest`
  (flat callables on the pm surface; update `test_pm_surface` pin).
- **Q4 + Q7 (merged)**: we own the debug location; it rides the EXISTING
  run-log machinery — `DIMOS_RUN_LOG_DIR/debug.db` when set, else the repo
  `logs/` run dir (where `main.jsonl` already goes). Untoggled policy stands
  as spec'd: over() off; live decisions-only ON when a run-log dir exists.
- **Q5**: default 4096 (explained to Ivan: drop-oldest of debug records only,
  ~400 KB decisions ceiling, rows-layer pinning bounded by capacity during
  writer stalls; constant, revisit when reality disagrees).
- **Q6, Q8, Q9, Q10**: defaults (interim writer-owned pacer retired by T9;
  mem2 WAL as-is; fixture = test .py + pickle sidecar; CLI = typer sub-app
  `dimos pure`).

## Implementer notes (deviations recorded, 2026-07-21)

QUESTION 12: the silent-run warning's tick count — hooks.ticks (C5 anchor 5) or the aligner's attempt count?
DEFAULT (taken): the aligner's `stats.ticks_fired`. C5/C8 literally say `hooks.ticks`, but `hooks.ticks` counts DRIVER rows — when every tick drops at the aligner (the exact PGO zero-rows incident this warning exists to catch) the driver sees 0 rows, so `hooks.ticks == 0` and the warning would never fire. `stats.ticks_fired` counts every tick that reached `_fire`, so `test_silent_run_warning_fires_without_debug_enabled` and the acceptance bar pass. `rows` stays `hooks.emits` (driver output).
OPTIONS: a) aligner ticks_fired (taken — fires on the incident); b) literal hooks.ticks (never fires on all-drop runs — fails the acceptance test).

QUESTION 13: the silent-run warning's placement — C5 says inside `_finalized`; I put it in a `run_over`-owned wrapper (`_warned`) and, for live, the rim's own finally (C8).
DEFAULT (taken): `_warned` around the returned `run_over` iterator + `_LiveSession._debug_teardown`. Reason: `_finalized` is shared with the rim and its `rows` param becomes the in-row TAP (not the Aligner) once the rows layer is on, so `getattr(rows, "stats")` would miss `drops_by_field`. `_warned` closes over the real Aligner. Intent (over()/graph AND live both warn, with or without debug) is fully satisfied — verified by the e2e smoke and the acceptance test.
OPTIONS: a) run_over wrapper + rim finally (taken); b) thread the aligner+module into `_finalized` (4 driver signatures + rim call sites change — more blast radius).

Note: `StreamMeta` stays exported (pinned skeleton surface) but is DEAD per the Q1 relaxation — the reader enumerates via `_parse_stream(list_streams())`, never a manifest.
