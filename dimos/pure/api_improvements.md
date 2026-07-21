# Pure modules: API improvements & debug systems

Field notes from building `PGOVoxelMapper` + `NavStackPGO` (2026-07-21) — every
item below bit for real during that session. Roughly ordered by pain.

## Architecture issues

### Silence is the default failure mode
The worst incidents all produced zero output and zero noise:

- A tick stamped before the first tf sample made the hold drain the ENTIRE tf
  stream, horizon-evict everything, and starve every later tick — the run
  "succeeded" with 0 rows. (Fixed: bounded hold + warning, `align.py`.)
- Required-tf drops were counted in `stats.drops_by_field` but nothing ever
  read them.
- `VoxelGrid` silently mapped `device="GPU:0"` → CPU (`startswith("CUDA")`
  check) — wrong config degraded instead of raising. (Fixed: `"CUDA:0"`
  defaults; the loud-validation lesson stands.)
- The one breadcrumb (device log) was gated off by `show_startup_log=False`.

The engine keeps good counters (`AlignStats`, `PortStats`, `TfStats`) — they
are write-only. Nothing at run end says "600 ticks fired, 0 rows emitted,
598 dropped on field `pose`".

### Hold-until-resolvable is unbounded consumption
"The pull IS the wait" let one unresolvable tick consume the whole upstream.
The tf hold is now bounded at `tick_ts + horizon`; the doctrine should be
general: any hold needs a give-up condition derived from what could still
change the answer.

### Guards are per-path, not systemic
The Go2 future-ts guard existed on aligner ports but not the tf side channel —
same stream pathology, two ingestion paths, one protected. There are now two
copies (`align.py`, `tfbuffer.py`); should be one shared stamp-gate at every
ingestion point. Everything hinges on stamp hygiene (75-day jumps, the
`Transform(ts=0.0)` wall-clock swap, payload-ts regressions eating 2/300
scans), but stamp validation is scattered ad hoc.

### The horizon is invisible global state
`DEFAULT_TF_HORIZON = 10.0` silently decided "the buffer now covers
[677..687]" — the mechanism behind the catastrophic stall — and nothing can
show that window. Load-bearing and unobservable.

## Debug systems needed

1. **End-of-run alignment report** (highest value). At `over()` / session end:
   rows emitted vs ticks fired, drops by field AND reason (nonmonotonic /
   future-ts / tf-unresolvable / missing-required), tf buffer coverage per
   edge. Loud warning whenever rows == 0 and ticks > 0. Would have turned a
   45-minute bisect into one log line.
2. **Per-tick trace mode** (`DIMOS_PURE_TRACE=1`): one line per tick — fired
   or dropped, and why. Was reimplemented by monkeypatching `step`; should be
   free.
3. **Step-latency stats per module** (max/p99) in the T9 health surface. The
   PGO loop-closure rebuild blocks its step for hundreds of ms; `min_hz`
   tolerates it but nothing measures it — live hitches get diagnosed by vibes.
4. **Recording stamp linter**: run over a db before an eval — ts regressions,
   >1 h jumps, duplicates, per-stream overlap windows. Would have
   pre-explained both `EXPECTED_TICKS=298` and the pre-tf first tick.

## API inconveniences

- **`Transform(ts=0.0)` → wall-clock swap** — the most-repeated footgun in the
  codebase; every construction site carries the `tf.ts = ts  # force`
  incantation. Kill the ctor behavior: this is a data-time system, wall-clock
  defaults are hostile.
- **Feeding tf in `over()` is manual**: odom→Transform conversion boilerplate
  (with the ts dance) appears in every test and eval. `over(tf=...)` should
  accept a PoseStamped stream + frame pair.
- **Module subclassing is guess-and-check**: bare `State` doesn't resolve in a
  subclass (needs `VoxelMapper2.State`), In-narrowing needs
  `# type: ignore[override]`, In/Out MRO-merge rules are discoverable only via
  error messages (excellent ones, but still). Needs a "how to subclass a
  module" recipe.
- **No optional graph exports** (`[graph-unset-output]`) — forced the
  `NavStackPGO` subclass. The subclass ended up the cleaner shape, but "flag
  on the graph" was the natural first spelling and the failure only surfaces
  at plan time, not class definition.
- **Mealy can't see stream end + tick-count cadence** — two dropped ticks meant
  a 600-scan slice never hit `n % 100 == 0` again, so no final map emission.
  Two fixes, both wanted:
  - **`finish(s) -> Out | None` hook** (ACCEPTED — DONE): optional Mealy
    method called once at stream exhaustion, giving the tail flush the fold
    shape already has. Restores `VoxelMapper` fold parity for `VoxelMapper2`
    and gives `PGOVoxelMapper` a final corrected-map emit.

    Implemented: `stepspec` classifies + validates an optional `finish` on
    Mealy classes (`[finish-not-mealy]`, `[finish-not-function]`,
    `[finish-params]`, `[finish-unannotated]`, `[finish-state]`,
    `[finish-return]`; `StepSpec.has_finish`). `drivers.drive_mealy(finish=...)`
    calls it once after the row loop and stamps the returned Out with the last
    consumed tick ts; `run_over` and the rim's `_dispatch` pass
    `spec.has_finish`. The emitted row flows the normal out-row path (T15 tap,
    tf_out, graph tees, rim egress), so graph downstream and `.save()` deliver
    the tail like any member output — no graph/rim change beyond that one flag.

    Decisions taken (proceeding on DEFAULT; none blocking):
    - **Q: fire on an empty stream (0 ticks consumed)?** DEFAULT: no — there is
      no last-tick ts to stamp, mirroring the fold's `last_ts is not None`
      guard. A 0-tick run emits nothing from either shape.
    - **Q: fire on early consumer close (generator `.close()`)?** DEFAULT: no —
      an abort is not a stream end. Falls out for free: `finish` sits after the
      `for` loop, so `GeneratorExit` at a suspended `yield` skips it. Also never
      on error unwind (a step raise propagates from the loop). Live: fires on a
      replay-fed source exhausting and on the graceful `stop()` drain, once,
      before resource teardown.
    - **Q: T15 rows-layer accounting?** DEFAULT: the finish emission is recorded
      via `tap_out_rows` (OUT layer) under the LAST fired tick's seq — it is not
      a tick attempt, so it gets no `TickDecision` and no `on_step`/`on_state`.
      `hooks.emits` counts it (so a batch-only module no longer trips the
      silent-run warning).
    - **Q: replay (`ModuleDebug.replay`)?** DEFAULT: does not fire `finish`
      (`drive_mealy` default `finish=False`; debugrec untouched to stay disjoint
      from the concurrent T16 agent). Replay reproduces per-tick steps, not the
      tail; the tail is already in the recorded OUT layer.
  - Data-time cadence (`emit_every_s`) as an alternative to tick counts —
    robust to drops.
- **Resource lifecycle has no recreate-mid-run story** — the loop-closure
  rebuild needed `VoxelGrid.reset()` because the engine owns disposal and the
  slot isn't reassignable. `reset()` worked; bless the pattern (or add engine
  support for swapping a resource mid-run).

## The debug tool (design sketch)

Investigate input msgs, ticks, outputs per pure module. Guiding insight:
**instrument the aligner, not the modules** — every interesting decision (tick
fired/dropped + why, per-port resolution, hold durations, step latency)
happens at one rim chokepoint per module; modules never cooperate.

**Substrate: mem2 sqlite.** Rows are already wire-safe msgs (module contract)
so LCM codecs exist; a recorded db is already the replay format
(`store.stream(...)` → `over()`); `range_seek` / `vis.plot` / CLI tooling all
apply. Purity then buys the killer feature: recording the POST-ALIGNMENT In
rows (what `step` actually saw) makes any module deterministically
re-drivable in isolation — REPL, debugger, or a modified module version —
without the robot or the rest of the graph. "Export tick N as a pytest
fixture" becomes mechanical.

**Three capture layers** (the toggle granularity):

| layer     | what                                                        | size  | when on |
|-----------|-------------------------------------------------------------|-------|---------|
| decisions | per tick: fired/dropped+reason, per-port resolved ts, step_ms | tiny  | always (flight recorder) |
| rows      | the In row step saw + the Out row it returned                | big   | debug toggle, per-module filter |
| raw       | pre-alignment port messages                                  | biggest | rarely; replay makes it redundant |

The decisions layer is the end-of-run report made persistent — cheap enough
to leave on in coordinator deploys as a black box. Rows need a filter
(`DIMOS_PURE_DEBUG=nav_stack/voxel_mapper2:rows`) and/or thinning or the
pointcloud copies double run I/O. Storage: ONE `DIMOS_RUN_LOG_DIR/debug.db`
mem2 store per run, streams namespaced by module path
(`<module_path>/decisions|in|out|state|config`); cross-process writes via
sqlite WAL. Full design: `tasks/t15-debug.md`.

**Views, in build order:**
1. **CODE** — thin query API over the db: `dbg.module("planner").ticks() /
   drops() / in_rows() / replay(step_fn, at=tick)`. Nearly free; replay needs
   it anyway.
2. **CLI** — `dimos pure debug <run>`: the summary report (rows vs ticks per
   module, drops by field+reason), `--module`, `--tick N` row dump. Trivial
   over (1).
3. **rerun as the "web" view** — tick/drop events and step-latency logged as
   scalar series on the same timeline as the data. A bespoke web UI is
   explicitly deferred until (1)+(2) prove insufficient.
