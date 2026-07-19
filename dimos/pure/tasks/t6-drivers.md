# T6 — Step drivers + `over()`

Status: spec. Implements against landed T1–T4 (`rows.py`, `config.py`,
`module.py`, `stepspec.py`, `typing.py`). Companion skeleton:
`dimos/pure/drivers.py`; tests: `dimos/pure/test_drivers.py` (module-skipped
until implementation). Source of truth: `puremodule_api_sketch3.py` §1–§3,
§6, §7, deployment sections; task boundary: `tasks/index.md` §T6. All design
calls made within T6's mandate are numbered D1–D16 in §16.

## 1. Scope and layer boundary

T6 owns the run engine's core: given a module instance, its `StepSpec`, and a
mapping of input streams, produce the typed `Iterator[Out]` that
`PureModule.over()` promises — alignment (T5) composed with a per-kind step
driver, engine ts-stamping, tick accounting, and airtight teardown.

**In scope**: `run_over()`, the four kind drivers, `RunHooks` (counters + the
T7 teardown seam), the run-error catalog, the `over()` body swap in
`typing.py`, and the `Streamable` tightening (T4 §13.1, delegated here).

**Out of scope**: alignment internals (T5 — consumed through the §3 boundary),
resources (T7 — only the seam is left), transports/live lifecycle (T8),
health rows (T9 — consumes the counters later), checkpoint (T10).

**Layering** (index cross-cutting rule, sharpened for this file):

- `drivers.py` is ENGINE. It may import the data layer — `rows`, `stepspec`,
  `typing` — and the engine sibling `align` (T5, lazily; §4.1). It must NEVER
  import `module.py` or `config.py`.
- Why: (a) drivers drive anything *step-shaped* — the protocols are
  structural, so tests drive plain classes with no `PureModule` anywhere
  (proved by `test_drivers.py`, which never imports `module.py`); (b) the
  class layer's only edge into the engine is `over()`'s lazy import (T4
  §4.4), and keeping the reverse edge nonexistent makes that inversion
  acyclic by construction, not by import-order luck.
- Consequence: `run_over` takes `module: Any`. The per-kind drivers recover
  precision instead — each is typed against its step protocol
  (`Stateless`/`Mealy`/`AsyncStateless`/`Fold` from `dimos.pure.typing`), so
  the untyped hop is exactly one function wide, and direct driver callers
  (tests, T8) keep full types.

## 2. Public API

All in `dimos.pure.drivers`. Exports (listed here, no `__init__.py` change in
this task; the `pm` surface adopts `PureModuleRunError` + `RunHooks` when T8
lands a runtime surface — T12 call):

```python
DEFAULT_MAX_INFLIGHT: Final[int] = 1

class RunRule(enum.Enum): ...                    # §10 slugs
class PureModuleRunError(RuntimeError): ...      # rule: RunRule | None
class StepError(PureModuleRunError): ...         # wraps user step failures

@dataclasses.dataclass
class RunHooks: ...                              # §9 counters + §8.4 seam

def run_over(
    module: Any,
    spec: StepSpec,
    streams: Mapping[str, Streamable],
    *,
    hooks: RunHooks | None = None,
) -> Iterator[Any]: ...

def drive_stateless(
    module: Stateless[_TInRow, _TOutRow],
    rows: Iterator[_TInRow],
    *,
    skips: bool,
    hooks: RunHooks,
) -> Iterator[_TOutRow]: ...

def drive_mealy(
    module: Mealy[_TState, _TInRow, _TOutRow],
    rows: Iterator[_TInRow],
    initial: _TState,
    *,
    skips: bool,
    hooks: RunHooks,
) -> Iterator[_TOutRow]: ...

def drive_async(
    module: AsyncStateless[_TInRow, _TOutRow],
    rows: Iterator[_TInRow],
    *,
    max_inflight: int,
    skips: bool,
    hooks: RunHooks,
) -> Iterator[_TOutRow]: ...

def drive_fold(
    module: Fold[_TInRow, _TOutRow],
    rows: Iterator[_TInRow],
    *,
    hooks: RunHooks,
) -> Iterator[_TOutRow]: ...
```

Typevar bounds (the typed-stamping trick):

- `_TInRow = TypeVar("_TInRow", bound=Stamped)` — every driver reads
  `row.ts` (the tick ts).
- `_TOutRow` for the three step drivers:
  `TypeVar("_TOutRow", bound="DataclassInstance")` (`_typeshed`,
  TYPE_CHECKING-only import — the `rows.py` precedent). Bound so
  `dataclasses.replace(out, ts=tick_ts)` typechecks under strict; T1 §3
  guarantees Out bundles satisfy it.
- `drive_fold`'s `_TOutRow` is bound `Stamped` instead — fold ts is read and
  validated, never replaced.
- `_TState = TypeVar("_TState")` — invariant, threaded opaquely.

Driver functions are **plain functions returning a not-yet-started
generator** — each composes its inner drive loop with `_finalized` (§8.2),
so every caller (run_over, tests, T8's live reuse) gets identical teardown
semantics; nothing runs until the first `next()` (§8.1). `run_over` is also
plain: its validation is eager (D1), its return value is the driver's
generator, unwrapped — drivers finalize themselves.

Ownership note: a driver takes ownership of `rows` — at teardown it calls
`rows.close()` if present (§8.2). Callers must not reuse a `rows` iterator
after the run ends.

Scalars, not spec (D2): drivers take `skips`/`max_inflight`/`initial` as
explicit parameters and never see `StepSpec` — `run_over` unpacks it. This
keeps drivers classification-agnostic (any step-shaped object + a bool), and
keeps `stepspec` imports out of every function but `run_over`.

## 3. The T5 boundary — assumed contract (reconcile at integration)

T5 is being designed in parallel. T6 consumes it through this minimal
surface; every assumption is labeled A* for reconciliation. The
`test_run_over_composes_align` test (§14) pins A1 executably — if T5 lands a
different shape, that test fails loudly at integration instead of silently
drifting.

- **A1 — signature.** `dimos.pure.align` exposes
  `align(in_type: type[Any], streams: Mapping[str, Streamable]) -> Iterator[Any]`
  yielding fully-constructed `in_type` rows, each stamped with its tick ts
  (`row.ts`), in tick order.
- **A2 — laziness.** *Calling* `align(...)` acquires nothing and pulls
  nothing from any stream; all work happens under iteration. (This is what
  lets `run_over` build the pipeline eagerly while preserving the §8.1
  nothing-before-first-`next()` invariant.) The returned iterator SHOULD
  support `.close()` (generators do); the driver calls it if present.
- **A3 — validation split.** T5 owns stream-value validation and the
  required-vs-`default=` missing-stream policy. T6 checks exactly one thing
  eagerly: *unknown* kwarg names against `spec.in_type.fields()` (T1 API) —
  safe set-membership with no policy content, huge DX gain (the error points
  at the `over()` call site, not the first `next()`).
- **A4 — accounting.** T5 keeps its own aligner-side counters (holds/drops at
  resolution). `RunHooks` here is driver-scoped; the two compose under T9.
  If T5 lands a shared hooks shape, unifying field naming is a rename-level
  follow-up, not a design conflict.
- **A5 — errors.** Alignment errors are T5's own named errors and pass
  through the driver **unwrapped** (they are engine errors, not step errors —
  §10's `StepError` never wraps them).
- **A6 — ordering.** Rows arrive tick-ordered per T5's total order. The
  drivers do not re-verify global ordering; the only driver-side ordering
  state is fold's causality cursor, which uses a running max (§7.3) so a T5
  equal-ts tie animal cannot break it.

## 4. `run_over` — composition and the eager/lazy split (D1)

```python
def run_over(module, spec, streams, *, hooks=None):
    hooks = RunHooks() if hooks is None else hooks
    # ── eager (at over() call time) ──
    _check_unknown_streams(spec, streams, module)      # [run-unknown-stream]
    if spec.kind is StepKind.ASYNC_STATELESS:
        max_inflight = _max_inflight(module)           # getattr + validate, §6.1
        _reject_running_loop(module)                   # [run-inside-loop], §6.2
    if spec.kind is StepKind.MEALY:
        initial = spec.state_type()                    # T3 guarantees State()
    from dimos.pure.align import align                 # lazy import, see below
    rows = align(spec.in_type, streams)                # lazy object (A2)
    # ── dispatch ──
    return <drive_* per spec.kind, with the scalars above>
    # drivers self-finalize (§8.2); run_over adds no wrapper of its own
```

- **Eager** (runs at the `over()` call): unknown-stream names (A3),
  `max_inflight` sourcing + validation (§6.1), running-loop rejection
  (§6.2), `State()` construction (D16 — a buggy user `State.__init__` errors
  at the call site, and T10's restore path later substitutes a snapshot by
  calling `drive_mealy(initial=...)` directly).
- **Lazy** (first `next()` onward): every stream pull, every step call, all
  acquisition. An `over()` result that is never iterated acquires nothing
  and therefore owes no teardown (§8.1).
- **`align` is imported lazily inside `run_over`** — symmetric with
  `over()`'s lazy drivers import: `drivers.py` stays importable and its
  eager-validation paths stay testable even where T5 hasn't landed, and the
  engine loads strictly on demand. This is permanent, not a stopgap.
- `hooks` is caller-injectable for tests/T8/T9; `over()` passes nothing →
  fresh `RunHooks` per run. Under plain `over()` the counters are internal
  (a consumer wanting them calls `run_over` directly, today's sanctioned
  spelling; a friendlier surface is a T9/T12 call).

Re-invokability: all run state lives in generator locals plus the per-run
`RunHooks`; the module instance is frozen (T2) and untouched. Calling
`over()` again builds a fresh pipeline; `align` re-`iter()`s the stream
objects, so re-runs need re-iterable streams (memory2 `Stream` is; a raw
generator supports exactly one run — documented, not detected).

## 5. Step-kind drivers (sync)

### 5.1 Stateless

Per row: `hooks.ticks += 1`; `out = module.step(row)` (positionally — T3/T4
settled positional-only data params).

- `out is None` and `skips` → `hooks.skips += 1`, next row.
- `out is None` and not `skips` → `[step-none-no-skip]` (§5.3), abort.
- else `hooks.emits += 1`; `yield dataclasses.replace(out, ts=row.ts)`.

Stamping is **unconditional** — T1 D11 adopted verbatim: user-set `ts` on a
step Out row (including an explicit sketch-style `ts=`) is overwritten, never
an error; `ts` is engine-owned for step modules. `UNSTAMPED` never escapes a
step driver.

Pull discipline: exactly one input row is pulled per loop iteration; nothing
is prefetched. Same-thread, same-frame execution — a step exception unwinds
through the driver per §10.2.

### 5.2 Mealy

State threading, from `initial` (== `State()` under `over()`):

```python
state = initial
for row in rows:
    hooks.ticks += 1
    state, out = module.step(state, row)
    # None/skips/stamp exactly as §5.1
```

- The driver holds exactly ONE state reference, rebound per tick — never
  copied, never aliased, never retained after the run (index: State is
  immutable plain data; immutability itself is doctrine + T3's shape checks,
  not runtime-verified — a deep freeze per tick would be pure overhead).
- The tuple unpack is deliberately organic: a step that returns a non-2-tuple
  raises `TypeError`/`ValueError` inside the driver frame and is wrapped as
  `StepError` like any step failure (§10.2). No pre-unpack shape check (D14).
- Determinism: `state_N` is a pure fold of `initial` over rows 0..N-1 —
  exactly the property T10 checkpointing and the inspector lean on.

### 5.3 Runtime `None` from a `skips=False` step — T3 §15 Q4, settled (D3)

**Decision: ERROR** — `[step-none-no-skip]`, raised by the driver at the
offending tick, all step kinds including async (where it *propagates*, unlike
user exceptions — §6.4).

Rationale:

- T3 made the step signature the module's whole contract and spent its budget
  making violations loud at import. A `-> Out` step returning `None` at
  runtime is the same lie surfacing later; the runtime driver is the last
  place it is cheaply catchable. Skip-and-count would convert a type-level
  falsehood into a silent cadence change — precisely the bug class the
  contract machinery exists to kill (house rule: constraints are invariants;
  don't absorb violations, trace them).
- The check is free: the driver already branches on `out is None` for
  `skips=True` modules; `skips=False` turns the same branch into a raise.
- Contrast with T1 D11 (ts overwrite, never error) is principled, not
  inconsistent: a user-set ts has a correct resolution (the engine owns the
  value; overwrite loses nothing), while annotation-vs-behavior contradiction
  has none — someone is wrong, and only the author can say who.
- Observability: the failure IS an exception (maximally observable), message
  names the module, the tick ts, and the fix (`-> Out | None`); §9's
  `errors` counter increments before the raise so post-mortem counter
  snapshots show it.

## 6. The async driver

The hardest piece; specified as a state machine. Two layers: an internal
async generator `_drive_async_rows(...)` (the machine) and the public sync
façade `drive_async(...)` that owns a private event loop (§6.2).

### 6.1 `max_inflight` — sourcing (D6)

A **module config field**, by convention: an async module MAY declare
`max_inflight: int = N` (the sketch's Captioner declares `= 4`; that
declaration is the pattern, not engine magic). `run_over` reads
`getattr(module, "max_inflight", DEFAULT_MAX_INFLIGHT)` with
`DEFAULT_MAX_INFLIGHT = 1`, then validates `isinstance(int, not bool)` and
`>= 1` → `[run-bad-max-inflight]`.

- Default **1** (serial): the engine never invents concurrency the author
  didn't declare. An undeclared async module still works — it just overlaps
  nothing — and widening is one config field away, sweepable like any config
  (`Captioner(max_inflight=8)`).
- An `over()` kwarg was rejected: T4's landed signature is
  `over(**streams: Streamable)` — every kwarg is a stream name, and a
  `max_inflight=` kwarg would collide with a legitimate In field of that
  name. Config-field sourcing also keeps run identity = class + config.
- `drive_async` itself takes `max_inflight` as an explicit parameter (D2) and
  re-validates at generator start (direct callers bypass `run_over`).

### 6.2 Event-loop policy (D7)

`over()` is a sync iterator façade even for async modules. Policy:

- **One private loop per run**, created by the façade generator at first
  `next()` via `asyncio.new_event_loop()`. NEVER `get_event_loop()`, never
  `set_event_loop()`, no policy mutation — the loop is reachable only through
  the run's locals. (`asyncio.Runner` is 3.11+; floor is 3.10 — manual loop.)
- **Stepped, not threaded**: the façade drives the async generator one row at
  a time — `loop.run_until_complete(agen.__anext__())` — yielding each row
  to the sync consumer. No background thread, no queue, no cross-thread
  marshaling: the entire fold-era bug family (leaked threads, teardown
  races) is structurally absent. In-flight step tasks make progress only
  while the loop runs, i.e. during those calls.
  - Priced consequence: I/O overlaps I/O (the point of `max_inflight`), but
    consumer-body time does not overlap I/O — while the `for` body runs, the
    loop is parked. Acceptable for offline `over()`; T8's live rim runs a
    persistent loop and does not inherit this property.
  - Input pulls (`next(rows)`) happen on the loop thread between awaits and
    briefly block task progress; correct, and cheap in practice (T5 pulls
    from buffers/stores, no waits by design).
- **Nesting guard**: called where an event loop is already running, sync
  `over()` cannot work (`run_until_complete` would throw a bare
  `RuntimeError` mid-iteration). `run_over` detects eagerly
  (`asyncio.get_running_loop()` probe) and raises `[run-inside-loop]` at the
  call site; the façade re-probes at first `next()` (defense for direct
  driver callers and iterators smuggled across contexts).
- **Shutdown order** (façade `finally`, every exit path):
  1. `loop.run_until_complete(agen.aclose())` — triggers the machine's
     CLOSING state (§6.3): cancel window, reap, `await hooks.ateardown()`.
  2. `loop.run_until_complete(loop.shutdown_asyncgens())` — reaps any
     stray async generators user step code created.
  3. `loop.close()`.
  All inside the loop's lifetime; nothing async outlives it.

### 6.3 The machine

State variables (all generator-local):

| var | type | invariant |
| --- | --- | --- |
| `window` | `deque[(tick_ts: float, task: asyncio.Task)]` | `len(window) <= max_inflight`; tick-ordered (admission order) |
| `exhausted` | `bool` | set exactly once, by the input pull |
| `rows` | `Iterator[InRow]` | pulled only in FILL |

**The window deque IS the reorder buffer** — the index's "bounded reorder
buffer (size = max_inflight)" is satisfied without a second structure:
completed-but-unemitted results live inside their `Task` objects, held in
tick order, popped head-first. Nothing is ever emitted out of order because
nothing is ever *awaited* out of order.

States and transitions:

- **FILL** — while `not exhausted and len(window) < max_inflight`: pull
  `next(rows)`; on `StopIteration` → `exhausted = True`; else
  `hooks.ticks += 1`, `task = asyncio.ensure_future(module.step(row))`
  (`ensure_future`, not `create_task`: the protocol promises `Awaitable`,
  not `Coroutine`), append `(row.ts, task)`. Admission order = tick order.
  - `window` empty and `exhausted` → **DONE** (StopAsyncIteration).
  - else → **AWAIT_HEAD**.
- **AWAIT_HEAD** — pop `(ts, task)` from the left; `await task`
  (head-of-line; younger tasks progress concurrently under the same loop):
  - result is a row → `hooks.emits += 1`;
    `yield dataclasses.replace(out, ts=ts)` (D11 stamp) → **FILL**.
  - result is `None`, `skips` → `hooks.skips += 1` → **FILL**.
  - result is `None`, not `skips` → raise `[step-none-no-skip]` →
    **CLOSING** (via the exception unwinding through `finally`).
  - task raised `Exception` → **dropped tick** (§6.4): `hooks.drops += 1`,
    `hooks.last_error = exc`, one `logging` warning naming module + tick ts
    → **FILL**.
  - task raised `BaseException` (`KeyboardInterrupt`, `SystemExit`;
    `CancelledError` is `BaseException` since 3.8) → propagate raw →
    **CLOSING**.
- **DRAIN** — *not a state*: when `exhausted` is set, FILL admits nothing and
  the FILL⇄AWAIT_HEAD loop naturally empties the window in tick order,
  emitting every completed row. Drain is emergent from the main loop — no
  second code path to get wrong.
- **CLOSING** — the machine's `finally` (reached on GeneratorExit from
  `aclose()`, on any raise, and as a no-op after DONE):
  1. `task.cancel()` for every task still in `window`.
  2. `await asyncio.gather(*tasks, return_exceptions=True)` — bounded reap;
     every task reaches a terminal state, none leaked. (No `yield` in this
     block — an async gen must not yield during close.)
  3. `await hooks.ateardown()` — the T7 async-disposal seam (§8.4), on the
     run loop, after quiescence, before the loop dies.

**"Cancellation must not drop already-completed rows", stated precisely**:
on every path where emission is still possible (the main loop, including its
emergent drain), a completed task's row is emitted at its tick position —
completed tasks wait in the window and are never discarded, never skipped
past. Discard happens only in CLOSING, where emission is impossible by
definition (the consumer abandoned the run or an error ended it). There is
no path that discards a completed row and then continues emitting later
ones.

Failed-step window semantics: a dropped tick frees its window slot at the
head like any other completion — one failure never stalls or shrinks the
pipeline.

### 6.4 Async error policy (D5 — reaffirms the index decision)

`Exception` from a step task = **counted dropped tick**; the run continues.
This is the index's settled text ("failed step = counted dropped tick"), kept
deliberately: async is the I/O spelling — transient environmental failure is
an expected event class, and with a window of in-flight ticks, propagation
would additionally have to discard their real completed work. Loudness is
preserved by accounting, not by dying:

- `hooks.drops += 1`; `hooks.last_error = exc` (raw, uncloaked);
- one `logging.getLogger("dimos.pure.drivers")` warning with module, tick
  ts, `exc_info` — on by default, no wall-clock in the data path (log
  timestamps are the logging subsystem's, outside the data plane);
- T9 later surfaces `drops`/`last_error` in health rows.

Boundary of the policy, explicit and consistent:

| origin | policy |
| --- | --- |
| user `Exception` from a step task | drop + count + log, continue |
| user `BaseException` (KeyboardInterrupt, SystemExit) | propagate raw, abort |
| engine-detected violation (`[step-none-no-skip]`) | propagate, abort — engine checks are never "dropped" |
| `CancelledError` | cancellation machinery, never counted |

A module that wants per-call robustness *with a value-level fallback* does it
the doctrinal way: catch inside `step`, return `None`, annotate
`-> Out | None`.

Sync kinds, for contrast, **propagate** user exceptions (wrapped, §10.2):
there is no in-flight work to protect, the consumer is a stack frame away,
and offline determinism makes the failure exactly reproducible.

### 6.5 Determinism (D15)

- **Guaranteed**: emission order = tick order, always; window admission
  order = tick order; for a given per-tick outcome sequence
  (row/None/exception), the emitted sequence, all counters, and all error
  raises are identical across runs — completion order is irrelevant by
  construction (head-of-line await).
- **Not guaranteed**: per-tick outcomes themselves when the step does real
  I/O (the sketch's nondeterministic-resource caveat — captions vary), and
  any wall-clock interleaving. Async determinism is *order determinism*, not
  value determinism; sync kinds with pure steps get both.

## 7. The fold driver

### 7.1 Contract

`gen = module.fold(proxy)` where `proxy` is the driver's counting wrapper
over `rows` — a **lazy, single-pass** iterator (`hooks.ticks += 1` per pull;
running-max causality cursor per §7.3). The generator owns the loop; the
driver owns validation and lifecycle:

```python
gen = module.fold(_CountingRows(rows, hooks, cursor))
try:
    for out in gen:            # user exceptions here → StepError (§10.2)
        _validate_fold_row(out, cursor, last_emitted)   # §7.3
        hooks.emits += 1
        yield out              # self-stamped: NEVER replaced (T1 D11 split)
finally:
    gen.close()                # exactly once; GeneratorExit into fold's frame
```

- The driver prefetches nothing: fold's own pulls are the only pulls.
- `for out in gen` naturally handles a fold that manually `next()`s the proxy
  and lets `StopIteration` escape — PEP 479 turns that into `RuntimeError`
  inside the generator, which propagates wrapped (§10.2).
- A fold may keep yielding **after input exhaustion** (tail flush — e.g. a
  ScanBatcher variant emitting its partial batch): legal, under the same ts
  rules — the causality cap (§7.3) still binds it to the newest consumed
  input ts, and strict monotonicity still orders it after everything emitted.
- A fold that yields while `close()` is being delivered raises the standard
  `RuntimeError("generator ignored GeneratorExit")` — propagated, not
  absorbed.
- `skips` is structurally 0 for fold (not skipping is not a skip — a fold
  controls emission; T3 fixes `spec.skips = False`); `drops` is 0 (no async).

### 7.2 Why validate at all

fold is the one shape where the module holds the stamp authority, so it is
the one shape where a wrong ts can silently poison downstream total order —
replay identity, T5 ingestion of a recorded fold edge, and T10 cursors all
assume strictly-increasing edge ts. The driver is the only chokepoint.

### 7.3 The ts rules (D10)

Two cursors, driver-local: `last_emitted` (init `-inf`),
`newest_in` = running **max** of pulled input `row.ts` (init `-inf`;
max, not last, so a T5 equal-ts tie order can never trip it — A6).

Checks per yielded `out`, in order:

1. `out is None` → `[fold-yielded-none]`. Explicit check purely for message
   quality — the alternative is an opaque `AttributeError: 'NoneType'` at
   rule 2.
2. `out.ts == UNSTAMPED` → `[fold-unstamped]`. Dedicated message ("fold
   stamps its own rows — construct Out(ts=...)"); note rule 4 would catch it
   for free (T1's design: `-inf` never beats a `-inf` cursor under strict
   `>`) — the dedicated check exists for the error text, the monotonic
   backstop remains load-bearing if this check is ever reordered.
3. `out.ts > newest_in` → `[fold-future-ts]` (causality cap): a fold cannot
   stamp ahead of the newest input it has consumed — it has no time
   authority beyond its input. Subsumes the yields-before-any-pull case
   (`newest_in == -inf`): a fold emitting before consuming any input has no
   basis for any ts, and the message calls that variant out.
4. `out.ts <= last_emitted` → `[fold-nonmonotonic]` (strict: equal ts is a
   violation — two rows at one ts on one edge breaks the replay total
   order). Note stamping at batch *start* ts is legal (monotone across
   batches); only regression/duplication is not.

Post-check: `last_emitted = out.ts`. All four checks are two float compares
and two identity checks per *emitted* row — emission is the rare event;
the hot per-input path carries only the counter and the max.

## 8. Teardown

### 8.1 The acquisition invariant

Nothing is acquired before the first `next()` of the returned iterator:
`run_over`'s eager phase is pure validation + `State()` + building lazy
objects (A2). Therefore: an `over()` iterator that is never started owes —
and runs — no teardown. (If the consumer abandons a *started* iterator
without `close()`, CPython's generator finalizer delivers GeneratorExit at
GC and the same teardown runs, at nondeterministic time — prefer explicit
consumption or `contextlib.closing`; documented, not fought.)

### 8.2 The `_finalized` composition (D8)

```python
def _finalized(inner, rows, hooks):
    try:
        yield from inner
    finally:
        try:
            close = getattr(rows, "close", None)
            if close is not None:
                close()                       # T5 aligner cleanup (A2)
        finally:
            hooks.teardown()                  # T7 seam — LAST, always
```

Applied **inside each public `drive_*`** (a plain function returning
`_finalized(_inner_kind(...), rows, hooks)`), not by `run_over` — so direct
driver callers (tests, T8) get the full teardown contract with no extra
wrapper to remember, and there is exactly one seam call site per run by
construction.

`yield from` gives the ordering for free: closing/aborting the outer
generator first closes `inner` (the kind driver's own `finally` —
cancel+reap+`ateardown`+loop shutdown for async, `gen.close()` for fold),
then `rows` closes, then `hooks.teardown()` — machinery quiesces
outer-to-inner before any disposal, disposal runs innermost-cleanup-first,
seam last. Exactly-once is Python's generator guarantee: one `finally`
execution per generator lifetime, whether by exhaustion, `close()`,
`throw`, or GC.

### 8.3 Teardown ordering — every exit path

"driver-internal" = per kind: **async**: cancel window → gather-reap →
`await hooks.ateardown()` → (façade) `shutdown_asyncgens` → `loop.close()`;
**fold**: `gen.close()`; **stateless/mealy**: none.

| # | exit path | trigger seen by driver | order | consumer sees |
| --- | --- | --- | --- | --- |
| 1 | exhaustion | `rows` raises StopIteration (async: emergent drain first) | driver-internal → `rows.close()` → `hooks.teardown()` | clean StopIteration |
| 2 | consumer breaks / `close()` | GeneratorExit at the current `yield` | same as 1 (async CLOSING discards in-flight; §6.3) | nothing (loop exit) |
| 3 | step error, sync kinds | user exception unwinding | same as 1 | `StepError` (cause = original) |
| 4 | engine violation (`none-no-skip`, fold ts rules) | driver raise | same as 1 | `PureModuleRunError` |
| 5 | async dropped tick | task `Exception` | **no teardown — not an exit**; count + continue | (nothing; run continues) |
| 6 | BaseException (Ctrl-C, SystemExit) | propagating raw | same as 1 (finallys run on BaseException) | the BaseException |
| 7 | teardown itself raises | exception inside a `finally` | remaining finallys still run (nested try/finally); original exception, if any, chains via `__context__` | teardown's exception (chained) |
| 8 | never started | — | nothing acquired, nothing run (§8.1) | — |
| 9 | started, abandoned to GC | GeneratorExit from finalizer | same as 2, at GC time | — |

`hooks.teardown()` runs exactly once on every path 1–4, 6, 7, 9 — and zero
times on path 8, by the acquisition invariant, which is what makes zero
correct.

### 8.4 The T7 seam (D9)

Two named fields on `RunHooks`, both defaulting to no-ops, both reserved for
T7's per-run resource disposal — **spec'd here, attached there**:

- `teardown: Callable[[], None]` — sync disposal; called once in
  `_finalized`'s `finally` (§8.2), last, for every kind.
- `ateardown: Callable[[], Awaitable[None]]` — async disposal; awaited once
  in the async machine's CLOSING (§6.3), on the run's loop, after task
  quiescence, before the loop closes. Exists because async resources (the
  sketch's `VLMClient.aclose()`, "engine awaits aclose at stop") are bound
  to the loop they were created on — disposal after `loop.close()` is a
  correctness bug, not a style choice. This is the brief's
  "`aclose` after drain", placed.
- Seam contract for T7: `ateardown` is awaited only for async-kind runs (the
  only runs with a loop); sync-kind runs invoke `teardown` only — T7 routes
  async-disposable resources accordingly (its problem, its spec).
  Exceptions from either propagate per row 7 of §8.3.

Nothing else about resources is built here; the seam is two fields and two
call sites.

## 9. `RunHooks` — tick accounting (D13)

```python
@dataclasses.dataclass
class RunHooks:
    ticks: int = 0                          # In rows entering the driver (fold: rows the fold pulled)
    emits: int = 0                          # Out rows yielded to the consumer
    skips: int = 0                          # None from a skips=True step
    drops: int = 0                          # async: failed step tasks (counted dropped ticks)
    errors: int = 0                         # propagating driver/step errors (incremented before raise)
    last_error: BaseException | None = None # most recent dropped/propagated user exception
    teardown: Callable[[], None] = _noop            # T7 seam (§8.4)
    ateardown: Callable[[], Awaitable[None]] = _anoop
```

Increment points (normative):

| counter | stateless | mealy | async | fold |
| --- | --- | --- | --- | --- |
| `ticks` | per row pulled | per row pulled | per row admitted to window | per row the fold pulls via proxy |
| `emits` | per yield | per yield | per yield (post-stamp) | per yield (post-validate) |
| `skips` | per None (skips) | per None (skips) | per None (skips) | never (structurally 0) |
| `drops` | never | never | per task `Exception` | never |
| `errors` | per raise §10 | per raise §10 | per *propagating* raise | per raise §10 |

Semantics: plain ints, mutated only by the driver's own thread, no locks —
the T9 pacer reads monotonic, possibly momentarily-stale values from another
thread (GIL-atomic int rebinding; exact totals are guaranteed only after the
run ends). `last_error` holds the raw user exception (never the `StepError`
wrapper — the wrapper adds coordinates for the *raise* path; the hook keeps
the forensic original). Invariant at clean exhaustion:
`ticks == emits + skips + drops` (step kinds; fold has no such identity —
its emission count is decoupled by design).

Fresh per run (created by `run_over` when not injected) — a reused injected
`RunHooks` accumulates across runs by caller choice; the driver only ever
increments.

## 10. Error catalog

### 10.1 Types

```python
class PureModuleRunError(RuntimeError):
    """A pure-module run violated a runtime contract."""
    rule: RunRule | None      # same machine-readable pattern as T3's Rule

class StepError(PureModuleRunError):
    """User step/fold code raised; carries module + tick coordinates.
    Always `raise ... from exc` — the original is __cause__, uncloaked."""
```

`RuntimeError` base (vs T3's `TypeError`): definition-time shape errors are
type errors; run-time contract violations are runtime errors. Both carry a
slugged message + `rule` attr so the two catalogs read as one system.

`RunRule` slugs: `run-unknown-stream`, `run-inside-loop`,
`run-bad-max-inflight`, `step-error`, `step-none-no-skip`,
`fold-yielded-none`, `fold-unstamped`, `fold-future-ts`,
`fold-nonmonotonic`.

### 10.2 Wrapping policy (D4)

Every **propagating user-code exception** (sync step raise, fold generator
raise, mealy unpack failure) is wrapped: `raise StepError(msg) from exc`.
The traceback keeps the user frames; `__cause__` keeps the type; the message
adds what the traceback cannot — the tick coordinates (`ts`, tick ordinal)
that are exactly the flight-recorder repro key ("(State, In row) at the
failing tick"). Priced: a consumer's `except ValueError` across the driver
boundary stops matching — deliberate; expected failures are handled *inside*
`step` (return `None`), and exceptions crossing the driver are bugs wearing
coordinates. Never wrapped: BaseExceptions, engine violations (they *are*
the error), T5 alignment errors (A5), teardown-path errors (§8.3 row 7).

### 10.3 Message templates (release copy; `{cls}` = `type(module).__module__ + '.' + __qualname__`)

| rule | template |
| --- | --- |
| `run-unknown-stream` | `{cls}.over() got unknown stream(s) {names}; In fields are: {fields}. Stream kwargs must match {cls}.In field names. [run-unknown-stream]` |
| `run-inside-loop` | `{cls}.over() cannot run inside a running event loop — over() blocks, driving a private loop for async steps. Iterate it from sync code, or move the call off the loop (e.g. a worker thread). [run-inside-loop]` |
| `run-bad-max-inflight` | `{cls}.max_inflight must be an int >= 1, got {value!r} — it bounds the async in-flight window (1 = serial). [run-bad-max-inflight]` |
| `step-error` | `{cls}.{impl} raised at tick ts={ts} (tick #{n}): {exc!r} [step-error]` (fold variant: `after consuming {n} rows, newest input ts={ts}`) |
| `step-none-no-skip` | `{cls}.step returned None at tick ts={ts} but is annotated '-> {out}' — a never-skip step must emit every tick. Annotate '-> {out} \| None' if some ticks legitimately skip. [step-none-no-skip]` |
| `fold-yielded-none` | `{cls}.fold yielded None — a fold emits rows only; to skip a tick, don't yield. [fold-yielded-none]` |
| `fold-unstamped` | `{cls}.fold yielded an unstamped row (ts is UNSTAMPED) — fold stamps its own rows: construct {out}(ts=..., ...) from an input row's ts (newest consumed: ts={in_ts}). [fold-unstamped]` |
| `fold-future-ts` | `{cls}.fold yielded ts={ts} but the newest input it has consumed is {in_ts} — a fold cannot stamp ahead of its input. [fold-future-ts]` (no-input variant: `...but it has not consumed any input yet — a fold's time authority is its input rows.`) |
| `fold-nonmonotonic` | `{cls}.fold yielded ts={ts} after already emitting ts={prev} — fold output ts must be strictly increasing (replay identity requires a total order per edge). [fold-nonmonotonic]` |

## 11. `over()` and the `typing.py` delta (T6-impl edits)

Three edits to `typing.py`, performed by the T6 implementer (sanctioned by
T4 §4.4 + §13.1; every T4 harness case must stay green — verified feasible:
all fixture `over()` calls pass zero stream kwargs, so the alias tightening
is invisible to the harness):

1. **`over()` body** — exactly T4 §4.4's contract:

   ```python
   def over(self: Any, **streams: Streamable) -> Iterator[Any]:
       """Align streams, drive the step, yield typed Out rows (T5 + T6)."""
       from dimos.pure.stepspec import step_spec   # lazy: sanctioned edge #1
       from dimos.pure.drivers import run_over     # lazy: sanctioned edge #2
       return run_over(self, step_spec(type(self)), streams)
   ```

2. **`Stamped` protocol** (new, in the data layer — `typing.py` is its home;
   `rows.py` owns row-ts doctrine, `typing.py` owns stream-facing protocols):

   ```python
   @runtime_checkable
   class Stamped(Protocol):
       """One element of a timestamped stream: anything carrying a float ts."""
       @property
       def ts(self) -> float: ...
   ```

   Read-only property spelling per T4 §5.3 doctrine — satisfied by mutable
   attributes (memory2 `Observation.ts`, dimos msgs' `ts: float`) AND by
   frozen-dataclass row fields; a plain `ts: float` member would demand
   settability and reject rows. Add `"Stamped"` to `__all__` (drivers and
   T5 import it; users writing custom sources will too).

3. **The alias swap** (T4 §13.1 resolved — D12):

   ```python
   Streamable: TypeAlias = Iterable[Stamped]
   ```

   `Iterable` because a stream argument is anything the aligner can
   `iter()` per run: memory2 `Stream` (yields `Observation` — satisfies),
   plain lists of rows/msgs (tests), generators (one run). The memory2
   `StreamAccessor`-protocol alternative was rejected: it is a store-access
   surface, not a data contract — binding to it would reject plain
   iterables and force the zero-dep data layer to describe a memory2 type.
   Overload structure untouched, exactly as T4 §4.3 planned.

The skeleton meanwhile defines `Stamped` locally in `drivers.py` with a
relocation note — the implementer moves it here and imports it back
(`drivers.py` may import `typing.py`; never the reverse).

## 12. Edge cases — index watch-outs → spec mapping

| index watch-out | resolved by |
| --- | --- |
| engine owns the task set; no leaked tasks on error | §6.3 CLOSING: cancel + `gather(return_exceptions=True)` reaps every task on every exit; test `test_async_break_cancels_inflight` |
| bounded reorder buffer (size = max_inflight) | §6.3: the window deque is the buffer; results wait inside their Tasks, popped head-first |
| cancellation must not drop already-completed rows | §6.3 precise statement: completed rows are discarded only where emission is impossible; drain is emergent, discards nothing; test `test_async_drain_emits_completed` |
| Mealy State immutable — never alias, never mutate | §5.2: single rebound reference, fresh `State()` per run, dropped at end |
| `over()` re-invokable; module carries zero run state | §4: all state in generator locals + per-run hooks; module frozen (T2) |
| teardown on early exit, exactly once | §8.2 `_finalized` + §8.3 table; tests `test_teardown_on_break*` |
| fold self-stamp validation catches forgotten stamp | §7.3 rules 2+4 (dedicated message + monotonic backstop) |
| no wall clock in the data path | nowhere in any driver: no `time.*`, no sleeps, no timeouts — drain waits are unbounded by design (timeout policy is a rim concern, T8); the only clock adjacency is log-record timestamps, outside the data plane |
| async loop ownership without leaks | §6.2 private loop, §6.2 shutdown order, `[run-inside-loop]` guard |

## 13. What T6 does NOT do (division of labor)

- No stream-value validation, no required/missing policy, no buffering, no
  interpolation — T5 (A3).
- No resource creation/disposal beyond calling the two seam callables — T7.
- No transports, no threads, no live loop, no stop() — T8 (which reuses
  these drivers with its own loop and its own hooks).
- No health rows, no pacing — T9 (reads `RunHooks`).
- No per-row `isinstance` of step outputs (D14): mypy + T3 own the row-type
  contract; a wrong object fails organically at stamping/validation with the
  step frame in the traceback. The only per-row runtime checks are the None
  policy and fold's ts rules — each justified by a silent-failure class,
  not by distrust of types.

## 14. Test plan (`dimos/pure/test_drivers.py`)

Module-level `pytestmark = pytest.mark.skip(...)` until implementation;
everything below is written as real bodies now. Fixtures are plain classes
(+ real `pm.In`/`pm.Out` bundles) — no `PureModule`, proving §1's layering.
Async tests drive through the public sync façade with **event-choreographed
steps** (steps await/set `asyncio.Event`s among themselves — synthetic
completion order, zero sleeps, zero wall-clock). `run_over` tests stub
`dimos.pure.align` in `sys.modules` (pins A1 executably).

| test | pins |
| --- | --- |
| `test_stateless_emits_and_stamps` | §5.1 basic drive; Out ts == tick ts |
| `test_stateless_skip_counting` | None → skipped tick; ticks/emits/skips |
| `test_stateless_ts_overwrite` | D11: explicit user ts overwritten, UNSTAMPED never escapes |
| `test_stateless_none_no_skip_raises` | D3/Q4: `[step-none-no-skip]`, errors counter |
| `test_stateless_lazy_pull` | pull-per-tick, no prefetch |
| `test_stateless_step_error_wrapped` | §10.2: StepError, `__cause__`, coordinates, raw `last_error` |
| `test_mealy_threads_state_from_default` | §5.2: starts at `State()`, exact thread sequence |
| `test_mealy_skip_and_emit` | Mealy None-in-slot skip + counters |
| `test_mealy_fresh_state_per_run` | two runs, identical outputs, no bleed |
| `test_fold_self_stamp_valid` | §7 happy path; ts untouched by driver; ticks = fold's pulls |
| `test_fold_unstamped_caught` | rule 2 message + rule-4 backstop reasoning |
| `test_fold_nonmonotonic_equal_ts` | strict: equal ts rejected |
| `test_fold_nonmonotonic_regression` | decreasing ts rejected |
| `test_fold_future_ts` | causality cap vs newest consumed input |
| `test_fold_yield_before_any_input` | causality cap's no-input variant |
| `test_fold_yield_none` | `[fold-yielded-none]` |
| `test_fold_lazy_single_pass` | proxy pull count == fold's own pulls |
| `test_fold_teardown_on_break` | fold's `finally` runs via `gen.close()`; seam once |
| `test_fold_tail_flush_after_exhaustion` | post-exhaustion yields legal under ts rules |
| `test_fold_step_error_wrapped` | §10.2 applies to fold-generator raises |
| `test_async_tick_order_emission` | §6.3: completion order ≠ emission order |
| `test_async_inflight_bound` | peak concurrency == max_inflight, never above |
| `test_async_drop_on_exception` | D5: counted drop, raw `last_error`, run continues, not an error |
| `test_async_none_skip` | skips=True None in async |
| `test_async_none_no_skip_propagates` | D3 in async: violation aborts (≠ drop) |
| `test_async_drain_emits_completed` | exhaustion drains window fully, in order |
| `test_async_break_cancels_inflight` | §6.3 CLOSING: started tasks see CancelledError; none leaked; seam once |
| `test_async_ateardown_runs_on_run_loop` | §8.4: `ateardown` awaited before the loop dies |
| `test_teardown_on_break_stateless` | §8.2/§8.3 row 2; exactly once |
| `test_teardown_exactly_once_on_error` | §8.3 rows 3/4; close-after-finish is a no-op |
| `test_teardown_closes_rows` | §8.2: drivers own and close `rows` |
| `test_run_over_unknown_stream_eager` | A3/D1: raises at call, before any pull (runs pre-T5) |
| `test_run_over_bad_max_inflight_eager` | §6.1 validation at call |
| `test_run_over_composes_align` | A1 pin: stubbed align receives `(in_type, streams)`; rows drive; hooks flow |
| `test_run_over_mealy_starts_from_default_state` | D16: `spec.state_type()` is the run's initial |
| `test_run_over_async_serial_default` | D6: no field → `DEFAULT_MAX_INFLIGHT` (serial, peak 1) |
| `test_run_over_async_max_inflight_from_module_field` | D6: the Captioner config-field pattern honored |
| `test_hooks_invariant_step_kinds` | `ticks == emits + skips + drops` at exhaustion |

Deferred to T5-integration/T12 (listed so they aren't lost): real
`align`×driver composition over a store, `over()` end-to-end on the sketch
modules, alignment-determinism property test (index T12), re-invoked
`over()` on a real re-iterable stream.

## 15. Acceptance checklist

- [ ] `uv run mypy dimos/pure/drivers.py` clean; `uv run mypy dimos/pure`
      stays clean (strict).
- [ ] `uv run pytest dimos/pure -q`: existing 167 still pass; T6 tests
      collect and skip (spec phase) / pass (implementation phase).
- [ ] Implementation replaces `over()`'s body per §11.1; ALL T4 harness
      cases stay green (`test_typing_static.py`), including
      `case_config_kwargs`/`case_tagger_floor` revealed types.
- [ ] `Stamped` + alias land per §11.2–3; `uv run mypy dimos/pure/typing.py`
      standalone clean.
- [ ] Every §10.3 message template appears verbatim (modulo interpolation)
      in the implementation, each with a test asserting slug + key phrases.
- [ ] §8.3 table holds path-by-path under tests (rows 1–4, 6 at minimum).
- [ ] Async machine invariants hold under the choreography tests: window
      bound, tick-order emission, no leaked tasks (§6.3).
- [ ] No wall-clock/timeout call in any driver code path (§12); no import
      of `module.py`/`config.py` from `drivers.py` (§1).
- [ ] fold hot path adds nothing beyond §7.3's counters to per-input cost.
- [ ] T4 §13.1 marked resolved (pointer to §11.3) when landing.

## 16. Decisions within mandate (numbered for review)

- **D1** `run_over` eager/lazy split: eager = unknown-stream names,
  `max_inflight` sourcing+validation, running-loop guard, `State()`
  construction; lazy = all acquisition (§4). `align` imported lazily inside
  `run_over`, permanently.
- **D2** Drivers take scalars + protocol-typed module params; `StepSpec`
  stops at `run_over`; `module: Any` only there (§1, §2).
- **D3** T3 §15 Q4: runtime None from a `skips=False` step is an ERROR
  (`[step-none-no-skip]`), every kind, propagating even in async (§5.3).
- **D4** Propagating user exceptions wrap as `StepError` with tick
  coordinates, `from` the original (§10.2).
- **D5** Async user `Exception` = counted dropped tick + `last_error` + log
  warning; BaseException propagates; engine violations propagate (§6.4 —
  reaffirms index text, so no relitigation entry).
- **D6** `max_inflight` is a module config field read by convention
  (`getattr`, default `DEFAULT_MAX_INFLIGHT = 1`); no `over()` kwarg (§6.1).
- **D7** Async runs on a private, stepped event loop owned by the run; no
  global loop/policy touch; `[run-inside-loop]` guard; shutdown order fixed
  (§6.2). Head-of-line await; the window deque is the reorder buffer (§6.3).
- **D8** Teardown composed via `_finalized` (`yield from` + `finally`),
  applied inside each public `drive_*` so every caller gets it; inner-first
  ordering; exactly-once by generator semantics; §8.3 table normative.
- **D9** T7 seam = `RunHooks.teardown` + `RunHooks.ateardown` (async,
  awaited on the run loop before it closes) — two named fields, no-op
  defaults (§8.4).
- **D10** fold ts rules: None check, UNSTAMPED (dedicated message; strict
  monotonic backstop), causality cap vs running-max input cursor, strict
  monotonicity; tail flush legal; two-compares hot cost (§7.3).
- **D11** T1 D11 adopted: unconditional `dataclasses.replace(out,
  ts=row.ts)` for step kinds; fold never restamped (§5.1, §7.1).
- **D12** `Streamable = Iterable[Stamped]`; `Stamped` = runtime-checkable
  protocol with read-only `ts: float` property, home `typing.py`;
  StreamAccessor bound rejected (§11.2–3).
- **D13** `RunHooks`: flat mutable dataclass, lock-free monotonic ints,
  fresh per run, caller-injectable via `run_over` only; increment table
  normative (§9).
- **D14** No per-row type validation of step outputs (§13).
- **D15** Determinism contract: order always; values modulo module purity;
  async completion order irrelevant (§6.5).
- **D16** Mealy `initial` constructed eagerly in `run_over` via
  `spec.state_type()`; `drive_mealy(initial=...)` is the T10 restore seam
  (§4, §5.2).

## 17. Open questions (non-blocking)

1. **A1–A6 reconciliation with T5** — the align signature, laziness, and
   validation split need a yes from the T5 spec when it lands;
   `test_run_over_composes_align` converts drift into a loud test failure.
2. **Hooks naming unification** (A4) — if T5's aligner counters land under a
   different container shape, unify names before T9 consumes both.
3. **Counter surface under plain `over()`** — `run_over(hooks=...)` is the
   only access today; whether a friendlier `pm`-level spelling is wanted is
   a T9/T12 call.
4. **`Stamped` in `pm.*` exports** — added to `typing.__all__` here; whether
   the `pm` surface re-exports it is T12's docs call.

No `## Relitigation` section: no settled decision is deviated from — D5
reaffirms the index's async-drop text, D11 adopts T1's recommendation
verbatim, D3 and D12 execute mandates explicitly delegated to T6.
