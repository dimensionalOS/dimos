# T5 — Alignment engine: implementation spec

Target: `dimos/pure/align.py` (+ `dimos/pure/test_align.py`). Source of truth:
`dimos/memory2/puremodule_api_sketch3.py` and `tasks/index.md` §T5, built on
the landed T1–T4 reality (`rows.py` `fields()`, `stepspec.StepSpec`,
`typing.Streamable`). Python floor 3.10. The static claims below (the
`Stamped` protocol spelling) were verified against mypy 1.19.0 under the repo
strict config on this branch.

**Judgment calls are numbered D1–D15 in §14.** Two index-level phrasings are
refined rather than followed literally; both are argued in §15 Relitigation.

---

## 1. Scope and layer boundary

`align.py` is the tick resolver and nothing else:

- `align(in_type, streams)` → `Aligner[InT]`: stamped-item iterables in,
  `In`-row iterator out. Pure, pull-based, thread-free, wall-clock-free.
- The `Stamped` input protocol (the currency T6 tightens `typing.Streamable`
  to).
- The `Interpolatable` protocol + interpolator registry.
- `AlignStats`/`PortStats` counters (the T9 seam) and `AlignmentError`.

Hard boundary: **no drivers, no module, no config, no transports, no
wall clock, no threads**. Imports: stdlib + `dimos.pure.rows` only
(`stepspec` is permitted by the task boundary but not needed — `align()`
takes the `In` class directly; T6 passes `spec.in_type`). `align.py` never
imports the `dimos.pure` package surface (cycle), never `dimos.memory2`
(engine-free data layer; stream-object coercion is T6's, §11.1).

Consumption map:

| Consumer | Uses from T5 |
| --- | --- |
| T6 drivers | `align(spec.in_type, streams)` behind `over()`; `Stamped` as the `Streamable` bound; the coercion obligation (§11.1) |
| T8 rim | the resolution core semantics (§5) driven by queue-backed iterators; same `Stamped` currency (§11.3) |
| T9 health | `Aligner.stats` counters + `held_tick_ts` gauge (§8) |
| T10 checkpoint | the cursor/resume property (§11.4) |
| T11 tf | the sampler-kind extension seam + the interpolator registry (§11.5) |

## 2. Public API

Everything below ships in `dimos/pure/align.py` exactly as spelled (bodies
elided). `__all__ = ["AlignRule", "AlignStats", "Aligner", "AlignmentError",
"Interpolatable", "PortStats", "Stamped", "align", "interpolator_for",
"register_interpolator"]`.

```python
from __future__ import annotations

import dataclasses
import enum
from typing import Any, Callable, Generic, Iterable, Iterator, Mapping, Protocol, TypeVar

from typing_extensions import Self, TypeAlias

from dimos.pure.rows import FieldSpec, In, InterpolateSpec, LatestSpec, TickSpec


class Stamped(Protocol):
    """One payload in a stream: anything carrying a readable float ts."""

    @property
    def ts(self) -> float: ...


InterpolatorFn: TypeAlias = Callable[[Any, Any, float], Any]

_TV = TypeVar("_TV")
InT = TypeVar("InT", bound=In)


class Interpolatable(Protocol):
    """Self-describing interpolation: a.interpolate(b, alpha) -> value at alpha."""

    def interpolate(self, other: Self, alpha: float) -> Self: ...


def register_interpolator(tp: type[_TV], fn: Callable[[_TV, _TV, float], _TV]) -> None:
    """Register fn(a, b, alpha) as the interpolator for tp (last wins)."""


def interpolator_for(tp: type[Any]) -> InterpolatorFn | None:
    """Resolve tp's interpolator (§6.2 lookup order), or None."""


class AlignRule(enum.Enum):
    """Every alignment rule, by its message slug (§9)."""
    NO_TICK = "align-no-tick"
    NOT_IN_ROW = "align-not-in-row"
    UNKNOWN_PORT = "align-unknown-port"
    MISSING_TICK_STREAM = "align-missing-tick-stream"
    MISSING_REQUIRED_STREAM = "align-missing-required-stream"
    SHARED_ITERATOR = "align-shared-iterator"
    UNSUPPORTED_KIND = "align-unsupported-kind"
    NOT_INTERPOLATABLE = "align-not-interpolatable"
    INTERP_UNTYPED = "align-interp-untyped"
    BAD_ITEM = "align-bad-item"
    BAD_TS = "align-bad-ts"
    UNSTAMPED_ITEM = "align-unstamped"


class AlignmentError(TypeError):
    """A stream wiring or stream data contract violation (§9)."""

    rule: AlignRule | None

    def __init__(self, message: str, rule: AlignRule | None = None) -> None: ...


@dataclasses.dataclass
class PortStats:
    """Monotonic per-port ingestion counters (mutated only by the pull loop)."""

    accepted: int = 0              # items past the monotonic filter (incl. a pending head)
    dropped_nonmonotonic: int = 0  # items with ts <= last accepted ts, discarded


@dataclasses.dataclass
class AlignStats:
    """Monotonic per-aligner counters; safe to read from another thread."""

    ticks_fired: int = 0    # tick items processed (emitted + dropped)
    rows_emitted: int = 0
    ticks_dropped: int = 0  # ticks with >= 1 unresolvable required field
    drops_by_field: dict[str, int] = dataclasses.field(default_factory=dict)
    ports: dict[str, PortStats] = dataclasses.field(default_factory=dict)


class Aligner(Generic[InT]):
    """One-shot iterator of aligned In rows; owns counters, owns no resources."""

    def __init__(self, in_type: type[InT], streams: Mapping[str, Iterable[Stamped]]) -> None:
        """Validate wiring eagerly (§7); pulls nothing until iteration."""

    def __iter__(self) -> Aligner[InT]: ...
    def __next__(self) -> InT: ...

    @property
    def stats(self) -> AlignStats:
        """Live counters (§8); reads are safe at any time."""

    @property
    def held_tick_ts(self) -> float | None:
        """ts of the tick currently held mid-merge, else None (§8.2)."""


def align(in_type: type[InT], streams: Mapping[str, Iterable[Stamped]]) -> Aligner[InT]:
    """Resolve stamped streams into tick-stamped In rows (validates now, pulls lazily)."""
```

Surface notes:

- `align()` is the public entry; `Aligner` is public for annotations and for
  reaching `.stats`. Construction via `align()` in every doc/example.
- `streams` is a `Mapping`, keyed by **port name** = In-bundle field name
  (from `in_type.fields()`). It is read once at construction; later mutation
  of the mapping has no effect. `iter()` is called on each value exactly once,
  at construction (D13) — passing a one-shot iterator is fine; passing the
  *same iterator object* for two ports is an error (§7 #6).
- The aligner is a **one-shot** iterator (`__iter__` returns self). It owns
  no resources and has no `close()`: input-iterable lifecycle belongs to the
  caller (house style — caller-managed lifecycle; T6's teardown closes what
  T6 opened).
- Re-exports for `__init__.py` (integrator applies; file not owned by T5):
  `align`, `Aligner`, `Stamped`, `AlignmentError`, `Interpolatable`,
  `register_interpolator`. Stats types and `AlignRule`/`interpolator_for`
  stay importable from `dimos.pure.align` without `pm.` aliases.

## 3. The input currency — `Stamped` (D1, D2)

```python
class Stamped(Protocol):
    @property
    def ts(self) -> float: ...
```

- **Read-only property spelling, verified.** The graph sketch
  (`puregraph_api_sketch.py` §transport) spells it `ts: float` with the
  comment "stamped msgs and In/Out rows both qualify" — but a plain protocol
  variable demands settability, and mypy **rejects frozen dataclasses** (all
  T1 rows) and property-backed msgs against it ("expected settable variable,
  got read-only attribute" — verified on this branch). The property spelling
  accepts frozen rows, plain-attribute msgs (`dimos.types.timestamped.
  Timestamped` and every msg class), and property implementations. This is
  T4 §5.3 doctrine applied. T6 should adopt this exact protocol when
  tightening `typing.Streamable` (§11.2); the graph effort inherits the
  corrected spelling.
- **Items are row values, verbatim (D2).** What a stream yields is exactly
  what lands in the In field: `align` reads `item.ts` once at ingestion and
  never inspects, copies, converts, or unwraps the item. ONE timestamp
  authority = the payload's own ts. Consequence: a `dimos.memory2` `Stream`
  (which yields `Observation` wrappers) must be coerced to a stamped-payload
  iterator *before* reaching `align` — that is T6's obligation at the
  `over()` boundary (§11.1). "Stamped payloads with no Observation wrapper
  in dataflow" (sketch banner §5) is enforced by construction here.
- **ts contract**: readable, `int | float`, finite. `float(item.ts)` is the
  ingested value. Violations are errors, not drops (§4.2, §9): a missing/
  unreadable ts is `[align-bad-item]`, a non-finite ts is `[align-bad-ts]`,
  and exactly `-inf` is `[align-unstamped]` — an Out row that reached
  alignment without being stamped by its driver (T1 `UNSTAMPED`), a real
  cross-layer bug class caught by name.
- No runtime payload-*type* validation (D15): `align` never checks items
  against field annotations — rows are dumb data on the hot path (T1 D3);
  mypy and the T6 coercion own payload typing.

## 4. Denotational semantics (normative)

The meaning of `align(in_type, streams)` is defined *denotationally* over
logical per-port item sequences — independent of buffering, chunking, pull
timing, or arrival interleaving. §5 gives the operational model and shows it
realizes this definition. Any behavior difference between §4 and an
implementation is an implementation bug.

### 4.1 Ports

`in_type.fields()` (T1 §5.2: declaration-ordered, ts excluded, may raise
`BundleDefinitionError` E-UNRESOLVED — propagated untouched, §9) partitions
into:

- exactly one `TickSpec` field — the **trigger** (§7 #2 enforces at-least-one
  here; at-most-one is T1 D5's definition-time half);
- zero+ `LatestSpec` fields;
- zero+ `InterpolateSpec` fields;
- any other In-side kind → `[align-unsupported-kind]` (the T11 seam, §11.5).

Each field is a **port**, identified by name, ordered by declaration index
(base-first, T1 ordering guarantee). `TickSpec.expect_hz` is a health hint —
ignored by T5 (T9 reads it). Out-side specs cannot occur (T1 side rules).

### 4.2 Kept sequences — the monotonic filter (D3, D4)

For each port `p` with a stream, the **kept sequence** `S_p` is derived from
the stream's yields in order:

- an item with unreadable/non-float ts → error `[align-bad-item]`/`[align-bad-ts]`
  (raised from the `__next__` call that pulls it; `-inf` refines to
  `[align-unstamped]`);
- an item with `ts <= ` the ts of the **last kept** item on `p` → dropped,
  `dropped_nonmonotonic += 1`, not an error;
- otherwise the item is kept (appended to `S_p`), `accepted += 1`.

So `S_p` is **strictly increasing** in ts. Strictness (rather than
non-decreasing) is deliberate: it discards duplicate deliveries and ts
regressions with one rule, makes tick ts strictly increasing (row output is
strictly ts-ordered — what T6's fold validation and T10's cursor lean on),
and makes interpolation brackets non-degenerate (no zero-width division,
§4.4). Drop-not-error is deliberate too: live transports reorder and
redeliver as a fact of life, and T8 routes *all* drop policy into the engine
(index §T8); a lone late packet must not kill a run — it is counted, and T9
surfaces it. Forward note, not built: a source that legitimately emits
distinct equal-ts items (the RPC-request merge sketched for the rim era
breaks equal ts by envelope sequence) will need a per-port sequence
component in item identity; out of scope with RPC itself.

An optional port with no stream has `S_p = ()` (§7 #5). The tick port's kept
sequence is `S_tick`; its elements are the **tick items**.

### 4.3 The total order (D6)

The engine's event order — replay identity, the order T8's flight recorder
and T10's cursor inherit — is, normatively:

> events ordered by `(ts, port declaration index, per-port arrival sequence)`.

Per-port arrival sequence never ties (strict monotonicity per §4.2), so the
order is total. Ticks are events like any other; resolution (§4.4) is
defined so that its outcome is a function of the sequences alone — the
equal-ts corner (a secondary item at exactly a tick's ts, on either side of
the tick's declaration index) resolves identically regardless of which event
the order commits first (§5.4 shows the mechanics).

### 4.4 Resolution (normative)

For each tick item `k` in `S_tick`, in order, with `T = k.ts`. Define per
secondary port `p`:

- `L(p, T)` = the last element of `S_p` with `ts <= T` (none if no such);
- `R(p, T)` = the first element of `S_p` with `ts >= T` (none if no such).

Each secondary field resolves to a **value**, or is **missing**:

| Field kind | Rule |
| --- | --- |
| `latest` | value = `L(p, T)` if it exists; else missing |
| `interpolate` | if `L` exists and `L.ts == T` → value = `L` (exact hit, interpolator NOT called); elif `L` and `R` both exist (then `L.ts < T < R.ts`) → value = `interp(L, R, alpha)` with `alpha = (T - L.ts) / (R.ts - L.ts)`; else missing |

- **No extrapolation, ever**: one-sided data (`L` only or `R` only) never
  resolves an interpolate field. `alpha` handed to an interpolator is
  strictly inside `(0, 1)` and the bracket has strictly positive width —
  guaranteed by strict monotonicity + the exact-hit peel. Interpolators are
  expected to be pure functions of `(a, b, alpha)` (determinism §10 assumes
  it; an impure interpolator forfeits replay identity for its field only).
- A missing field with a default (`spec.required` is False, T1 §5) resolves
  to `spec.default` — **the default object itself**, delivered by reference,
  identical across rows (defaults are plain data, never copied).
- A missing **required** field drops the tick: `ticks_fired += 1`,
  `ticks_dropped += 1`, and `drops_by_field[name] += 1` for **every** missing
  required field (all fields are checked, not first-missing —
  `sum(drops_by_field.values()) >= ticks_dropped` is expected). No row is
  emitted for that tick.
- Otherwise the row is emitted: `in_type(ts=T, **values)` with the tick field
  = `k` itself — the T1 §4.4 construction, `InCls(ts=tick_ts, **resolved)`,
  verbatim. `ticks_fired += 1`, `rows_emitted += 1`.

The output of `align` is the sequence of emitted rows, in tick order —
strictly increasing `row.ts`.

**Resolution is two-pass (D12)**: pass 1 decides resolvability for every
field (no interpolator calls); pass 2 materializes values — so interpolators
run **only for ticks that emit**. User interpolation code is never invoked
on a dropped tick.

### 4.5 Hold vs drop — the pull-based answer (D5)

The index demands "REQUIRED fields hold the tick until resolvable" with no
blocking waits. In a pull-based component the two states are:

- **Held**: a tick item has been pulled (it is the tick stream's frontier)
  but has not yet *won the merge* — some other port's frontier is still at
  or behind `T`. While held, the aligner pulls **other** streams forward
  (that pulling *is* the hold: in a replay it's instantaneous; behind T8's
  queue-backed iterators it is the engine parked on data that hasn't arrived
  — latency skew, the case the old `_wait_get` blocked a thread for). No
  thread waits, no wall clock, no timeout: time comes from payloads only.
- **Dropped**: the moment a tick wins the merge, every resolution question
  is **decidable forever** — every other port's kept frontier is strictly
  past `T` (or its stream is exhausted), and by strict per-port monotonicity
  no future item can land at or before `T`, while the right bracket, if it
  will ever exist, is already the frontier item. So `L(p, T)` and `R(p, T)`
  are fully known at fire time, and a missing required field is a *final*
  fact. Held ticks are therefore processed exactly once; there is no revisit
  queue — "later resolvable" is the empty set by construction.

Corollaries, stated honestly: a live required stream that stays silent holds
the head tick indefinitely (align blocks in `next()` on that stream — the
correct pure semantics; dead-sensor detection is T9's, cutoff policy is
T8's, §11.3). Emission is in-order by design, so a held tick head-of-line
blocks later ticks — rows must be ts-ordered.

## 5. Operational model

### 5.1 State

Per port, a `_PortState` (private dataclass):

| Field | Meaning |
| --- | --- |
| `name`, `index` | port name; declaration index (merge tiebreak) |
| `spec` | the T1 `FieldSpec` (kind, default, annotation) |
| `it: Iterator[Stamped] \| None` | live iterator; `None` = exhausted or no stream |
| `head: Stamped \| None` | the frontier: pulled, filter-passed, not yet committed |
| `head_ts: float` | `float(head.ts)`, cached at pull |
| `newest: Stamped \| None` | the last **committed** item (and its cached ts) |
| `last_ts: float` | ts of the last accepted item (monotonic filter cursor), init `-inf` |
| `interp: InterpolatorFn \| None` | resolved at construction for interpolate ports |
| `stats: PortStats` | shared into `AlignStats.ports` |

**Buffering is O(1) per port** — `newest` + `head`, nothing else. This is
not an optimization but a consequence of §4/§4.5: at fire time
`L(p, T)` ∈ {`newest`, `head`} and `R(p, T)` ∈ {`newest`, `head`} (§5.4),
and ticks only move forward, so anything older than `newest` is
unreachable by any future resolution. There is no history deque and no
retention window (D7; relitigated in §15.1 — the index names a retention
knob; the goal it serves, bounded memory on a never-ticking module, is met
by construction: a silent tick stream leaves secondaries at one held item
each, O(#ports) total).

### 5.2 The pull loop (normative pseudocode)

```
__next__():
  loop:
    # (a) refill: every port with a live iterator and an empty head pulls
    #     one kept item (declaration order — the deterministic pull order):
    for p in ports:                      # declaration order
      while p.it is not None and p.head is None:
        item = next(p.it)                # StopIteration → p.it = None
        ts = _checked_ts(p, item)        # §4.2 errors raise from here
        if ts <= p.last_ts: p.stats.dropped_nonmonotonic += 1; continue
        p.head, p.head_ts, p.last_ts = item, ts, ts
        p.stats.accepted += 1

    # (b) terminate: the trigger is done ⇒ alignment is done (secondaries
    #     are NOT drained — lazy; their pending heads simply age out):
    if tick.head is None and tick.it is None: raise StopIteration

    # (c) select the least event by (head_ts, index) over ports with heads:
    p = min(ports with head, key=(head_ts, index))

    # (d) data event: commit into newest, loop:
    if p is not tick: p.newest = p.head; p.head = None; continue

    # (e) tick event fires: T = tick.head_ts; k = tick.head.
    #     Merge-min guarantees every other port: head is None (exhausted /
    #     no stream) or head_ts > T or (head_ts == T and index > tick.index).
    resolve per §5.4 → emitted row | drop
    tick.head = None                     # consume the trigger either way
    if dropped: continue
    return row
```

Notes:

- Refill-at-loop-top means nothing is pulled beyond what selecting the next
  event requires; after a row is returned, the next tick item is pulled by
  the *next* `__next__` call. Step (b) before (c) means an exhausted tick
  stream ends iteration without pulling secondaries further.
- All state mutation happens inside `__next__` on the calling thread —
  thread-free by construction. `stats`/`held_tick_ts` reads from another
  thread see monotonic ints / a float-or-None (GIL-atomic attribute reads;
  no locks anywhere).
- Exceptions raised by input iterators or by `_checked_ts` propagate from
  the `__next__` that pulled the offending item — a deterministic position.
  The aligner makes no attempt to continue past a raising stream.

### 5.3 Merge invariant

When a tick fires at `T` (step e), for every secondary port `p`:

1. every kept item with `(ts, index) < (T, tick.index)` has been committed —
   so `p.newest` is the last kept item with `ts < T`, or `ts == T` when
   `p.index < tick.index`;
2. `p.head`, if present, is the first kept item with
   `(ts, index) > (T, tick.index)` — so `head_ts > T`, or `head_ts == T`
   when `p.index > tick.index`;
3. nothing between `newest` and `head` exists (they are adjacent in `S_p`).

Hence `L(p, T)` = `head` if `head_ts == T` else `newest` (when
`newest.ts <= T`, always true by 1); and `R(p, T)` = `newest` if
`newest.ts == T` else `head` (when `head_ts >= T`, always true by 2).

### 5.4 Fire-time resolution table (implements §4.4)

For each secondary field, in declaration order — pass 1 (decide), then
pass 2 (materialize, only if no required field is missing):

| Kind | Decision |
| --- | --- |
| `latest` | candidate `c` = `head` if `head_ts == T` else `newest`; value = `c` if `c` exists else default/missing |
| `interpolate` | exact hit: `head_ts == T` → `head`; `newest.ts == T` → `newest`. Else bracket: `newest` and `head` both present → `interp(newest, head, (T − newest.ts)/(head_ts − newest.ts))`. Else default/missing |

The two `== T` branches are exactly the §4.3 equal-ts corner: an equal-ts
secondary on a lower-index port has been committed (`newest.ts == T`), on a
higher-index port it is the pending head (`head_ts == T`). Both resolve —
resolution is order-independent, as §4.4 requires. Resolution **never
consumes** items: committing is the merge's business only, so consecutive
ticks sharing one bracket (dense trigger, sparse secondary) each resolve
against it.

## 6. Interpolation — protocol + registry (D8, D9)

### 6.1 Surface

Type-specific interpolation, never engine special-cases:

- **Self-describing types** implement
  `def interpolate(self, other: Self, alpha: float) -> Self` (the
  `Interpolatable` protocol; name verified unclaimed across `dimos/msgs` and
  `dimos/protocol` — no clash). T11's `Transform.slerp`-style math lands as
  this method or a registration, not as align.py code.
- **Foreign types** (`float`, numpy scalars, closed classes) get
  `register_interpolator(tp, fn)` with `fn(a, b, alpha) -> value`.
  Registration is import-time configuration (module-level table); last
  registration for a type wins; not synchronized — register at import time.
- Pre-seeded: `float` → `lambda a, b, alpha: a + (b - a) * alpha`. Nothing
  else — `int`/`bool` interpolation is ill-defined and deliberately absent
  (declaring `interpolate()` on an `int` field is a wiring error, §9).
  `numpy.float64` resolves via MRO (subclasses `float`).
- Honest note on the float seed: a bare-`float` In field passes §6.3 wiring
  but can never be *fed* — bare floats carry no ts, so any stream item for
  it fails ingestion (§4.2). Primitives ride a stamped envelope msg at the
  rim (the graph-sketch transport convention); the float seed exists for
  envelope implementations composing their fields' interpolation, for
  MRO-derived scalar types, and for direct `interpolator_for(float)` use.

### 6.2 Lookup (`interpolator_for`)

For each class `C` in `tp.__mro__`, in order: (1) the registry entry for
exactly `C`, if present; (2) `C`'s **own-dict** `interpolate` callable, if
defined — wrapped as `lambda a, b, alpha: a.interpolate(b, alpha)`. First
hit wins; `None` if the walk exhausts. Interleaving per MRO level means the
most-derived declaration wins regardless of mechanism (a subclass's method
beats a base's registry entry and vice versa at the subclass level).

### 6.3 Dispatch is declared-type, at wiring (D9)

For every `interpolate()` field, `align()` resolves the interpolator **at
construction** from the field's resolved annotation (`fields()[name]
.annotation`), after stripping an optional `| None` (any spelling). Rules:

- stripped annotation is not a single runtime class (a union of two+
  payload types, `Any`, a parameterized alias) → `[align-interp-untyped]`;
- `interpolator_for(cls)` is `None` → `[align-not-interpolatable]`.

Both are wiring errors at `align()` time — the "non-interpolatable type with
`interpolate()`" case fails before any data moves. At resolution time the
*pre-resolved* function is used unconditionally: dispatching on runtime
sample types could diverge mid-stream and is rejected; samples are assumed
instances of the declared type (not validated, D15 — a lying stream gets
whatever the interpolator does with its items).

## 7. Wiring validation at `align()` (D10, D14)

Runs eagerly in `Aligner.__init__` — wiring errors surface at the call, not
at first `next()`. In order:

1. `in_type` must be a class subclassing `pm.In` (and not `In` itself is not
   required — the root simply has no tick and fails #2) →
   `[align-not-in-row]`. `in_type.fields()` is called once; a
   `BundleDefinitionError` (T1 E-UNRESOLVED — unresolvable field types
   surface at wiring, per the T1 consumption map) propagates **untouched**:
   T1's message is the teaching one.
2. Exactly one `TickSpec` field → zero ticks is `[align-no-tick]`
   (at-least-one lives here — T1 D5's other half; >1 is unreachable, T1
   E-MULTITICK).
3. Every In-side spec kind must be tick/latest/interpolate →
   `[align-unsupported-kind]` (T11 seam).
4. Every `streams` key must name a declared port → `[align-unknown-port]`
   (message enumerates the ports; when the key is `"ts"` it additionally
   teaches that ts is engine-stamped row infrastructure, not a port).
5. The tick port must have a stream → `[align-missing-tick-stream]`. Every
   **required** latest/interpolate port must have a stream →
   `[align-missing-required-stream]` (every tick would drop unresolved;
   failing loud beats a run that silently emits nothing). An **optional**
   port may be omitted: it resolves to its default on every tick — the
   degraded-sensor mode `default=` exists for (sketch §4's
   `relocalized_map` without a relocalizer).
6. `iter()` is called once per stream value; if the *same iterator object*
   arrives under two ports (`iter(x) is x` and identity-shared) →
   `[align-shared-iterator]` — two ports draining one iterator would
   interleave-steal items. The same *re-iterable* (a list) under two ports
   is legal (independent iterators).
7. Interpolator resolution per §6.3.

## 8. Accounting — the T9 seam (D12)

### 8.1 Counters

`AlignStats` / `PortStats` as spelled in §2. Semantics:

- `ports` maps **every declared secondary port and the tick port** by name
  (streamless optional ports included, permanently zero — the dict shape is
  fixed at construction, so T9 never key-checks).
- `accepted` counts items past the monotonic filter, including a pending
  head not yet committed. `dropped_nonmonotonic` counts §4.2 drops.
- `ticks_fired` = tick items processed (= `rows_emitted + ticks_dropped`).
- `drops_by_field` holds only required-field names that have caused a drop;
  keys appear on first drop.
- Counters are plain ints mutated only inside `__next__`; cross-thread reads
  (the T9 pacer) need no lock. **Counters are deterministic** (§10): the
  pull pattern is a function of the logical inputs, so replays reproduce
  stats exactly, not just rows.

### 8.2 The held gauge

`Aligner.held_tick_ts` — the ts of the tick currently held (pulled, not yet
fired), else `None`. Between `__next__` calls it is `None` by construction
(a call returns only by firing or `StopIteration`); its value is *observed
mid-call from another thread* — precisely T9's "module held on tick T for
N seconds of data time" diagnostic while a pull is parked on a silent
stream. Implementation: an attribute written when the tick head is pulled
(step a) and cleared when consumed (step e).

No callbacks, no hook objects (counters suffice for T9's brief; "spec the
counter interface, don't build health").

## 9. Error catalog

One exception class: `AlignmentError(TypeError)` with a machine-readable
`rule: AlignRule | None` (the T3 pattern: release-copy message +
`[slug]` suffix). `{bundle}` = `f"{in_type.__module__}.{in_type.__qualname__}"`.
Wiring errors (W) raise from `align()`; data errors (D) raise from the
`__next__` that pulls the offending item. Monotonicity violations are
**drops, never errors** (§4.2). T1's `BundleDefinitionError` from `fields()`
propagates unwrapped.

| Rule | When | Message template |
| --- | --- | --- |
| `NOT_IN_ROW` (W) | in_type not an In subclass | `align() takes a pm.In row bundle, got {obj!r} — pass the module's In class (its step signature names it; T6 passes spec.in_type). [align-not-in-row]` |
| `NO_TICK` (W) | no TickSpec field | `{bundle}: declares no tick() field — alignment needs exactly one trigger to drive rows. Mark exactly one In field as tick(); latest()/interpolate() fields resolve at its ts. [align-no-tick]` |
| `UNSUPPORTED_KIND` (W) | unknown In-side spec kind | `{bundle}: field {name!r} declares {kind}(), which this resolver does not support (tick/latest/interpolate). New sampler kinds must extend the alignment engine (tf() arrives with T11). [align-unsupported-kind]` |
| `UNKNOWN_PORT` (W) | streams key not a port | `{bundle}: unknown stream {key!r} — declared ports: {names}.{ts_note} [align-unknown-port]` where `ts_note` = ` (ts is row infrastructure, stamped by the engine — not a port.)` iff key == "ts", else empty |
| `MISSING_TICK_STREAM` (W) | tick port has no stream | `{bundle}: no stream for tick port {name!r} — the trigger drives every row; without it nothing ticks. Pass {name}=<stream>. [align-missing-tick-stream]` |
| `MISSING_REQUIRED_STREAM` (W) | required secondary has no stream | `{bundle}: no stream for required {kind}() port {name!r} — every tick would drop unresolved. Pass {name}=<stream>, or give the field default= to make it optional. [align-missing-required-stream]` |
| `SHARED_ITERATOR` (W) | one iterator object under two ports | `{bundle}: ports {a!r} and {b!r} share one iterator object — each port consumes its stream independently; a shared iterator would interleave-steal items. Pass independent iterables. [align-shared-iterator]` |
| `INTERP_UNTYPED` (W) | interpolate annotation not one class | `{bundle}: field {name!r} is interpolate() but its annotation {ann} is not a single class — interpolation dispatches on the declared payload type (\| None is fine with default=None). [align-interp-untyped]` |
| `NOT_INTERPOLATABLE` (W) | no interpolator for the class | `{bundle}: field {name!r} is interpolate() but {cls} has no interpolator — implement interpolate(self, other, alpha) on the type or register_interpolator({cls.__name__}, fn); use latest() if nearest-sample semantics suffice. [align-not-interpolatable]` |
| `BAD_ITEM` (D) | item.ts missing/unreadable/non-numeric | `{bundle}.{port}: stream yielded {type} with no readable float ts — align consumes stamped payloads (msgs and rows carry ts). [align-bad-item]` |
| `BAD_TS` (D) | ts non-finite (nan/+inf) | `{bundle}.{port}: item ts is {value!r} — ts must be a finite float. [align-bad-ts]` |
| `UNSTAMPED_ITEM` (D) | ts == -inf | `{bundle}.{port}: item ts is UNSTAMPED (-inf) — an Out row reached alignment without being stamped by its driver. Rows on the wire are stamped by the engine; hand-built feeds must set ts. [align-unstamped]` |

## 10. Determinism (normative property)

The emitted row sequence **and** the final `AlignStats` are a pure function
of `(in_type, the logical per-stream item sequences)`. Not consulted, by
construction: wall clock, thread timing, the `streams` mapping's key order
(port order comes from `fields()`), iterable chunking or delivery batching
(the iterator protocol yields one item at a time; the pull pattern in §5.2
depends only on item content), dict iteration order anywhere, process
randomness. Same inputs → byte-identical rows (T1 rows have field-tuple
equality; payload equality is the msg layer's obligation, T1 §4.1 caveat).

The property test (§12, `test_determinism_*`): random schedules × {list
input, generator input, `itertools.chain` of shards, shuffled mapping
order} → identical row lists and identical stats; plus engine-vs-oracle
equivalence, where the oracle is a direct transcription of §4 (materialize
`S_p`, resolve each tick by `L`/`R`) — the denotational spec doubles as the
reference implementation.

## 11. Boundaries (referenced, not designed)

### 11.1 T6 — `over()` and stream coercion (binding)

- `run_over(module, spec, streams)` (T4 §4.4) builds
  `rows = align(spec.in_type, streams)` and drives the T3-classified step
  over it; for fold modules it hands `rows` to `fold` directly (sketch §7's
  `Iterator[VoxelGridMapper.In]` is exactly `Aligner[VoxelGridMapper.In]`).
- **Coercion obligation**: `align` consumes stamped payloads verbatim (D2).
  A `dimos.memory2.stream.Stream` yields `Observation` wrappers; passing it
  raw would put Observations into row fields. T6 must coerce stream objects
  at the `over()` boundary — recommended: a lazy
  `isinstance(x, dimos.memory2.stream.Stream)` check (function-local import,
  the sanctioned lazy-edge pattern) unwrapping to `(obs.data for obs in x)`
  when payloads are stamped. The obligation is T6's to spec; T5's contract
  is only: whatever iterable T6 passes, §4 semantics apply to its items.
- Teardown: the aligner owns nothing; T6 closes whatever iterators/streams
  T6 opened (index §T6 early-exit rule), including on consumer break.

### 11.2 T6 — tightening `typing.Streamable`

Recommended: `Streamable: TypeAlias = Iterable[Stamped]` — accepts raw
stamped-msg iterables *and* memory2 `Stream` objects statically
(`Stream.__iter__` yields `Observation`, which satisfies read-only-property
`ts`). Where `Stamped` lives: preferred, `typing.py` imports it from
`dimos.pure.align` (both are data-layer, edge is acyclic — align imports
only rows — but this relaxes t4-typing.md §6's "imports only
collections.abc + typing" by one sibling); alternative, `typing.py` declares
a structural twin (protocols compare structurally — no user-visible seam,
at the cost of two definitions). T6's call; T5 ships the protocol either
way.

### 11.3 T8 — the live rim

The pull merge needs each port's *frontier* to fire a tick — that is
intrinsic (§4.5), not an implementation choice. T8 can drive alignment as
queue-backed iterators whose `next()` parks the module loop until arrival
(the hold made temporal), accepting head-of-line blocking on silent
required streams as the semantics — with dead-sensor cutoff as rim/health
policy layered above — or drive the §5 resolution core push-mode from its
own loop. Either way: T5's buffers stay O(1); arrival backlog, bounded
queues, and every drop *policy* live in T8's buffers (index §T8); the
currency is `Stamped` on both sides.

### 11.4 T10 — cursor and resume

Alignment progress is fully characterized by the last fired tick's ts
`T_c` (ticks process atomically — there is no mid-tick state; "checkpoints
only at tick boundaries" is trivially satisfiable). Aligner state at `T_c`
beyond the cursor is exactly `{port: newest item with ts <= T_c}` —
O(#ports) plain data. A resume replays each stream filtered to
`ts > T_c` after re-seeding per-port `newest`/`last_ts` (or simpler:
re-pulls from recorded streams and skips rows with `ts <= T_c`). T10 specs
the snapshot; T5 guarantees the characterization.

### 11.5 T11 — tf as a sampler extension

`tf()` arrives as a new In-side `FieldSpec` kind, rejected today by
`[align-unsupported-kind]` — the deliberate extension point. T11 extends
align.py with a port-state kind whose backing store is the engine-owned
shared MultiTBuffer-derived accumulator (per-*edge* history — a multiplexed
tf stream breaks the one-port-one-sequence model, which is why it is not
shoehorned into `_PortState`), passed in from the run context, never a
process global. Chain composition = this resolver's hold semantics + graph
search; `Transform` interpolation registers through §6 (slerp adapted from
`dimos/protocol/tf/tf.py` math, per the index; `_wait_get` stays dead —
holds, not blocking gets). Every interpolated tf value appears in the
emitted row — edges stay recordable (index preclusion guard).

## 12. Test plan

`dimos/pure/test_align.py` ships with this spec — real bodies, module-level
`pytestmark = pytest.mark.skip(reason="T5 skeleton — enable with
implementation")`; the implementer deletes that line. Fixtures: a tiny local
`@dataclass S(ts, v)` stamped payload + bundles built from real T1 machinery
at module level (rows are landed; only `align()` calls stay inside skipped
test bodies). Zero engine imports beyond `dimos.pure.align` +
`dimos.pure.rows`. The denotational oracle (§10) lives in the test file.

| Test | Pins |
| --- | --- |
| `test_tick_only_bundle` | single tick port: every item → row; `row.ts == item.ts`; tick field is the item itself (identity) |
| `test_latest_resolution_matrix` | latest × {empty, before-only, exact-hit, spanning}: value/`L(p,T)` per §4.4 |
| `test_latest_required_warmup_drops` | ticks before first secondary drop; counted in `ticks_dropped` + `drops_by_field`; later ticks emit |
| `test_latest_optional_defaults` | missing → default; default object identity shared across rows |
| `test_optional_port_stream_omitted` | omitted optional port: default every tick; `ports` entry exists at zero |
| `test_interpolate_exact_hit_and_bracket` | exact hit returns the sample (identity, interpolator not called); bracket interpolates a registered stamped type (numeric, alpha exact at 0.5) |
| `test_interpolate_one_sided_no_extrapolation` | before-only and after-only → drop (required) / default (optional) |
| `test_interpolate_tail_exhausted` | ticks after the last secondary sample drop (no right bracket) |
| `test_multiple_ticks_share_bracket` | dense tick, sparse secondary: consecutive ticks resolve against one bracket; resolution consumes nothing |
| `test_equal_ts_secondary_lower_index` | secondary at exactly T, declared before tick: exact hit |
| `test_equal_ts_secondary_higher_index` | secondary at exactly T, declared after tick: exact hit (the pending-head corner) |
| `test_monotonic_regression_dropped` | ts regression + duplicate ts dropped + counted; not an error; rows unaffected |
| `test_tick_port_monotonic_filter` | duplicate tick ts dropped; output strictly ts-increasing |
| `test_nan_ts_raises` / `test_unstamped_ts_raises` | `[align-bad-ts]` for nan/+inf; `[align-unstamped]` for -inf with the stamping hint; raised at the pulling `next()` |
| `test_bad_item_raises` | ts-less item → `[align-bad-item]` naming bundle.port |
| `test_hold_is_pull_order` | instrumented generators: the tick row is not yielded until the secondary iterator has been pulled past T (the operational hold, pinned) |
| `test_lazy_tail` | tick stream exhausts → StopIteration without draining remaining secondary items (pull counts pinned) |
| `test_empty_tick_stream` | no ticks → empty output, no error |
| `test_wiring_no_tick` / `test_wiring_not_in_row` | `[align-no-tick]`, `[align-not-in-row]` at `align()` time |
| `test_wiring_unknown_port_and_ts_key` | `[align-unknown-port]`; the ts-key variant carries the infrastructure note |
| `test_wiring_missing_tick_and_required_streams` | `[align-missing-tick-stream]`, `[align-missing-required-stream]` |
| `test_wiring_shared_iterator` | same iterator object twice → `[align-shared-iterator]`; same list twice → legal |
| `test_wiring_errors_are_eager` | all wiring errors raise from `align()`, before any pull |
| `test_interp_registry_and_protocol` | `interpolator_for(float)` seed pinned directly (§6.1 note); `interpolate`-method type through align; last-registration-wins |
| `test_interp_mro_precedence` | subclass method beats base registry entry; same-level registration beats the method; unrelated type → None |
| `test_interp_optional_none_annotation` | `X \| None` + `interpolate(default=None)` dispatches on X |
| `test_interp_not_interpolatable_and_untyped` | int field → `[align-not-interpolatable]`; `Any`/union annotation → `[align-interp-untyped]` |
| `test_stats_scenario` | a scripted schedule → exact `AlignStats` (all counters), `ticks_fired == rows_emitted + ticks_dropped` |
| `test_interpolator_not_called_on_dropped_tick` | doomed tick (missing required latest) with a resolvable interpolate field: interpolator never invoked (two-pass, D12) |
| `test_inherited_bundle_aligns` | a bundle extending another's In: merged `fields()` ports align; base-first order respected |
| `test_determinism_chunking_and_mapping_order` | list vs generator vs chained shards vs shuffled dict order → identical rows and stats |
| `test_determinism_oracle_random_schedules` | seeded random schedules: engine ≡ denotational oracle (rows + drop pattern) |
| `test_aligner_is_one_shot` | `iter(a) is a`; exhausted aligner stays exhausted |

## 13. Acceptance criteria

- [ ] `uv run mypy dimos/pure/align.py` clean; `uv run mypy dimos/pure`
      stays clean (strict, repo config).
- [ ] `align.py` imports stdlib + `typing_extensions` +
      `dimos.pure.rows` only — no engine, no memory2, no clock, no threads.
- [ ] `uv run pytest dimos/pure -q`: suite collects; pre-existing 167 pass;
      T5 tests skip against the skeleton and pass once implemented.
- [ ] §4 denotational semantics implemented exactly; §10 oracle equivalence
      green under the seeded property tests.
- [ ] All §9 errors raise `AlignmentError` with the templates and rules;
      wiring errors eager; data errors at the pulling `next()`; T1's
      `BundleDefinitionError` propagates unwrapped.
- [ ] O(1) per-port state (no history collections anywhere).
- [ ] Interpolators invoked only for emitted rows, never with alpha ∉ (0,1).
- [ ] Sketch floor unchanged: `Tagger`/`CostMapper` bundles align with zero
      added declarations; `over(global_map=..., relocalized_map=...)`
      kwarg spelling maps to ports by name.
- [ ] Stats deterministic under re-runs of identical inputs.

## 14. Decisions within mandate (numbered for review)

- **D1** `Stamped` is a read-only-property protocol (verified: the
  plain-attribute spelling rejects frozen rows); defined in `align.py` (§3).
- **D2** Items are row values verbatim; ONE ts authority = `item.ts`;
  no Observation unwrapping in T5 — coercion is a T6 boundary obligation
  (§3, §11.1).
- **D3** Per-port monotonicity is **strict** (`ts <= last kept` ⇒ drop +
  count, never an error); equal-ts duplicates die with regressions (§4.2).
- **D4** Non-finite ts is an error, not a drop; `-inf` is recognized as T1
  `UNSTAMPED` with its own rule (§3, §9).
- **D5** Hold = merge-pull order; drop decisions are provably final at fire
  time; no revisit queue exists (§4.5).
- **D6** Total order `(ts, port declaration index, per-port seq)`;
  resolution defined denotationally so the equal-ts corner is
  order-independent (§4.3, §5.4).
- **D7** Per-port state is O(1) (`newest` + `head`); no retention window
  (§5.1; relitigated §15.1).
- **D8** Interpolators are `fn(a, b, alpha)`, alpha strictly in (0,1),
  exact hits bypass, no extrapolation; registry + `interpolate` method,
  MRO-interleaved lookup, `float` pre-seeded, last-registration-wins (§6).
- **D9** Interpolation dispatches on the **declared** annotation
  (Optional-stripped) resolved at `align()`; runtime-type dispatch rejected
  (§6.3).
- **D10** Optional ports may omit streams; tick + required ports must have
  them; unknown keys and shared iterator objects are wiring errors (§7).
- **D11** One `AlignmentError(TypeError)` + `AlignRule` slugs, T3-style;
  monotonic drops are counters, not errors (§9).
- **D12** Two-pass resolution: interpolators (user code) run only for
  emitted rows (§4.4).
- **D13** `align()` returns an `Aligner` — one-shot iterator carrying
  `.stats` + `.held_tick_ts`; owns no resources, no `close()`;
  caller-managed input lifecycle (§2, §8).
- **D14** At-least-one-tick enforced here (T1 D5's wiring half): a tickless
  bundle is `[align-no-tick]`; multi-trigger stays a definition-time error
  (§7).
- **D15** No runtime payload-type validation against annotations (T1 D3
  consistency) (§3, §6.3).

## 15. Relitigation

### 15.1 Retention window: knob dropped, goal met by construction

- **Decision under review**: index §T5 "Bound the history buffers
  (retention window config); unbounded growth on a never-ticking module is
  the classic leak" — it names a mechanism (a retention knob).
- **What changed**: under the pull merge with strict monotonicity, no
  resolution can ever reach behind the per-port `newest` item (§5.1), so
  history depth is structurally 1 + the pending head. The never-ticking
  leak cannot occur: a silent trigger leaves each secondary holding exactly
  one parked item; nothing accumulates inside the aligner (backlog lives in
  the *sources*, which T5 pulls lazily and T8 bounds in its queues, where
  the index already places all drop policy). A retention parameter would
  configure a buffer that does not exist.
- **Proposal**: no `retention` parameter in v1. If a future sampler needs
  windowed history (tf's per-edge buffers in T11, or a hypothetical
  `latest_n()`), the window arrives with that sampler's state kind — per
  §11.5 — not as a global knob on `align()`.
- **Blast radius**: none landed (no caller exists); T8's queue bounds and
  T11's MultiTBuffer retention are unaffected (different buffers, already
  owned there); index §T5 wording should be amended "bounded buffers — met
  structurally; retention knobs live with windowed samplers" if accepted.

### 15.2 "Hold until history exists" refined to hold-vs-final-drop

- **Decision under review**: T1 §5.3's gloss for required `latest`
  ("required ⇒ hold tick until history exists (T5)") and the index's
  "REQUIRED fields hold the tick until resolvable".
- **What changed**: T5 distinguishes the two situations those phrases
  conflate. While a qualifying item *may still arrive* (a secondary frontier
  at or behind T), the tick is **held** — the pull waits for data, which is
  the live latency-skew case `_wait_get` used to block threads for. Once the
  frontier passes T, monotonicity makes "history will exist later" *provably
  false*, and the tick **drops, finally** (counted). A tick before a
  stream's first sample therefore drops rather than holding forever — the
  only behavior consistent with "no blocking waits" and with `latest`
  meaning *at tick ts*, and the exact analogue of `align_timestamped`'s
  unmatched warmup primaries.
- **Proposal**: adopt the §4.5 vocabulary (held = frontier-pending; dropped
  = provably unresolvable, final). T1's gloss stays correct read as the live
  case; no T1 text change required.
- **Blast radius**: T9 (drops vs holds are distinct signals — served by
  §8's counters + gauge); T6/T12 docs teach the refined vocabulary.

## 16. Open questions (non-blocking)

1. **T6 coercion default** (§11.1): unwrap memory2 `Observation` to `.data`
   at the `over()` boundary — confirm when T6 is planned (default assumed).
2. **`Stamped` home for the `Streamable` swap** (§11.2): `typing.py` imports
   from `align.py` (single definition, one-sibling relaxation of t4 §6) vs
   a structural twin — T6's one-line call.
3. **`pm` export set** (§2): proposed six names; integrator applies with the
   next `__init__.py` touch.
