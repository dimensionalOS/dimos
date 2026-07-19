# Pure modules — implementation tasks

## Status

`ready` → `planning` (spec agent at work) → `spec-ready` (spec merged, awaiting
implementation) → `implementing` → `done`. Specs land next to this index as
`tN-<slug>.md`.

| Task | Status | Spec branch |
| --- | --- | --- |
| T1 rows | done | `pure/impl-t1-rows` |
| T2 config | done | `pure/impl-t2-config` |
| T3 validation | done | `pure/impl-t3-stepspec` |
| T4 typing | done (static surface; `over()`/ports bodies land with T6/T8) | `pure/spec-t4-typing` |
| T5 alignment | implementing | `pure/impl-t5-align` |
| T6 drivers | implementing | `pure/impl-t6-drivers` |

Wave-B interface reconciliation (orchestrator, binding for the T5/T6
implementers): `Stamped` gets ONE home — `typing.py` (t6-drivers.md §11.2;
overrides t5-align.md Q2's default) — with `align.py`/`drivers.py` importing
it and `Streamable` tightening to `Iterable[Stamped]` at T6 impl.
memory2-`Observation` coercion at the `over()` boundary is a T6-impl
obligation (t5-align.md Q1 default confirmed). T5's `[align-unknown-port]`
is authoritative for wiring names; T6's eager `[run-unknown-stream]` is the
pre-align fast path, same copy standard.
| T7 resources | ready | |
| T8 rim | ready | |
| T9 health | ready | |
| T10 checkpoint | ready | |
| T11 tf | ready | |
| T12 examples | ready | |

Source of truth: `dimos/memory2/puremodule_api_sketch3.py` (the module layer).
Each task below is a dev-task boundary; a planning agent expands it into a full
design with type specs. Tasks land independently, in dependency order (graph at
the bottom).

## Decisions already made — settled defaults, not dogma

- Step signature is the single typing authority: no generics on module classes;
  engine methods recover `TIn`/`TOut` via protocol-typed `self` (verified under
  mypy --strict, incl. `-> Out | None`, async, Mealy — see sketch §TYPING).
- Bundles nest (`Tagger.In`), bases imported as `pm.In`/`pm.Out`; nesting is
  placement, not mechanism.
- Flat config fields are the API; a synthesized **frozen** pydantic BaseConfig
  is the substance (sketch §5c). Module identity = class + config.
- Effects are outputs; state vs resource doctrine; health engine-owned;
  TF is a sampler, not a service.

Don't re-derive these from scratch per task — but they are NOT beyond
challenge. If a decision doesn't survive contact with implementation, or you
see a genuinely better design, RELITIGATE IT — planners and implementers
alike. The one forbidden move is deviating silently (or complying silently
while believing it's wrong). Escalate up the chain: write a short case —
which decision, what breaks or what's better, proposed alternative, blast
radius (which tasks/sections it touches) — and bring it to the architecture
session that owns the sketch. If accepted, the sketch and this file get
amended; the sketch stays source of truth AS AMENDED, so downstream tasks
inherit the fix instead of a fork.

## Cross-cutting rules

- **Layering**: rows/specifiers/config/typing form a zero-runtime-dep data
  layer. Engine imports it; never the reverse. Rim (transports/ports) imports
  engine. Health/TF plug into the engine through narrow interfaces.
- **Python floor is 3.10** → `dataclass_transform` etc. from
  `typing_extensions` (already a dependency).
- **Package + import convention**: the package is `dimos/pure` — readable,
  greppable paths; shortness lives in the ALIAS, not the path (the numpy/np
  pattern). Canonical import, used in every doc and example:
  `from dimos import pure as pm` — full surface via `pm.*` (`pm.In`, `pm.Out`,
  `pm.PureModule`, `pm.tick`, ...; bare `from dimos.pure import tick` is fine
  for heavy-use specifiers). The sketches import this package for real now.
  The class stays `PureModule`, not `pm.Module`: it must coexist with legacy
  `Module` during migration, and an unqualified "Module" in tracebacks and
  reviews would be ambiguous.
- Every task lands mypy --strict clean, with unit tests AND static-typing
  regression tests (the static behavior is product surface here).
- **API floor**: the sketch's Tagger stays four declarations. Any task that
  adds required ceremony to it is an API regression — stop and rethink.
- Bundle field types must be **runtime-importable** (the sketches'
  TYPE_CHECKING-only msg imports are a sketch liberty, not a pattern).

## Out of scope (separate efforts; don't preclude, don't build)

PureGraph/wire (graph sketch), RPC + flows (`puremodule_api_sketch3_rpc.py`),
run inspector / behavioral diff / edge cache tooling. Preclusion guard: keep
config identity canonical (T2) and keep every edge recordable (T5/T8).

## Proposed layout

```
dimos/pure/
  __init__.py      # the pm surface
  rows.py          # T1  In/Out bases, field specifiers
  config.py        # T2  dataclass_transform + synthesized BaseConfig
  module.py        # T2/T3  PureModule, __init_subclass__ validation
  typing.py        # T4  protocols, overloads (static surface)
  align.py         # T5  sampler/alignment engine
  drivers.py       # T6  stateless/mealy/async/fold drivers, over()
  resources.py     # T7  @resource
  rim.py           # T8  ports, transports, live lifecycle
  health.py        # T9
  checkpoint.py    # T10
  tf.py            # T11
```

---

## T1 — Row bundles: `In`/`Out` bases + field specifiers

`pm.In`/`pm.Out` as `@dataclass_transform` bases (subclass applies machinery,
no decorator); specifiers `tick(expect_hz=)`, `latest(default=)`,
`interpolate()`, `contract(min_hz=)`, plus plain `= None` sparse fields.
Kw-only, engine-stamped `ts` on every row. Introspection:
`BundleCls.fields() -> {name: FieldSpec}`. Data model ONLY — no engine, no
streams.

Watch out for:
- Declare `field_specifiers=` in the `dataclass_transform` call so mypy models
  a specifier-with-no-default as REQUIRED in the constructor — this is the
  whole point (tests hand-construct rows).
- Specifier objects are class-level metadata; they must never survive as
  instance attribute values. `ts` must stay kw-only so field order never
  breaks construction.
- In and Out may share field names; bundles defined nested inside a class
  body must work identically to module-level ones (no name assumptions).
- Rows are plain serializable data — codec integration comes later, but don't
  design anything that blocks "recording ticks = serializing rows".

## T2 — Config machinery + PureModule constructor

`@dataclass_transform(kw_only_default=True)` on `PureModule`: plain annotated
class fields become typed constructor kwargs (verified: typo kwargs and wrong
types are mypy errors; nested classes/descriptors are NOT fields).
`__init_subclass__` collects the same fields into a synthesized pydantic
model (`extra="forbid"`, `frozen=True`), routes `__init__` kwargs through it,
exposes read-only `m.config` with `model_dump()` as THE canonical config
serialization. Flat attribute access stays (`self.emit_every`).
`warmup/start/stop` signatures satisfy the `Service` protocol
(`dimos/protocol/service/spec.py`) for blueprint interop.

Watch out for:
- Do NOT make PureModule a pydantic BaseModel — metaclass fights
  dataclass_transform and descriptors. Synthesize a *separate* model per
  class; rebuild it per subclass (shape subclasses inherit + extend fields).
- Frozen enforcement at BOTH surfaces (module attr set + config model).
- Mutable defaults need default_factory treatment.
- Canonical, deterministic `model_dump()` ordering — it feeds memo keys and
  checkpoint identity later.

## T3 — Step-shape classification + definition-time validation

In `__init_subclass__`: exactly one of `step`/`fold`; classify
stateless / Mealy / async-stateless / fold from the signature via
`get_type_hints(step, localns=vars(cls))` + module globalns; resolve In/Out
(nested or referenced); `State` declared iff Mealy; async requires stateless.
Import fails loudly, error names the module and the violated rule.

Watch out for:
- Bare `In`/`Out` annotations resolve only in the defining class body;
  subclass overrides use `Shape.In` — resolution must handle both, plus
  cross-module bundle references (`VoxelGridMapper.In` in another class).
- This is the compensation for losing definition-site mypy errors (priced in
  the sketch banner) — invest in the error messages.
- Validate, don't instantiate: no bundle construction, no resource touch at
  import time.

## T4 — Static typing surface + mypy regression harness

`Stateless/Mealy/AsyncStateless/Fold` protocols; `over()` overload set;
`m.i`/`m.o` port-handle descriptors with `__get__` overloads on
protocol-typed instances; `Stateless[C.In, C.Out]` exported for structural
shapes. One runtime implementation behind the overloads, dispatching on T3's
classification.

Watch out for:
- Overload ORDER: AsyncStateless before Stateless (a coroutine-returning step
  must not unify with the sync overload). Mealy/Stateless are disjoint by
  arity. All four shapes + `-> Out | None` inference are already verified —
  turn the session's scratchpad experiments into permanent CI tests
  (mypy-api or pytest-mypy-plugins; pick one and stick to it).
- Variance: TIn contravariant, TOut covariant.
- Protocol members for descriptor-provided attributes are spelled as
  read-only properties (plain variables demand settability).

## T5 — Alignment engine (streams → In rows)

check docs/usage/data_streams/temporal_alignment.md if helpful

The tick resolver: per-port history buffers; `tick()` port triggers;
`latest`/`interpolate` resolve at trigger ts; REQUIRED fields hold the tick
until resolvable (no blocking waits anywhere — this is what killed
`_wait_get`); `default=` fields go optional; engine stamps the In row with
tick ts. Pure, pull-based, thread-free component: stamped-msg iterators in,
row iterator out. Deterministic: same inputs → same rows, regardless of input
chunking. Drop/hold accounting exposed via hooks (health consumes in T9).

Watch out for:
- ONE timestamp authority: payload ts. Enforce monotonicity per port at
  ingestion; define the equal-ts total order once (port declaration order,
  then sequence) — replay identity depends on it.
- Interpolation is type-specific (float lerp vs Transform slerp): an
  `Interpolatable` protocol / registry on msg types, NOT engine special-cases.
- Policy decision to make explicit: exactly ONE `tick()` field per In bundle
  for now (multi-trigger joins are a plan error until a real use case) —
  document it, cheapest correct semantics.
- Bound the history buffers; unbounded growth on a never-ticking module is
  the classic leak. (Orchestrator amendment per t5-align.md §15: the pull
  merge makes per-port state structurally O(1) — newest + pending head — so
  there is no retention knob to configure; windowed buffers arrive with the
  samplers that need them, T11 tf. "Hold until resolvable" refines to
  held-while-frontier-pending vs dropped-once-provably-final.)

## T6 — Step drivers + `over()`

Drive classified steps over aligned rows: stateless (None return = skip),
Mealy (thread State, start from `State()`), fold (generator owns loop; engine
validates self-stamped, monotonic Out ts), async (inflight window ≤
`max_inflight`, emit in tick order, failed step = counted dropped tick, drain
or cancel on stop, `aclose` after drain). `over(**streams)` = alignment (T5)
+ driver + typed Out iterator, stamping Out rows with tick ts; full teardown
at exhaustion.

Watch out for:
- The async driver is the hardest piece and the reason fold-era code had
  bugs: the ENGINE owns the task set — no leaked tasks on error, bounded
  reorder buffer (size = max_inflight) for tick-order emission, cancellation
  must not drop already-completed rows.
- Mealy State is immutable plain data — never alias, never mutate in place.
- `over()` is re-invokable: a fresh run per call; the module instance carries
  config + resources only, zero run state.
- Teardown must also run on early exit — consumer breaks out of the for loop
  → GeneratorExit → resources still disposed, exactly once.

## T7 — Resources (`@resource`)

Descriptor: outside a run, lazy cached property (tests just touch it). Inside
a run: created at warmup in declaration order, disposed in reverse at stop —
`dispose()`/`close()`/`aclose()` sniffed in that order; `over()` teardown
uses the same path.

Watch out for:
- Per-RUN lifecycle: a resource created for a run is disposed with it; the
  lazy-test-mode instance is per-module-instance. Don't blur the two.
- Partial warmup failure unwinds already-created resources in reverse.
- Thread-safe lazy init; async factories/aclose handled on the module loop.
- Ownership rule: dispose only what the factory created (never injected
  objects).
- Leave a cheap "handler context" flag hook (RPC's no-resources-in-handlers
  guard lands later, sketch3_rpc §3 — just don't make it impossible).

## T8 — Live rim: ports, transports, lifecycle

Runtime `m.i.x`/`m.o.y` handles: `.transport =`, `.source =`,
`.subscribe()`. Live loop: transport delivery → marshal onto the module's
single loop → T5 buffers → tick → T6 driver → out ports (publish + local
subscribers). `warmup()/start()/stop()` with reverse-order drain;
single-input `transform()` equivalence and `save()/drain()` interop with
`dimos.memory2.stream` (ports speak stamped msgs — same currency).

Watch out for:
- Transports deliver on arbitrary threads and never drop by policy; ALL
  backpressure/drop decisions live in the engine buffers. One loop per
  module; marshal, don't lock.
- `over()` must be structurally unable to touch transports (replay can't
  rebroadcast — shared drivers, separated rim).
- `source` and `transport` are mutually exclusive per port; `stop()` is
  idempotent, closes rims reverse-topologically, drains before disposing.
- Reuse `StreamAccessor`/`Stream` conventions rather than inventing a
  parallel stream API (store-as-owner, caller-managed lifecycle — house
  style).

## T9 — Health

ONE global stamped `Health` row (path, ticks, drops, out_hz, contract_ok,
step_ms_p95, inflight; sparse violation/resource_error). Wall-clock pacer
(`health_hz`, default 1 Hz, per-member override) fed by engine counters from
T5/T6/T8. Exposed as a rim stream `m.health` (dogfoods T8 port machinery).

Watch out for:
- Wall clock is legal HERE ONLY. Keep it out of the data path — pacer thread
  reads monotonic counters, no locks on the tick path.
- A stalled module still healths (that's the point) — the pacer must not
  depend on ticks happening.
- Off by default under `over()`: offline cadence is a post-hoc query over
  recorded ts, not fake wall-clock rows.

## T10 — Checkpoint / restore

`m.checkpoint()` → plain-data snapshot `{schema_version, class qualname,
config dump (T2), State, tick cursor}`; `restore(snap)` validates class +
config match (explicit override flag for migrations). Checkpoints only at
tick boundaries.

Watch out for:
- Include `schema_version` NOW — State migration arrives with hot reload
  later; retrofitting versioning into shipped snapshots is misery.
- fold/async modules have no checkpointable mid-run state: `checkpoint()`
  raises with a message quoting the doctrine, not a silent partial snapshot.
- Cursor semantics must be defined by T5's ordering (never mid-tick).

## T11 — TF: `tf()` sampler + `tf_out()` port

Consume: `tf(parent, child)` In field = interpolate-with-chain-composition;
required holds the tick, `default=None` optional. Publish: `tf_out(parent,
child)` sparse Transform port declaring its asserted edge; engine routes
emissions to (a) the engine-owned shared buffer — sibling samplers see it
in-process, (b) the recorder path, (c) the tf topic only where bound at the
rim. Frame names accept config templates (`"{prefix}/odom"`), resolved at
build; empty segments drop. `port.frames` introspection; single-writer check
at module/engine scope. Statics seeded at warmup from config.

Watch out for:
- ADAPT `dimos/protocol/tf/tf.py` (`TBuffer`/`MultiTBuffer` math: graph
  search, interpolation, retention) as the implementation — do not fork the
  math, do not keep the service API, and delete the blocking `_wait_get`
  pattern (hold-the-tick replaces it).
- The shared buffer belongs to the RUN/engine context, passed in — never a
  process global (tests, multi-engine processes).
- Template resolution errors at build, naming module + field + template.
- `tf_view()` snapshot escape hatch is OPTIONAL — defer unless trivial.

## T12 — Examples, docs, and the UX floor (continuous)

Port the sketch modules (Tagger, QualityGate, VoxelGridMapper, CostMapper
shape + impls, Captioner, ScanBatcher, Go2Connection) as executable examples
+ tests as their tasks land. The scratch mypy experiments from the design
sessions become the T4 regression suite. Docs page under `docs/usage/`
(follow `docs/development/writing_docs.md`; executable via md-babel).

Watch out for:
- Tests-need-no-engine is a PROPERTY to enforce: every example module gets a
  row-construct + step-call unit test with zero engine imports.
- Alignment determinism deserves a property test (same inputs, shuffled
  chunking → identical rows).
- The Tagger-floor guard: the canonical four-declaration Tagger compiles and
  runs in CI; if a task forces it to grow, the task is wrong, not the Tagger.

---

## Dependency graph

```
T1 rows ─→ T2 config ─→ T3 validation ─→ T4 typing surface
                             │
              T5 alignment ──┼── T7 resources
                     │       │       │
                     └── T6 drivers/over() ──┬── T8 rim ── T9 health
                                             ├── T10 checkpoint
                                             └── T11 tf   (also needs T5)
T12 runs alongside everything from T3 on.
```

Suggested landing order: T1 → T2 → T3 → T4 → T5 → T6 → T7 → T8 → {T9, T10,
T11} → T12 closes out. T4 can start once T1 exists; T7 once T3 exists.
