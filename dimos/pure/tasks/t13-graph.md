# T13 — PureGraph: the composition layer

Status: **spec-ready** (this document). Charter: `dimos/pure/puregraph_api_sketch.py`
— its banner (§1–§8 + DEPLOYMENT) is the settled model; this spec turns it into an
implementable design. Skeleton: `dimos/pure/graph.py` (+ `PureModule.__call__`
overloads and `PureModule.blueprint()` in `module.py`). Tests:
`dimos/pure/test_graph.py` (skip-gated where impl-pending) and the static fixture
`test_typing_fixtures/case_graph_apply.py` (live).

Phases: **A** core + local `over()` (independently landable), **B** blueprint
lowering + the GO2 grounding test, **C** deferred runtime (feedback closing,
interior injection, native distributed, flight recorder). §9 has the split and
per-phase acceptance tests.

---

## 0. Amendments — post-review (Ivan, 2026-07-20, BINDING)

These resolve the ratification items and re-spec the runtime after review. Where
an amendment supersedes original prose, **the amendment wins** and the superseded
text is kept only for context. The Phase-A implementer builds to §0.

### 0.1 Ratifications (accepted)

- **T1 `BareSpec` amendment (§2.4): ACCEPTED.** Phase A implements it — bare In
  fields become definable so graph rims can spell `class In(pm.In)`; module steps
  still reject unsampled In fields via the relocated `[in-field-unsampled]` check.
- **Error subclassing: ACCEPTED.** `PureGraphDefinitionError`/`PureGraphRunError`
  subclass the module error types.
- **`PureModule.__call__` unified operator + `PureModule.blueprint()`: RATIFIED**
  (landed with this spec, live-tested).
- **Q4 (GraphRun outputs are raw payloads): CONFIRMED.** `run.path.to_list()`
  yields `Path` objects, not Out-row envelopes.
- Q1/Q2/Q3/Q5/Q6/Q7/Q9 defaults stand unchanged.

### 0.2 Runtime is bounded streaming — SUPERSEDES §6.2 (edge logs)

There are **no edge logs**. The runtime never retains an append-only per-edge
history. Fan-out is lossless *without* buffering by construction:

- **Offline (`over()`): synchronous topological distribution.** Each edge value
  is delivered to ALL of that edge's consumers *before* the producer advances, so
  nothing is ever held for a lagging branch — one value in flight per edge, then
  gone. This is lossless and bounded with zero fan-out buffer. The only offline
  buffering is each member's own aligner hold, which T5 bounds to O(1) per port.
  No capacity ring is used offline (no threads, no real-time deadline —
  backpressure is just the pull semantics; a slow consumer slows the whole run,
  losslessly).
- **Live (rim/transports): bounded capacity rings (T8 model), drop-old.** Because
  transports deliver asynchronously on their own threads, live fan-out uses the
  landed per-port capacity/KeepLast rings — never unbounded. A slow live consumer
  drops old; lossless-live-to-slow-sink is not offered (physics: drop or fall
  behind real time).
- **Independent divergent-rate output pulling is not a supported mode.** To read
  two outputs at genuinely different rates, re-run (over() is deterministic and
  re-invokable — a fresh run per call) or spill one to a store (§0.3). Never
  buffer one output while draining another.

### 0.3 Recording & query go through mem2 stores — SUPERSEDES §6.2/§6.3 interior taps + removes Q8

Post-hoc inspection/query/replay is **not** a bespoke graph API. To keep edges,
`.save()` them into a mem2 store and use mem2's existing query/replay:

- **Per-edge = one `Observation` stream.** Payload is the edge's `.data`, ts is
  `.ts`: `store.stream(path, PayloadType).append(msg, ts=msg.ts)`. Symmetric with
  the ingestion path, which already unwraps `Observation → .data` (T6 boundary).
- **A graph recording = one mem2 store holding N named typed streams**, keyed by
  edge path (namespaced member path). Any backend — `MemoryStore` (RAM),
  `SqliteStore` (disk), future — chosen by the caller. That store *is* the
  replay/query system.
- **No new "bundle" message type.** mem2 stores are already multi-stream. A
  module's multi-field In/Out row is NOT stored atomically; it is reconstructed on
  replay by `over()`'s deterministic alignment from the per-field streams (the
  shared `ts` carries the bundle relationship). This is exactly the
  `_record_then_debug_one_member` pattern.
- **`GraphRun` exposes only the exported outputs** as bounded streams. Interior
  edges stay addressable by path ONLY to *attach* a store or transport (record /
  cross-process cut), never for in-memory readback. **Removed:** the
  `GraphRun.stream("interior.path")` tap and the `graph-unknown-path` run error.
- **Q8 (edge-log retention) is void** — there is nothing to retain, so nothing to
  truncate.

### 0.4 `cmd_vel` seam (Q10 / §7.4): RESOLVED — translator module, not covariant matching

Do **not** make the legacy autoconnect matcher covariant. A small
`TwistStamped → Twist` translator pure module bridges the command edge; GO2Connection
changes to accept `TwistStamped` later. Phase B item.

---

## 1. Scope and the settled model

A `PureGraph` composes configured pure modules into a validated DAG by running
`wire(self, i: In) -> Out` **once, symbolically, at build**. The decisions below
are inherited from the charter and are *not* revisited here:

| # | Decision |
| - | -------- |
| 1 | Application is the one operator: `m(x=...)` on ports returns the member's Out as typed port refs; `over()` is the same application on real streams. |
| 2 | `wire()` is pure and rerunnable; construction is config-only; building does nothing. Parametric DAGs are plain Python for-loops. |
| 3 | A graph is wiring, not computation: no tick of its own; samplers live on member In ports; graph bundles carry bare types; nesting flattens; members namespace by path. |
| 4 | Edges are streams of `.data`-only stamped msgs — the transport currency; interior edges are in-process handoffs; fan-out tees; any edge can gain a transport/recording without touching `wire()`. |
| 5 | A source is a polymorphic value: upstream member, live transport, or recorded stream, in the same slot — rim or interior. |
| 6 | Cycles only through a sampled (latest/interpolate/tf) back edge, named by `feedback()` before its producer, closed exactly once; tick back edge = plan error. |
| 7 | One aligner: a member's In row is resolved from inbound edges by the T5 aligner `over()` already uses. |
| 8 | Wiring is validated at build (§5), before anything runs. |
| D | One `wire()`, three targets: local `over()`; `graph.blueprint()` lowering to the existing coordinator (autoconnect *convention* kept, autoconnect *function* dropped); future native `partition().bind()`. |

Layering: `dimos/pure/graph.py` is an engine-layer module (same stratum as
`rim.py`). At module scope it imports the data layer (`rows`, `stepspec`,
`typing`, `config`) plus `drivers` (for the run-error base); the tf buffer,
memory2, and `dimos.core` are imported lazily inside the functions that need
them (`over()`, `blueprint()`), exactly like `EngineSurface.over()`'s
sanctioned lazy edges. `blueprint()` is the graph's one
`dimos.core` edge and rides the already-sanctioned T8 bridge (`legacy.py`).
Nothing in `dimos/pure/__init__.py` changes: graph names are imported from
`dimos.pure.graph` (the charter sketch's own spelling) — see Q7.

---

## 2. The graph object model

### 2.1 `PureGraph` and `classify_graph`

`PureGraph` is its own base — **not** a `PureModule` subclass:
`PureModule.__init_subclass__` ends in `classify(cls)`, which demands a
`step`/`fold`; a graph has neither, by doctrine (§1 row 3). It reuses the T2
config machinery by importing the same helpers `module.py` uses
(`_collect_config_fields`, `_synthesize_config_model` — intra-package, same
license as `module.py`'s use):

```python
@dataclass_transform(kw_only_default=True, frozen_default=True)
class PureGraph:
    __pure_config_model__: ClassVar[type[PureModuleConfig]]
    __pure_graph__: ClassVar[GraphSpec]     # stamped LAST — certifies all gates

    def __init_subclass__(cls, **kwargs):
        # 1. super(); 2. config fields + synthesized frozen model (T2, §2.2);
        # 3. cls.__pure_graph__ = classify_graph(cls)   — LAST statement
```

`__init__`, `__setattr__`/`__delattr__` (FrozenModuleError), `config`,
`__eq__`/`__hash__` (class + config), `__repr__`, `__reduce__` mirror
`PureModule` verbatim (same semantics, same error copy — a graph is a value the
same way a module is). Implementation may copy the ~60 lines or extract a
shared mixin; extraction must not change `module.py` behavior (454 tests).

`classify_graph(cls) -> GraphSpec` is the T3 twin — pure, structural, raises
`PureGraphDefinitionError` at the class statement. Gate order:

- **G0 discovery** — `wire` found over the MRO's own dicts
  (`[graph-wire-missing]` if absent). A `step` or `fold` in the class body is
  `[graph-step]` (a graph is wiring, not computation).
- **G1 function kind** — plain instance function; not static/classmethod, not
  async, not a generator (`[graph-wire-shape]`).
- **G2 parameter shape** — exactly `(self, i)`, plain positional, no defaults
  (`[graph-wire-shape]`).
- **G3 annotation resolution** — `get_type_hints` with owner-body localns
  (reuse `stepspec._resolve_step_hints` with `name="wire"`); unresolvable →
  `[graph-wire-unresolvable]` (same lesson text as T3), missing →
  `[graph-wire-unannotated]`.
- **G4 row types** — `i` must annotate a `pm.In` subclass
  (`[graph-in-not-row]`); the return must be exactly a `pm.Out` subclass — no
  unions, **no `| None`**: a graph always wires every declared output
  (`[graph-out-not-row]`).
- **G5 bundle policy (§2.3)** — every In field bare (`[graph-port-sampler]`,
  `[graph-port-default]`); every Out field plain (`[graph-port-sampler]` for
  `contract()`/`tf_out()`).

```python
@dataclass(frozen=True)
class GraphSpec:
    in_type: type[Any]
    out_type: type[Any]
    owner: type[Any]      # the class whose body defines wire
```

**Invariant (mirrors EngineSurface):** `PureGraph` itself must never declare
`wire` — a base-level `wire` would make every subclass match the `Wires`
protocol (§3.4) with the base's types and destroy per-graph inference.

### 2.2 Config

Same machinery, same semantics as T2: flat annotated fields → synthesized
frozen pydantic model; mirrored inheritance across `PureGraph` bases; kwargs
validated in `__init__`; `FrozenModuleError` on mutation; identity = class +
config. Graph config flows into member config the only way it can: `wire()`
reads `self.voxel_size` and passes it to `VoxelMapper(voxel_size=...)` — plain
Python, no threading machinery. Reserved names on graphs additionally include
`wire`, `blueprint`, `over`, `build`, `bind`, `partition` (a config field of
those names shadows the surface; `[config-*]` catalog from T2 applies
unchanged, extended with these names).

### 2.3 Graph bundles: `pm.In` / `pm.Out`, bare fields

Graph rims reuse `pm.In`/`pm.Out` — the charter's own spelling
(`class In(pm.In): lidar: PointCloud2`). `ts` rides along inert: graph bundles
are never engine-stamped rows in a dataflow; symbolically a rim row is
constructed with `ts=UNSTAMPED` (§3.5), and no runtime path constructs a graph
bundle with data. Rules:

- In fields: **bare annotation, no specifier, no default**. A sampler is
  `[graph-port-sampler]`; a default is `[graph-port-default]` (optionality
  lives on the member sampler: `latest(default=...)`).
- Out fields: **bare annotation only** — `contract()`/`tf_out()` are
  `[graph-port-sampler]`; an exported output inherits its producing member's
  contract (checked/enforced at the member, where it lives).

### 2.4 Required T1 amendment: bare In fields (`BareSpec`)

Today `rows._process` step 11 rejects a specifier-less In field at bundle
definition (“In field needs a sampler specifier”) — which makes the charter's
own graph-In spelling unable to *define*. Amendment (additive, flagged for
orchestrator ratification):

1. `rows.py`: new spec kind
   ```python
   @dataclasses.dataclass(frozen=True)
   class BareSpec(FieldSpec):
       """In field with no sampler — legal only on graph rims (T13)."""
       kind = "bare"
       side = "in"
   ```
   Step 11's `side == "in"` branch becomes `own_specs[fname] = BareSpec(default=f.default)`
   instead of raising.
2. `stepspec.py`: a new G5 sub-gate — a **module** step's In bundle containing
   `BareSpec` fields fails at the *module* class statement with the exact old
   copy, relocated:
   `"{cls}: In field {name!r} needs a sampler specifier (tick()/latest()/interpolate()); plain defaults are only valid on Out bundles [in-field-unsampled]"`
   (new `Rule.IN_FIELD_UNSAMPLED`). Loudness timing is preserved for the case
   that matters — a bundle is declared to be used, and module use is same-import.
3. Backstops unchanged: T5's `[align-unsupported-kind]` still rejects a
   `BareSpec` port handed to the aligner directly; the rim inherits via T5.
4. Blast radius: `test_rows.py` pins of the old bundle-time error move to
   `test_stepspec.py`-style module-time pins (implementer updates in the same
   commit); `BareSpec` joins `rows.__all__` and the `pm` surface? — **no**:
   graphs never spell it, it is introspection-only; export from `rows` but keep
   off `pm.__all__` (so `test_pm_surface.py` is untouched). Documented in
   `fields()` consumers.

---

## 3. Symbolic application and its typing (the hard problem)

### 3.1 Two planes, two checkers

- The **data plane** — payload types flowing field-to-field — is checked by
  mypy, through the *bundle annotations*, with zero per-call machinery: inside
  `wire()`, `i.lidar` is statically `PointCloud2` (a real attribute of the In
  bundle) and `plan.path` is statically `Path` (a real attribute of
  `Planner.Out`), because application *returns the member's Out type*.
- The **wiring plane** — which port feeds which field — is checked at build by
  §5's validator (“mypy for wiring”), exactly as `over()`'s `**streams` are
  runtime-checked today (`[run-unknown-stream]`): T4 already establishes that
  per-kwarg payload typing through `**kwargs` is out of static reach; the graph
  inherits that split rather than fighting it.

The trick that makes both planes work with **no `Any` on the user's page**:
symbolic values are `Port` objects *at runtime* that travel under *payload
static types* — they are smuggled through the very annotations the bundles
already declare. mypy never sees a `Port` except where the user explicitly
declares one (`feedback()`, §3.6).

### 3.2 `Port[T]`, `PortRef`

```python
@dataclass(frozen=True)
class PortRef:
    member: str | None      # member path ("voxel_mapper", "nav.planner"); None = graph rim
    field: str              # port (field) name on that member / rim

class Port(Generic[_TMsg]):
    """Symbolic reference to one producer port; the build-time stand-in for its payload."""
    ref: PortRef
    annotation: Any               # the producer field's payload type (Optional stripped)
    def close(self, producer: _TMsg) -> None: ...   # feedback ports only (§4.3)
```

`Port` is a plain runtime class (not type-erased): it appears statically only
in `feedback()` declarations, where its `close()` needs to typecheck —
`prev: Port[TwistStamped] = feedback()` then `prev.close(plan.cmd_vel)` checks
`plan.cmd_vel: TwistStamped` against `_TMsg`. `feedback() -> Port[Any]`; the
user's annotation narrows it (charter spelling preserved verbatim).

Each `Port` also carries a private token of the build that minted it, so a port
smuggled across `wire()` runs is `[graph-foreign-port]` — this is what makes
“`wire()` is pure/rerunnable” *checkable*, not aspirational.

### 3.3 `PureModule.__call__`: the application overloads (module.py edit)

`__call__` is today the transformer seam (`m(rows)` → `rim.transformer`), typed
`(self, rows: Any) -> Any`. It becomes the **one operator** — keyword
application, positional transformer — typed in the T4 house style (overloads on
protocol-typed `self`, async-first ordering):

```python
@overload
def __call__(self: AsyncStateless[_TIn, _TOut], **ports: Any) -> _TOut: ...
@overload
def __call__(self: Mealy[_TState, _TIn, _TOut], **ports: Any) -> _TOut: ...
@overload
def __call__(self: Stateless[_TIn, _TOut], **ports: Any) -> _TOut: ...
@overload
def __call__(self: Fold[_TIn, _TOut], **ports: Any) -> _TOut: ...
@overload
def __call__(self, rows: Iterable[Any], /) -> Iterator[Any]: ...
def __call__(self, rows: Any = None, /, **ports: Any) -> Any:
    if ports and rows is not None: raise TypeError(...)          # mixed call
    if rows is not None: return rim.transformer(self)(rows)      # unchanged seam (S5)
    from dimos.pure.graph import apply_symbolic                  # lazy engine edge
    return apply_symbolic(self, ports)
```

Why this typechecks per-module with zero annotations on the module: identical
mechanism to `over()` — mypy unifies the solver twins `_TIn/_TOut` against the
concrete step signature via the protocol-typed `self`, so
`VoxelMapper()(scan=i.lidar)` is statically `VoxelMapper.Out` and
`.global_map` is `PointCloud2`. Keyword calls cannot match the positional-only
transformer overload; positional calls cannot match the kwargs-only application
overloads — the two faces never collide. A zero-argument call `m()` is a legal
application of an all-optional In (validated at build like any other).

The transformer overload's return narrows from `Any` to `Iterator[Any]` —
strictly more informative; no landed caller pins `Any`.

### 3.4 `Wires` and the graph's own application

```python
class Wires(Protocol[TIn, TOut]):
    def wire(self, i: TIn, /) -> TOut: ...
```

(`TIn` contravariant, `TOut` covariant — declaration pair per T4; solver twins
for overload positions.) A user's `def wire(self, i: In) -> Out` satisfies the
positional-only protocol parameter. On `PureGraph`:

```python
def __call__(self: Wires[_TIn, _TOut], **ports: Any) -> _TOut: ...
def over(self: Wires[_TIn, _TOut], source: Any | None = None, *,
         remap: Mapping[str, str] | None = None,
         at: Mapping[str, Iterable[Any]] | None = None,
         **streams: Streamable) -> GraphRun[_TOut]: ...
def build(self: Wires[_TIn, _TOut]) -> GraphPlan: ...
```

so a graph applies exactly like a module (nesting, §1 row 3) and `over()`
returns a typed run handle. Graphs need no transformer overload (nothing
single-input about a graph rim); `__call__` is application-only.

### 3.5 What actually happens at build

`wire()` receives a **genuine In row instance** whose fields hold `Port`
objects — frozen dataclasses store what they're given; no masquerade class:

```python
in_row = cast(Any, in_type)(ts=UNSTAMPED, **{f: Port(PortRef(None, f), spec.annotation)
                                             for f, spec in in_type.fields().items()})
out = graph.wire(in_row)             # user code; applications record into the build ctx
```

Symbolic application (`apply_symbolic`) likewise returns a genuine member-Out
row carrying `Port`s: `out_type(**{f: Port(PortRef(path, f), ann) for ...})`
(every field passed; `ts` defaults `UNSTAMPED`). The user's
`NavStack.Out(path=plan.path, ...)` is then ordinary — statically payload-typed
✓, runtime a row of ports the builder reads back. The `cast(Any, ...)`
constructions live inside `graph.py` only; user code needs no casts, no
ignores.

### 3.6 Static regression

`test_typing_fixtures/case_graph_apply.py` (live from this branch, auto-collected
by `test_typing_static.py`): toy module + graph; reveals pinned:

- `m(x=i.a)` reveals the member's `Out` type (per step kind, incl. Mealy/async);
- `i.a` inside `wire` reveals the payload type;
- `feedback()` assigned to `Port[X]` accepts `close(x_payload)` and rejects
  `close(wrong_payload)` (`E[arg-type]`);
- `m(iter_of_rows)` (positional) reveals `Iterator[Any]` — the transformer face
  survives the overload split.

---

## 4. The build

### 4.1 Build context and member naming

`build()` (and every entry that needs the DAG: `over()`, `blueprint()`) pushes
a `_BuildCtx` on a thread-local stack (a stack, because nested graph
application recurses), constructs the symbolic In, calls `wire()`, validates,
pops, and freezes a `GraphPlan`. Application outside any active build is
`[graph-apply-context]`.

Member naming: each application registers a node under
`snake_case(type(m).__name__)` (`VoxelMapper` → `voxel_mapper`); the second and
later applications of the same auto-name get `_2`, `_3`, … in application
order (deterministic — application order *is* program order). Explicit naming:

```python
voxel = named("front", VoxelMapper())(scan=i.lidar)   # graph.py helper
```

`named(name, m)` tags the instance for its next application in the current
build; a reused explicit name is `[graph-duplicate-name]`. (Q1.)

The same module *instance* applied twice is two members (a node is an
application, not an object); config is shared by value, which is exactly what
module identity means (class + config).

### 4.2 Application validation (at each `m(**ports)`)

In order, all against the member's `StepSpec` (`type(m).__pure_step__`):

1. reserved kwarg `tf` handled per §4.4; remaining kwarg names must be In
   fields → `[graph-unknown-kwarg]`;
2. every value must be a `Port` of *this* build → `[graph-not-a-port]`,
   `[graph-foreign-port]`;
3. every required In field (no default) must be bound → `[graph-unbound-input]`
   (the tick field is always required — `tick()` takes no default by T1);
4. payload agreement → `[graph-type-mismatch]`: `issubclass(src, want)` after
   Optional-stripping both sides; unresolvable/`Any` annotations pass
   (permissive on undeterminable, house rule);
5. a feedback port bound into a `tick()` field → `[graph-tick-cycle]` (§4.3);
6. record `Binding(src=port.ref, dst=PortRef(path, field))` per kwarg; mint and
   return the Out-row-of-ports (§3.5).

**Applying a graph** (a `PureGraph` value): recursively build the sub-graph's
own plan, then splice — members re-keyed `f"{sub_path}.{member}"`, sub-rim
input refs substituted with the caller's bound ports, sub-exports becoming the
ports the application returns. Flattening at build, namespacing by path
(charter §3). The sub-build inherits the caller's build token (ports cross the
splice legally).

### 4.3 `feedback()`

`feedback()` mints a `Port[Any]` flagged as a back edge, registered with the
current build. `close(producer)` runs once with a producer `Port` of the same
build; re-close is `[graph-feedback-reclosed]`, `close` on an ordinary port is
`[graph-close-not-feedback]`, a non-port argument is `[graph-not-a-port]`. At
build close-out, any unclosed feedback is `[graph-unclosed-feedback]`. Closing
resolves every binding that consumed the feedback port to the producer's ref.

**Cycle analysis is structural, not graph-search:** ordinary application can
only reference ports that already exist (Python data flow), so a cycle without
`feedback()` is unconstructible. The charter's tick-cycle rule therefore
reduces to a *local* check: every field a feedback port binds into must be a
sampled non-trigger kind (latest/interpolate) — a tick binding is
`[graph-tick-cycle]` at the application site. No DFS, no second chance to be
wrong. (Runtime loop closing is Phase C; the *validation* is Phase A.)

### 4.4 The member tf side channel

`Planner`'s `position` is a `tf()` field — resolved from the tf side channel,
never a kwarg. Application accepts the reserved kwarg `tf=` (a `Port` whose
payloads are transforms, e.g. `odom_tf.tf`) feeding the member's tf stream —
the symbolic twin of `over(tf=...)` / `m.i.tf`. Checks (build-time mirrors of
T11's runtime copy): `tf=` on a member with no `tf()` fields →
`[graph-tf-unexpected]`; required `tf()` fields with no `tf=` →
`[graph-tf-missing]`. (`"tf"` cannot collide with a field: T1 reserves the In
field name.) The type-mismatch check applies (payload must be
Transform-carrying per T11's ingestion contract; validated as in step 4 §4.2).

### 4.5 Rim close-out and the build product

After `wire()` returns:

- return value must be an instance of the declared Out → `[graph-wire-return]`;
- every Out field's value must be a member-output `Port` of this build →
  `[graph-unset-output]`; a rim-input passthrough is `[graph-export-input]`;
  two Out fields exporting the same producer port is `[graph-export-alias]`
  (an edge has one topic when lowered; parity across targets — Q3);
- every rim In port must be consumed by at least one binding →
  `[graph-unused-input]` (strict by default — Q2);
- unclosed feedbacks (§4.3).

```python
@dataclass(frozen=True)
class GraphPlan:
    graph: type[Any]                      # the PureGraph subclass
    members: tuple[tuple[str, Any], ...]  # (path, configured PureModule), application order
    bindings: tuple[Binding, ...]         # src → dst, one per bound In field
    tf_bindings: tuple[Binding, ...]      # src → (member, "tf") side-channel feeds
    exports: tuple[tuple[str, PortRef], ...]   # rim Out name → producer port
    inputs: tuple[tuple[str, Any], ...]        # rim In name → payload annotation
    feedbacks: tuple[Binding, ...]        # producer → consumer field (closed back edges)
```

Everything is a value; `PureModule.__eq__` is class + config, so **plan
equality is graph equality** — `NavStack(voxel_size=0.1).build() ==
NavStack(voxel_size=0.1).build()` pins purity/rerunnability, and plan diffing
is `==`/field-diff. Members are in application order, which is a topological
order by construction (§4.3's structural argument).

---

## 5. Build-validation error catalog

Two exception types, mirroring the T3/T8 pattern (ratify: new types vs reuse —
the reasoning: `except PureModuleDefinitionError` / `except PureModuleRunError`
keep catching everything, while `graph_rule` stays machine-readable):

```python
class PureGraphDefinitionError(PureModuleDefinitionError):   # definition + build
    graph_rule: GraphRule | None
class PureGraphRunError(PureModuleRunError):                 # over()/run surface
    graph_rule: GraphRunRule | None
```

Release copy (verbatim; `{g}` = graph class path, `{cls}` = applied member
class path; format `f"{g}: {msg} [{slug}]"` unless shown otherwise). The
`_e_*` helpers in `graph.py` are the single source; tests pin them verbatim.

| Slug | When | Message body |
| ---- | ---- | ------------ |
| `graph-wire-missing` | class stmt | `defines no wire. A pure graph declares exactly def wire(self, i: In) -> Out — members applied to typed ports, run once at build. Per-tick computation belongs in a member PureModule.` |
| `graph-step` | class stmt | `defines {name} — a graph is wiring, not computation; it has no tick of its own. Move per-tick code into a member PureModule and apply it in wire().` |
| `graph-wire-shape` | class stmt | variants: `wire must be a plain instance method — def wire(self, i: In) -> Out — not a {what}.` · `wire cannot be async — it runs once, symbolically, at build; runtime concurrency is a member affair.` · `wire takes {n} data parameters — expected exactly def wire(self, i: In) -> Out.` · `wire parameter '{p}' is {label}; wire's parameters must be plain positional: (self, i).` · `wire parameter '{p}' has a default. The build always passes the symbolic In row — remove it.` |
| `graph-wire-unannotated` | class stmt | `wire parameter '{p}' has no annotation. The wire signature is the graph's whole contract — annotate it with the In row.` / `wire has no return annotation. The wire signature is the graph's whole contract — annotate the return: -> Out.` |
| `graph-wire-unresolvable` | class stmt | T3 `step-unresolvable` copy with `wire` substituted (reuse `_resolve_step_hints`). |
| `graph-in-not-row` | class stmt | `wire input '{p}' is {t}, not a pm.In row. Declare class In(pm.In) with one bare-typed field per rim input.` |
| `graph-out-not-row` | class stmt | `wire returns {t}, not a pm.Out row — a graph always wires every declared output (no unions, no \| None). Declare class Out(pm.Out) and construct it in wire().` |
| `graph-port-sampler` | class stmt | In: `In field {f!r} carries {kind}() — graph ports are bare types; samplers live on member In fields, the graph only wires.` · Out: `Out field {f!r} carries {kind}() — graph ports are bare types; an exported output inherits its producing member's contract.` |
| `graph-port-default` | class stmt | `In field {f!r} has a default — graph rim inputs are bare, required ports; optionality lives on the consuming member's sampler (latest(default=...)).` |
| `graph-apply-context` | apply | (`{cls}`-prefixed) `applied to ports outside wire() — symbolic application only means something during a graph build. To run on real streams use over().` |
| `graph-unknown-kwarg` | apply | `wire applies {cls} with unknown port(s) {names}; In fields are: {declared}. Application kwargs must match the member's In field names (tf= is the reserved tf side channel).` |
| `graph-not-a-port` | apply/close | `wire binds {cls}.{f} to {t} — application binds ports, not values; pass a graph input (i.{f}), a member output, or a feedback(). Constants ride module config.` |
| `graph-unbound-input` | apply | `wire applies {cls} without required port(s) {names} — bind each, or give the member field a default to make it optional.` |
| `graph-type-mismatch` | apply | `wire binds {cls}.{f} (a {want} port) from {src}, which carries {got} — edge and field payload types must agree.` |
| `graph-tick-cycle` | apply | `feedback closes into {cls}.{f}, a tick() input — a back edge through the trigger would define time by itself. Cycles are legal only through a sampled input (latest/interpolate/tf).` |
| `graph-tf-unexpected` | apply | `wire applies {cls} with tf= but its In bundle declares no tf() fields — the tf stream feeds tf() samplers only.` |
| `graph-tf-missing` | apply | `wire applies {cls}, whose In declares required tf() field(s) {names}, without tf= — bind a transform-carrying port, or give the fields default= to make them optional.` |
| `graph-unclosed-feedback` | close-out | `{n} feedback port(s) never closed — every feedback() names a back edge that close(...) must tie to a producer port exactly once.` |
| `graph-feedback-reclosed` | close | (no `{g}`) `feedback port closed twice — every feedback closes exactly once; rebuild the graph for a different wiring.` |
| `graph-close-not-feedback` | close | (no `{g}`) `close() on a non-feedback port — only feedback() ports close; ordinary edges are tied by application kwargs.` |
| `graph-foreign-port` | apply/close | `wire received a port minted by a different build — ports do not survive across wire() runs; use only this build's i.* and application results.` |
| `graph-duplicate-name` | apply | `member name {name!r} used twice — named() names must be unique within a build.` |
| `graph-wire-return` | close-out | `wire returned {t} — construct and return {out} (the declared Out bundle).` |
| `graph-unset-output` | close-out | `Out field {f!r} is not a wired port (got {t}) — set every graph output from a member application's Out.` |
| `graph-export-input` | close-out | `Out field {f!r} exports rim input {inp!r} unchanged — a graph output is a member's output; passthrough re-export is not wiring.` |
| `graph-export-alias` | close-out | `Out fields {a!r} and {b!r} both export the same producer port — one export name per port (an edge has one topic when lowered).` |
| `graph-unused-input` | close-out | `rim input(s) {names} are consumed by no member — delete them, or wire them (an unused port is a wiring bug, the graph twin of an unread variable).` |

Run-surface (`PureGraphRunError`, `f"{g}.over(): {msg} [{slug}]"` style):

| Slug | Message body |
| ---- | ------------ |
| `graph-unknown-stream` | `got unknown stream(s) {names}; rim In ports are: {declared}. Stream kwargs must match the graph's In field names.` |
| `graph-unbound-rim` | `rim input(s) {names} have no source — pass a store carrying them by name, a named stream kwarg, or remap= a store channel onto them.` |
| `graph-store-missing` | `the store has no stream named {chan!r} for rim input {f!r} — remap= it to the store's channel name, or pass {f}= explicitly.` |
| `graph-unknown-path` | (on `run.stream`) `no edge {path!r} in this run — edges are addressed '<member_path>.<field>'; available: {available}.` |
| `graph-impl-pending` | Phase-gate copy for C surfaces (`at=`, `bind`, `partition`): `{what} is not implemented yet (T13 Phase C) — see tasks/t13-graph.md §8.` |

---

## 6. The local runtime: `graph.over()` (Phase A)

### 6.1 Source resolution

`over(source=None, *, remap=None, at=None, **streams)`:

1. `**streams` — explicit rim streams by In field name; unknown names →
   `[graph-unknown-stream]`. Values are anything `over()` accepts today
   (`Streamable`; mem2 `Stream` unwrapping happens inside each member's
   `run_over` — the graph adds *no* second coercion path).
2. `source` — an object exposing named streams (a mem2 store: resolution via
   `source.streams.<name>` / `source.stream(name)` accessor, structural, no
   mem2 import at module scope). Each rim In field not already given a stream
   kwarg resolves `remap.get(name, name)` from the store; a missing channel →
   `[graph-store-missing]`.
3. Any rim input still unsourced → `[graph-unbound-rim]`.
4. `at=` — interior injection (`{"planner.costmap": stream}`) severs the named
   edge's producer and substitutes the stream. **Phase C** (raises
   `[graph-impl-pending]` until then); spec'd in §8.

### 6.2 The pull engine: edge logs

> **SUPERSEDED by §0.2/§0.3.** No edge logs; the runtime is bounded streaming
> (offline: synchronous distribution, lossless, zero fan-out buffer; live: T8
> capacity rings). Recording/query is `.save()` into a mem2 store, not an
> in-memory tap. The text below is retained for context only.

Runtime state per run: for every *edge* (identified by producer `PortRef`) an
**edge log** — an append-only list plus its producer handle:

- **Rim edges**: producer = the resolved source iterator; pulling appends the
  next `.data`-currency item.
- **Member edges**: producer = the member's landed driver. Each member gets
  exactly one `run_over(module, spec, {field: _EdgeReader(edge_of(binding))},
  tf=<reader over tf_bindings, if any>)` — **the same T5 aligner + T6 drivers
  as standalone `over()`**, no graph-side alignment. Pulling one Out row
  appends `getattr(row, f)` to each of the member's per-field edge logs
  (`None` fields append nothing — sparse doctrine).

`_EdgeReader` is a cursor over a log: yield `log[i]` if present, else advance
the producer until it appends or exhausts. Fan-out is N readers on one log
(tee-by-index, the general form of `example_hk_nav`'s `itertools.tee`);
fan-in is one member reading N logs, joined by its own aligner. The whole run
is single-threaded, pull-driven, wall-clock-free — deterministic by
construction, and identical to hand-chaining `over()` calls (the Phase A
equivalence bar). Logs retain the full run in Phase A (re-iterability beats
memory; Q8).

### 6.3 `GraphRun`

```python
class GraphRun(Generic[_TOut]):
    def stream(self, path: str) -> EdgeStream: ...   # "voxel_mapper.global_map"
    def __getattr__(self, name: str) -> EdgeStream: ...  # exported rim outs
    def close(self) -> None: ...                     # idempotent; also __exit__
class EdgeStream:
    def __iter__(self) -> Iterator[Any]: ...         # re-iterable (fresh cursor)
    def to_list(self) -> list[Any]: ...              # drain fully, return payloads
```

`run.path` iterates the export's edge; iteration *drives* the DAG lazily
(pull). `run.stream("voxel_mapper.global_map")` addresses any interior edge by
`<member_path>.<field>`; a bad path → `[graph-unknown-path]` naming the
available edges. Attribute access on a non-export → same error via
`__getattr__`. Streams stay valid after exhaustion (the log persists until
`close()`).

### 6.4 Teardown

`close()` closes member drivers in **reverse application order** (each driver
self-finalizes per T6 §8.2: aligner close, T7 teardown), then closes closeable
rim source iterators. Runs on: explicit `close()`, `with` exit, and full
exhaustion of all rim exports is *not* auto-close (interior taps may still be
wanted) — the handle owns the lifetime, caller-managed (house preference).
Early exit (partial iteration then `close()`) must dispose every member's
resources exactly once — the acceptance test pins it with a counting
`@resource`.

---

## 7. Blueprint lowering (Phase B)

### 7.1 `PureModule.blueprint()` — landed with this spec

```python
@classmethod
def blueprint(cls, *, name: str | None = None, **kwargs: Any) -> Blueprint:
    from dimos.pure.legacy import legacy_blueprint   # lazy: the sanctioned core edge
    return legacy_blueprint(cls, name=name, **kwargs)
```

Pure sugar over the T8 bridge; gives the charter's uniformity (`.blueprint()`
on both PureModule and PureGraph) at zero cost, and drops a pure module beside
a legacy blueprint: `autoconnect(GO2Connection.blueprint(), VoxelMapper.blueprint())`.
Landed live in this branch (module.py), lazily imported so the pm surface
stays core-free (`test_pm_surface` zero-engine pin unaffected).

### 7.2 `PureGraph.blueprint()` — the lowering

```python
@classmethod
def blueprint(cls, *, namespace: str | None = None, **config: Any) -> Blueprint: ...
```

Algorithm (`ns = namespace or snake_case(cls.__name__)`):

1. `plan = cls(**config).build()` — one `wire()`, this is a compile target.
2. Per member `(path, m)`:
   `legacy_blueprint(type(m), instance_name=path.replace(".", "/"), **m.config.model_dump())`
   — one atom per member, per-member process (`dedicated_worker`, T8 §11).
3. **Interior topics = path-qualified by convention.** For every member
   Out-port that feeds an interior edge, remap the producer stream
   `f → "{path}/{f}"` (paths use `/` in blueprint-land, `.` in pure-land); for
   every consumer binding on that edge, remap the consumer stream to the same
   name. Two members' same-named outputs can no longer collide.
4. **Rim inputs.** For each graph In port `n` and each consumer binding
   `(member c, field f)`: remap `c.f → n` (a no-op when names already agree —
   the autoconnect *convention* preserved: rims interoperate by (name, type)).
   Add `n` to the expose set.
5. **Rim exports.** For each export `(n, producer p.f)`: remap `p.f → n`;
   interior consumers of the same edge remap to `n` too (one stream, one name;
   the export name wins). Add `n` to the expose set.
6. `bp = autoconnect(*atoms).remappings([...]) .namespace(ns, expose=rim_names)`
   — interior streams become `{ns}/{path}/{field}`, rim ports stay bare and
   link globally by (name, type). The graph blueprint IS a blueprint namespace;
   `.namespace/.remappings/.transports` compose on the result unchanged (no
   fork).

Feedback edges lower like any interior edge (the sampled consumer subscribes
the producer's topic — pubsub is naturally cyclic; the well-foundedness came
from the sampler, which rides the member). `tf_bindings` lower onto the legacy
tf mechanism: the producing member's `tf_out` publishes through the bridge's
tf conventions; consumers with `tf()` fields subscribe the shared tf topic —
same convention legacy modules use (T8 §12 health-style conventional topic; the
implementer wires `m.i.tf` ↔ the namespace tf stream in `legacy.py`'s start
path if not already carried — verify against `test_legacy` e2e).

Config surface: the composite blueprint's per-atom config keys are
`config_key(instance_name)` (existing mechanism), so CLI/env overrides reach
individual members (`--nav_stack_planner.cost_threshold=...`) with zero new
machinery.

### 7.3 Worked lowering — `NavStack` (charter version)

`NavStack.blueprint(voxel_size=0.1)`, `ns="nav_stack"`; members
`odom_tf, voxel_mapper, cost_mapper, planner`:

| Stream (atom.field) | Remap → effective name | After namespace | Why |
| --- | --- | --- | --- |
| `voxel_mapper.scan` | `lidar` | `lidar` (exposed) | rim In, step 4 |
| `odom_tf.odom` | `odom` (no-op) | `odom` (exposed) | rim In |
| `planner.goal` | `goal` (no-op) | `goal` (exposed) | rim In |
| `voxel_mapper.global_map` | `voxel_mapper/global_map` | `nav_stack/voxel_mapper/global_map` | interior edge |
| `cost_mapper.global_map` | `voxel_mapper/global_map` | ditto | consumer joins producer topic |
| `cost_mapper.global_costmap` | `costmap` | `costmap` (exposed) | exported (name-crossing edge: export wins) |
| `planner.costmap` | `costmap` | `costmap` (exposed name) | interior consumer of an exported edge |
| `planner.path` | `path` (no-op) | `path` (exposed) | exported |
| `odom_tf.tf` | `odom_tf/tf` | `nav_stack/odom_tf/tf` | unexported interior — still addressable (taps/record) |
| each member `health` | — | `nav_stack/health` | merged per-namespace (T8 §12) |

The charter's name-crossing edge (`cost.global_costmap → planner.costmap`)
appears exactly as promised: explicit in `wire()`, a remapping when lowered.

### 7.4 The grounding test (north star)

```python
bp = autoconnect(
    GO2Connection.blueprint(),                # --replay drives lidar/odom onto the bus
    NavStack.blueprint(voxel_size=0.1),
)
ModuleCoordinator.build(bp).loop()
```

GO2's `lidar: Out[PointCloud2]` / `odom: Out[PoseStamped]` link to NavStack's
exposed rim by (name, type); NavStack's interior never collides (namespaced).
Acceptance: with `--replay go2_hongkong_office.db`, a subscriber on `path`
observes ≥1 `Path` and on `costmap` ≥N grids within the replay window; no
topic collisions; coordinator clean shutdown. Marked integration (needs
`get_data` + worker pool), Phase B's exit bar.

**The `cmd_vel` seam** (flagged for ratification): GO2's `cmd_vel: In[Twist]`
is unstamped legacy currency; a pure controller must emit stamped payloads
(rim egress enforces readable `ts` — `[rim-unstamped-out]`), i.e.
`TwistStamped` (which *subclasses* `Twist`). Autoconnect's `(name, type)` key
matches exact types, so `("cmd_vel", TwistStamped)` ↛ `("cmd_vel", Twist)`:
silent no-link. Proposed resolution, in preference order:
(a) **autoconnect matches covariantly** — `In[T]` accepts `Out[S]` when
`issubclass(S, T)`; principled (a stamped twist *is* a twist), fixes every
future stamped/unstamped seam, one small matcher change;
(b) GO2Connection gains `cmd_vel_stamped: In[TwistStamped]` feeding `move()`;
(c) a one-field adapter module. Default: (a); not blocking Phase B (NavStack
exports no cmd_vel).

---

## 8. Sources are values (§5) — deferred surfaces

- **Interior injection `at=`** (Phase C): `over(at={"planner.costmap": stream})`
  severs `planner.costmap`'s inbound edge and feeds the stream through the same
  `_EdgeReader` slot — develop a downstream member against a recording with no
  upstream in the loop. Validation: the path must name a bound member In field;
  type agreement as §4.2/4. *The sanctioned escape hatch — explicitly listed
  for the owner's blessing.*
- **Live single-process `bind()`** (Phase C): `NavStack().bind(robot).source(goal=[goal])`
  — rim In ports bind transports (the member rims already do this via T8; the
  graph composes per-member rim sessions over in-process edges). Phase B's
  coordinator path covers live deployment until then (Q6).
- **`partition()` native distributed** (Phase C): cut edges by member-path
  groups, pubsub transports on cuts, per-partition placement. Same `wire()`;
  config over paths.
- **Flight recorder** (Phase C): `run/graph.store = SqliteStore(...)` writes
  every edge log to channels named `"{ns}.{path}.{field}"` — the record example
  in the charter. In Phase A the in-memory logs already give post-run access.

All Phase-C entry points exist in the skeleton with real signatures and raise
`[graph-impl-pending]`.

---

## 9. Phasing

### Phase A — core + local `over()` (independently landable)

Delivers: T1 amendment (§2.4); `graph.py` (classification, symbolic
application, build + full §5 catalog, `GraphPlan`); `module.py` `__call__`
overloads + application dispatch; `PureModule.blueprint()`; local pull runtime
(§6) with `GraphRun`; static fixture green.

Acceptance:
1. every §5 build/definition error verbatim-tested;
2. **equivalence bar**: a toy 3-member DAG (fan-out + fan-in + name-crossing)
   run via `graph.over(**streams)` yields per-edge sequences identical to
   hand-chained `m.over()` calls with `tee` — the graph adds nothing to the
   data;
3. **NavStack port**: `example_hk_nav`'s DAG respelled as `NavStack(PureGraph)`
   over `go2_hongkong_office.db` (data-marked test): same number of paths, same
   final map/costmap payloads as the hand wiring; the example file's DAG body
   is replaced by the graph (sink stays a member or a tap);
4. purity: `build()` twice → equal plans; building runs no resources
   (counting `@resource` untouched);
5. teardown: early `close()` after k rows disposes every member resource once;
6. mypy --strict clean; 454 landed tests unregressed.

### Phase B — blueprint lowering

Delivers: `PureGraph.blueprint()` (§7.2), tf/health conventions verified
through the bridge, the seam decision executed.

Acceptance: 1. lowering unit tests — the §7.3 table asserted literally against
the produced `Blueprint` (atom names, remapping_map, expose survivors,
transport-free); 2. a two-pure-module composite runs under `ModuleCoordinator`
in-process workers end-to-end; 3. the §7.4 grounding test (integration-marked).

### Phase C — deferred runtime

Feedback loop closing in the local engine (bounded by sampler semantics);
`at=` interior injection; `bind()`/`source()` live graph; `partition()`;
flight recorder. Acceptance per feature: SlewNav (charter) runs locally with a
latest() back edge and replays identically; planner-only re-run over an
injected recorded costmap matches the charter's §5(c) sketch.

---

## 10. Reused from the landed engine

| Landed piece | Used by | How |
| --- | --- | --- |
| T5 `align` (via T6) | §6.2 | every member's In row resolution — the ONE aligner; graph adds no alignment |
| T6 `run_over`/drivers | §6.2 | one driver per member over `_EdgeReader`s; teardown via driver self-finalization |
| T7 `@resource` | §6.4 | member warmup/teardown ride `RunHooks` untouched |
| T3 `StepSpec`/`step_spec` | §4.2 | application validation against `in_type.fields()`/kinds |
| T3 `_resolve_step_hints` | §2.1 G3 | wire annotation resolution + error lesson copy |
| T2 `_collect_config_fields`/`_synthesize_config_model` | §2.2 | graph config synthesis, mirrored inheritance |
| T1 `fields()`/`FieldSpec` kinds | §2.3, §4.2 | bundle policy + sampler-kind checks (+`BareSpec` amendment) |
| T11 tf specs | §4.4 | tf side-channel binding validation (build-time mirror of runtime copy) |
| T8b `legacy_blueprint`/`legacy_actor` | §7 | each member's deployment face; `PureModule.blueprint()` sugar |
| Blueprint `.namespace/.remappings`/`autoconnect` | §7.2 | the lowering — convention kept, function dropped; no fork of namespace |
| T4 overload/self-protocol doctrine | §3.3–3.4 | `__call__` application typing; `Wires` protocol |
| mem2 `Stream` coercion (T6) | §6.1 | store streams pass through member `run_over` unchanged — no second boundary |

---

## 11. Open Questions

- **Q1 member naming** — default: `snake_case(cls.__name__)` + `_2/_3` by
  application order; explicit via `named(name, m)`. Options: (a) as specced;
  (b) reserved application kwarg (`name_=`) — rejected: pollutes the port
  namespace; (c) AST/varname sniffing — rejected: nondeterministic.
- **Q2 `[graph-unused-input]` severity** — default: error (mypy-for-wiring
  strictness). Options: (a) error; (b) warning + plan flag.

  USER ANSWER: B - warn

- **Q3 export aliasing** — default: error for both alias and rim passthrough
  (target parity: one topic per edge). Options: (a) error; (b) legal locally,
  error only at `blueprint()` — rejected: wire() would mean different things
  per target.
- **Q4 `GraphRun` port streams carry payloads** (not Out rows) — default:
  payload-per-edge (`run.path` yields `Path`), matching the edge currency and
  the charter's `run.path.to_list()`. Option: rows-with-ts envelopes.
- **Q5 flight recorder phase** — default: C (in-memory logs suffice for A/B).
- **Q6 live single-process `bind()` phase** — default: C (Phase B's
  coordinator path is the live deployment story “today”, per the charter).
- **Q7 pm surface** — default: graph names stay in `dimos.pure.graph` (the
  charter imports them from there); `pm.__all__` untouched, `test_pm_surface`
  unchanged. Option: re-export `PureGraph/Port/feedback` from `pm` later.
- **Q8 edge-log retention** — VOID (resolved by §0.3): no edge logs exist, so
  there is nothing to retain. Recording/query is `.save()` into a mem2 store.
- **Q9 blueprint namespace** — default: `namespace or snake_case(cls.__name__)`;
  `namespace` is reserved in graph config (collision → T2 catalog).
- **Q10 `cmd_vel` seam** — RESOLVED (§0.4): a `TwistStamped → Twist` translator
  pure module, not covariant matching; GO2 accepts `TwistStamped` later.

## 12. Relitigation

None. The one enabling change to landed code — §2.4's T1 `BareSpec` amendment —
implements the charter's own §3 spelling (`class In(pm.In)` with bare fields
*cannot define* under landed T1); it is additive, preserves module-side
loudness by relocating the check to the module class statement, and is flagged
for orchestrator ratification rather than argued around. Everything else in
the charter lowered onto the landed engine without contradiction; notably the
tick-cycle rule reduced to a local feedback-binding check (§4.3) because
non-feedback cycles are structurally unconstructible — a simplification of the
charter's rule, not a change to it.
