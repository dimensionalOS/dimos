# T4 — Static typing surface + mypy regression harness

Status: **spec + verified implementation of the static surface**. Unlike the
sibling specs, T4's deliverable IS static behavior, so this branch carries the
real `dimos/pure/typing.py` (mypy --strict clean) and a running harness
(`dimos/pure/test_typing_static.py` + `test_typing_fixtures/`, 19 tests, all
green, ~1 s). Every claim below marked *verified* was checked against mypy
1.19.0 with the repo config on this branch; the harness pins each one in CI.
What remains for implementers is the runtime half (T6 drivers behind `over()`,
T8 rim behind the port accessors) and the adoption notes for T1/T2.

Source of truth: `dimos/memory2/puremodule_api_sketch3.py` §TYPING, as amended
by the T3 coordination decisions recorded here (§2.3, §10).

---

## 1. Design at a glance

No generics on module classes. The step signature is the single typing
authority; engine methods recover `TIn`/`TOut` through a **protocol-typed
`self`**: each step shape is a `Protocol`, `over()` is an overload set whose
`self` parameters are those protocols, and mypy unifies the protocol's
typevars against the concrete step at every call site. `m.i`/`m.o` use the
same trick one level down — descriptors whose `__get__` is overloaded on a
protocol-typed instance argument.

Layering (index.md, cross-cutting): `typing.py` is part of the zero-runtime-dep
data layer. It imports only `collections.abc` + `typing`. The engine imports
it, never the reverse — with exactly two sanctioned *lazy* edges (§4.4, §5.5)
whose module-level import graph stays acyclic.

```
users:      class Tagger(PureModule): ... def step(self, i: In) -> Out
T2 module.py:   class PureModule(EngineSurface, ...)      # inherits the surface
T4 typing.py:   EngineSurface.over() overloads, _In/_OutAccessor, protocols
T6 drivers.py:  run_over(module, spec, streams)           # behind over()
T8 rim.py:      builds InPorts/OutPorts views              # behind m.i / m.o
```

## 2. The protocols

Defined in `dimos/pure/typing.py`, exported via `__all__` (§6). All members
are `...`-bodied protocol methods; data parameters are **positional-only**.

```python
TIn = TypeVar("TIn", contravariant=True)
TOut = TypeVar("TOut", covariant=True)
TState = TypeVar("TState")                      # invariant

class Stateless(Protocol[TIn, TOut]):
    def step(self, i: TIn, /) -> TOut | None: ...

class AsyncStateless(Protocol[TIn, TOut]):
    def step(self, i: TIn, /) -> Awaitable[TOut | None]: ...

class Mealy(Protocol[TState, TIn, TOut]):
    def step(self, state: TState, i: TIn, /) -> tuple[TState, TOut | None]: ...

class Fold(Protocol[TIn, TOut]):
    def fold(self, rows: Iterator[TIn], /) -> Iterator[TOut]: ...
```

### 2.1 Variance — why, member by member

- **`TIn` contravariant.** `TIn` appears only in parameter position. A step
  that accepts a *wider* row works anywhere a narrower row is fed:
  `Stateless[BaseIn, X] <: Stateless[RichIn, X]`. This is what lets a generic
  module (accepts `BaseIn`) slot into a pipeline that produces `RichIn` rows.
  *Verified:* `case_variance.py` pins the acceptance and the `[arg-type]`
  rejection of the converse (a step *demanding* `RichIn` offered `BaseIn`).
- **`TOut` covariant.** `TOut` appears only in return position. A step that
  emits a *richer* row satisfies consumers expecting the base:
  `Stateless[X, RichOut] <: Stateless[X, BaseOut]`. *Verified:* same fixture,
  both directions.
- **`TState` invariant.** It appears in both parameter (`state: TState`) and
  return (`tuple[TState, ...]`) position; any declared variance is unsound and
  mypy would reject the protocol definition. Plain `TypeVar`.
- In `Fold`, `Iterator[TIn]` as a parameter is a covariant constructor in
  contravariant position — net contravariant in `TIn`; return `Iterator[TOut]`
  is covariant in `TOut`. Same declared pair, mypy accepts the variance.

### 2.2 Spelling decisions (each deliberate)

- **`TOut | None` in the sync/Mealy/async return.** This single spelling
  makes both `-> Out` and `-> Out | None` steps match with `TOut = Out`:
  mypy solves `Out <: TOut | None` as `TOut = Out`, so *skips never reach
  consumers* — `over()` yields `Iterator[Out]`, not `Iterator[Out | None]`.
  *Verified:* `case_infer_stateless.py`, `case_infer_optional.py`,
  `case_infer_mealy.py`, `case_infer_async.py`.
  - Degenerate corner, pinned as documented behavior: a `-> None`-only step
    leaves `TOut` unconstrained; mypy solves it to `Never`, so `over()` is
    `Iterator[Never]` — apt for a module that never emits
    (`case_infer_optional.py::NoneOnly`).
- **`Awaitable[...]`, not `Coroutine[...]` and not `async def` in the
  protocol.** `async def step(self, i: In) -> Out` has type
  `(In) -> Coroutine[Any, Any, Out]`; `Coroutine <: Awaitable` and covariance
  in the result solves `TOut = Out`. `Awaitable` is the widest sound member
  type and keeps the protocol syntax-agnostic. The static/runtime divergence
  this opens is deliberate and documented (§10b): a *sync* `def` returning
  `Awaitable[Out]` matches statically but T3 rejects it at import with
  `[step-returns-awaitable]` (one-spelling doctrine). *Verified:*
  `case_infer_async.py::AwaitableReturner` pins the mypy-accepts side with a
  comment pointing at T3.
- **`tuple[TState, TOut | None]`** also accepts a user's `tuple[State, Out]`
  (tuple covariance) — a Mealy step that always emits needs no `| None`.
  *Verified:* `case_infer_mealy.py::AlwaysEmits`.
- **Mealy parameter order `Protocol[TState, TIn, TOut]`** matches the sketch;
  keep it — reordering is a silent API break for structural aliases.

### 2.3 Positional-only data params — DECISION (T3 coordination 2a): ACCEPTED

Protocols spell data params `def step(self, i: TIn, /)`. Rationale:

1. Without `/`, structural conformance binds every implementation to the
   literal names `i`/`state`/`rows`: renaming `i` → `frame` would silently
   stop the module matching any protocol, surfacing as a distant
   `Invalid self argument` at `over()` — a cosmetic rename must never do that.
2. The engine invokes steps positionally (T3/T6), so the protocol only
   promises what the engine uses.
3. It strictly widens acceptance; nothing that matched before stops matching.
4. Cost, pinned as documented behavior: keyword-calling `step` through a
   *protocol-typed* reference is rejected (`m.step(i=row)` →
   `[call-arg] Unexpected keyword argument "i"`). Concrete-typed calls
   (`Tagger().step(i=row)`) keep the concrete signature and still allow it.

*Verified:* `case_param_names.py` (renamed stateless + renamed Mealy params
infer fully; keyword call through `Stateless[...]` errors). Consequence
accepted by the orchestrator: T3's G2 gate permits positional-only impl
params rather than adding a param-name rule.

### 2.4 Unbound typevars — final, not provisional

`TIn`/`TOut`/`TState` are **permanently unbound** (no `bound=pm.In` etc.).
A protocol describes step *shape*; bundle-ness (rows are `pm.In`/`pm.Out`
subclasses with `ts`) is enforced at import time by T3, not by the type
system. Binding would (a) force `typing.py` to import `rows.py` at runtime,
breaking the data-layer independence and the sibling-task boundary, and
(b) buy nothing: mismatched bundles already fail structurally on the field
types. The harness fixtures deliberately use plain classes as row stand-ins;
production bundles are strictly more typed.

## 3. What PureModule must (not) do — contract for T2/T3

- `class PureModule(EngineSurface)` — inherit the surface; do not redeclare
  `over`, `i`, `o`.
- **Never declare `step` or `fold` on `EngineSurface`/`PureModule`** (not
  even an abstract stub or a doc placeholder). A base-level `step` would make
  *every* subclass match the protocols with the base's types and destroy
  per-module inference. This is stated in the `EngineSurface` docstring; the
  harness would catch a violation as mass reveal drift.
- `i = _InAccessor()` / `o = _OutAccessor()` are *unannotated* class
  attributes, so T2's `@dataclass_transform` field collection (annotated
  assignments only) never sees them. *Verified:* `case_config_kwargs.py` and
  `case_tagger_floor.py` run the full transform stack over `EngineSurface`
  with zero interference.
- A module with both `step` and `fold` statically resolves to the step
  overload (first match wins); T3 rejects it at import. *Verified:*
  `case_invalid_self.py::StepAndFold`.

## 4. `over()` — the overload set

On `EngineSurface` (typing.py), inherited by `PureModule`:

```python
@overload
def over(self: AsyncStateless[_TIn, _TOut], **streams: Streamable) -> Iterator[_TOut]: ...
@overload
def over(self: Mealy[_TState, _TIn, _TOut], **streams: Streamable) -> Iterator[_TOut]: ...
@overload
def over(self: Stateless[_TIn, _TOut], **streams: Streamable) -> Iterator[_TOut]: ...
@overload
def over(self: Fold[_TIn, _TOut], **streams: Streamable) -> Iterator[_TOut]: ...
def over(self: Any, **streams: Streamable) -> Iterator[Any]: ...
```

### 4.1 ORDER is load-bearing

`AsyncStateless` **must precede** `Stateless`: a coroutine-returning step
also matches `-> TOut | None` by solving
`TOut = Coroutine[Any, Any, Out]` — listed sync-first, an async module's
`over()` would yield coroutines. *Verified by mutation:* swapping the two
overloads makes exactly `case_infer_async.py` fail (revealed
`Coroutine[Any, Any, Captioner.Out]`) while all else stays green — the
harness is the permanent guard. `Mealy`/`Stateless` are disjoint by arity
(*verified:* `case_arity.py`, both `[arg-type]` rejections), so their
relative order is free; keep the sketch's. `Fold` matches only fold-modules
(no `step` member) — last, as the escape hatch.

### 4.2 Solver twins

Overload signatures use fresh *invariant* typevars `_TIn`/`_TOut`/`_TState`
(module-private), because declared-variance typevars are for generic class
params, not function generics. The public variant pair (§2) stays on the
protocols. Do not merge the families.

### 4.3 `Streamable` — provisional by design

`Streamable: TypeAlias = object` (module-level, not exported). It exists so
the overload set is stable while T6 owns the real bound: when drivers land,
T6 tightens the alias (expected: the stamped-msg stream protocol shared with
`dimos.memory2.stream` accessors) *without touching the overload structure*.
Until then every kwarg value is accepted statically; alignment (T5) validates
at runtime.

### 4.4 The runtime implementation (contract for T6)

One implementation behind the overloads, `self: Any` (this exact spelling —
it is what lets the impl accept the protocol-typed overload selfs without a
`type: ignore[misc]`, an improvement over the design-session scratch which
needed the ignore). Final body:

```python
def over(self: Any, **streams: Streamable) -> Iterator[Any]:
    from dimos.pure.stepspec import step_spec   # lazy: sanctioned edge #1
    from dimos.pure.drivers import run_over     # T6

    return run_over(self, step_spec(type(self)), streams)
```

- Dispatch is on **T3's classification record**: `step_spec(cls)` returns the
  frozen `StepSpec` stamped as `cls.__pure_step__` (module
  `dimos.pure.stepspec`; fields `kind/in_type/out_type/state_type/skips/owner`;
  `kind: StepKind ∈ {STATELESS, ASYNC_STATELESS, MEALY, FOLD}`). `over()`
  never re-inspects signatures — T3 classified once at import.
- The **lazy local import** is the sanctioned inversion of the layering rule:
  module-level import graph stays engine-free (constructing/unit-testing
  modules never loads drivers); the runtime edge exists only when a run is
  actually requested. Skeleton ships `raise NotImplementedError` until T6.
- Failure mode when a module reached `over()` without a spec (T3 bypassed —
  shouldn't happen): raise `TypeError` naming the class and pointing at the
  step/fold rule.

### 4.5 The priced error surface

A malformed module (no step, wrong arity) is **not** a definition-site mypy
error; it fails at the first engine touch as
`Invalid self argument "X" to attribute function "over"` — code `[misc]`
(*verified:* `case_invalid_self.py`; exact wording pinned there, including
that the message quotes the *first* overload, i.e. AsyncStateless). T3's
`__init_subclass__` is the compensation with the good error message at import
time. Docs should teach: "Invalid self argument at `.over()` ⇒ your step
signature doesn't match any shape — the import-time error names the rule."

## 5. Port handles — `m.i` / `m.o`

### 5.1 Accessors

`_InAccessor` / `_OutAccessor` are descriptor classes instantiated once each
on `EngineSurface`. `__get__` is overloaded on the *instance* argument:

```python
class _InAccessor:
    @overload
    def __get__(self, obj: None, owner: type) -> _InAccessor: ...          # class access
    @overload
    def __get__(self, obj: AsyncStateless[_TIn, _TOut], owner: type | None = None) -> InPorts[_TIn]: ...
    @overload
    def __get__(self, obj: Mealy[_TState, _TIn, _TOut], owner: type | None = None) -> InPorts[_TIn]: ...
    @overload
    def __get__(self, obj: Stateless[_TIn, _TOut], owner: type | None = None) -> InPorts[_TIn]: ...
    @overload
    def __get__(self, obj: Fold[_TIn, _TOut], owner: type | None = None) -> InPorts[_TIn]: ...
    def __get__(self, obj: Any, owner: type | None = None) -> Any: ...
```

`_OutAccessor` is identical with `OutPorts[_TOut]` returns. Notes:

- **Same overload order as `over()`, same reason** — and here the sync-first
  bug would corrupt `m.o` (solving `_TOut = Coroutine[...]`) while leaving
  `m.i` correct, which is why the canary pins `AsyncMod().o`
  (*verified:* `case_ports.py`).
- The `obj: None` overload first: `Tagger.i` (class access) reveals the
  accessor itself (*verified*), keeping introspection sane.
- All four shapes covered so fold/Mealy modules get typed ports too
  (*verified:* `case_ports.py::{StatefulMod, FoldMod}`).

### 5.2 Views and handles

```python
class InPorts(Generic[_TBundle]):
    def __getattr__(self, name: str) -> InPort[Any]: ...

class OutPorts(Generic[_TBundle]):
    def __getattr__(self, name: str) -> OutPort[Any]: ...

class InPort(Generic[_TMsg]):
    transport: Any          # settable; T8 tightens
    source: Any             # settable; mutually exclusive with transport (T8)

class OutPort(Generic[_TMsg]):
    transport: Any          # settable; T8 tightens
    @property
    def frames(self) -> tuple[str, str]: ...        # tf_out ports only (T11)
    def subscribe(self, fn: Callable[[_TMsg], None]) -> Any: ...
```

The sketch's deployment lines all typecheck against this surface
(*verified:* `case_ports.py` — `m.i.x.transport = ...`, `m.i.x.source = ...`,
`m.o.y.transport = ...`, `m.o.y.frames`, `m.o.y.subscribe(...)`).

### 5.3 Read-only-property rule (design-session scar, now doctrine)

**Any protocol/surface member that a runtime descriptor or engine provides is
spelled as a read-only `@property`, never a plain variable** — a plain
variable demands settability from every conforming implementation and
forbids property-based ones. Applied here: `OutPort.frames`. Deliberately
NOT applied to `transport`/`source` — settability is their point (users
assign them), so they are plain `Any` attributes. *Verified:* assigning
`frames` is `[misc] Property "frames" defined in "OutPort" is read-only`
(`case_ports.py`).

### 5.4 The per-field ceiling — honest statement

`m.i`/`m.o` are typed **per-module** (the bundle parameter is exact, zero
annotations on the module); per-**field** access returns `InPort[Any]` /
`OutPort[Any]` via `__getattr__`, so field-name typos and per-field value
types are NOT statically checked (`case_ports.py` pins the hole
deliberately: `m.i.imagee` reveals `InPort[Any]`). Why this is the ceiling
and the alternatives were rejected — see §11 Relitigation. Runtime
compensation (T8 contract): the views validate attribute names against
`Bundle.fields()` (T1) and raise `AttributeError` naming the bundle and its
fields — same compensation pattern as T3-for-def-site.

### 5.5 Runtime contract (T8)

Accessor `__get__` bodies mirror `over()`: lazy-import the rim and build the
per-module view (sanctioned edge #2), keyed off the same `StepSpec`
(`in_type`/`out_type` provide the field sets via T1's `fields()`).
Skeletons `raise NotImplementedError` until T8. `InPorts`/`OutPorts`/
`InPort`/`OutPort` stay in `typing.py` as the *static* surface; T8
implements against them (subclass or duck — instances only need to satisfy
the declared members). `frames` raises on non-tf ports until T11 defines it.

## 6. Exports & stdlib-shadowing hygiene

`__all__ = ["AsyncStateless", "EngineSurface", "Fold", "InPort", "InPorts",
"Mealy", "OutPort", "OutPorts", "Stateless"]`.

- **Users** import `Stateless` (structural shape aliases —
  `CostMapperLike = Stateless[CostMapper.In, CostMapper.Out]`), occasionally
  the other protocols for helper signatures, and the port types for rim
  helper annotations. `pm.Stateless` comes from `dimos/pure/__init__.py`
  re-exporting *names* (T2/T12 own `__init__.py`; rule: re-export the names
  in `__all__`, never the submodule, and never `import *`).
- **Internal** (importable, not exported): `EngineSurface` is consumed by
  exactly one client, T2's `PureModule`. The typevars and `Streamable` are
  module-level for T6 to reference; the accessors are private.
- **The module is named `typing` inside the package.** Python 3 absolute
  imports make `import typing` inside sibling modules unambiguous (stdlib).
  House rules, stated in the module docstring: import it absolutely
  (`from dimos.pure.typing import Stateless` or
  `import dimos.pure.typing as pure_typing`); never bind the bare name
  `typing` to this module; no star re-exports anywhere in `dimos/pure`.
  mypy resolves the package-qualified module against the stdlib one
  correctly (*verified* throughout the harness, which imports it from
  fixtures).
- Python floor 3.10: everything used exists in 3.10's `typing`
  (`TypeAlias`, `Protocol`, `overload`); `typing_extensions` is only needed
  by fixtures for `dataclass_transform` (T1/T2 surface, already a repo dep).
  No PEP 695 syntax anywhere (3.12-only).

## 7. The harness

### 7.1 Mechanism — DECISION: mypy-under-pytest (subprocess), not pytest-mypy-plugins

| | mypy subprocess from pytest (chosen) | pytest-mypy-plugins |
|---|---|---|
| New dev dep | **none** (mypy already a dev dep; not in dev deps today) | yes — needs adding |
| Invocations | ONE batched run for the whole suite (~0.9 s cold) | one mypy run *per case* (tens of seconds) |
| Assertions | plain pytest asserts on structured diagnostics; codes exact, messages substring | YAML DSL, exact-output matching, brittle across mypy versions |
| Config | repo `pyproject.toml` verbatim — the gate's own strictness | separate ini blocks per case, drift risk |
| Isolation | subprocess: no in-process global state, xdist-safe, `cwd` pinned | in-process plugin machinery |
| Fixture form | real `.py` files mypy-checkable standalone | cases embedded in YAML strings (no editor/typing support) |

`mypy.api.run()` (in-process) was considered and rejected in favor of
`subprocess([sys.executable, "-m", "mypy", ...])`: same engine and output,
but the subprocess is immune to mypy's process-global state, pytest's cwd,
and coverage tracing. **No pyproject.toml change was needed or made.**

### 7.2 Invocation (exact, implemented)

```
{sys.executable} -m mypy --config-file pyproject.toml --cache-dir {os.devnull}
                 --no-error-summary --no-color-output <fixture paths…>
cwd = repo root
```

- `--config-file pyproject.toml` + repo root cwd: fixtures are checked under
  the *gate's own strict config* (incl. `python_version = 3.12`,
  `mypy_path = stubs/`). Runtime floor stays 3.10 — fixtures use nothing
  3.11+.
- `--cache-dir os.devnull` disables incremental caching: hermetic, no
  cross-run staleness, no cache races with the gate; the whole batched run
  is < 1 s cold, so caching buys nothing.
- Explicit file args: *verified* that mypy checks explicitly-passed files
  even when they match the config `exclude` (this is what makes the layout
  in §7.3 possible), and that `files = ["dimos/"]` is ignored when paths are
  passed.
- Exit codes 0/1 are success (1 = "errors found", expected); 2 = mypy
  usage/crash fails the session fixture.

### 7.3 Layout & gate mechanics (verified against pyproject)

```
dimos/pure/test_typing_static.py        driver — pytest collects (test_*.py),
                                        gate mypy EXCLUDES (matches .*/test_.)
dimos/pure/test_typing_fixtures/        fixtures — path contains /test_t…
    case_*.py                           → whole dir invisible to gate mypy;
                                        files NOT named test_*.py
                                        → invisible to pytest collection
```

The repo gate (`[tool.mypy] files=["dimos/"]`,
`exclude = …|.*/test_.|…`) discovers recursively, so **both** the driver and
the fixtures are invisible to it — deliberate errors can never break the
gate. *Verified twice:* recursive `mypy dimos/pure` reports
"no issues found in 2 source files" (typing.py, module.py); and the guard
test `test_gate_excludes_fixtures` runs discovery over the fixtures dir and
asserts mypy's "There are no .py[i] files" refusal, so a future rename of
the dir or an edit to the exclude regex fails CI *before* it breaks the gate.
pytest-side: `testpaths=["dimos"]` + default `test_*.py` globs collect the
driver and skip `case_*.py`; `--dist=loadfile` keeps the whole harness on
one xdist worker, so the session-scoped mypy fixture runs once
(*verified* under `-n 2`).

### 7.4 Marker DSL (implemented in the driver)

Same-line comments in fixtures, **one marker per line**, one diagnostic per
marked line:

```
# E[code]: substring     expect an error with exactly this code, message
                         must contain the substring
# R: type                expect note `Revealed type is "type"`; compared
                         EXACTLY after stripping the fixture's own module
                         prefix (dimos.pure.test_typing_fixtures.<stem>.)
                         from the revealed string
```

Matching is **bidirectional and exact**: an unmarked error, an unmarked
reveal, a wrong code, a missing substring, or a marker with no diagnostic
all fail the case, with per-line explanations. Non-reveal notes (Liskov
explanations, protocol conflict dumps, "defined here" companions) are
ignored by design — messages drift across mypy versions; codes and revealed
types are the stable currency. Stray *errors* outside fixture files (e.g. a
regression inside typing.py itself, which the run analyzes as an import) fail
`test_only_fixture_diagnostics`; stray *notes* pointing at typing.py are
expected companions and ignored.

Because markers ride their line, license-header insertion and edits above a
marker never desynchronize expectations — the parser recomputes line numbers
from the same file mypy reads.

### 7.5 Fixture style rules (enforced, some by tests)

1. Self-contained: each fixture defines its own row/module stand-ins; the
   ONLY project import is `dimos.pure.typing` (so the suite tests the real
   shipped surface, and a broken shared helper can't fail everything).
2. Never imported at runtime — fixtures may use bare `reveal_type` and
   deliberate errors; the driver never imports them, pytest never collects
   them (`case_` prefix, enforced by the dir's naming convention).
3. Marked statements stay on ONE line ≤ 100 chars, so ruff-format cannot
   move a marker off its diagnostic line; `test_fixture_format_stability`
   asserts `ruff format --check` is a no-op over the fixtures dir.
4. No bare attribute/comparison expression statements (ruff B015/B018);
   assign to a name or wrap in `reveal_type(...)`.
5. Non-None-returning stub bodies `raise NotImplementedError` (strict mode's
   `[empty-body]` rejects `...` bodies outside protocols).
6. New case = new `case_<name>.py` with markers; the driver's
   `glob("case_*.py")` parametrization picks it up — no registration.

### 7.6 Guard tests (beyond the per-fixture cases)

- `test_only_fixture_diagnostics` — typing.py contributes zero errors when
  analyzed as the fixtures' import.
- `test_gate_excludes_fixtures` — §7.3, pins gate invisibility.
- `test_fixture_format_stability` — §7.5.3.
- `test_runtime_surface` — `import dimos.pure.typing` works at runtime
  (namespace pkg), `__all__` names exist, protocols are runtime-subscriptable
  (structural aliases like `CostMapperLike` work outside `TYPE_CHECKING`),
  and every unimplemented surface (`over`, `m.i`, `m.o`) raises
  `NotImplementedError` rather than silently doing nothing.

### 7.7 Harness self-test protocol

The harness was mutation-tested during design: swapping the
AsyncStateless/Stateless overload order broke exactly
`case_infer_async.py`; a stray error in typing.py trips
`test_only_fixture_diagnostics`. When touching the surface, implementers
should repeat the habit: mutate, watch the specific case fail, revert.

## 8. The seed suite (all implemented and green)

| case | pins |
|---|---|
| `case_infer_stateless.py` | bare `-> Out`: `over()` → `Iterator[Tagger.Out]`; row field types flow |
| `case_infer_optional.py` | `-> Out \| None` solves `TOut = Out`; `-> None`-only solves `Never` (documented) |
| `case_infer_async.py` | async → `Out` (ORDER canary); async `\| None`; sync `-> Awaitable[Out]` matches statically w/ T3 divergence note |
| `case_infer_mealy.py` | Mealy → `Out`; `tuple[State, Out]` w/o None also matches |
| `case_infer_fold.py` | fold-only module lands on Fold overload → `Iterator[Out]` |
| `case_arity.py` | Mealy ⇸ Stateless and Stateless ⇸ Mealy, both `[arg-type]` |
| `case_variance.py` | contravariant-In & covariant-Out acceptances + both `[arg-type]` rejections |
| `case_override.py` | shape conformance at the def: wrong return `[override]`; narrowed input `[override]`; conforming impl infers through `over()` |
| `case_twin.py` | structural twin `Stateless[Shape.In, Shape.Out]`: nominal & unrelated-structural accepted; different bundle `[arg-type]` |
| `case_param_names.py` | §2.3: renamed params infer (stateless + Mealy); keyword call through protocol `[call-arg]` |
| `case_invalid_self.py` | no-step / bad-arity → `[misc] Invalid self argument` at `over()`; step+fold → step wins statically |
| `case_ports.py` | `m.i`/`m.o` per-module reveals for all four shapes; accessor ORDER canary (`AsyncMod().o`); transport/source/subscribe/frames surface; `frames` read-only `[misc]`; the per-field Any hole, pinned; class access reveals accessor |
| `case_rows_required.py` | T1 seed: specifier w/o `default=` ⇒ REQUIRED (`[call-arg]` missing); defaulted ⇒ optional; wrong type `[arg-type]`; unknown kwarg `[call-arg]`; positional `[misc]`; **ts-on-transformed-layer finding (§9)** |
| `case_config_kwargs.py` | T2 seed: typed config kwargs; typo `[call-arg]` w/ did-you-mean; wrong type `[arg-type]`; nested class not a field `[call-arg]`; `@resource` not a field; transform + protocols compose (`c.over()` infers) |
| `case_tagger_floor.py` | **THE floor**: canonical 4-declaration Tagger over full stand-in stack — construction, `step` call, `over()`, `m.i`/`m.o`, required-field errors — zero added annotations |

## 9. Findings exported to sibling tasks

- **T1 (rows), load-bearing:** annotations on the
  `@dataclass_transform`-decorated class itself are NOT fields (the decorated
  class is not transformed; *verified* — `ts` declared there produced
  `Unexpected keyword argument "ts"`). Therefore the decorated root must sit
  **above** `pm.In`/`pm.Out`, which subclass it and declare `ts: float` on the
  transformed layer — then `ts` is a real inherited kw-only field
  (*verified:* `case_rows_required.py`, `case_tagger_floor.py` both use this
  structure). T1's `rows.py` must adopt it and should absorb
  `case_rows_required.py` (keep it here; T1 extends).
- **T1:** specifier functions need no `default=` parameter to mark
  requiredness — mypy keys on the *call site* lacking `default=`; `tick()`
  with only `expect_hz` yields a required field. Declare
  `field_specifiers=(tick, latest, interpolate, contract, …)` exhaustively.
- **T2:** `EngineSurface` inheritance is transparent to
  `@dataclass_transform` (accessors unannotated, `over` a method); nothing
  extra needed beyond §3.
- **T3:** the `[step-returns-awaitable]` divergence is now documented on the
  static side (§2.2, §10b); G2 relaxes to positional-only impl params (§2.3).

## 10. T3 coordination decisions (recorded)

- **(a) Positional-only protocol params: ACCEPTED** — §2.3.
- **(b) Sync-`Awaitable` divergence: DOCUMENTED** — §2.2;
  `case_infer_async.py::AwaitableReturner` carries the cross-reference.
- **(c) Dispatch names adopted** — `dimos.pure.stepspec.StepSpec`
  (`kind/in_type/out_type/state_type/skips/owner`), `StepKind`
  (`STATELESS/ASYNC_STATELESS/MEALY/FOLD`), accessor `step_spec(cls)`,
  dunder `cls.__pure_step__` — §4.4; referenced by name only (spec text +
  future `TYPE_CHECKING` import), never imported at module level here.

## 11. Relitigation

### Per-field port typing stops at `Port[Any]` (vs "m.o.pose types as the Out bundle's fields")

- **Decision under review:** the T4 brief's aspiration that `m.o.pose` be
  statically typed *per field* (name-checked, value-typed `OutPort[PoseStamped]`).
- **What breaks:** Python's type system has no mapped types — there is no way
  to derive "same attribute names as `TOut`, each wrapped in `OutPort[…]`"
  from a generic parameter. The candidates:
  1. *SQLAlchemy-style descriptor annotations* (`pose: Mapped[PoseStamped]`)
     would give exact per-field class-level types — but changes the bundle
     spelling from `pose: PoseStamped = contract(min_hz=10)` and thus **breaks
     the Tagger four-declaration floor** (index.md API-floor rule). Rejected.
  2. *A mypy plugin* could synthesize the view types — the house has no
     plugin, it would bind us to mypy internals, and pyright users lose
     everything. Rejected.
  3. *`m.i` typed as the bundle itself* (`TIn`) gives field-name checking but
     makes every `.transport = …` deployment line a type error — the sketch's
     wiring API stops typechecking. Rejected.
- **Adopted:** per-module-exact views + `__getattr__ → Port[Any]` +
  **runtime name validation in the rim** (§5.4) — the same
  static-hole/runtime-backstop trade the whole design already prices for
  def-site errors (T3). The hole is *pinned as a fixture* so any future
  improvement flips a marker consciously.
- **Blast radius:** T8 (must validate names, may later tighten
  `transport`/`source` types), T12 docs (teach `m.i.<field>` is
  runtime-checked), sketch §deployment unaffected (all lines typecheck).
- **Escalation:** if per-field static precision is ever demanded, the
  decision point is the bundle-spelling change (candidate 1) — an
  architecture-session call, since it moves the API floor itself.

No other settled decision was deviated from.

## 12. Acceptance criteria

- [x] `uv run mypy dimos/pure/typing.py` clean (strict, standalone).
- [x] Recursive `uv run mypy dimos/pure` clean — harness invisible to gate.
- [x] `uv run pytest dimos/pure/test_typing_static.py -q` — 19 passed
      (< 1 s single-proc; green under `-n 2`).
- [x] All four shapes + `-> Out | None` + `Never` corner infer per §2/§8.
- [x] Overload-order canaries in place for `over()` AND `m.o`; mutation-tested.
- [x] LSP override, structural twin, variance, arity cases pin exact codes.
- [x] Tagger floor fixture: zero added annotations, full inference.
- [x] T1/T2 shared seed fixtures present with adoption notes.
- [x] T3 coordination decisions (a)/(b)/(c) recorded and, where static,
      pinned as fixtures.
- [x] No new dev dependency; no pyproject.toml change.
- [ ] T6 replaces `over()` body per §4.4 (keeps every harness case green).
- [ ] T8 implements accessor `__get__`/views per §5.5 (adds runtime port
      tests; static cases unchanged).
- [ ] T2 makes `PureModule` inherit `EngineSurface` (§3) and re-exports §6
      names from `dimos/pure/__init__.py`.

## 13. Open questions (non-blocking)

1. `Streamable` final bound (T6): stamped-msg iterable vs memory2
   `StreamAccessor` protocol — decide when `run_over` exists; alias swap is
   one line here.
2. `OutPort.subscribe` return type: rim subscription handle
   (unsubscribe callable? context manager?) — T8's call; `Any` until then.
3. Whether `pm` re-exports the port types at all, or keeps them
   qualified-only — T12 docs call.
4. mypy version bumps may reword messages; markers pin codes + substrings to
   absorb that, but revealed-type spellings are exact — if a bump reformats
   them (rare), the fix is mechanical marker updates in one commit.
