# T1 — Row bundles: implementation spec

Target: `dimos/pure/rows.py` (+ `dimos/pure/test_rows.py`). Source of truth:
`dimos/memory2/puremodule_api_sketch3.py` and `tasks/index.md` §T1. Every
behavior below was verified against a working prototype under the repo mypy
config (mypy 1.19.0, `--strict`, `python_version = 3.12`) and CPython 3.12;
the design contains no speculative static behavior. Python floor is 3.10:
`dataclass_transform` comes from `typing_extensions`.

**No relitigation.** Everything here fits the settled decisions. Judgment
calls made within T1's mandate are numbered D1–D11 in §11 for review.

---

## 1. Scope and layer boundary

`rows.py` is the data model for tick rows and nothing else:

- `In` / `Out` bases (subclass applies machinery, no decorator).
- Field specifiers `tick()`, `latest()`, `interpolate()`, `contract()`.
- `FieldSpec` introspection model + `BundleCls.fields()`.
- `UNSTAMPED` sentinel and `BundleDefinitionError`.

Hard boundary: **no engine, no streams, no sampling logic, no I/O**. Imports
are stdlib + `typing_extensions` only — zero imports from anywhere in
`dimos`, including `dimos.pure` siblings. Specifiers *declare* policy
(`expect_hz`, `min_hz`, defaults); interpreting it is T5/T6/T9's job.
Stamping *ownership* is T5/T6's (§4.4); T1 only makes both stamped and
unstamped rows representable.

Downstream consumption map (who reads what — all "provided by" other tasks):

| Consumer | Uses from T1 |
| --- | --- |
| T2 config | must *skip* nested `_Bundle` subclasses when collecting config fields |
| T3 validation | resolves step hints to bundle classes; bundle internals stay opaque to it (orchestrator amendment: T3 must NOT call `fields()` in `__init_subclass__` — that would eagerly resolve field types and break the forward-reference case §5.2 exists for; resolution errors surface at wiring, T5) |
| T4 typing | bundles are ordinary classes in protocol params (`Stateless[C.In, C.Out]`); nothing else needed |
| T5 alignment | `In.fields()` → tick/latest/interpolate specs + defaults; constructs `InCls(ts=tick_ts, **resolved)` |
| T6 drivers | stamps step Out rows via `dataclasses.replace(row, ts=tick_ts)`; validates fold self-stamped ts (`UNSTAMPED` + monotonicity) |
| T8 rim | port handles iterate `fields()` (ts excluded ⇒ no ts port) |
| T9 health | reads `ContractSpec.min_hz`, `TickSpec.expect_hz` via `fields()` |
| T11 tf | extends the specifier set via the extension contract (§5.4) |

## 2. Public API

Everything below ships in `dimos/pure/rows.py` exactly as spelled (bodies
elided). `__all__ = ["In", "Out", "tick", "latest", "interpolate",
"contract", "FieldSpec", "TickSpec", "LatestSpec", "InterpolateSpec",
"ContractSpec", "PlainSpec", "UNSTAMPED", "BundleDefinitionError"]`.

```python
from __future__ import annotations

import dataclasses
from dataclasses import MISSING
from typing import Any, ClassVar, Final, Literal, TypeVar, overload

from typing_extensions import dataclass_transform

_T = TypeVar("_T")

UNSTAMPED: Final[float] = float("-inf")


class BundleDefinitionError(TypeError):
    """Raised at class-definition time for a malformed In/Out bundle."""


@dataclasses.dataclass(frozen=True)
class FieldSpec:
    """Introspection record for one declared bundle field."""

    kind: ClassVar[str]                      # subclass sets: "tick" | "latest" | ...
    side: ClassVar[Literal["in", "out"]]     # which bundle side the specifier is legal on

    name: str = ""                           # field name (filled by fields())
    annotation: Any = None                   # resolved type (filled by fields())
    default: Any = dataclasses.field(default_factory=lambda: MISSING)

    @property
    def required(self) -> bool: ...          # default is MISSING


@dataclasses.dataclass(frozen=True)
class TickSpec(FieldSpec):
    kind = "tick"
    side = "in"
    expect_hz: float | None = None


@dataclasses.dataclass(frozen=True)
class LatestSpec(FieldSpec):
    kind = "latest"
    side = "in"


@dataclasses.dataclass(frozen=True)
class InterpolateSpec(FieldSpec):
    kind = "interpolate"
    side = "in"


@dataclasses.dataclass(frozen=True)
class ContractSpec(FieldSpec):
    kind = "contract"
    side = "out"
    min_hz: float = 0.0                      # always set by contract(); default is inert


@dataclasses.dataclass(frozen=True)
class PlainSpec(FieldSpec):
    kind = "plain"
    side = "out"


def tick(*, expect_hz: float | None = None) -> Any:
    """Declare the In trigger field: a new message here fires the tick."""


@overload
def latest() -> Any: ...
@overload
def latest(*, default: _T) -> _T: ...
def latest(*, default: Any = MISSING) -> Any:
    """Declare an In field resolved to the newest message at tick time."""


@overload
def interpolate() -> Any: ...
@overload
def interpolate(*, default: _T) -> _T: ...
def interpolate(*, default: Any = MISSING) -> Any:
    """Declare an In field interpolated to the tick timestamp."""


@overload
def contract(*, min_hz: float) -> Any: ...
@overload
def contract(*, min_hz: float, default: _T) -> _T: ...
def contract(*, min_hz: float, default: Any = MISSING) -> Any:
    """Declare an Out field carrying a minimum-cadence contract."""


@dataclass_transform(
    kw_only_default=True,
    frozen_default=True,
    field_specifiers=(tick, latest, interpolate, contract),
)
class _Bundle:
    """Shared machinery for In/Out row bundles."""

    __bundle_side__: ClassVar[Literal["in", "out"]]
    __pure_own_specs__: ClassVar[dict[str, FieldSpec]]   # per-class raw spec table

    def __init_subclass__(cls, **kwargs: object) -> None: ...

    @classmethod
    def fields(cls) -> dict[str, FieldSpec]: ...


class In(_Bundle):
    """Base for input row bundles; ts is the tick time, required at construction."""

    __bundle_side__: ClassVar[Literal["in", "out"]] = "in"
    ts: float


class Out(_Bundle):
    """Base for output row bundles; ts stays UNSTAMPED until the engine stamps it."""

    __bundle_side__: ClassVar[Literal["in", "out"]] = "out"
    ts: float = UNSTAMPED
```

Notes on the surface:

- The specifier functions are the API for creating specs; direct `TickSpec(...)`
  construction is legal but not documented usage. Specifier arguments are
  keyword-only (matches every sketch call site).
- `tick()` deliberately has **no** `default=` — a trigger with a fallback is
  meaningless (no message ⇒ no tick). Statically this makes
  `tick(default=3)` a mypy `[call-arg]` error (verified), and at runtime a
  `TypeError`.
- `latest()` / `interpolate()` with no default declare a **required** field
  (engine holds ticks until resolvable — T5); `default=` makes it optional.
  `contract(min_hz=...)` is required unless `default=` is given (sparse-with-
  contract stays expressible; the sketch's `tf_out` composition needs the
  same shape later).
- `contract(min_hz=)` is mandatory in `contract()` — a contract without a
  cadence is not a contract; if you don't want one, use a bare annotation or
  a plain default (§6.2).
- `_Bundle` is private. Public bases are `In`/`Out` (exported as `pm.In` /
  `pm.Out` by the package surface — provided by T2's `__init__.py`).

## 3. Static contract (`dataclass_transform`)

The exact spelling, on the shared private base so both `In` and `Out` (and
every user bundle below them) inherit transformed semantics:

```python
@dataclass_transform(
    kw_only_default=True,
    frozen_default=True,
    field_specifiers=(tick, latest, interpolate, contract),
)
class _Bundle: ...
```

- `field_specifiers=(tick, latest, interpolate, contract)` is the whole
  point: mypy models a specifier call **without** a `default=` argument as a
  field with no default ⇒ REQUIRED in the synthesized constructor. Tests
  hand-construct rows; omitting a required field must be a static error.
- `kw_only_default=True`: every field is keyword-only in the synthesized
  `__init__`. This is what makes "defaulted `ts` on `Out`, required payload
  fields after it" legal, and why field order can never break construction
  (index watch-out).
- `frozen_default=True`: mypy rejects any assignment to row attributes.
- The overloaded specifiers return `_T` when `default=` is passed, so the
  default's type is checked against the field annotation (same trick as
  typeshed's `dataclasses.field`).

Verified static behaviors (mypy 1.19.0 strict; these become the T4/T12
regression cases, §9.2):

| Case | mypy result |
| --- | --- |
| omit required specifier field (`Tagger.In(ts=0, pose=p)`) | `[call-arg]` Missing named argument "image" |
| omit required bare Out field | `[call-arg]` |
| wrong payload type (`image=5`) | `[arg-type]` |
| unknown kwarg | `[call-arg]` Unexpected keyword argument |
| positional construction (`Tagger.In(0.0, img)`) | `[misc]` Too many positional arguments |
| omit `ts` on an In bundle | `[call-arg]` Missing named argument "ts" |
| omit `ts` on an Out bundle | clean |
| explicit `ts=` on an Out bundle (fold style) | clean |
| omit `latest(default=None)` / plain `= None` field | clean |
| `row.ts = 1.0` (any field write) | `[misc]` Property "ts" … is read-only |
| `x: int = latest(default="s")` | `[assignment]` |
| `x: int = tick(default=3)` | `[call-arg]` |
| inherited bundle: subclass ctor requires base + new fields | verified |
| `i.ts` reveals `float`; `i.image` reveals payload type | verified |
| `dataclasses.replace(row, ts=5.0)` reveals the row type | verified |
| `dataclasses.asdict(row)` reveals `dict[str, Any]` | verified |
| `Bundle.fields()` reveals `dict[str, FieldSpec]` | verified |

The last three matter downstream: transform-marked rows satisfy typeshed's
`DataclassInstance` protocol, so T6's stamping path (`dataclasses.replace`)
and codec paths type-check with no casts *on rows*. (Engine helpers that
take a bare `type` and call `dataclasses.fields(cls)` on it will need a
`TYPE_CHECKING`-guarded `cast("type[DataclassInstance]", cls)` — a known
typeshed wart, noted for the implementation.)

## 4. Runtime semantics

### 4.1 What subclassing generates

`_Bundle.__init_subclass__` runs `_process(cls)` on every subclass —
including the `In`/`Out` roots themselves (root path) and bundles nested
inside module class bodies (nesting is placement, not mechanism). After
processing, a bundle is a **real stdlib dataclass**:

```python
dataclasses.dataclass(frozen=True, kw_only=True, eq=True, repr=True)(cls)
```

applied **in place** (no slots ⇒ `dataclass()` mutates and returns the same
class object, so nested-class identity like `Tagger.In` is preserved).
Parameters, decided:

- `kw_only=True` — see §3. All fields, all layers.
- `frozen=True` — rows fan out to multiple consumers in a graph; step must
  not be able to mutate its input row; "Out is constructed, not written" is
  doctrine. The engine stamps via `dataclasses.replace` (§4.4), so frozen
  costs the engine one small allocation per tick — payloads dwarf it.
  Runtime mutation raises `dataclasses.FrozenInstanceError`.
- `eq=True` — replay-identity and golden-run tests compare rows
  (T5 determinism property: same inputs ⇒ identical rows). Equality is
  field-tuple equality and requires the same class. Caveat, documented: a
  payload type whose `__eq__` returns a non-bool (e.g. raw ndarray) breaks
  row equality — msg types must define scalar equality; that is a msg-layer
  obligation, not policed here.
- `repr=True` — debugging/inspector; shows every field including `ts`.
- `order=False`, `unsafe_hash=False` — rows are not ordered; `frozen + eq`
  auto-generates `__hash__` (hashable iff all payloads are; calling `hash()`
  on a row with unhashable payloads raises `TypeError`, same as tuples).
- **No `slots=True`** — `dataclass(slots=True)` must build a *replacement*
  class, which `__init_subclass__` cannot substitute (and a metaclass could,
  at exactly the metaclass-fight cost T2 forbids for modules). Rows are
  payload-dominated; the per-row dict overhead is noise. Revisit only with
  measurements, as a whole-layer change.

There is **no runtime type validation** of field values (no pydantic, no
`__post_init__`): rows are dumb data on the hot path; mypy is the checker
for rows, pydantic is for config (T2). `x: int = latest(default="s")` is a
static error but constructs at runtime — intended division of labor.

### 4.2 Processing pipeline (normative)

`_process(cls)` in order; every failure raises `BundleDefinitionError`
(§7) at class-definition time, i.e. at import of the defining module:

1. **Root?** If `"__bundle_side__" in cls.__dict__` (only the `In`/`Out`
   roots assign it in their own bodies), apply the dataclass call, set an
   empty own-spec table, return. Roots get no further validation; `ts` is
   defined here (required on `In`, `= UNSTAMPED` on `Out`).
2. **Side.** `getattr(cls, "__bundle_side__", None)` — inherited `"in"` or
   `"out"`; `None` ⇒ E-SIDELESS (someone subclassed `_Bundle` directly).
3. **Both roots?** `issubclass` of both `In` and `Out` ⇒ E-BOTH.
4. **Foreign dataclass bases.** Any MRO entry that `dataclasses.is_dataclass`
   but is not a `_Bundle` subclass ⇒ E-FOREIGN. (Fields from non-bundle
   dataclasses would bypass the spec table.)
5. **Generic?** `cls.__parameters__` non-empty ⇒ E-GENERIC. Rows are
   concrete serializable data; shapes share bundles by inheritance.
6. **Per-annotation checks** over `cls.__dict__["__annotations__"]` (own
   annotations only; annotations are strings under
   `from __future__ import annotations` and are *not* evaluated here):
   reserved names (`ts`, `fields`) ⇒ E-RESERVED; textual `"InitVar"` in the
   annotation ⇒ E-INITVAR; class-dict value is a `property` ⇒ E-PROPERTY;
   value is a raw `dataclasses.Field` ⇒ E-RAWFIELD.
7. **Specifier/annotation pairing.** Every `FieldSpec` instance in
   `cls.__dict__` must have a matching own annotation ⇒ else E-NOANN.
8. **Normalization** (this is how specifier objects die before instances
   exist): for each specifier-valued attribute —
   - required (`spec.default is MISSING`): `setattr(cls, name,
     dataclasses.field())` — a fresh default-less `Field` marker. Not
     `delattr`: a subclass *redeclaring* an inherited defaulted field as
     required must not let `dataclass()` find the ancestor's class-attribute
     default through the MRO. (Verified: redeclare-as-required works.)
   - defaulted: `setattr(cls, name, spec.default)` — the plain default
     value. Mutable defaults (`list`/`dict`/`set`) are rejected by stdlib
     `dataclass()` itself with its own `ValueError`; deliberately not
     wrapped (message already names the field), and there is no
     `default_factory` spelling in v1 (D10).
9. **Dataclassify** (the call in §4.1). Stdlib semantics from here:
   synthesized `__init__` (required fields have no class attribute at all
   afterwards — `dataclass` deletes the `Field` markers), `__eq__`,
   `__repr__`, `__hash__`, `FrozenInstanceError` on mutation, inherited
   fields accumulated MRO-wise.
10. **Leftover check.** Specifier names that did not become dataclass fields
    (ClassVar-annotated, or any other fieldness escape — `dataclass()` is
    the fieldness authority) ⇒ E-NOTFIELD.
11. **Own spec table.** For each `dataclasses.fields(cls)` entry with an own
    annotation, excluding `ts`: specifier present ⇒ check `spec.side`
    matches the bundle side (E-SIDE) and record it; no specifier ⇒ on an In
    bundle E-PLAIN-IN, on an Out bundle record
    `PlainSpec(default=<field default or MISSING>)`. Store as
    `cls.__pure_own_specs__` (per-class, own fields only).
12. **Merged validation.** Merge own-spec tables over `reversed(cls.__mro__)`
    (nearest-ancestor-wins on redeclaration, base-first ordering). More than
    one `TickSpec` in the merged table ⇒ E-MULTITICK. Zero ticks is legal
    here — at-least-one-trigger is a *resolution* requirement enforced by
    T5 when the bundle is actually aligned (D5).
13. **Leak sweep.** Any `FieldSpec` instance still present in
    `cls.__dict__.values()` ⇒ E-LEAK (internal invariant backstop).

### 4.3 Specifier objects never survive

Class-side: step 8 replaces every specifier value with either a `Field`
marker (later deleted by `dataclass()`) or the plain default; step 13
asserts none remain. Instance-side: `__init__` only ever assigns
constructor arguments and defaults, so `vars(row)` contains exactly the
declared fields plus `ts` and nothing else. The raw specs live only in
`cls.__pure_own_specs__` (and the resolved cache, §5.2). Test-enforced
(§9.1 `test_no_specifier_leakage`).

### 4.4 `ts` — the engine-stamped timestamp

`ts: float`, keyword-only like everything else, **not** a port (excluded
from `fields()`, so T8 creates no handle for it). Float seconds; one
timestamp authority = payload ts (T5).

- **In: required.** The engine (T5 alignment) stamps at construction —
  `InCls(ts=tick_ts, **resolved)`; there is no post-hoc stamping of In
  rows. Hand-construction in tests must pass it:
  `Tagger.In(ts=0.0, image=..., pose=...)` — exactly the spelling of both
  sketches' test sections, verbatim. Justification: every sketch
  hand-construction passes `ts=` explicitly; step logic reads `i.ts`
  (RelocalizationModule's throttle), so a forgotten `ts` in a test must be
  a loud error, not a silent sentinel.
- **Out: defaults to `UNSTAMPED = float("-inf")`.** step constructs
  `Tagger.Out(located=...)` with no `ts` — the sketch floor — and the step
  driver (T6) stamps the returned row with the tick ts via
  `dataclasses.replace(row, ts=tick_ts)`. fold self-stamps
  (`VoxelGridMapper.Out(ts=r.ts, ...)`, sketch §7) and the fold driver (T6)
  validates strictly-increasing ts. `ts` stays typed `float` everywhere —
  no `Optional` pollution downstream (`row.ts = tick time` in consumer
  loops).
- **Why `-inf`, not NaN or `None` (D1):** `None` would make every
  downstream `row.ts` read `float | None`. NaN breaks equality
  (`NaN != NaN`), so two identical unstamped rows — e.g.
  `step(row_in) == Tagger.Out(located="x")` in a test — would compare
  unequal; `-inf == -inf` holds and unstamped rows compare naturally.
  Bonus: a fold that *forgets* to stamp emits `-inf`, which any
  strictly-increasing validation catches for free (`-inf` is never >
  anything, including the driver's `-inf` initial cursor with a strict
  comparison). `-inf` is never a legitimate row time.
- A step module that passes `ts=` explicitly is doctrinally confused; the
  step driver overwrites regardless (recommendation to T6: overwrite, don't
  error — `ts` is engine-owned for step modules, and a per-tick check buys
  nothing).

## 5. `FieldSpec` and `fields()`

### 5.1 Model

One frozen dataclass per specifier kind, subclassing `FieldSpec` (§2). Why
subclasses instead of a kind-tagged single class (D7): typed per-kind
params (`expect_hz` only on `TickSpec`, `min_hz` only on `ContractSpec`),
`isinstance` dispatch in T5, and extensibility — T11 adds tf specs without
touching the base model. `kind` and `side` are `ClassVar`s (deliberately
not fields — they describe the spec class, not the instance).
`spec.required` ⇔ `spec.default is dataclasses.MISSING` —
`dataclasses.MISSING` is *the* no-default sentinel (D9; no second MISSING
invented; inside the frozen `FieldSpec` dataclass it must be spelled via
`default_factory=lambda: MISSING`, since a literal `MISSING` default would
read as "no default" to `dataclass()` itself).

### 5.2 `fields() -> dict[str, FieldSpec]`

- Returns a **fresh dict copy** per call (mutation-safe); the `FieldSpec`
  values are immutable and shared.
- **Excludes `ts`** (D6): `fields()` enumerates *declared data fields* — the
  ports. `ts` is row infrastructure with no port, no sampler, no contract.
  The physical layout including `ts` is `dataclasses.fields(cls)` — rows are
  real dataclasses; codecs use that.
- **Ordering guaranteed**: declaration order, base bundle first, then
  subclass additions; a redeclared field keeps its original position
  (dict-update semantics, matching `dataclasses.fields`). T5's equal-ts
  tiebreak ("port declaration order") depends on this guarantee.
- **Lazy resolution, cached**: the first call runs
  `typing.get_type_hints(cls, include_extras=True)` and builds finalized
  specs — `dataclasses.replace(raw, name=name, annotation=resolved)` — then
  caches the table in `cls.__dict__` (per-class cache; subclasses resolve
  their own). Laziness is required: bundles may reference types defined
  later in their module (string annotations under
  `from __future__ import annotations`). Resolution *failure is not
  cached* — a later call may succeed once the name exists. Racing first
  calls are benign (idempotent construction, last write wins with equal
  content).
- `get_type_hints` resolves each MRO class against its own defining
  module's globals, so cross-module bundle inheritance resolves each layer
  correctly.
- Resolution failure raises `BundleDefinitionError` (E-UNRESOLVED, chained
  from the underlying `NameError`) with the qualified-path hint (§6.4).
- On the roots: `In.fields() == Out.fields() == {}`.
- `annotation` holds exactly what `get_type_hints` returns (unions as
  formed, `Annotated` extras preserved via `include_extras=True`).

### 5.3 What each spec means (declared here, interpreted elsewhere)

| Spec | Side | Params | Declares |
| --- | --- | --- | --- |
| `TickSpec` | In | `expect_hz: float \| None` | the trigger field; expected input rate (health hint, T9) |
| `LatestSpec` | In | `default` | resolve to newest message at tick ts; required ⇒ hold tick until history exists (T5) |
| `InterpolateSpec` | In | `default` | interpolate to tick ts; type-specific interpolation is T5's registry, not T1's concern |
| `ContractSpec` | Out | `min_hz: float`, `default` | output cadence obligation (measured by T9) |
| `PlainSpec` | Out | `default` | no engine policy; `default=None` fields are the sparse ports (None at runtime = silent this tick, a port never carries None — T6/T8) |

Specifier argument validation happens at *call* time with plain
`ValueError` (§7 bottom): `expect_hz`/`min_hz` must be > 0 when given.

### 5.4 Extension contract (for T11's `tf()` / `tf_out()`)

New specifier kinds plug in without redesign, but **must touch rows.py in
exactly one place**:

1. Define a `FieldSpec` subclass with `kind`/`side` ClassVars and its
   params (frame names/templates are plain data — zero engine deps, so tf
   spec classes qualify for the data layer).
2. Define the constructor function and **append it to the
   `field_specifiers` tuple** in the `dataclass_transform` call. This is
   mandatory: an unlisted specifier call is treated by mypy as a plain
   default value, silently making required tf fields constructor-optional.
3. Side legality and the rest of the pipeline then apply automatically
   (an In-side `kind="tf"` spec passes the In sampler requirement because
   the requirement is "has an In-side spec", not a hardcoded kind list).

## 6. Bundle rules

### 6.1 Naming and nesting

- In and Out may share field names freely (separate classes; `m.i.x` /
  `m.o.x` disambiguate at the rim — T8).
- Bundles nested inside a module class body behave identically to
  module-level ones: the machinery never keys on `__name__`/`__qualname__`
  (names appear only in error messages and repr). No registry, no
  name-based magic. Nested bundles pickle by module + qualname path
  (verified), so `Tagger.In` instances round-trip.
- The class need not be named `In`/`Out` — a shared module-level
  `ScanRow(pm.In)` is fully supported (nesting is placement, not
  mechanism).

### 6.2 Field declarations

| Declaration | In bundle | Out bundle |
| --- | --- | --- |
| `x: T = tick()/latest()/interpolate()` | field (required unless `default=`) | E-SIDE |
| `x: T = contract(min_hz=...)` | E-SIDE | field (required unless `default=`) |
| `x: T` (bare annotation) | E-PLAIN-IN | required field, no contract |
| `x: T = value` (plain default) | E-PLAIN-IN | optional field (`= None` ⇒ sparse port) |
| `x: ClassVar[T] = value` | class constant, not a field | same |
| unannotated assignment / `def` / `@property` (unannotated) | not a field (helpers allowed) | same |

Rationale for the asymmetry (D4): every In field is *resolved by the
engine* and therefore needs a sampling policy; sparse/plain is an
*emission* semantic and belongs to Out. In-side optionality is spelled
`latest(default=None)` / `interpolate(default=None)`.

### 6.3 Inheritance

- Shape subclasses extend bundles: `class Ext(CostMapper.In): extra: float
  = latest()` — constructor takes base + new fields; `fields()` merges
  base-first.
- Redeclaration: a subclass may redeclare an inherited field with a new
  spec; nearest ancestor wins, position preserved, required-over-defaulted
  works (§4.2 step 8).
- Multiple bundle inheritance is allowed (C3 merge); the merged table is
  validated (e.g. two inherited ticks ⇒ E-MULTITICK at the subclass).
- Forbidden: mixing In and Out lineages, non-bundle dataclass bases,
  `Generic` parameters (§4.2 steps 3–5).

### 6.4 Field types must be runtime-importable

Cross-cutting rule, and mechanically required: `fields()` resolves
annotations with `get_type_hints`, so every field type must be resolvable
**at module scope** of the defining module (TYPE_CHECKING-only imports of
payload types are a sketch liberty, not a pattern). Corollary for nested
bundles: a field whose type is another class's nested bundle must be
spelled by qualified path (`x: Tagger.Out = latest()`), never by bare
sibling name from inside the same outer class body — class bodies are not
scopes that annotation resolution can see. The E-UNRESOLVED message
teaches this.

### 6.5 Serialization guarantees ("recording ticks = serializing rows")

T1 ships no codec; it guarantees the representation codecs need:

1. Rows are plain stdlib dataclasses: `dataclasses.fields(row)`,
   `asdict`, `astuple`, `replace` all work; no `__post_init__`, no hidden
   construction side effects, `__init__` is exactly the synthesized one.
2. Instance state is exactly the declared fields plus `ts` — nothing else
   in `vars(row)`, ever (no cached properties, no back-references, no spec
   objects).
3. The class is recoverable by reference: `(cls.__module__,
   cls.__qualname__)` resolves to the class, including nested bundles ⇒
   pickle round-trips (verified) and by-reference codecs work.
4. The schema is derivable: `fields()` gives name → (kind, params,
   resolved type, default) in a deterministic order; `ts` is uniformly
   `float`.

Nothing in the layer holds state that isn't in the row; nothing
requires an engine to construct a row. What T1 does *not* guarantee:
that payloads themselves are serializable (a `Callable` field is legal
data-model-wise and the codec's problem later).

## 7. Error behavior

One exception class: `BundleDefinitionError(TypeError)`, raised at class
definition (import time of the defining module) or, for E-UNRESOLVED, at
first `fields()` resolution. Every message is prefixed with the bundle's
`f"{cls.__module__}.{cls.__qualname__}"` (nested classes read
`mymod.Tagger.In`). Templates (normative; `{bundle}` = that prefix):

| Code | Condition | Message template |
| --- | --- | --- |
| E-SIDELESS | direct `_Bundle` subclass | `{bundle}: bundles must subclass pm.In or pm.Out` |
| E-BOTH | inherits In and Out | `{bundle}: a bundle cannot inherit from both In and Out` |
| E-FOREIGN | non-bundle dataclass base | `{bundle}: base {base!r} is a dataclass but not a bundle; bundles may only inherit fields from other In/Out bundles` |
| E-GENERIC | `Generic[...]` bundle | `{bundle}: generic bundles are not supported; rows are concrete data (share fields via bundle inheritance instead)` |
| E-RESERVED | field named `ts`/`fields` | `{bundle}: field name {name!r} is reserved (row infrastructure: ['fields', 'ts'])` |
| E-INITVAR | `InitVar` annotation | `{bundle}: field {name!r} is an InitVar; bundles are plain rows and take no init-only parameters` |
| E-PROPERTY | annotated field whose value is a `property` | `{bundle}: {name!r} is annotated as a field but its value is a property; bundles are plain data` |
| E-RAWFIELD | value is `dataclasses.field(...)` | `{bundle}: {name!r} uses dataclasses.field(); use a field specifier (tick()/latest()/interpolate()/contract()) or a plain default instead` |
| E-NOANN | specifier without annotation | `{bundle}: {name!r} is assigned a field specifier but has no field annotation (write \`{name}: <Type> = ...\`)` |
| E-NOTFIELD | specifier on a non-field (ClassVar etc.) | `{bundle}: {names} carry field specifiers but are not fields (ClassVar-annotated?); specifiers are only valid on data fields` |
| E-PLAIN-IN | In field without specifier | `{bundle}: In field {name!r} needs a sampler specifier (tick()/latest()/interpolate()); plain defaults are only valid on Out bundles` |
| E-SIDE | wrong-side specifier | `{bundle}: {kind}() is not valid on an {In\|Out} bundle field ({name!r})` |
| E-MULTITICK | >1 tick in merged fields | `{bundle}: multiple tick() fields ({names}); an In bundle declares at most one trigger` |
| E-LEAK | internal invariant backstop | `{bundle}: specifier object survived as class attribute {name!r}` |
| E-UNRESOLVED | `get_type_hints` failure in `fields()` | `{bundle}: cannot resolve field types ({exc}); field types must be importable at module scope — reference nested classes by qualified path (e.g. 'Tagger.Out')` (chained `from exc`) |

Not wrapped (deliberately, stdlib message is already precise and named):

- Constructor misuse — missing required field, unknown kwarg, positional
  args — plain `TypeError` from the synthesized `__init__` (and a mypy
  error first, §3).
- Field assignment — `dataclasses.FrozenInstanceError`.
- Mutable plain default (`tags: list[str] = []`) — stdlib `ValueError`
  ("mutable default … use default_factory" — and we don't offer
  default_factory, see D10; the fix is to not put mutable defaults on
  rows).
- Specifier argument validation — `ValueError` at the call site:
  `tick(): expect_hz must be > 0, got {v}` /
  `contract(): min_hz must be > 0, got {v}`.

## 8. Edge cases — enumerated and resolved

Index.md T1 watch-outs first, then the rest found during design. Each maps
to a spec section and/or a named test (§9).

| # | Edge case | Resolution | Where |
| --- | --- | --- | --- |
| 1 | specifier-with-no-default must be constructor-REQUIRED under mypy | `field_specifiers=` + verified | §3, S1 |
| 2 | specifier objects surviving as instance values | normalize + leak sweep + test | §4.3, R-leak |
| 3 | `ts` kw-only so field order never breaks | `kw_only=True` everywhere | §3, §4.4 |
| 4 | In/Out sharing field names | separate classes, no registry | §6.1, R-samename |
| 5 | nested ≡ module-level bundles | no name assumptions; pickle by qualname | §6.1, R-nested |
| 6 | "recording ticks = serializing rows" must stay possible | four representation guarantees | §6.5, R-serial |
| 7 | Out hand-construction without ts vs fold self-stamp | `ts = UNSTAMPED` default; drivers stamp/validate | §4.4, R-ts |
| 8 | unstamped-row equality (sentinel poison) | `-inf`, not NaN | D1, R-eq |
| 9 | redeclaring inherited defaulted field as required | `dataclasses.field()` normalize trick | §4.2#8, R-redecl |
| 10 | duplicate tick via inheritance merge | merged-table check | §4.2#12, R-2tick |
| 11 | zero-tick In bundle | legal at T1; T5 errors at resolution | D5 |
| 12 | forward refs / types defined later in module | lazy `fields()`, failure not cached | §5.2, R-fwd |
| 13 | bare sibling-name annotation in nested bundle | E-UNRESOLVED with qualified-path hint | §6.4, R-qualpath |
| 14 | ClassVar constants and methods in bundle bodies | not fields; allowed | §6.2, R-helpers |
| 15 | specifier on ClassVar / no annotation / property / raw `field()` / InitVar | definition errors | §7, R-errors |
| 16 | mutable plain default | stdlib ValueError, unwrapped | §7, R-mutable |
| 17 | generic bundles, foreign dataclass bases, In+Out mix, bare `_Bundle` | definition errors | §7, R-errors |
| 18 | payload types with array-like `__eq__` | documented msg-layer obligation | §4.1 |
| 19 | empty bundle (`class X(pm.In): pass`) | legal; `fields() == {}` | R-empty |
| 20 | future specifier kinds (tf) silently unmodeled by mypy | extension contract: must join `field_specifiers` tuple | §5.4 |
| 21 | `dataclasses.fields(cls)` on bare `type` in engine helpers | documented cast wart | §3 |

## 9. Test plan

Runtime tests live in `dimos/pure/test_rows.py` (excluded from the mypy
gate — fine, they're fixtures). Static cases are enumerated here;
**T4 owns the harness choice** (mypy-api vs pytest-mypy-plugins) — T1 just
fixes the case list. Fixture bundles reuse
the sketch shapes (Tagger/CostMapper/QualityGate) with tiny local payload
classes — zero engine imports, zero dimos imports (the tests-need-no-engine
property, enforced by construction).

### 9.1 Runtime unit tests

**The suite ships with this spec** as `dimos/pure/test_rows.py` — real
bodies, validated green (31 tests) against the design prototype. It is
skip-gated with a module-level
`pytestmark = pytest.mark.skip(reason="T1 skeleton — enable with implementation")`
so it collects cleanly against the skeleton; **the implementer deletes that
one line to activate it**. All fixture bundles are defined inside test
bodies (module-level bundles would run specifier calls at import time,
which raise in the skeleton).

Shipped tests:

| Test | Asserts |
| --- | --- |
| `test_construction_required_and_defaulted` | kw construction; values land; `latest(default=None)` and plain `= None` omissible |
| `test_constructor_type_errors` | missing required / unknown kwarg / missing In-ts ⇒ `TypeError` |
| `test_ts_kw_only` | positional construction rejected; defaulted `ts` before required fields is fine |
| `test_out_ts_unstamped_and_explicit` | `Out(...)` ⇒ `ts == UNSTAMPED`; explicit `ts=` honored (fold style) |
| `test_frozen` | field write ⇒ `FrozenInstanceError` |
| `test_eq_and_replace_stamping` | unstamped rows compare equal; hash consistent; `replace(row, ts=t)` stamps a new row, original untouched |
| `test_fields_introspection` | declaration order, `ts` excluded, kinds, `expect_hz`/`min_hz`/`default`/`required`/`name`, resolved annotations incl. `X \| None` |
| `test_fields_returns_fresh_copy` | mutating the returned dict doesn't corrupt the class |
| `test_fields_lazy_resolution_not_cached_on_failure` | unresolvable name ⇒ E-UNRESOLVED with "qualified path" hint; failure not cached — resolves once the name exists |
| `test_roots_and_empty_bundle` | zero-field bundle legal; roots constructible; `fields() == {}` |
| `test_no_specifier_leakage` | `vars(row)` == fields ∪ {ts}; no `FieldSpec` in instance or class attrs; required fields have no class attr |
| `test_asdict_and_dataclass_fields` | `asdict` content incl. ts; `dataclasses.fields` order `[ts, …]` |
| `test_nested_class_body_equivalence` | nested-in-class-body bundle ≡ flat bundle: same `fields()`, construction, behavior |
| `test_in_out_share_field_names` | same field name on an In and an Out coexists |
| `test_bundle_inheritance_extends` | `Ext(CostIn)` ctor takes base+new; merged `fields()` order |
| `test_redeclare_required_override` | defaulted→required redeclaration enforced at runtime + in `fields()`; position kept |
| `test_helpers_allowed` | ClassVar constants + methods usable, not fields |
| `test_error_specifier_without_annotation` | E-NOANN, names the field |
| `test_error_plain_field_on_in` | E-PLAIN-IN for plain default and bare annotation |
| `test_error_wrong_side_specifier` | E-SIDE both directions (contract-on-In, tick-on-Out) |
| `test_error_reserved_names` | E-RESERVED for `ts` and `fields` |
| `test_error_multiple_ticks` | E-MULTITICK direct + via inheritance merge |
| `test_error_in_out_mix_and_bare_bundle` | E-BOTH; E-SIDELESS |
| `test_error_foreign_dataclass_base` | E-FOREIGN |
| `test_error_generic_bundle` | E-GENERIC |
| `test_error_property_raw_field_initvar` | E-PROPERTY, E-RAWFIELD, E-INITVAR |
| `test_error_classvar_specifier` | E-NOTFIELD |
| `test_error_messages_name_the_bundle` | messages carry `module.qualname` prefix |
| `test_specifier_arg_validation` | `tick(expect_hz=0)`, `contract(min_hz=-1)` ⇒ `ValueError` |
| `test_mutable_default_rejected` | stdlib `ValueError` surfaces |
| `test_sketch_floor_tagger` | sketch-spelled nested `class In(In)` bundles construct exactly as sketch3's tests do (In with `ts=`, Out without) |

Added at implementation time (they need module-level fixture bundles,
impossible while skeleton specifiers raise): `test_nested_pickle_roundtrip`
(pickle/unpickle a bundle nested in a module-level class),
`test_cross_module_inheritance` (bundle extending a bundle from a second
fixture module; each MRO layer resolves against its own module globals),
and the positive qualified-path case (`x: Outer.Sib = tick()` resolves).
All three were verified against the prototype during design.

### 9.2 Static regression cases (mypy-level; harness by T4)

Case list = the §3 table, as concrete snippets: S1 required-specifier
omission `[call-arg]`; S2 required-bare omission `[call-arg]`; S3 wrong
type `[arg-type]`; S4 unknown kwarg `[call-arg]`; S5 positional `[misc]`;
S6 defaulted fields omissible (clean); S7 In-ts required `[call-arg]` /
Out-ts omissible / explicit Out-ts clean; S8 frozen write `[misc]`; S9
`default=` type mismatch `[assignment]`; S10 inheritance ctor
accumulation; S11 attribute types reveal correctly (`ts` → `float`,
payloads exact); S12 `replace`/`asdict` typing on rows; S13 `fields()` →
`dict[str, FieldSpec]`; S15 `tick(default=…)` `[call-arg]`.

## 10. Acceptance criteria

- [ ] `uv run mypy dimos/pure/rows.py` clean under repo strict config.
- [ ] rows.py imports stdlib + `typing_extensions` only; no `dimos.*`
      imports; no imports of `dimos.pure` siblings.
- [ ] The sketch3 §1/§2/§4 bundles (Tagger, QualityGate, CostMapper) compile
      against `pm.In`/`pm.Out` + specifiers verbatim — the Tagger stays four
      declarations, zero added ceremony.
- [ ] Sketch test spellings work verbatim: `Tagger.In(ts=0.0, image=…,
      pose=…)`, `Tagger.Out(located=…)`, `VoxelGridMapper.Out(ts=r.ts, …)`.
- [ ] `uv run pytest dimos/pure/test_rows.py --collect-only` green against
      the skeleton; after removing the module-level skip, all §9.1 tests
      pass, plus the three implementation-time additions.
- [ ] All §9.2 static cases behave as listed.
- [ ] All §7 error paths raise `BundleDefinitionError` with the templates.
- [ ] `fields()` guarantees hold: ts excluded, declaration order, fresh
      dict, lazy + cached + failure-not-cached resolution.
- [ ] Specifier objects never observable as class or instance attribute
      values after class creation.
- [ ] Nested-bundle pickle round-trip passes.
- [ ] No engine, stream, transport, or clock code anywhere in the module.

## 11. Decisions within mandate (numbered for review)

- **D1** `In.ts` required; `Out.ts` defaults to `UNSTAMPED = -inf` (§4.4).
- **D2** `dataclass(frozen=True, kw_only=True, eq=True, repr=True)`, no
  slots (§4.1).
- **D3** No runtime value validation on rows; mypy is the row checker
  (§4.1).
- **D4** In fields require an In-side specifier; bare/plain fields are
  Out-only (§6.2).
- **D5** At-most-one `tick()` enforced at definition; at-least-one deferred
  to T5 resolution (§4.2#12).
- **D6** `fields()` excludes `ts`, is declaration-ordered, returns a fresh
  dict, resolves lazily with per-class caching (§5.2).
- **D7** Spec-subclass-per-kind model with `kind`/`side` ClassVars (§5.1).
- **D8** `default=` on `latest`/`interpolate`/`contract`; never on `tick`
  (§2).
- **D9** `dataclasses.MISSING` is the no-default sentinel (§5.1).
- **D10** Strictness set: reserved `{ts, fields}`; no generics, foreign
  dataclass bases, raw `dataclasses.field`, `InitVar`, property-fields,
  specifier-on-ClassVar; no `default_factory` spelling in v1 (§4.2, §7).
- **D11** Stamping ownership: T5 constructs In stamped; T6 step driver
  stamps Out via `replace` (overwrite, don't error); fold self-stamps, T6
  validates strictly-increasing (§4.4).
