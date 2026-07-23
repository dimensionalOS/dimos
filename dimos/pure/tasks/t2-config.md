# T2 — Config machinery + PureModule constructor: implementation spec

Status: spec-ready. Source of truth: `dimos/memory2/puremodule_api_sketch3.py`
§5c (CONFIG), §5b (Go2Connection), §1 (Tagger), banner. Interop target:
`dimos/protocol/service/spec.py`. Deliverable files:
`dimos/pure/config.py`, `dimos/pure/module.py` (skeletons landed with this
spec), `dimos/pure/test_config.py` (real tests, module-skipped until
implementation — the skip line is `pytestmark = pytest.mark.skip(...)`;
the implementer deletes it).

Every behavioral claim below marked **[verified]** was empirically probed in
this design session against the exact toolchain: pydantic 2.12.5,
mypy 1.19.0 strict, python_version=3.12 (runtime floor 3.10 →
`dataclass_transform` imports from `typing_extensions`).

---

## 1. Scope and doctrine

T2 delivers the config half of PureModule:

- Flat annotated class fields are **the API**: typed constructor kwargs,
  `self.emit_every` reads throughout step.
- A synthesized per-subclass **frozen** pydantic model is **the substance**:
  runtime validation, canonical serialization, identity.
- `m.config.model_dump()` is THE canonical config serialization — memo keys,
  checkpoint identity (T10), inspector panels, sweeps all consume it.
- **Module identity = class + config.** Frozen is load-bearing; the idiom is
  rebuild-never-mutate (rebuilding is free — construction is validation plus
  attribute copies, no I/O, no resources touched).
- `warmup()/start()/stop()` exist with Service-compatible signatures;
  bodies are T8's (T2 ships no-op hooks).

Out of T2's scope (owned elsewhere, only *reserved* here): `In`/`Out` bases
and field specifiers (T1), `step`/`fold` discovery and classification (T3),
`over()`/protocols/`m.i`/`m.o` (T4/T6/T8), `@resource` (T7),
`checkpoint()/restore()` (T10).

API floor check: the canonical Tagger has zero config fields and stays four
declarations; `VoxelGridMapper` adds config as plain `voxel_size: float =
0.05` lines and nothing else. T2 adds **no** required ceremony to any sketch
module.

---

## 2. Public API surface

### 2.1 `dimos/pure/module.py`

```python
from dimos.pure.typing import EngineSurface   # T4 — binding final form (§10.2)

@dataclass_transform(kw_only_default=True, frozen_default=True)
class PureModule(EngineSurface):
    __pure_config_model__: ClassVar[type[PureModuleConfig]]

    def __init_subclass__(cls, **kwargs: object) -> None: ...
    def __init__(self, **kwargs: object) -> None: ...

    @property
    def config(self) -> PureModuleConfig: ...

    def __setattr__(self, name: str, value: object) -> None: ...  # raises
    def __delattr__(self, name: str) -> None: ...                 # raises
    def __eq__(self, other: object) -> bool: ...
    def __hash__(self) -> int: ...
    def __repr__(self) -> str: ...
    def __reduce__(self) -> tuple[Any, ...]: ...

    def warmup(self) -> None: ...   # Service interop; no-op until T8
    def start(self) -> None: ...    # Service interop; no-op until T8
    def stop(self) -> None: ...     # Service interop; no-op until T8

def _rebuild_module(cls: type[PureModule], dump: dict[str, Any]) -> PureModule: ...
```

PureModule deliberately defines **none of** `step`, `fold`, `In`, `Out`,
`State` — not even abstract placeholders. T3's discovery depends on their
absence from the base (binding seam constraint, §10.1), and T4 adds the
typing reason: a base-level `step` would make every subclass match the
step protocols with the *base's* types and destroy per-module `TIn`/`TOut`
inference. Likewise T2 must not redeclare `over`, `i`, or `o` — they live
on T4's `EngineSurface` base (§10.2).

PureModule's own body carries no plain annotated attributes (only the
`ClassVar`-annotated `__pure_config_model__`). Doubly safe: T1/T4 confirmed
annotations on the `dataclass_transform`-decorated class itself are not
fields anyway — but the rule for any future internal state on the base
stays "unannotated or dunder".

`@dataclass_transform` decorates the **class** (PEP 681 base-class form):
every subclass statically behaves like a `@dataclass(kw_only=True,
frozen=True)` — typed synthesized `__init__` per subclass, fields read-only.
**[verified]** A runtime `__init__(self, **kwargs: object)` on the base does
not suppress per-subclass synthesis (same trick pydantic itself uses):
typo kwargs error with did-you-mean, wrong types are `[arg-type]` errors,
positional calls are `Too many positional arguments`.

`frozen_default=True` is an addition relative to the design-session
experiment and is a pure win: mutation is now a **static** error too
(`Property "odom_timeout" defined in "Go2Connection" is read-only [misc]`
**[verified]**), matching the runtime guard in §6. It requires nothing from
subclasses.

`eq_default` is True by PEP 681 default; runtime `__eq__` (§9) makes it
true in fact.

### 2.2 `dimos/pure/config.py`

```python
RESERVED_CONFIG_FIELDS: Final[frozenset[str]]  # §3.3

class ConfigFieldError(TypeError): ...          # definition-time (§10, §11)
class FrozenModuleError(AttributeError): ...    # mutation-time (§6, §11)

class PureModuleConfig(BaseConfig):             # BaseConfig from protocol/service/spec.py
    model_config = ConfigDict(
        arbitrary_types_allowed=True,
        extra="forbid",
        frozen=True,
        validate_default=True,
        protected_namespaces=(),
    )

def _collect_config_fields(cls: type) -> dict[str, tuple[Any, Any]]: ...
def _synthesize_config_model(
    cls: type,
    fields: Mapping[str, tuple[Any, Any]],
    bases: tuple[type[PureModuleConfig], ...],
) -> type[PureModuleConfig]: ...
```

Layering: `config.py` imports pydantic and `dimos.protocol.service.spec`
only — never `module.py`, never T1/T3/T4 files (one exception: a *lazy,
function-local* import of `dimos.pure.rows` for the specifier guard in
§3.4, tolerant of the module not existing). `module.py` imports `config.py`.

### 2.3 Exports for the `pm` surface

`dimos/pure/pm.py` is owned by the surface integrator, not T2. When it
is assembled, T2 contributes: `PureModule`, `PureModuleConfig`,
`ConfigFieldError`, `FrozenModuleError`. (`RESERVED_CONFIG_FIELDS` stays
importable from `dimos.pure.config` but does not need to ride `pm.*`.)

---

## 3. Field collection: what is a config field

Collection runs per subclass on **that class's own body only** (base fields
arrive via config-model inheritance, §4.3). Inputs: `cls.__dict__` and
`cls.__dict__.get("__annotations__", {})`.

### 3.1 IS a field

A name is a config field iff it appears in the class body's
`__annotations__`, its resolved annotation is not `ClassVar[...]`, and it
passes the §3.3 rejection gauntlet.

- With an assigned value → field with that default (`prefix: str = ""`).
- Without an assigned value → **required** field (`robot_ip: str`), i.e. a
  required constructor kwarg. **[verified]** mypy: `Missing named argument
  "robot_ip" for "NeedsIp"`; runtime: pydantic `missing`.
- `Annotated[T, ...]` metadata is preserved (`include_extras=True`) and
  flows into pydantic untouched. Free consequence **[verified]**:
  `n: Annotated[int, Field(gt=0)] = 1` gains a runtime `gt` constraint
  while mypy sees plain `int`. This is passthrough, not core API — docs may
  mention it, nothing may depend on it.

### 3.2 is NOT a field (and why both checkers agree)

| member | runtime reason | static reason **[verified]** |
| --- | --- | --- |
| nested classes (`In`, `Out`, `State`, any) | class statements don't annotate | `Unexpected keyword argument "In"` |
| methods (`step`, helpers) | not annotated | `Unexpected keyword argument "step"` |
| decorated descriptors (`@resource`, `@property`, `cached_property`) | not annotated | `Unexpected keyword argument "grid"`; `c.grid` reveals the descriptor's `__get__` type |
| `ClassVar[...]` names | filtered by origin check | `Unexpected keyword argument "LIMIT"` |
| annotated attrs on **non-PureModule mixin** bases | mixins never pass through `__init_subclass__` | mypy synthesizes only from the transformed lineage: `Unexpected keyword argument "mixin_attr"` for both base orders |
| `__pure_step__` (set by T3's classify) | assigned unannotated, after collection, and dunder | never in the synthesized signature |
| `i` / `o` port accessors, `over` (T4's EngineSurface) | deliberately unannotated attrs on a base body collection never scans (and `__get__` descriptors besides); names also in `RESERVED_CONFIG_FIELDS` | T4-verified: its fixtures run the full transform stack over EngineSurface with no field capture |

The static and runtime rules are the *same rule* observed from two sides:
"annotated, non-ClassVar, in a PureModule-lineage class body". Divergence
cases that would break this equivalence are **rejected at class definition**
(§3.3) rather than allowed to disagree silently.

Mixin nuance: mypy leaves `m.mixin_attr = 9` legal (mixin attrs are not
frozen fields) while the runtime `__setattr__` guard (§6) still raises.
Runtime-stricter is the safe direction; documented, not "fixed".

### 3.3 Rejected at class definition — the gauntlet

All raise `ConfigFieldError` (§11 for exact messages), in this per-field
order after the `ClassVar` exemption:

1. **Leading underscore** (`_rate: float = 1.0`). pydantic silently treats
   underscore names as private non-fields **[verified]** while mypy treats
   them as fields — a silent static/runtime split, so it's an error.
   Internal constants spell `ClassVar`.
2. **Reserved machinery names** — `RESERVED_CONFIG_FIELDS = {"config",
   "step", "fold", "In", "Out", "State", "warmup", "start", "stop", "over",
   "checkpoint", "restore", "i", "o", "health"}`. Reserves future tasks'
   surface (T4/T6/T8/T10/T9) now. Note this only fires for *annotated*
   uses (e.g. `step: int = 3`); defining `def step(...)` is of course T3's
   normal path, not a field.
3. **pydantic namespace collision** — any name with
   `hasattr(pydantic.BaseModel, name)`: `model_dump`, `model_fields`,
   `dict`, `json`, `copy`, `schema`, `construct`, ... **[verified]**
   pydantic only *warns* and lets the field shadow the method
   (`model_dump='y'` was constructible!), which would corrupt the canonical
   surface — hard error at our layer. Ordinary `model_*` names that are
   not BaseModel attributes (`model_path`) remain **legal**
   **[verified — no warning under 2.12.5]**, and
   `protected_namespaces=()` pins that across pydantic versions.
4. **`Final[...]`** — mypy accepts a `Final` field kwarg while pydantic
   silently drops the field from the model (construction with it →
   `extra_forbidden`) **[verified both]**. Silent divergence → error.
   Fields are already frozen; `Final` is either redundant or a misspelled
   `ClassVar`.
5. **`dataclasses.InitVar`** — implies `__post_init__` machinery PureModule
   does not run. Error.
6. **Ellipsis default** (`x: object = ...`) — `...` is pydantic's required
   marker; as a user default it would silently flip the field to required
   at runtime while mypy sees a default. Error.
7. **Descriptor default** — an *annotated* assignment whose value has
   `__get__` (`image: Image = tick()` misplaced in the module body instead
   of `In`). Error, message points at In/Out bundles and `@resource`.
   Post-T1 hardening: also `isinstance(default, rows.FieldSpec)` via a
   lazy `from dimos.pure import rows` inside the function, skipped
   (`ImportError`) while rows.py doesn't exist.
8. **Unannotated plain-data assignment** (`emit_every = 5`). Neither
   checker makes it a field; the author almost certainly meant one.
   Precedent: pydantic v2 raises `PydanticUserError` for exactly this.
   Suspicious = value in `cls.__dict__` that is not in `__annotations__`,
   name doesn't start with `_`, value is not callable, not a class, and
   has no `__get__` (skips functions, properties, resources, nested
   classes, dunders, `_private` helpers).
9. **`__post_init__` defined in the body** — dataclass users will expect it
   to run; PureModule never calls it. Error names `@resource` and `step`
   as the doctrine homes for derived values.
10. **Unresolvable annotation** — collection resolves types via
    `typing.get_type_hints(cls, include_extras=True, localns=dict(vars(cls)))`;
    a `NameError` (TYPE_CHECKING-only import, typo) is wrapped in
    `ConfigFieldError` with best-effort field attribution (match
    `NameError.name` against raw annotation strings). Config field types
    must be runtime-importable — same cross-cutting rule as bundle fields.

`localns=vars(cls)` mirrors sketch §3 scoping: a config annotation may
reference a name defined in the class body, same resolution the engine
uses for step hints.

### 3.4 Collection algorithm (normative pseudocode)

```python
def _collect_config_fields(cls):
    own_ann = cls.__dict__.get("__annotations__", {})
    ns = dict(cls.__dict__)

    if "__post_init__" in ns:
        raise ConfigFieldError(...)                       # §3.3.9

    for name, value in ns.items():                        # §3.3.8
        if name in own_ann or name.startswith("_"):
            continue
        if callable(value) or isinstance(value, type) or hasattr(value, "__get__"):
            continue
        raise ConfigFieldError(...)                       # unannotated data

    try:
        hints = get_type_hints(cls, include_extras=True, localns=dict(vars(cls)))
    except NameError as e:
        raise ConfigFieldError(...) from e                # §3.3.10

    fields: dict[str, tuple[Any, Any]] = {}
    for name in own_ann:                                  # body declaration order
        hint = hints[name]
        if get_origin(hint) is ClassVar:
            continue                                      # constant, exempt
        # gauntlet §3.3.1–7 in order: underscore, reserved, pydantic
        # namespace, Final, InitVar, Ellipsis default, descriptor default
        ...
        default = ns.get(name, _REQUIRED)
        fields[name] = (hint, ... if default is _REQUIRED else default)
    return fields
```

`Final` detection must handle both `typing.Final` and
`typing_extensions.Final` (`get_origin(hint) is Final` catches the
subscripted form; also compare the bare form). `InitVar`: `hint is InitVar
or isinstance(hint, InitVar)`.

---

## 4. Synthesis mechanics

### 4.1 `__init_subclass__` sequence (normative)

```python
def __init_subclass__(cls, **kwargs: object) -> None:
    super().__init_subclass__(**kwargs)                       # 1 cooperative
    fields = _collect_config_fields(cls)                      # 2 §3
    bases = tuple(
        b.__pure_config_model__
        for b in cls.__bases__
        if b is not PureModule and issubclass(b, PureModule)
    ) or (PureModuleConfig,)                                  # 3 §4.3
    cls.__pure_config_model__ = _synthesize_config_model(cls, fields, bases)  # 4
    cls.__pure_step__ = classify(cls)                         # 5 T3 SEAM — LAST
```

Step 5 is the binding T3 contract (§10.1): the import is a module-level
`from dimos.pure.stepspec import classify`, the assignment is the **last
statement** — `__pure_step__`'s presence certifies that *all*
definition-time machinery passed, and config errors deterministically
precede shape errors. It runs for **every** subclass, config-only bases
included — no short-circuiting. Unknown class kwargs
(`class M(PureModule, weird=1)`) flow through `super().__init_subclass__`
and fail there natively.

### 4.2 Why not pydantic BaseModel as the module base (settled, priced in)

PureModule is a plain class. Making it a `BaseModel` would put pydantic's
metaclass in charge of the class body: it fights `@dataclass_transform`
(two competing synthesized-`__init__` stories), captures or mangles nested
classes and descriptors (`In`, `@resource`), and forbids the shapes-as-
plain-inheritance story. The synthesized model is a *separate* class per
module subclass; the module holds it (`__pure_config_model__`) and its
instance (`m.config`).

### 4.3 Per-subclass model, mirrored inheritance

`_synthesize_config_model` builds with `pydantic.create_model`:

- **Name**: `f"{cls.__name__}Config"` → validation errors self-identify
  (`1 validation error for Go2ConnectionConfig`) **[verified]**.
- **`__module__`**: `cls.__module__` (introspection/debugging hygiene; the
  dynamic class is still not qualname-importable — that's why pickling
  goes through the module, §9).
- **Bases**: the config models of the module's *direct* PureModule-derived
  bases, in declaration order; `(PureModuleConfig,)` for direct PureModule
  children. The model hierarchy is an order-isomorphic image of the module
  hierarchy, so shape subclasses inherit + extend fields for free, C3
  included, and "rebuild the model per subclass" is literally one
  `create_model` call per class.
- **Fields**: `{name: (type, default_or_...)}` from §3, in body
  declaration order.

`PureModuleConfig` extends `dimos.protocol.service.spec.BaseConfig`, so
every `m.config` `isinstance`-checks as the `Configurable.config: BaseConfig`
currency (§9.1). Its `ConfigDict` (full form in §2.2): `extra="forbid"`
(unknown kwargs die), `frozen=True` (§6), `arbitrary_types_allowed=True`
(inherited posture from spec.py; see §7.3 note), `validate_default=True`
(§5.3), `protected_namespaces=()` (§3.3.3).

### 4.4 Field ordering rule (canonical)

Fields appear in **reverse-MRO first-declaration order**: walk
`cls.__mro__` from most-base to most-derived; each class appends its own
body's fields in declaration order; a re-declaration (override) replaces
type/default but **keeps the original position**. **[verified]** — child of
`(alpha, beta)` adding `gamma` and overriding `alpha` dumps
`['alpha', 'beta', 'gamma']` with the new default; multi-base
`class Multi(B1, B2)` yields B2's fields, then B1's, then own (`['two',
'one', 'three']`) — exactly pydantic's own collection over the mirrored
bases, so the implementation inherits the rule by construction and the
test suite pins it against pydantic drift (§13.1).

---

## 5. Constructor semantics

### 5.1 Runtime routing (normative)

```python
def __init__(self, **kwargs: object) -> None:
    model = getattr(type(self), "__pure_config_model__", None)
    if model is None:                                  # only PureModule itself
        raise TypeError(
            "PureModule cannot be instantiated directly — subclass it "
            "(config fields + In/Out + step)"
        )
    cfg = model(**kwargs)                              # pydantic validates; raw ValidationError
    object.__setattr__(self, "__pure_config__", cfg)
    for name in model.model_fields:                    # class-level access (2.12 deprecates instance)
        object.__setattr__(self, name, getattr(cfg, name))
```

- Flat attributes are copied **from the validated model**, so
  `m.odom_timeout` and `m.config.odom_timeout` are always the *same
  object* (coercions included; a mutable value is intentionally aliased —
  one value, two views). Invariant: `all(getattr(m, k) is getattr(m.config, k)
  for k in model_fields)`.
- Instance attributes shadow the raw class-body defaults (which remain as
  plain class attributes, exactly like stdlib dataclasses;
  `Go2Connection.prefix` on the class is the raw default — a mild,
  familiar oddity).
- No `__post_init__`, no hooks. Construction = validation + copies.
- Positional args: `TypeError` from the `**kwargs`-only signature natively;
  statically `Too many positional arguments` **[verified]**.
- Shapes (step raising `NotImplementedError`, e.g. `CostMapper`) are
  instantiable on purpose — abstractness is doctrine, not a metaclass.
  Only `PureModule` itself is guarded.

### 5.2 Validation errors are pydantic's, unwrapped (decision)

Sketch §5c names pydantic behavior explicitly ("pydantic extra=\"forbid\" at
runtime", "pydantic ValidationError"); wrapping would cost the structured
`.errors()` API that YAML/CLI/blueprint loaders want, and the model name
already carries the module identity. So: **raw
`pydantic.ValidationError`**, error types observed **[verified]**:
`extra_forbidden` (unknown kwarg), `missing` (required omitted), per-type
errors (`string_type`, `int_parsing`, ...), `frozen_instance` (§6).

### 5.3 Validation mode: pydantic default (lax), defaults validated

Config increasingly arrives from YAML/CLI/blueprints where everything is
strings and ints; lax mode is the ergonomic choice and mypy is the strict
layer for code. Observed conversion posture **[verified]**:

| input | outcome |
| --- | --- |
| `float` → `str` field | **rejected** (`string_type`) — the sketch's `robot_ip=0.5` example holds |
| `int` → `float` field | accepted, becomes `1.0` |
| `"5"` → `int` field | accepted, becomes `5` |
| `True` → `int` field / `1` → `bool` field | accepted (stdlib bool-is-int semantics) |

`validate_default=True`: a type-wrong default (`n: int = "x"`) raises at
**first construction** that uses it, not at class definition
**[verified]** — mypy already flags it at the def site, and eager
class-def validation was rejected (it needs per-field `TypeAdapter`
machinery that fights `arbitrary_types_allowed`). Honest limitation,
documented.

---

## 6. Frozen at BOTH surfaces (plus the third: mypy)

| surface | trigger | exception | notes |
| --- | --- | --- | --- |
| module | `m.odom_timeout = 1.0`, `m.anything = x`, `del m.x` | `FrozenModuleError` (an `AttributeError`) | `__setattr__`/`__delattr__` raise **unconditionally** — non-field names too: per-run state belongs in State, heavyweight objects in `@resource`. Machinery writes go through `object.__setattr__` (the documented internal seam used by `__init__` and by T7's resource cache). |
| config model | `m.config.odom_timeout = 1.0` | `pydantic.ValidationError`, error type `frozen_instance` **[verified]** | pydantic's native frozen guard; also makes instances hashable (§8). |
| static | either assignment above | mypy `[misc]` read-only property errors **[verified]** | from `frozen_default=True` + the read-only `config` property. |

Two different exception types at the two runtime surfaces is deliberate:
module-side is attribute-protocol-shaped (`AttributeError` keeps
`hasattr`/`setattr` probing semantics sane), model-side is pydantic-native.
Both spell the same doctrine; `FrozenModuleError`'s field-message teaches
the rebuild idiom (§11).

Rebuild-never-mutate, the canonical idiom (sketch §5c verbatim):

```python
c2 = Go2Connection(**{**c.config.model_dump(), "odom_timeout": 1.0})
```

---

## 7. Mutable defaults

### 7.1 Rule: raw mutable defaults are legal, copied per instance

`layers: list[str] = []` is allowed. Routing defaults through pydantic
gives default_factory semantics natively: each construction deep-copies
mutable defaults — `list`, `dict`, `set` all verified isolated across
instances **[verified]**. No `pm.field(default_factory=...)` specifier is
introduced (zero new ceremony; the sketch shows only plain `name: type =
default` and that stays the whole story). This satisfies the index's
"default_factory treatment" with pydantic as the factory.

### 7.2 What frozen does NOT promise

Frozen is reassignment-level, not deep: `m.layers.append(...)` mutates the
shared value and shifts `model_dump()` — identity drift. Doctrine: config
values are immutable data; prefer `tuple[str, ...]` over `list[str]`
(bonus: keeps the config hashable, §8). The spec makes this a documented
sharp edge, not a guard — deep-freezing containers would mean coercing
user-declared types.

### 7.3 Field-type posture (note for T10)

`arbitrary_types_allowed=True` is inherited posture from spec.py's
BaseConfig and stays (Path/Enum/odd types keep working). Canonical
identity uses python-mode `model_dump()`; tuples dump as tuples in python
mode and lists in JSON mode **[verified]** — T10/memo layers that need a
*hashable/serializable* key should use `model_dump(mode="json")` and will
surface non-JSON-able config types as errors **there**, at the feature
that needs the guarantee. T2 does not restrict field types.

---

## 8. Determinism

- `model_dump()` returns a plain `dict` whose key order is the §4.4
  canonical field order — dict insertion order, stable, pinned by test.
- Same config → equal models → equal dumps; frozen models hash by value,
  equal configs hash equal **[verified]**.
- Config equality is class-scoped: two synthesized models with identical
  names/fields but different classes compare unequal **[verified]** —
  aligned with identity = class + config.
- Hashability requires hashable field values; a `list`-valued config makes
  `hash(m.config)` (and `hash(m)`) raise `TypeError` **[verified]** —
  acceptable, documented next to the `tuple` recommendation (§7.2).

---

## 9. Identity conveniences and Service interop

### 9.1 Service protocol satisfaction (decision: duck, not subclass)

`dimos/protocol/service/spec.py` defines `Service(Configurable, ABC)` with
`start() -> None`, `stop() -> None`, and `Configurable.config: BaseConfig`
built by an untyped `**kwargs: Any` `__init__`. Repo survey: **zero**
`isinstance(x, Service)`/`isinstance(x, Configurable)` call sites —
consumption is structural. PureModule therefore does **not** inherit
Service:

- `Configurable.__init__(**kwargs: Any)` is the exact untyped constructor
  §5c exists to replace; inheriting it buys nothing and confuses MRO.
- `Configurable` declares `config` as a *writable attribute*; overriding
  with a read-only property would be an `[override]` error to suppress.
- The interop that matters is carried anyway: `m.config` **is** a
  `spec.BaseConfig` instance (PureModuleConfig extends it, §4.3), and
  `warmup/start/stop` have Service-compatible `(self) -> None` signatures.

A pure module thus drops into any slot that duck-hosts a Service today.
(`warmup` has no Service counterpart — the legacy `Module.build()` plays
that role in blueprints; mapping those lifecycles is T8's business.
T2 ships all three as **no-op hooks** — `return None` — so pre-T8 code and
tests can call the full lifecycle harmlessly.)

If a future blueprint host adds nominal `isinstance(Service)` checks, that
host is the thing to fix (structural check or explicit adapter) — record
in review, do not reshape PureModule.

### 9.2 `__eq__` / `__hash__`

```python
def __eq__(self, other):
    if other is self: return True
    if not isinstance(other, PureModule): return NotImplemented
    return type(other) is type(self) and other.config == self.config

def __hash__(self):
    return hash((type(self), self.config))
```

Exact-class match (no subclass symmetry holes); config compares by value
within the same synthesized model class. Two identically-configured
instances are equal — they denote the same computation (memo doctrine).
Unhashable field values make `hash` raise (§8).

### 9.3 `__repr__`

`f"{type(self).__name__}({', '.join(f'{k}={v!r}' for k, v in self.config.model_dump().items())})"`
— e.g. `Go2Connection(prefix='go2a', robot_ip=None, odom_timeout=0.5)`.
Canonical order, eval-ish, inspector-friendly.

### 9.4 `__reduce__`: pickle/copy = rebuild from config

```python
def __reduce__(self):
    return (_rebuild_module, (type(self), self.config.model_dump()))
```

Synthesized model classes are not qualname-importable, so modules pickle
through their (importable) class + canonical dump — which is also exactly
the doctrine: an instance IS class + config, nothing else travels.
Consequences, intended: `copy.copy`/`copy.deepcopy` produce fresh rebuilds
(cached lazy resources are dropped, per-run state can't leak);
`pickle.dumps(m.config)` directly is unsupported — dump instead.

---

## 10. Seams to T3 and T4 (binding)

### 10.1 T3 — classification (from t3-validation.md §8)

Verbatim contract: `module.py` gains

```python
from dimos.pure.stepspec import classify
```

and `__init_subclass__` ends with

```python
cls.__pure_step__ = classify(cls)
```

as its **last statement** — after config synthesis, so `__pure_step__`'s
presence certifies all definition-time machinery passed and config errors
deterministically precede shape errors (§4.1).

Constraints honored by T2:

- (a) PureModule defines none of `step`/`fold`/`In`/`Out`/`State` (§2.1).
- (b) `__pure_step__` is a dunder, assigned unannotated, after collection —
  structurally unable to become a config field; pinned by
  `test_dunder_pure_step_not_a_field`.
- (c) `__init_subclass__` never short-circuits: classification runs for
  every subclass, config-only bases included.

Sequencing: `stepspec.py` does not exist while T2 implements (landing
order T2 → T3), so the T2-landed `module.py` carries the import and the
assignment as a **marked comment placeholder** at the exact call site
(skeleton already does); T3's landing activates both lines. Until then the
`__init_subclass__` sequence simply ends at step 4.

**Decision (requested by T3): reuse `PureModuleDefinitionError` — yes.**
One user-facing definition-error type is right: `except
PureModuleDefinitionError` should catch "my class definition is wrong"
regardless of which subsystem noticed. Mechanics: `ConfigFieldError`
remains the concrete type raised by config machinery and *re-bases* onto
`stepspec.PureModuleDefinitionError` when T3 lands (one-line change in
`config.py`, performed as part of T3's module.py integration; import
direction config → stepspec is acyclic provided stepspec does not import
config — if T3's internals ever need config metadata, they read
`cls.__pure_config_model__` off the class, no import). Both types subclass
`TypeError` before and after, so the flip is behavior-compatible;
`test_config_field_errors_are_type_errors` pins the stable contract and a
post-flip test should add the unified-base assertion.

**Open question (requested by T3, recorded, NOT built): `abstract=True`
class kwarg for config-only mixin bases.** A base like `class
RobotBase(PureModule): robot_ip: str` (shared config, no step) currently
cannot exist if classification rejects step-less classes — and shared
config *cannot* ride a plain mixin (non-PureModule annotations are not
fields, §3.2). Recommendation: if the need materializes, prefer the
explicit `abstract=True` kwarg over implicitly tolerating step-less
classes — implicit-abstract would silently defer "forgot to write step"
from import time to instantiation, weakening T3's loud-import guarantee,
while the kwarg preserves it and reads as intent. The `**kwargs` channel
through `__init_subclass__` (§4.1) already accommodates it without
signature change. Defer until T12's example ports show a real config-only
base; until then, shared config lives on shape classes (which carry
step-raising contracts anyway).

### 10.2 T4 — EngineSurface base (binding)

The final class statement is

```python
from dimos.pure.typing import EngineSurface

class PureModule(EngineSurface):
```

`EngineSurface` (T4, `dimos/pure/typing.py`) hosts the `over()` overload
set and the `i = _InAccessor()` / `o = _OutAccessor()` port descriptors.
T2 constraints:

- Inherit it; **never redeclare** `over`, `i`, or `o` on PureModule.
- The accessors are deliberately **unannotated** class attributes, so
  `@dataclass_transform` field collection (annotated assignments only)
  never sees them; T4 verified the composition end-to-end (its
  `case_config_kwargs.py` / `case_tagger_floor.py` fixtures run the full
  transform stack over EngineSurface). T2 pins non-interference from its
  side with `test_engine_surface_accessors_not_fields` (`i`/`o`/`over`
  absent from `model_fields`, kwarg use rejected, annotated use →
  `ConfigFieldError` via `RESERVED_CONFIG_FIELDS`).
- `super().__init_subclass__` in §4.1 now traverses EngineSurface —
  cooperative chain unchanged; the §4.1 `bases` computation is unaffected
  (EngineSurface is above PureModule, never PureModule-derived).

While `typing.py` is absent (worktree/spec time), the skeleton carries a
marked placeholder comment at the class statement; the T2 implementer
wires the real base.

---

## 11. Error catalog (exact types and message templates)

Definition time — `ConfigFieldError(TypeError)` (base flips per §10):

| trigger | template |
| --- | --- |
| reserved name | `{cls.__qualname__}.{name}: {name!r} is reserved by PureModule machinery and cannot be a config field` |
| pydantic namespace | `{cls.__qualname__}.{name}: {name!r} collides with a pydantic BaseModel attribute on the synthesized config model; rename the field` |
| leading underscore | `{cls.__qualname__}.{name}: config field names must not start with '_' (pydantic treats them as private while mypy treats them as fields); use ClassVar for internal constants` |
| `Final` | `{cls.__qualname__}.{name}: Final is not allowed on config fields — they are already frozen, and pydantic would silently drop the field; use ClassVar for a class constant` |
| `InitVar` | `{cls.__qualname__}.{name}: InitVar is not supported — PureModule has no __post_init__` |
| Ellipsis default | `{cls.__qualname__}.{name}: Ellipsis is not a valid config default` |
| descriptor default | `{cls.__qualname__}.{name}: descriptors cannot be config field defaults — row specifiers belong inside In/Out bundles, resources use @resource` |
| unannotated data | `{cls.__qualname__}.{name} = {value!r}: assignment without a type annotation is ambiguous — write {name}: {type(value).__name__} = ... to make it a config field, or annotate with ClassVar to keep a class constant` |
| `__post_init__` | `{cls.__qualname__}: __post_init__ is unsupported — derive values in @resource factories or in step` |
| unresolvable annotation | `{cls.__qualname__}: config field annotation {raw!r} is not resolvable at runtime ({err}); config field types must be runtime-importable` |

Construction time — raw pydantic (§5.2): `ValidationError` titled
`{ClassName}Config` with native `extra_forbidden` / `missing` / type
errors; `TypeError` for positional args and for direct `PureModule()`
(§5.1 message).

Mutation time — `FrozenModuleError(AttributeError)`:

| trigger | template |
| --- | --- |
| set a config field | `{cls.__name__}.{name} is frozen config — rebuild: {cls.__name__}(**{{**m.config.model_dump(), {name!r}: ...}})` |
| set anything else | `{cls.__name__} is immutable — per-run state belongs in State, heavyweight objects in @resource` |
| delete | `{cls.__name__}.{name} cannot be deleted — modules are immutable` |

Model mutation — pydantic `ValidationError` (`frozen_instance`), unwrapped.

---

## 12. Edge cases → sections/tests (index "watch out" map first)

| index/task bullet | resolved by |
| --- | --- |
| don't make PureModule a BaseModel; separate model per class, rebuilt per subclass | §4.2, §4.3; `test_shape_subclass_inherits_and_extends_fields` |
| frozen at BOTH surfaces | §6; `test_frozen_module_surface`, `test_frozen_config_surface` + static cases |
| mutable defaults → default_factory treatment | §7.1; `test_mutable_default_not_shared` |
| canonical deterministic dump ordering (memo/checkpoint feed) | §4.4, §8; `test_canonical_model_dump`, `test_model_dump_declaration_order_with_inheritance` |
| unknown kwarg / wrong type / mutation / bad default error behavior | §5.2, §5.3, §6, §11; construction/frozen tests |
| Service signatures for blueprint interop | §9.1; `test_service_interop_surface` |
| T3 seam placement + contract | §10.1, §4.1; `test_dunder_pure_step_not_a_field` |
| T4 EngineSurface base + i/o non-interference | §10.2, §3.2; `test_engine_surface_accessors_not_fields` |
| Tagger stays four declarations | §1 floor check; T12 CI guard |

Additional edges specified: required (default-less) fields §3.1; ClassVar
constants §3.2/§3.3; non-PureModule mixins §3.2 (incl. the
runtime-stricter setattr note); `Annotated` passthrough §3.1; `Final` /
`InitVar` / Ellipsis / descriptor defaults / unannotated / underscore /
`model_*` names §3.3; string annotations + `localns` scoping + TYPE_CHECKING
imports §3.3.10; multi-base field order §4.4; lax-coercion table §5.3;
class-attr default shadowing §5.1; hashability limits §8; pickle/copy
semantics §9.4; direct `PureModule()` §5.1; instantiable shapes §5.1.

---

## 13. Test plan

### 13.1 Runtime unit tests — DELIVERED as real bodies

`dimos/pure/test_config.py` ships with this spec: 29 tests, collection
green **[verified: 29 collected / 29 skipped]**, module-level
`pytestmark = pytest.mark.skip(reason="T2 skeleton — enable with
implementation")` — the implementer **deletes that one line**; every test
must then pass unmodified (they are the executable form of §§3–9; example
classes are defined inside test bodies so importing the file never triggers
`__init_subclass__`). Groups: construction (happy path, flat-vs-config
identity, canonical dump verbatim from sketch §5c, unknown kwarg, wrong
type, missing required, lax int→float, positional, direct-PureModule);
non-fields (machinery members, `__pure_step__` pin, `i`/`o`/`over`
EngineSurface pin); frozen (both
surfaces, rebuild idiom); mutable defaults; determinism (declaration
order + override position, shape inherit+extend); identity (round-trip
eq/hash, class-scoped eq, repr, `__reduce__` rebuild); definition-time
errors (reserved, pydantic-collision, unannotated, underscore, Final,
TypeError stability across the §10 base flip); Service interop surface.

### 13.2 Static-typing regression cases — verified today, land in T4's harness

T4 owns the harness, and its `case_config_kwargs.py` fixture **already
pins** the kwarg/field-collection statics (typo `[call-arg]` with
did-you-mean, wrong-type `[arg-type]`, nested-class-not-a-field,
`@resource`-not-a-field) — do not duplicate those. The T2 case file below
covers the **T2-specific remainder**: reveal-types incl. the `config`
property and `model_dump()`, ClassVar exclusion, required-field
enforcement, positional rejection, and the `frozen_default` read-only
errors. It was run against the landed skeletons under `mypy --strict`
(mypy 1.19.0) and produced **exactly** the annotated output — T4 folds it
in (deduping any overlap in favor of its fixtures); until then it is
reproducible by hand (`MYPYPATH=<repo> mypy --strict cases_t2.py`).

```python
from __future__ import annotations

from typing import ClassVar

from dimos.pure.config import PureModuleConfig
from dimos.pure.module import PureModule


class Go2Connection(PureModule):
    LIMIT: ClassVar[int] = 3
    prefix: str = ""
    robot_ip: str | None = None
    odom_timeout: float = 0.5


class NeedsIp(PureModule):
    robot_ip: str  # no default -> required kwarg


class Sub(Go2Connection):
    extra_field: int = 7


c = Go2Connection(prefix="go2a", robot_ip="192.168.12.1")
reveal_type(c.prefix)                # N: Revealed type is "builtins.str"
reveal_type(c.odom_timeout)          # N: Revealed type is "builtins.float"
reveal_type(c.config)                # N: Revealed type is "dimos.pure.config.PureModuleConfig"
reveal_type(c.config.model_dump())   # N: Revealed type is "builtins.dict[builtins.str, Any]"

Go2Connection(prefx="go2a")     # E: Unexpected keyword argument "prefx" for "Go2Connection"; did you mean "prefix"?  [call-arg]
Go2Connection(robot_ip=0.5)     # E: Argument "robot_ip" to "Go2Connection" has incompatible type "float"; expected "str | None"  [arg-type]
Go2Connection(LIMIT=5)          # E: Unexpected keyword argument "LIMIT" for "Go2Connection"  [call-arg]
Go2Connection("go2a")           # E: Too many positional arguments for "Go2Connection"  [misc]
NeedsIp()                       # E: Missing named argument "robot_ip" for "NeedsIp"  [call-arg]
Sub(prefix="a", extra_field=1)  # ok: inherited + own fields

c.odom_timeout = 1.0  # E: Property "odom_timeout" defined in "Go2Connection" is read-only  [misc]
c.config = None       # E: Property "config" defined in "PureModule" is read-only  [misc]
                      # E: Incompatible types in assignment (expression has type "None", variable has type "PureModuleConfig")  [assignment]

_cfg: PureModuleConfig = c.config  # ok: BaseConfig-compatible currency
```

The session experiment's nested-bundle/descriptor lines
(`Go2Connection(In=3)`, `Go2Connection(grid=Grid())`, `reveal_type(c.grid)`)
are already carried by T4's `case_config_kwargs.py`; runtime-side they are
preserved in `test_machinery_members_are_not_fields`.

### 13.3 Not tested here (owned elsewhere)

Step/bundle resolution (T3), `over()` typing (T4), full-lifecycle behavior
of warmup/start/stop (T8), `model_dump(mode="json")` hashing rules (T10),
whole-file `pickle.dumps` of module-level example classes (T12 examples).

---

## 14. Acceptance criteria

- [x] `uv run mypy dimos/pure/config.py dimos/pure/module.py` clean
      (repo strict config) — already true for the skeletons.
- [x] Skip line removed from `dimos/pure/test_config.py`; all 29 tests
      pass unmodified; no engine imports anywhere in the file.
- [x] §13.2 case file reproduces its annotated mypy output exactly.
      (Landed as `test_typing_fixtures/case_config_real.py`; one adaptation
      to the harness's one-diagnostic-per-line rule — see appendix.)
- [x] Sketch §5c executes literally: the `Go2Connection` construction,
      flat access, `model_dump()` dict equality (declaration-ordered),
      both frozen failures, and the sweep-rebuild line.
- [x] `VoxelGridMapper(emit_every=2)` from sketch `_unit_tests` constructs
      once T1/T3 land (config fields `voxel_size`, `emit_every` collected;
      nested `In`/`Out`/`State`, `@resource grid`, `step` are not fields).
      (T1/T3 — the config half is proven by `test_machinery_members_are_not_fields`.)
      (Landed with T3's seam activation as
      `test_voxelgridmapper_style_construction`; `@resource grid` is T7 and
      excluded from the construction. See note 4.)
- [x] Tagger floor: zero-config modules define no extra line for T2.
- [x] `__init_subclass__` order matches §4.1 with the T3 placeholder as
      the marked final step; no step/fold/In/Out/State on PureModule.
- [x] `class PureModule(EngineSurface)` wired per §10.2; no
      `over`/`i`/`o` declared or shadowed by T2; T4's fixtures still pass
      over the composed base.
- [x] All §11 messages implemented verbatim (templates are normative).
- [x] No new files beyond config.py / module.py / test_config.py; no
      `__init__.py` created; no imports of not-yet-existing siblings
      except the §3.4 lazy rows guard and the §10 activated seam.
      (Plus the orchestrator-sanctioned static fixture `case_config_real.py`
      — no engine file, no `__init__.py`, no missing-sibling import.)

## Implementation notes

Deviations / decisions made during implementation, none altering the spec's
intent:

1. **`__pure_config__` instance-attr annotation on the base (module.py).**
   §2.1 states PureModule's body carries "no plain annotated attributes (only
   the ClassVar-annotated `__pure_config_model__`)", with the operative rule
   "any future internal state on the base stays 'unannotated or dunder'". The
   config backing store needs a type for mypy on the read-only `config`
   property, so it is declared as `__pure_config__: PureModuleConfig` — a
   **dunder**, blessed by that rule. Verified non-interfering: it is not a
   dataclass_transform field (T4 §9 finding), never collected (base body is
   never scanned; underscore-prefixed regardless), and `case_config_real.py`
   confirms subclasses do not require it and reveals are unaffected.

2. **`case_config_real.py` — one adaptation vs §13.2 verbatim.** The spec's
   `c.config = None` line emits *two* diagnostics (`[misc]` read-only +
   `[assignment]`), but T4's harness enforces one diagnostic per marked line.
   The read-only-ness of `config` is pinned instead by `c.config = c.config`
   (type-matched RHS → single `[misc]`); the value type is separately pinned
   by `reveal_type(c.config)` and the `_cfg: PureModuleConfig = c.config`
   assignability line. All other §13.2 lines are reproduced exactly. Required
   a one-line note appended to `t4-typing.md` §7.5 rule 1 (real-surface
   fixtures allowed once the surface ships), per the task mandate.

3. **`mypy dimos/pure` and the untracked seed.** Recursive
   `uv run mypy dimos/pure` surfaces 5 diagnostics, *all* from the untracked
   design seed `config_fields_experiment.py` (self-contained `reveal_type` +
   deliberate-error lines; imports nothing from `dimos.pure`). It is
   pre-existing and left untracked per instruction; the deliverable and every
   committed file are clean (`uv run mypy dimos/pure --exclude
   config_fields_experiment` → success, 5 source files).

4. **T3 seam executed (by the T3 landing, recorded here).** §10.1 performed
   verbatim: `module.py` imports `classify` and stamps
   `cls.__pure_step__ = classify(cls)` as the LAST `__init_subclass__`
   statement (plus a `__pure_step__: ClassVar[StepSpec]` dunder declaration,
   blessed by note 1's rule, so the stamp is strict-mypy-clean);
   `ConfigFieldError` re-based onto `stepspec.PureModuleDefinitionError`
   (one line + import; still a `TypeError`), with the unified-base assertion
   landed as `test_config_field_errors_share_definition_error_base`. With
   classification mandatory, every definition-succeeding `test_config.py`
   class gained a minimal step (shared `_In`/`_Out` rows + `def step`;
   `Bad` classes need none — config errors precede classification per §4.1);
   all original assertions preserved. Suite: 29 → 31 tests.

## 15. Relitigation

None. Settled decisions are implemented as given. Notable within-mandate
decisions (beyond the sketch's letter, flagged for review):
`frozen_default=True` (§2.1, static freeze); unconditional instance
`__setattr__` freeze incl. non-fields (§6); raw-pydantic error surface
(§5.2); lax validation mode + `validate_default` at-construction (§5.3);
the §3.3 definition-time gauntlet (each entry prevents a verified silent
static/runtime divergence or a doctrine leak); duck-typed Service interop
instead of inheritance (§9.1); `__eq__/__hash__/__repr__/__reduce__`
identity conveniences (§9.2–9.4); mutable defaults accepted with pydantic
copy semantics rather than rejected (§7.1).
