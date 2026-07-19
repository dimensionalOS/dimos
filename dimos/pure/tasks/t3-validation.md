# T3 — Step-shape classification + definition-time validation

Implementation spec. Source of truth for doctrine:
`dimos/memory2/puremodule_api_sketch3.py` (§TYPING block + banner);
task boundary: `tasks/index.md` §T3. A coding agent implements from this
document without re-deriving design.

**Layout amendment (flagged per index relitigation rule):** the index co-locates
T3 in `module.py`. This spec moves T3's machinery to its own file
**`dimos/pure/stepspec.py`**. Rationale: T2 and T3 land as separate tasks with
disjoint file ownership (parallel branches, no merge collisions), and it keeps
`module.py` small — the T2 seam (§8) is a single assignment. Blast radius: one
line in the index layout table; `module.py` gains one import + one statement.

---

## 0. Scope and doctrine

The sketch removed generics from module classes; the priced cost (sketch
banner) is that a malformed step no longer errors at its own class statement
under mypy — it surfaces at the first engine call site as an opaque
`Invalid self argument`. T3 is the compensation: `PureModule.__init_subclass__`
classifies and validates every subclass **at import time**, so a malformed
module still fails at its own class statement — with a better message than
mypy ever gave. **The error messages are the product.**

Doctrine pins:

- **Validate, don't instantiate.** Classification inspects functions,
  signatures, and annotations. It never constructs the module, a bundle, a
  State, or touches a resource. §6 makes this a hard guarantee.
- **The step signature is the single typing authority.** Classification reads
  the signature; nested `In`/`Out` declarations are placement, not authority.
  A declared-but-unread nested bundle is an error (§5.6), not a tiebreak.
- **Runtime resolution mirrors mypy's lexical scoping.** Anything mypy resolves
  in a step annotation, `classify` resolves; anything mypy rejects as
  out-of-scope, `classify` rejects with guidance (§4). No MRO-walking name
  magic that would accept code mypy rejects.
- **Fail fast, fail loud, fail once.** First violated rule raises; import
  stops. One clear error beats a wall of cascades (dataclasses precedent).
- **Abstractness is a doctrine, not a metaclass.** A shape's step raises
  `NotImplementedError` in its *body*; T3 never reads bodies, so shapes
  validate as ordinary modules (§6).

## 1. Deliverables and files

| File | Content |
| --- | --- |
| `dimos/pure/stepspec.py` | `PureModuleDefinitionError`, `Rule`, `StepKind`, `StepSpec`, `classify()`, `step_spec()`, `PURE_STEP_ATTR`, private helpers |
| `dimos/pure/test_stepspec.py` | unit tests (§12) |
| `dimos/pure/module.py` (T2-owned) | one import + one assignment (§8) — T2 places it, this spec defines it |

`stepspec.py` sits in the zero-runtime-dep data layer. Its only intra-package
import is `from dimos.pure.rows import In, Out` (T1's bases, needed for the
row checks) — imported **as a module-level import of the submodule path**,
never through the `dimos.pure` package surface (`module.py` imports
`stepspec.py` and `__init__.py` imports `module.py`; importing the package
surface from here would cycle). Constraint on T1 (already implied by
layering): `rows.py` imports nothing from `dimos.pure`.

Recommended `pm` surface re-exports (owner of `__init__.py` decides):
`PureModuleDefinitionError`, `StepSpec`, `StepKind`, `step_spec`. `Rule` and
`classify` stay importable from `dimos.pure.stepspec` (tests, tooling) but
need not be on the `pm` surface.

## 2. Public surface of `stepspec.py`

```python
PURE_STEP_ATTR: Final = "__pure_step__"          # where T2 stores the spec

class PureModuleDefinitionError(TypeError): ...  # §9

class StepKind(Enum):                            # §7
    STATELESS = "stateless"
    ASYNC_STATELESS = "async-stateless"
    MEALY = "mealy"
    FOLD = "fold"

class Rule(Enum): ...                            # one member per error slug, §9

@dataclass(frozen=True)
class StepSpec: ...                              # §7

def classify(cls: type[Any]) -> StepSpec: ...    # §3–§5; raises PureModuleDefinitionError
def step_spec(cls: type[Any]) -> StepSpec: ...   # accessor, §7.3
```

`classify` is **pure and structural**: it takes any class (no `PureModule`
import — none exists in this file, avoiding the T2 cycle and making T3
testable without T2), inspects it, and returns a `StepSpec` or raises. It
does not mutate `cls`; the caller (T2) stores the result.

Reserved member names on a module class: **`step`, `fold`, `In`, `Out`,
`State`**. T3 owns the semantics of all five; a helper method or config field
may not use these names (violations surface as the shape errors below —
e.g. a `step = 3` class attribute is `[step-not-function]`).

## 3. The classification decision table

Classification runs in gate order. Each gate either passes, refines the
candidate, or raises with the named rule (§9 has every message verbatim).
Later gates may assume earlier ones passed.

### G0 — candidate discovery

For `name` in (`"step"`, `"fold"`): find the **owner** = first class in
`cls.__mro__` whose `__dict__` contains `name`. (Verified: `__init_subclass__`
runs after the class dict is fully populated and after `__set_name__`, so
`vars(cls)` is complete.) `PureModule` itself defines neither — a hard
constraint on T2 (§8) — and `object` defines neither, so any hit is
user-authored. Discovery reads only these two names: no `dir()`, no attribute
sweep (§6).

| `step` found | `fold` found | Result |
| --- | --- | --- |
| no | no | **error `[step-missing]`** |
| yes | yes | **error `[step-and-fold]`** (message names both owners) |
| yes | no | candidate = `step`, continue |
| no | yes | candidate = `fold`, continue |

There is no "abstract base without a step" carve-out: a class with neither
attribute errors, and the message teaches the shape pattern (declare the I/O
contract plus a step that raises `NotImplementedError`, like `CostMapper`).
Config-only mixins that intentionally lack a step are deferred (§15 Q1).

### G1 — function kind

`fn = vars(owner)[name]` (raw class-dict object — never `getattr`, which for
exotic descriptors could execute code).

| `fn` is | Result |
| --- | --- |
| `staticmethod` | **error `[step-not-function]`** (staticmethod variant) |
| `classmethod` | **error `[step-not-function]`** (classmethod variant) |
| not `types.FunctionType` (int, partial, callable object, …) | **error `[step-not-function]`** (names the type found) |
| plain function (incl. `async def`, generator, `functools.wraps`-decorated wrapper) | continue |

Decorated steps: T3 classifies the outermost function — the wrapper's own
annotations (copied by `functools.wraps`) and its own coroutine-ness
(`inspect.iscoroutinefunction`, which honors `inspect.markcoroutinefunction`)
decide. A decorator that hides either is out of contract; no unwrapping.

### G2 — parameter shape

From `inspect.signature(fn)` (never on the bound method — on the raw
function, so param 0 is the receiver):

| Observation | Result |
| --- | --- |
| zero parameters | **error `[step-no-self]`** |
| any param of kind `VAR_POSITIONAL` / `VAR_KEYWORD` / `KEYWORD_ONLY` | **error `[step-params]`** (names param + kind) |
| params of kind `POSITIONAL_ONLY` | allowed — orchestrator amendment: T4 adopted positional-only protocol params (t4-typing.md §2.3), so mypy accepts a `/`-spelled step; T3 must too |
| any param (incl. the receiver) with a default | **error `[step-param-default]`** |
| otherwise | `data_params = params[1:]`, continue |

Rationale for the strict rows: mypy rejects a keyword-only or variadic
implementation of T4's protocols (parameter-kind compatibility), so runtime
must too — positional-only impls, by the same parity principle, are ACCEPTED
now that T4's protocols are themselves positional-only; and parameter defaults make `typing.get_type_hints`
output *version-dependent* (Python 3.10 wraps `= None` params in `Optional`,
3.11+ does not — observed) — a module must classify identically across the
supported floor, so defaults are banned outright. The receiver is identified
**by position**, never by the name `self`.

### G3 — arity × async → kind

`is_coro = inspect.iscoroutinefunction(fn)`,
`is_agen = inspect.isasyncgenfunction(fn)`, `n = len(data_params)`.

| Candidate | `n` | coro / asyncgen | → |
| --- | --- | --- | --- |
| step | 1 | no / no | **STATELESS** (provisional — G5 may upgrade to error) |
| step | 1 | yes / — | **ASYNC_STATELESS** |
| step | 2 | no / no | **MEALY** |
| step | 2 | yes / — | **error `[async-mealy]`** |
| step | any | — / yes | **error `[step-not-function]`** (async-generator variant: a step is one tick → one result; whole-stream is `fold`) |
| step | 0 | — | **error `[step-arity]`** (zero-row variant, with forgot-`self` heuristic: if the receiver-position param resolves best-effort to a `pm.In` subclass, say so; resolution failure here is swallowed) |
| step | ≥3 | — | **error `[step-arity]`** (bundle-your-inputs variant) |
| step (sync) | any | generator function | permitted mechanically — a generator `step` returns a generator object, which then fails G5's return-annotation rules (`[out-not-row]` unless annotated as rows; annotating `-> Iterator[...]` fails row check). No special gate needed; the G5 message covers it. |
| fold | 1 | no / no | **FOLD** (generator function *or* plain function returning an iterator — both legal; the return annotation is the contract, not `isgeneratorfunction`) |
| fold | ≠1 | — | **error `[fold-arity]`** |
| fold | — | yes / — or — / yes | **error `[fold-async]`** |

### G4 — annotation resolution

Resolve hints per §4. Any evaluation failure → **error
`[step-unresolvable]`** (guidance ladder, §9). Then: every data param and
`"return"` must be present in the resolved hints, else **error
`[step-unannotated]`** (missing keys = unannotated; verified behavior of
`get_type_hints`).

### G5 — per-kind annotation validation

§5. Output on success: a `StepSpec` (§7).

## 4. Annotation resolution

### 4.1 The formula

```python
owner   = <MRO owner of the candidate function>        # G0
fn      = vars(owner)[name]
localns = {**vars(owner), owner.__name__: owner}
hints   = typing.get_type_hints(fn, localns=localns)   # globalns defaults to fn.__globals__
```

Three deliberate choices:

1. **`localns` is the *owner's* class dict, not `cls`'s.** A config-only
   subclass (`class TunedTagger(Tagger): threshold: float = 0.5`) inherits
   `step`; its bare `In` annotation was written in `Tagger`'s body and must
   resolve against `vars(Tagger)`. Resolving against `vars(TunedTagger)`
   would fail — `__dict__` does not include inherited names. (Verified.)
2. **`globalns` is left to default to `fn.__globals__`** — the module that
   lexically contains the annotation. Never pass `cls`'s module globals: for
   cross-module inheritance the subclass's module is the *wrong* namespace.
   (Verified via exec'd-module fixture.)
3. **The owner's class name is injected into `localns`.** During
   `__init_subclass__` the class object exists but its module-level name is
   not yet bound, so a self-qualified annotation (`def step(self, i: Tagger.In)
   -> Tagger.Out` inside `Tagger`'s own body) would `NameError` on the bare
   formula. mypy *accepts* the self-qualified spelling (ordinary forward
   reference), so runtime must too — the injection restores parity.
   (Verified both ways: without injection → `NameError: name 'C3b' is not
   defined`; with injection → resolves.)

This is a **minor amendment** to the index's stated formula
(`get_type_hints(step, localns=vars(cls))`): `vars(cls)` → owner's vars +
owner-name injection. Both changes are strictly parity-restoring (index
formula rejects code mypy accepts); flagged in §14.

`get_type_hints` semantics relied on (all verified on CPython 3.12, stdlib
documented since 3.10): string and non-string annotations handled uniformly
(objects pass through, strings `eval` in `globalns`/`localns`);
`Annotated[X, …]` stripped by default (mypy also sees through it — no rule
needed); mappingproxy accepted as `localns` (we build a fresh dict anyway);
`async def` return hint is the *awaited result type*, not `Coroutine`;
unannotated params are absent keys.

### 4.2 What resolves where — the contract users learn

| Spelling | Defining class body | Subclass body | Another module |
| --- | --- | --- | --- |
| bare `In` / `Out` / `State` | ✓ (class-body names, via `localns`) | ✗ — class attributes are not lexically scoped; mypy agrees | ✗ |
| self-qualified `Tagger.In` inside `Tagger` | ✓ (owner-name injection) | n/a | n/a |
| `Shape.In` (module-level name) | ✓ | ✓ — the canonical subclass spelling (`HeightCostMapper`) | ✓ with import (`VoxelGridMapper.In` — the `ScanBatcher` pattern) |
| explicit string `"Shape.In"` without future import | ✓ | ✓ | ✓ |
| re-declared nested `In` in a subclass body, bare | ✓ — `vars(cls)` has it again | — | — |

Both annotation regimes are supported identically: eager (no
`from __future__ import annotations` — annotations are already objects) and
stringized (future import or explicit strings). The test matrix covers both
(§12.2).

### 4.3 Failure modes

`get_type_hints` evaluation can raise `NameError` (unknown name),
`AttributeError` (`CostMapper.Inn` typo), `SyntaxError` (malformed string),
or in principle anything (`In[int]` on a non-generic → `TypeError`).
`classify` catches **all exceptions** from the resolution call, chains the
original (`raise … from e`), and raises `[step-unresolvable]` with a guidance
ladder (§9) whose first line adapts to the failing name:

- name ∈ {`In`, `Out`, `State`} → the not-lexically-scoped lesson: "bare
  `In`/`Out`/`State` resolve only inside the class body that defines them; in
  a subclass write `Base.In`."
- otherwise → the deferred-annotation lesson: "step annotations resolve at
  import time against the defining module's globals — a name imported under
  `TYPE_CHECKING` or inside a function is invisible; move it to a module-level
  runtime import." (Cross-cutting rule already requires bundle *classes* to be
  runtime-importable.)

**Known limitation, documented and tested, not worked around:** a bundle
defined as a *function local* (e.g. at test-function scope, outside the module
class) is invisible to stringized annotations — `localns` has the class body,
`globalns` has the module, nothing has the enclosing function frame. mypy
accepts it; runtime cannot (PEP 563's classic gap). We do **not** walk stack
frames to compensate (fragile, explicitly discouraged). The error guidance
names both fixes: nest the bundle in the module class (house pattern — always
works, including for classes defined inside functions, which is how every
unit test defines modules), or move it to module scope. Pinned by
`test_resolve_function_local_bundle_fails`.

## 5. Validation rules per kind

Shared helper `_require_row(cls, tp, base, where)`: `tp` must be a class
(not a `TypeVar` → generics-are-gone message; not a union → one-row message;
not a parameterized alias → plain-class message; not a non-type) and a
**subclass of `base`** (`pm.In` or `pm.Out` from T1). `issubclass(pm.In,
pm.In)` is `True`, so the bases themselves technically pass — bundle
*internals* (fields, tick policy) are T1/T5's domain, not T3's (§10).
Row-ness errors: `[in-not-row]` / `[out-not-row]` with adaptive variants.

Union normalization (verified): `X | None`, `Optional[X]`, `Union[X, None]`
all yield `get_origin(...) ∈ {typing.Union, types.UnionType}` with args
`{X, NoneType}` — `_split_optional(tp) -> (inner, had_none)` handles all
three; a union with any non-`None` second member is not an optional and
falls through to the union error rows below.

### 5.1 STATELESS — `def step(self, i: In) -> Out [| None]`

- input: `hints[data_params[0].name]` → `_require_row(…, pm.In)`.
- return `r = hints["return"]`:

| `r` | Result |
| --- | --- |
| `NoneType` | **error `[step-returns-nothing]`** |
| union incl. exactly one non-`None` member `X` | `out = X`, `skips = True` |
| union with ≥2 non-`None` members | **error `[out-union]`** |
| origin ∈ {`Awaitable`, `Coroutine`} (sync def) | **error `[step-returns-awaitable]`** — one spelling per capability: respell as `async def`. mypy would structurally accept this against `AsyncStateless`; runtime deliberately rejects (§14 R3). |
| otherwise | `out = r`, `skips = False` |

- `out` → `_require_row(…, pm.Out)`.

### 5.2 ASYNC_STATELESS — `async def step(self, i: In) -> Out [| None]`

Same as 5.1 with one variant: the return hint of an `async def` is already
the awaited result (verified), so origin ∈ {`Awaitable`, `Coroutine`} means
the user double-wrapped — **error `[step-returns-awaitable]`** (async
variant: "the return annotation of `async def` is the awaited result — write
`-> Out`, not `-> Awaitable[Out]`").

### 5.3 MEALY — `def step(self, state: State, i: In) -> tuple[State, Out [| None]]`

Order of checks (fail-fast):

1. `cls.State` must exist (attribute lookup — inherited counts, so shape
   implementations inherit the base's State) → else **error
   `[mealy-no-state]`**.
2. `cls.State` must be a class; must **not** be a `pm.In`/`pm.Out` subclass →
   **error `[state-is-row]`**.
3. `state_ann = hints[data_params[0].name]` must be **identical** (`is`) to
   `cls.State` → else **error `[state-mismatch]`**, with two adaptive
   variants: swapped-order (second data param *is* `cls.State` and first is a
   `pm.In` row → "parameters are swapped: Mealy order is `(state, i)`") and
   shadowed-State (`owner is not cls` and `"State" in vars(cls)` → the
   inherited step is typed against the owner's State; override step or remove
   the shadowing declaration).
4. input: `hints[data_params[1].name]` → `_require_row(…, pm.In)`.
5. return `r`:

| `r` | Result |
| --- | --- |
| origin is not `tuple` (covers `typing.Tuple` — same origin, verified) | **error `[mealy-return]`** |
| bare `tuple` (no args) / args length ≠ 2 / `tuple[X, ...]` (Ellipsis) | **error `[mealy-return]`** |
| `args[0] is not cls.State` | **error `[mealy-return]`** (first-slot variant, names both types) |
| `args[1]` is `NoneType` | **error `[step-returns-nothing]`** |
| `args[1]` union → split | `skips = True` on `None` presence; ≥2 non-None → `[out-union]` |
| `args[1]` → `_require_row(…, pm.Out)` | done |

6. **Default-constructibility**: T6 starts every run from `State()`; T3 moves
   that failure to import time *by inspection, never by calling*:
   `inspect.signature(cls.State)` — if any parameter lacks a default →
   **error `[state-not-default-constructible]`**. Works uniformly for
   NamedTuple (verified: defaults reflected in the signature), dataclasses,
   and plain `__init__`s. If `inspect.signature` itself fails (exotic C
   types), the check is **skipped** (permissive on undeterminable, strict on
   determinable-and-wrong).

### 5.4 FOLD — `def fold(self, rows: Iterator[In]) -> Iterator[Out]`

Accepted origins mirror T4's `Fold` protocol variance exactly (params are
contravariant — wider is fine; returns are covariant — narrower is fine):

- `rows` annotation: origin ∈ {`collections.abc.Iterator`,
  `collections.abc.Iterable`} (typing spellings normalize to these via
  `get_origin`), exactly one type arg → element → `_require_row(…, pm.In)`.
  Anything else (incl. unparameterized, `Generator[…]` as a param —
  *narrower* than the `Iterator` the driver passes, which mypy rejects too) →
  **error `[fold-rows-param]`**.
- return annotation: origin ∈ {`collections.abc.Iterator`,
  `collections.abc.Generator`}; `Iterator[Out]` → 1 arg; `Generator[Out, S,
  R]` → 3 args, element is `args[0]` (any Generator is-an Iterator regardless
  of S/R). `Iterable` return is **rejected** with a dedicated variant
  ("declaring `Iterable` does not promise an `Iterator`; T4's protocol —
  and the driver — need `Iterator[Out]`"), matching mypy's verdict →
  **error `[fold-return]`**. Element → `_require_row(…, pm.Out)`.
- `skips = False` always (a fold controls its own emission; skipping is not
  a signature property).
- `state_type = None`; a declared `State` is caught by §5.5.

### 5.5 State coherence (all kinds)

After kind is known: if `kind is not MEALY` and `"State" in vars(cls)` (own
body only — an *inherited* State on a class that overrides step as stateless
is mypy's `[override]` problem, not T3's, §10):

- kind FOLD → **error `[fold-state]`** (doctrine message: fold state is
  generator-local; engine-threaded State is a step affair).
- kind STATELESS / ASYNC_STATELESS → **error `[state-unused]`**.

### 5.6 Bundle coherence (all kinds)

The signature is the authority; a nested declaration the signature doesn't
use is dead weight that will mislead readers and wiring. If `"In" in
vars(cls)` and `vars(cls)["In"] is not spec.in_type` → **error
`[bundle-shadowed]`**; same for `"Out"` vs `spec.out_type`. Two message
variants: same-class (delete or use the nested bundle) and inherited-step
(`owner is not cls`: the inherited step is typed against the owner's bundle;
override step or remove the shadowing declaration). Inherited nested bundles
(shape implementations) are exempt by construction — the check reads *own*
`vars` only, and for `HeightCostMapper` the attribute isn't in its own dict.
Additive-strictness decision flagged in §14 R5.

## 6. Abstractness and the no-instantiation guarantee

`CostMapper` — a step whose *body* raises `NotImplementedError` — is the
abstractness mechanism, and it is invisible to T3 **by design**:
classification reads functions' signatures and annotations, never bodies, so
a shape classifies as a perfectly normal STATELESS module and gets a
perfectly normal `StepSpec`. No `abc`, no `__abstractmethods__`, no
instantiation guard: "a shape is just a module whose step raises, and
abstractness is a doctrine, not a metaclass" (sketch). Attempting to *run* a
shape raises `NotImplementedError` at the first tick — runtime's honest
answer, out of T3's scope.

The hard guarantees, stated as implementation constraints and each pinned by
a test (§12.5):

1. **No instantiation** — of the module, its bundles, or its State.
   `State()` constructibility is checked via `inspect.signature`, never by
   calling (§5.3.6).
2. **No descriptor execution.** Every attribute read is via `vars(...)` on
   MRO entries or plain class-attribute lookup of the five reserved names
   (`step`, `fold`, `In`, `Out`, `State`) — for which class-level `getattr`
   returns the class/function without running user code. `@resource`
   descriptors (T7) are never touched: no `dir()`, no attribute enumeration,
   no `getattr` of any name outside the reserved five.
3. **No I/O, no imports of user code** beyond what evaluating the step's own
   annotations requires (that evaluation is mypy-equivalent work: resolving
   names the user wrote in a signature).
4. **No mutation of `cls`** — `classify` returns; T2 assigns (§8).

## 7. The output artifact — `StepSpec`

### 7.1 Definition

```python
@dataclass(frozen=True)
class StepSpec:
    kind: StepKind             # STATELESS | ASYNC_STATELESS | MEALY | FOLD
    in_type: type[Any]         # pm.In subclass; for FOLD: the rows element type
    out_type: type[Any]        # pm.Out subclass; for FOLD: the yielded element type
    state_type: type[Any] | None   # the class's State; None unless MEALY
    skips: bool                # return annotation admits None (MEALY: Out slot); always False for FOLD
    owner: type[Any]           # the MRO class whose body defines the step/fold function

    @property
    def impl_name(self) -> str: ...   # "fold" if kind is FOLD else "step"
    @property
    def arity(self) -> int: ...       # data params: 2 if MEALY else 1
    @property
    def is_async(self) -> bool: ...   # kind is ASYNC_STATELESS
```

`owner` earns its slot: it names where an inherited step came from in
downstream diagnostics ("step inherited from `CostMapper`"), and lets
tooling fetch the raw function (`vars(spec.owner)["step"]`) without
re-walking the MRO. The three properties are derivable views, deliberately
not stored — kind is the single source; they exist so consumers never
re-derive the mapping.

### 7.2 Storage and consumers

Stored by T2 (never by `classify`) at `cls.__pure_step__`
(`PURE_STEP_ATTR`), one entry **per class in its own `__dict__`** — a
config-only subclass gets its own (value-equal) spec recomputed from the
inherited step. Dunder-style name follows `__dataclass_fields__` precedent
and is naturally invisible to T2's config-field collection.

Invariant consumers may rely on: **`PURE_STEP_ATTR in vars(cls)` ⇔ the class
passed every definition-time check** (T2 assigns it as the last statement of
`__init_subclass__`, §8).

| Consumer | Reads |
| --- | --- |
| T4 runtime dispatch behind `over()` overloads | `kind` |
| T5 alignment | `in_type` (port derivation via T1's `fields()`) |
| T6 drivers | `kind`, `state_type` (start run from `State()`), `out_type`, `skips` (diagnostics: a `skips=False` step returning `None` at runtime is a contract anomaly — policy owned by T6, §15 Q4) |
| T8 rim | `in_type` / `out_type` (port handles) |
| T10 checkpoint | `kind` (FOLD/ASYNC refuse checkpoint), `state_type` |

### 7.3 Accessor

`step_spec(cls)` returns `vars(cls)[PURE_STEP_ATTR]`; if absent (not
inherited-lookup — *own dict*, enforcing the §7.2 invariant) raises
`PureModuleDefinitionError` `[not-classified]`. Takes a class, not an
instance (callers hold `type(m)`).

### 7.4 Stability guarantees

- Frozen, value-equality dataclass. Fields and properties above are **stable
  API from the first landing; evolution is additive-only** (new optional
  fields with defaults).
- `StepKind` members and their string values are stable (they appear in
  diagnostics).
- `in_type`/`out_type`/`state_type`/`owner` are class *objects* — identity is
  stable per import. After `importlib.reload`, a re-created class gets a
  fresh spec with fresh class objects; the engine already keys members by
  path + config, never by class object (sketch reload note), so this is
  correct behavior, not a caveat to fix.
- **Not** a serialization format: it contains classes. T10 checkpoints store
  qualnames + config dumps, never a `StepSpec`.
- Mutating `step`/`fold`/`In`/`Out`/`State` on a class after creation is
  undefined behavior; the spec reflects definition time.

## 8. The T2 seam

Contract with `module.py` (T2 places this; T3 defines it):

```python
# module.py — T2-owned file
from dimos.pure.stepspec import classify   # module-level import; stepspec never imports module.py

class PureModule:
    def __init_subclass__(cls, **kwargs: Any) -> None:
        super().__init_subclass__(**kwargs)
        # … T2: config-field collection, pydantic model synthesis, frozen wiring …
        cls.__pure_step__ = classify(cls)   # LAST statement — see ordering rule
```

Six requirements on T2:

1. **`classify` runs for every subclass** — including shape implementations
   (`HeightCostMapper` revalidates and stores its own spec) and config-only
   subclasses. `__init_subclass__` gives this for free; T2 must not
   short-circuit it.
2. **The assignment is the last statement** of `__init_subclass__`, after
   config synthesis. The two are independent (classification never reads
   config; config never reads bundles), so the order is fixed by the §7.2
   invariant alone: the attribute's presence must certify that *all*
   definition-time machinery (config included) succeeded. It also yields
   deterministic error precedence: config errors before shape errors.
3. **`PureModule` defines neither `step` nor `fold`** — not even an
   abstract placeholder — and pre-defines none of `In`/`Out`/`State`.
   G0's discovery depends on it (a base-provided `step` would make
   `[step-missing]` unreachable and corrupt owner attribution).
4. `__init_subclass__` runs `classify` for direct *and* indirect subclasses
   naturally; PureModule itself is never classified (Python semantics — the
   hook doesn't fire for the defining class).
5. T2's config collector must not treat `__pure_step__` as a field (it's
   unannotated and dunder — naturally excluded) and must leave the five
   reserved names to T3.
6. `PureModuleDefinitionError` is importable from `stepspec.py`; T2 **may**
   reuse it for config-shape violations (recommended for a single
   user-facing "your module definition is wrong" type) or define its own —
   T2's call, no coupling either way.

Timing note (verified): `__init_subclass__` fires inside `type.__new__`,
after the namespace is complete and after `__set_name__` — so `vars(cls)` is
final — but **before** the class name is bound in the enclosing module —
hence §4.1's owner-name injection.

## 9. The error catalog

### 9.1 The exception type

```python
class PureModuleDefinitionError(TypeError):
    """A PureModule subclass violated a definition-time shape rule."""
```

Lives in `dimos/pure/stepspec.py`. Subclasses `TypeError`: a mis-shaped class
is a type error in spirit and precedent (dataclasses' bad field order, MRO
conflicts). Raised during class creation, so the traceback ends at the
user's own `class` statement — the same moment a bad dataclass fails today.
When resolution failure wraps an underlying exception, chain it
(`raise … from e`).

Machine-readable rule identity: construct with the `Rule`; the instance
exposes `.rule: Rule` and the message ends with the slug in brackets,
mypy-style. Tests assert on `.rule`, humans grep the slug.

### 9.2 Message format

```
{cls.__module__}.{cls.__qualname__}: {problem}. {fix}. [{slug}]
```

One line where possible; `[step-unresolvable]` is the one sanctioned
multi-line message. Every message names the class, states the violated rule,
and states the fix — release copy, verbatim below. `{X}` = the offending
class's qualname (module prefix per the format above); other placeholders
per message. Where a rule has variants, all variants share one slug (tests
assert per-variant substrings).

### 9.3 The catalog — verbatim

`Rule` member names are the slugs upper-snaked (`STEP_MISSING = "step-missing"`).

**`[step-missing]`** — G0
> `{X}: defines neither step nor fold. A pure module declares exactly one: def step(self, i: In) -> Out for per-tick, or def fold(self, rows: Iterator[In]) -> Iterator[Out] for whole-stream. An abstract shape declares its In/Out and a step whose body raises NotImplementedError. [step-missing]`

**`[step-and-fold]`** — G0
> `{X}: defines both step (from {step_owner}) and fold (from {fold_owner}) — a module is per-tick or whole-stream, never both. Remove one (rename it if it is a helper: step, fold, In, Out and State are reserved). [step-and-fold]`

**`[step-not-function]`** — G1/G3; four variants, one slug
> staticmethod: `{X}: step must be a plain instance method — def step(self, i: In) -> Out — not a staticmethod. It needs self for config and resources. [step-not-function]`
> classmethod: `{X}: step must be a plain instance method — def step(self, i: In) -> Out — not a classmethod. [step-not-function]`
> other: `{X}: step must be a function defined in the class body, got {type_name}. (step, fold, In, Out and State are reserved member names.) [step-not-function]`
> async generator: `{X}: step is an async generator — a step maps one tick to one result. For whole-stream processing write fold (sync); async concurrency stays per-tick: async def step(self, i: In) -> Out. [step-not-function]`

**`[step-no-self]`** — G2
> `{X}: step takes no parameters — expected def step(self, i: In) -> Out. [step-no-self]`

**`[step-params]`** — G2
> `{X}: step parameter '{name}' is {kind}; step parameters must be plain positional: (self, i) or (self, state, i). The engine calls step positionally. [step-params]`
> (`{kind}` ∈ "*args" / "**kwargs" / "keyword-only" — positional-only is legal, see G2 amendment.)

**`[step-param-default]`** — G2
> `{X}: step parameter '{name}' has a default. The engine always passes every argument, and defaults change how annotations resolve across Python versions — remove it. [step-param-default]`

**`[step-arity]`** — G3; two variants
> zero rows: `{X}: step takes no input row — expected def step(self, i: In) -> Out.{maybe_forgot_self} [step-arity]`
>   where `{maybe_forgot_self}` = ` (Its first parameter '{name}' is a pm.In row — did you forget self?)` when the heuristic fires, else empty.
> too many: `{X}: step takes {n} inputs — a step takes exactly one In row (Mealy: state first, then the row). Multiple inputs are fields of one In bundle: class In(pm.In) with one field per input. [step-arity]`

**`[async-mealy]`** — G3
> `{X}: async def step cannot take State — in-flight ticks would race the state thread. Make step sync, or split: an async stateless module for the I/O feeding a sync Mealy module for the state. [async-mealy]`

**`[fold-arity]`** — G3
> `{X}: fold takes {n} data parameters — expected exactly def fold(self, rows: Iterator[In]) -> Iterator[Out]. Fold state lives in generator locals, not parameters. [fold-arity]`

**`[fold-async]`** — G3
> `{X}: fold cannot be async — fold owns its loop synchronously. Async concurrency is a per-tick policy: async def step(self, i: In) -> Out. [fold-async]`

**`[step-unannotated]`** — G4; two variants
> `{X}: step parameter '{name}' has no annotation. The step signature is the module's whole contract — annotate it with the row type. [step-unannotated]`
> `{X}: step has no return annotation. The step signature is the module's whole contract — annotate the return: -> Out, -> Out | None, or tuple[State, Out] for Mealy. [step-unannotated]`

**`[step-unresolvable]`** — G4; the sanctioned multi-line message
> ```
> {X}: cannot resolve step's annotations: {underlying error!r}.
> {lesson}
> Annotations resolve at import time against the defining module's globals plus the defining class body.
> ```
> `{lesson}` when the failing name ∈ {In, Out, State}:
> `Bare {name} resolves only inside the class body that defines it — class attributes are not lexically scoped. In a subclass write {owner}.{name} (or redeclare the bundle in this class body).`
> `{lesson}` otherwise:
> `If '{name}' is imported under TYPE_CHECKING or inside a function, move it to a module-level runtime import — deferred annotations cannot see function locals or checking-only imports.`
> (Original exception chained via `raise … from e`. Substitute `fold` for `step` in fold contexts — true for every message in this catalog.)

**`[in-not-row]`** — G5; adaptive variants, one slug
> base: `{X}: step input '{name}' is {t}, not a pm.In row. Declare class In(pm.In) with one field per input and annotate step with it (bare In inside the defining body; {X}.In elsewhere). [in-not-row]`
> union: `{X}: step input '{name}' is a union ({t}) — a step takes exactly one In row type. Optional inputs are latest(default=None) fields inside the bundle, not unions of bundles. [in-not-row]`
> TypeVar: `{X}: step input '{name}' is a TypeVar ({t}) — module classes are not generic by design; annotate with a concrete pm.In row. [in-not-row]`
> parameterized: `{X}: step input '{name}' is a parameterized alias ({t}) — rows are plain classes; annotate with the row class itself. [in-not-row]`

**`[out-not-row]`** — G5; same four variants with "returns" phrasing
> base: `{X}: step returns {t}, not a pm.Out row. Declare class Out(pm.Out) and construct it in step — effects are outputs too. [out-not-row]`
> (union/TypeVar/parameterized variants mirror `[in-not-row]`.)

**`[out-union]`** — G5
> `{X}: step returns {t} — a step returns exactly one Out row type, plus | None if some ticks skip. Alternative results are sparse fields of one Out bundle. [out-union]`

**`[step-returns-nothing]`** — G5
> `{X}: step returns None — a module that never emits has no outputs to wire. Return an Out row; use -> Out | None if some ticks skip; effects are outputs too. [step-returns-nothing]`

**`[step-returns-awaitable]`** — G5; two variants
> sync: `{X}: step returns {t} but is not async. One spelling per capability: declare it async def step(self, i: In) -> Out and return the row. [step-returns-awaitable]`
> async: `{X}: async step's return annotation is the awaited result — write -> Out, not -> {t}. [step-returns-awaitable]`

**`[mealy-no-state]`** — §5.3
> `{X}: step takes (state, i) but {X} declares no State. Declare class State(NamedTuple) with a default for every field — a run starts from State(). [mealy-no-state]`

**`[state-mismatch]`** — §5.3; three variants
> base: `{X}: step's state parameter is {t} but {X}.State is {s} — the state parameter must be the class's own State. [state-mismatch]`
> swapped: `{X}: step's parameters are swapped — Mealy order is def step(self, state: State, i: In). [state-mismatch]`
> shadowed: `{X}: declares State but inherits step from {owner}, which is typed against {owner}.State. Override step or remove the State declaration. [state-mismatch]`

**`[state-is-row]`** — §5.3
> `{X}: State is a row bundle (subclass of {base}) — State is engine-threaded plain data (e.g. NamedTuple); rows are dataflow. Give State its own class. [state-is-row]`

**`[state-not-default-constructible]`** — §5.3
> `{X}: State() must construct with no arguments — a run starts from State(), and field '{name}' has no default. Give every State field a default. [state-not-default-constructible]`

**`[mealy-return]`** — §5.3; two variants
> shape: `{X}: a Mealy step returns tuple[State, Out] (or tuple[State, Out | None] to skip), got {t}. [mealy-return]`
> first slot: `{X}: Mealy step's return tuple carries {t} first, but the state slot must be {X}.State ({s}). [mealy-return]`

**`[fold-rows-param]`** — §5.4
> `{X}: fold's rows parameter is {t} — expected Iterator[In] (Iterable[In] also accepted). The driver hands fold one lazy pass over the aligned rows. [fold-rows-param]`

**`[fold-return]`** — §5.4; two variants
> base: `{X}: fold must return Iterator[Out] (a generator function qualifies), got {t}. [fold-return]`
> Iterable: `{X}: fold declares -> Iterable[{t}] — declaring Iterable does not promise an Iterator, and the driver consumes a single pass. Declare -> Iterator[{t}]. [fold-return]`

**`[state-unused]`** — §5.5
> `{X}: declares State but step is {stateless|async stateless} — State is threaded only through a Mealy step: def step(self, state: State, i: In) -> tuple[State, Out]. Take it or delete it. [state-unused]`

**`[fold-state]`** — §5.5
> `{X}: declares State but fold keeps its state in generator locals — engine-threaded State is a step affair. Delete State; if you need checkpointable state, respell as a Mealy step. [fold-state]`

**`[bundle-shadowed]`** — §5.6; two variants
> same-class: `{X}: declares a nested {In|Out} but step reads {t} — the step signature is the single typing authority, so the nested declaration is dead. Use it in step or delete it. [bundle-shadowed]`
> inherited-step: `{X}: declares {In|Out} but inherits step from {owner}, which is typed against {owner}.{In|Out}. Override step or remove the declaration. [bundle-shadowed]`

**`[not-classified]`** — accessor (§7.3)
> `{X}: has no step spec — it was not validated as a PureModule subclass. Only classes that ran PureModule.__init_subclass__ carry one. [not-classified]`

## 10. What T3 does NOT check — division of labor

| Concern | Owner | Note |
| --- | --- | --- |
| Override conformance (narrowed `In`, widened `Out`, kind change in a subclass) | **mypy** `[override]` | The sketch's shape story leans on def-site mypy; T3 re-implementing Liskov (variance over bundles) would be a parallel type checker. T3's only override-adjacent checks are the cheap coherence rules (§5.3.3, §5.5, §5.6). An inherited-State + stateless-override combo is `[override]` territory, not T3's. |
| Bundle internals: field specifiers, kw-only `ts`, defaults, specifier-as-required | **T1** | T3 checks `issubclass` of the bases, nothing inside. |
| Exactly-one-`tick()`-per-In policy, alignment resolvability | **T5** | Wiring-time policy, per index. A field-less `In` passes T3 and is rejected at wiring. |
| Config fields, frozen-ness, constructor typing | **T2** | Runs before the T3 hook (§8.2). |
| Step body correctness, `NotImplementedError` shapes | **mypy / runtime** | Bodies are never read (§6). |
| Protocol matching at engine call sites (`Invalid self argument`) | **mypy + T4 harness** | T3 is the runtime mirror; T4's static tests pin the mypy side. |
| Whether a `skips=False` step returning `None` at runtime is skip or error | **T6** | `StepSpec.skips` is the input to that policy (§15 Q4). |

## 11. Edge cases — index "watch out for" → spec mapping

| Index bullet / edge | Where handled | Test (§12) |
| --- | --- | --- |
| Bare `In`/`Out` resolve only in defining body | §4.2, `[step-unresolvable]` lesson | `test_resolve_bare_in_subclass_fails`, `test_resolve_bare_nested_*` |
| Subclass overrides use `Shape.In` | §4.2 | `test_resolve_shape_qualified_in_subclass` |
| Cross-module bundle references | §4.1(2) | `test_resolve_cross_module_bundle` |
| String / `from __future__ import annotations` | §4.2 matrix | `test_resolve_bare_nested_eager` vs `_stringized`, `test_resolve_explicit_string_annotations` |
| Unresolvable name → which error, what guidance | §4.3, §9 `[step-unresolvable]` | `test_err_step_unresolvable_*` |
| Error messages are the product | §9 (verbatim catalog) | one test per slug |
| Validate, don't instantiate | §6 | `test_classify_never_instantiates`, `test_classify_no_mutation` |
| Self-qualified name in own body (`Tagger.In` inside `Tagger`) | §4.1(3) | `test_resolve_self_qualified_in_own_body` |
| Inherited step on config-only subclass | §4.1(1), §7.2 | `test_config_only_subclass_inherits_owner` |
| Shape (raising step) validates normally; impls revalidate | §6, §8.1 | `test_classify_shape_abstract`, `test_classify_shape_impl_inherits` |
| Mealy skip form `tuple[State, Out \| None]` | §5.3 | `test_classify_mealy_skips` |
| `Optional[X]` vs `X \| None` vs `Union[X, None]` | §5 preamble (verified normalization) | `test_classify_optional_spellings` |
| Sync step returning `Awaitable` (mypy accepts, runtime rejects) | §5.1, §14 R3 | `test_err_step_returns_awaitable_sync` + T4 static twin |
| Function-local bundle under PEP 563 | §4.3 limitation | `test_resolve_function_local_bundle_fails` |
| Module class defined inside a function (test pattern) | §4.3 | `test_resolve_class_defined_in_function` |
| Defaults change `get_type_hints` across 3.10/3.11 | §G2 rationale | `test_err_step_param_default` |
| Decorated / wrapped steps | §G1 note | `test_classify_wrapped_step` |
| `typing.Tuple`/`typing.Iterator` spellings | §5.3.5, §5.4 (origin normalization) | `test_classify_typing_spellings` |
| Reserved names collision (`step = 3`, helper named `fold`) | §2, `[step-not-function]` / `[step-and-fold]` | `test_err_step_not_function_attr`, `test_err_step_and_fold_helper` |
| Generator `step` (sync `def` with `yield`) | §G3 row (falls to G5 return rules) | `test_err_generator_step` |
| `importlib.reload` / class identity | §7.4 | (doc-pinned; engine keys by path — no T3 test) |

## 12. Test plan

`dimos/pure/test_stepspec.py`. Modules under test are defined *inside test
functions* with nested bundles (which itself exercises §4.3's supported
pattern); `classify` is called directly — **no T2, no engine imports**
(tests-need-no-engine property). Until implementation lands the file is
module-level skip-marked. mypy excludes `test_*` (repo config) — no
annotation burden. Fixture modules for cross-module and eager-annotation
cases are created via `types.ModuleType` + `exec` + `sys.modules`
registration (verified recipe: exec'd functions carry the module dict as
`__globals__`).

### 12.1 Classification happy paths (the sketch seven + floor)

- `test_tagger_floor_zero_ceremony` — **the API-floor guard**: the sketch's
  4-declaration Tagger shape (nested In/Out, bare-name step) classifies
  STATELESS with correct `in_type`/`out_type`/`skips=False`/`owner`, zero
  added ceremony.
- `test_classify_stateless_multi_out` — QualityGate shape (sparse Out field
  is T1's business; T3 sees a normal Out).
- `test_classify_mealy` — VoxelGridMapper shape: MEALY, `state_type` is the
  nested State, `skips=True` (returns `tuple[State, Out | None]`).
- `test_classify_mealy_no_skip` — `tuple[State, Out]` → `skips=False`.
- `test_classify_shape_abstract` — CostMapper (raising body) classifies
  STATELESS; no instantiation occurred.
- `test_classify_shape_impl_inherits` — HeightCostMapper pattern:
  override spelled `Shape.In`/`Shape.Out`, own spec, `owner` is the subclass.
- `test_classify_async_stateless` — Captioner shape; `is_async` property.
- `test_classify_fold_cross_module` — ScanBatcher pattern:
  `Iterator[Other.In] -> Iterator[Other.Out]`, no own bundles.
- `test_classify_fold_plain_function` — non-generator fold returning an
  iterator expression is FOLD (annotation is the contract).
- `test_config_only_subclass_inherits_owner` — `Tuned(Tagger)` classifies;
  `owner` is Tagger; spec value-equal to Tagger's.
- `test_classify_optional_spellings` — `X | None` / `Optional[X]` /
  `Union[X, None]` all → `skips=True`, same `out_type`.
- `test_classify_typing_spellings` — `typing.Tuple` / `typing.Iterator` /
  `typing.Generator` forms classify identically to builtin/abc spellings.
- `test_classify_wrapped_step` — `functools.wraps`-decorated step classifies
  from the wrapper's preserved annotations.
- `test_classify_generator_fold_with_generator_return` —
  `-> Generator[Out, None, None]` accepted, element extracted.

### 12.2 Resolution matrix

- `test_resolve_bare_nested_stringized` — future-import module, bare `In`.
- `test_resolve_bare_nested_eager` — no future import (eager objects).
- `test_resolve_self_qualified_in_own_body` — `Tagger.In` inside Tagger.
- `test_resolve_shape_qualified_in_subclass` — `Base.In` in an override.
- `test_resolve_cross_module_bundle` — bundle imported from a second
  (exec-created) module.
- `test_resolve_explicit_string_annotations` — `"Shape.In"` literals, no
  future import.
- `test_resolve_redeclared_bare_in_subclass` — subclass redeclares nested
  `In` (subclassing the base's) and overrides step with it — but note this
  trips §5.6 unless step reads it; the test asserts the *resolution*
  succeeds and coherent redeclaration classifies.
- `test_resolve_class_defined_in_function` — canonical nested-bundle module
  inside a `def` passes (the pattern every other test uses, pinned
  explicitly).
- `test_resolve_bare_in_subclass_fails` — bare `In` in subclass body →
  `[step-unresolvable]`, message contains the `Base.In` guidance.
- `test_resolve_function_local_bundle_fails` — function-local bundle +
  stringized annotations → `[step-unresolvable]`, guidance names the fix
  (§4.3 limitation pinned).
- `test_resolve_type_checking_import_fails` — TYPE_CHECKING-only name in a
  step annotation → `[step-unresolvable]`, guidance mentions runtime import.
- `test_resolve_attribute_typo` — `Base.Inn` → `[step-unresolvable]` with
  chained `AttributeError`.

### 12.3 Error catalog — one test per slug

For each: assert `PureModuleDefinitionError`, `.rule` is the `Rule` member,
message contains the class qualname and the slug. Variants asserted by
substring. Names: `test_err_step_missing`, `test_err_step_and_fold`,
`test_err_step_and_fold_helper` (helper named fold),
`test_err_step_not_function_static`, `test_err_step_not_function_classmethod`,
`test_err_step_not_function_attr`, `test_err_generator_step`,
`test_err_async_generator_step`, `test_err_step_no_self`,
`test_err_step_params_kwonly`, `test_err_step_params_varargs`,
`test_err_step_params_posonly`, `test_err_step_param_default`,
`test_err_step_arity_zero`, `test_err_step_arity_zero_forgot_self`,
`test_err_step_arity_many`, `test_err_async_mealy`, `test_err_fold_arity`,
`test_err_fold_async`, `test_err_fold_async_generator`,
`test_err_step_unannotated_param`, `test_err_step_unannotated_return`,
`test_err_step_unresolvable_bare_in`, `test_err_in_not_row`,
`test_err_in_not_row_union`, `test_err_in_not_row_typevar`,
`test_err_out_not_row`, `test_err_out_union`,
`test_err_step_returns_nothing`, `test_err_mealy_returns_nothing`,
`test_err_step_returns_awaitable_sync`,
`test_err_step_returns_awaitable_async`, `test_err_mealy_no_state`,
`test_err_state_mismatch`, `test_err_state_mismatch_swapped`,
`test_err_state_mismatch_shadowed`, `test_err_state_is_row`,
`test_err_state_not_default_constructible`, `test_err_mealy_return_shape`,
`test_err_mealy_return_first_slot`, `test_err_fold_rows_param`,
`test_err_fold_return`, `test_err_fold_return_iterable`,
`test_err_state_unused`, `test_err_fold_state`,
`test_err_bundle_shadowed`, `test_err_bundle_shadowed_inherited`.

### 12.4 Seam and accessor

- `test_seam_recipe` — a local `FakePureModule` whose `__init_subclass__`
  assigns `classify(cls)` per §8: valid subclass gets `__pure_step__` in its
  own dict; **invalid subclass raises at the `class` statement**
  (`pytest.raises` around the class definition) — the import-fails-loudly
  moment, demonstrated without T2.
- `test_step_spec_accessor` / `test_step_spec_accessor_unclassified`
  (`[not-classified]`; also: inherited-only attribute does not satisfy the
  own-dict rule).

### 12.5 Purity guarantees

- `test_classify_never_instantiates` — module whose State `__init__` raises
  if called, whose resource-like descriptor `__get__` raises if touched, and
  bundle classes that record construction: `classify` succeeds, nothing ran.
- `test_classify_no_mutation` — `vars(cls)` keys and values identical before
  and after `classify`.

### 12.6 Static-typing counterparts (T4's harness — listed here, built there)

- Narrowed-In / widened-Out override → mypy `[override]` (shape conformance
  at the def).
- Malformed steps from §12.3 that mypy *can* see → `Invalid self argument`
  at `.over()` (documents which rules have static twins and which are
  runtime-only: e.g. `[state-not-default-constructible]`,
  `[bundle-shadowed]`, `[step-unresolvable]` are runtime-only).
- Sync-step-returning-Awaitable: mypy accepts (AsyncStateless match), runtime
  rejects — the divergence is *documented* by a static test asserting the
  mypy acceptance next to the runtime rejection test.
- Recommendation to T4 (from §15 Q3): protocol params spelled positional-only
  (`def step(self, state: TState, i: TIn, /) -> …`) so implementations keep
  parameter-name freedom. **Resolved: T4 ACCEPTED** (t4-typing.md §2.3,
  pinned by `case_param_names.py`); G2 amended — positional-only impl params
  are legal, no param-name rule needed.

## 13. Acceptance criteria

- [x] `uv run mypy dimos/pure/stepspec.py` clean (strict, Python floor 3.10 —
      no `Self`, no `StrEnum`, `typing_extensions` only if needed).
- [x] All seven sketch shapes classify correctly (§12.1 green), including
      the abstract CostMapper and the cross-module ScanBatcher.
- [x] The 4-declaration Tagger classifies with zero added ceremony (API
      floor).
- [x] Every `Rule` member has: a message template implemented verbatim from
      §9.3, and at least one test in §12.3.
- [x] Both annotation regimes (eager + stringized) pass the full resolution
      matrix; explicit-string annotations pass.
- [x] `classify` is pure: no instantiation, no descriptor execution, no
      mutation (§12.5 green).
- [x] `classify` works on plain classes with no `PureModule`/T2 import
      anywhere in `stepspec.py` (seam test §12.4 green without T2).
- [x] Classification is identical on 3.10 and 3.12 for every test module
      (the no-defaults rule closes the known divergence; CI floor run
      suffices). (Implemented + verified on 3.12; the 3.10 half rides the
      CI floor run — appendix N8.)
- [x] Error precedence is deterministic and documented (= gate order G0–G5).
- [x] `StepSpec` fields exactly as §7.1; `PURE_STEP_ATTR == "__pure_step__"`.
- [x] No file other than `stepspec.py` / `test_stepspec.py` modified by the
      T3 implementation (module.py line is T2's). (Ticked under the amended
      integration scope — appendix N1: this landing also activates the seams,
      updates test_config.py, and assembles the `pm` surface, per the
      orchestrator's T3 integration mandate.)

## 14. Relitigation record

Deviations or additions vs the index text, per the never-deviate-silently
rule. R1 was mandated by the task assignment; the rest are this spec's calls.

- **R1 — `stepspec.py` file split** (index colocates T3 in `module.py`).
  Better: disjoint T2/T3 ownership, smaller module.py. Breaks: nothing;
  blast radius: index layout table line + one import in module.py.
- **R2 — resolution formula refined**: owner's `vars` (not `cls`'s) +
  owner-name injection (§4.1). The index's literal formula mis-resolves
  inherited steps and rejects self-qualified spellings mypy accepts. Blast
  radius: none outside T3; strictly accepts more mypy-valid code.
- **R3 — sync step returning `Awaitable` is rejected** even though mypy's
  structural AsyncStateless would accept it. One-spelling-per-capability
  doctrine; the runtime driver contract (T6) is built for `async def`.
  Blast radius: T4 documents the divergence (§12.6); trivially relaxable
  later (accept + classify ASYNC_STATELESS) without breaking anything.
- **R4 — `State()` default-constructibility checked at definition time**
  (index lists it nowhere; T6 implies it). Moves a guaranteed run-start
  failure to import, by inspection only. Blast radius: none; permissive on
  undeterminable signatures.
- **R5 — declared-but-unread nested bundles are errors** (`[bundle-shadowed]`,
  §5.6). Additive strictness beyond the index. Rationale: the
  signature-is-authority decision makes a dead nested bundle actively
  misleading. Blast radius: none on the sketch seven (all coherent); could
  annoy hypothetical deliberate-alias patterns — relax to a warning if a
  real case appears.
- **R6 — no abstract-base escape hatch**: a class with neither step nor fold
  errors even if meant as a config-only mixin. Doctrine-consistent (shapes
  declare their contract); escape hatch (`class Base(PureModule,
  abstract=True)`) is deferred to §15 Q1 rather than built speculatively.

## 15. Open questions (for review, none blocking)

1. **Config-only mixin bases** — is `[step-missing]` acceptable for them, or
   does T2 want an `abstract=True` class kwarg that skips classification
   (and instantiation)? Cheap to add later; not built now.
2. **`pm` surface exports** — RESOLVED: the T3 landing assembled
   `__init__.py` (first assembler wins) and adopted the §1 recommended set
   verbatim (`PureModuleDefinitionError`, `StepSpec`, `StepKind`,
   `step_spec`); `Rule` and `classify` stay importable from
   `dimos.pure.stepspec` only.
3. **T4 protocol parameter names** — RESOLVED: T4 adopted positional-only
   protocol params (t4-typing.md §2.3); G2 amended accordingly, no
   param-name rule.
4. **Runtime `None` from a `skips=False` step** — T6 policy (skip + health
   count vs error). `StepSpec.skips` is provided either way.
5. **T2 reuse of `PureModuleDefinitionError`** for config violations —
   RESOLVED yes (t2-config.md §10.1); the flip was performed with this
   landing: `ConfigFieldError(PureModuleDefinitionError)`, unified-base test
   in `test_config.py`.

## Implementation notes

Deviations / decisions made during implementation, per the
never-deviate-silently rule. None alter the spec's intent.

- **N1 — amended scope (orchestrator T3 integration mandate).** Beyond
  `stepspec.py`/`test_stepspec.py`, this landing activates the §8 seam in
  `module.py` (import + `cls.__pure_step__ = classify(cls)` as the LAST
  statement, plus a `__pure_step__: ClassVar[StepSpec]` declaration so the
  assignment is mypy-clean under strict), performs the t2-config.md §10.1
  error-base flip in `config.py`, gives every definition-succeeding
  `test_config.py` class a minimal valid step (shared `_In`/`_Out` rows +
  `def step`; classes that raise `ConfigFieldError` need none — config errors
  deterministically precede classification), and assembles the `pm` surface
  (`__init__.py` + `test_pm_surface.py` smoke). T2's suite grew 29 → 31
  (unified-base test; VoxelGridMapper-style construction, its §14 box).
- **N2 — posonly test conversion (amended G2).** The stub
  `test_err_step_params_posonly` predates the G2 amendment (T4 adopted
  positional-only protocol params, t4-typing.md §2.3): positional-only step
  params are legal. Converted to `test_classify_posonly_step`, pinning that
  `/`-spelled stateless and Mealy steps classify identically to their plain
  twins.
- **N3 — `PureModuleDefinitionError.__init__` takes `rule: Rule | None =
  None`** (skeleton had it required). Required by T2's one-line
  `ConfigFieldError` re-base: config machinery constructs with a message
  alone. Every T3-raised instance carries its `Rule`; `.rule` is typed
  `Rule | None`.
- **N4 — private-helper signatures extended keyword-only.** `_require_row`
  gained `*, impl: str = "step"` and `_resolve_step_hints` gained
  `*, name: str = "step"` to implement §9.3's "substitute `fold` for `step`"
  note in the shared messages; the skeleton's positional shapes are preserved
  (all 4-positional calls remain valid).
- **N5 — mirrored `[out-not-row]` variant copy.** §9.3 gives the base variant
  verbatim and says union/TypeVar/parameterized "mirror `[in-not-row]`" with
  returns phrasing; the mirrored union variant borrows `[out-union]`'s
  teaching sentence ("Alternative results are sparse fields of one Out
  bundle") since exact copy was unspecified.
- **N6 — `[step-unresolvable]` details.** The slug is appended to the final
  line (per §9.1's ends-with-slug rule; the §9.3 block elides it). When the
  underlying exception exposes no `.name` (e.g. `SyntaxError`), the
  otherwise-lesson substitutes "a name in the annotation" for `'{name}'`.
- **N7 — two literal-vs-normalized annotation forms handled.** Builtin
  `tuple[State, None]` keeps the literal `None` object in `get_args`
  (`typing.Tuple` normalizes to `NoneType`) — both are treated as the None
  slot (§5.3.5). And `State()` constructibility ignores
  `VAR_POSITIONAL`/`VAR_KEYWORD` parameters (a bare `*args`/`**kwargs` does
  not block `State()`), refining §5.3.6's "any parameter lacks a default".
- **N8 — 3.10 parity is design-guaranteed, CI-verified.** The no-defaults
  rule (G2) closes the known `get_type_hints` divergence; this landing ran on
  3.12 (repo interpreter), the 3.10 half rides the CI floor run per the §13
  box text.
- **N9 — test-module import hygiene.** `test_stepspec.py` imports the row
  bases as `PmIn`/`PmOut` so bare `In`/`Out` never resolve accidentally from
  the test module's globals — with plain imports, `get_type_hints`'s
  globalns fallback would silently satisfy the §12.2 negative cases
  (`issubclass(In, In)` is true).
