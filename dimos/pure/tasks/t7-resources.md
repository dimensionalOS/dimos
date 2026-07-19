# T7 — Resources (`@resource`)

Task boundary (index §T7): the `@resource` descriptor. OUTSIDE a run it is a
lazy cached property — tests just touch `m.grid`. INSIDE a run, resources are
created at warmup in declaration order and disposed in reverse order at stop;
disposal sniffs `dispose()` / `close()` / `aclose()` in that order; `over()`
teardown uses the same path.

Source of truth: `dimos/memory2/puremodule_api_sketch3.py` §3
(VoxelGridMapper — `@resource def grid`, "engine calls .dispose() at
teardown"), §5 (RelocalizationModule — `premap` may be `None`, plain data),
§6 (Captioner — sync factory, async `aclose`, "engine awaits aclose() at
stop"), tests section (lazy touch, no engine), deployment section.
`puremodule_api_sketch3_rpc.py` §3 doctrine ("a mutating handler cannot
touch @resource — the engine flags handler context; resource access raises")
is a HOOK obligation only; RPC itself is out of scope.

Landed seams this spec attaches to (binding upstream contracts):

- `module.py` / t2-config.md §6 — module instances are FROZEN;
  `__setattr__`/`__delattr__` raise unconditionally; machinery writes go
  through `object.__setattr__`, documented as "the internal seam used by
  `__init__` and by T7's resource cache".
- `drivers.py` / t6-drivers.md §8.4 (D9), §8.2–8.3, §9 — `RunHooks.teardown`
  (sync, called last in `_finalized`, every kind, exactly once per §8.3) and
  `RunHooks.ateardown` (async, awaited on the run's still-live loop after
  task quiescence). §8.1 acquisition invariant: nothing is acquired before
  the first `next()` of the returned iterator.
- `config.py` / t2-config.md §3.2 + §3.3.7 — descriptors are structurally
  not config fields; `test_typing_fixtures/case_config_kwargs.py` pins
  `@resource`-not-a-field statically and must keep passing.
- `stepspec.py` §6 purity — classification never executes descriptors (raw
  `vars` access only); T7 discovery must uphold the same rule.

---

## 1. Scope and layer boundary

`dimos/pure/resources.py` — one new file. It owns:

1. `Resource[T]` — the descriptor class — and `resource`, the decorator
   (bare `@resource` and `@resource(dispose=False)`).
2. The lazy per-instance cache (test mode, outside runs).
3. `RunResources` — the per-RUN resource context: ordered creation, reverse
   disposal, partial-warmup unwind, sniffing.
4. `attach_resources(module, hooks, *, async_run)` — the run-attach seam
   function `run_over` calls.
5. The handler-context flag (`IN_HANDLER`) for the future RPC layer.

**Imports** (direction justified, no cycles):

- Data layer: `stepspec` (for `PureModuleDefinitionError` — same single
  user-facing definition-error type T2 reused, t3-validation.md §8 req 6).
- Engine: `drivers` at MODULE scope, for `RunHooks` (typing) and
  `PureModuleRunError` (the runtime base — a resource failure IS a run
  failure; callers catching `PureModuleRunError` around a run must catch
  it). Acyclic because the reverse edge is lazy: `drivers` imports
  `resources` only inside `run_over`'s body (§6.1), exactly the pattern of
  drivers' lazy `align` import. Import chain at `import
  dimos.pure.resources` time: resources → drivers → {rows, stepspec,
  typing} — terminates, no cycle.
- NEVER `module.py` / `config.py` (resources.py works on any object with a
  `__dict__`; the frozen seam is `object.__setattr__`, not a PureModule
  API). NEVER `dimos.core` (§8.6), NEVER `align`.

`drivers.py` receives a small named seam edit (§6 — the T7 RESOURCE SEAM),
spec'd here, applied by the implementer. No other file changes hands except
the `pm` surface line (§2.3) and the T4-harness fixture addition (§15).

## 2. Public API surface

### 2.1 `dimos/pure/resources.py` exports

```python
__all__ = [
    "IN_HANDLER",
    "Resource",
    "ResourceDefinitionError",
    "ResourceError",
    "ResourceRule",
    "RunResources",
    "attach_resources",
    "resource",
]
```

### 2.2 The user surface (sketch parity — the whole story)

```python
class VoxelGridMapper(PureModule):
    voxel_size: float = 0.05

    @resource
    def grid(self) -> VoxelGrid:           # engine calls .dispose() at teardown
        return VoxelGrid(voxel_size=self.voxel_size, carve_columns=True)

    def step(self, state: State, i: In) -> tuple[State, Out | None]:
        self.grid.add_frame(i.lidar)       # instance access → VoxelGrid
        ...
```

- `self.grid` (instance access) → the resource VALUE, statically typed
  `VoxelGrid` (§3.4).
- `VoxelGridMapper.grid` (class access) → the `Resource[VoxelGrid]`
  descriptor itself (introspection; never runs the factory).
- `@resource` may decorate `async def` factories (§7.3); `@resource(
  dispose=False)` marks a non-owned return (§9). Those three spellings are
  the entire ceremony — the sketch's bare `@resource` stays one line
  (index API floor: Tagger, which has no resources, gains nothing).

### 2.3 `pm` export (integrator line)

`resource` — the decorator — is added to `dimos/pure/__init__.py` imports
and `__all__`. Nothing else: `Resource`, the error types, `ResourceRule`,
`RunResources`, `attach_resources`, and `IN_HANDLER` stay module-qualified
(`from dimos.pure.resources import ...`) — engine/introspection surface,
not authoring surface. (The sketch's own import list, sketch3 line 69–77,
names exactly `resource`.)

## 3. The descriptor

### 3.1 Class and construction

```python
class Resource(Generic[_T]):
    factory: Callable[..., Any]      # the undecorated function (self) -> _T | Coroutine[..., _T]
    disposes: bool                   # False under @resource(dispose=False)
    is_async: bool                   # inspect.iscoroutinefunction(factory)
    name: str                        # attribute name, from __set_name__
    owner: type | None               # defining class, from __set_name__
```

`__init__(factory, *, dispose=True)` validates at DECORATION time — which is
class-body execution, i.e. import time, the same fail-loud moment as T2/T3:

- *factory* must be a plain function (`inspect.isfunction`). A
  `classmethod`/`staticmethod`/`property`/non-callable raises
  `ResourceDefinitionError`: `@resource must wrap a plain method taking only
  self — got {type}. It needs self for config and sibling resources.`
  (echoes stepspec's `[step-not-function]` voice).
- Signature must be exactly one positional parameter (`self`), no
  varargs/kwargs/defaults: `@resource factory {qualname} must take exactly
  (self) — resources are built from config on self, not from arguments; got
  signature {sig}.`
- Applying `@resource` to a `Resource` (double decoration) raises:
  `{qualname} is already a resource — apply @resource once.`

Definition-time errors are plain-message `ResourceDefinitionError`s (T2's
`ConfigFieldError` style — a small closed set); the runtime catalog gets the
enum treatment instead (§11).

### 3.2 `__set_name__`

Captures `owner` and `name`. If the descriptor object is already bound under
a different name (aliasing one `Resource` instance to two attributes, or
sharing across classes by assignment), raise `ResourceDefinitionError`:
`resource {old_qualname} is also bound as {new_owner}.{new_name} — each
@resource declaration is one descriptor; declare a second factory instead.`
Rebinding under the SAME name in a subclass never calls `__set_name__` on
the inherited object (Python only fires it for names in the defining body),
so inheritance is unaffected; an OVERRIDE is a new `@resource` in the
subclass body — a new descriptor — and is legal (§4).

### 3.3 `__get__` — one attribute, three answers

Normative order inside `__get__(self, obj, owner)`:

```
0.  obj is None            → return self                     (class access)
1.  IN_HANDLER flag set    → raise ResourceError [resource-in-handler]   (§10)
2.  run context installed  → return the per-run instance     (§5.2, §7)
                             (missing name → [resource-order], §7.4)
3.  otherwise              → lazy per-instance cache          (§5.1)
```

Class access returns the descriptor and does nothing else — no factory
call, no flag check (introspection and stepspec-adjacent tooling must stay
side-effect free; stepspec itself never getattrs non-reserved names, but
user tooling will).

`Resource` is a NON-DATA descriptor (no `__set__`/`__delete__`), and that is
load-bearing twice over: (a) `PureModule.__setattr__` already raises
unconditionally, so assignment is blocked at the instance surface without
descriptor help; (b) the lazy cache must NEVER be stored in the instance
`__dict__` under the attribute name itself — a same-name entry would shadow
a non-data descriptor and freeze the answer, bypassing the run/lazy switch
and the handler flag forever after. Hence the cache spelling of §5.1.

### 3.4 Static typing (mypy --strict, T4 house pattern)

```python
class Resource(Generic[_T]):
    @overload
    def __get__(self, obj: None, owner: type) -> Resource[_T]: ...
    @overload
    def __get__(self, obj: Any, owner: type | None = None) -> _T: ...
    def __get__(self, obj: Any, owner: type | None = None) -> Any: ...
```

First-match resolution gives class access the descriptor and instance
access `_T` — the shape `typing.py`'s `_InAccessor` uses and
`case_config_kwargs.py` pins (`reveal_type(c.grid)  # R: Grid`). The
instance overload's `obj: Any` (not `object`) is deliberate and verified:
`None` is an `object`, so an `object` spelling overlaps overload 1 with an
incompatible return (`[overload-overlap]` under strict); `Any` is exempt
from the overlap check — the typeshed descriptor idiom.

The decorator's overload set — ASYNC FIRST, T4's ordering doctrine (a
coroutine-returning factory must not unify with the sync overload, else
`_T = Coroutine[...]`):

```python
class _ResourceMarker(Protocol):
    @overload
    def __call__(self, factory: Callable[[Any], Coroutine[Any, Any, _T]], /) -> Resource[_T]: ...
    @overload
    def __call__(self, factory: Callable[[Any], _T], /) -> Resource[_T]: ...

@overload
def resource(factory: Callable[[Any], Coroutine[Any, Any, _T]], /) -> Resource[_T]: ...
@overload
def resource(factory: Callable[[Any], _T], /) -> Resource[_T]: ...
@overload
def resource(*, dispose: bool = ...) -> _ResourceMarker: ...
```

The factory's `self` parameter is typed `Any` deliberately: binding a
TypeVar to the owner buys nothing (the factory is invoked by machinery, not
users) and would fight the `dataclass_transform` lineage. Verified shape
lands in the T4 harness as `case_resources.py` (§15).

## 4. Discovery and declaration order

`_resource_specs(cls) -> tuple[Resource[Any], ...]` — module-private,
computed on demand (once per `attach_resources` call; the lazy path never
needs the list — the descriptor reaching `__get__` IS its own spec):

- Walk `reversed(cls.__mro__)`, skipping `object`; for each class, scan its
  OWN `__dict__` in definition order for `Resource` instances. No `dir()`,
  no `getattr` — raw mapping access only, upholding stepspec §6's
  no-descriptor-execution rule (own-`__dict__` MRO walk, the house pattern).
- Name-keyed dict built base-first with plain dict-update semantics:
  inherited resources come first in their base's declaration order, own
  resources append, and an override (same name redeclared) replaces the
  VALUE but keeps the first-declared POSITION — byte-for-byte the ordering
  rule T2 §4.4 gets from pydantic `create_model`, so config fields and
  resources order by one law.
- Zero class mutation, zero caching on the class (stepspec doctrine: "no
  mutation of `cls`"). The walk is a few dict scans; once per run is free.

Declaration order is a USER-FACING contract, not an implementation detail:
creation runs in it, disposal runs in its reverse, and a factory may read
sibling resources declared ABOVE it (§7.4).

## 5. Two lifecycles, one attribute

The index watch-out, restated as the invariant: the LAZY instance is
per-module-instance and belongs to test mode; the RUN instance is per-run
and belongs to the engine. They never blur — different storage, different
creator, different disposer, and the run world shadows the lazy world for
exactly the duration of the run.

### 5.1 Outside a run — lazy cached property (test mode)

Storage: one dunder key on the instance,

```python
_LAZY_ATTR: Final = "__pure_lazy_resources__"   # dict[str, object], created on first touch
```

written with `object.__setattr__` — the documented T2 §6 internal seam
("used by `__init__` and by T7's resource cache"). One dict under one
dunder, never the attribute name itself (§3.3b).

Normative lazy path (double-checked locking):

```
fast path (no lock):  cache dict present and name present → return value
slow path (module-level threading.RLock):
    re-check; create the dict via object.__setattr__ if absent
    cache[name] = _CREATING          # in-progress sentinel
    value = invoke factory           # §5.1.1
    on failure: del cache[name]; re-raise raw
    cache[name] = value; return value
```

- **Thread-safe**: the lock is ONE module-level `RLock` shared by all lazy
  creation. Per-descriptor locks would deadlock on cross-resource factory
  references (thread 1 holds a→wants b, thread 2 holds b→wants a);
  reentrancy (a's factory touching b on the same thread) needs the R in
  RLock. Coarse is correct here: the lazy path is test mode, contention is
  irrelevant. The lock-free fast path relies on CPython dict-read atomicity
  and publish-under-lock; values are stored only after fully constructed.
- **Cycle detection**: re-entering a slot holding `_CREATING` (self-
  reference or a→b→a) raises `ResourceError` `[resource-cycle]` naming the
  re-entered field. Without the sentinel this is a RecursionError with a
  useless traceback.
- **Failure = no cache**: a raising factory leaves the slot absent; the
  exception propagates RAW (test mode wants the original, not a wrapper);
  the next touch retries. `None` is a legitimate cached value (the sketch's
  `premap`) — presence is tracked by key, never by truthiness.
- **No disposal**: T7 defines no lazy-instance disposal. The cache lives
  and dies with the module instance (GC); T8's live `stop()` may later
  choose to dispose it — machinery is reusable (§13). Equality, hash, and
  pickle are untouched: `__eq__`/`__hash__` read config only, and
  `__reduce__` rebuilds from config, so caches never travel.
- **Async factories, lazily**: invoked via `asyncio.run(factory(self))`
  when no loop is running — a plain sync test "just touches" `m.client`
  and it works. Touched INSIDE a running loop, raise `ResourceError`
  `[resource-async-lazy-in-loop]` (an attribute access cannot await; §11
  copy says to touch from sync code or drive a run).

### 5.2 Inside a run — engine-owned, per-run

Storage: one dunder key on the instance,

```python
_RUN_ATTR: Final = "__pure_run_resources__"     # RunResources | None
```

installed by `RunResources.create()`/`acreate()` (via `object.__setattr__`)
BEFORE the first factory runs, cleared (set `None`) at the top of
disposal. While installed, `__get__` answers ONLY from the run context —
the lazy cache is shadowed, untouched, and never disposed by run teardown.

Interaction rules (each a test, §14):

- **Fresh per run.** Warmup creates new instances unconditionally — a
  pre-existing lazy instance is NOT reused. Rationale: a run is a pure
  computation `(code, config, State₀, inputs) → outputs`; reusing a
  test-mode leftover would make run N's outputs depend on instance history
  (a warmed cache, a half-full VoxelGrid), silently breaking replay
  identity and the memo key. Fresh-per-run is also what makes repeated
  `over()` calls independent by construction (T6 "a fresh run per call").
- **After the run**, the lazy world is exactly as it was: same object,
  undisposed, still cached.
- **One run at a time per instance** (resource-bearing modules only).
  `create()` finding `_RUN_ATTR` already non-`None` raises `ResourceError`
  `[resource-concurrent-run]`. Two interleaved `over()` iterators over ONE
  instance would otherwise fight over the context slot and dispose each
  other's resources. The guard fires at WARMUP (first `next()`), not at the
  `over()` call — preparing two iterators and consuming them sequentially
  stays legal (T6 §8.1: an unstarted iterator owes nothing). The error
  copy teaches the fix: construct a second instance — identity is
  class + config and construction is free. Resource-FREE modules keep
  full concurrent-run freedom (attach no-ops, §6.2).

## 6. Run wiring — the T7 RESOURCE SEAM in `drivers.py`

T6 §8.4 delivered the disposal half: two `RunHooks` fields, two call sites.
Creation needs the symmetric half, and it CANNOT live at the `over()` call:
T6 §8.1's acquisition invariant ("nothing is acquired before the first
`next()`") and D1's zero-acquisition eager phase force creation to
generator start. So the seam is: bind closures eagerly, run them lazily.

Five edits, all in `drivers.py`, all marked `# T7 RESOURCE SEAM`:

**(1) `RunHooks` gains the creation pair** (placed before the teardown
pair; both default no-op, mirroring §8.4):

```python
    warmup: Callable[[], None] = _noop                    # T7 seam — sync-run creation (t7 §6)
    awarmup: Callable[[], Awaitable[None]] = _anoop       # T7 seam — async-run creation, on the loop
```

**(2) `_finalized` runs `warmup` first, INSIDE the try** — before the first
row is pulled, sharing the §8.3 exactly-once frame:

```python
def _finalized(inner, rows, hooks):
    try:
        hooks.warmup()          # T7 RESOURCE SEAM — create before the first row pull (t7 §7)
        yield from inner
    finally:
        ...unchanged (§8.2)...
```

Inside-the-try is the load-bearing choice: a partially-failed warmup
unwinds through the SAME `finally` → `hooks.teardown()` path as every
other exit — §7.5's shared-unwind requirement drops out of the generator
protocol instead of needing its own mechanism. §8.3 gains one row:

| # | exit path | trigger | order | consumer sees |
| --- | --- | --- | --- | --- |
| 10 | warmup fails | `hooks.warmup()` raises at generator start | `rows.close()` → `hooks.teardown()` (unwinds the created prefix, §7.5) | `ResourceError` (cause = factory's original) |

**(3) `_drive_async_rows` awaits `awarmup` first, inside its try** — on the
run's loop, before FILL admits the first task, symmetric with the existing
`await hooks.ateardown()` in its `finally`:

```python
    try:
        await hooks.awarmup()   # T7 RESOURCE SEAM — create on the run loop, before FILL (t7 §7.3)
        while True:
```

A failed `awarmup` propagates into the same `finally`: the window is empty,
`await hooks.ateardown()` unwinds the created prefix ON the loop, the error
surfaces through the facade with the loop closed properly behind it.

**(4) `run_over` binds the seam eagerly** — pure closure binding, zero
acquisition, D1 intact. After the Mealy `initial` block, before the `align`
import:

```python
    from dimos.pure.resources import attach_resources  # T7 RESOURCE SEAM — lazy, permanently

    attach_resources(module, hooks, async_run=spec.kind is StepKind.ASYNC_STATELESS)
```

Lazy-import direction mirrors `align` (§1); "permanently" for the same
reason — the class layer imports drivers via `over()`, so a module-scope
`resources` import in drivers would still be acyclic today, but the lazy
spelling keeps drivers' import graph flat and the seam grep-able.

**(5) Docstring touch (polish, optional)**: `_noop`/`_anoop` docstrings say
"Default `RunHooks.teardown`/`ateardown`" — widen to "Default `RunHooks`
seam callable: nothing attached (T7 fills the seam)".

### 6.1 `attach_resources` — the seam function

```python
def attach_resources(module: object, hooks: RunHooks, *, async_run: bool) -> None
```

Normative behavior:

1. `specs = _resource_specs(type(module))`; if empty → RETURN with `hooks`
   untouched. The Tagger floor: a resource-free module pays nothing — no
   context, no closures, no per-tick cost, and concurrent runs stay
   unrestricted.
2. Build `ctx = RunResources(module, specs, async_run=async_run)` — inert:
   no instance write, no creation (that is `create()`'s job, at warmup).
3. Bind by CHAINING, never clobbering — T6 §9 allows caller-injected
   `RunHooks`, and a future T8 may pre-bind its own lifecycle callables.
   LIFO composition: previously-bound warmups run first (earlier
   acquisitions acquire first), previously-bound teardowns run last
   (released last):

   - sync run:  `hooks.warmup = chain(prev_warmup, ctx.create)`;
     `hooks.teardown = chain(ctx.dispose, prev_teardown)`.
   - async run: `hooks.awarmup = achain(prev_awarmup, ctx.acreate)`;
     `hooks.ateardown = achain(ctx.adispose, prev_ateardown)`;
     `hooks.teardown = chain(ctx.dispose, prev_teardown)` — the BACKSTOP:
     per T6 §8.4 `teardown` fires for every kind; `ctx.dispose` no-ops
     when `adispose` already ran (§8.5) and sweeps sync-best-effort when
     the loop died before `ateardown` could run.

   `warmup` is left untouched for async runs (creation belongs on the
   loop), and `awarmup`/`ateardown` untouched for sync runs (never invoked
   for sync kinds — T6 §8.4 seam contract).

### 6.2 Who runs when — the composed timeline

Sync kinds (stateless/mealy/fold), first `next()`:
`hooks.warmup()` → `ctx.create()` installs `_RUN_ATTR`, creates in
declaration order → rows flow → exhaustion/break/error → `_finalized`
finally → `rows.close()` → `hooks.teardown()` → `ctx.dispose()` reverse
order, clears `_RUN_ATTR`.

Async kind, first `next()`: facade builds the private loop →
`_drive_async_rows` starts → `await hooks.awarmup()` → `ctx.acreate()` on
the loop → FILL/AWAIT_HEAD → CLOSING → cancel window → reap →
`await hooks.ateardown()` → `ctx.adispose()` on the still-live loop →
`shutdown_asyncgens` → `loop.close()` → `_finalized` finally →
`rows.close()` → `hooks.teardown()` → `ctx.dispose()` no-op (already
closed). Never started → nothing created → nothing disposed (§8.3 row 8).

## 7. Creation semantics

### 7.1 `RunResources.create()` (sync runs)

```
1. concurrent guard: getattr(module, _RUN_ATTR, None) is not None
       → ResourceError [resource-concurrent-run]
2. object.__setattr__(module, _RUN_ATTR, self)        # BEFORE any factory
3. for i, spec in enumerate(specs):                    # declaration order
       spec.is_async → ResourceError [resource-async-sync-run]
       value = spec.factory(module)                    # user code
       record (spec, value) in created list + by-name map
```

Step 2 before step 3 is deliberate: a factory that reads a SIBLING resource
(`self.premap` inside `grid`'s factory) resolves through the RUN path and
gets the per-run instance — never a silently-materialized lazy one mid-run
(that would blur the two lifecycles at their most invisible point).

### 7.2 Factory errors in a run

A raising factory aborts warmup: the `Exception` is wrapped —
`ResourceError` `[resource-warmup-error]`, `raise ... from exc` (T6 D4
wrapping policy: coordinates in the wrapper, original uncloaked as
`__cause__`) — and propagates into the driver's `finally`, which unwinds
the created prefix through the NORMAL teardown path (§7.5).
`BaseException` (Ctrl-C, SystemExit) propagates RAW; the same `finally`
still unwinds (T6 §8.3 row 6 discipline). One carve-out, T6's own
engine-violation discipline: a `ResourceError` escaping the factory (a
sibling reference hitting `[resource-order]` via `__get__`) propagates
UNWRAPPED — machinery errors are never double-wrapped as user failures.

### 7.3 `RunResources.acreate()` (async runs) — on the run loop

Same algorithm, hosted in a coroutine awaited by the seam's `awarmup` call
site — so EVERY factory of an async run executes on the run's loop thread,
before the first task is admitted: sync factories are called inline (plain
calls inside the coroutine), async factories are awaited. One pass, one
order — mixed sync/async declarations keep strict declaration order, and a
loop-affine resource (the Captioner's `VLMClient`) is born on the loop it
will live and `aclose` on (T6 §8.4's stated reason the `ateardown` seam
exists).

Async factory in a SYNC run: `ResourceError` `[resource-async-sync-run]`
at warmup (settled, D6): a sync run has no loop at any point in the
resource's lifetime — an object that needs a loop to be BORN will need one
to live and die, and `over()` on a sync step never makes one. The copy
teaches both exits: make `step` async, or make the factory sync.
(Sync-factory resources whose only disposer is `aclose` are the softer
case and stay legal — §8.4.)

### 7.4 Touch-before-created — `[resource-order]`

During a run, `__get__` for a resource not (yet) in the by-name map raises
`ResourceError` `[resource-order]`: reachable from a factory referencing a
sibling declared BELOW it (or itself), or from a foreign thread touching
mid-warmup. The copy states the law: factories may use only resources
declared above them — reorder the declarations. (In-run cycles surface as
this error; lazy-mode cycles get the dedicated `[resource-cycle]`, §5.1.)

### 7.5 Partial warmup failure — unwind is teardown

No unwind code exists in `create()`/`acreate()`. The failure propagates;
the driver's `finally` invokes the seam's disposal (`teardown` for sync,
`ateardown` for async — §6.2 timeline); `dispose()`/`adispose()` sweep
`reversed(created)` — which at that moment is exactly the created prefix.
ONE disposal path serves partial warmup, clean exhaustion, consumer break,
step error, and GC finalization (index requirement "unwinds already-created
resources in reverse" is a corollary, not a feature).

## 8. Disposal

### 8.1 Sniffing (binding order)

Per resource, on the INSTANCE at disposal time (the factory's return type
is unknowable earlier):

```
value is None                      → skip (a disabled resource — sketch premap)
spec.disposes is False             → skip (§9)
dispose := getattr(value, "dispose", None); callable → call it, done
close   := getattr(value, "close",   None); callable → call it, done
aclose  := getattr(value, "aclose",  None); callable → await it (async ctx)
                                              / asyncio.run it (sync ctx, §8.4)
none of the three                  → no-op, silently
```

First hit wins; later names are never called. The silent no-op tail is
deliberate: plain-data resources (`premap: PointCloud2 | None`) are legal
and common — warning on them would train users to ignore warnings. Sniffing
uses `getattr`, so a property named `dispose` executes during the sniff —
attribute-protocol semantics, documented, not fought.

### 8.2 Order and exactly-once

Reverse creation order, always (`reversed(created)` — reverse declaration
order for a full warmup, reverse created-prefix for a partial one).
Exactly-once by a `closed` flag on `RunResources`: the first
`dispose()`/`adispose()` call flips it (and clears `_RUN_ATTR`) before
sweeping; every later call returns immediately. This is what makes the
async double-invocation (`ateardown` then the `teardown` backstop) safe,
and a resource whose disposal RAISED counts as attempted — the backstop
never retries a failed disposer (retry risks double-release side effects
worse than a leak).

### 8.3 Error-in-disposal policy (settled, D7)

`contextlib.ExitStack` semantics, the language's own precedent: attempt
EVERY disposal (one failure must not strand the rest), log each failure
immediately (`logger.warning` + `exc_info`, naming `{cls}.{name}` — T6 D5's
loudness-by-accounting voice), and after the sweep, if anything failed,
raise ONE `ResourceError` `[resource-dispose-error]` chained
`from` the FIRST failure, naming every failed field. Fail-loud where a
human is watching (exhaustion, break — T6 §8.3 row 7 sanctions teardown
exceptions, chained via `__context__` over any primary error), harmless
where none is (the GC path swallows finalizer exceptions by design;
`BaseException` during a sweep aborts it — Ctrl-C wins).

### 8.4 `aclose` routing

- Async run: `adispose()` awaits `aclose()` on the run's still-live loop —
  T6 §8.4's contract verbatim ("disposal after `loop.close()` is a
  correctness bug"). Sync `dispose`/`close` methods of the same run are
  called inline on the loop thread, same sweep, same order.
- Sync run (or the backstop after a dead loop): an `aclose`-only resource
  is disposed via `asyncio.run(value.aclose())` — best-effort, correct for
  objects created OFF-loop (a sync run's resources always were), and any
  failure (including `asyncio.run` inside a running outer loop — legal for
  sync-kind `over()`, which never probes) lands in the §8.3 policy: logged,
  swept past, raised after.

### 8.5 The async backstop, precisely

`teardown` fires for EVERY kind (T6 §8.2), so for async runs `ctx.dispose`
runs after `ctx.adispose` by construction (§6.2 timeline) and no-ops on the
`closed` flag. The one path where it acts: the loop broke before
`ateardown` ran (e.g. `run_until_complete` raising mid-close) — then the
backstop sweeps sync-best-effort per §8.4 rather than leaking the run's
resources. Conservative by design: a HALF-run `adispose` (BaseException
mid-sweep) already flipped `closed`, so the backstop stays out — no
double-dispose risk is accepted to chase a rarer leak.

### 8.6 Interop with `dimos/core/resource.py` (structural, no import)

The legacy rim currency: `dimos.core.resource.Resource(DisposableBase)` —
abstract `start()`/`stop()`, `dispose()` implemented as `self.stop()`,
context-manager `__enter__`/`__exit__` = start/stop, and
`CompositeResource` disposing registered children on `stop()`. Transports
(`LCMTransport` et al.) are these.

- **Interop is structural and guaranteed**: the dispose-FIRST sniff order
  means a factory returning any core `Resource` gets `dispose()` → which IS
  `stop()` → children swept for composites. No `dimos.core` import appears
  anywhere in `dimos/pure` — the guarantee is pinned by a shape-fake test
  (§14 #10) mirroring core's exact surface, not by a type check.
- **`start()` is the factory's job (settled, D8)**: the engine sniffs
  DISPOSAL only, never lifecycle-start. A factory returning a core
  Resource starts it itself: `t = LCMTransport(...); t.start(); return t`.
  Rationale: (a) the index binding names dispose/close/aclose — start-
  sniffing would grow the engine contract beyond it; (b) an engine-side
  `start()` call would make warmup behavior depend invisibly on the return
  TYPE (plain data vs core Resource) — the module author can't see it at
  the declaration; (c) double-start hazard: core `start()` implementations
  spawn threads and subscriptions, and factories that already start (the
  natural spelling) would be started twice; (d) the factory body IS the
  warmup hook — arbitrary code running at exactly the right moment; one
  explicit line beats invisible magic. Doctrine line lands in the
  `resource` docstring and the docs page.
- Objects exposing ONLY `stop()` (Service-shaped, no `dispose`) are NOT
  sniffed — the binding's three names stand. The factory wraps or the
  declaration says `dispose=False` and ownership lives elsewhere.

## 9. Ownership — dispose only what the factory created

Doctrine: a factory CREATES. Its return is engine-owned for the run (or
cache-owned in test mode) and gets disposed. A factory that returns an
object owned elsewhere — a process global, a shared client, anything
injected by the surrounding program — declares it:

```python
@resource(dispose=False)
def bus(self) -> EventBus:
    return GLOBAL_BUS          # engine will never dispose this
```

`disposes=False` skips §8.1 entirely; creation timing and the two
lifecycles are unchanged. This is the MINIMAL honest expression: a
per-declaration, statically-visible marker at the only place the knowledge
exists (the author knows what the factory returns; the engine cannot).
Runtime ownership tracking (identity registries, refcounts) is rejected as
machinery-for-a-doctrine. The rpc sketch's `injected()` slot (§5,
wiring-supplied capabilities) is the eventual first-class channel for
injected objects; `dispose=False` is T7's forward-compatible spelling and
stays correct alongside it.

## 10. The RPC handler hook (hook ONLY — no RPC here)

sketch3_rpc §3: "a mutating handler cannot touch @resource (the engine
flags handler context; resource access raises)". T7 ships the flag and the
raise; the future RPC layer ships the flagging:

```python
IN_HANDLER: Final[ContextVar[bool]] = ContextVar("dimos_pure_in_handler", default=False)
```

`__get__` step 1 (§3.3): `if IN_HANDLER.get(): raise ResourceError
[resource-in-handler]` — before the run/lazy split, so BOTH paths are
guarded (the doctrine bans touching, not a particular lifecycle). The RPC
layer will `token = IN_HANDLER.set(True)` around a mutating handler and
`reset(token)` after — `ContextVar` (not a module global) because
read-only handlers run OFF the module loop on snapshot state and async
interleaving must not leak the flag across tasks. Cost when unset: one
`ContextVar.get()` per resource access (~tens of ns against a step's real
work) — the "cheap flag" the index asked for. Nothing else is built:
no decorator, no handler classification, no set-side helper.

## 11. Error catalog

### 11.1 Types

```python
class ResourceDefinitionError(PureModuleDefinitionError):
    """A @resource declaration is malformed (raised at class definition)."""

class ResourceError(PureModuleRunError):
    """Resource machinery violated a runtime contract.

    resource_rule: ResourceRule | None   # machine-readable, T7's catalog
    (base `rule` stays None — RunRule is drivers' namespace)
    """

class ResourceRule(enum.Enum):
    WARMUP_ERROR = "resource-warmup-error"
    ORDER = "resource-order"
    CYCLE = "resource-cycle"
    CONCURRENT_RUN = "resource-concurrent-run"
    ASYNC_SYNC_RUN = "resource-async-sync-run"
    ASYNC_LAZY_IN_LOOP = "resource-async-lazy-in-loop"
    DISPOSE_ERROR = "resource-dispose-error"
    IN_HANDLER = "resource-in-handler"
```

Definition-time inherits T3's single user-facing definition-error type
(t3-validation.md §8 req 6, as T2 did); runtime inherits T6's run-error
root so `except PureModuleRunError` around a run catches resource failures
too. Lazy-path machinery violations (cycle, in-loop, in-handler) reuse
`ResourceError` — "Run" in the name notwithstanding, one runtime type for
one catalog beats a third root. Lazy-path FACTORY exceptions are raw (§5.1).

### 11.2 Message templates (release copy; `{cls}` = module-qualified class,
`{name}` = attribute, house standard)

- `[resource-warmup-error]` — `{cls}.{name} factory raised during warmup
  (resource {i} of {n}): {exc!r} — already-created resources were disposed
  in reverse order. [resource-warmup-error]` (chained `from exc`)
- `[resource-order]` — `{cls}.{name} was touched during a run before it was
  created — factories may only use resources declared ABOVE them; move
  {name} earlier in the class body or drop the reference.
  [resource-order]`
- `[resource-cycle]` — `{cls}.{name} is being created and was touched again
  — resource factories form a cycle; break it by removing the circular
  reference. [resource-cycle]`
- `[resource-concurrent-run]` — `{cls} already has a run in flight —
  resources are per-run, so one instance drives one run at a time. Build a
  second instance for a second run: identity is class + config, and
  construction is free. [resource-concurrent-run]`
- `[resource-async-sync-run]` — `{cls}.{name} is an async factory but the
  run is sync — a sync run has no event loop for the resource to live on.
  Make step async, or make the factory sync. [resource-async-sync-run]`
- `[resource-async-lazy-in-loop]` — `{cls}.{name} is an async factory
  touched from inside a running event loop — attribute access cannot
  await. Touch it from sync code (it runs via asyncio.run), or use it in a
  run. [resource-async-lazy-in-loop]`
- `[resource-dispose-error]` — `{cls} teardown: {k} of {n} resources failed
  to dispose ({names}); first failure: {exc!r} — every remaining resource
  was still disposed. [resource-dispose-error]` (chained `from` first)
- `[resource-in-handler]` — `{cls}.{name} was touched inside an RPC handler
  — handlers are pure functions of (config, state, request); all I/O
  belongs behind step's resources. [resource-in-handler]`

Log line (per §8.3 failure, before the summary raise):
`"{cls}.{name} failed to dispose; continuing teardown"` + `exc_info`.

## 12. Edge cases — index watch-outs → spec/tests

| index watch-out | spec | tests (§14) |
| --- | --- | --- |
| per-RUN vs per-instance lazy — don't blur | §5 (shadowing, fresh-per-run, untouched-after) | 12, 13 |
| partial warmup unwinds in reverse | §7.5 (shared path), §6 row 10 | 8, 15 |
| thread-safe lazy init | §5.1 (RLock, double-checked, sentinel) | 2, 4 |
| async factories/aclose on the module loop | §7.3, §8.4, seam edit 3 | 14, 15 |
| ownership — never dispose injected objects | §9 (`dispose=False`) | 11 |
| cheap handler-context flag for RPC | §10 (`IN_HANDLER`) | 19 |
| `over()` teardown same path | §6.2 timeline (hooks are THE path) | 6, 17, 22 |
| repeated `over()` = fresh resources | §5.2 fresh-per-run | 12 |
| dispose/close/aclose sniff order | §8.1 | 9, 10 |
| declaration order create / reverse dispose | §4, §8.2 | 6, 20 |

Additional edges settled here: `None`-valued resources (§5.1, §8.1);
descriptor aliasing (§3.2); inheritance + override ordering (§4);
concurrent runs on one instance (§5.2); sniff-miss silence (§8.1); core
`Resource` interop + `start()` doctrine (§8.6); caches invisible to
eq/hash/pickle (§5.1).

## 13. Division of labor (what T7 does NOT do)

- **No live lifecycle**: `PureModule.warmup()/start()/stop()` stay T2's
  no-ops. T8 wires the live rim and MAY drive `RunResources` (or
  `attach_resources` with its own hooks) for `warmup()`-created,
  `stop()`-disposed live resources — the ctx API is deliberately
  standalone-callable for exactly that.
- **No RPC**: §10 ships one ContextVar and one raise; classification,
  serialization, deadlines are sketch3_rpc's future.
- **No health plumbing**: disposal failures are logged and raised; T9's
  `resource_error` health field will consume them via its own seam —
  nothing pre-built beyond the log line's stable shape.
- **No replacement of `dimos/core/resource.py`**: the legacy `Resource`
  (start/stop/dispose, CompositeResource) remains the rim/transport
  currency; T7 interops with it structurally (§8.6) and never imports it.
  Migration of core resources into factories is per-module work under T12.
- **No memoization/cassette machinery** for nondeterministic resources
  (sketch reload note) — recorded-edge tooling, out of scope.

## 14. Test plan (`dimos/pure/test_resources.py`)

Real bodies, module-level
`pytestmark = pytest.mark.skip(reason="T7 skeleton — enable with implementation")`.
All module classes defined INSIDE test functions (import-time decoration
would execute the skeleton). Recording fakes: an event log (list of
`("create"|"dispose"|"close"|"aclose"|"step", name)` tuples) shared
per-test; loop-identity capture via `asyncio.get_running_loop()`.

1. `test_lazy_cached_identity` — two touches → one factory call, same
   object; second instance → its own object (per-instance isolation).
2. `test_lazy_thread_safety` — 8 threads through a barrier touch one
   resource → exactly one factory call, all threads see one object.
3. `test_lazy_factory_error_no_cache_retry` — factory raises once (raw
   exception observed), next touch retries and caches.
4. `test_lazy_cycle_detection` — a↔b factories → `ResourceError`
   `[resource-cycle]`.
5. `test_lazy_none_value_cached` — factory returning `None` runs once.
6. `test_run_create_dispose_order` — three resources over `over()`: log
   shows `create a,b,c` before the first step, `dispose c,b,a` after the
   last row.
7. `test_run_created_before_first_tick` — first step observes all
   resources created (log prefix check).
8. `test_partial_warmup_unwind` — second of three factories raises:
   first disposed, third never created, `ResourceError`
   `[resource-warmup-error]` with original as `__cause__`; the SAME
   instance runs cleanly afterwards.
9. `test_sniff_precedence` — value with dispose+close+aclose → only
   dispose; close+aclose → only close; no methods → silent no-op; `None`
   → skipped.
10. `test_core_resource_shape_interop` — shape-fake mirroring
    `dimos/core/resource.py` (`start`/`stop`, `dispose()` calling
    `stop()`, `__enter__`/`__exit__`): factory starts and returns it;
    teardown → `stop` ran exactly once, via `dispose`; engine never called
    `start` (factory-owned, §8.6).
11. `test_ownership_dispose_false` — `@resource(dispose=False)` value with
    a `dispose` method → never called at teardown.
12. `test_fresh_per_run` — two sequential `over()` runs → two distinct
    instances, two disposals; plus lazy-then-run: run instance is NOT the
    lazy one.
13. `test_run_shadows_lazy` — lazy touch, then a run capturing what step
    sees (different object), then after the run: lazy touch returns the
    ORIGINAL object, undisposed.
14. `test_async_factory_on_run_loop` — async module, async factory:
    factory-loop == step-loop == aclose-loop; `aclose` awaited before the
    run ends.
15. `test_async_partial_warmup_unwind` — async run, second async factory
    raises → first disposed on the loop, error surfaces.
16. `test_async_sync_run_error` — async factory on a sync-step module →
    `[resource-async-sync-run]` at first `next()`; created prefix
    disposed.
17. `test_early_break_disposes_once` — consumer takes one row and closes →
    exactly one disposal (and only one) per resource.
18. `test_dispose_error_policy` — last-created disposer raises → earlier
    resource STILL disposed; `ResourceError` `[resource-dispose-error]`
    chained from the original.
19. `test_handler_flag` — `IN_HANDLER.set(True)` → touch raises
    `[resource-in-handler]` (lazy AND mid-run); reset → touch works.
20. `test_declaration_order_inheritance` — base declares `a`, subclass
    declares `b` and overrides `a`: creation order `a,b` (first-declared
    position kept), disposal `b,a`, override factory the one invoked.
21. `test_definition_errors` — zero-arg factory, extra-arg factory,
    `@resource` on a classmethod, double decoration, aliasing one
    descriptor under two names → `ResourceDefinitionError` at class
    definition, message substrings pinned.
22. `test_e2e_over_with_resource` — sketch-shaped Mealy mapper
    (`emit_every`-style) with a recording grid resource over list streams:
    correct Out rows, resource created fresh, disposed at exhaustion.
23. `test_class_access_returns_descriptor` — `M.grid` is a `Resource`;
    factory not run.
24. `test_concurrent_run_guard` — interleaved second run on one instance →
    `[resource-concurrent-run]`; first run completes; a THIRD run after
    both closed succeeds; resource-free module: interleaved runs stay
    legal.
25. `test_run_sibling_reference` — factory of `b` reads `self.a` (declared
    above) → gets the RUN instance of `a`; reversed declaration →
    `[resource-order]`.
26. `test_lazy_async_factory` — async factory touched in sync test →
    value created (via `asyncio.run`); touched inside a running loop →
    `[resource-async-lazy-in-loop]`.

## 15. Static-typing fixture (T4 harness; implementer adds
`test_typing_fixtures/case_resources.py`)

New fixture, real machinery (unlike `case_config_kwargs.py`'s frozen local
stub, which stays untouched and must keep passing). Content, ready to
land (markers per the harness DSL; adjust revealed spellings to mypy's
actual output on first run):

```python
"""T7: @resource static surface — instance→T, class→Resource[T], async
factories unify on the payload, dispose=False preserves types, resources
are never config fields.  Static-typing fixture — never imported at runtime."""

from __future__ import annotations

from dimos import pure as pm
from dimos.pure import PureModule, contract, resource, tick
from dimos.pure.resources import Resource


class Grid:
    def dispose(self) -> None: ...


class VLM:
    async def aclose(self) -> None: ...


class Mapper(PureModule):
    voxel: float = 0.05

    class In(pm.In):
        x: float = tick()

    class Out(pm.Out):
        y: float = contract(min_hz=1)

    @resource
    def grid(self) -> Grid:
        raise NotImplementedError

    @resource
    async def client(self) -> VLM:
        raise NotImplementedError

    @resource(dispose=False)
    def shared(self) -> Grid:
        raise NotImplementedError

    def step(self, i: In) -> Out:
        raise NotImplementedError


m = Mapper(voxel=0.1)
reveal_type(m.grid)  # R: Grid
reveal_type(m.client)  # R: VLM
reveal_type(m.shared)  # R: Grid
reveal_type(Mapper.grid)  # R: dimos.pure.resources.Resource[Grid]

Mapper(grid=Grid())  # E[call-arg]: Unexpected keyword argument
```

The `m.client` reveal is the async-ordering pin (a `Coroutine` unification
bug would reveal `Coroutine[Any, Any, VLM]`); the last line re-pins
not-a-field against the REAL descriptor (case_config_kwargs pins it against
the stub).

## 16. Acceptance checklist

- [x] `resources.py` implements §3–§10; `uv run mypy dimos/pure` clean
      (strict), including the drivers seam edits.
- [x] drivers.py seam: exactly the five §6 edits, each marked
      `# T7 RESOURCE SEAM`; `RunHooks` field order
      warmup/awarmup/teardown/ateardown; no other drivers changes.
- [x] All §14 tests enabled and green; suite total = previous + T7's
      (no regressions, no skips left).
- [x] `case_resources.py` added; T4 harness green including
      `case_config_kwargs.py` unchanged.
- [x] `pm.resource` exported (`__init__.py` import + `__all__`); nothing
      else from resources.py on the surface.
- [x] Error copy matches §11.2 templates; every `ResourceError` carries its
      `ResourceRule`; definition errors are `ResourceDefinitionError`.
- [x] Sketch parity: VoxelGridMapper §3, RelocalizationModule §5
      (None premap), Captioner §6 (aclose on loop) all expressible
      verbatim; Tagger unchanged (API floor).
- [x] No `dimos.core` imports under `dimos/pure` (grep-clean); interop
      test 10 green.
- [x] `_finalized`/`_drive_async_rows` §8.3 semantics preserved for
      resource-free modules (T6 test suite untouched and green).

## 17. Decisions (numbered for review)

- **D1 — `Resource[T]` non-data descriptor + `resource` decorator**; bare
  and `(dispose=False)` forms only; decoration-time validation; cache never
  under the attribute name (§3).
- **D2 — Two worlds, one attribute**: run context (`__pure_run_resources__`)
  shadows lazy cache (`__pure_lazy_resources__`); both written via
  `object.__setattr__` (T2 §6 seam); `__get__` order flag → run → lazy (§3.3,
  §5).
- **D3 — Fresh per run, never reuse the lazy instance**; lazy world
  untouched by runs; one active run per resource-bearing instance, guarded
  at warmup, error copy teaches build-a-second-instance (§5.2).
- **D4 — Declaration order = base-first MRO own-`__dict__` walk with
  dict-update override semantics** (T2 §4.4's order law); computed per
  attach, zero class mutation/caching (§4).
- **D5 — Creation rides the hooks**: `RunHooks` gains `warmup`/`awarmup`;
  `_finalized` calls `warmup()` inside its try; `_drive_async_rows` awaits
  `awarmup()` inside its try; `run_over` binds via `attach_resources`
  eagerly (closures only — T6 D1/§8.1 preserved). Chaining, never
  clobbering, LIFO (§6).
- **D6 — Async factories require an async run**; in async runs ALL
  factories execute on the run loop, declaration order, one pass; lazy
  async touch uses `asyncio.run`, erroring inside a running loop (§5.1,
  §7.3).
- **D7 — Disposal policy = ExitStack semantics**: reverse order, attempt
  all, log each failure, raise one `[resource-dispose-error]` from the
  first; exactly-once via the `closed` flag; failed disposal counts as
  attempted (§8.2–8.3).
- **D8 — `start()` is the factory's job**; engine sniffs disposal only
  (dispose → close → aclose, binding order); sniff-miss and `None` are
  silent no-ops; core `Resource` interops via dispose→stop, structurally,
  no import (§8.1, §8.6).
- **D9 — Ownership is declared, not inferred**: `@resource(dispose=False)`
  (§9).
- **D10 — RPC hook = `IN_HANDLER` ContextVar + one raise in `__get__`**,
  both lifecycles guarded; nothing else (§10).
- **D11 — Errors**: `ResourceDefinitionError(PureModuleDefinitionError)`
  plain-message; `ResourceError(PureModuleRunError)` + `ResourceRule` enum;
  lazy factory exceptions raw; run factory exceptions wrapped
  `[resource-warmup-error]` from the original (§11).
- **D12 — Layering**: resources imports drivers top-level (RunHooks,
  PureModuleRunError); drivers imports resources lazily in `run_over`;
  acyclic; no module.py/config.py/dimos.core imports (§1).
- **D13 — `pm` exports `resource` only** (§2.3).
- **D14 — Unwind IS teardown**: no dedicated partial-warmup code path;
  the created prefix unwinds through the normal seam disposal (§7.5).

## 18. Open questions (non-blocking; defaults chosen)

- **Q1**: Should the sniff also accept `stop()` (Service-shaped objects
  without `dispose`)? / default: NO — the index binding names three
  methods; core Resources are covered via `dispose()`; factories wrap
  anything else / options: (a) keep three (default), (b) append `stop` as
  a fourth sniff — one-line change later, no design impact.
- **Q2**: Should T8's live `stop()` dispose the LAZY cache (today: nobody
  disposes it)? / default: leave to T8 — the ctx machinery is reusable and
  the lazy cache is reachable; T7 documents "test mode has no disposal" /
  options: (a) T8 decides (default), (b) pre-commit T7 to a
  `dispose_lazy()` helper now.
- **Q3**: `[resource-dispose-error]` raising on the consumer-break path can
  surface an exception at an innocent-looking `break`. / default: keep —
  ExitStack/`with` behave identically and silence would hide leaks /
  options: (a) raise always (default), (b) log-only on GeneratorExit paths
  — revisit only with field evidence.

## Implementation notes (landed)

Implemented on `pure/impl-t7-resources`. All 26 §14 tests pass UNMODIFIED;
`case_resources.py` reveals matched mypy's output verbatim (no §15 spelling
adjustment needed). Suite: 292 passed (265 prior + 26 T7 + 1 harness case),
zero skips; `uv run mypy dimos/pure` clean (strict, 10 files). Two refinements,
both within the spec's stated invariants:

- **Disposal clears `_RUN_ATTR` only when the ctx still owns it** (§8.2 says
  "clears `_RUN_ATTR`"). `RunResources._relinquish()` clears the slot iff it
  currently references `self`. A concurrent-run guard (§5.2) fails BEFORE
  installing (§7.1 step 1 precedes step 2), so the second run's ctx is
  stillborn — its `dispose()` (fired by the driver's `finally`) must NOT
  clobber the in-flight run's slot. Unconditional clearing would break
  test 24's "first run unaffected". This is the sole correct reading of
  "clears `_RUN_ATTR`" and is invisible to the normal ownership path.
- **`_chain(ctx.dispose, prev_teardown)` is sequential** (§6.1 LIFO): if
  `ctx.dispose` raises `[resource-dispose-error]`, `prev_teardown` is skipped.
  In T7 `prev_teardown` is always `_noop` (`over()` mints fresh hooks), so this
  is unobservable; a future T8 injecting a real teardown alongside resources
  would want error-isolated composition there.

No spec contradictions surfaced; §18 defaults (Q1–Q3) implemented as written.
