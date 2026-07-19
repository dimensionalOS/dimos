# T8 — Live rim: ports, transports, lifecycle

> Comparison-run note: a parallel planning session occupies the canonical
> paths (`t8-rim.md`, `rim.py`, `legacy.py`). THIS spec's artifacts land
> suffixed: spec `t8-rim-b.md`, skeletons `rim_b.py` / `coordination_b.py`,
> tests `test_rim_b.py` / `test_coordination_b.py`. Intended FINAL paths
> (used throughout the prose): `dimos/pure/rim.py`,
> `dimos/pure/coordination.py`, `dimos/pure/test_rim.py`,
> `dimos/pure/test_coordination.py`. Rename on adoption.

Spec owner: T8 planning agent (branch `pure/spec-t8-rim`). Implements index
§T8 **including the Legacy parity amendment (2026-07-19, binding)**. Source
of truth: `dimos/memory2/puremodule_api_sketch3.py` (deployment section,
§5b, doctrine banner) as amended by `tasks/index.md`.

Two layers, two files, split for two implementation waves:

- **T8a — rim core** (`dimos/pure/rim.py` + engine seam edits §13): runtime
  port views behind `m.i`/`m.o`, transport/source binding, the live loop,
  `warmup()/start()/stop()`, live stats. Pure-module terms only; **never
  imports `dimos.core`**.
- **T8b — coordinator adapter** (`dimos/pure/coordination.py` + the memory2
  seam §13.3, parity tests, the e2e): `live(PureCls)` — a synthesized legacy
  `Module` subclass that makes a pure module deployable, autoconnectable and
  blueprint-instantiable with **zero** changes to `dimos/core`.

Everything here was designed against T1–T7 **as landed** (verified against
`typing.py`, `align.py`, `drivers.py`, `resources.py`, `stepspec.py`,
`module.py` at `cf0db1a04`) and against the legacy machinery as read
(`python_worker.py`, `module_coordinator.py`, `blueprints.py`,
`worker_manager_python.py`, `rpc_client.py`, `core/module.py`,
`core/stream.py`, `core/transport.py`, `protocol/pubsub`,
`protocol/service`).

---

## 1. Scope and layer boundary

### 1.1 What T8 delivers

1. Runtime `m.i.<field>` / `m.o.<field>` handles implementing T4 §5.5's
   accessor contract: lazy rim import in the accessor bodies, views keyed
   off the module's `StepSpec`, per-field handles name-validated against
   `Bundle.fields()` (closes T4 §5.4's `Port[Any]` hole at runtime).
2. `.transport =` / `.source =` (mutually exclusive) / `.subscribe()` on
   the handles; a live loop that takes transport deliveries on arbitrary
   threads, marshals them onto the module's ONE engine thread, feeds T5's
   aligner, drives the T6 driver, and publishes Out fields (transport +
   local subscribers).
3. `warmup()/start()/stop()` over T7's run machinery: fresh run resources
   per live session, reverse-order drain, idempotent stop.
4. Live stats: `rim.session(m).stats` (§10) — the answer to the
   VoxelMapper report's "drop counters unreachable" gap, scoped to LIVE
   runs (the `over()` surface stays T9's boundary, §10.3).
5. memory2 interop: `source =` accepts memory2 streams; `save()/drain()`
   recipes; single-input `transform()` equivalence (§9).
6. The coordinator adapter: `live(PureCls)` satisfying every parity-matrix
   row (§2), including the `g` absorption and the health-topic name
   convention (payload and cadence are T9's).

### 1.2 Layering (hard rules)

```
rows / config / stepspec / typing          (data layer)
        ▲
align / drivers / resources                (engine)
        ▲
rim.py                                     (T8a — imports engine + data ONLY)
        ▲
coordination.py ──────► dimos.core.*       (T8b — the bridge; also imports rim)
```

- `rim.py` imports `dimos.pure.{typing,rows,stepspec,align,drivers,
  resources}` and stdlib. It must not import `dimos.core`, `dimos.protocol`
  or `dimos.memory2`. Transports and sources are **structural** protocols
  (§4.4) — legacy transports satisfy them without being imported.
- `coordination.py` imports `dimos.core.module`, `dimos.core.stream`,
  `dimos.core.core`, plus `dimos.pure.{module,stepspec,rows}` and the rim.
  Direction justified: the adapter IS the bridge — it adapts the pure world
  to the legacy coordinator, so it must see both. Nothing in `dimos.core`
  imports `dimos.pure` (no cycle back into the engine); nothing in the
  engine imports `coordination.py`.
- The engine seam edits (§13) are the only touches outside T8's four files,
  each marked `# T8 RIM SEAM`.

### 1.3 Out of scope (explicit)

Per the parity amendment's honesty clause: parity covers **wiring /
lifecycle / transport / topic surface only**. NOT reproduced (matrix rows
P22–P25, P27): legacy module refs (`x: SomeSpec`), user `@rpc` methods
(RPC wave, `puremodule_api_sketch3_rpc.py`), mid-run attribute mutation,
legacy `self.tf` (T11), `restart_module(reload_source=True)` (hot-reload
wave). Health payload/cadence is T9 (T8 pins the topic name only, §12.4).

---

## 2. Phase-1 parity matrix (binding deliverable)

Every outside-visible behavior of a legacy module, found by reading the
machinery → how the pure adapter satisfies it → the test that pins it.
"inherited" means: the adapter class subclasses legacy
`dimos.core.module.Module` (decision D1, §17) and the behavior is the same
code path legacy modules execute.

Legend: pw = `core/coordination/python_worker.py`, mc =
`core/coordination/module_coordinator.py`, bp =
`core/coordination/blueprints.py`, wm =
`core/coordination/worker_manager_python.py`, rc = `core/rpc_client.py`,
mod = `core/module.py`, st = `core/stream.py`. Tests: `TC` =
`test_coordination.py`, `TR` = `test_rim.py`, `E2E` = §15.4.

| # | Legacy outside behavior | Where | Pure adapter answer | Test |
|---|---|---|---|---|
| P1 | Deploy = pickle `DeployModuleRequest(module_id, module_class, kwargs)` over a forkserver pipe; worker constructs `module_class(**kwargs)` | pw:216–250, 371–381 | `live(cls)` returns a class picklable by recipe (copyreg reducer → `live(pure_cls, name)`, D2); construction runs inherited `Module.__init__` | TC: pickle round-trip of class + construct from kwargs |
| P2 | **`kwargs["g"] = global_config` force-injected** at deploy; worker also reads it to sync `global_config.transport` | pw:226, 374–377 | Absorbed by inherited `ModuleConfig.g` field — never reaches the pure config (`extra="forbid"` stays intact) | TC: construct with `g=GlobalConfig()` |
| P3 | `instance_name` kwarg set by blueprints (and forced by `namespace()`) | bp:144, 326–331; mod:116 | Absorbed by inherited `ModuleConfig.instance_name` | TC |
| P4 | `namespace()` force-injects `kwargs["frame_id_prefix"]` | bp:333–338 | Absorbed by inherited `ModuleConfig.frame_id_prefix` | TC |
| P5 | `Actor.set_ref` executes `instance.ref = actor` on the worker side — the instance must accept attribute assignment | pw:128–131, 383–385 | The **adapter** instance is an ordinary mutable legacy module (`self.ref = None` in inherited `__init__`); the frozen PureModule rides inside it, untouched | TC: `a.ref = object()` succeeds; `a._pure` config stays frozen |
| P6 | `GetAttrRequest` proxies attribute reads; results must pickle (streams reduce to `RemoteIn/RemoteOut` via `owner.ref`) | pw:387–388; st:169–180, 235–238 | Inherited `__getstate__`/stream `__reduce__`; the rim session and port views live in engine-private slots on the *pure* instance and are dropped by `PureModule.__reduce__` (rebuild-from-config) | TC: pickle adapter instance pre-start |
| P7 | **Lifecycle + wiring calls do NOT ride the pipe**: `proxy.build/start/stop/set_transport` are `RpcCall`s on the LCM/zenoh RPC rail at topic `{name}/{method}`, served by the module's own RPC server started in `ModuleBase.__init__` | rc:63–81, 152–175; mod:156–166 | Inherited: `build/start/stop/set_transport` are the legacy `@rpc`-tagged methods (adapter overrides `build/start/stop` bodies, keeps `@rpc` tags); server + topic prefix come from `ModuleBase.__init__` | TC: `live(X).rpcs` ⊇ {build, start, stop, set_transport}; E2E |
| P8 | RPC topic prefix = class name, or instance name when deployed under one | mc:644–646; mod:165 | Inherited (`serve_module_rpc(self, name=config.instance_name)`) | E2E (implicit) |
| P9 | **Autoconnect port discovery**: `BlueprintAtom.create` runs `get_type_hints(module)` and collects class annotations with origin `In`/`Out` → `StreamRef(name, type, direction)` — it inspects *annotations*, it never calls the module | bp:97–154 | `live()` **synthesizes `__annotations__`**: one `In[T]` per `StepSpec.in_type.fields()` entry, one `Out[T]` per `out_type.fields()` entry (T = field annotation with `\| None` stripped), plus the `health` port (P21). This is "autoconnect answered from `fields()` + StepSpec" — answered at class-synthesis time | TC: `BlueprintAtom.create(live(X), {})` yields exactly the expected StreamRefs — a full parity row without processes |
| P10 | Wiring: streams matched across modules by **(remapped name, msg type)**; each match key gets ONE shared transport; both directions receive it via `instance.set_transport(name, transport)` | mc:298–330 | Names/types come from bundle fields (P9), so matching is free; `set_transport` inherited (`stream._transport = t`, mod:782–792); the adapter hands bound transports to the rim at `start()` (§12.3) | TC: `set_transport` unit; E2E data flow |
| P11 | Topic naming: default topic `/{stream_name}` when the name is unique in the blueprint, else `/{short_id()}`; `pLCMTransport` when the type lacks `lcm_encode`, typed `LCMTransport`/Zenoh otherwise; zenoh prefixes `dimos/` | mc:663–665; `transport_factory.py` | Free — topic naming is coordinator-side and keys off the stream name = the bundle field name. Convention inherited exactly | E2E: assert transport topic `/x` |
| P12 | `remappings()` / `namespace()` rename streams (and hence topics) before matching; `expose=` crosses the namespace | bp:262–368 | Free — name-based, adapter-agnostic (P9 names participate identically) | TC: namespaced blueprint atom keeps remapped names |
| P13 | Blueprint declaration: `Cls.blueprint(**kwargs)` → `Blueprint.create` | mod:429–434 | Inherited classproperty | TC |
| P14 | Blueprint config surface: `Blueprint.config()` reads `get_type_hints(module)["config"]` for the per-module CLI/env override model | bp:221–249 | `live()` synthesizes `config: <Name>LiveConfig` — a `ModuleConfig` subclass extended with the pure config fields (§12.2) | TC: `Blueprint.create(live(X)).config()` includes the pure fields |
| P15 | Deploy ordering contract: construct → `set_transport`× → `build()` (24 h timeout, heavy one-time work) → `start()` | mc:332–363; mod:182–188 | `build()` → `rim.warmup(pure)` (sync-module resources created here — the heavy phase lands in the long-timeout slot); `start()` → `rim.start(pure)` (ensures warmup) | TC lifecycle-order; E2E |
| P16 | `stop()` called MORE than once and from two rails: coordinator (RPC `call_nowait`, reverse deploy order) and worker shutdown/undeploy (`instance.stop()` in `finally`) — must be idempotent and concurrent-safe | mc:98–117; pw:352–368, 394–398; mod:196–232 | Adapter `stop()` = `rim.stop(pure)` (idempotent latch, §7.3) then `super().stop()` (legacy `_module_closed` latch) | TC: double stop; E2E |
| P17 | Undeploy: `proxy.stop()` + `UndeployModuleRequest` → worker `instance.stop()`, worker torn down when empty | mc:447–501; wm:128–148 | Same path as P16 | E2E teardown |
| P18 | `dedicated_worker: ClassVar[bool]` → module gets a worker process to itself | mod:133–137; wm:239–255 | `live()` sets `dedicated_worker = True` — **the amendment's one-module-per-process lands here, with zero coordinator changes** | TC: attr; E2E: distinct pids |
| P19 | Optional `on_system_modules` hook (hasattr-guarded) | mc:292–296 | Not defined → skipped. Optional in the legacy contract; parity holds | TC: `hasattr` is False |
| P20 | Introspection: `peek_stream`, `io()`, `module_info`, `list_modules` row | mod:308–427, 794–807; mc:141–154 | Inherited; works because the rim fans Out rows into the legacy `Out` streams via `Out.publish` (§12.3), which feeds `peek_stream`'s local subscribers | TC: local peek after publish |
| P21 | **Health topic (pure-only extra, amendment)** | — | Every `live()` class declares `health: Out[Health]` (T9's payload type). Stream name `health` + one shared type ⇒ standard (name, type) matching gives ALL pure modules **one shared `/health` topic** — exactly the sketch's `g.health` merged, path-keyed stream. T8 pins name + topology; T9 fills payload + cadence via a marked seam in `live()` | T9 (name convention pinned as a seam-comment assertion in TC) |
| P22 | Module refs: `x: SomeSpec` / `x: OtherModule` annotations resolved by `_connect_module_refs` → `set_module_ref` | bp:119–142; mc:981–1045 | **Out of scope** (RPC wave): pure modules declare no refs (their annotations are config fields, never Specs); a pure module cannot yet *provide* a spec (no user rpcs). Consumers must not spec-match it — structurally true since `live()` classes expose no user `@rpc` methods | out-of-scope row |
| P23 | User `@rpc` methods callable via module proxies / `rpcs` listing | mod:308–317 | **Out of scope** (RPC wave, sketch3_rpc). `live(X).rpcs` lists only the inherited lifecycle rpcs | out-of-scope row |
| P24 | Mid-run attribute mutation on the instance (legacy internal habit) | — | **Not reproduced** by doctrine (amendment). The adapter shell is mutable (P5) but the pure module inside is frozen | out-of-scope row |
| P25 | `self.tf` ambient TF service on every module | mod:276–290 | **Out of scope** → T11 (tf() sampler / tf_out ports). Inherited lazy property exists but nothing activates it | out-of-scope row |
| P26 | `main()` async-gen and `handle_<input>` auto-binding conventions | mod:550–641 | Not applicable — pure classes define neither; inherited machinery is inert (`_start_main` no-ops without `main`) | TC: start without main |
| P27 | `restart_module(reload_source=True)`: reload the class's source module, re-deploy by `getattr(source_mod, cls.__name__)` | mc:548–632 | **Documented gap**: the synthesized class is not an attribute of the pure class's source module, so reload-resolution cannot find it. `reload_source=False` restarts work (class re-pickles by recipe). Full hot reload belongs to the reload wave (sketch's reload note) | out-of-scope row (deferred) |
| P28 | Worker syncs `global_config.transport` from the deployed `g` so child transports pick the host backend | pw:374–377 | Free — happens before construction, adapter-agnostic | E2E under default backend |

**Surprising findings (for the record):** (P7) coordinator→module calls
ride the pubsub RPC rail, not the Actor pipe — the pipe is
deploy/undeploy/attr-get/set_ref only; (P9) autoconnect never calls the
module — port enumeration is class-annotation shape, so the adapter
answers it at synthesis time; (P5) `set_ref` *mutates* the instance, which
a frozen PureModule would refuse — a second reason (after `g`) the adapter
must be a shell around the pure instance rather than the pure instance
itself; (P16) `stop()` arrives twice on different threads in every normal
shutdown.

---

## 3. Rim core — public API (`rim.py`)

```python
__all__ = [
    "DEFAULT_CAPACITY",
    "LiveSession",
    "RimError",
    "RimInPort", "RimInPorts", "RimOutPort", "RimOutPorts",
    "RimPortStats", "RimPublishStats", "RimRule", "RimStats",
    "SessionState", "SourceLike", "TransportLike",
    "build_in_ports", "build_out_ports",
    "session", "start", "stop", "warmup",
]
```

Module-level lifecycle functions take the module instance (they are what
the §13 seams delegate to):

```python
def warmup(module: Any) -> None       # idempotent; §7.1
def start(module: Any) -> None        # §7.2; [rim-already-running] on a running session
def stop(module: Any) -> None         # idempotent; §7.3
def session(module: Any) -> LiveSession | None   # the live surface; None when never started
def build_in_ports(module: Any) -> RimInPorts    # cached per instance; §4
def build_out_ports(module: Any) -> RimOutPorts
```

`Any` by design, mirroring `run_over` (t6 §1): the rim drives anything
step-shaped; it recovers the shape via `step_spec(type(module))` and never
imports the class layer at module scope.

Engine-private instance slots (all installed with `object.__setattr__`,
following T7's `_RUN_ATTR` pattern; all dropped by `PureModule.__reduce__`
because it rebuilds from config):

| slot | holds |
|---|---|
| `__pure_ports_in__` | the cached `RimInPorts` view |
| `__pure_ports_out__` | the cached `RimOutPorts` view |
| `__pure_rim__` | the current/last `LiveSession`, or absent |

---

## 4. Port views and handles (runtime)

### 4.1 Views

`RimInPorts(InPorts[Any])` / `RimOutPorts(OutPorts[Any])` — implement
against T4's static classes (subclassing them so the declared members line
up; T4 §5.5 allows subclass-or-duck, we subclass). Built once per module
instance, keyed off `step_spec(type(module))`:

- `RimInPorts` holds one `RimInPort` per `spec.in_type.fields()` entry, in
  declaration order; `RimOutPorts` one `RimOutPort` per
  `spec.out_type.fields()` entry.
- `__getattr__(name)`: unknown names raise **`AttributeError`** (not
  RimError — attribute semantics stay attribute semantics, T4 §5.4):
  `"{Bundle} has no port {name!r} — ports: a, b, c. [rim-unknown-port]"`.
  This is the runtime closure of the `Port[Any]` typing hole.
- Views are cached in the instance slots; repeated `m.i` returns the same
  view, `m.i.x` the same handle (binding state must persist — tests pin
  identity).

### 4.2 `RimInPort`

State: `transport: Any = None`, `source: Any = None`,
`capacity: int | None = DEFAULT_CAPACITY`, plus internals (ring,
normalized unsubscribe) once live.

- `transport =` / `source =` are **mutually exclusive**: assigning one
  while the other is set raises `[rim-both-bindings]`. Assigning either
  while the session is RUNNING raises `[rim-rebind-running]`. Re-assigning
  the same kind while stopped is allowed (last wins); assigning `None`
  clears.
- Binding is duck-validated at assignment (D8): a transport must expose
  callable `subscribe` and `publish`; a source must expose callable
  `subscribe`. Violations raise `[rim-bad-transport]` / `[rim-bad-source]`
  naming what's missing. Static types stay `Any` per T4 (settability is
  the point; no typing.py churn).
- `capacity`: `None` (unbounded) or `int >= 1`; else `[rim-bad-capacity]`.
  Default `DEFAULT_CAPACITY = 1` — **KeepLast** controller semantics
  (prior art: `docs/usage/pure_modules.md` backpressure table; a slow step
  always sees the freshest item, skipped ones are counted, not warned
  about). Recorder-style modules set `m.i.lidar.capacity = None`.

### 4.3 `RimOutPort`

State: `transport: Any = None` (same duck validation / rebind rules) and a
subscriber list.

- `subscribe(fn) -> Callable[[], None]` — local subscriber; returns an
  unsubscribe callable (mirrors legacy `Out.subscribe`). Allowed at any
  time, including while running (list mutations under a small lock;
  publish iterates a snapshot tuple).
- `frames` raises `NotImplementedError` until T11 (T4 §5.5).
- Unbound out port (no transport, no subscribers): publishing silently
  discards — matches legacy `Out.publish` with no transport, and effects
  may legitimately be unobserved.

### 4.4 Structural protocols

```python
class TransportLike(Protocol):
    def publish(self, msg: Any, /) -> None: ...
    def subscribe(self, callback: Callable[[Any], None], /) -> Callable[[], None]: ...

class SourceLike(Protocol):
    def subscribe(self, callback: Callable[[Any], None], /) -> Any: ...
```

Legacy `Transport` satisfies `TransportLike` exactly: `publish(msg)` is
the base convenience (`broadcast(None, msg)`, st:99–101) and
`subscribe(callback, selfstream=None)` is single-arg-callable with the
selfstream defaulted. `SourceLike.subscribe` may return either an
unsubscribe callable **or** a Disposable (`dispose()` sniffed — memory2's
`Stream.subscribe` returns a `DisposableBase`); the rim normalizes at
subscription time. Legacy `In` streams and `RimOutPort`s are `SourceLike`,
so in-process module composition is `m2.i.x.source = m1.o.y`.

---

## 5. The live loop

### 5.1 Data path (one module, one engine thread)

```
transport / source threads                 engine thread ("<Cls>-rim")
──────────────────────────                 ─────────────────────────────
callback(msg) ──► ring.append ──wake──►    ring iterators (blocking)
   (bounded; overflow drops oldest,             │ align(spec.in_type, …)   T5
    counted per port)                           │ drive_*(module, rows, …) T6
                                                │ per-field Out publish    §5.4
                                                ▼ local subscribers
```

- **Rings**: one per bound In port. `capacity=1` (default) = a single
  overwrite slot; `capacity=n` = deque-maxlen semantics dropping oldest;
  `None` = unbounded. Overflow increments the port's rim drop counter.
  One mutex + condvar pair per session (not per ring) protects
  append/pull/close and wakes the engine; **this is the only lock in the
  data path** — transports never block beyond the append, and drops
  happen here or nowhere (transports never drop by policy — index rule).
- **Ring iterators**: each bound port's stream handed to the aligner is a
  blocking iterator: `next()` pops the ring head, waiting on the condvar
  when empty; a CLOSE sentinel (set by `stop()`) makes `next()` drain
  remaining items and then raise `StopIteration`. Blocking inside the
  aligner's refill **is** T5's hold semantics live: a tick is held exactly
  while some other port's frontier is unknown (T5 §15
  held-while-frontier-pending); memory stays bounded because the rings
  drop, not the merge.
- **Run composition** (D4): the rim composes the run from PUBLIC engine
  API only — no `run_over`, no private helpers, no engine edits:

  ```python
  spec = step_spec(type(module))
  hooks = RunHooks()
  attach_resources(module, hooks, async_run=spec.is_async)   # T7 seam
  aligner = align(spec.in_type, live_streams)   # validates eagerly, pulls nothing
  rows: Iterator[Any] = aligner
  if spec.kind is StepKind.MEALY:
      out = drive_mealy(module, rows, spec.state_type(), skips=spec.skips, hooks=hooks)
  elif spec.kind is StepKind.ASYNC_STATELESS:
      out = drive_async(module, rows, max_inflight=mi, skips=spec.skips, hooks=hooks)
  elif spec.kind is StepKind.FOLD:
      out = drive_fold(module, rows, hooks=hooks)
  else:
      out = drive_stateless(module, rows, skips=spec.skips, hooks=hooks)
  ```

  `mi` follows T6 D6: `getattr(module, "max_inflight",
  DEFAULT_MAX_INFLIGHT)`; validation is already inside `drive_async`'s
  facade (re-checked at generator start). Wiring errors surface from
  `align()`'s own eager validation with T5's release copy
  (`[align-missing-tick-stream]` etc.) — the rim adds no duplicate check.
- The engine thread's body: `for row in out: fan_out(row)`. Any engine
  exception (StepError, PureModuleRunError, ResourceError) terminates the
  loop; the driver's own `_finalized` teardown has already run by the time
  the exception escapes; the session records it (`session.error`), logs it
  once at ERROR with the module class, and transitions to STOPPED-with-
  error. It is re-raised from `stop()` only when stop is what the caller
  is executing; a spontaneous mid-run death surfaces via `session.error` +
  log (a live module has no caller to raise into — T9's health stream is
  the operational surface).
- **Async steps**: `drive_async` is reused UNCHANGED — its sync facade
  owns a private asyncio loop that lives entirely inside the engine thread
  (`_reject_running_loop` passes: the engine thread runs no outer loop).
  In-flight window, tick-order emission, drain-on-stop, `aclose` — all T6
  semantics apply verbatim to live runs.

### 5.2 Timestamps and ordering

- The ONE ts authority stays the payload (T5 §3): rings carry msgs
  untouched, no arrival stamping, **no wall clock anywhere in the rim data
  path** (wall time lives in transports below and T9's pacer above; the
  single wall-time use in the rim is the stop-join timeout, a lifecycle
  boundary, §7.3).
- Per-port monotonicity is enforced by the aligner at ingestion
  (`dropped_nonmonotonic`) — out-of-order transport delivery is coped
  with, counted, never fatal.
- Live cross-port arrival order is inherently racy; determinism is NOT
  live's promise. The deterministic story is: record the edges
  (`m.o.x.subscribe` → store append, §9.2 — the index's "keep every edge
  recordable" preclusion guard), replay with `over()`.

### 5.3 Backpressure doctrine

ALL drop policy lives in the rings (engine buffers); transports never drop
by policy and the aligner/driver never discard except by their own spec'd
semantics. Accounting: per-port `rim_dropped` counters (§10) + T5's
`AlignStats` + T6's `RunHooks` — three layers, each visible in
`session.stats`.

### 5.4 Out fan-out

Per emitted (already tick-stamped) Out row, in Out-bundle declaration
order, for each field:

1. `value = getattr(row, field)`; `None` → skip (sparse silence — "a port
   stays quiet that tick").
2. If the port has a transport: `transport.publish(value)`.
3. Then local subscribers, in subscription order, with the raw value.

Wire currency is the **raw stamped field msg**, never the row envelope —
legacy consumers subscribe `In[PointCloud2]` and must receive
`PointCloud2`. (Consequence, documented: a field type without a `ts`
attribute is legacy-interop-only; a *pure* consumer's aligner will reject
it with `[align-bad-item]`. Pure-to-pure edges use stamped msg types —
"ports speak stamped msgs" is doctrine, enforced by the consumer, at this
layer.)

Subscriber/transport exceptions during fan-out: caught per callback,
logged at WARNING with module + port + exception, counted
(`publish_errors`), run continues (D7 — loudness by accounting, mirroring
T6's async-drop stance; one bad dashboard callback must not kill a robot
module).

---

## 6. `over()` isolation

`over()` remains structurally unable to touch transports: `run_over`
consumes only its `**streams` kwargs and never consults port views; the
rim's live path is entered only via `start()`. Nothing to disable — the
property is architectural. Additionally, a live session holds T7's
`_RUN_ATTR` for resource-bearing modules, so `m.over(...)` during a live
run fails fast with `[resource-concurrent-run]` (one instance, one run —
T7 §5.2). Resource-free modules may run `over()` concurrently with a live
session (harmless by construction: disjoint aligners, drivers, buffers).
Both behaviors are pinned by tests.

---

## 7. Lifecycle

`SessionState`: `IDLE → WARMED → RUNNING → STOPPED` (`STOPPED` may carry
`error`). One `LiveSession` per `start()`; a stopped session is never
restarted — `start()` after stop builds a FRESH session (fresh `RunHooks`,
fresh run resources — T7 per-run doctrine), reusing the port bindings.

### 7.1 `warmup()`

Idempotent (second call while WARMED/RUNNING no-ops).

- Sync-shaped modules (stateless/mealy/fold): create resources NOW, on the
  caller's thread — `attach_resources(module, hooks, async_run=False)`
  then `hooks.warmup()`; then neutralize the seam
  (`hooks.warmup = lambda: None`) so the driver's `_finalized` warmup call
  at first `next()` doesn't re-create (D5). On factory failure: run
  `hooks.teardown()` (unwinds the created prefix in reverse — the
  shared-unwind path T7 §7.5 expects), clear state, re-raise the
  `ResourceError`.
- Async modules: **validation only** — creation must happen on the run
  loop (`awarmup` → `acreate`, T7 §7.3), which exists only once
  `drive_async` starts. `warmup()` records the intent; resources are
  created at `start()` on the driver's loop. Documented consequence: for
  async pure modules the heavy phase lands in `start()`, not `build()`
  (P15 note).

Under the coordinator, `build()` maps here — heavy sync warmups get the
24 h build timeout (P15).

### 7.2 `start()`

On the caller's thread, in order:

1. `[rim-already-running]` if a session is RUNNING.
2. Ensure `warmup()` ran (call it if IDLE).
3. Build rings for every **bound** In port (transport or source set);
   unbound ports contribute no stream — then construct
   `align(spec.in_type, live_streams)` HERE: eager validation, zero
   pulls, so a missing tick/required binding fails fast on this thread
   with T5's release copy.
4. Compose the driver (§5.1).
5. Subscribe every bound port: `binding.subscribe(ring_append)`; store
   the normalized unsubscribe. From this instant deliveries queue
   (bounded by capacity) even though the engine hasn't started — nothing
   is lost at startup beyond ring policy.
6. Spawn the engine thread (name `"{ClassName}-rim"`, daemon), mark
   RUNNING, install the session slot.

Failure at any step unwinds in reverse (unsubscribe what was subscribed,
dispose what §7.1 created via `hooks.teardown()`), state → STOPPED, error
re-raised to the caller.

### 7.3 `stop()` — reverse-order drain

Idempotent (latch; concurrent calls: first wins, others join the same
latch — P16's two-rail reality). Order:

1. Unsubscribe all inputs (reverse bind order; no new deliveries).
2. Close every ring (CLOSE sentinel) — queued items remain consumable:
   **drain before dispose**. The engine thread's iterators exhaust; the
   aligner terminates on tick exhaustion (secondaries may retain pending
   heads — lazy, per T5); held ticks whose required secondaries never
   arrived drop with accounting; the driver finishes emitting for every
   row already resolved, then its own `_finalized` runs: `rows.close()` →
   `hooks.teardown()` → resources disposed in reverse creation order
   (async: window cancelled, reaped, `ateardown` on the loop, then the
   sync backstop — all T6 §6.3 / T7 §6.2, unchanged).
3. Join the engine thread (`STOP_JOIN_TIMEOUT_S = 5.0`; on expiry log
   ERROR naming the module and abandon the daemon thread — a stuck user
   step must not hang process shutdown).
4. If the thread never started (stop from WARMED): call
   `hooks.teardown()` directly — `RunResources.dispose` is latched, so
   this is safe in every interleaving.
5. State → STOPPED; a driver error captured in §5.1 is re-raised here if
   this stop discovered it, else left on `session.error`.

**T7 §18 Q2 decided (D11): live `stop()` does NOT dispose the lazy
cache.** The lazy cache is the test/notebook world; a run's resources are
per-run and fully disposed by the teardown chain above. Disposing
lazily-created objects on stop would surprise the notebook workflow
(module outlives many sessions) and blur T7's two-world split. T7's "test
mode has no disposal" stands.

**T7 `_chain` hardening declined (D12):** T7's implementation note warned
that `_chain(ctx.dispose, prev_teardown)` skips `prev_teardown` if dispose
raises — relevant only if T8 injected its own teardown callables into
`RunHooks`. The rim deliberately injects NOTHING into the hook seams
(§7.3 sequences unsubscribe/close/join outside the hooks; disposal runs
inside the driver's own finalizer). No seam edit needed; recorded here so
T7's flag has its answer.

### 7.4 Teardown ordering table (house standard)

| # | step | thread | mechanism |
|---|---|---|---|
| 1 | inputs unsubscribed | stop() caller | stored unsubscribe fns, reverse bind order |
| 2 | rings closed (drain mode) | stop() caller | CLOSE sentinel + condvar broadcast |
| 3 | remaining rows resolved + emitted | engine | aligner exhaustion semantics |
| 4 | driver teardown: `rows.close()` | engine | T6 §8.2 `_finalized` |
| 5 | resources disposed, reverse creation | engine (async: run loop) | T7 chain via `hooks.teardown`/`ateardown` |
| 6 | engine thread joined (5 s) | stop() caller | `Thread.join` |
| 7 | session STOPPED, error surfaced | stop() caller | latch |

---

## 8. Threading model (normative)

| work | thread |
|---|---|
| transport/source delivery → ring append | the transport's own thread (LCM handle thread, zenoh runtime, memory2 pool) — never blocked beyond the append |
| align → step → stamp → fan-out → local subscribers | THE engine thread, one per module |
| async step tasks + async resource create/dispose | private asyncio loop owned by `drive_async`, inside the engine thread |
| `warmup()` sync resource creation | caller thread (coordinator: the RPC dispatch thread serving `build`) |
| `start()`/`stop()` | caller thread; stop joins the engine |
| stats reads | any thread — lock-free snapshots of monotonic counters (T5 §8 / T6 §9 conventions) |
| (adapter only) legacy `_loop` + RPC serving | inherited legacy threads; they call rim lifecycle, never rim data |

Marshal, don't lock: the single session mutex guards only ring
append/pull/close; user code (`step`, factories, subscribers) never runs
under it.

---

## 9. memory2 interop

### 9.1 In: sources

`m.i.pose.source = db.streams.pose` feeds a port from a recording through
the REAL live machinery (threads, rings, drops) — memory2
`Stream.subscribe(on_next)` schedules iteration on the dimos pool and
delivers `Observation`s. The rim unwraps at the ring boundary exactly like
T6's `_coerce_stream`: an item recognized as a memory2 `Observation`
(via `sys.modules` sniff, zero memory2 import) contributes `obs.data`;
payload ts stays the one authority. `db.streams.pose.live()` gives the
endless variant. This is the old design doc's `input_sources` mode
(docs/usage/pure_modules.md), respelled per-port on T4's surface.

### 9.2 Out: `save()` / `drain()`

No new API — the recipe is the interop (ports speak stamped msgs, stores
append them):

```python
m.o.global_map.subscribe(lambda msg: store.streams.global_map.append(msg, ts=msg.ts))
```

Runs on the engine thread; `Backend.append` is the sink. For pipelines,
`store.streams.x.save(other).drain()` remains pure memory2 — the rim's
job ends at the subscriber.

### 9.3 `transform()` equivalence (T8b seam, memory2-side)

`store.streams.lidar.transform(m)` for a single-required-input module `m`
must be equivalent to `m.over(<tick_field>=store.streams.lidar)` yielding
Out ROWS as the stream payloads (sketch `_as_transform`:
`.map(lambda row: row.global_map)` follows). Marked seam edit in
`dimos/memory2/stream.py::Stream.transform` (`# T8 RIM SEAM`): when the
argument's type carries `__pure_step__` (duck-sniff, zero import of
`dimos.pure` — the attribute IS the marker), route to `over()` binding the
stream to the module's single required In field; more than one required
field → `TypeError` naming the fields and pointing at `over()`.
Observation ts = row ts.

---

## 10. Live stats — `session(m).stats`

```python
@dataclasses.dataclass
class RimPortStats:      # per In port
    delivered: int = 0       # callback invocations accepted into the ring
    rim_dropped: int = 0     # ring-overflow drops (backpressure)

@dataclasses.dataclass
class RimPublishStats:   # per Out port
    published: int = 0
    publish_errors: int = 0

@dataclasses.dataclass
class RimStats:
    ports: dict[str, RimPortStats]
    publish: dict[str, RimPublishStats]
    align: AlignStats        # T5's counters, live view
    run: RunHooks            # T6's tick/emit/skip/drop/error counters
```

- All counters monotonic, mutated by their owning thread only, read
  lock-free from anywhere (momentarily-stale reads are fine) — the T5/T6
  convention extended.
- `session.state`, `session.error` complete the surface. This is what
  T9's wall-clock pacer will read (T9 seam: the pacer holds the
  `LiveSession`, samples `stats`, never touches the tick path).
- **§10.3 boundary note**: the `m.over()` offline-stats gap flagged by
  the VoxelMapper report is NOT solved here (`over()` returns a bare
  iterator). The rim owns the LIVE surface only; the offline surface is
  T9's boundary to design (candidate: `run_over`'s existing `hooks=`
  kwarg). Recorded so T9 inherits the decision explicitly.

---

## 11. Error catalog (release copy)

`RimRule(enum)` slugs + `RimError(PureModuleRunError)`; `{cls}` =
`{module}.{qualname}` as everywhere.

| slug | message (normative copy) |
|---|---|
| `rim-unknown-port` | *(AttributeError, not RimError)* `{Bundle} has no port {name!r} — ports: {fields}. [rim-unknown-port]` |
| `rim-both-bindings` | `{cls}.i.{port}: transport and source are mutually exclusive — a port is fed by the wire or by a local stream, never both. Clear one first. [rim-both-bindings]` |
| `rim-bad-transport` | `{cls}.i.{port}: {obj!r} is not a transport — it lacks callable {missing}. A transport exposes publish(msg) and subscribe(callback) -> unsubscribe. [rim-bad-transport]` |
| `rim-bad-source` | `{cls}.i.{port}: {obj!r} is not a source — it lacks callable subscribe. Sources are subscribables: memory2 streams, rim out ports, legacy In streams. [rim-bad-source]` |
| `rim-bad-capacity` | `{cls}.i.{port}.capacity must be an int >= 1 or None (unbounded), got {value!r} — it bounds the delivery ring; 1 keeps only the freshest item. [rim-bad-capacity]` |
| `rim-rebind-running` | `{cls}.i.{port}: cannot rebind while the module is running — stop() first, rebind, start(). [rim-rebind-running]` |
| `rim-already-running` | `{cls} is already running — one live session per instance. stop() it, or build a second instance (identity is class + config). [rim-already-running]` |
| `live-not-pure` *(adapter)* | `live() takes a PureModule subclass, got {obj!r}. [live-not-pure]` |
| `live-port-collision` *(adapter)* | `{cls}: In and Out both declare a port named {name!r} — legacy autoconnect has one namespace per module, so live() cannot expose both. Rename one field. [live-port-collision]` |
| `live-reserved-port` *(adapter)* | `{cls}: port {name!r} collides with a legacy Module member — rename the field. [live-reserved-port]` |
| `live-config-collision` *(adapter)* | `{cls}: config field {name!r} collides with ModuleConfig.{name} (coordinator plumbing) — rename the field. [live-config-collision]` |

Wiring-completeness errors are deliberately NOT duplicated: `start()`
surfaces T5's `[align-missing-tick-stream]` /
`[align-missing-required-stream]` verbatim (§7.2 step 3). Fan-out
failures log with slug `[rim-publish-error]` (accounting, never raised).
Adapter errors are `PureModuleDefinitionError`s (definition-time), rim
errors are `RimError(PureModuleRunError)` (run-time) — same split T3/T7
use.

---

## 12. Coordinator adapter (`coordination.py`)

### 12.1 `live()` — the factory

```python
def live(cls: type[PureModule], *, name: str | None = None) -> type[Module]
```

Synthesizes (memoized per `(cls, name)`) a legacy-`Module` subclass — the
**shell** — holding one PureModule instance (`self._pure`). Parity by
construction (D1): every matrix row marked "inherited" is the identical
legacy code path.

Synthesized class dict:

- `__name__` / `__qualname__`: `name or cls.__name__` (D2). Same-name
  default reads naturally in blueprints/topics/instance keys; the edge —
  a legacy class and its pure twin in one blueprint colliding on
  `name.lower()` — surfaces as the blueprint's own duplicate-name error,
  fixed by `live(X, name="XLive")` or `instance_name`.
- `__module__`: `dimos.pure.coordination` (honest — it does not live in
  the pure class's source module; see P27).
- `__annotations__`: per P9 — `{f: In[T_f]}` from
  `spec.in_type.fields()`, `{f: Out[T_f]}` from `spec.out_type.fields()`
  (each `T_f` = the FieldSpec annotation with `| None` stripped), plus
  `config: <Name>LiveConfig` and the T9 health seam (§12.4). Validation
  first: In/Out name collision → `[live-port-collision]`; any field name
  present on legacy `Module` (computed as `dir(Module)` ∪
  `{"config", "ref", "rpc", "tf"}`) → `[live-reserved-port]`.
- `dedicated_worker = True` (P18); `deployment = "python"` inherited.
- `__pure_class__ = cls`, `__live_name__ = name` (reducer inputs).
- Pickling (P1): a dedicated metaclass `_LiveMeta(type)` registered with
  `copyreg.pickle(_LiveMeta, _reduce_live_class)` where
  `_reduce_live_class(c) -> (live, (c.__pure_class__,), {"name":
  c.__live_name__})`-shaped recipe (spell it as a module-level partial so
  the reducer itself pickles by name). pickle's `save` consults the
  copyreg dispatch before the `issubclass(t, type)` `save_global`
  fallback, so the class crosses the forkserver pipe by recipe;
  memoization keeps parent-side identity stable and rebuilds identically
  in the worker (pinned by TC pickle tests).

### 12.2 Config (P2–P4, P14)

`<Name>LiveConfig = create_model(..., __base__=ModuleConfig,
**pure_fields)` where `pure_fields` come from
`cls.__pure_config_model__.model_fields` (annotation + default). A pure
field named like a `ModuleConfig` field → `[live-config-collision]`.
Result: `g`, `instance_name`, `frame_id_prefix`, rpc/tf plumbing absorbed
by the base; pure fields typed and validated on the same model; the
blueprint CLI/env override surface (P14) sees both.

`__init__` (runs inside the worker, P1): inherited `Module.__init__`
consumes kwargs into `self.config` and builds the legacy `In`/`Out`
stream objects from the synthesized annotations; then the adapter's
`__init__` tail constructs the pure instance from the pure subset:

```python
self._pure = cls(**{n: getattr(self.config, n) for n in pure_field_names})
```

— revalidated through the frozen pure model; module identity (class +
config) intact. `_pure` is underscore-private: invisible to
`inputs`/`outputs` scans and to the Actor `__getattr__` passthrough.

### 12.3 Lifecycle mapping (P7, P15–P17)

The adapter overrides three methods, keeping the `@rpc` tags (they are
what the coordinator's `RpcCall`s dispatch to):

- `build()` → `rim.warmup(self._pure)`. Heavy sync resource creation in
  the 24 h-timeout slot.
- `start()` → bind, then run:
  1. For each In field with a wired legacy stream
     (`self.inputs[f]._transport` set by `set_transport`, P10):
     `self._pure.i.<f>.source = self.inputs[f]` — the legacy `In` stream
     IS a `SourceLike` (`In.subscribe(cb)` → `transport.subscribe(cb,
     self)`). Transports stay legacy-owned under the coordinator (D15);
     the rim's own `.transport =` binding is the standalone path (sketch
     `_shape_at_wiring_time`).
  2. For each Out field: `self._pure.o.<f>.subscribe(
     self.outputs[f].publish)` — routes through legacy `Out.publish`,
     which broadcasts on the shared transport AND feeds legacy local
     subscribers (`peek_stream` parity, P20).
  3. `rim.start(self._pure)`; then `super().start()` (inert without
     `main`/`handle_*`, P26).
- `stop()` → `rim.stop(self._pure)` (drain + dispose, idempotent) then
  `super().stop()` (legacy latch: loop thread, rpc, transports, ref-cycle
  break). Rim first: the engine must drain before legacy closes the
  transports under it.

`set_transport`, `set_module_ref`, `ref`, `peek_stream`, `blueprint`,
`rpcs`, `io`, pickling: inherited, untouched.

### 12.4 Health topic (P21, T9 seam)

Convention pinned by T8: stream/annotation name **`health`**, direction
Out, payload type `dimos.pure.health.Health` (T9). Because every live
module declares the same (name, type), standard autoconnect matching
gives one shared **`/health`** topic across all pure modules in a
blueprint — the sketch's merged, path-keyed `g.health` for free, and the
"one extra conventionally-named health topic" of the amendment. `live()`
carries a marked `# T9 HEALTH SEAM` at the annotation-synthesis site;
until T9 lands the annotation is omitted (matrix row P21 is completed by
T9's tests).

---

## 13. Engine seam edits (spec'd here, applied by the implementer)

All marked `# T8 RIM SEAM`.

### 13.1 `typing.py` — accessor bodies (T8a)

`_InAccessor.__get__` / `_OutAccessor.__get__` runtime bodies (currently
`raise NotImplementedError`):

```python
def __get__(self, obj: Any, owner: type | None = None) -> Any:
    if obj is None:
        return self
    from dimos.pure.rim import build_in_ports  # T8 RIM SEAM — lazy: sanctioned edge #2
    return build_in_ports(obj)
```

(`build_out_ports` for `_OutAccessor`.) Per T4 §5.5 verbatim; overloads
untouched.

### 13.2 `module.py` — lifecycle bodies (T8a)

`PureModule.warmup/start/stop` no-op stubs become:

```python
def warmup(self) -> None:
    from dimos.pure import rim  # T8 RIM SEAM — lazy, sanctioned
    rim.warmup(self)
```

(likewise `start`/`stop`). Service-protocol signatures unchanged.

### 13.3 `dimos/memory2/stream.py::Stream.transform` (T8b)

§9.3's duck-sniff routing. Exact rule: `if getattr(type(xf),
"__pure_step__", None) is not None:` → resolve the module's single
required In field from the stored `StepSpec`; bind `self` to it via
`over()`; wrap resulting rows as observations (`ts=row.ts`). Zero
`dimos.pure` import (the attribute IS the marker). Multi-required-input
modules → `TypeError` naming fields and `over()`.

### 13.4 NOT edited (decisions)

`drivers.py`, `resources.py`, `align.py`: untouched. The rim composes
their public API (D4); the `_chain` hardening is declined with rationale
(§7.3, D12). If T8a implementation discovers a genuine need, that is a
Relitigation case, not a quiet edit.

---

## 14. Edge cases — index watch-outs and amendment clauses → spec

| watch-out / clause | where handled |
|---|---|
| transports deliver on arbitrary threads, never drop by policy | §5.1 rings; §5.3 |
| all backpressure in engine buffers | §5.3, capacity default D6 |
| one loop per module; marshal, don't lock | §5.1, §8 |
| `over()` structurally unable to touch transports | §6 |
| source/transport mutually exclusive | §4.2 |
| stop idempotent, reverse-topological, drains before disposing | §7.3, §7.4 |
| reuse StreamAccessor/Stream conventions, no parallel stream API | §9 (recipes, not new API) |
| `kwargs["g"]` wrinkle | P2, §12.2 |
| one module per process | P18 (`dedicated_worker`) |
| parity-scope honesty (no legacy habits, no legacy RPC) | §1.3, P22–P25 |
| acceptance bar (blueprint, autoconnect, both directions, health) | §15.4, §16 |
| no wall clock in the data path | §5.2 (join timeout at lifecycle boundary only) |
| T7: `_chain` sequential-teardown flag | D12, §7.3 |
| T7 §18 Q2: lazy-cache disposal | D11, §7.3 |
| T5/T6 stats reachability (VoxelMapper report) | §10 + §10.3 boundary note |
| unstamped Out field types | §5.4 consequence note |
| In/Out sharing a field name (legal in T1) | adapter-only restriction, `[live-port-collision]` §12.1 |
| Python 3.10 floor | no 3.11+ constructs in skeletons |

---

## 15. Test plan

### 15.1 `test_rim.py` (T8a; skeleton lands skipped)

Fixtures: `FakeTransport` (TransportLike; records published msgs;
`deliver(msg)` invokes subscribers from a *spawned thread* to exercise
the marshal); `FakeSource` (subscribable over a list); probe modules
(stateless, mealy+resource, async, fold) over a `Sample(ts, v)` msg.

1. Port views: `m.i.x` is a `RimInPort`; identity stable across accesses;
   class access reveals the accessor; unknown name → AttributeError
   naming ports.
2. Binding: transport-then-source → `[rim-both-bindings]`; junk objects →
   `[rim-bad-transport]`/`[rim-bad-source]`; capacity validation; rebind
   while running → `[rim-rebind-running]`.
3. Live e2e (stateless): ticks delivered from a foreign thread; out
   transport receives stamped values in tick order; local subscriber sees
   the same.
4. Marshal + align: tick + interpolate secondary delivered from two
   threads; emitted rows carry correctly interpolated values (semantics
   delegated to T5, pinned end-to-end here).
5. Backpressure: step blocked on an event; N ticks delivered; default
   capacity 1 → exactly the freshest processed after release, drops
   counted per port; `capacity=None` → all processed.
6. Lifecycle: warmup creates resources in declaration order (recorder
   resource); stop disposes in reverse; stop idempotent; stop-before-
   start disposes warmed; warmup factory failure unwinds created prefix
   and re-raises `ResourceError`; second `start()` while running →
   `[rim-already-running]`; start-after-stop = fresh session, fresh
   resources.
7. Drain-before-dispose: items queued, `stop()` immediately; queued rows
   still emitted before teardown.
8. Async module live: fake async client; in-flight window respected;
   `stop()` drains window; `aclose` awaited (T6 semantics through the
   rim).
9. Sparse Out field `None` → no publish on that port, sibling port fires.
10. Fan-out error: raising subscriber → counted, logged, run continues.
11. `over()` isolation: transport-bound module's `over()` ignores
    bindings; resource-bearing module `over()` during live →
    `[resource-concurrent-run]`; resource-free module `over()` during
    live succeeds.
12. Stats: counters visible cross-thread; `session(m).state` / `error`.
13. Missing tick binding: `start()` raises T5's
    `[align-missing-tick-stream]` on the caller thread.
14. Source path: `FakeSource` feeding; Observation-unwrap sniff (fake
    `dimos.memory2.stream`-shaped module in `sys.modules`, as T6's
    coercion tests do).

### 15.2 `test_coordination.py` (T8b; skeleton lands skipped)

1. Synthesis: annotations (name → In/Out[T], direction, `| None`
   stripped); `BlueprintAtom.create(live(X), {})` StreamRefs — parity
   row P9 with no processes; `dedicated_worker`; `rpcs` ⊇
   {build, start, stop, set_transport}; `hasattr(on_system_modules)` is
   False.
2. Config: construct with pure kwargs + `g=` / `instance_name=` /
   `frame_id_prefix=` (P2–P4); typo kwarg → pydantic error; collision
   errors (`[live-port-collision]`, `[live-reserved-port]`,
   `[live-config-collision]`); `Blueprint.create(live(X)).config()`
   exposes pure fields (P14).
3. Identity + pickle: `live(X) is live(X)`; class pickle round-trip (P1);
   instance pickle pre-start (P6); `a.ref = obj` OK while `a._pure`
   stays frozen (P5).
4. Lifecycle without coordinator: `set_transport(f, FakeTransport())` on
   both sides; `build(); start()`; deliver on the in-transport from a
   thread; out-transport receives; legacy local subscriber
   (`adapter.outputs["y"].subscribe`) also receives (P20 peek path);
   `stop()` twice OK (P16); `blueprint` classproperty returns a one-atom
   blueprint (P13).
5. E2E (§15.4), marked per repo conventions.

### 15.3 Skeleton-phase gating

Both test files carry a module-level
`pytestmark = pytest.mark.skip(reason="T8 skeleton — bodies land with "
"T8a/T8b")`; they must COLLECT cleanly (imports resolve against the
skeletons) and the suite stays `304 passed, 1 deselected` plus these
skips. `uv run mypy` on the T8 files stays clean.

### 15.4 The acceptance-bar e2e (real test, T8b)

The harness permits it: `dimos/core/coordination/test_worker.py` /
`test_module_coordinator.py` already spawn forkserver workers in CI
(`@pytest.mark.skipif_macos_bug` convention). Specification:

- Test module defines (importable at module level, per test_worker
  patterns): `PureEcho(PureModule)` — In `x: Sample = tick()`, Out
  `y: Sample = contract(min_hz=1)` (`Sample` stamped + picklable, no
  `lcm_encode` → pLCM route, P11); legacy `Feeder(Module)` with
  `x: Out[Sample]` and legacy `Sink(Module)` with `y: In[Sample]` +
  `handle_y` collecting.
- `autoconnect(Feeder.blueprint(), live(PureEcho).blueprint(),
  Sink.blueprint())` → `ModuleCoordinator.build(...)`.
- Assert: pure module deployed on its OWN worker (P18, pids); `Feeder`
  publishes on `/x` → `Sink` receives transformed values via `/y`
  (**data both directions through the pure module**); coordinator
  `stop()` clean, no orphan processes (run-id sweep conventions).
- Health-visible assertion joins at T9 (P21); until then the e2e pins
  wiring + lifecycle + transport + topics — T8's share of the acceptance
  bar, stated honestly.

---

## 16. Acceptance checklist

- [ ] Parity matrix rows P1–P20, P26, P28 each pinned by a passing test
      (P21 name convention pinned; payload deferred to T9; P22–P25, P27
      documented out-of-scope).
- [ ] `m.i`/`m.o` runtime views per T4 §5.5; unknown-name AttributeError.
- [ ] Live loop: foreign-thread delivery → tick-ordered stamped output;
      KeepLast default; drops counted at three layers.
- [ ] `warmup/start/stop` per §7 incl. both idempotency rails and
      drain-before-dispose.
- [ ] `drive_async` reused unchanged for live async modules.
- [ ] No engine files changed beyond §13's marked seams; `rim.py` imports
      no `dimos.core`.
- [ ] `over()` isolation tests green.
- [ ] memory2: source feed + append recipe + `transform()` seam behavior.
- [ ] E2E per §15.4 green in CI (linux).
- [ ] mypy strict-clean; suite green.

## 17. Decisions (numbered for review)

- **D1** Adapter inherits legacy `dimos.core.module.Module` — parity by
  construction; matrix rows become behavior tests, not reimplementation.
- **D2** `live()` memoized factory; synthesized class pickles by recipe
  (copyreg + `_LiveMeta`); `__name__` defaults to the pure class's name
  with `name=` override.
- **D3** One-module-per-process via `dedicated_worker = True` — zero
  coordinator changes.
- **D4** Rim composes PUBLIC engine API (`align` + `drive_*` +
  `attach_resources` + `RunHooks`); `run_over` untouched; no engine
  edits beyond §13's seams.
- **D5** Warmup timing: sync-shaped modules create resources at
  `warmup()`/`build()` on the caller thread (then neutralize the
  driver's warmup seam); async modules defer creation to the run loop at
  `start()` — validation-only warmup, documented.
- **D6** Single engine thread per module; per-port rings between
  transport threads and engine; default capacity 1 (KeepLast, prior-art
  controller semantics), `capacity = n | None` knob per port.
- **D7** Fan-out: per-field raw stamped msgs, declaration order, `None`
  skips, transport before local subscribers; fan-out exceptions counted
  + logged, never fatal.
- **D8** Transports/sources duck-validated at bind against structural
  protocols; static surface stays `Any` (T4 shipped it; settability is
  the point).
- **D9** `source =` accepts subscribables (unsub-callable or Disposable,
  normalized); memory2 Observations unwrapped at the ring via
  sys.modules sniff; `transform()` routed by the `__pure_step__`
  duck-marker.
- **D10** Stop: unsubscribe → close-with-drain → driver self-teardown →
  bounded join (5 s, warn) → latch; stop-before-start disposes directly.
- **D11** T7 Q2: live stop leaves the lazy cache untouched.
- **D12** T7 `_chain` hardening declined — rim injects nothing into the
  hook seams.
- **D13** Live stats surface = `rim.session(m).stats` (`RimStats`
  composing ring/align/run counters); offline-stats gap explicitly left
  at T9's boundary.
- **D14** Health stream name `health`, shared `/health` topic via
  standard (name, type) matching; payload/cadence T9.
- **D15** Under the coordinator, transports stay legacy-owned: adapter
  binds legacy `In` streams as rim *sources* and legacy `Out.publish` as
  rim *subscribers*; rim's own `.transport =` is the standalone path.
- **D16** Errors: `RimError(PureModuleRunError)` + slug enum; unknown
  port name is `AttributeError`; adapter synthesis errors are
  `PureModuleDefinitionError`s.
- **D17** `live()` rejects In/Out port-name collisions and
  legacy-reserved names at synthesis (adapter-only restriction; T1's
  shared-name legality is untouched for pure-only use).
- **D18** `restart_module(reload_source=True)` is a documented gap (P27)
  deferred to the reload wave.
- **D19** `over()`-during-live: rejected via T7's concurrent-run guard
  for resource-bearing modules; allowed (and harmless) for resource-free
  ones — both pinned.

## 18. Division of labor — the T8a/T8b split

| | T8a (rim core) | T8b (adapter + parity) |
|---|---|---|
| implements | `rim.py` bodies; §13.1 + §13.2 seams | `coordination.py` bodies; §13.3 seam |
| enables | `test_rim.py` (drop module skip) | `test_coordination.py` incl. §15.4 e2e |
| depends on | T1–T7 as landed | T8a merged |
| out of scope | anything importing `dimos.core` | engine edits beyond §13.3 |

T8a is fully testable with fakes (no coordinator, no LCM). T8b touches no
engine file and no rim file — if the adapter needs something the rim
doesn't expose, that is a spec gap to escalate, not an ad-hoc edit.

Exports: `dimos/pure/__init__.py` is orchestrator-owned; T8a adds nothing
to the `pm` surface (ports are reached via `m.i`/`m.o`; `session` stays a
rim-qualified call — `from dimos.pure import rim; rim.session(m)`). T8b
exports `live` from `dimos.pure.coordination` only (not on `pm.*` — it is
bridge vocabulary, and it sunsets with the legacy system).

## 19. Open questions (non-blocking; defaults chosen)

- **Q1**: Should `RimOutPort.subscribe` also offer a row-level
  subscription (whole Out row, not per-field)? / default: NO — per-field
  is the T4 surface and the recording story goes through per-field
  stores; a row consumer can use `over()` or subscribe every field /
  options: (a) per-field only (default), (b) add a row hook later
  without breaking anything.
- **Q2**: Engine-thread join timeout on `stop()` — 5 s default? /
  default: yes, constant `STOP_JOIN_TIMEOUT_S = 5.0`, log ERROR and
  abandon the daemon thread on expiry / options: (a) 5 s (default),
  (b) configurable later if field evidence demands.
- **Q3**: Should `live()` (T8b) eagerly validate that every In field
  type is transport-serializable? / default: NO — serializability is the
  transport's concern (pLCM pickles anything); failures surface at
  publish with the transport's own error / options: (a) no check
  (default), (b) warn-once for exotic types.
- **Q4**: `capacity` default of 1 for the tick port of *fold* modules
  (fold may want every row)? / default: keep 1 uniformly — fold modules
  wanting recorder semantics set `capacity=None` like everyone else; one
  rule, no per-kind magic / options: (a) uniform (default), (b) fold
  defaults to `None` — revisit with a real fold-live user.
