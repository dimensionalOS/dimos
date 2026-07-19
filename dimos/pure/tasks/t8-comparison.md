# T8 A/B comparison verdict

Judge run, 2026-07-20, over `pure/spec-t8-rim` @ `1fd005847` (both variants
present). Contestants:

- **Variant A** (canonical paths): `t8-rim.md`, `rim.py`, `legacy.py`,
  `test_rim.py`, `test_legacy.py`. Factory `legacy_actor(cls)` / spelling
  `legacy_blueprint`.
- **Variant B** (suffixed): `t8-rim-b.md`, `rim_b.py`, `coordination_b.py`,
  `test_rim_b.py`, `test_coordination_b.py`. Factory `live(cls)`.

Every fact-check below was resolved by reading the legacy code in this tree
and, where pure reading was insufficient (pickling), by running the
mechanism against the real pipe pickler (`multiprocessing.reduction.
ForkingPickler`).

## Verdict (one paragraph)

**Keep A as canonical; graft named B sections into it** (option i). The two
specs converge on every load-bearing architectural decision — subclass
legacy `Module` for parity-by-inheritance, synthesized `__annotations__`
answered from `fields()`, `dedicated_worker = True` for one-per-process,
compose the public engine API (`align` + `drive_*` + `attach_resources`)
instead of `run_over`, decline T7's `_chain` hardening, identical T7 §18 Q2
answer, drain-before-dispose with a 5 s join, shared `/health` by standard
`(name, type)` matching — so this is a graft, not a rewrite. A wins on
completeness (31-row matrix incl. wire currency, deploy-error channel, and
restart rows pinned by tests; the amendment's health bar actually honored in
T8; smaller seam footprint — no edit outside `dimos/pure`; richer stats
incl. `held_tick_ts`; the honest live-hold paragraph; broader test suite).
But A's single most load-bearing mechanism — pickling the synthesized actor
class across the deploy pipe — is **broken as specced**, and B has the
correct half of that fix (plus a second correctness catch: A's reserved-name
list is dangerously incomplete). Neither variant's pickling works verbatim;
the verified hybrid is A's metaclass base + B's copyreg registration.

## Dimension 1 — fact-check

### 1a. The four headline claims (B's report; did A find them, who is right)

| Claim | A | B | What the code says | Verdict |
|---|---|---|---|---|
| (a) Lifecycle/wiring calls ride the LCM/zenoh RPC rail at `{name}/{method}`, served by the module's own RPC server; the pipe is deploy/undeploy/attr-get/set_ref | Found (§2.1 "control plane is the RPC bus, not the pipe", P12); also notes `CallMethodRequest` on the pipe | Found (P7); says pipe = "deploy/undeploy/attr-get/set_ref **only**" | `RpcCall.__call__` → `rpc.call_sync(f"{remote_name}/{method}")`, `stop` → `call_nowait` (`rpc_client.py` ~63–81); proxy = `RPCClient(actor, …)` (`worker_manager_python.py` ~100); server started in `ModuleBase.__init__` via `rpc.serve_module_rpc(self, name=config.instance_name)` (`module.py` ~156–166). Pipe requests: Deploy/SetRef/GetAttr/**CallMethod**/Undeploy/SuppressConsole (`python_worker.py::_handle_request`) | **Both right on the rail.** B's "only" slightly understates: `CallMethodRequest` also rides the pipe (A had it). Immaterial to either design |
| (b) Autoconnect never calls the module; port discovery = `get_type_hints()` over class annotations with origin `In`/`Out` → adapter must synthesize annotations | Found (P5) | Found (P9) | `BlueprintAtom.create` runs `get_type_hints(module, globalns=…)` and collects `get_origin(annotation) in (In, Out)` → `StreamRef(name, type, direction)`; nothing else consulted (`blueprints.py` ~106–127) | **Both right.** Annotation synthesis is mandatory; both specs do it |
| (c) `Actor.set_ref` executes `instance.ref = actor` — a mutating write, fatal on a frozen PureModule without a shell | Found (P17; relies on the mutable actor) | Found (P5; explicitly names it as the *second* reason, after `g`, that the adapter must be a shell) | `case SetRefRequest(...): state.instances[module_id].ref = ref` — plain setattr (`python_worker.py::_handle_request`); called on every deploy (`actor.set_ref(actor)` in `deploy_module`) | **Both right.** B's framing (shell forced by two independent legacy behaviors) is the sharper statement of *why* |
| (d) `stop()` arrives twice, on different threads, in normal shutdown | Found (P15, "double-stop is the normal path") | Found (P16, "two rails") | Coordinator `stop()`: `reversed(_deployed_modules)` → `module.stop()` (RPC rail); worker `finally`: `instance.stop()` for all instances (`python_worker.py` ~352–368); `_close_module` carries a latch with the literal comment "race when stopping concurrently (from RPC and worker shutdown)" (`module.py` ~201–231) | **Both right** |

### 1b. Genuine disputes / asymmetric claims

| Claim | A's position | B's position | What the code/interpreter says | Verdict |
|---|---|---|---|---|
| **Pickling the synthesized class over the deploy pipe** | Metaclass defines `__reduce__ → (legacy_actor, (PureCls,))`; class "pickles through the pipe (P1)" | `copyreg.pickle(_LiveMeta, reducer)`; explicitly argues pickle consults the copyreg dispatch table **before** the `issubclass(t, type)` → `save_global` fallback | Empirically verified (plain pickle AND `ForkingPickler`, the actual `Connection.send` pickler): a metaclass `__reduce__` on a class object is **ignored** — pickle hits the `issubclass(t, type)` branch and calls `save_global`, which fails on a class that isn't importable → `PicklingError`. The copyreg registration works and round-trips to the cached identical class | **A's P1 mechanism is broken; B's copyreg is correct.** A's TL-pickle test would fail on day one of T8b. B's mechanics explanation matches CPython's `pickle.py`/`_pickle.c` exactly |
| **Metaclass base for the synthesized class** | `_ActorMeta(type(Module))` — "resolved dynamically (ABCMeta via Resource)" | `_LiveMeta(type)` (spec §12.1 and skeleton `coordination_b.py:40`) | `type(Module) is abc.ABCMeta`; synthesizing a `Module` subclass with a plain-`type` metaclass raises `TypeError: metaclass conflict` (verified) | **A right, B broken.** B's `live()` would crash at synthesis time |
| → net on pickling | — | — | The **verified hybrid**: `class _Meta(type(Module))` + `copyreg.pickle(_Meta, reducer)` round-trips over `ForkingPickler` to the cached class | **Both wrong as written; each holds the other's fix.** Mandatory graft |
| **memory2 `transform()` seam** | No memory2 edit needed: `Stream.transform` "accepts bare callables (`stream.py:384-401`)"; S5 adds `PureModule.__call__` (a `dimos/pure`-only edit) delegating to `rim.transformer` | Requires a marked seam edit in `dimos/memory2/stream.py::Stream.transform` (duck-sniff `__pure_step__`) | `Stream.transform(xf: Transformer \| Callable[[Iterator[Obs]], Iterator[Obs]])` — non-`Transformer` callables are wrapped in `FnIterTransformer` (verified at `dimos/memory2/stream.py:384–401`). A callable PureModule needs zero memory2 changes | **A right; B's memory2 edit is unnecessary** and enlarges the out-of-tree footprint for nothing. A's `derive(data=row, ts=row.ts)` re-enveloping also preserves observation metadata, which B's plain wrap loses |
| **Reserved-name validation set** | Static frozenset: `{ref, config, rpc, tf, io, inputs, outputs, rpcs, name, blueprint, module_info, health}` + underscore names (`legacy.py:50–66`) | Computed: `dir(Module)` ∪ `{config, ref, rpc, tf}` | A's list misses `set_transport`, `peek_stream`, `stop`, `start`, `build`, `main`, `set_module_ref`, … — a pure field named `stop` would synthesize `stop: In[T]`, and `Module.__init__` would `setattr(self, "stop", In(...))`, shadowing the lifecycle RPC method. Silent catastrophic breakage | **B right.** A's explicit list is a correctness bug; `dir(Module)` is drift-proof |
| **Health topic in T8** | Actor synthesizes `health: Out[_HEALTH_PAYLOAD]` now (placeholder `object`); topic wired, silent; e2e asserts wired (P21, D12, Q3) | Annotation **omitted** until T9 — only a `# T9 HEALTH SEAM` comment; e2e explicitly excludes health, "stated honestly" | The amendment (index §T8, binding) puts "health visible" in the T8 acceptance bar: "…autoconnected, data flowing both directions, health visible" | **A conforms to the binding amendment; B does not** (its honesty framing doesn't override "binding"). A's `Out[object]` placeholder is ugly (autoconnect key `("health", object)`) — see architect item AD2 |
| **Line-number citations** | Mostly accurate; one bogus: `blueprints.py:827` for `instance_name` (file is 448 lines; real sites ~144, ~331) | Spot-checked citations all accurate (bp:97–154, bp:333–338, pw:226, pw:374–377, mc:298–330, mc:663–665, mod:156–166, mod:782–792, rc:63–81, st:169/235, rpc timeouts 86400/1200/120) | — | Both matrices are factually sound about the *mechanics*; A carries one dead line ref (cosmetic) |

Everything else the two matrices assert in common was spot-checked true:
`kwargs["g"]` force-injection + worker-side `kwargs.get("g")` backend sync,
`(remapped_name, type)` grouping with one transport per key, `/{name}` vs
`/{short_id()}` topics, `In.subscribe(cb)` → `transport.subscribe(cb, self)`,
`Out.publish` → transport broadcast **and** local subscribers, `In/Out
__reduce__` → `RemoteIn/RemoteOut` via `owner.ref`, `dedicated_worker`
routing, `restart_module(reload_source=True)`'s `importlib.reload` +
`getattr(source_mod, cls.__name__)` (both variants' documented gap is real;
A additionally pins its failure mode with a test).

## Dimension 2 — architecture scores

Scores are A vs B out of 5, with the reason. Where the designs are
convergent no score is given.

| Axis | A | B | Reasoning |
|---|---|---|---|
| Adapter approach | 4 | 4 | Same architecture (shell subclassing legacy `Module`; frozen pure instance inside; `ModuleConfig`-based `g`/`instance_name`/`frame_id_prefix` absorption; `extra="forbid"` intact both layers). A: right metaclass, broken pickle, incomplete reserved-name list, no name-collision escape. B: right pickle, broken metaclass, `dir(Module)` validation, `name=` override for blueprint collisions, honest `__module__`. Net wash — the hybrid is strictly better than either |
| Rim-core threading | 5 | 4 | Identical spine (delivery threads only enqueue; ONE session thread runs align→drive→emit+teardown; `drive_async`'s private loop inside it; lock only around ring ops; wall clock only at the stop-join). A adds the "honest paragraph" on live hold semantics with `held_tick_ts` surfaced in stats — the operationally crucial bit B leaves implicit (and B's stats lack the held gauge entirely) |
| Backpressure | 3 | 4 | A: fixed depth 256 drop-oldest, override only via `start(port_depth=)` mapping, **no unbounded option** — a recorder-style module cannot opt out of drops (A's lossless story exists only for iterable-source pumps, not transports). B: per-port `capacity` attribute, default 1 (KeepLast, cites the prior-art controller table), `None` = unbounded. B's knob is the right surface; the *default* (1 vs 256) is an architect call (AD1) — note A's own justification ("drop-oldest is latest-wins; legacy `backpressure()` coalesces to latest") argues for B's 1 |
| Engine-seam footprint | 5 | 4 | Both compose only public API (`align` + `drive_*` + `attach_resources` + `RunHooks`), both decline `run_over` reuse and T7 `_chain` hardening with the same architectural argument (rim injects nothing into hook chains). A: 3 seams, all inside `dimos/pure` (typing accessors, module lifecycle bodies, `__call__`). B: same 2 + an **unnecessary** `dimos/memory2/stream.py` edit (see fact-check) |
| Layering cleanliness | 5 | 5 | Identical invariants: rim core never imports `dimos.core`/`dimos.protocol`/`dimos.memory2`; bridge is a leaf never re-exported from `__init__`; structural `Publishable/Subscribable` (A) ≙ `TransportLike/SourceLike` (B), both verified against legacy `Transport`'s actual signatures. A additionally pins the import ban with a grep test |
| Lifecycle correctness | 4 | 5 | Both: idempotent latched stop, drain-before-dispose, stop-before-start direct teardown, fresh session per start, warmup-neutralization trick for the sync/build mapping, async-resources-at-start divergence documented, same T7 Q2 answer. B is more complete on the failure paths: `warmup()` factory-failure unwind (reverse prefix + re-raise) and `start()` partial-failure reverse unwind — A never specs partial-start failure. B's concurrent-stop "losers join the latch" beats A's "losers return immediately" (a caller returning before drain completes is a real race for undeploy) |
| T8a/T8b split | 5 | 5 | Both clean: T8b consumes only rim public functions; both state "adapter needs → spec gap, not ad-hoc edit". Equivalent |
| Test quality | 5 | 4 | Both suites are real bodies behind a module skip; both collect green today (verified: 412/413 + skips). A: ~72 tests, superset parity coverage — deploy-error via `_handle_request` without a process, restart-shape AND reload-limit pinned, autoconnect grouping + type-conflict, remote stream pickling, unstamped-out, hold semantics, transformer-vs-over equivalence, no-core-import grep guard, e2e asserting health wired + pids + transport-registry keys. B: ~60, with four genuinely valuable cases A lacks (live interpolation marshal — actually exercises T5 semantics through the rings; warmup-failure unwind; the D19 over()-during-live pair; observation-unwrap monkeypatch), and a 3-module Feeder→Pure→Sink e2e. **A's TL-pickle would fail as specced** (see fact-check) — graded here as a spec fault, already counted above |
| Doctrine conformance | 5 | 5 | Both: no wall clock in the data path (join-timeout boundary only, both cite the license); over() structurally isolated (A pins with a fakes-observe-zero-traffic test; B adds the sharper D19 resource-interplay analysis); API floor untouched; ports speak stamped msgs (A enforces producer-side on transport-bound ports; B consumer-side — AD4) |

**Overall: A 41/45, B 40/45** — but the aggregate hides the shape: A is the
better *document* (completeness, amendment conformance, footprint, tests) with
one fatal mechanism bug; B is the sharper *debugger* of legacy reality
(pickle mechanics, reserved names, failure paths) with a broken metaclass, a
missed amendment requirement, and an unnecessary out-of-tree edit.

## Dimension 3 — the graft

**Recommendation: (i) keep A canonical; graft the following from B.**
Delete `t8-rim-b.md`, `rim_b.py`, `coordination_b.py`, `test_rim_b.py`,
`test_coordination_b.py` after grafting.

### Mandatory (correctness — the winner must absorb these)

1. **G1 — pickling (both were wrong; hybrid verified in this review).**
   Replace A §11.1's metaclass-`__reduce__` clause with: metaclass
   `_ActorMeta(type(Module))` (A's base — `Module`'s metaclass is `ABCMeta`)
   **registered via `copyreg.pickle(_ActorMeta, <module-level reducer>)`**
   returning `(legacy_actor, (cls.__pure_class__,))` (B §12.1's mechanism +
   its rationale paragraph about pickle's dispatch order, which is exactly
   right). Reducer must be a module-level function so it pickles by name.
   Update `legacy.py` skeleton comment and keep TL-pickle (it will now pass).
2. **G2 — reserved-name computation.** Replace A §11.2's static
   `_MODULE_SURFACE` frozenset with B's computed set (`dir(Module)` ∪
   explicit extras). A's list misses `set_transport`/`peek_stream`/`stop`/
   `start`/`build`/`main`/`set_module_ref` — any of these as a pure field
   name silently shadows a lifecycle RPC method on the actor.
3. **G3 — start/warmup failure unwind.** Add B §7.1–7.2's failure clauses to
   A §6.1/§7.2: warmup factory failure → `hooks.teardown()` (reverse-prefix
   unwind) + re-raise; start failure at step N → unsubscribe/dispose in
   reverse, state STOPPED, re-raise. Graft B's two tests
   (`test_warmup_failure_unwinds_prefix`, implicit start-unwind coverage).
4. **G4 — concurrent stop joins the latch.** Adopt B's semantics (§7.3:
   "others join the same latch") over A's "losers return immediately" —
   P15/P16's two-rail stop means the second caller must not return before
   drain/dispose completes.

### Strongly recommended (coverage/ergonomics)

5. **G5 — parity row + test for remapping/namespace** (B's P12 + TC case):
   A's matrix has no row pinning that `remappings()`/`namespace()` rename a
   pure module's streams identically to a legacy module's. Free behavior,
   but the matrix's charter is "every outside-visible behavior → test".
6. **G6 — the D19 pair**: A tests over()-isolation only; graft B's two
   cases (resource-bearing module: `over()` during live → T7's
   `[resource-concurrent-run]`; resource-free: allowed, harmless) and B §6's
   two-sentence analysis.
7. **G7 — B's unique tests**: live interpolation marshal (two threads, T5
   interpolate through the rings), observation-unwrap via `sys.modules`
   monkeypatch, rim-session-reachable-through-shell.
8. **G8 — `name=` override on the factory** (B D2): the collision escape for
   a legacy class and its pure twin sharing `__name__.lower()` in one
   blueprint. One kwarg, cheap, real scenario during migration.
9. **G9 — B's honest `__module__`**: with G1's copyreg in place, A's
   `__module__ = cls.__module__` no longer serves pickling and actively
   misleads (P26's reload path resolves the WRONG class loudly, but a
   `dimos.pure.legacy` module string makes the synthesized-ness visible in
   logs/tracebacks). Keep A's `__name__`/`__qualname__` scheme. Adjust
   TL-restart-limit's pinned failure mode accordingly (becomes
   `AttributeError` on the reloaded module rather than the `g` rejection).

### Corrections to carry into A's text (no design change)

10. Fix the dead citation `blueprints.py:827` (real sites: ~144, ~331).
11. Note in P-row prose that `CallMethodRequest` also rides the pipe (A §2.1
    already says it; B's "only" phrasing should not survive the merge).

## Architect decisions (Ivan) — irreconcilable conflicts

| # | Conflict | A | B | Recommendation |
|---|---|---|---|---|
| AD1 | Ingress default | depth 256, drop-oldest; override via `start(port_depth={...})`; no unbounded | capacity 1 (KeepLast); per-port `m.i.x.capacity = n \| None` | **B** — per-port attribute matches the `.transport =` fluency, `None` covers recorder modules (A has no lossless transport path at all), and default 1 is the house coalesce-to-latest semantics A itself cites as justification. If 1 feels aggressive for the tick port, that's a one-constant change later |
| AD2 | Health topic timing | Synthesize `health: Out[object]` placeholder now; topic wired + e2e-asserted; T9 swaps the payload constant | Omit annotation until T9; seam comment only | **A's timing** (the amendment's acceptance bar includes "health visible" and is marked binding), but replace the `object` placeholder with a minimal named placeholder type in `dimos/pure` so the autoconnect key is never `("health", object)` — T9 then replaces the type, not the mechanism |
| AD3 | Out-side bridge binding | `pure.o.f.transport = legacy_out` (legacy Out is the rim port's Publishable; publish failure = session error; `published` counters; local-subscriber slot stays free) | `pure.o.f.subscribe(legacy_out.publish)` (fan-out slot; failures counted, never fatal) | **A** — a dead wire should be a real failure, not a warning counter, and keeping the local-subscribe slot free is what makes the one-line recording lambda composable under the coordinator |
| AD4 | Unstamped Out payloads on transport-bound ports | Producer-side check `[rim-unstamped-out]` (first publish of ts-less payload raises) | No producer check — consumer's aligner enforces; ts-less types are "legacy-interop-only", documented | **A** (doctrine: "ports speak stamped msgs" — the producer is the right place to blame), but be aware A forbids a pure module publishing ts-less legacy types to legacy-only consumers, which B deliberately permits. If migration surfaces a real ts-less legacy interop need, downgrade to warn-once |
| AD5 | transform() seam | `PureModule.__call__` (S5, `dimos/pure`-only) — `Stream.transform` already accepts callables; re-envelope via `last_obs.derive(data=row, ts=row.ts)` | Duck-sniff edit inside `dimos/memory2/stream.py` | **A** — fact-check settled this: the memory2 edit is unnecessary, and `derive` keeps observation metadata. Not truly irreconcilable, listed because it deletes one of B's spec'd seams |
| AD6 | Bridge naming | `legacy.py`, `legacy_actor()`, `legacy_blueprint()` | `coordination.py`, `live()` | **A** — "legacy" self-documents the sunset intent (the file dies with the legacy system); `coordination.py` reads as permanent infrastructure and collides mentally with `dimos/core/coordination/`. Taste call, hence listed |

## Also for the record

- The B skeleton set is thinner than A's (`coordination_b.py` is 64 lines vs
  `legacy.py`'s 102 + helper stubs; `rim_b.py` omits the transformer and the
  queue/feed internals A stubs out). Not scored heavily — skeletons are
  scaffolding — but grafting into A is also the cheaper mechanical path.
- Both specs decided T7 §18 Q2 identically (live stop does NOT touch the
  lazy test-mode cache) with the same per-run/ownership rationale — that
  question can be marked closed in T7's tracker with two independent
  confirmations.
- Both specs independently discovered the same "surprising rows" (RPC rail,
  annotation-only autoconnect, mutating `set_ref`, double-stop) — the
  parity amendment's phase-1 investigation requirement did its job twice
  over; confidence in the shared findings is correspondingly high.
