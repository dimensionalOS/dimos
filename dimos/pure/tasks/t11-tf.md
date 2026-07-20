# T11 — TF: `tf()` sampler + `tf_out()` port: implementation spec

Target: `dimos/pure/rows.py` (specifiers — landed), `dimos/pure/module.py`
(build resolution — landed), `dimos/pure/tfbuffer.py` (engine skeleton —
bodies pending), plus wired extensions to `align.py`, `drivers.py`, `rim.py`,
`typing.py` (§6.6, §8.3, §9.5). Source of truth:
`dimos/memory2/puremodule_api_sketch3.py` (§5, §5b, §NOTE: TF) and
`tasks/index.md` §T11, built on landed T1–T8 reality. The math is ADAPTED from
`dimos/protocol/tf/tf.py` per the §13 mapping table — never forked, never a
service, `_wait_get` deleted.

**Judgment calls are numbered D1–D18 in §16.** Open questions Q1–Q6 in §17.
Two deviations from written-down source material are argued in §18 — the
engine file name (§18.1) and the `tf_out` sparse spelling (§18.2). Static
claims (mypy field-specifier default handling) were verified against mypy
under the repo strict config on this branch (§3.4).

What is already landed with this spec (live-tested, `test_tf.py`):
specifiers + call-time validation, frame templates + build-time resolution,
module-scope single-writer/self-edge/empty checks, `port.frames`
introspection, the `pm` surface additions. What the implementer lands:
`TfBuffer`/`TfContext` bodies, the align/drivers/rim integration, the
`tf_out` tap.

---

## 1. Scope and layer boundary

### 1.1 What T11 is

The tf sampler is `interpolate()` generalized by one step: interpolation PLUS
chain composition over one well-known stream (sketch §NOTE: TF). A module
declares:

```python
class In(pm.In):
    image: Image = tick(expect_hz=30)
    world_to_base: Transform = tf("world", "base_link")              # required
    world_to_cam: Transform | None = tf("world", "cam", default=None)  # optional

class Out(pm.Out):
    tf_world_map: Transform | None = tf_out("world", "map", default=None)
```

and gets chain-composed, time-interpolated transforms in its In row at every
tick, with zero extra ceremony — no manual slerp, no buffer, no service
handle, no wall clock. TF is a **sampler, not a service** (settled): no
threads, no callbacks, no wait loops; pull-based state interrogated by the
align engine at tick resolution. A module that declares no tf field pays
zero cost (pinned: `test_tf_free_module_pays_nothing`).

### 1.2 Files and the engine module's name (D1, D2)

- **`rows.py`** — `TfSpec`/`TfOutSpec` + the `tf()`/`tf_out()` specifiers +
  frame-template machinery (`format_frame`, `normalize_frame`). They MUST
  live here: `@dataclass_transform`'s `field_specifiers` tuple on `_Bundle`
  must name them for mypy to model required-ness, and that tuple is evaluated
  at `rows.py` import. Template logic is pure string/dataclass code — data
  layer, zero new imports.
- **`module.py`** — the build hook: templates resolve at instance
  construction against the instance's config (§3.2). Pure logic, no new
  import weight.
- **`tfbuffer.py`** — the engine: `TfBuffer`, `TfContext`, `tf_out_tap`,
  `attach`, the `TfError` catalog. A LEAF engine module like
  `interpolators.py`: it imports `Transform` (which pulls numpy via
  `Vector3`/`Quaternion`), so it is **never** imported by
  `dimos/pure/__init__.py` (the pinned zero-engine-import test bans numpy at
  surface import) nor by `align.py` at module scope — the aligner, `run_over`
  and the rim import it lazily, only when a bundle declares tf fields (the
  sanctioned lazy-edge pattern, T4 §5.5 / T6 §4).

Why not `dimos/pure/tf.py`, as the index layout proposed: Python binds a
submodule as an attribute on its parent package at import — importing
`dimos.pure.tf` would **clobber the `pm.tf` specifier function** on the
package namespace, and `from dimos.pure import tf` would return the module or
the function depending on import history. The sketch requires `pm.tf` to be
the specifier; the two cannot share the name. The function keeps the sketch
name; the file is `tfbuffer.py`. Argued as relitigation §18.1.

### 1.3 Layering map

```
rows.py          specs + templates          (data layer; no new deps)
module.py        build resolution           (imports rows, stepspec — unchanged deps)
tfbuffer.py      buffer + context + tap     (leaf: msgs + align + stepspec)
align.py         sampler seam               (lazy → tfbuffer, TYPE_CHECKING types)
drivers.py       run_over: tf= + tap        (lazy → tfbuffer)
rim.py           m.i.tf, session ctx, tap   (lazy → tfbuffer; frames landed eagerly)
typing.py        InPort.frames stub, over() tf= param (static only)
```

Consumption map:

| Consumer | Uses from T11 |
| --- | --- |
| user modules | `pm.tf`/`pm.tf_out` declarations; `m.i.X.frames`/`m.o.X.frames` |
| T6 `over()` | `tf=` argument → `TfContext.for_over` + `tf_out_tap` |
| T8 rim | `m.i.tf` port, `tfbuffer.attach`, `TfContext.for_live` |
| T9 health | `AlignStats.ports["tf"]`, `drops_by_field`, `TfBuffer.stats`, `held_tick_ts` |
| T10 checkpoint | nothing — the buffer is NOT snapshot state (§12.2) |
| wire()/plan (future) | `port.frames`, `format_frame`/`normalize_frame` for `tf_prefix` bind rewriting |

## 2. Public surface

Landed in `rows.py` (spelled exactly; see the file for bodies):

```python
@dataclasses.dataclass(frozen=True)
class TfSpec(FieldSpec):
    kind = "tf"; side = "in"
    parent: str = ""; child: str = ""          # templates, unresolved

@dataclasses.dataclass(frozen=True)
class TfOutSpec(FieldSpec):
    kind = "tf_out"; side = "out"
    parent: str = ""; child: str = ""
    min_hz: float | None = None                # T9 contract composition (Q3)

@overload
def tf(parent: str, child: str) -> Any: ...
@overload
def tf(parent: str, child: str, *, default: _T) -> _T: ...

@overload
def tf_out(parent: str, child: str, *, min_hz: float | None = None) -> Any: ...
@overload
def tf_out(parent: str, child: str, *, min_hz: float | None = None, default: _T) -> _T: ...

def normalize_frame(frame: str) -> str: ...     # drop empty '/'-segments
def format_frame(template: str, values: Mapping[str, Any]) -> str: ...
```

`pm` surface additions (pinned in `test_pm_surface.py`): `tf`, `tf_out`,
`TfSpec`, `TfOutSpec`. The engine stays qualified:
`from dimos.pure.tfbuffer import TfBuffer`.

Skeleton in `tfbuffer.py` (bodies pending):

```python
DEFAULT_TF_HORIZON: Final[float] = 10.0
STATIC_OWNER: Final[str] = "<static>"
EdgeKey: TypeAlias = tuple[str, str]                       # (parent, child)
TfSource: TypeAlias = TfBuffer | Iterable[Any] | tuple[TfBuffer, Iterable[Any]]

class TfError(AlignmentError):                              # rule: TfRule (§10)
class TfBuffer:
    def __init__(self, *, horizon: float = 10.0, statics: Iterable[Transform] = ()): ...
    def ingest(self, item: Transform | Any) -> None: ...    # TFMessage duck-unwraps
    def set_static(self, transform: Transform) -> None: ...
    def claim(self, edge: EdgeKey, owner: str) -> None: ...
    def release(self, owner: str) -> None: ...
    def resolve(self, parent: str, child: str, at: float) -> Transform | None: ...
    def frames(self) -> set[str]: ...
    def edges(self) -> set[EdgeKey]: ...
    stats: TfStats                                          # property

@dataclasses.dataclass
class TfContext:                                            # per-run, engine-built
    buffer: TfBuffer
    frames_in: Mapping[str, EdgeKey]; frames_out: Mapping[str, EdgeKey]
    required: frozenset[str]; owner: str
    stream: Iterator[Any] | None
    port_stats: PortStats
    @classmethod
    def for_over(cls, module, spec, tf_arg) -> TfContext | None: ...
    @classmethod
    def for_live(cls, module, spec, buffer, feed) -> TfContext | None: ...
    def advance_past(self, ts: float) -> None: ...
    def pull(self) -> bool: ...
    def resolve_field(self, name: str, at: float) -> Transform | None: ...
    def release(self) -> None: ...

def tf_out_tap(rows, module, ctx) -> Iterator: ...
def attach(module, buffer: TfBuffer) -> None: ...           # live-session injection
```

## 3. Declaration and build

### 3.1 Specifier-call validation (landed)

At the class-body call (T1 house style — plain `ValueError`, like
`tick(expect_hz=0)`):

- each frame template must be a non-empty `str`;
- placeholders must be plain config-field names: `{prefix}` — no positional
  `{}`, no `{a.b}`, `{a[0]}`, `{a!r}`, `{a:>3}` (parsed with
  `string.Formatter`, so `{{` escapes work);
- a template with NO placeholders must normalize non-empty (`"///"` dies
  now, not at build);
- identical parent and child templates die now (identical templates always
  resolve identical → guaranteed self-edge);
- `tf_out(min_hz=)` must be > 0 when given.

Side-ness comes free from the T1 machinery: `tf()` on an Out bundle /
`tf_out()` on an In bundle raise the standard "not valid on an
{Side} bundle field" `BundleDefinitionError`.

**Reserved name (D4):** an In-bundle field named `tf` is rejected
unconditionally at bundle definition — `tf` is the stream key
(`over(tf=...)`, `m.i.tf`) that feeds tf() samplers. Out-side `tf` stays
legal (no routing surface conflicts; the legacy bridge separately bans it on
its own surface). Copy: `In field name 'tf' is reserved — it is the tf
stream key (over(tf=...), m.i.tf) that feeds tf() samplers; rename the
field`.

### 3.2 Build resolution (landed)

Frame names resolve **at build** = module instantiation, when config is
known — never at import (sketch §5b: five `Go2Connection(prefix=...)`
instances are five distinct edges).

Mechanics (in `module.py`):

- `__init_subclass__` collects declared tf specs from BOTH bundles' merged
  raw spec tables (`rows._merged_specs` — no annotation resolution, so
  TYPE_CHECKING-imported field types stay legal until wiring) into
  `cls.__pure_tf_templates__: dict[(side, field), spec]`, stamped just
  before `__pure_step__` (which stays LAST — its presence still certifies
  every gate passed).
- `PureModule.__init__`, after config flattening: if templates exist,
  resolve each against `{name: getattr(self, name)}` for every config field
  and store `self.__pure_tf_frames__: dict[(side, field), (parent, child)]`
  via `object.__setattr__`. A tf-free module never touches any of this.

Resolution rules (`format_frame`):

- placeholders substitute config values: `str`/`int`/`float`/`bool` via
  `str()`; `None` resolves empty (its segment drops with the separator —
  `"{prefix}/odom"` with `prefix=None` or `prefix=""` gives `"odom"`); any
  other value type is an error;
- then `normalize_frame`: split on `/`, drop empty segments, rejoin —
  `"go2a//odom"` → `"go2a/odom"`, `"/odom"` → `"odom"`.

Build-time checks, in field order (In fields first, then Out — declaration
order within each; deterministic error precedence), raising
`PureModuleDefinitionError` with release copy per §10.1:

1. unknown placeholder → `[tf-template-unknown]` (names module, field,
   template, and the actual config fields — charter requirement);
2. unusable placeholder value → `[tf-template-value]`;
3. resolved frame empty → `[tf-frame-empty]`;
4. resolved parent == child → `[tf-self-edge]`;
5. two `tf_out` fields of one module resolving to the same edge — reversed
   orientation included (`frozenset({parent, child})` keying) →
   `[tf-duplicate-edge]`. This is the **module-scope** single-writer check;
   engine scope is §4.4.

Two `tf()` (consumer) fields sampling the same edge are legal — pointless
but harmless.

### 3.3 `port.frames` introspection (landed)

`m.i.world_to_base.frames` / `m.o.tf_odom_base.frames` return the
build-resolved `(parent, child)` tuple, working from construction (no
warmup needed — the sketch's `_five_robots_one_graph` assertions hold).
Non-tf ports keep raising `NotImplementedError` (the T4 §5.5 contract,
message now "not a tf()/tf_out() port"). `typing.InPort` gained the same
read-only `frames` property stub `OutPort` already had.

### 3.4 The sparse spelling: `default=None` is mandatory (D3)

Verified on this branch: mypy derives dataclass-field required-ness ONLY
from the presence of a `default=` argument **at the specifier call site** —
a defaulted `default` parameter in the specifier's own signature is ignored,
in both plain and overloaded spellings. Therefore
`tf_world_map: Transform | None = tf_out("world", "map")` would be
statically REQUIRED at row construction while runtime made it optional — an
unacceptable static/runtime split. The sparse spelling is:

```python
tf_world_map: Transform | None = tf_out("world", "map", default=None)
```

exactly parallel to `latest(default=None)` / `contract(min_hz=1,
default=None)`. `tf_out("a", "b")` without a default is a REQUIRED Out
field — a module that must assert its edge on every emitted row. The sketch
writes the bare form in §5/§5b; flagged as an amendment in §18.2. `tf()`
follows the same rule and the sketch already agrees (`default=None` in the
§NOTE example).

### 3.5 Payload-type doctrine (D15)

The declared annotation of a tf field is not validated at runtime (T1 D3 /
T5 D15 consistency): the engine always delivers a `Transform`; mypy holds
users to annotating `Transform` / `Transform | None`. Unlike
`interpolate()`, tf dispatches on nothing — interpolation is internal to the
buffer (§5.1) — so no `[align-interp-untyped]`-style wiring check exists or
is needed.

## 4. The buffer — `TfBuffer`

One engine-owned accumulator of per-edge transform history. It belongs to
the RUN/engine context and is **passed in — never a process global** (tests
and multi-engine processes stay isolated; the charter is explicit).

### 4.1 Storage model

- `edges: dict[EdgeKey, _EdgeHistory]` — keyed by the DIRECTED `(frame_id,
  child_frame_id)` pair as ingested (like `MultiTBuffer.buffers`); reverse
  lookups invert at resolution, they do not create reverse entries.
- `_EdgeHistory`: a ts-sorted sequence of `Transform` samples (list +
  `bisect` recommended; strictly increasing ts by construction) OR a single
  static value (§4.3). One edge is either dynamic or static, never both
  (§4.4 claims make the conflict loud).
- memory bound: O(#edges × horizon × rate) — the window (§4.2) is the bound.

### 4.2 Ingestion — sorted insert per edge (D6)

`ingest(item)` accepts a `Transform`, or a TFMessage-duck (anything with a
`.transforms` iterable — `receive_tfmessage` adapted); each transform needs
readable `frame_id`/`child_frame_id` and a finite float ts, else
`[tf-bad-item]` (UNSTAMPED `-inf` included — there is no legitimate
unstamped transform on this path).

Per edge, in ingestion order:

- ts already present on the edge → dropped, `dropped_duplicate += 1`
  (**first-in wins** — the deterministic dedupe §7.4 leans on);
- ts older than `newest_kept_ts - horizon` → dropped, `dropped_expired += 1`
  (it would land outside the live window);
- otherwise **sorted insert** (`accepted += 1`), then evict from the front
  while `oldest.ts < newest.ts - horizon` (`evicted += n`). The newest
  sample per edge survives by construction — a slow (0.2 Hz) edge always
  keeps at least its latest sample, so `L` never vanishes for a live edge.

Sorted insert — NOT T5's strict per-port monotonic frontier filter — is
deliberate and load-bearing: the tf stream multiplexes many edges from many
writers, and the run's own `tf_out` assertions (§7.3) are ingested at tick
time, potentially BEHIND stream samples the aligner already pulled while
advancing past the tick (`advance_past` reads ahead by design). A frontier
filter would silently discard them; the adapted `TBuffer` (InMemoryStore
sorted `save`) always tolerated out-of-order inserts. The T5 frontier
doctrine is untouched — it governs merge ports; the buffer is a **window**,
not a frontier (no T5 rule bends; see §16 D6).

Retention knob: `horizon` (data-time seconds, default `DEFAULT_TF_HORIZON =
10.0` — `TFConfig.buffer_size` parity). It rides the buffer object (D16):
`TfBuffer(horizon=...)` — no knob on `align()`/`over()` (T5 §15.1's rule:
retention lives with the windowed sampler that needs it — this is that
sampler).

Eviction consequences, stated honestly: a tick can need data older than an
edge's window only while a long hold (§6.3) lets OTHER edges advance the
buffer far past the held T. If a hold outlasts `horizon` in data time, a
bracket near T may be evicted; resolution then finds no `L` and, at stream
exhaustion, the tick drops (final, counted). Window sizing bounds hold
utility; T9 sees both the hold (`held_tick_ts`) and the drop.

### 4.3 Statics (D17)

`set_static(t)` / `TfBuffer(statics=[...])`: one value per edge, valid at
EVERY ts, never evicted, `stats.statics` counted. This is the pure
`publish_static`: extrinsics/URDF edges are **config, seeded at warmup** —
concretely, whoever owns the run context builds the buffer from deployment
config (`TfBuffer(statics=load_extrinsics(cfg))`) and hands it in
(`over(tf=...)` §8; `attach()` §9.2). Statics register a writer claim under
`STATIC_OWNER` (§4.4), so a static edge colliding with a `tf_out` edge or a
second static is loud. A static edge resolves at any `at` to its value
re-stamped `ts=at` (frames preserved).

### 4.4 Claims — engine-scope single-writer + forest (D12)

The buffer carries the writer registry:

- `claim(edge, owner)`: registers `owner` (a module class path, or
  `STATIC_OWNER`) as the writer of `edge`. Conflicts key on the UNDIRECTED
  pair — `("world","map")` vs `("map","world")` is the same edge — and
  raise `[tf-multi-writer]` naming both parties. Re-claiming an edge you
  already own is idempotent (session restart).
- forest check: before accepting, walk connectivity over **claimed + static
  edges only**; if the new edge's endpoints are already connected, raise
  `[tf-frame-cycle]` — the asserted graph must stay a forest, so exactly
  one asserted chain connects any two frames (the classic tf-tree
  ambiguity, caught at claim time, per-buffer scope).
- `release(owner)`: drops all of `owner`'s claims (idempotent); run/session
  teardown calls it, so stop → rebind → start works.

Scope honesty: claims validate the edges DECLARED into this buffer
(this process's modules + its statics). Edges merely *ingested* from the tf
topic are foreign observations — a remote writer cannot be validated here;
whole-graph single-writer/cycle checking across processes belongs to
wire()/plan (out of scope, Q5, sketch: "plan checks run on the wired
instance graph").

### 4.5 Thread-safety (D14)

All buffer methods take one internal plain `threading.Lock` (never a
Condition, never a wait — the `_wait_get` ghost stays dead; holds live in
the aligner's pull). Under `over()` the lock is uncontended
single-thread; live, it serializes sibling sessions sharing one buffer
(§9.3). Stats reads are lock-free snapshots of monotonic ints (house rule).

### 4.6 Stats — the T9 seam

`TfStats { edges: dict[EdgeKey, TfEdgeStats], statics: int }` with
`TfEdgeStats { accepted, dropped_duplicate, dropped_expired, evicted }`.
Plus, aligner-side (§6.5): the `"tf"` entry in `AlignStats.ports` counts
stream items pulled (accepted = ingest-accepted; the buffer's per-edge
detail refines it), and per-field drops land in the existing
`drops_by_field`. A held tf tick is visible through the existing
`held_tick_ts` gauge. No new hook machinery.

## 5. Resolution semantics (normative)

`resolve(parent, child, at)` returns the Transform mapping child-frame
coordinates into the parent frame at data-time `at`, or `None`.

### 5.1 Edge value at `at`

For a directed edge `e` with dynamic samples `S_e` (ts-sorted), define
`L` = last sample with `ts <= at`, `R` = first with `ts >= at`:

| Case | Value |
| --- | --- |
| static edge | the static value, re-stamped `ts=at` |
| `L.ts == at` (exact hit) | `L`, the sample object itself (interpolator NOT called) |
| `L` and `R` exist, `L.ts < at < R.ts` | `interp_transform(L, R, alpha)`, `alpha = (at - L.ts)/(R.ts - L.ts)` |
| one-sided or empty | unresolvable |

**No extrapolation, ever** (T5 D8 verbatim): one-sided data never resolves;
`alpha` is strictly inside (0, 1) with a non-degenerate bracket (distinct
sorted ts guarantee it). Interpolation REUSES
`dimos.pure.interpolators.interp_transform` — position lerp + shortest-arc
slerp, ts lerped to `at`, same-frame enforcement — imported directly (D7):
no second slerp is written, no registry indirection, no `install()`
prerequisite (the T5 registry keeps dispatching user `interpolate()` fields;
tf's interpolation is buffer-internal and dispatches on nothing).

The reversed edge `(child, parent)` resolves as the forward value's
`.inverse()` (value computed at `at` first, then inverted).

### 5.2 Chain search (adapted BFS)

When no direct or reversed edge resolves, search — `get_transform_search`
adapted:

- BFS from `parent` toward `child` over the undirected adjacency of ALL
  edges present in the buffer (dynamic or static);
- a hop `u → v` is traversable iff its edge value **resolves at `at`**
  (forward entry, or reversed entry inverted) — resolvability prunes the
  graph before pathing, exactly like the original's per-hop
  `get_transform` check, so a found path is composable by construction;
- neighbor iteration order = edge first-creation order in the buffer
  (dict insertion order) — the deterministic tie-break for equal-length
  paths; BFS shortest-path itself bounds chain length;
- no path → `None`.

Foreign ingested edges may create alternate paths the claims forest never
saw (§4.4); shortest-wins + deterministic tie-break keeps replay identity.

### 5.3 Composition convention (+ worked example)

`Transform(frame_id=A, child_frame_id=B)` = `T_A_B`, mapping points in B
to points in A: `p_A = R·p_B + t`. Chains compose left-to-right with
`Transform.__add__`:

```
T_A_C = T_A_B + T_B_C        # frame_id=A, child_frame_id=C
```

which is the original's `reduce(lambda t1, t2: t1 + t2, chain)`. The
composed result is re-stamped `ts=at`, `frame_id=parent`,
`child_frame_id=child`.

Worked example (pinned in `test_chain_composition_worked_example`): with

- `T_world_odom`: rotation Rz(+90°) (quaternion `(0, 0, sin45°, cos45°)`),
  translation `(10, 0, 0)`;
- `T_odom_base`: identity rotation, translation `(1, 0, 0)`;

then `tf("world", "base")` resolves
`T_world_base = T_world_odom + T_odom_base` with translation
`(10, 0, 0) + Rz90·(1, 0, 0) = (10, 1, 0)` — the classic sign-error trap:
the CHILD-side translation is rotated by the PARENT-side rotation before
adding. The reverse request `tf("base", "world")` returns the inverse:
rotation Rz(−90°), translation `−Rz(−90°)·(10, 1, 0) = (−1, 10, 0)`.

### 5.4 Identity fast path (D10)

`resolve(f, f, at)` returns an identity Transform stamped `ts=at` (the
original's `parent==child` branch) — declarations reject self-edges (§3.2),
but a chain endpoint pair collapsing to the same frame after bind-time
rewriting must not crash a run.

## 6. Align integration — the sampler

### 6.1 Wiring shape (D5)

tf fields do NOT become `_PortState` merge ports — a multiplexed tf stream
breaks the one-port-one-sequence model (T5 §11.5 anticipated exactly this).
The tf machinery is a **demand-pulled side channel**:

- `align(in_type, streams, *, tf: TfContext | None = None)` — new
  keyword-only parameter (TYPE_CHECKING import; the aligner uses the
  context structurally). `Aligner.__init__` partitions fields: tick /
  latest / interpolate / **tf** (extend `_SUPPORTED_KINDS` with `TfSpec`;
  the `[align-unsupported-kind]` seam keeps rejecting genuinely unknown
  kinds).
- Wiring checks at construction (eager, T5 §7 style):
  - tf fields present and `tf is None` and any tf field is required →
    `[tf-missing-stream]` (all-optional bundles may run contextless: every
    tick defaults);
  - no tf fields → the caller never passes a context (run_over guarantees;
    assert).
- A `streams` key `"tf"` remains `[align-unknown-port]` (no In field can be
  named `tf`, D4); the error's teaching note extends: when the key is
  `"tf"`, append ` (tf is the reserved tf-stream key — pass it as the tf=
  argument, not a stream.)` — mirroring the existing ts note.
- `self._stats.ports["tf"] = ctx.port_stats` so T9 sees stream-level
  counters in the familiar place.

### 6.2 `advance_past` — the frontier rule

At tick fire time (pull-loop step (e), before `_fire`): if a context
exists, `ctx.advance_past(T)` — pull the tf stream, ingesting EVERY item
into the buffer, until an item with `ts > T` has been ingested (running-max
frontier — the multiplexed stream is only near-sorted, so the stop
condition is the max seen, not the last item) or the stream is exhausted.
This runs for every fired tick, resolvable or not, so buffer evolution is a
deterministic function of the data (§11). Items pulled beyond `T` are
harmless by construction: they land in the window and serve as right
brackets for this and later ticks.

Data errors (`[tf-bad-item]`) raise from the pulling `__next__`, T5 §9
placement.

### 6.3 Hold vs final drop — required fields (D8)

`_fire` pass 1 gains, after the existing latest/interpolate decisions
(cheap checks first — if a required non-tf field is already missing the
tick drops without tf pulling):

```
unresolved = {name for name in tf_required if ctx.resolve_field(name, T) is None}
while unresolved and ctx.pull():          # THE HOLD — pulling is the wait
    unresolved = {n for n in unresolved if ctx.resolve_field(n, T) is None}
if unresolved: -> missing (drop path: ticks_dropped, drops_by_field[name])
```

- **Held**: while the chain is unresolvable and the stream is alive, the
  aligner pulls tf forward. Under `over()` this is instantaneous iteration;
  behind the rim's queue-backed feed it parks the session thread on data
  that hasn't arrived — the operational hold, T5 §4.5 verbatim, replacing
  `_wait_get`'s blocked thread. No wall clock, no timeout, no revisit
  queue.
- **Final**: the only finality proof is stream exhaustion (unlike T5
  secondaries, future tf data can create brand-new edges and paths — the
  charter's "multi-edge chain with one stale edge" holds until that edge
  produces its bracket or the stream ends). Offline, exhaustion makes every
  remaining tick's resolution final (drop, counted per field). Live, a
  silent edge holds the head tick indefinitely — the T5 corollary,
  verbatim: correct pure semantics; dead-sensor detection is T9's
  (`held_tick_ts` shows T), cutoff policy is T8's.
- Head-of-line: emission is ts-ordered by design, so a held tick blocks
  later ticks — accepted, same as T5.

Note the asymmetry with §6.2: `advance_past` stops at frontier > T; the
required-hold keeps pulling **beyond** the frontier until the specific
chains resolve. Both are pure functions of the data.

### 6.4 Optional fields

`default=` tf fields NEVER hold (charter: `default=None` makes it
optional). They resolve at fire time — after `advance_past(T)` and any
required-field holds — and take their default when unresolvable. A
consequence worth stating: an optional field whose right bracket would have
arrived two items later defaults instead; that is what optional means, and
it is deterministic.

### 6.5 Row construction and recording

Resolved tf values are ordinary row fields: `in_type(ts=T, ..., name=value)`
— plain data, hand-constructible in tests, serialized when rows are
recorded. Every interpolated tf value appears in the emitted row, so edges
stay recordable (index preclusion guard); replaying a module does NOT need
the tf machinery to reproduce its inputs — the recorded rows already carry
them.

### 6.6 Exact insertion points (`align.py`)

1. imports: `TfSpec` joins the `rows` import; `TfContext` under
   TYPE_CHECKING from `dimos.pure.tfbuffer`.
2. `_SUPPORTED_KINDS` += `TfSpec`; tf fields skip `_PortState` creation —
   collect `self._tf_fields: list[tuple[str, TfSpec]]` in declaration
   order; validate per §6.1 (lazy-import the error helpers).
3. `__next__` step (e): `ctx.advance_past(tick.head_ts)` before `_fire`.
4. `_fire`: pass-1 tf block per §6.3/§6.4; resolved values join `resolved`
   for row construction (pass 2 unchanged — tf materializes in pass 1
   because resolution IS the decision; the D12 two-pass rule protects
   *user* interpolators, and tf's are engine-internal).
5. stats registration per §6.1.

## 7. `tf_out` routing

### 7.1 The tap

`tf_out_tap(rows, module, ctx)` is a generator wrapping the DRIVER's output
iterator (after T6 stamping — rows arrive engine-stamped). Per row, for
each `tf_out` field in Out declaration order:

- `None` → silent (sparse doctrine, nothing routed);
- validate: payload is a `Transform` → else `[tf-out-bad-payload]`; finite
  ts → else `[tf-out-unstamped]` (construct with `ts=i.ts` — data time,
  never `.now()`); `(frame_id, child_frame_id)` equals the field's
  build-resolved edge → else `[tf-out-frame-mismatch]` (the "payload
  validates against the declaration on emission" charter line — the msg-ts
  check's twin);
- `ctx.buffer.ingest(payload)`;
- yield the row through unchanged; `finally: ctx.release()`.

Generator chaining guarantees ordering: a row's assertions are ingested
before the NEXT tick resolves.

`run_over` applies the tap when `spec.out_type` declares any `TfOutSpec`
(§8); the rim wraps its `_dispatch` result identically (§9) — one
implementation, two callers.

### 7.2 Claims at run start

`TfContext.for_over`/`for_live` claim every declared `tf_out` edge
(`owner` = the module's class path) at construction — conflicts and cycles
surface at `over()` call / `start()`, on the caller thread, before data
moves. `release()` runs in the tap teardown (over) / session stop (live).

### 7.3 Three routes + self-ingestion (D11)

The engine routes an assertion three ways, module oblivious (sketch §5):

| Route | over() | live |
| --- | --- | --- |
| (a) run/engine buffer — sibling (and own) samplers | YES — the tap | YES — the tap |
| (b) recorder — the emitted Out row carries the field | YES (rows ARE the recording currency) | YES |
| (c) the tf topic | structurally impossible (over is rim-less) | only where `m.o.<field>.transport` is bound |

Route (a) runs under `over()` too — deliberately. Buffer ingestion is
engine-internal sampler state, not an effect; the EFFECT half (topic
publish) stays inert offline, exactly like every Out field. This keeps
single-module semantics identical across over() and live: a module
consuming a chain through its own asserted edge behaves the same way in
both (its own assertions can form left brackets for later ticks — pinned in
`test_self_ingestion_brackets_with_stream` — and, since assertion ts never
exceeds the current tick ts, can never deadlock a hold: the hold loop pulls
the STREAM, and stream exhaustion is still final).

### 7.4 Replay dedupe (worked micro-example)

Replaying module M with `tf=` a recorded topic that includes M's own past
assertions: at tick T, `advance_past(T)` ingests the RECORDED assertion
(ts = T′ ≤ T arrives with the stream); M's re-computed assertion for T′
was already ingested by the tap at T′ — same edge, same ts — so the
recorded copy drops as `dropped_duplicate` (first-in wins, §4.2), or vice
versa depending on data order; either way exactly one sample per (edge, ts)
survives, deterministically. Recommended replay wiring: exclude M's own
asserted edges from the fed tf stream — they are M's outputs, not inputs —
then fresh assertions are the only writer and re-evaluation (changed code →
changed assertions) flows through. The flight-recorder tooling owns that
filter, not T11.

## 8. `over()` / `run_over` integration

### 8.1 Surface (typed, not smuggled)

`tf` is a reserved keyword-only parameter of `over()`, NOT a `**streams`
key — honest static typing (`TfSource | None`, TYPE_CHECKING import in
`typing.py`) instead of pretending a `TfBuffer` is `Iterable[Stamped]`:

```python
def over(self, *, tf: TfSource | None = None, **streams: Streamable) -> Iterator[TOut]
```

(all four overloads; `run_over(module, spec, streams, *, tf=None,
hooks=None)` mirrors it). Call sites read exactly as the sketch promised:
`m.over(image=..., tf=store.streams.tf)`.

### 8.2 Normalization (`TfContext.for_over`)

| `tf=` value | Meaning |
| --- | --- |
| `None` | no stream; context exists iff the module declares tf fields (fresh private `TfBuffer()`; required tf fields → `[tf-missing-stream]` unless every tf field has a default) |
| `Iterable` | stream into a fresh private `TfBuffer()`; memory2 `Stream` objects pass through T6's `_coerce_stream` Observation unwrap like any stream value |
| `TfBuffer` | statics/shared buffer, no stream (statics-only runs are legal — pinned) |
| `(TfBuffer, Iterable)` | both |
| `tf=` given but no tf/tf_out fields declared | `[tf-unexpected-stream]` |
| anything else | `[tf-bad-source]` |

A module with `tf_out` fields but no `tf()` fields still gets a context
(claims + tap need the buffer); one with neither gets `None` and pays
nothing.

### 8.3 Exact insertion points (`drivers.py`)

1. `run_over` signature gains `tf: Any | None = None` (runtime-untyped like
   `module`; static type lives on `over()`).
2. after `_check_unknown_streams` (unchanged — `"tf"` in `**streams` cannot
   name a port and errors there): lazy-import `TfContext` iff `tf is not
   None` or the bundles declare tf fields; `ctx = TfContext.for_over(module,
   spec, tf)`.
3. `rows = align(spec.in_type, coerced_streams, tf=ctx)`.
4. dispatch as today; if `ctx` and Out has `TfOutSpec` fields, wrap:
   `return tf_out_tap(driver_iter, module, ctx)`; if `ctx` and no tf_out
   fields, still ensure `ctx.release()` on teardown (a finally-wrapper or a
   no-out tap variant — implementer's choice, release exactly once).
5. teardown ordering: tap release runs on generator close, after the
   driver's own `_finalized` chain — claims outlive the last row, die
   before `over()` returns.

## 9. Live rim integration

### 9.1 `m.i.tf` — the synthetic stream port

Present iff the In bundle declares `tf()` fields (name collision impossible
— D4). `RimInPorts.__getattr__` special-cases `"tf"`: a cached `RimInPort`
carrying a module-level sentinel `TfSpec` (marker; not a bundle field), with
the full binding surface — `.transport =` / `.source =` (exclusive),
`.capacity` (ingress ring, default `DEFAULT_CAPACITY`), live-rebind guard.
Unwired is LEGAL (D18): statics, an attached shared buffer, or sibling
assertions may be the only sources — required-field starvation then shows
as a held tick, T9's business, not a wiring error (contrast T5's
`[align-missing-tick-stream]`: the trigger drives rows; tf merely feeds
them).

### 9.2 Buffer injection — `tfbuffer.attach` (D18)

`attach(module, buffer)` stores the buffer in the module's rim state
(`ports_of`), guarded by the live-rebind rule; the next session uses it.
Default (no attach): the session builds a private `TfBuffer()` at start.
One spelling per capability: `attach()` is THE injection point live (works
for consumers and tf_out-only publishers alike); `over(tf=...)` is THE
injection point offline — different run contexts, symmetric with
streams-vs-transports.

In-process sibling sharing = the host attaches ONE buffer to several
modules: A's session tap ingests A's assertions; B's sampler resolves
against them with no wire round-trip (the sketch's route (a)). B may run
with `m.i.tf` unwired in this topology.

### 9.3 Session lifecycle (`rim.py` insertion points)

1. `_LiveSession.start`, after feed construction: if the module declares tf
   or tf_out fields, lazy-import tfbuffer; `buffer` = attached or fresh;
   if `m.i.tf` is wired, build its `_PortQueue`/`_LiveFeed` exactly like a
   declared port (capacity from the port handle; transport `subscribe(
   queue.enqueue)`, iterable sources pumped — reuse `_bind_ingress`);
   `ctx = TfContext.for_live(module, spec, buffer, feed_or_None)` — claims
   raise here, on the caller thread, G3-unwound like any start failure.
2. `self._aligner = align(in_type, feeds, tf=ctx)`;
   `self._driver = tf_out_tap(self._dispatch(...), module, ctx)` when Out
   has tf_out fields.
3. stop: `ctx.release()` after drain (step 5 of the §7.3 table — with the
   driver finalized; the tap's own finally covers the crash path).
4. TFMessage-speaking transports need no adapter on ingestion (`ingest`
   unwraps `.transforms` ducks). Egress to a shared `/tf` topic: bind
   `m.o.<field>.transport` to any `Publishable` accepting a `Transform` —
   `PubSubTF.publish(*args: Transform)` satisfies it structurally today;
   a dedicated thin topic adapter is deferred (Q2).
5. `warmup` unchanged (the dry-check never touches tf).

### 9.4 Live determinism honesty

Shared-buffer visibility across sessions is arrival-order — live mode is
not deterministic and never was (T8 doctrine); determinism claims (§11)
attach to `over()`. Replay parity for a live graph comes from recording the
tf edges (rows and/or topic) and feeding them back per §7.4.

### 9.5 Static typing (`typing.py`)

Landed: `InPort.frames` stub. Pending with the impl: the `over()` overloads
gain `tf: TfSource | None = None` (TYPE_CHECKING import), and the T4 static
fixture set gains a case pinning `m.i.world_to_base.frames` /
`over(tf=...)` acceptance + `over(tf=3)` rejection.

## 10. Error catalog

### 10.1 Build-time — `PureModuleDefinitionError` (landed; verbatim-tested)

`{cls}` = `f"{cls.__module__}.{cls.__qualname__}"`. Raised at instance
construction (templates resolve at build). Specifier-call misuse raises
plain `ValueError` (T1 house style, §3.1); bundle-shape misuse raises T1's
`BundleDefinitionError` (side rules, reserved `tf` name).

| Slug | When | Message template |
| --- | --- | --- |
| `[tf-template-unknown]` | placeholder names no config field | `{cls}: tf field {field!r} template {template!r} names {key!r}, which is not a config field — frame templates resolve against the module's own config fields ({names}). [tf-template-unknown]` |
| `[tf-template-value]` | placeholder value unusable | `{cls}: tf field {field!r} template {template!r}: config field {key!r} value {value!r} is not usable in a frame name — use str, int, float, bool, or None (None resolves empty and its segment drops). [tf-template-value]` |
| `[tf-frame-empty]` | resolved frame empty | `{cls}: tf field {field!r} frame template {template!r} resolved to an empty frame name — after empty segments drop, at least one segment must remain. [tf-frame-empty]` |
| `[tf-self-edge]` | resolved parent == child | `{cls}: tf field {field!r} resolved to identical parent and child frames ({frame!r}) — a tf edge relates two distinct frames. [tf-self-edge]` |
| `[tf-duplicate-edge]` | two tf_out fields, one edge | `{cls}: tf_out fields {a!r} and {b!r} both assert the edge between {p!r} and {c!r} — one writer per edge (reversed orientation included); merge them or rename a frame. [tf-duplicate-edge]` |

### 10.2 Runtime — `TfError(AlignmentError)` with `tf_rule: TfRule` (D13)

tf is a sampler: its wiring/ingestion errors are alignment-family (T5's
`AlignmentError(TypeError)`), one class, T3-style slugs, message helpers
landed in `tfbuffer.py` (release copy — verbatim-tested once raised).
Wiring errors (W) raise at `over()`/`start()`; data errors (D) raise from
the pulling `__next__` / the tap.

| Slug | When | Message template |
| --- | --- | --- |
| `[tf-unexpected-stream]` (W) | `tf=` but no tf fields | `{cls}.over() got tf=<...> but its In bundle declares no tf() fields — the tf stream feeds tf() samplers only. Remove tf=, or declare a tf() field. [tf-unexpected-stream]` |
| `[tf-missing-stream]` (W) | required tf fields, no source | `{cls}: In declares required tf() field(s) {names} but the run got no tf source — pass tf=<stream or TfBuffer> (over) / bind m.i.tf or attach a buffer (live), or give the fields default= to make them optional. [tf-missing-stream]` |
| `[tf-bad-source]` (W) | `tf=` value wrong type | `{cls}: tf= must be a TfBuffer, an iterable of stamped transforms, or a (TfBuffer, iterable) pair, got {type}. [tf-bad-source]` |
| `[tf-multi-writer]` (W) | claim conflict | `tf edge {p!r} -> {c!r} is already asserted by {owner!r}; {claimant!r} cannot claim it — one writer per edge (reversed orientation included). [tf-multi-writer]` |
| `[tf-frame-cycle]` (W) | claim closes a cycle | `tf edge {p!r} -> {c!r} claimed by {claimant!r} would close a cycle in the asserted tf graph — asserted edges (tf_out declarations plus statics) must form a forest: exactly one asserted chain may connect any two frames. [tf-frame-cycle]` |
| `[tf-bad-item]` (D) | stream item malformed | `tf stream yielded {type} with no readable frame pair and finite ts — the tf stream carries Transform msgs (or TFMessage batches of them). [tf-bad-item]` |
| `[tf-out-bad-payload]` (D) | tf_out value not a Transform | `{cls}.{field}: tf_out payload is {type}, not a Transform — a tf_out field carries its asserted edge's Transform (or None to stay silent). [tf-out-bad-payload]` |
| `[tf-out-unstamped]` (D) | tf_out ts not finite | `{cls}.{field}: tf_out Transform has ts {value!r} — assertions are timeline samples; construct with ts=i.ts (data time, never wall clock). [tf-out-unstamped]` |
| `[tf-out-frame-mismatch]` (D) | payload frames ≠ declaration | `{cls}.{field}: tf_out Transform carries {p!r} -> {c!r} but the declared edge is {dp!r} -> {dc!r} — the payload's frames must match the declaration. [tf-out-frame-mismatch]` |

Not errors: unresolvable required tf at stream exhaustion (counted drop,
§6.3); duplicate/expired ingestion (counters, §4.2); unwired `m.i.tf`
(§9.1).

## 11. Determinism (normative property)

Under `over()`: the emitted rows, the final `AlignStats` (including the
`"tf"` port entry and `drops_by_field`), and the buffer's stats are a pure
function of `(module class + config, the logical per-stream item sequences,
the tf item sequence, the buffer's statics)`. Not consulted: wall clock
(assertions and lookups are data-time only — the no-wall-clock doctrine),
thread timing, chunking, mapping order. The pull pattern (advance +
hold) is a function of the data; buffer evolution (sorted inserts, evictions,
dedupes) is a function of ingestion order, which is a function of the pull
pattern. Same inputs → identical rows and stats (pinned:
`test_determinism_chunking`). Live mode: best-effort, §9.4.

## 12. Boundaries (referenced, not designed)

### 12.1 T9 health

Consumes: `ports["tf"]` + `drops_by_field` (existing shapes), `held_tick_ts`
("held on tick T" — now also covering tf holds), `TfBuffer.stats` for
per-edge forensics, `TfOutSpec.min_hz` as contract cadence on the asserting
port (contract machinery composes; T9 measures emission rate like any
contract field). No new hook surface.

### 12.2 T10 checkpoint

The buffer is NOT checkpoint state: `checkpoint()` stays `{State, cursor,
config}`. Resume re-warms the buffer by replaying the recorded tf edge from
`cursor − horizon` (the sketch: "re-warm needs only the retention horizon,
not t0"). Statics re-seed from config, claims re-register at start — all
reconstruction, no snapshot.

### 12.3 T12 examples

Port sketch §5 `RelocalizationModule` + §5b `Go2Connection` as executable
examples once the impl lands (mind §18.2's `default=None`); the Tagger
floor is untouched by construction.

### 12.4 Legacy bridge (T8b)

Nothing in `legacy.py` changes for T11 correctness: `tf_out` fields bridge
as ordinary `Out[Transform]` autoconnect topics (Optional stripped), rows
carry the values. `ModuleConfig.frame_id_prefix` (parity row P4 "consumed
by T11 later") and a `/tf`-topic adapter for mixed legacy/pure graphs are
deferred (Q2) — `format_frame`/`normalize_frame` are the naming functions a
bind-time `tf_prefix` rewrite will reuse (sketch: "the same pure naming
function as topics").

### 12.5 `tf_view()` — deferred (Q1)

The frame-agnostic snapshot escape hatch (charter: OPTIONAL, defer unless
trivial) is deferred: it needs a snapshot/immutability story and a
recording-cursor contract that earn their keep only with a real consumer
(viz). `TfBuffer.resolve` is the substrate it would wrap; nothing here
precludes it.

## 13. Adapted from `dimos/protocol/tf/tf.py` — the mapping

Function-by-function disposition of the old implementation. ADAPTED = the
math/shape survives in the named new home; DELETED = gone with rationale.

| Old | Disposition |
| --- | --- |
| `TFLookup` protocol | DELETED — service read-side; pure consumers declare `tf()` fields instead. (`StreamTF`/replay backends feed `tf=` streams now.) |
| `TFConfig.buffer_size = 10.0` | ADAPTED → `DEFAULT_TF_HORIZON = 10.0` (§4.2) |
| `TFConfig.rate_limit` | DELETED — publisher-side throttling; cadence is contract machinery (`min_hz`) + module cadence |
| `TFSpec` (Service) | DELETED — the service API wholesale; lifecycle is the rim's |
| `TFSpec.publish` | REPLACED by `tf_out` fields + the tap (§7) |
| `TFSpec.publish_static` (never implemented) | REPLACED by `TfBuffer(statics=)` / `set_static` (§4.3) — the pure publish_static, actually implemented |
| `TFSpec.get_pose` | DELETED — call `.to_pose()` on the resolved Transform |
| `receive_transform` / `receive_tfmessage` | ADAPTED → `TfBuffer.ingest` (TFMessage duck-unwrap kept; Condition/notify dropped) |
| `TBuffer(InMemoryStore)` sorted `save` | ADAPTED → `_EdgeHistory` sorted insert (§4.2) — out-of-order tolerance kept, duplicate-ts first-wins added |
| `TBuffer.add` + `prune_old(ts − buffer_size)` | ADAPTED → ingest + per-edge window eviction relative to the edge's own newest (§4.2) — the always-keep-newest property preserved |
| `TBuffer.get(time_point, tolerance)` = `find_closest` | REPLACED — nearest-within-tolerance semantics upgraded to exact-hit / bracketed interpolation / no extrapolation (§5.1); `time_tolerance` dies with it |
| `MultiTBuffer.buffers` dict keyed `(parent, child)` | ADAPTED → `TfBuffer.edges` (§4.1) |
| `MultiTBuffer._cv` Condition + `notify_all` | DELETED — thread-free pull; buffer gets a plain Lock only for live sharing (§4.5) |
| `get_frames` / `get_connections` | ADAPTED → `frames()` / private adjacency for BFS (§5.2) |
| `get_transform` (direct + reverse-inverse + identity) | ADAPTED → edge-value + reversed-inverse + identity fast path (§5.1, §5.4) |
| `_get` (direct → search → `reduce(t1 + t2)`) | ADAPTED → `resolve` (§5.2–5.3), same composition operator |
| `_wait_get` (Condition wait + `forward_tolerance` deadline) | **DELETED** — hold-the-tick replaces it (§6.3); the only wall clock in the old path dies here |
| `get(..., forward_tolerance=)` + warning log | DELETED — no timeout parameter exists; unresolvable is hold/drop, counted not logged |
| `get_transform_search` (BFS, per-hop resolvability) | ADAPTED → §5.2 with deterministic neighbor order pinned |
| `graph()` (diagon), `__str__` | DELETED — debug niceties; inspector-era tooling reads `edges()`/`stats` |
| `PubSubTF` / `LCMTF` / `ZenohTF` + configs | DELETED — transports bind at rim ports; `PubSubTF` interop survives structurally (§9.3.4) |

## 14. Test plan

`dimos/pure/test_tf.py` ships with this spec. Live now (30 passing):

| Test | Pins |
| --- | --- |
| `TestSpecifiers` (6) | spec shapes, required/default, min_hz>0, template misuse at call, identical-template rejection |
| `TestFrameTemplates` (4) | normalize/format semantics: empty-segment drop, None-drops, int values, unknown → KeyError, bad value → ValueError |
| `TestBundleRules` (4) | side rules via T1 machinery; In-side `tf` name reserved; Out-side `tf` legal |
| `TestBuildResolution` (10) | per-instance resolution (go2a/go2b distinct edges), empty-prefix drop, `fields()` carry specs, non-tf `frames` raises, `[tf-template-unknown]` names module+field+template, `[tf-self-edge]`, `[tf-frame-empty]`, `[tf-duplicate-edge]` incl. reversed, tf-free module pays nothing |

Skip-gated (`T11 impl pending` — implementer deletes the marks; 24 written):

| Test | Pins |
| --- | --- |
| `TestTfBuffer` (12) | direct interpolation at bracket, exact-hit identity, no extrapolation both sides, reverse-inverts, chain worked example (§5.3 numbers), identity fast path, statics at any ts + composing with dynamics, sorted out-of-order insert, duplicate first-wins, window eviction + expired drops + newest survives, claims single-writer (reversed, idempotent, release), claims forest |
| `TestOverIntegration` (8) | required tf at tick ts via `over(tf=...)`, hold across a late bracket on a multiplexed stream, final drop at exhaustion, optional defaults without holding, statics-only buffer run, `[tf-unexpected-stream]`, `[tf-missing-stream]`, chunking determinism |
| `TestTfOutRouting` (3) | self-ingestion brackets with stream (§7.3), `[tf-out-frame-mismatch]`, `[tf-out-unstamped]` |

Implementer additions expected (not written here — need live rim scaffolding
from `test_rim.py`): shared-buffer sibling visibility across two live
modules; live hold parks the session thread and `stop()` still drains;
claims released on stop → restart re-claims; TFMessage batch ingestion via
a transport binding; a T4 static fixture for `over(tf=...)` typing and
`m.i.X.frames`; an oracle-style random-schedule determinism test on the
buffer (mirror T5 §10) if cheap.

## 15. Acceptance criteria

- [ ] `uv run mypy dimos/pure/` clean (strict) — holds for the skeleton
      now; must still hold with bodies + align/drivers/rim/typing wiring.
- [x] `dimos/pure/__init__.py` imports pull no numpy/msgs (pinned
      zero-engine test) — tfbuffer stays off the surface.
- [x] Build-time behavior landed + live-tested (specifiers, templates,
      frames, module-scope checks).
- [ ] All 24 skip-gated tests pass with the marks deleted; the 430-test
      suite stays green (no behavior change for tf-free modules).
- [ ] §10 error copy verbatim; wiring errors eager (over()/start(), caller
      thread); data errors at the pulling `next()`/the tap.
- [ ] No forked math: composition via `Transform.__add__`/`inverse`,
      interpolation via `interpolators.interp_transform`, BFS/retention per
      §13 — no second slerp, no Condition, no wall clock anywhere in
      `dimos/pure/tfbuffer.py` (grep-clean for `time.time`, `monotonic`,
      `Condition`, `wait`).
- [ ] Determinism: `test_determinism_chunking` green; buffer stats
      reproduce across identical runs.
- [ ] API floor: the four-declaration Tagger unchanged; a tf-free module
      allocates no tf state (pinned).

## 16. Decisions within mandate (numbered for review)

- **D1** Specifiers + templates live in `rows.py` (the `field_specifiers`
  tuple demands it); engine in a leaf module (§1.2).
- **D2** Engine file named `tfbuffer.py` — `tf.py` would clobber `pm.tf`
  (§1.2; relitigated §18.1).
- **D3** Sparse `tf_out` requires explicit `default=None` — mypy call-site
  rule, verified (§3.4; sketch amendment §18.2). `tf_out` without default
  is a required Out field.
- **D4** `tf` is a reserved In-side field name, unconditionally; the tf
  stream rides `over(tf=...)` / `m.i.tf` (§3.1, §8.1).
- **D5** tf is a demand-pulled side channel at fire time, not a merge port
  — no declaration index, no `_PortState` (§6.1–6.2).
- **D6** Buffer ingestion is sorted-insert per edge with duplicate-ts
  first-wins and expiry/eviction by per-edge window; NOT the T5 frontier
  filter (windows aren't frontiers; the old sorted `save` is the adapted
  behavior; self-assertions require it) (§4.2).
- **D7** Interpolation via `interpolators.interp_transform` directly —
  engine-internal, no registry, no `install()` dependency (§5.1).
- **D8** Required tf holds until resolvable or stream exhausted (the only
  finality proof — new edges can appear); exhaustion → counted final drop.
  Optional never holds (§6.3–6.4).
- **D9** Chain = BFS shortest path over hops resolvable at `at`; reversed
  hops invert; neighbor order = edge first-creation order (deterministic)
  (§5.2).
- **D10** `resolve(f, f, at)` = identity at `at` (runtime only;
  declarations reject self-edges) (§5.4).
- **D11** The tap ingests assertions into the run buffer in BOTH over()
  and live — buffer state is sampler machinery, not an effect; topic
  publish stays the rim-only effect (§7.3).
- **D12** Claims: undirected single-writer + forest over claimed+static
  edges, per buffer; foreign ingested edges exempt; `release(owner)` at
  teardown (§4.4).
- **D13** One `TfError(AlignmentError)` runtime family with `TfRule` slugs;
  build-time errors are `PureModuleDefinitionError` (§10).
- **D14** Buffer thread-safety = one plain Lock; no Condition, no waiting
  (§4.5).
- **D15** tf field annotations unvalidated at runtime; mypy owns them
  (§3.5).
- **D16** `horizon` rides `TfBuffer` (default 10.0); no retention knob on
  `align()`/`over()` — T5 §15.1's placement honored (§4.2).
- **D17** Statics seed via `TfBuffer(statics=)`/`set_static` by the run
  owner; claimed under `STATIC_OWNER` (§4.3).
- **D18** Live injection: synthetic `m.i.tf` port (stream) +
  `tfbuffer.attach` (buffer); unwired-legal; over() uses `tf=` only —
  attach must not leak into over() runs (fresh-per-call doctrine) (§9.1–9.2).

## 17. Open questions (proceeding on defaults)

- **Q1**: `tf_view()` snapshot escape hatch? / default: DEFERRED (§12.5) /
  options: (a) defer until a viz consumer exists (chosen — charter marks it
  optional); (b) ship a thin frozen wrapper over `resolve` now.
- **Q2**: Legacy `/tf` topic interop (mixed legacy/pure graphs)? / default:
  defer the dedicated adapter to T12/e2e; `PubSubTF` already satisfies
  `Publishable` structurally for egress, TFMessage ducks ingest on the way
  in / options: (a) chosen; (b) ship a `TfTopicTransport` adapter in T11;
  (c) wire `ModuleConfig.tf_transport` through the legacy actor now.
- **Q3**: `min_hz=` on `tf_out()`? / default: kept on the spec (T9 consumes
  it as contract cadence; "cadence composes" per the sketch) / options:
  (a) chosen; (b) drop it and require a separate `contract()` field —
  rejected: the edge and its cadence are one declaration.
- **Q4**: Should optional tf fields get a bounded hold (resolve-if-cheap)?
  / default: NO — optional never holds; deterministic and simple / options:
  (a) chosen; (b) hold up to N pulls — rejected as a tuning knob with no
  principled N.
- **Q5**: Cross-process single-writer/cycle enforcement? / default: out of
  scope — per-buffer claims here; whole-graph checks belong to wire()/plan
  on the instance graph (sketch text) / options: (a) chosen; (b) a
  claims-over-topic protocol — new distributed machinery, wrong task.
- **Q6**: Buffer `PortStats` naming — reuse T5's `PortStats` for the
  aligner's `"tf"` entry with buffer detail in `TfStats`? / default: yes
  (T9 reads one familiar shape; detail stays qualified) / options: (a)
  chosen; (b) a dedicated stats type in `AlignStats` — schema churn for T9.

## 18. Relitigation / amendments

### 18.1 Engine file name: `tfbuffer.py`, not the index layout's `tf.py`

- **Decision under review**: index "Proposed layout" row `tf.py  # T11`.
- **What breaks**: Python's import system binds a submodule onto its parent
  package attribute; any import of `dimos.pure.tf` would overwrite the
  `pm.tf` specifier function on the package, and `from dimos.pure import
  tf` becomes import-history-dependent (module vs function). The sketch's
  `pm.tf` spelling and a `tf` submodule are mutually exclusive.
- **Alternative considered**: rename the specifier — rejected; the sketch
  (source of truth) fixes `tf("world", "base_link")`.
- **Blast radius**: one index layout row (amend to `tfbuffer.py  # T11`);
  no API change — users never import the engine module directly except for
  `TfBuffer` in run-owner code.

### 18.2 Sketch spelling: sparse `tf_out` gains `default=None`

- **Decision under review**: sketch §5 `tf_world_map: Transform | None =
  tf_out("world", "map")` and §5b's analogous line — sparse-by-default.
- **What breaks**: verified mypy behavior (§3.4) — without a call-site
  `default=`, the field is statically REQUIRED at row construction while
  runtime treats it sparse; hand-constructed rows (the "tests need no
  engine" doctrine) would fight the type checker on every Out.
- **Proposal**: canonical sparse spelling `tf_out(parent, child,
  default=None)`, parallel to `latest`/`contract`; bare `tf_out(...)` means
  required-every-emission. Sketch §5/§5b lines amended when the sketch is
  next touched; T12 examples use the new spelling.
- **Blast radius**: two sketch lines + future examples; no landed code
  assumed the old spelling.
