# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# SKETCH 3 — not functional. Companion to puremodule_api_sketch2 (sketch 2):
# the same modules, respelled so a module is ONE lexical unit and the step
# signature is the single typing authority. What changed, and why:
#
#   1. GENERICS ARE GONE. `class Tagger(PureModule)` — no type parameters, no
#      separate TaggerIn/TaggerOut declarations feeding a base-class subscript.
#      The step signature already tells the engine everything (stateless vs
#      Mealy vs async — sketch 2 dispatched on it anyway); now it tells mypy
#      too: engine methods recover TIn/TOut through a protocol-typed self
#      (§typing at the bottom), so over() / transform() / ports stay fully
#      typed at every call site. Verified: bare `-> Out`, skipping
#      `-> Out | None`, async, and Mealy all solve to the right row type.
#   2. BUNDLES NEST. In / Out / State live inside the module class. A module
#      reads top-to-bottom as its complete spec — config fields, input row,
#      output row, state, resources, step — and the row type's public name is
#      `Tagger.In`: self-namespacing, no TaggerIn/TaggerOut name inflation.
#      Nesting is placement, not mechanism: a shared bundle may live at module
#      level, and any module may reference another's rows by path (§7 reuses
#      VoxelGridMapper.In). Everything sketch 2 says about bundles still
#      holds — real dataclasses via @dataclass_transform bases, kw-only
#      engine-stamped ts, required-vs-sparse in the constructor.
#   3. SCOPING, stated once. The bases come in as `pm.In` / `pm.Out` — the
#      bare names belong to the nested classes, so there is no shadowing to
#      squint at. Inside the defining class body, `def step(self, i: In)`
#      resolves to the nested class (class-body names are visible to
#      annotations there — both mypy and runtime agree). OUTSIDE the body —
#      subclasses included, since class attributes are not lexically scoped —
#      you write `Tagger.In`. Engine internals resolve step's hints with
#      localns=vars(cls); users never see that.
#   4. SHAPES. A module whose step raises NotImplementedError is an
#      inheritable I/O contract: implementations inherit the bundles and
#      override step, and mypy enforces conformance AT THE DEF — wrong return
#      type or a narrowed input (demanding more fields than the shape
#      declares) is an [override] error, before any wiring exists. The
#      structural twin comes free from the same machinery: Stateless[C.In,
#      C.Out] accepts anything step-shaped, no MRO membership required (§4).
#   5. Everything else is sketch 2, unchanged: step-not-fold with fold as the
#      escape hatch, Out constructed not written, State-vs-resource doctrine,
#      effects as outputs, async as a policy, stamped payloads with no
#      Observation wrapper in dataflow.
#
# The cost, priced: definition-site errors move. With explicit generics a
# malformed module errored at its own class statement; here mypy flags a bad
# step only where the engine touches it ("Invalid self argument" at .over()).
# PureModule.__init_subclass__ compensates at import time — step XOR fold,
# signature shape, bundle sanity — so the runtime failure is still immediate
# and named, just not a mypy definition-site failure.

from __future__ import annotations

from collections.abc import Iterator
from typing import TYPE_CHECKING, NamedTuple

from dimos import pure as pm
from dimos.pure import (
    PureModule,
    contract,
    interpolate,
    latest,
    resource,
    tf_out,
    tick,
)

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
    from dimos.msgs.nav_msgs.Odometry import Odometry
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


# ══════════════════════════════════════════════════════════════════════════════
# 1. Tagger — the common case, now at its floor: one class, four declarations.
#    Nothing is said twice — no generic subscript, no module-level row types
#    echoing the class name. Bare In/Out in step's signature are the nested
#    classes (§3 scoping); `-> Out` still documents cadence: emits every tick.
# ══════════════════════════════════════════════════════════════════════════════
class Tagger(PureModule):
    class In(pm.In):
        image: Image = tick(expect_hz=30)
        pose: PoseStamped = interpolate()

    class Out(pm.Out):
        located: str = contract(min_hz=10)

    def step(self, i: In) -> Out:
        return Tagger.Out(
            located=f"bright={i.image.brightness:.2f} @ ({i.pose.position.x:.2f}, {i.pose.position.y:.2f})"
        )


# ══════════════════════════════════════════════════════════════════════════════
# 2. QualityGate — single-in → multi-out, sparse, unchanged in substance:
#    required-vs-sparse is still a property of the Out type. Only the spelling
#    moved indoors.
# ══════════════════════════════════════════════════════════════════════════════
class QualityGate(PureModule):
    class In(pm.In):
        image: Image = tick(expect_hz=30)

    class Out(pm.Out):
        quality: float = contract(min_hz=10)
        alert: str | None = None  # sparse — silent unless set

    def step(self, i: In) -> Out:
        b = i.image.brightness
        return QualityGate.Out(
            quality=round(b, 3),
            alert=f"dark frame ({b:.2f})" if b < 0.42 else None,
        )


# ══════════════════════════════════════════════════════════════════════════════
# 3. VoxelGridMapper — resource + State, and the payoff of nesting: config,
#    In, Out, State, the resource, and step are one continuous read. State
#    was already nested in sketch 2 — In/Out simply joined it.
# ══════════════════════════════════════════════════════════════════════════════
class VoxelGrid:  # placeholder heavyweight resource
    def __init__(self, voxel_size: float, carve_columns: bool = False) -> None: ...
    def add_frame(self, scan: PointCloud2) -> None: ...
    def get_global_pointcloud2(self) -> PointCloud2: ...
    def dispose(self) -> None: ...


class VoxelGridMapper(PureModule):
    voxel_size: float = 0.05
    emit_every: int = 5

    class In(pm.In):
        lidar: PointCloud2 = tick()

    class Out(pm.Out):
        global_map: PointCloud2 = contract(min_hz=1)

    class State(NamedTuple):
        n: int = 0  # ticks folded so far

    @resource
    def grid(self) -> VoxelGrid:  # engine calls .dispose() at teardown
        return VoxelGrid(voxel_size=self.voxel_size, carve_columns=True)

    def step(self, state: State, i: In) -> tuple[State, Out | None]:
        self.grid.add_frame(i.lidar)
        state = state._replace(n=state.n + 1)

        if state.n % self.emit_every:
            return state, None

        return state, VoxelGridMapper.Out(global_map=self.grid.get_global_pointcloud2())


# ══════════════════════════════════════════════════════════════════════════════
# 4. CostMapper — a SHAPE with implementations. The multi-in-with-optional
#    module from sketch 2, now abstract: the class fixes the I/O contract
#    (including samplers and the output contract — implementations inherit the
#    cadence obligation, not just the types), and step raises. Conformance is
#    checked at the def: an implementation that narrows In or widens Out is an
#    [override] error before any wiring exists. Implementations inherit the
#    bundles — note HeightCostMapper and NeuralCostMapper add config but no
#    new I/O declarations, and write `CostMapper.In` per §3 scoping.
# ══════════════════════════════════════════════════════════════════════════════
class CostMapper(PureModule):
    class In(pm.In):
        global_map: PointCloud2 = tick()
        relocalized_map: PointCloud2 | None = latest(default=None)

    class Out(pm.Out):
        global_costmap: OccupancyGrid = contract(min_hz=2)

    def step(self, i: In) -> Out:
        raise NotImplementedError


class HeightCostMapper(CostMapper):
    max_height: float = 0.35

    def step(self, i: CostMapper.In) -> CostMapper.Out:
        src = i.relocalized_map or i.global_map
        return CostMapper.Out(global_costmap=self._height_cost(src))

    def _height_cost(self, msg: PointCloud2) -> OccupancyGrid: ...


class NeuralCostMapper(CostMapper):
    checkpoint_file: str = "costnet.pt"

    def step(self, i: CostMapper.In) -> CostMapper.Out:
        src = i.relocalized_map or i.global_map
        return CostMapper.Out(global_costmap=self._infer(src))

    def _infer(self, msg: PointCloud2) -> OccupancyGrid: ...


# The structural twin: anything step-shaped conforms, inheritance optional.
# `def deploy(m: CostMapperLike)` accepts HeightCostMapper AND an unrelated
# module with the same step signature. Inherit to share bundles and get
# def-site checking; use the alias to accept "anything costmapper-shaped".
if TYPE_CHECKING:
    from dimos.pure import Stateless

    CostMapperLike = Stateless[CostMapper.In, CostMapper.Out]


# ══════════════════════════════════════════════════════════════════════════════
# 5. RelocalizationModule — Mealy + resource + effects-as-outputs, verbatim
#    from sketch 2 modulo the spelling. The reloc throttle still reads i.ts.
#    The TF broadcast is now tf_out("world", "map"): a sparse Transform port
#    that DECLARES the frame edge it asserts — single-writer per edge is a
#    plan-time check, payload frames validate against the declaration on
#    emission. The engine routes an assertion three ways, module oblivious:
#    local shared buffer (sibling tf() samplers see it, no wire round-trip),
#    flight recorder, and the tf topic only where bound at the rim. Inert
#    under over(), like every effect.
# ══════════════════════════════════════════════════════════════════════════════
RELOC_INTERVAL = 2.0
MIN_LOCAL_POINTS = 50_000


class RelocalizationModule(PureModule):
    map_file: str | None = None
    publish_loaded_map: bool = False

    class In(pm.In):
        global_map: PointCloud2 = tick()

    class Out(pm.Out):
        relocalized_map: PointCloud2 = contract(min_hz=1)
        tf_world_map: Transform | None = tf_out("world", "map")  # frame edge asserted
        loaded_map: PointCloud2 | None = None  # sparse — viz only

    class State(NamedTuple):
        world_to_map: Transform | None = None
        last_reloc_ts: float = float("-inf")

    @resource
    def premap(self) -> PointCloud2 | None:  # warmup — read the prebuilt map
        return self._load_premap()

    def step(self, state: State, i: In) -> tuple[State, Out | None]:
        if self.premap is None:
            return state, None  # disabled: no map_file
        scan = i.global_map
        if i.ts - state.last_reloc_ts >= RELOC_INTERVAL and len(scan) >= MIN_LOCAL_POINTS:
            state = state._replace(
                world_to_map=self._try_relocalize(scan) or state.world_to_map,
                last_reloc_ts=i.ts,
            )
        if state.world_to_map is None:
            return state, None  # silent until first fix
        return state, RelocalizationModule.Out(
            relocalized_map=self._merge(scan, state.world_to_map, self.premap),
            tf_world_map=state.world_to_map.now(),
            loaded_map=self.premap if self.publish_loaded_map else None,
        )

    def _load_premap(self) -> PointCloud2 | None: ...
    def _try_relocalize(self, scan: PointCloud2) -> Transform | None: ...
    def _merge(self, scan: PointCloud2, tf: Transform, premap: PointCloud2) -> PointCloud2: ...


# ══════════════════════════════════════════════════════════════════════════════
# 5b. Go2Connection — tf_out with CONFIG-TEMPLATED frames. Frame names
#     resolve at build against instance config: "{prefix}/odom" is a
#     template; the resolver drops empty segments, so prefix="" gives plain
#     "odom" and prefix="go2a" gives "go2a/odom". The class declares the
#     SHAPE of the assertion; each configured instance owns a concrete,
#     distinct frame edge — five connections with five prefixes are five
#     edges, and the plan's single-writer check validates them as such.
#     Module code never mentions the fleet.
# ══════════════════════════════════════════════════════════════════════════════
class Go2Connection(PureModule):
    prefix: str = ""  # per-robot namespace — the ONLY fleet-aware line, and it's config
    robot_ip: str | None = None
    odom_timeout: float = 0.5

    class In(pm.In):
        odom: Odometry = tick(expect_hz=50)

    class Out(pm.Out):
        pose: PoseStamped = contract(min_hz=10)
        tf_odom_base: Transform | None = tf_out("{prefix}/odom", "{prefix}/base_link")

    def step(self, i: In) -> Out:
        return Go2Connection.Out(
            pose=self._pose(i.odom),
            tf_odom_base=self._body_transform(i.odom),
        )

    def _pose(self, odom: Odometry) -> PoseStamped: ...
    def _body_transform(self, odom: Odometry) -> Transform: ...


def _five_robots_one_graph() -> None:
    # a coordinator graph's wire() does this in a loop; shown flat here.
    conns = {name: Go2Connection(prefix=name) for name in ("go2a", "go2b", "go2c")}

    # frames are concrete on the configured instance — introspectable, and
    # what the plan's single-writer/cycle checks actually run against:
    assert conns["go2a"].o.tf_odom_base.frames == ("go2a/odom", "go2a/base_link")
    assert conns["go2b"].o.tf_odom_base.frames == ("go2b/odom", "go2b/base_link")

    # consumers parameterize the same way — a per-robot follower samples its
    # OWN robot's chain:  tf("{prefix}/odom", "{prefix}/base_link")


# The other topology — one graph PER robot — needs no templates at all:
# declare canonical frames (prefix="") and let bind() rewrite them at the
# rim, symmetric with topic naming:
#     g.bind(pLCMTransport, prefix="/go2a", tf_prefix="go2a")


# ══════════════════════════════════════════════════════════════════════════════
# 5c. CONFIG — flat fields are the API, a synthesized FROZEN BaseConfig is
#     the substance (interop with dimos/protocol/service/spec.py).
#     __init_subclass__ already walks the class body; it also collects the
#     plain annotated fields — everything that isn't In/Out/State, @resource,
#     or @rpc — into a generated pydantic model (extra="forbid", frozen=True)
#     and routes __init__ kwargs through it. @dataclass_transform on
#     PureModule types the constructor statically — Configurable's
#     `**kwargs: Any` never could. FROZEN is load-bearing: module identity =
#     class + config everywhere the sketches lean (memo keys, member paths,
#     checkpoint metadata, counterfactual sweeps); mutable config would
#     quietly break all of it — rebuild instead, rebuilding is free.
#     warmup/start/stop satisfy the Service protocol, so a pure module drops
#     into any blueprint slot that hosts a Service today: the migration
#     path, not a redesign.
# ══════════════════════════════════════════════════════════════════════════════
def _config_machinery() -> None:
    c = Go2Connection(prefix="go2a", robot_ip="192.168.12.1")  # typed kwargs — mypy-checked
    assert c.robot_ip == "192.168.12.1"  # flat access: the API, self.x throughout step

    # validation happened AT CONSTRUCTION, from either checker:
    #   Go2Connection(prefx="go2a")   → mypy error, AND pydantic extra="forbid" at runtime
    #   Go2Connection(robot_ip=0.5)   → mypy error, AND pydantic ValidationError
    # (runtime matters: config increasingly arrives from YAML/CLI/blueprints,
    #  where mypy isn't looking.)

    # the synthesized model is the canonical serialization the rest of the
    # file already assumed existed:
    assert c.config.model_dump() == {
        "prefix": "go2a",
        "robot_ip": "192.168.12.1",
        "odom_timeout": 0.5,
    }
    #   memo/cache key = (code hash, c.config.model_dump(), input edge hashes)
    #   checkpoint     = {"state": ..., "config": c.config.model_dump()}
    #   inspector      = member config panel, same dump
    #   sweep          = Go2Connection(**{**c.config.model_dump(), "odom_timeout": 1.0})

    # frozen — rebuild, never mutate:
    #   c.config.odom_timeout = 1.0  → pydantic frozen error
    #   c.odom_timeout = 1.0         → same guard at the module surface


# ══════════════════════════════════════════════════════════════════════════════
# 6. Captioner — ASYNC step, engine semantics unchanged from sketch 2
#    (max_inflight window, tick-order emission, drain-on-stop, aclose).
#    `async def step` matches the AsyncStateless protocol, so over() on a
#    Captioner still yields typed Out rows.
# ══════════════════════════════════════════════════════════════════════════════
class VLMClient:  # placeholder async I/O client
    async def caption(self, image: Image) -> str: ...
    async def aclose(self) -> None: ...


class Captioner(PureModule):
    max_inflight: int = 4

    class In(pm.In):
        image: Image = tick(expect_hz=2)

    class Out(pm.Out):
        caption: str = contract(min_hz=1)

    @resource
    def client(self) -> VLMClient:  # engine awaits aclose() at stop
        return VLMClient()

    async def step(self, i: In) -> Out:
        return Captioner.Out(caption=await self.client.caption(i.image))


# ══════════════════════════════════════════════════════════════════════════════
# 7. fold — the escape hatch, unchanged in doctrine (generator-local state, no
#    checkpoint, fold stamps its own rows). Nesting is placement (§2): this
#    module has VoxelGridMapper's exact I/O, so it says so — no bundle of its
#    own, rows referenced by path. Reading `rows: Iterator[VoxelGridMapper.In]`
#    IS the compatibility claim.
# ══════════════════════════════════════════════════════════════════════════════
class ScanBatcher(PureModule):
    batch_size: int = 8

    def fold(self, rows: Iterator[VoxelGridMapper.In]) -> Iterator[VoxelGridMapper.Out]:
        batch: list[PointCloud2] = []
        for r in rows:
            batch.append(r.lidar)
            if len(batch) == self.batch_size:
                yield VoxelGridMapper.Out(ts=r.ts, global_map=self._concat(batch))
                batch.clear()

    def _concat(self, scans: list[PointCloud2]) -> PointCloud2: ...


# Deliberately left out, still: a @pure_module function decorator and
# bare-return sugar (one spelling per capability). Newly left out: a separate
# Shape/Interface base — a shape is just a module whose step raises, and
# abstractness is a doctrine, not a metaclass.


# ══════════════════════════════════════════════════════════════════════════════
# TESTS NEED NO ENGINE — identical to sketch 2, with row types spelled by
# path. Construct a row, call step, assert on the bundle.
# ══════════════════════════════════════════════════════════════════════════════
def _fake_image(brightness: float) -> Image: ...
def _fake_pose(x: float, y: float) -> PoseStamped: ...
def _fake_scan() -> PointCloud2: ...


def _unit_tests() -> None:
    out = Tagger().step(Tagger.In(ts=0.0, image=_fake_image(0.80), pose=_fake_pose(1.0, 2.0)))
    assert out.located == "bright=0.80 @ (1.00, 2.00)"

    m = VoxelGridMapper(emit_every=2)
    state, o1 = m.step(VoxelGridMapper.State(), VoxelGridMapper.In(ts=0.0, lidar=_fake_scan()))
    state, o2 = m.step(state, VoxelGridMapper.In(ts=0.1, lidar=_fake_scan()))
    assert o1 is None and o2 is not None


# ══════════════════════════════════════════════════════════════════════════════
# TYPING — how over() knows its row types with no generics on the class.
# This is dimos.pure INTERNALS, shown because it is the load-bearing trick:
# each step shape is a Protocol, and engine methods take a protocol-typed
# self; mypy solves TIn/TOut against the concrete step at every call site.
# All four shapes verified under mypy --strict: bare `-> Out`, skipping
# `-> Out | None` (TOut still solves to Out, not Out | None), async, Mealy.
# Ordering matters: AsyncStateless is listed before Stateless so a coroutine-
# returning step can't unify with the sync overload. Mealy vs Stateless are
# disjoint by arity. A fold module matches none of the step protocols and
# falls to the Fold overload. m.i / m.o use the same trick one level down:
# port handles are descriptors whose __get__ is overloaded on a
# protocol-typed instance, so m.i is typed per-module with zero annotations
# on the module itself.
#
#     TIn = TypeVar("TIn", contravariant=True)
#     TOut = TypeVar("TOut", covariant=True)
#
#     class AsyncStateless(Protocol[TIn, TOut]):
#         def step(self, i: TIn) -> Awaitable[TOut | None]: ...
#
#     class Mealy(Protocol[TState, TIn, TOut]):
#         def step(self, state: TState, i: TIn) -> tuple[TState, TOut | None]: ...
#
#     class Stateless(Protocol[TIn, TOut]):
#         def step(self, i: TIn) -> TOut | None: ...
#
#     class Fold(Protocol[TIn, TOut]):
#         def fold(self, rows: Iterator[TIn]) -> Iterator[TOut]: ...
#
#     class PureModule:
#         @overload
#         def over(self: AsyncStateless[TIn, TOut], **s: Streamable) -> Iterator[TOut]: ...
#         @overload
#         def over(self: Mealy[TState, TIn, TOut], **s: Streamable) -> Iterator[TOut]: ...
#         @overload
#         def over(self: Stateless[TIn, TOut], **s: Streamable) -> Iterator[TOut]: ...
#         @overload
#         def over(self: Fold[TIn, TOut], **s: Streamable) -> Iterator[TOut]: ...
#
# What moved where, honestly: a malformed step is no longer a mypy error at
# the class statement — it surfaces at the first engine call site as an
# "Invalid self argument". PureModule.__init_subclass__ backstops at import
# time: exactly one of step/fold, a recognized signature shape, In/Out
# resolvable (via get_type_hints(step, localns=vars(cls))), State declared
# iff step is Mealy. Import fails loudly with the module named — the same
# moment a dataclass with a bad field order fails today.
# ══════════════════════════════════════════════════════════════════════════════


# ══════════════════════════════════════════════════════════════════════════════
# DEPLOYMENT — sketch 2 verbatim: one way to run offline (configure, then
# over()), ports under m.i / m.o, transform equivalence for single-input
# modules. Shown here only where the respelling is visible: row types by
# path, shapes at wiring time.
# ══════════════════════════════════════════════════════════════════════════════
def _offline_over() -> None:
    from dimos.memory2.store.sqlite import SqliteStore
    from dimos.utils.data import get_data

    store = SqliteStore(path=get_data("go2_run.db"))
    costmaps = HeightCostMapper(max_height=0.35).over(
        global_map=store.streams.global_map,
        relocalized_map=store.streams.relocalized_map,
    )
    for row in costmaps:  # CostMapper.Out rows, tick-ordered, row.ts = tick time
        print(row.global_costmap)


def _shape_at_wiring_time() -> None:
    from dimos.core.transport import pLCMTransport

    # deployment code targets the SHAPE; which implementation runs is config.
    def launch(m: CostMapper) -> None:  # or CostMapperLike for structural
        m.i.global_map.transport = pLCMTransport("/map/global")
        m.i.relocalized_map.transport = pLCMTransport("/map/relocalized")
        m.o.global_costmap.transport = pLCMTransport("/costmap/global")
        m.warmup()
        m.start()

    launch(HeightCostMapper())
    launch(NeuralCostMapper(checkpoint_file="costnet_v2.pt"))


def _as_transform() -> None:
    from dimos.memory2.store.sqlite import SqliteStore

    store = SqliteStore(path="walk.db")
    maps = store.streams.lidar.transform(VoxelGridMapper(voxel_size=0.1))
    maps.map(lambda row: row.global_map).save(store.streams.global_map).drain()


def _checkpoint_resume() -> None:
    m = RelocalizationModule(map_file="office_map")
    m.warmup()
    m.start()
    snap = m.checkpoint()  # State + tick cursor, plain data
    m.stop()

    m2 = RelocalizationModule(map_file="office_map")
    m2.restore(snap)  # resumes with the reloc fix already in hand
    m2.warmup()
    m2.start()


# ══════════════════════════════════════════════════════════════════════════════
# NOTE: HOT RELOAD & RE-EVAL — a corollary, not a feature. A run over a store
# is a pure computation: (code, config, State₀, recorded inputs) → outputs.
# step has no wall clock (time is i.ts), no hidden I/O beyond declared
# resources, effects are outputs (replay can't touch the live system), and
# wire() is pure and costs nothing — so the coarse loop is free by
# construction: watch the file, reload, re-wire, re-run over(). The notebook
# workflow is the same loop by hand: construction is config-only and
# resources are lazy, so re-instantiating in a cell costs nothing.
#
# The incremental version rides the flight recorder: with every edge
# recorded, an edit to member M is dataflow cache invalidation — upstream
# edges are unchanged recordings, so only M and its downstream cone recompute,
# M reading its inputs straight from the store. Memoization key per member:
# (code hash, config, input edge hashes) — a build system whose dependency
# graph is literally the DAG. Mid-run swap follows from engine-owned State:
# checkpoint() is State + tick cursor as plain data, so swap code →
# restore(snap) → continue — bounded by State schema compatibility (an edit
# that reshapes State needs a migration, or re-fold from t0) and by the
# resource doctrine (an accumulating resource rebuilds only by replaying its
# member's cone — cheap offline, since that edge is recorded).
#
# The one place "pure" and "re-eval is identical" come apart: nondeterministic
# resources. The Captioner's VLM client returns different captions for the
# same rows, so an untouched downstream cone still re-runs on changed data.
# Escalating remedies: accept it (re-eval is exact only for deterministic
# members); memoize step outputs keyed by input row (rows are serializable —
# the cassette move); pin recorded outputs and re-execute only the edited
# member. Mechanical footnote: importlib.reload changes class identity — the
# engine keys members by path + config (graph namespacing already does),
# never by class object.
# ══════════════════════════════════════════════════════════════════════════════


# ══════════════════════════════════════════════════════════════════════════════
# NOTES: WHAT ELSE RUNS-AS-VALUES BUYS — each a tool or workflow, none
# requiring new doctrine. Small notes, roughly in order of leverage:
#
# • RUN INSPECTOR (forensics tool). Per-member message view, rewind, State at
#   any tick — State at tick N reconstructs by folding from the nearest
#   checkpoint, and over() runs faster than realtime (no timers), so
#   scrubbing is cheap. "When did it break" is binary search over ticks —
#   git-bisect over time. A live crash under the flight recorder yields
#   (State, In row) at the failing tick: a complete deterministic repro,
#   small enough to attach to an issue; delta-debug the input stream for the
#   minimal failing sequence. Provenance is the same query read backwards:
#   "why did it turn left at t=4032" = replay that tick, inspect the row.
#
# • BEHAVIORAL DIFF IN CI. Old and new code over the same store, diff the
#   edge streams structurally: "this PR changes cmd_vel from tick 4032
#   onward." Golden-run regression tests with no assertions written.
#
# • COUNTERFACTUAL FLEET. N variants of the same graph — a member swapped
#   (the shape guarantees the slot), or a config swept — over identical
#   recorded inputs; everything else evaluates the same, so the runs diverge
#   only downstream of the change: diff the paths. With content-addressed
#   edges the shared prefix computes ONCE — N variants cost one common
#   prefix plus N cones. Parameter tuning is this in a for-loop (configs are
#   constructor args); Bayesian optimization on real data, no robot.
#
# • PROPERTY-BASED TESTING. In rows are constructible dataclasses →
#   Hypothesis generates them; a Mealy module is an explicit (State, In) →
#   (State, Out) transition function — exactly the shape stateful property
#   testing and model checking want.
#
# • BATCH / DISTRIBUTED OFFLINE. Pure step + serializable rows → a member
#   runs anywhere; a stateless module's ticks are independent, so captioning
#   a two-hour run fans out embarrassingly parallel. Async (§6) is the
#   live-time spelling of the same property.
#
# • SHARED EDGE CACHE. The memoization key from the reload note,
#   content-addressed and remote: expensive stages computed once per
#   (code, config, inputs) and reused across the team — a build cache for
#   robot data.
#
# • LIVE MIGRATION. checkpoint on robot A, restore on robot B — State is
#   plain data, the rim is just transports; nothing about a run is pinned to
#   a host.
# ══════════════════════════════════════════════════════════════════════════════


# ══════════════════════════════════════════════════════════════════════════════
# NOTE: HEALTH — engine-owned, one shared type, its own clock. Modules only
# DECLARE (contract/expect_hz on ports); the engine MEASURES: ONE global
# stamped Health row (path, out_hz, drops, contract_ok, step_ms_p95, sparse
# violation/resource_error) — never per-module, never in an Out bundle.
# Rows are wall-clock paced (health_hz knob, default 1 Hz, per-member
# override), decoupled from module cadence: a 0.1 Hz mapper healths at 1 Hz,
# and a stalled module still healths — the most important health row
# describes a module that emitted nothing, so health can't ride the output.
# Exposed as a rim stream (m.health; g.health = members merged, path-keyed):
# transport → pubsub for dashboards, recorded by the flight store, one
# uniform type covers every module ever written. Offline, data-time cadence
# is a QUERY over recorded ts — only live-only facts (wall latency, drops,
# resource errors) need the recorded stream. Domain metrics (voxel count)
# are sparse Out ports on the data plane, not health.
# ══════════════════════════════════════════════════════════════════════════════


# ══════════════════════════════════════════════════════════════════════════════
# NOTE: TF — a sampler, not a service, and never State. Today's PubSubTF
# (dimos/protocol/tf/tf.py) is an ambient service with a blocking wait; the
# pure spelling: TF lookup is interpolate() generalized by one step —
# interpolation PLUS chain composition over one well-known stream — so it
# declares in In:
#
#     class In(pm.In):
#         image: Image = tick(expect_hz=30)
#         world_to_base: Transform = tf("world", "base_link")            # required
#         world_to_cam: Transform | None = tf("world", "cam", default=None)
#
# Declaring a tf() field IS "announce we care about the tf topic": it makes
# the module a consumer of the tf edge — visible to wire(), bindable like
# any input (tf=store.streams.tf under over()). The row carries the RESOLVED
# transform at tick ts — plain data: tests hand-construct a Transform,
# recording ticks = serializing rows stays true. No module holds a buffer:
# samplers already keep engine-side history per port (latest/interpolate —
# nobody calls those module state); the tf accumulator is ONE engine-owned
# MultiTBuffer per process, shared by all declarers — tf.py's machinery
# (graph search, interpolation, retention) survives as the sampler's
# implementation, not as API surface. Rebuilds from the recorded tf edge;
# re-warm needs only the retention horizon (10 s), not t0. _wait_get dies:
# a required tf field HOLDS ticks until the chain resolves (interpolate's
# waiting semantics); default=None makes it optional — waiting is the
# engine's job, modules never block. Publish side: tf_out(parent, child) —
# a sparse Transform port that DECLARES its asserted edge (§5's
# tf_world_map): single-writer per edge and frame-cycle checks at plan
# time, payload validated against the declaration on emission (like msg
# ts); cadence composes — tf_out(..., min_hz=1) is contract machinery. The
# tf tree is the merge of everyone's tf_out ports — the inspector shows who
# asserts each edge. Statics (extrinsics, URDF) are config, seeded into the
# buffer at warmup — the pure publish_static. Frame names RESOLVE AT BUILD,
# not import: tf()/tf_out() take config templates —
# tf_out("{prefix}/odom", "{prefix}/base_link") with prefix an ordinary
# config field — and plan checks (single-writer, cycles) run on the wired
# instance graph, where names are concrete: five Go2Connection members with
# different prefixes are five distinct frame edges, validated as such.
# Whole-graph fleet instantiation instead prefixes at bind(), symmetric
# with topic naming — g.bind(..., tf_prefix="go2a") rewrites every tf()/
# tf_out() name through the same pure naming function as topics; module
# code stays fleet-agnostic either way.
# Escape hatch for frame-agnostic consumers (viz): frames: TfSnapshot =
# tf_view() — immutable queryable snapshot-at-ts, reconstructible from the
# recorded tf edge up to the tick cursor (recording stores the cursor);
# fold's trade — more power, weaker serialization, use rarely.
# ══════════════════════════════════════════════════════════════════════════════


# ══════════════════════════════════════════════════════════════════════════════
# NOTE: RPC — requests are inputs, replies are outputs (ROS2 services are
# this underneath: a request/reply topic pair, correlation id in the
# envelope). Client side needs nothing new: a resource + async step (§6).
# Server side keeps the @rpc spelling but the handler is a TRANSITION on the
# same State — an alternate step for a different input kind:
#
#     @rpc
#     def navigate_to(self, state: State, req: PoseStamped) -> tuple[State, bool]: ...
#
# (stateless form without state; read-only queries return just the reply).
# A handler is a ONE-TICK transition: navigate_to -> bool means ACCEPTED,
# never arrived — a long-running command is still goal-in-State + a status
# output port, the RPC merely sets the goal (a ROS action decomposes into
# exactly this; @rpc is the service half). The engine serializes handlers
# with step on the module loop, stamps each request like an input row, and
# merges it into tick order by ts (equal ts breaks by envelope sequence
# number — replay identity needs the total order) — so RPC-caused state
# changes record, replay, checkpoint, and appear in the inspector timeline
# like any tick. Replies are effects → outputs: inert under over() unless
# wired; a replay can't answer a live caller. Caller sugar: @rpc is a
# descriptor like @resource, so in-process it's `await m.navigate_to(pose)`
# — fully mypy-typed, verified (a .rpc namespace would need a plugin);
# cross-process the topic pair — a transport adapter, invisible to modules.
# Doctrine: module→module RPC INSIDE a graph is discouraged — a hidden edge
# wire() can't see; restructure as ports. RPC's audience is the rim:
# external clients, teleop, the CLI.
# ══════════════════════════════════════════════════════════════════════════════
