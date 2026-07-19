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

# SKETCH 2 — not functional. Companion to puremodule_api_sketch.py (sketch 1):
# the same six modules, redesigned so the common case is a pure function and the
# ENGINE keeps the loop. What changed, and why:
#
#   1. step, not fold. Stateless: `def step(self, i: TIn) -> TOut | None`.
#      Stateful: `def step(self, state: State, i: TIn) -> tuple[State, TOut | None]`
#      — `state` as the first parameter flips the module to Mealy; the engine
#      starts from `State()`. The engine owns iteration, so it can checkpoint
#      state, count drops, mediate backpressure, and replay — the properties that
#      justify "Pure". fold survives as the escape hatch for modules that must
#      own the loop (§7).
#   2. In/Out bundles are REAL dataclasses. In/Out are @dataclass_transform
#      (PEP 681) bases — subclassing applies the machinery, no decorator — with
#      tick/latest/interpolate/window/contract as field specifiers: a field with
#      a sampler and no default stays REQUIRED in the constructor and mypy models
#      it, so tests build rows by hand and call step() with no engine anywhere.
#      Every In bundle carries `ts` (kw-only, engine-stamped); the bundle
#      instance IS the resolved tick row, so recording ticks = serializing these.
#   3. Out is CONSTRUCTED, not written through a step handle. Required ports are
#      constructor-enforced (forgetting one is a mypy error, not a runtime
#      check); sparse ports are `X | None = None` fields where None means silent
#      (a port never carries None); returning None skips the tick. One stamped
#      Out row per tick (rows carry kw-only ts, like In), monotonic — emit-twice
#      and late-emit are unrepresentable.
#   4. State vs resource, named. State is small, serializable, engine-owned →
#      checkpoint / journal / resume / migrate. @resource is declared impurity
#      with engine lifecycle: created at warmup, disposed at stop
#      (close/dispose/aclose), under over() teardown too; outside a run it
#      degrades to a lazy cached property, so unit tests just touch it. A module
#      that accumulates into a resource stays deterministic but replays only
#      from t0 — keep journalable progress in State.
#   5. Effects are outputs. Sketch 1's mid-fold `self.tf.publish(...)` becomes a
#      sparse Out port: live deployment wires it to the TF tree; over() replay
#      just yields values — a replay can no longer rebroadcast onto the live
#      system by accident.
#   6. Async is a policy, not a pattern. `async def step` declares I/O
#      concurrency; a max_inflight knob bounds it. The engine owns the inflight
#      window, drain-on-stop, cancellation, failure accounting, and emits in
#      TICK ORDER by default. Async requires a stateless module (a state chain
#      would serialize it anyway). Sketch 1's hand-rolled task set — which
#      leaked tasks on error and closed the client under running work — has no
#      user-code counterpart left to get wrong.
#   7. STAMPED payloads — no Observation wrapper in dataflow. Streams carry the
#      values themselves: msgs already have .ts, In/Out rows have .ts, and
#      primitives (str/float ports) ride a tiny Stamped[T] envelope only at
#      wire/store boundaries. The engine stamps rows at tick time and validates
#      msg ts on emission. Observation stays what it really is — the memory2
#      store RECORD (id, spatial pose, tags, lazy blob loading) on the query
#      plane — instead of a wrapper on every sample in flight. One timestamp
#      authority; transports speak plain msgs, wire-compatible with the legacy
#      Module stack.

from __future__ import annotations

from collections.abc import Iterator
from typing import TYPE_CHECKING, NamedTuple

from dimos.memory2.puremodule2 import (
    In,
    Out,
    PureModule,
    contract,
    interpolate,
    latest,
    resource,
    tick,
)

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.geometry_msgs.Transform import Transform
    from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


# ══════════════════════════════════════════════════════════════════════════════
# 1. Tagger — the common case, at cost zero: multi-in → single-out, stateless,
#    one expression. pose is REQUIRED (non-Optional) so there is no guard — the
#    engine skips ticks until pose history exists; image fires, pose lerps to
#    its timestamp. No loop, no yield, no handle. The return annotation
#    documents cadence: `-> TaggerOut` says this module emits every tick.
# ══════════════════════════════════════════════════════════════════════════════
class TaggerIn(In):
    image: Image = tick(expect_hz=30)
    pose: PoseStamped = interpolate()


class TaggerOut(Out):
    located: str = contract(min_hz=10)


class Tagger(PureModule[TaggerIn, TaggerOut]):
    def step(self, i: TaggerIn) -> TaggerOut:
        return TaggerOut(
            located=f"bright={i.image.brightness:.2f} @ ({i.pose.position.x:.2f}, {i.pose.position.y:.2f})"
        )


# ══════════════════════════════════════════════════════════════════════════════
# 2. QualityGate — single-in → multi-out, sparse, as ONE constructor call.
#    Required-vs-sparse is a property of the Out type: forgetting `quality` is a
#    mypy error; `alert` defaults to None = that port stays silent this tick.
# ══════════════════════════════════════════════════════════════════════════════
class QualityIn(In):
    image: Image = tick(expect_hz=30)


class QualityOut(Out):
    quality: float = contract(min_hz=10)
    alert: str | None = None  # sparse — silent unless set


class QualityGate(PureModule[QualityIn, QualityOut]):
    def step(self, i: QualityIn) -> QualityOut:
        b = i.image.brightness
        return QualityOut(
            quality=round(b, 3),
            alert=f"dark frame ({b:.2f})" if b < 0.42 else None,
        )


# ══════════════════════════════════════════════════════════════════════════════
# 3. VoxelGridMapper — resource + State. Sketch 1's fold-with-resource, split
#    along the doctrine line: the grid is a RESOURCE (created at warmup,
#    disposed at stop, under over() too; outside a run it's a lazy cached
#    property — see _unit_tests). The emit cadence is STATE — a serializable
#    counter, so the module's position survives a checkpoint even though the
#    grid itself only rebuilds by replay. Still single-in/out, still a pipeline
#    transform.
# ══════════════════════════════════════════════════════════════════════════════
class VoxelGrid:  # placeholder heavyweight resource
    def __init__(self, voxel_size: float, carve_columns: bool = False) -> None: ...
    def add_frame(self, scan: PointCloud2) -> None: ...
    def get_global_pointcloud2(self) -> PointCloud2: ...
    def dispose(self) -> None: ...


class VoxelIn(In):
    lidar: PointCloud2 = tick()


class VoxelOut(Out):
    global_map: PointCloud2 = contract(min_hz=1)


class VoxelGridMapper(PureModule[VoxelIn, VoxelOut]):
    voxel_size: float = 0.05
    emit_every: int = 5

    @resource
    def grid(self) -> VoxelGrid:  # engine calls .dispose() at teardown
        return VoxelGrid(voxel_size=self.voxel_size, carve_columns=True)

    class State(NamedTuple):
        n: int = 0  # ticks folded so far

    def step(self, state: State, i: VoxelIn) -> tuple[State, VoxelOut | None]:
        self.grid.add_frame(i.lidar)
        state = state._replace(n=state.n + 1)

        if state.n % self.emit_every:
            return state, None

        return state, VoxelOut(global_map=self.grid.get_global_pointcloud2())


# ══════════════════════════════════════════════════════════════════════════════
# 4. CostMapper — multi-in, optional input. relocalized_map may be None until
#    the relocalizer speaks; `default=None` is both the sampler's fallback and
#    the constructor default (tests can omit the field). The resource-free,
#    state-free multi-input shape stays a two-liner.
# ══════════════════════════════════════════════════════════════════════════════
class CostIn(In):
    global_map: PointCloud2 = tick()
    relocalized_map: PointCloud2 | None = latest(default=None)


class CostOut(Out):
    global_costmap: OccupancyGrid = contract(min_hz=2)


class CostMapper(PureModule[CostIn, CostOut]):
    algo: str = "height_cost"

    def step(self, i: CostIn) -> CostOut:
        return CostOut(global_costmap=self._costmap(i.relocalized_map or i.global_map))

    def _costmap(self, msg: PointCloud2) -> OccupancyGrid: ...


# ══════════════════════════════════════════════════════════════════════════════
# 5. RelocalizationModule — Mealy + resource + effects-as-outputs. Sketch 1's
#    locals (world_to_map, last_reloc_ts) become State: plain data, so a run
#    can checkpoint and resume WITH its fix. The premap prologue becomes a
#    resource. The TF broadcast becomes the tf_world_map port — wired live in
#    _effects_wired_live(), inert under over(). The reloc throttle reads off
#    i.ts exactly as before.
# ══════════════════════════════════════════════════════════════════════════════
RELOC_INTERVAL = 2.0
MIN_LOCAL_POINTS = 50_000


class RelocIn(In):
    global_map: PointCloud2 = tick()


class RelocOut(Out):
    relocalized_map: PointCloud2 = contract(min_hz=1)
    tf_world_map: Transform | None = None  # sketch 1's side effect, now a port
    loaded_map: PointCloud2 | None = None  # sparse — viz only


class RelocalizationModule(PureModule[RelocIn, RelocOut]):
    map_file: str | None = None
    publish_loaded_map: bool = False

    @resource
    def premap(self) -> PointCloud2 | None:  # warmup — read the prebuilt map
        return self._load_premap()

    class State(NamedTuple):
        world_to_map: Transform | None = None
        last_reloc_ts: float = float("-inf")

    def step(self, state: State, i: RelocIn) -> tuple[State, RelocOut | None]:
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
        return state, RelocOut(
            relocalized_map=self._merge(scan, state.world_to_map, self.premap),
            tf_world_map=state.world_to_map.now(),
            loaded_map=self.premap if self.publish_loaded_map else None,
        )

    def _load_premap(self) -> PointCloud2 | None: ...
    def _try_relocalize(self, scan: PointCloud2) -> Transform | None: ...
    def _merge(self, scan: PointCloud2, tf: Transform, premap: PointCloud2) -> PointCloud2: ...


# ══════════════════════════════════════════════════════════════════════════════
# 6. Captioner — ASYNC step. `async def step` declares I/O concurrency; the
#    engine runs up to max_inflight ticks at once on the module's loop, emits
#    in TICK ORDER by default (the output stream stays ts-monotonic; an
#    as-completed policy would be an explicit knob), drains or cancels inflight
#    work on stop(), awaits the client's aclose(), and health-counts a failed
#    step as a dropped tick. Requires a stateless module. Compare sketch 1: the
#    task-set / wait / gather bookkeeping is gone, and its bugs went with it.
# ══════════════════════════════════════════════════════════════════════════════
class VLMClient:  # placeholder async I/O client
    async def caption(self, image: Image) -> str: ...
    async def aclose(self) -> None: ...


class CaptionIn(In):
    image: Image = tick(expect_hz=2)


class CaptionOut(Out):
    caption: str = contract(min_hz=1)


class Captioner(PureModule[CaptionIn, CaptionOut]):
    max_inflight: int = 4

    @resource
    def client(self) -> VLMClient:  # engine awaits aclose() at stop
        return VLMClient()

    async def step(self, i: CaptionIn) -> CaptionOut:
        return CaptionOut(caption=await self.client.caption(i.image))


# ══════════════════════════════════════════════════════════════════════════════
# 7. fold — the escape hatch. For the rare module whose control flow doesn't
#    fit one-tick-in / one-bundle-out: cross-tick joins, adaptive batching,
#    custom drains. Rows in, stamped rows out — fold sets ts on each row it
#    emits: the freedom, and the responsibility. A module defines step()
#    XOR fold() — both is a plan-time error; internally the engine's step
#    driver IS a fold, so there is one runtime and two front doors. What fold
#    gives up, stated: state lives in generator locals → no checkpoint/resume,
#    replay from t0 only; internal buffering is invisible to backpressure until
#    an output contract trips. This batch buffer is exactly why fold exists —
#    bulk data (wrong for State) driving control flow (wrong for a resource).
# ══════════════════════════════════════════════════════════════════════════════
class ScanBatcher(PureModule[VoxelIn, VoxelOut]):
    batch_size: int = 8

    def fold(self, rows: Iterator[VoxelIn]) -> Iterator[VoxelOut]:
        batch: list[PointCloud2] = []
        for r in rows:
            batch.append(r.lidar)
            if len(batch) == self.batch_size:
                yield VoxelOut(ts=r.ts, global_map=self._concat(batch))
                batch.clear()

    def _concat(self, scans: list[PointCloud2]) -> PointCloud2: ...


# Deliberately left out: a @pure_module function decorator and a bare-return
# sugar for single-output modules — each is a second spelling of an existing
# form, and the ceremony it would shave is one class line / one constructor
# name. One spelling per capability.


# ══════════════════════════════════════════════════════════════════════════════
# TESTS NEED NO ENGINE — bundles are real dataclasses and step is a function.
# Construct a row, call step, assert on the bundle. Mealy modules thread state
# explicitly, so a cadence property is provable in two lines. The grid resource
# materializes lazily on first touch — no warmup() in sight.
# ══════════════════════════════════════════════════════════════════════════════
def _fake_image(brightness: float) -> Image: ...
def _fake_pose(x: float, y: float) -> PoseStamped: ...
def _fake_scan() -> PointCloud2: ...


def _unit_tests() -> None:
    out = Tagger().step(TaggerIn(ts=0.0, image=_fake_image(0.80), pose=_fake_pose(1.0, 2.0)))
    assert out.located == "bright=0.80 @ (1.00, 2.00)"

    m = VoxelGridMapper(emit_every=2)
    state, o1 = m.step(VoxelGridMapper.State(), VoxelIn(ts=0.0, lidar=_fake_scan()))
    state, o2 = m.step(state, VoxelIn(ts=0.1, lidar=_fake_scan()))
    assert o1 is None and o2 is not None


# ══════════════════════════════════════════════════════════════════════════════
# DEPLOYMENT — sketch 1's three modes carry over; two spellings are fixed:
#   • ONE way to run offline: configure, then over() — an instance method; the
#     class-or-instance dual dispatch is gone.
#   • ports live under m.i / m.o (the blueprint direction:
#     bp.connect(cam.o, nav.i)) — In and Out may share a field name, and
#     sources / transports / subscribers hang off the same runtime-typed
#     handles (payload typing lives in the bundles).
# Transform equivalence is unchanged from sketch 1: a single-input module IS a
# Transformer; multi-input modules use over().
# ══════════════════════════════════════════════════════════════════════════════
def _offline_over() -> None:
    from dimos.memory2.store.sqlite import SqliteStore
    from dimos.utils.data import get_data

    store = SqliteStore(path=get_data("go2_run.db"))
    costmaps = CostMapper(algo="height_cost").over(
        global_map=store.streams.global_map,
        relocalized_map=store.streams.relocalized_map,
    )
    for row in costmaps:  # CostOut rows, tick-ordered, row.ts = tick time
        print(row.global_costmap)


def _as_transform() -> None:
    from dimos.memory2.store.sqlite import SqliteStore

    store = SqliteStore(path="walk.db")
    maps = store.streams.lidar.transform(VoxelGridMapper(voxel_size=0.1))
    maps.map(lambda row: row.global_map).save(store.streams.global_map).drain()  # stamped msgs out


def _live_with_transports() -> None:
    from dimos.core.transport import pLCMTransport

    m = CostMapper(algo="height_cost")
    m.i.global_map.transport = pLCMTransport("/map/global")
    m.i.relocalized_map.transport = pLCMTransport("/map/relocalized")
    m.o.global_costmap.transport = pLCMTransport("/costmap/global")
    m.o.global_costmap.subscribe(lambda grid: ...)
    m.warmup()  # resources created, parks on empty buffers
    m.start()  # ports open → ticks → step
    m.stop()  # drain, dispose resources — the same teardown over() runs at exhaustion


def _effects_wired_live() -> None:
    from dimos.core.transport import pLCMTransport

    # sketch 1's side effect, deployed: the TF fix drives the tree only when wired
    m = RelocalizationModule(map_file="office_map", publish_loaded_map=True)
    m.i.global_map.transport = pLCMTransport("/map/global")
    m.o.relocalized_map.transport = pLCMTransport("/map/relocalized")
    m.o.loaded_map.transport = pLCMTransport("/map/loaded")
    m.o.tf_world_map.subscribe(lambda tf: ...)  # e.g. the TF broadcaster
    m.warmup()
    m.start()


def _live_from_recording() -> None:
    from dimos.memory2.store.sqlite import SqliteStore

    # the live↔stored switch, per port: set sources before start(). No transports.
    store = SqliteStore(path="go2_run.db")
    m = CostMapper()
    m.i.global_map.source = store.replay(speed=2.0).streams.global_map
    m.i.relocalized_map.source = store.streams.relocalized_map
    m.o.global_costmap.subscribe(lambda grid: ...)
    m.warmup()
    m.start()


def _checkpoint_resume() -> None:
    # what engine-owned State buys — impossible while state was a generator frame:
    m = RelocalizationModule(map_file="office_map")
    m.warmup()
    m.start()
    snap = m.checkpoint()  # State + tick cursor, plain data
    m.stop()

    m2 = RelocalizationModule(map_file="office_map")
    m2.restore(snap)  # resumes with the reloc fix already in hand
    m2.warmup()
    m2.start()
    m2.start()
