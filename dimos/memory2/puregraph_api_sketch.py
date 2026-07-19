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

# SKETCH — not functional. The GRAPH layer over puremodule_api_sketch2: compose
# pure modules into stacks that are themselves module-shaped. The pure
# counterpart of the blueprint system — what blueprints do with runtime wiring
# and launch config, a graph does with one pure function over typed ports.
#
#   1. APPLICATION is the one composition operator. A configured module (or
#      graph — they compose uniformly) is applied to named ports:
#      `m(x=..., y=...)` returns its Out bundle as typed ports. over() is the
#      same application on real streams; wire() is over() run symbolically.
#      Port[T] advertises T, so the bundle annotations still typecheck.
#   2. wire() is PURE and rerunnable. Module construction is config-only
#      (resources are lazy — sketch 2 §4), so building a graph does nothing:
#      inspect it, diff it, draw it, throw it away. And wire() is plain
#      Python, so parametric DAGs (N cameras → N branches) are a build-time
#      for-loop, not a templating DSL.
#   3. A graph is WIRING, not computation: it has no tick of its own. Samplers
#      and contracts stay on member ports; graph In/Out bundles carry BARE
#      types (a sampler on a graph field is a plan error; exported outputs
#      inherit the producing member's contract). Boundaries are free — nesting
#      flattens at build; member names namespace by nesting path
#      (patrol.nav.voxel_grid_mapper), the pure analogue of blueprint
#      namespaces.
#   4. EDGES are streams. Fan-out is legal — the runtime tees: in-process
#      subscription live, materialized through the run store offline. Any edge
#      can get a transport (cross-process split, wire tap) or be recorded,
#      without touching wire().
#   5. CYCLES are legal only through a sampled non-trigger input
#      (latest/interpolate): the back edge reads strictly-earlier data, so the
#      loop is well-founded by timestamp and replays identically offline. A
#      tick() back edge is a plan error (it would define time by itself).
#      feedback() declares the back edge before its producer exists; every
#      feedback must be closed exactly once.
#   6. The ENGINE owns group lifecycle: warmup/start in topological order,
#      stop/drain in reverse; g.checkpoint() = {member_path: State} (resources
#      rebuild by replay — sketch 2 doctrine); g.health aggregates members;
#      unexported member outputs stay reachable by path for taps, transports,
#      and recording.
#   7. Wiring is validated at BUILD: unknown application kwarg, unbound
#      required input, port/field type mismatch, tick-cycle, unclosed
#      feedback, unset Out field — all before anything runs. The graph
#      equivalent of mypy for wiring.

from __future__ import annotations

from collections.abc import Callable
import math
from typing import TYPE_CHECKING, NamedTuple, Protocol, TypeVar

from dimos.memory2.puregraph import Port, PureGraph, feedback
from dimos.memory2.puremodule2 import (
    In,
    Out,
    PureModule,
    contract,
    interpolate,
    latest,
    tick,
)
from dimos.memory2.puremodule_api_sketch2 import (
    CostMapper,
    RelocalizationModule,
    VoxelGridMapper,
)
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

if TYPE_CHECKING:
    from reactivex.disposable import Disposable

    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
    from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid


# ══════════════════════════════════════════════════════════════════════════════
# Planner — one more member module (sketch 2 shapes throughout). prev_cmd is a
# sampled OPTIONAL input so it can legally close a cycle: the planner slew-
# limits against the command it emitted on an earlier tick. Port payloads are
# STAMPED (sketch 2 §7) — Twist has no ts (ROS legacy), so the port speaks
# TwistStamped.
# ══════════════════════════════════════════════════════════════════════════════
class PlannerIn(In):
    global_costmap: OccupancyGrid = tick()
    goal: PoseStamped = interpolate()
    prev_cmd: TwistStamped | None = latest(default=None)  # back edge lands here


class PlannerOut(Out):
    cmd_vel: TwistStamped = contract(min_hz=2)


class Planner(PureModule[PlannerIn, PlannerOut]):
    max_accel: float = 0.5

    def step(self, i: PlannerIn) -> PlannerOut:
        cmd = self._toward(i.global_costmap, i.goal)
        if i.prev_cmd is not None:
            cmd = self._slew(i.prev_cmd, cmd, self.max_accel)
        return PlannerOut(cmd_vel=cmd)

    def _toward(self, costmap: OccupancyGrid, goal: PoseStamped) -> TwistStamped: ...
    def _slew(self, prev: TwistStamped, cmd: TwistStamped, max_accel: float) -> TwistStamped: ...


# ══════════════════════════════════════════════════════════════════════════════
# NavStack — a graph. wire() runs ONCE at build, on typed port refs: i.lidar is
# a Port[PointCloud2] standing in for the payload. Each application returns the
# member's Out bundle as ports; constructing NavOut exports ports — the same
# gesture as step constructing its output, one level up. gm.global_map is used
# twice (fan-out, §4); the planner's self-loop goes through feedback() (§5).
# reloc's tf_world_map / loaded_map aren't exported — not lost: reachable by
# path ("nav.relocalization_module.tf_world_map") for taps and transports.
# ══════════════════════════════════════════════════════════════════════════════
class NavIn(In):
    lidar: PointCloud2
    goal: PoseStamped


class NavOut(Out):
    global_costmap: OccupancyGrid
    cmd_vel: TwistStamped


class NavStack(PureGraph[NavIn, NavOut]):
    map_file: str | None = None  # graph config flows into member config

    def wire(self, i: NavIn) -> NavOut:
        voxel = VoxelGridMapper(voxel_size=0.05)
        reloc = RelocalizationModule(map_file=self.map_file)
        cost = CostMapper()
        plan = Planner()

        gm = voxel(lidar=i.lidar)
        rm = reloc(global_map=gm.global_map)
        cm = cost(global_map=gm.global_map, relocalized_map=rm.relocalized_map)

        prev: Port[TwistStamped] = feedback()  # declared before its producer exists
        pl = plan(global_costmap=cm.global_costmap, goal=i.goal, prev_cmd=prev)
        prev.close(pl.cmd_vel)  # the loop, well-founded by ts

        return NavOut(global_costmap=cm.global_costmap, cmd_vel=pl.cmd_vel)


# ══════════════════════════════════════════════════════════════════════════════
# WaypointCycler — an ordinary Mealy member (sketch 2 §1/§4): pose paces it,
# State is the waypoint index, the current goal emits every tick.
# ══════════════════════════════════════════════════════════════════════════════
class CyclerIn(In):
    pose: PoseStamped = tick(expect_hz=10)


class CyclerOut(Out):
    goal: PoseStamped = contract(min_hz=1)


class WaypointCycler(PureModule[CyclerIn, CyclerOut]):
    waypoints: tuple[tuple[float, float], ...] = ((0.0, 0.0), (4.0, 0.0), (4.0, 3.0))
    arrive_radius: float = 0.3

    class State(NamedTuple):
        idx: int = 0

    def step(self, state: State, i: CyclerIn) -> tuple[State, CyclerOut]:
        wx, wy = self.waypoints[state.idx]
        if math.hypot(i.pose.position.x - wx, i.pose.position.y - wy) < self.arrive_radius:
            state = state._replace(idx=(state.idx + 1) % len(self.waypoints))
        return state, CyclerOut(goal=self._as_pose(self.waypoints[state.idx]))

    def _as_pose(self, xy: tuple[float, float]) -> PoseStamped: ...


# ══════════════════════════════════════════════════════════════════════════════
# Patrol — NESTING. A graph applies exactly like a module (§1): NavStack is one
# node here. i.lidar fans out to nobody else and i.pose paces the cycler; the
# flattened DAG namespaces members by path: patrol.waypoint_cycler,
# patrol.nav.voxel_grid_mapper, ... (§3).
# ══════════════════════════════════════════════════════════════════════════════
class PatrolIn(In):
    lidar: PointCloud2
    pose: PoseStamped


class PatrolOut(Out):
    cmd_vel: TwistStamped


class Patrol(PureGraph[PatrolIn, PatrolOut]):
    map_file: str | None = None

    def wire(self, i: PatrolIn) -> PatrolOut:
        cyc = WaypointCycler()(pose=i.pose)
        nav = NavStack(map_file=self.map_file)(lidar=i.lidar, goal=cyc.goal)
        return PatrolOut(cmd_vel=nav.cmd_vel)


# ══════════════════════════════════════════════════════════════════════════════
# DEPLOYMENT — a graph deploys exactly like a module, because it is one from
# the outside: g.i / g.o are the synthesized group ports (the unbound inputs
# and exported outputs); over() runs the whole DAG offline; warmup/start/stop
# run it live. Everything below works on any nesting depth.
# ══════════════════════════════════════════════════════════════════════════════
def _offline_over() -> None:
    from dimos.memory2.store.sqlite import SqliteStore
    from dimos.utils.data import get_data

    # pull-based, deterministic, no threads: the DAG evaluates through a run
    # store, so every edge — including internal ones — is re-iterable and
    # queryable afterwards. The run handle exposes exported outputs as streams.
    store = SqliteStore(path=get_data("go2_run.db"))
    run = Patrol(map_file="office_map").over(lidar=store.streams.lidar, pose=store.streams.pose)
    cmds = run.cmd_vel.to_list()
    maps = run.stream("nav.voxel_grid_mapper.global_map").to_list()  # internal tap
    _use(cmds, maps)


def _live() -> None:
    from dimos.core.transport import pLCMTransport

    # transports live only at the rim: unbound inputs and exported outputs.
    # Internal edges are in-process handoffs — no serialization, no topics.
    g = Patrol(map_file="office_map")
    g.i.lidar.transport = pLCMTransport("/lidar")
    g.i.pose.transport = pLCMTransport("/pose")
    g.o.cmd_vel.transport = pLCMTransport("/cmd_vel")
    g.warmup()  # members warm up in topological order
    g.start()
    g.stop()  # reverse-topological drain: downstream sees every upstream tail


def _split_and_record() -> None:
    from dimos.core.transport import pLCMTransport
    from dimos.memory2.store.sqlite import SqliteStore

    g = Patrol(map_file="office_map")
    # externalize ONE internal edge by path — a wire tap or a future process
    # split point — without touching wire():
    g.edge("nav.voxel_grid_mapper.global_map").transport = pLCMTransport("/map/global")
    # flight recorder: give the graph a store and every edge records as a side
    # effect of running (sketch 2's make_store, one level up):
    g.store = SqliteStore(path="patrol_flight.db")
    g.warmup()
    g.start()
    g.stop()

    # afterwards: re-run any single member over the recording to debug it in
    # isolation — the graph recorded that member's exact inputs.
    rec = SqliteStore(path="patrol_flight.db")
    cm = CostMapper().over(
        global_map=rec.stream("nav.voxel_grid_mapper.global_map"),
        relocalized_map=rec.stream("nav.relocalization_module.relocalized_map"),
    )
    _use(cm.to_list())


def _checkpoint_resume() -> None:
    # group checkpoint = {member_path: State} — plain data, because every
    # member's state is plain data. Resources rebuild at warmup (§6).
    g = Patrol(map_file="office_map")
    g.warmup()
    g.start()
    snap = g.checkpoint()  # {"waypoint_cycler": State(idx=2), "nav.relocalization_module": ...}
    g.stop()

    g2 = Patrol(map_file="office_map")
    g2.restore(snap)  # resumes mid-patrol, reloc fix in hand
    g2.warmup()
    g2.start()


# ══════════════════════════════════════════════════════════════════════════════
# TRANSPORTS — writing a new wire protocol. A transport is deliberately dumb:
# it moves STAMPED payloads between a topic and callbacks — plain msgs, ts
# inside the payload (sketch 2 §7), the same currency the legacy Module stack
# speaks, so pure rims interoperate with existing nodes for free. It never
# sees samplers, ticks, or contracts (alignment is the consumer's input
# declaration), never drops or reorders by policy (the engine's buffers own
# backpressure), and may deliver on any thread (the engine marshals onto the
# module's loop). codec(typ) is the same codec layer the stores use, so a
# transport never invents serialization; primitive ports (str/float) ride the
# Stamped[T] envelope msg at the wire. Anything satisfying the protocol plugs
# in wherever a transport goes: rim ports, bind() below, g.edge()
# externalization.
# ══════════════════════════════════════════════════════════════════════════════
class Stamped(Protocol):
    ts: float  # the one timestamp: stamped msgs and In/Out rows both qualify


T = TypeVar("T", bound=Stamped)


class Channel(Protocol[T]):
    def publish(self, msg: T) -> None: ...
    def subscribe(self, on_msg: Callable[[T], None]) -> Disposable: ...
    def close(self) -> None: ...  # idempotent; stop() closes rims in reverse-topo order


class Transport(Protocol):
    def channel(self, topic: str, typ: type[T]) -> Channel[T]: ...


class ZenohTransport:
    """A second protocol, complete: dimos codecs make the bytes, zenoh moves
    them — ts travels inside the message, nothing extra to envelope."""

    def __init__(self, config: str | None = None) -> None:
        self._config = config  # sessions open at first channel — construction stays pure

    def channel(self, topic: str, typ: type[T]) -> Channel[T]:
        # publish   = codec(typ).encode(msg) → session.put(key)   (ts is in the bytes)
        # subscribe = zenoh handler thread → decode → on_msg(msg)
        #             (zenoh's thread is fine — the engine marshals, per the banner)
        ...


# ══════════════════════════════════════════════════════════════════════════════
# MULTI-INSTANCE — graphs are values, so a fleet is a loop. bind() assigns the
# whole rim at once: it takes a channel factory (topic → channel) or a
# Transport (bind knows each port's type), and topic naming is a pure function
# port-path → topic — default "/" + name, prefix= closes over it, topics=
# overrides individual ports. Together that is the pure counterpart of
# blueprint namespaces and remappings: no registry, no globals, just a naming
# function applied at bind time. Each instance owns its DAG, state, health,
# and checkpoint. (bind() exists on plain modules too — a module is the
# one-node graph.) Open-ended fleets coordinate over the rim — a coordinator
# graph binds the same topics — not inside wire().
# ══════════════════════════════════════════════════════════════════════════════
def _fleet() -> None:
    from dimos.core.transport import pLCMTransport

    robots: dict[str, Patrol] = {}
    for name in ("go2a", "go2b"):
        g = Patrol(map_file="office_map")
        g.bind(pLCMTransport, prefix=f"/{name}")  # /go2a/lidar, /go2a/pose, /go2a/cmd_vel
        g.warmup()
        g.start()
        robots[name] = g

    snaps = {name: g.checkpoint() for name, g in robots.items()}  # per-robot, for free
    _use(robots, snaps)


def _mixed_protocols_and_remaps() -> None:
    from dimos.core.transport import pLCMTransport

    g = Patrol(map_file="office_map")
    # a remap is a dict entry, not a DSL:
    g.bind(pLCMTransport, prefix="/go2a", topics={"lidar": "/go2a/ouster/points"})
    # a per-port assignment wins over bind — fat pointclouds ride zenoh
    # shared-memory while everything else stays on LCM:
    g.i.lidar.transport = ZenohTransport().channel("/go2a/ouster/points", PointCloud2)
    g.warmup()
    g.start()


def _use(*results: object) -> None: ...


# Deliberately left out: a @graph function decorator (second spelling of
# wire()); a remapping DSL (wiring remaps are application kwargs, topic remaps
# are bind()'s naming function); process / host placement (deployment config
# layered on paths, not wiring); runtime rewiring (a graph is static per build
# — rebuild with different config to get a different graph, it costs nothing).
