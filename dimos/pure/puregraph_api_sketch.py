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

# SKETCH — not functional. The GRAPH layer over the landed pure engine (T1-T8,
# T11). Compose pure modules into stacks that are THEMSELVES module-shaped —
# the pure counterpart of the blueprint system: what blueprints do with runtime
# wiring, a graph does with one pure function over typed ports. Written against
# the real `from dimos import pure as pm` surface and the landed nav modules;
# `dimos.pure.graph` (PureGraph / Port / feedback) is the thing being designed.
#
#  1. APPLICATION is the one operator. A configured module — or graph, they
#     compose uniformly — applied to named ports `m(x=..., y=...)` returns its
#     Out bundle as typed port refs. over() is that same application on real
#     streams; wire() is over() run SYMBOLICALLY (i.x is a Port[T] standing in
#     for the payload, so the bundle annotations still typecheck). One gesture,
#     two modes.
#
#  2. wire() is PURE and rerunnable. Module construction is config-only
#     (resources are lazy — T7), so building a graph does NOTHING: inspect it,
#     diff it, draw it, throw it away. wire() is plain Python, so a parametric
#     DAG (N sensors -> N branches) is a build-time for-loop, not a templating
#     DSL.
#
#  3. A graph is WIRING, not computation — it has no tick of its own. Samplers
#     (pm.tick/latest/interpolate/tf) and contracts live on MEMBER In ports;
#     graph In/Out bundles carry BARE types (a sampler on a graph field is a
#     plan error; an exported Out inherits its producing member's contract).
#     Nesting flattens at build; members namespace by path
#     (nav.voxel_mapper.global_map) — the pure analogue of blueprint namespaces.
#
#  4. EDGES are streams of .data-only stamped msgs — the SAME currency a live
#     transport delivers, so the identical graph eats a recording or a live
#     topic with no difference (the boundary is `.data`: pose/tags are mem2-side
#     metadata the pure layer never sees). Interior edges are in-process
#     handoffs by default; fan-out is legal (the runtime tees). Any edge can get
#     a transport (a cross-process cut, a wire tap) or be recorded, WITHOUT
#     touching wire().
#
#  5. A SOURCE is a value, and polymorphic: any port — a rim input OR an
#     interior edge — can be sourced by an upstream member, a live transport, or
#     a recorded stream, interchangeably, in the same slot. This is the whole
#     trick: connect-to-robot, feed-from-db, and replay-injection-into-the-
#     middle-of-the-graph are ONE api (§DEPLOY / §SOURCES). Overriding an
#     interior port with a replay is the deliberate escape hatch — an interior
#     edge is not sacrosanct; you may sever its real producer and substitute a
#     stream to develop the downstream against a recording.
#
#  6. CYCLES are legal only through a SAMPLED non-trigger input
#     (latest/interpolate/tf): the back edge reads strictly-earlier data, so the
#     loop is well-founded by timestamp and replays identically offline. A
#     tick() back edge is a plan error (it would define time by itself).
#     feedback() names the back edge before its producer exists; every feedback
#     is closed exactly once.
#
#  7. ONE aligner. A member's In row is resolved from its inbound edges by the
#     SAME T5 aligner over() already uses — the graph does not introduce a
#     second alignment path. (mem2's weaker Stream.align(tolerance=) is the same
#     problem solved worse; long-term it folds into this engine, one join with
#     two front doors.)
#
#  8. Wiring is validated at BUILD, before anything runs: unknown application
#     kwarg, unbound required input, port/field type mismatch, tick-cycle,
#     unclosed feedback, unset Out field. The graph equivalent of mypy for
#     wiring.
#
# DEPLOYMENT — one wire(), three targets (§DEPLOY):
#   • LOCAL single-process — over() runs the whole DAG in one process, lazy
#     pull, in-process handoffs, every edge re-iterable through a run store.
#     The native home now: dev, replay, offline batch.
#   • MULTIPROCESS TODAY via the existing system — each member is already a
#     legacy-compatible Module (the T8 bridge), so graph.blueprint() LOWERS the
#     explicit edges into autoconnect wiring: one process per member (or
#     partition), topics named by the autoconnect CONVENTION, name-crossing
#     edges emitted as remappings. You keep chucking pure modules into
#     blueprints/coordinator — but the graph GENERATES that wiring instead of
#     you hand-writing it.
#   • MULTIPROCESS FUTURE, our own — the graph itself becomes the distributed
#     unit: pick cut edges, put a pubsub transport on each, the graph runtime
#     places subgraphs across processes/hosts. Same wire(); deployment config
#     layered on PATHS, not wiring. Eventually supersedes the blueprint path for
#     pure stacks.
#
# Kept from legacy autoconnect: the naming CONVENTION (port name -> topic,
# match by name+type) as the rim's default, so pure rims interoperate on the
# bus with legacy modules for free. Dropped: the autoconnect FUNCTION (a flat
# implicit matcher with no addressable interior and random-topic collisions) —
# replaced by the explicit, inspectable, injection-capable graph.

from __future__ import annotations

from typing import TYPE_CHECKING

from dimos import pure as pm
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.pure.graph import Port, PureGraph, feedback
from dimos.pure.modules.costmapper import CostMapper
from dimos.pure.modules.odom_tf import OdomTf
from dimos.pure.modules.planner import Planner
from dimos.pure.modules.voxel_mapper import VoxelMapper

if TYPE_CHECKING:
    from dimos.pure.transport import Transport

WORLD = "world"


# ── A graph. wire() runs ONCE at build on typed port refs: i.lidar is a
# Port[PointCloud2] standing in for the payload. Each application returns the
# member's Out bundle as ports; constructing NavStack.Out EXPORTS ports — the
# same gesture as a step constructing its Out, one level up. cost.global_costmap
# is used twice (fan-out, §4). The cost->plan edge crosses a name boundary
# (global_costmap -> costmap): explicit here, a remapping when lowered to
# blueprints (§DEPLOY). odom_tf.tf / reloc outputs left unexported are NOT lost
# — reachable by path for taps, transports, recording (§4).
class NavStack(PureGraph):
    """Local pure nav DAG: hk lidar+odom -> map -> costmap -> plan, goal at origin."""

    voxel_size: float = 0.1  # graph config flows into member config (§3)

    class In(pm.In):
        lidar: PointCloud2  # bare types on graph ports — samplers live on members (§3)
        odom: PoseStamped
        goal: PoseStamped

    class Out(pm.Out):
        path: Path
        costmap: OccupancyGrid

    def wire(self, i: In) -> Out:
        """Symbolic application: bind each member input to a producer's output port."""
        odom_tf = OdomTf()(odom=i.odom)  # -> .tf : Transform
        voxel = VoxelMapper(voxel_size=self.voxel_size)(scan=i.lidar)  # -> .global_map
        cost = CostMapper()(global_map=voxel.global_map)  # -> .global_costmap
        plan = Planner()(  # name-crossing edge: cost.global_costmap -> planner.costmap
            costmap=cost.global_costmap, goal=i.goal, tf=odom_tf.tf
        )  # -> .path : Path
        return NavStack.Out(path=plan.path, costmap=cost.global_costmap)  # fan-out on cost


# ── FEEDBACK (§6). A back edge is legal only into a SAMPLED input. The planner
# slew-limits against the command it emitted an earlier tick: prev_cmd is a
# latest() port, so the loop reads strictly-earlier data — well-founded by ts,
# identical offline. feedback() names the edge before plan exists; close() ties
# it once. (Illustrative: SlewPlanner is a member with a prev_cmd latest input.)
class SlewNav(PureGraph):
    """A nav graph whose planner feeds its own previous command back through latest()."""

    class In(pm.In):
        lidar: PointCloud2
        goal: PoseStamped

    class Out(pm.Out):
        cmd_vel: TwistStamped

    def wire(self, i: In) -> Out:
        """Close the planner's self-loop through a sampled (latest) back edge."""
        from dimos.pure.modules.planner import SlewPlanner  # sketch: a prev_cmd variant

        voxel = VoxelMapper()(scan=i.lidar)
        cost = CostMapper()(global_map=voxel.global_map)
        prev: Port[TwistStamped] = feedback()  # declared before its producer
        plan = SlewPlanner()(costmap=cost.global_costmap, goal=i.goal, prev_cmd=prev)
        prev.close(plan.cmd_vel)  # the loop, well-founded by ts
        return SlewNav.Out(cmd_vel=plan.cmd_vel)


# ── NESTING (§3). A graph applies exactly like a module. Patrol drives NavStack
# as one node; the flattened DAG namespaces members by path
# (patrol.nav.voxel_mapper, ...). A goal source (a waypoint cycler, say) would
# pace it — omitted for brevity.
class Patrol(PureGraph):
    """Nesting: NavStack as a single node inside a larger graph."""

    class In(pm.In):
        lidar: PointCloud2
        odom: PoseStamped
        goal: PoseStamped

    class Out(pm.Out):
        path: Path

    def wire(self, i: In) -> Out:
        """A graph is a member: apply NavStack, re-export one of its outputs."""
        nav = NavStack(voxel_size=0.05)(lidar=i.lidar, odom=i.odom, goal=i.goal)
        return Patrol.Out(path=nav.path)


# ══ DEPLOY — one wire(), three targets ═══════════════════════════════════════


def _local_single_process() -> None:
    """Native home: over() runs the whole DAG in ONE process, lazy pull."""
    from dimos.memory2.store.sqlite import SqliteStore
    from dimos.utils.data import get_data

    goal = PoseStamped(frame_id=WORLD, position=[0.0, 0.0, 0.0])
    goal.ts = 1.0  # below every recording ts, so latest() sees it from tick 1

    with SqliteStore(path=str(get_data("go2_hongkong_office.db"))) as store:
        # store channels -> rim In ports BY NAME (.data-only), the offline dual of
        # autoconnect; remap= handles the exceptions. goal isn't a channel -> pass it.
        run = NavStack().over(store, goal=[goal])
        paths = run.path.to_list()  # an exported output, as a stream
        # every interior edge is addressable by path and re-iterable afterward:
        maps = run.stream("voxel_mapper.global_map").to_list()  # interior tap
        _use(paths, maps)


def _multiprocess_today() -> None:
    """Deploy across processes via the EXISTING system — the graph lowers to blueprints."""
    from dimos.core.coordination.module_coordinator import ModuleCoordinator

    # Each member is a legacy-compatible Module (T8 bridge), so blueprint() emits the
    # autoconnect wiring FROM the explicit edges: one process per member, topics by the
    # naming convention, the cost.global_costmap -> planner.costmap edge as a remapping.
    # wire() authored once; this is a compile target, not a rewrite.
    coordinator = ModuleCoordinator.build(NavStack.blueprint(voxel_size=0.1))
    coordinator.loop()  # runs distributed across the worker pool, on the shared bus


def _multiprocess_future() -> None:
    """Our OWN distributed runtime: partition the graph, put pubsub on the cut edges."""
    from dimos.pure.transport import ZenohTransport

    # Pick cut edges; the graph runtime places each subgraph in its own process/host and
    # the cut edges ride pubsub. Everything else stays in-process. Same wire(); placement
    # is config over PATHS, not wiring. Supersedes the blueprint path for pure stacks.
    deployment = NavStack().partition(
        sensing=["voxel_mapper", "odom_tf"],
        planning=["cost_mapper", "planner"],
    )
    deployment.bind(ZenohTransport())  # interior cut edges -> topics; rim -> topics
    deployment.warmup()
    deployment.start()
    deployment.stop()  # reverse-topological drain across processes


# ══ SOURCES are values — connect, feed, and inject are one api (§5) ═══════════


def _sources_are_values() -> None:
    """The SAME graph, sourced three ways; the third injects a replay mid-graph."""
    from dimos.memory2.store.sqlite import SqliteStore

    robot: Transport = _robot()  # live transports at the rim
    rec = SqliteStore(path="patrol_flight.db")  # a prior graph recording
    goal = PoseStamped(frame_id=WORLD, position=[0.0, 0.0, 0.0])

    # (a) recorded db at the rim (offline batch / eval):
    with SqliteStore(path="go2_hongkong_office.db") as store:
        NavStack().over(store, goal=[goal]).path.to_list()

    # (b) a live robot at the rim (the case we need first):
    live = NavStack().bind(robot).source(goal=[goal])
    live.warmup()
    live.start()

    # (c) REPLAY INJECTION into an INTERIOR port — develop the planner against a
    # recorded costmap, with NO mapper or costmapper in the loop. The escape hatch
    # of §5: an interior edge's real producer is severed, a recorded stream stands in.
    NavStack().over(
        goal=[goal],
        at={"planner.costmap": rec.stream("nav.cost_mapper.global_costmap")},
    ).path.to_list()


# ══ RECORD — a graph records every edge as a side effect of running ══════════


def _record_then_debug_one_member() -> None:
    """Give the graph a store; every edge records. Later, re-run one member in isolation."""
    from dimos.memory2.store.sqlite import SqliteStore

    robot: Transport = _robot()
    goal = PoseStamped(frame_id=WORLD, position=[0.0, 0.0, 0.0])

    g = NavStack().bind(robot).source(goal=[goal])
    g.store = SqliteStore(path="nav_flight.db")  # flight recorder: interior edges too
    g.warmup()
    g.start()
    g.stop()

    # Afterwards: re-run any single member over the recording of its EXACT inputs —
    # the same over() a graph edge uses, now pointed at the store. This is §5 (c) with
    # the module standing alone instead of inside the graph.
    rec = SqliteStore(path="nav_flight.db")
    replanned = Planner().over(
        costmap=rec.stream("nav.cost_mapper.global_costmap"),
        goal=[goal],
        tf=rec.stream("nav.odom_tf.tf"),
    )
    _use([r.path for r in replanned])


def _robot() -> Transport: ...
def _use(*results: object) -> None: ...


# Deliberately left out, and why:
#   • a @graph function decorator — a second spelling of wire(), not a new idea.
#   • a remapping DSL — wiring remaps are application kwargs; topic remaps are the
#     bind() naming function; both are values, not syntax.
#   • runtime rewiring — a graph is static per build; rebuild with different config
#     to get a different graph, it costs nothing (wire() is pure).
#   • an implicit name+type matcher (legacy autoconnect's core) — deferred until
#     there is demand, and when built it LOWERS to an explicit graph and ERRORS on
#     ambiguity (naming the candidates) rather than minting a random topic.
