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

# SKETCH 3-RPC — not functional. Companion to puremodule_api_sketch3: the
# call layer over pure modules — services, actions, callers, and long-running
# skills — expanding that sketch's RPC note into code. Doctrine:
#
#   1. REQUESTS ARE INPUTS, REPLIES ARE OUTPUTS. An @rpc request is a stamped
#      row merged into tick order by ts (equal ts breaks by envelope sequence
#      number — replay identity needs the total order); the handler is a
#      TRANSITION on the module's own State. So RPC-caused changes record,
#      replay, checkpoint, and appear in the inspector timeline like any
#      tick. Replies are effects → outputs: inert under over(); a replay can
#      never answer a live caller.
#   2. THE SIGNATURE CLASSIFIES THE HANDLER, and only one form freezes
#      anything. Mutating `(self, state, req) -> (state, reply)` serializes
#      with step on the module loop — the module's data-time halts while it
#      runs (ticks queue; overflow is counted drops in health). Read-only
#      `(self, state, req) -> reply` runs OFF-loop on the latest committed
#      State snapshot — State is immutable plain data, so reads cost the
#      module nothing. Stateless `(self, req) -> reply` likewise. Nothing to
#      annotate: __init_subclass__ classifies by shape.
#   3. LIGHTNESS IS ENFORCED, not requested. A mutating handler cannot touch
#      @resource (the engine flags handler context; resource access raises)
#      and cannot be async (plan error) — with all sanctioned I/O behind
#      resources, a handler is structurally a pure function of
#      (config, state, req). Wall cost is a declared, measured contract:
#      @rpc(deadline_ms=...) — over-deadline is a health violation event,
#      visible live and diffable in CI, not a runtime kill.
#   4. SERVICES, NOT ACTIONS. A handler is a ONE-TICK transition:
#      navigate_to -> Ack means ACCEPTED, never arrived. A long-running
#      command is goal-in-State + a status output port; the reply carries the
#      correlation token (goal_id). A ROS action decomposes into exactly
#      this; @rpc is the service half. Preemption is not a mechanism — a new
#      goal bumps goal_id, and step's next status row tells the old goal's
#      awaiters.
#   5. CAPABILITY SLOTS, wiring-supplied. No boot-time attachment, no
#      registry: a caller declares the SMALLEST protocol of calls it needs
#      and the graph supplies the server at application — the server module
#      satisfies the protocol STRUCTURALLY via its @rpc descriptors
#      (verified; protocol members spelled as read-only properties). The
#      capability edge is an application kwarg, so wire() sees it →
#      topological warmup gives the old "attached when the other module
#      boots" guarantee for free. Cross-process, the engine swaps in a
#      transport-backed proxy satisfying the same protocol. Calls from step
#      are declared impurity: the resource bucket, cassette remedy under
#      replay. Rare by doctrine — commands-as-ports stays the default inside
#      graphs.
#   6. ONE FACADE, TWO HOSTS. The action client (ack + status stream
#      composed into a single awaitable that resolves on TERMINAL status) is
#      written once. Hosted at the rim it is plain asyncio — maximally
#      convenient, composes with gather/timeouts, and EPHEMERAL: if the
#      caller's process dies, the await is gone (the robot keeps navigating —
#      the goal is in Navigation's State; only the caller's position in its
#      own plan is lost). Hosted in a FLOW it is durable (§7).
#   7. FLOWS — the sanctioned home for long-running imperative skills. A
#      flow is a coroutine whose awaits are restricted to RECORDED things
#      (action facades, rpc replies, port reads, data-time timers). Every
#      resolved await appends to the flow's JOURNAL — a recorded edge like
#      any other. Recovery is the Temporal trick: re-run the coroutine from
#      the top; awaits return journaled values instantly (no RPCs re-fired,
#      no sleeps re-slept) until the journal is exhausted — the frame is
#      REPLAYED INTO EXISTENCE at the await where it died, then goes live.
#      checkpoint() = journal prefix: plain data. Flows are fold's cousin —
#      control flow owns the loop — but their state is recoverable because
#      the awaits are data.
#
# The costs, priced: (a) a mutating handler freezes its module — bounded by
# no-resources + deadline, visible as queued ticks in health, never hidden;
# (b) flows import the Temporal VERSIONING problem — code between awaits
# must be deterministic, and editing a flow with journals in flight can
# diverge on replay: pin code hash per journal (finish in flight on old
# code), or migrate journals explicitly. Written down here so it is
# discovered on paper, not in production.

from __future__ import annotations

from typing import TYPE_CHECKING, NamedTuple, Protocol

from dimos.pure import (  # Flow/injected/rpc are the un-built RPC wave — no source module yet
    Flow,
    injected,
    pm,
    rpc,
)
from dimos.pure.module import PureModule
from dimos.pure.rows import latest, tick

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
    from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
    from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
    from dimos.pure import BoundRpc


# ══════════════════════════════════════════════════════════════════════════════
# 1. Wire currency — plain stamped-able msgs. Ack carries the correlation
#    token; NavStatus is an ordinary sparse output port's payload. Nothing
#    RPC-specific about either type: they record, replay, and diff like any
#    other msg.
# ══════════════════════════════════════════════════════════════════════════════
class Ack(NamedTuple):
    goal_id: int
    accepted: bool


class NavStatus(NamedTuple):
    goal_id: int
    phase: str  # "active" | "arrived" | "failed" | "preempted"


# ══════════════════════════════════════════════════════════════════════════════
# 2. Navigation — the SERVER, and the point: the handler is the smallest part
#    of the module. It sees (state, req), folds the request into State, mints
#    the ack — and can do nothing else (§3 enforcement). All long-lived
#    behavior — driving, arrival, terminal status — is step's business,
#    across ticks, recorded. The handler never knows who called: teleop, rim
#    client, flow — all arrive as the same stamped request row.
# ══════════════════════════════════════════════════════════════════════════════
class Navigation(PureModule):
    arrive_radius: float = 0.3

    class In(pm.In):
        pose: PoseStamped = tick(expect_hz=10)
        global_costmap: OccupancyGrid = latest()

    class Out(pm.Out):
        cmd_vel: TwistStamped | None = None  # sparse — silent when idle
        status: NavStatus | None = None  # sparse — emits on transitions

    class State(NamedTuple):
        goal: PoseStamped | None = None
        goal_id: int = 0
        phase: str = "idle"

    @rpc(deadline_ms=5)
    def navigate_to(self, state: State, req: PoseStamped) -> tuple[State, Ack]:
        # mutating, one tick: ACCEPTED, never arrived. Preemption = goal_id
        # bump — step's next status row tells the old goal's awaiters (§4).
        s = state._replace(goal=req, goal_id=state.goal_id + 1, phase="active")
        return s, Ack(goal_id=s.goal_id, accepted=True)

    @rpc(deadline_ms=5)
    def cancel(self, state: State, req: int) -> tuple[State, bool]:
        if req != state.goal_id or state.phase != "active":
            return state, False
        return state._replace(goal=None, phase="idle"), True

    @rpc
    def current_goal(self, state: State, req: None) -> PoseStamped | None:
        return state.goal  # read-only form: off-loop, snapshot, zero freeze

    def step(self, state: State, i: In) -> tuple[State, Out | None]:
        if state.phase != "active" or state.goal is None:
            return state, None
        if self._arrived(i.pose, state.goal):
            return state._replace(goal=None, phase="idle"), Navigation.Out(
                cmd_vel=self._zero(),
                status=NavStatus(state.goal_id, "arrived"),
            )
        return state, Navigation.Out(cmd_vel=self._drive(i, state.goal))

    def _arrived(self, pose: PoseStamped, goal: PoseStamped) -> bool: ...
    def _zero(self) -> TwistStamped: ...
    def _drive(self, i: In, goal: PoseStamped) -> TwistStamped: ...


# ══════════════════════════════════════════════════════════════════════════════
# 3. Capability slot — the caller's side of §5. The protocol declares ONLY
#    the calls this caller needs, members spelled as read-only properties
#    (descriptor access is read-only; a plain-variable member would demand
#    settability — mypy rejects it). Navigation() satisfies NavApi
#    structurally: no facade class, no registration, no MRO membership. A
#    test stub is any object with the same attribute shape.
# ══════════════════════════════════════════════════════════════════════════════
class NavApi(Protocol):
    @property
    def navigate_to(self) -> BoundRpc[PoseStamped, Ack]: ...
    @property
    def cancel(self) -> BoundRpc[int, bool]: ...


# ══════════════════════════════════════════════════════════════════════════════
# 4. NavAction — THE facade, written once (§6). Composes accepted-ack with a
#    status-port subscription into one awaitable that resolves on TERMINAL
#    status. Subscribe BEFORE sending — a status row between send and
#    subscribe would otherwise be missed. Both hosts below use exactly this
#    class; only the durability of the code AROUND the await differs.
# ══════════════════════════════════════════════════════════════════════════════
class NavAction:
    def __init__(self, nav: NavApi, status: object) -> None:  # status: Stream[NavStatus]
        self._nav = nav
        self._status = status

    async def navigate_to(self, goal: PoseStamped) -> bool:
        with self._status.subscribe() as statuses:  # type: ignore[attr-defined]
            ack = await self._nav.navigate_to(goal)  # one tick: ACCEPTED
            if not ack.accepted:
                return False
            async for st in statuses:
                if st.goal_id == ack.goal_id and st.phase != "active":
                    return st.phase == "arrived"
        return False

    async def cancel_current(self) -> bool: ...


# ══════════════════════════════════════════════════════════════════════════════
# 5. Rim host — plain asyncio outside the graph: composes with gather /
#    wait_for / retries, costs no doctrine, and is EPHEMERAL. Right for
#    teleop, CLIs, notebooks, dashboards; wrong for anything that must
#    survive its own process.
# ══════════════════════════════════════════════════════════════════════════════
async def _rim_caller(g: object) -> None:
    import asyncio

    nav = NavAction(g.member("nav.navigation"), g.edge("nav.navigation.status"))  # type: ignore[attr-defined]
    ok = await asyncio.wait_for(nav.navigate_to(_dock_pose()), timeout=120)
    if not ok:
        await nav.cancel_current()


def _dock_pose() -> PoseStamped: ...


# ══════════════════════════════════════════════════════════════════════════════
# 6. Flow host — the same facade, durable (§7). Reads as straight-line skill
#    code; every await lands in the journal. Crash at waypoint 3 → re-run
#    from the top, the first three navigate_to awaits return journaled
#    results instantly, execution resumes mid-loop, live. self.sleep is
#    DATA-TIME (journaled — not re-slept on recovery); wall clock and bare
#    random are unavailable inside flow, per doctrine.
# ══════════════════════════════════════════════════════════════════════════════
class PatrolFlow(Flow):
    waypoints: tuple[PoseStamped, ...] = ()
    home: PoseStamped | None = None

    nav: NavAction = injected()  # wiring supplies it — built from the member (§5)

    async def flow(self) -> None:
        for wp in self.waypoints:
            ok = await self.nav.navigate_to(wp)  # await = until TERMINAL
            if not ok:
                if self.home is not None:
                    await self.nav.navigate_to(self.home)
                return
            await self.sleep(30.0)


# ══════════════════════════════════════════════════════════════════════════════
# TESTS NEED NO ENGINE — handlers are transitions, so test them AS
# transitions: the bound handle exposes the raw function (.transition), same
# doctrine as calling step directly. The facade tests against a protocol
# stub; the flow tests by feeding it a scripted NavAction.
# ══════════════════════════════════════════════════════════════════════════════
def _fake_pose(x: float, y: float) -> PoseStamped: ...


def _unit_tests() -> None:
    m = Navigation()
    s0 = Navigation.State()

    s1, ack = m.navigate_to.transition(s0, _fake_pose(1.0, 2.0))
    assert ack == Ack(goal_id=1, accepted=True) and s1.phase == "active"

    s2, ack2 = m.navigate_to.transition(s1, _fake_pose(3.0, 0.0))
    assert ack2.goal_id == 2  # preemption is just the bump

    assert m.cancel.transition(s2, 1) == (s2, False)  # stale goal_id: no-op


# ══════════════════════════════════════════════════════════════════════════════
# TYPING — dimos.pure internals for @rpc, verified under mypy --strict.
# @rpc is a descriptor like @resource: the decorator's overloads classify the
# handler form; the bound attribute is an async callable, so the caller
# spelling is `await m.navigate_to(pose)` — fully typed, no plugin. (A .rpc
# NAMESPACE is not mypy-expressible without a plugin — hence the descriptor
# spelling. Deliberately left out.)
#
#     class BoundRpc(Generic[TReq, TReply]):
#         async def __call__(self, req: TReq) -> TReply: ...   # stamp, enqueue, await
#         transition: Callable[..., Any]                       # raw fn — for tests
#
#     class RpcHandle(Generic[TReq, TReply]):
#         def __get__(self, obj, owner=None) -> BoundRpc[TReq, TReply]: ...
#
#     @overload  # mutating
#     def rpc(f: Callable[[M, TState, TReq], tuple[TState, TReply]]) -> RpcHandle[TReq, TReply]: ...
#     @overload  # read-only
#     def rpc(f: Callable[[M, TState, TReq], TReply]) -> RpcHandle[TReq, TReply]: ...
#     @overload  # stateless
#     def rpc(f: Callable[[M, TReq], TReply]) -> RpcHandle[TReq, TReply]: ...
#     @overload  # parameterized: @rpc(deadline_ms=5)
#     def rpc(*, deadline_ms: float) -> Callable[..., RpcHandle[Any, Any]]: ...
#
# Verified end-to-end: `await Navigation().navigate_to(pose)` reveals Ack;
# the read-only form reveals its reply type; Navigation() satisfies NavApi
# structurally (property-spelled protocol); a two-line stub satisfies it for
# tests.
# ══════════════════════════════════════════════════════════════════════════════


# ══════════════════════════════════════════════════════════════════════════════
# REPLAY & RECOVERY — how the pieces behave offline, stated once:
#   • Requests are recorded rows → replay RE-INJECTS them at their ts; the
#     server's transitions re-run identically. Replies/status are outputs —
#     inert unless wired, so a replay never answers a live caller.
#   • A rim client has no offline story and needs none: its effects (request
#     row, status rows) were recorded; its own control flow is ephemeral by
#     definition.
#   • A flow recovers by journal replay (§7) — and the inspector shows a
#     flow as its journal: which await, at which tick, resolved with what.
#     "Where in its plan was the skill at t=4032" is a row lookup.
# ══════════════════════════════════════════════════════════════════════════════


# Deliberately left out: a .rpc namespace (unTypeable without a plugin — the
# descriptor spelling IS the API); module→module RPC inside a graph
# (restructure as ports — capability slots are for the skills layer and the
# rim); streaming RPCs (a stream is a port — that's the other half of the
# system); preemptible/abortable handlers (a handler is one tick — there is
# nothing to abort; actions preempt via goal_id); flow-to-flow calls (compose
# at the facade level or restructure — one journal per durable unit).
