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

"""DuckControlModule: mux/mode/lock semantics and nav_state derivation,
driven through the handlers with a fake planner and a fake clock."""

from __future__ import annotations

from collections.abc import Callable, Generator
from dataclasses import dataclass, field
import json
import math
import time
from typing import Any

from pydantic import ValidationError
import pytest

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.base import NavigationState
from dimos.robot.pollen.microduck.control_module import (
    NAV_STATES,
    NO_PATH_TIMEOUT_SEC,
    STOP_SETTLE_SEC,
    DuckControlModule,
)


class FakeClock:
    def __init__(self) -> None:
        self.now = 100.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class FakeNavigation:
    """NavigationInterfaceSpec stand-in: scripted state, recorded calls."""

    def __init__(self) -> None:
        self.state = NavigationState.IDLE
        self.reached = False
        self.calls: list[str] = []
        self.raise_on: set[str] = set()

    def _call(self, name: str) -> None:
        self.calls.append(name)
        if name in self.raise_on:
            raise TimeoutError(f"{name} timed out")

    def set_goal(self, goal: PoseStamped) -> bool:
        self._call("set_goal")
        return True

    def get_state(self) -> NavigationState:
        self._call("get_state")
        return self.state

    def is_goal_reached(self) -> bool:
        self._call("is_goal_reached")
        return self.reached

    def cancel_goal(self) -> bool:
        self._call("cancel_goal")
        self.state = NavigationState.IDLE
        return True

    @property
    def cancels(self) -> int:
        return self.calls.count("cancel_goal")


class FakeTransport:
    """In-stream transport stub for start()/stop(): publishes synchronously."""

    def __init__(self) -> None:
        self.subscribers: list[Callable[[Any], Any]] = []

    def subscribe(self, cb: Callable[[Any], Any], stream: Any = None) -> Callable[[], None]:
        self.subscribers.append(cb)

        def unsubscribe() -> None:
            self.subscribers.remove(cb)

        return unsubscribe

    def publish(self, msg: Any) -> None:
        for cb in list(self.subscribers):
            cb(msg)

    def stop(self) -> None:
        self.subscribers.clear()


@dataclass
class Captured:
    cmd_vel: list[Twist] = field(default_factory=list)
    mode: list[dict[str, Any]] = field(default_factory=list)
    nav_state: list[dict[str, Any]] = field(default_factory=list)
    policy_request: list[dict[str, Any]] = field(default_factory=list)

    def clear(self) -> None:
        self.cmd_vel.clear()
        self.mode.clear()
        self.nav_state.clear()
        self.policy_request.clear()


def _attach(module: DuckControlModule) -> tuple[Captured, list[Callable[[], None]]]:
    captured = Captured()
    unsubs = [
        module.cmd_vel.subscribe(captured.cmd_vel.append),
        module.mode.subscribe(lambda raw: captured.mode.append(json.loads(raw))),
        module.nav_state.subscribe(lambda raw: captured.nav_state.append(json.loads(raw))),
        module.policy_request.subscribe(
            lambda raw: captured.policy_request.append(json.loads(raw))
        ),
    ]
    return captured, unsubs


@dataclass
class Rig:
    module: DuckControlModule
    captured: Captured
    nav: FakeNavigation
    clock: FakeClock


@pytest.fixture()
def rig() -> Generator[Rig, None, None]:
    module = DuckControlModule(tele_cooldown_sec=1.0)
    clock = FakeClock()
    nav = FakeNavigation()
    module._clock = clock
    module._navigation = nav  # what _connect_module_refs does at deploy time
    captured, unsubs = _attach(module)
    try:
        yield Rig(module, captured, nav, clock)
    finally:
        for unsub in unsubs:
            unsub()
        module._close_module()


def _twist(lx: float = 0.0, ly: float = 0.0, wz: float = 0.0) -> Twist:
    return Twist(linear=Vector3(lx, ly, 0.0), angular=Vector3(0.0, 0.0, wz))


def _goal(x: float = 1.0, y: float = 2.0, yaw: float = 0.5) -> PoseStamped:
    return PoseStamped(
        ts=time.time(),
        frame_id="world",
        position=Vector3(x, y, 0.0),
        orientation=Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)),
    )


def _command(name: str, **args: Any) -> str:
    payload: dict[str, Any] = {"name": name}
    if args:
        payload["args"] = args
    return json.dumps(payload)


def _policy_state(locked: bool) -> str:
    return json.dumps({"variant": "default", "active": "walk", "locked": locked, "t": 0.0})


def _is_zero(msg: Twist) -> bool:
    return (
        msg.linear.x,
        msg.linear.y,
        msg.linear.z,
        msg.angular.x,
        msg.angular.y,
        msg.angular.z,
    ) == (
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


# ---------------------------------------------------------------- ui_command


def test_set_mode_publishes_mode_json(rig: Rig) -> None:
    rig.module._on_ui_command(_command("set_mode", mode="agent"))
    (msg,) = rig.captured.mode
    assert set(msg) == {"mode", "t"}
    assert msg["mode"] == "agent"
    assert isinstance(msg["t"], float)

    rig.module._on_ui_command(_command("set_mode", mode="teleop"))
    assert rig.captured.mode[-1]["mode"] == "teleop"


def test_set_mode_rejects_unknown_mode(rig: Rig) -> None:
    rig.module._on_ui_command(_command("set_mode", mode="autopilot"))
    rig.module._on_ui_command(_command("set_mode"))
    assert rig.captured.mode == []
    assert rig.module._mode == "teleop"


def test_switch_to_agent_mid_teleop_zeroes_cmd_vel(rig: Rig) -> None:
    rig.module._on_teleop(_twist(lx=0.3))
    rig.captured.clear()
    rig.module._on_ui_command(_command("set_mode", mode="agent"))
    (zero,) = rig.captured.cmd_vel
    assert _is_zero(zero)


def test_policy_command_is_republished_as_policy_request(rig: Rig) -> None:
    rig.module._on_ui_command(_command("policy", policy="kick_left", action="start"))
    rig.module._on_ui_command(_command("policy", action="stop"))
    rig.module._on_ui_command(_command("policy", policy="sitstand", action="toggle"))
    first, bare_stop, toggle = rig.captured.policy_request
    assert {k: v for k, v in first.items() if k != "t"} == {
        "policy": "kick_left",
        "action": "start",
    }
    assert {k: v for k, v in bare_stop.items() if k != "t"} == {"action": "stop"}
    assert toggle["policy"] == "sitstand" and toggle["action"] == "toggle"
    assert all(isinstance(m["t"], float) for m in rig.captured.policy_request)


@pytest.mark.parametrize(
    "args",
    [
        {"policy": "kick_left", "action": "launch"},
        {"policy": "kick_left"},
        {"action": "start"},  # start needs a policy
        {"policy": "", "action": "start"},
        {"policy": 7, "action": "start"},
    ],
    ids=["bad_action", "no_action", "start_without_policy", "empty_policy", "non_str_policy"],
)
def test_invalid_policy_commands_are_dropped(rig: Rig, args: dict[str, Any]) -> None:
    rig.module._on_ui_command(_command("policy", **args))
    assert rig.captured.policy_request == []


def test_cancel_nav_command_cancels_goal_and_zeroes(rig: Rig) -> None:
    rig.module._on_goal_request(_goal())
    rig.captured.clear()

    rig.module._on_ui_command(_command("cancel_nav"))

    assert rig.nav.cancels == 1
    (zero,) = rig.captured.cmd_vel
    assert _is_zero(zero)
    (state,) = rig.captured.nav_state
    assert state["state"] == "cancelled"
    assert state["goal"] is None


def test_cancel_nav_without_goal_still_stops_but_keeps_state(rig: Rig) -> None:
    rig.module._on_ui_command(_command("cancel_nav"))
    assert rig.nav.cancels == 1
    assert len(rig.captured.cmd_vel) == 1 and _is_zero(rig.captured.cmd_vel[0])
    assert rig.captured.nav_state == []  # nothing was in progress: no 'cancelled' flash


@pytest.mark.parametrize(
    "raw",
    [
        "not json",
        "[1, 2]",
        json.dumps({"name": "reboot"}),
        json.dumps({"name": "set_mode", "args": ["agent"]}),
        json.dumps({"args": {"mode": "agent"}}),
    ],
    ids=["not_json", "list", "unknown_name", "args_not_object", "no_name"],
)
def test_invalid_ui_commands_are_ignored(rig: Rig, raw: str) -> None:
    rig.module._on_ui_command(raw)
    assert rig.captured.cmd_vel == []
    assert rig.captured.mode == []
    assert rig.captured.policy_request == []
    assert rig.nav.cancels == 0


# ---------------------------------------------------------------- teleop mux


def test_teleop_nonzero_cancels_active_goal_and_forwards(rig: Rig) -> None:
    rig.module._on_goal_request(_goal())
    rig.captured.clear()

    rig.module._on_teleop(_twist(lx=0.3, wz=0.2))

    assert rig.nav.cancels == 1
    zero, forwarded = rig.captured.cmd_vel
    assert _is_zero(zero)
    assert forwarded.linear.x == pytest.approx(0.3)
    assert forwarded.angular.z == pytest.approx(0.2)
    assert rig.captured.nav_state[-1]["state"] == "cancelled"

    # No goal any more: the next twist just forwards, no second cancel.
    rig.captured.clear()
    rig.module._on_teleop(_twist(lx=0.4))
    assert rig.nav.cancels == 1
    (forwarded,) = rig.captured.cmd_vel
    assert forwarded.linear.x == pytest.approx(0.4)


def test_teleop_scaling_applies_to_vx_vy_wz(rig: Rig) -> None:
    rig.module.config.tele_cmd_vel_scaling = (0.5, 2.0, 0.25)
    rig.module._on_teleop(_twist(lx=1.0, ly=1.0, wz=1.0))
    (published,) = rig.captured.cmd_vel
    assert published.linear.x == pytest.approx(0.5)
    assert published.linear.y == pytest.approx(2.0)
    assert published.angular.z == pytest.approx(0.25)


def test_teleop_zero_is_estop_after_motion_and_stray_when_idle(rig: Rig) -> None:
    # Stray zero while nothing moves: dropped entirely.
    rig.module._on_teleop(_twist())
    assert rig.captured.cmd_vel == []
    assert rig.nav.cancels == 0

    # Release after motion: one zero + cancel.
    rig.module._on_teleop(_twist(lx=0.3))
    rig.captured.clear()
    rig.module._on_teleop(_twist())
    assert rig.nav.cancels == 1
    (zero,) = rig.captured.cmd_vel
    assert _is_zero(zero)

    # The pad repeats the zero; those are stray again.
    rig.captured.clear()
    rig.module._on_teleop(_twist())
    rig.module._on_teleop(_twist())
    assert rig.captured.cmd_vel == []
    assert rig.nav.cancels == 1


def test_teleop_zero_with_active_goal_cancels_it(rig: Rig) -> None:
    rig.module._on_goal_request(_goal())
    rig.captured.clear()
    rig.module._on_teleop(_twist())  # Space / e-stop while navigating
    assert rig.nav.cancels == 1
    (zero,) = rig.captured.cmd_vel
    assert _is_zero(zero)
    assert rig.captured.nav_state[-1]["state"] == "cancelled"


def test_agent_mode_ignores_teleop(rig: Rig) -> None:
    rig.module._on_ui_command(_command("set_mode", mode="agent"))
    rig.module._on_goal_request(_goal())
    rig.captured.clear()

    rig.module._on_teleop(_twist(lx=0.5))
    rig.module._on_teleop(_twist())

    assert rig.captured.cmd_vel == []
    assert rig.nav.cancels == 0
    # Navigation keeps flowing in agent mode.
    rig.module._on_nav(_twist(lx=0.2))
    (nav,) = rig.captured.cmd_vel
    assert nav.linear.x == pytest.approx(0.2)


# ---------------------------------------------------------------- nav mux


def test_nav_twists_dropped_inside_cooldown_then_resume(rig: Rig) -> None:
    rig.module._on_teleop(_twist(lx=0.3))
    rig.captured.clear()

    rig.clock.advance(0.5)
    rig.module._on_nav(_twist(lx=0.9))
    assert rig.captured.cmd_vel == []

    rig.clock.advance(0.6)  # past tele_cooldown_sec=1.0
    rig.module._on_nav(_twist(lx=0.9))
    (nav,) = rig.captured.cmd_vel
    assert nav.linear.x == pytest.approx(0.9)
    assert rig.module._teleop_active is False


def test_nav_twists_dropped_in_cooldown_after_estop_zero(rig: Rig) -> None:
    rig.module._on_teleop(_twist(lx=0.3))
    rig.clock.advance(0.9)
    rig.module._on_teleop(_twist())  # release restamps the cooldown
    rig.captured.clear()
    rig.clock.advance(0.5)
    rig.module._on_nav(_twist(lx=0.9))
    assert rig.captured.cmd_vel == []
    rig.clock.advance(0.6)
    rig.module._on_nav(_twist(lx=0.9))
    assert len(rig.captured.cmd_vel) == 1


def test_nav_twists_pass_when_no_teleop_happened(rig: Rig) -> None:
    rig.module._on_nav(_twist(lx=0.9))
    (nav,) = rig.captured.cmd_vel
    assert nav.linear.x == pytest.approx(0.9)


def test_nav_twists_dropped_while_locked(rig: Rig) -> None:
    rig.module._on_policy_state(_policy_state(locked=True))
    rig.captured.clear()
    rig.module._on_nav(_twist(lx=0.9))
    assert rig.captured.cmd_vel == []

    rig.module._on_policy_state(_policy_state(locked=False))
    rig.module._on_nav(_twist(lx=0.9))
    assert len(rig.captured.cmd_vel) == 1


# ---------------------------------------------------------------- policy lock


def test_locked_rising_edge_cancels_nav_and_zeroes_once(rig: Rig) -> None:
    rig.module._on_goal_request(_goal())
    rig.captured.clear()

    rig.module._on_policy_state(_policy_state(locked=True))
    assert rig.nav.cancels == 1
    (zero,) = rig.captured.cmd_vel
    assert _is_zero(zero)
    assert rig.captured.nav_state[-1]["state"] == "cancelled"

    # Still locked: no repeated cancel / zero.
    rig.captured.clear()
    rig.module._on_policy_state(_policy_state(locked=True))
    assert rig.nav.cancels == 1
    assert rig.captured.cmd_vel == []


def test_teleop_while_locked_publishes_one_zero_then_nothing(rig: Rig) -> None:
    rig.module._on_policy_state(_policy_state(locked=True))
    rig.captured.clear()
    # The rising edge already zeroed; held keys produce nothing more.
    rig.module._on_teleop(_twist(lx=0.5))
    rig.module._on_teleop(_twist(lx=0.5))
    assert rig.captured.cmd_vel == []

    rig.module._on_policy_state(_policy_state(locked=False))
    rig.module._on_teleop(_twist(lx=0.5))
    (forwarded,) = rig.captured.cmd_vel
    assert forwarded.linear.x == pytest.approx(0.5)

    # Lock again: a zero goes out once more (the edge), and the next pad twist adds none.
    rig.captured.clear()
    rig.module._on_policy_state(_policy_state(locked=True))
    rig.module._on_teleop(_twist(lx=0.5))
    assert len(rig.captured.cmd_vel) == 1 and _is_zero(rig.captured.cmd_vel[0])


def test_locked_from_the_start_without_edge_zero_sends_one_zero_on_teleop(rig: Rig) -> None:
    # Lock flag flips on with the module's own edge handling disabled by a
    # malformed first message: cover the 'publish zero once' path directly.
    rig.module._on_policy_state("garbage")  # ignored
    rig.module._locked = True  # as if the edge had been missed
    rig.module._on_teleop(_twist(lx=0.5))
    rig.module._on_teleop(_twist(lx=0.5))
    assert len(rig.captured.cmd_vel) == 1 and _is_zero(rig.captured.cmd_vel[0])


# ---------------------------------------------------------------- nav_state


def _tick(rig: Rig, state: NavigationState, reached: bool) -> dict[str, Any]:
    rig.nav.state = state
    rig.nav.reached = reached
    rig.module._tick()
    return rig.captured.nav_state[-1]


def test_nav_state_json_shape_and_lifecycle(rig: Rig) -> None:
    initial = _tick(rig, NavigationState.IDLE, False)
    assert set(initial) == {"state", "goal", "since", "t"}
    assert initial["state"] == "idle" and initial["goal"] is None
    assert rig.captured.mode[-1]["mode"] == "teleop"  # mode rides every tick

    rig.module._on_goal_request(_goal(1.0, 2.0, 0.5))
    pending = rig.captured.nav_state[-1]  # published immediately
    assert pending["state"] == "idle"
    assert pending["goal"] == {"x": 1.0, "y": 2.0, "yaw": pytest.approx(0.5)}

    following = _tick(rig, NavigationState.FOLLOWING_PATH, False)
    assert following["state"] == "following_path"
    assert following["goal"]["x"] == 1.0
    since_following = following["since"]

    still = _tick(rig, NavigationState.FOLLOWING_PATH, False)
    assert still["since"] == since_following  # since only moves on state change

    recovery = _tick(rig, NavigationState.RECOVERY, False)
    assert recovery["state"] == "recovery"

    reached = _tick(rig, NavigationState.IDLE, True)
    assert reached["state"] == "reached"
    assert reached["goal"] is None
    # Sticky until the next goal.
    assert _tick(rig, NavigationState.IDLE, True)["state"] == "reached"

    rig.module._on_goal_request(_goal(-1.0, 0.0, 0.0))
    assert rig.captured.nav_state[-1]["state"] == "idle"
    assert rig.captured.nav_state[-1]["goal"]["x"] == -1.0
    assert all(s in NAV_STATES for s in {m["state"] for m in rig.captured.nav_state})


def test_nav_state_ignores_reached_latched_from_previous_goal(rig: Rig) -> None:
    rig.module._on_goal_request(_goal())
    _tick(rig, NavigationState.FOLLOWING_PATH, False)
    assert _tick(rig, NavigationState.IDLE, True)["state"] == "reached"

    # New goal while the planner still reports the old 'reached' flag.
    rig.module._on_goal_request(_goal(0.5, 0.5))
    assert _tick(rig, NavigationState.IDLE, True)["state"] == "idle"
    # The planner processes the goal (flag drops), then really arrives.
    assert _tick(rig, NavigationState.IDLE, False)["state"] == "idle"
    assert _tick(rig, NavigationState.IDLE, True)["state"] == "reached"


def test_nav_state_no_path_after_timeout(rig: Rig) -> None:
    rig.module._on_goal_request(_goal())
    assert _tick(rig, NavigationState.IDLE, False)["state"] == "idle"
    rig.clock.advance(NO_PATH_TIMEOUT_SEC - 0.5)
    assert _tick(rig, NavigationState.IDLE, False)["state"] == "idle"
    rig.clock.advance(1.0)
    no_path = _tick(rig, NavigationState.IDLE, False)
    assert no_path["state"] == "no_path"
    assert no_path["goal"] is None


def test_nav_state_cancelled_when_planner_stops_early(rig: Rig) -> None:
    rig.module._on_goal_request(_goal())
    _tick(rig, NavigationState.FOLLOWING_PATH, False)
    # Brief replan gap: not cancelled yet, goal still shown.
    gap = _tick(rig, NavigationState.IDLE, False)
    assert gap["state"] == "idle" and gap["goal"] is not None
    rig.clock.advance(STOP_SETTLE_SEC / 2)
    assert _tick(rig, NavigationState.FOLLOWING_PATH, False)["state"] == "following_path"

    # Stopped from elsewhere (skill stop_moving via RPC): idle for the settle window.
    _tick(rig, NavigationState.IDLE, False)
    rig.clock.advance(STOP_SETTLE_SEC)
    cancelled = _tick(rig, NavigationState.IDLE, False)
    assert cancelled["state"] == "cancelled"
    assert cancelled["goal"] is None


def test_nav_state_tracks_goal_set_behind_our_back(rig: Rig) -> None:
    # A skill calling set_goal over RPC never publishes goal_request.
    following = _tick(rig, NavigationState.FOLLOWING_PATH, False)
    assert following["state"] == "following_path"
    assert following["goal"] is None
    assert _tick(rig, NavigationState.IDLE, True)["state"] == "reached"


def test_nav_state_cancelled_after_teleop_then_new_goal_starts_fresh(rig: Rig) -> None:
    rig.module._on_goal_request(_goal())
    _tick(rig, NavigationState.FOLLOWING_PATH, False)
    rig.module._on_teleop(_twist(lx=0.2))
    assert rig.captured.nav_state[-1]["state"] == "cancelled"
    # Planner reports idle/not reached afterwards: stays cancelled.
    assert _tick(rig, NavigationState.IDLE, False)["state"] == "cancelled"


def test_planner_rpc_failures_keep_the_loop_alive(rig: Rig) -> None:
    rig.module._on_goal_request(_goal())
    _tick(rig, NavigationState.FOLLOWING_PATH, False)
    rig.nav.raise_on = {"get_state", "is_goal_reached", "cancel_goal"}
    rig.captured.clear()

    rig.module._tick()  # must not raise
    # A failed poll changes nothing but still publishes the last snapshot.
    assert rig.captured.nav_state[-1]["state"] == "following_path"
    assert rig.captured.mode[-1]["mode"] == "teleop"

    rig.module._on_ui_command(_command("cancel_nav"))  # cancel_goal raising
    assert rig.captured.nav_state[-1]["state"] == "cancelled"
    assert _is_zero(rig.captured.cmd_vel[-1])


def test_missing_navigation_ref_is_tolerated(rig: Rig) -> None:
    del rig.module._navigation
    rig.module._tick()
    rig.module._on_ui_command(_command("cancel_nav"))
    assert rig.captured.nav_state[-1]["state"] == "idle"
    assert _is_zero(rig.captured.cmd_vel[-1])


# ---------------------------------------------------------------- lifecycle


def test_start_publishes_mode_runs_state_thread_and_stops_cleanly() -> None:
    module = DuckControlModule(state_hz=50.0)
    nav = FakeNavigation()
    module._navigation = nav
    transports = {
        name: FakeTransport()
        for name in ("nav_cmd_vel", "tele_cmd_vel", "ui_command", "policy_state", "goal_request")
    }
    for name, transport in transports.items():
        getattr(module, name).transport = transport
    captured, unsubs = _attach(module)
    try:
        module.start()
        assert captured.mode and captured.mode[0]["mode"] == "teleop"
        deadline = time.monotonic() + 5.0
        while len(captured.nav_state) < 3 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert len(captured.nav_state) >= 3
        assert "get_state" in nav.calls and "is_goal_reached" in nav.calls

        # Inputs are wired through the transports.
        transports["ui_command"].publish(
            json.dumps({"name": "set_mode", "args": {"mode": "agent"}})
        )
        assert captured.mode[-1]["mode"] == "agent"
        transports["goal_request"].publish(_goal(0.3, 0.4))
        assert captured.nav_state[-1]["goal"]["x"] == pytest.approx(0.3)
        transports["tele_cmd_vel"].publish(_twist(lx=0.5))  # agent mode: dropped
        assert all(_is_zero(m) or m.linear.x != 0.5 for m in captured.cmd_vel)
        transports["nav_cmd_vel"].publish(_twist(lx=0.7))
        assert captured.cmd_vel[-1].linear.x == pytest.approx(0.7)

        thread = module._state_thread
        assert thread is not None and thread.is_alive()
    finally:
        for unsub in unsubs:
            unsub()
        module.stop()
    assert not thread.is_alive()
    assert module._state_thread is None
    assert all(t.subscribers == [] for t in transports.values())


def test_stop_without_start_is_safe() -> None:
    module = DuckControlModule()
    module.stop()


def test_unwired_inputs_are_skipped_on_start() -> None:
    module = DuckControlModule(state_hz=50.0)
    module._navigation = FakeNavigation()
    try:
        module.start()  # no transports at all: subscriptions skipped, thread runs
        assert module._state_thread is not None and module._state_thread.is_alive()
    finally:
        module.stop()


def test_config_rejects_unknown_default_mode() -> None:
    with pytest.raises(ValidationError):
        DuckControlModule(default_mode="autopilot")


def test_default_mode_agent_is_honoured() -> None:
    module = DuckControlModule(default_mode="agent")
    captured, unsubs = _attach(module)
    try:
        module._on_teleop(_twist(lx=0.5))
        assert captured.cmd_vel == []
    finally:
        for unsub in unsubs:
            unsub()
        module._close_module()
