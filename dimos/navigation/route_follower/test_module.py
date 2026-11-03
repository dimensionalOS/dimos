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

from __future__ import annotations

from collections.abc import Callable, Generator
from dataclasses import dataclass, field
import json
import time
from typing import Any

from dimos_lcm.std_msgs import Bool, String
import pytest

from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.route_follower.module import RouteFollower


class FakeTransport:
    def __init__(self) -> None:
        self._callbacks: list[Callable[[Any], Any]] = []

    def subscribe(
        self, callback: Callable[[Any], Any], selfstream: Any = None
    ) -> Callable[[], None]:
        self._callbacks.append(callback)

        def unsubscribe() -> None:
            self._callbacks.remove(callback)

        return unsubscribe

    def broadcast(self, stream: Any, msg: Any) -> None:
        for callback in list(self._callbacks):
            callback(msg)

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass


@dataclass
class Captured:
    clicked_point: list[PointStamped] = field(default_factory=list)
    route_goal: list[PoseStamped] = field(default_factory=list)
    route_stop: list[Bool] = field(default_factory=list)
    route_state: list[String] = field(default_factory=list)
    route_path: list[Path] = field(default_factory=list)


@pytest.fixture()
def follower() -> Generator[tuple[RouteFollower, Captured], None, None]:
    module = RouteFollower()
    captured = Captured()
    unsubs = [
        module.clicked_point.subscribe(captured.clicked_point.append),
        module.route_goal.subscribe(captured.route_goal.append),
        module.route_stop.subscribe(captured.route_stop.append),
        module.route_state.subscribe(captured.route_state.append),
        module.route_path.subscribe(captured.route_path.append),
    ]
    try:
        yield module, captured
    finally:
        for unsub in unsubs:
            unsub()
        module._close_module()


def _cmd(payload: dict[str, Any]) -> String:
    return String(data=json.dumps(payload))


def _click(x: float = 1.0, y: float = 2.0, z: float = 0.0, frame: str = "world") -> PointStamped:
    return PointStamped(ts=100.0, frame_id=frame, x=x, y=y, z=z)


def _odom(x: float, y: float, z: float = 0.0) -> PoseStamped:
    return PoseStamped(ts=100.0, frame_id="world", position=[x, y, z])


def _state(captured: Captured) -> dict[str, Any]:
    return json.loads(captured.route_state[-1].data)


def _wait_for(predicate: Callable[[], bool], timeout: float = 5.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    assert predicate()


def test_click_forwarded_when_add_mode_off(follower: tuple[RouteFollower, Captured]) -> None:
    module, captured = follower
    click = _click(x=3.0, y=4.0)

    module._on_viewer_click(click)

    assert captured.clicked_point == [click]
    assert captured.route_goal == []
    assert captured.route_stop == []


def test_click_becomes_waypoint_when_add_mode_on(follower: tuple[RouteFollower, Captured]) -> None:
    module, captured = follower
    module._on_route_cmd(_cmd({"op": "set_mode", "enabled": True}))

    module._on_viewer_click(_click(x=1.5, y=-0.5))

    assert captured.clicked_point == []
    assert captured.route_goal == []
    assert captured.route_stop == []
    assert _state(captured)["waypoints"] == [
        {"index": 1, "x": 1.5, "y": -0.5, "z": 0.0, "state": "pending"}
    ]
    assert len(captured.route_path[-1].poses) == 1


def test_set_mode_toggles_the_setting(follower: tuple[RouteFollower, Captured]) -> None:
    module, captured = follower

    module._on_route_cmd(_cmd({"op": "set_mode", "enabled": True}))
    assert _state(captured)["add_mode"] is True

    module._on_route_cmd(_cmd({"op": "set_mode", "enabled": False}))
    assert _state(captured)["add_mode"] is False


def test_start_publishes_goal_for_first_waypoint(follower: tuple[RouteFollower, Captured]) -> None:
    module, captured = follower
    module._on_route_cmd(_cmd({"op": "set_mode", "enabled": True}))
    module._on_viewer_click(_click(x=3.0, y=4.0, z=0.5, frame="map"))

    module._on_route_cmd(_cmd({"op": "start"}))

    assert len(captured.route_goal) == 1
    goal = captured.route_goal[0]
    assert (goal.x, goal.y, goal.z) == (3.0, 4.0, 0.5)
    assert goal.frame_id == "map"
    assert (goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w) == (
        0.0,
        0.0,
        0.0,
        1.0,
    )


def test_odom_within_pass_radius_publishes_next_goal_immediately(
    follower: tuple[RouteFollower, Captured],
) -> None:
    module, captured = follower
    module._on_route_cmd(_cmd({"op": "set_mode", "enabled": True}))
    module._on_viewer_click(_click(x=0.0, y=0.0))
    module._on_viewer_click(_click(x=5.0, y=0.0))
    module._on_route_cmd(_cmd({"op": "start"}))
    assert len(captured.route_goal) == 1

    module._on_odom(_odom(0.05, 0.0))

    assert len(captured.route_goal) == 2
    assert (captured.route_goal[1].x, captured.route_goal[1].y) == (5.0, 0.0)
    assert captured.route_stop == []
    state = _state(captured)
    assert [w["state"] for w in state["waypoints"]] == ["reached", "current"]
    assert state["current_index"] == 2


def test_goal_reached_on_the_final_waypoint_completes_the_route(
    follower: tuple[RouteFollower, Captured],
) -> None:
    module, captured = follower
    module._on_route_cmd(_cmd({"op": "set_mode", "enabled": True}))
    module._on_viewer_click(_click(x=0.0, y=0.0))
    module._on_viewer_click(_click(x=5.0, y=0.0))
    module._on_route_cmd(_cmd({"op": "start"}))
    module._on_odom(_odom(0.05, 0.0))
    assert len(captured.route_goal) == 2

    module._on_goal_reached(Bool(data=True))

    state = _state(captured)
    assert state["status"] == "completed"
    assert state["current_index"] is None
    assert [w["state"] for w in state["waypoints"]] == ["reached", "reached"]


def test_waypoint_within_reach_at_its_turn_is_reached_without_waiting(
    follower: tuple[RouteFollower, Captured],
) -> None:
    module, captured = follower
    module._on_odom(_odom(0.0, 0.0))
    module._on_route_cmd(_cmd({"op": "set_mode", "enabled": True}))
    module._on_viewer_click(_click(x=0.1, y=0.0))
    module._on_viewer_click(_click(x=5.0, y=0.0))

    module._on_route_cmd(_cmd({"op": "start"}))

    assert len(captured.route_goal) == 1
    assert (captured.route_goal[0].x, captured.route_goal[0].y) == (5.0, 0.0)
    state = _state(captured)
    assert [w["state"] for w in state["waypoints"]] == ["reached", "current"]


def test_second_waypoint_at_the_same_spot_works(
    follower: tuple[RouteFollower, Captured],
) -> None:
    module, captured = follower
    module._on_route_cmd(_cmd({"op": "set_mode", "enabled": True}))
    module._on_viewer_click(_click(x=1.0, y=1.0))
    module._on_viewer_click(_click(x=1.0, y=1.0))
    module._on_viewer_click(_click(x=5.0, y=5.0))
    module._on_route_cmd(_cmd({"op": "start"}))

    module._on_odom(_odom(1.0, 1.0))

    assert len(captured.route_goal) == 2
    assert (captured.route_goal[1].x, captured.route_goal[1].y) == (5.0, 5.0)
    state = _state(captured)
    assert [w["state"] for w in state["waypoints"]] == ["reached", "reached", "current"]


def test_duplicate_start_publishes_nothing_new(
    follower: tuple[RouteFollower, Captured],
) -> None:
    module, captured = follower
    module._on_route_cmd(_cmd({"op": "set_mode", "enabled": True}))
    module._on_viewer_click(_click(x=5.0, y=5.0))
    module._on_route_cmd(_cmd({"op": "start"}))
    assert len(captured.route_goal) == 1
    revision = _state(captured)["revision"]

    module._on_route_cmd(_cmd({"op": "start"}))

    assert len(captured.route_goal) == 1
    assert _state(captured)["revision"] == revision


def test_click_while_running_is_dropped_with_message(
    follower: tuple[RouteFollower, Captured],
) -> None:
    module, captured = follower
    module._on_route_cmd(_cmd({"op": "set_mode", "enabled": True}))
    module._on_viewer_click(_click(x=5.0, y=5.0))
    module._on_route_cmd(_cmd({"op": "start"}))

    module._on_viewer_click(_click(x=9.0, y=9.0))
    assert _state(captured)["message"] == "stop the route first"

    module._on_route_cmd(_cmd({"op": "set_mode", "enabled": False}))
    module._on_viewer_click(_click(x=8.0, y=8.0))
    assert _state(captured)["message"] == "stop the route first"

    assert captured.clicked_point == []
    assert captured.route_stop == []
    assert len(captured.route_goal) == 1
    assert [w["index"] for w in _state(captured)["waypoints"]] == [1]


def test_robot_position_known_false_until_first_odom(
    follower: tuple[RouteFollower, Captured],
) -> None:
    module, captured = follower
    module._on_route_cmd(_cmd({"op": "set_mode", "enabled": True}))
    assert _state(captured)["robot_position_known"] is False

    module._on_odom(_odom(0.0, 0.0))
    assert _state(captured)["robot_position_known"] is True


def test_route_outputs_republish_at_the_configured_rate() -> None:
    module = RouteFollower(state_publish_hz=50.0)
    for port in (
        module.odom,
        module.viewer_click,
        module.goal_reached,
        module.stop_seen,
        module.goal_seen,
        module.tele_cmd_seen,
        module.route_cmd,
    ):
        port.transport = FakeTransport()
    states: list[String] = []
    paths: list[Path] = []
    module.route_state.subscribe(states.append)
    module.route_path.subscribe(paths.append)
    module.start()
    try:
        _wait_for(lambda: len(states) >= 2 and len(paths) >= 2)
    finally:
        module.stop()

    revisions = {json.loads(s.data)["revision"] for s in states[:2]}
    assert revisions == {0}
    assert paths[0].poses == []


def test_remove_out_of_range_leaves_route_unchanged_and_sets_message(
    follower: tuple[RouteFollower, Captured],
) -> None:
    module, captured = follower
    module._on_route_cmd(_cmd({"op": "set_mode", "enabled": True}))
    module._on_viewer_click(_click(x=1.0, y=0.0))

    module._on_route_cmd(_cmd({"op": "remove", "index": 2}))

    state = _state(captured)
    assert state["message"] == "no waypoint 2"
    assert [w["x"] for w in state["waypoints"]] == [1.0]


def test_twenty_first_waypoint_click_is_refused(follower: tuple[RouteFollower, Captured]) -> None:
    module, captured = follower
    module._on_route_cmd(_cmd({"op": "set_mode", "enabled": True}))
    for i in range(20):
        module._on_viewer_click(_click(x=float(i), y=0.0))

    module._on_viewer_click(_click(x=20.0, y=0.0))

    state = _state(captured)
    assert len(state["waypoints"]) == 20
    assert state["message"] == "route is full"


def test_remove_and_clear_are_refused_while_running(
    follower: tuple[RouteFollower, Captured],
) -> None:
    module, captured = follower
    module._on_route_cmd(_cmd({"op": "set_mode", "enabled": True}))
    module._on_viewer_click(_click(x=1.0, y=0.0))
    module._on_viewer_click(_click(x=2.0, y=0.0))
    module._on_route_cmd(_cmd({"op": "start"}))

    module._on_route_cmd(_cmd({"op": "remove", "index": 1}))
    assert _state(captured)["message"] == "stop the route first"
    module._on_route_cmd(_cmd({"op": "clear"}))
    assert _state(captured)["message"] == "stop the route first"

    state = _state(captured)
    assert [w["index"] for w in state["waypoints"]] == [1, 2]
    assert state["status"] == "running"


def test_set_mode_while_running_leaves_the_waypoint_list_unchanged(
    follower: tuple[RouteFollower, Captured],
) -> None:
    module, captured = follower
    module._on_route_cmd(_cmd({"op": "set_mode", "enabled": True}))
    module._on_viewer_click(_click(x=1.0, y=0.0))
    module._on_route_cmd(_cmd({"op": "start"}))

    module._on_route_cmd(_cmd({"op": "set_mode", "enabled": False}))

    state = _state(captured)
    assert state["add_mode"] is False
    assert [w["index"] for w in state["waypoints"]] == [1]
    assert state["status"] == "running"


def test_state_is_renumbered_after_remove(follower: tuple[RouteFollower, Captured]) -> None:
    module, captured = follower
    module._on_route_cmd(_cmd({"op": "set_mode", "enabled": True}))
    module._on_viewer_click(_click(x=1.0, y=0.0))
    module._on_viewer_click(_click(x=2.0, y=0.0))
    module._on_viewer_click(_click(x=3.0, y=0.0))

    module._on_route_cmd(_cmd({"op": "remove", "index": 2}))

    state = _state(captured)
    assert [(w["index"], w["x"]) for w in state["waypoints"]] == [(1, 1.0), (2, 3.0)]


def _twist(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> Twist:
    return Twist(linear=Vector3(x, y, z), angular=Vector3(0.0, 0.0, 0.0))


def test_stop_publishes_stop_and_ends_route_stopped(
    follower: tuple[RouteFollower, Captured],
) -> None:
    module, captured = follower
    module._on_route_cmd(_cmd({"op": "set_mode", "enabled": True}))
    module._on_viewer_click(_click(x=1.0, y=0.0))
    module._on_viewer_click(_click(x=2.0, y=0.0))
    module._on_viewer_click(_click(x=3.0, y=0.0))
    module._on_route_cmd(_cmd({"op": "start"}))

    module._on_route_cmd(_cmd({"op": "stop"}))

    assert len(captured.route_stop) == 1
    assert captured.route_stop[0].data is True
    state = _state(captured)
    assert state["status"] == "stopped"
    assert [w["state"] for w in state["waypoints"]] == ["current", "pending", "pending"]

    module._on_stop_seen(Bool(data=True))
    assert _state(captured)["status"] == "stopped"


def test_stop_pending_guard_ignores_the_planner_failure(
    follower: tuple[RouteFollower, Captured],
) -> None:
    module, captured = follower
    module._on_route_cmd(_cmd({"op": "set_mode", "enabled": True}))
    module._on_viewer_click(_click(x=1.0, y=0.0))
    module._on_route_cmd(_cmd({"op": "start"}))
    module._on_route_cmd(_cmd({"op": "stop"}))

    module._on_goal_reached(Bool(data=False))

    assert _state(captured)["status"] == "stopped"


def test_outside_stop_ends_a_running_route_interrupted(
    follower: tuple[RouteFollower, Captured],
) -> None:
    module, captured = follower
    module._on_route_cmd(_cmd({"op": "set_mode", "enabled": True}))
    module._on_viewer_click(_click(x=1.0, y=0.0))
    module._on_route_cmd(_cmd({"op": "start"}))

    module._on_stop_seen(Bool(data=True))

    assert _state(captured)["status"] == "interrupted"


def test_idle_zero_twist_stop_is_ignored(
    follower: tuple[RouteFollower, Captured],
) -> None:
    module, captured = follower
    module._on_route_cmd(_cmd({"op": "set_mode", "enabled": True}))
    module._on_viewer_click(_click(x=1.0, y=0.0))
    module._on_route_cmd(_cmd({"op": "start"}))

    module._on_tele_cmd_seen(_twist())
    module._on_stop_seen(Bool(data=True))
    assert _state(captured)["status"] == "running"

    module._on_tele_cmd_seen(_twist(x=0.3))
    module._on_stop_seen(Bool(data=True))
    assert _state(captured)["status"] == "interrupted"


def test_outside_goal_ends_a_running_route_interrupted(
    follower: tuple[RouteFollower, Captured],
) -> None:
    module, captured = follower
    module._on_route_cmd(_cmd({"op": "set_mode", "enabled": True}))
    module._on_viewer_click(_click(x=1.0, y=0.0))
    module._on_route_cmd(_cmd({"op": "start"}))

    module._on_goal_seen(PoseStamped(ts=50.0, frame_id="world", position=[9.0, 9.0, 0.0]))

    assert _state(captured)["status"] == "interrupted"


def test_own_goal_echo_does_not_interrupt(
    follower: tuple[RouteFollower, Captured],
) -> None:
    module, captured = follower
    module._on_route_cmd(_cmd({"op": "set_mode", "enabled": True}))
    module._on_viewer_click(_click(x=1.0, y=0.0))
    module._on_route_cmd(_cmd({"op": "start"}))

    module._on_goal_seen(captured.route_goal[0])

    assert _state(captured)["status"] == "running"


def test_stop_publishes_in_every_status(follower: tuple[RouteFollower, Captured]) -> None:
    module, captured = follower
    module._on_route_cmd(_cmd({"op": "set_mode", "enabled": True}))

    module._on_route_cmd(_cmd({"op": "stop"}))

    assert len(captured.route_stop) == 1
    assert captured.route_stop[0].data is True
    assert _state(captured)["status"] == "planning"


def test_watchdog_holds_a_running_route_and_stops_advancing(
    follower: tuple[RouteFollower, Captured], monkeypatch: pytest.MonkeyPatch
) -> None:
    module, captured = follower
    now = [0.0]
    monkeypatch.setattr(module, "_clock", lambda: now[0])
    module._on_route_cmd(_cmd({"op": "set_mode", "enabled": True}))
    module._on_viewer_click(_click(x=0.0, y=0.0))
    module._on_viewer_click(_click(x=5.0, y=5.0))
    module._on_route_cmd(_cmd({"op": "start"}))
    assert len(captured.route_goal) == 1

    now[0] = 2.5
    module._maintenance_tick()
    assert _state(captured)["status"] == "running"

    now[0] = 3.5
    module._maintenance_tick()
    state = _state(captured)
    assert state["status"] == "held"
    assert state["add_mode"] is False

    module._on_odom(_odom(0.05, 0.0))
    state = _state(captured)
    assert len(captured.route_goal) == 1
    assert state["status"] == "held"
    assert [w["state"] for w in state["waypoints"]] == ["reached", "current"]


def test_keepalive_resets_the_watchdog_timer(
    follower: tuple[RouteFollower, Captured], monkeypatch: pytest.MonkeyPatch
) -> None:
    module, captured = follower
    now = [0.0]
    monkeypatch.setattr(module, "_clock", lambda: now[0])
    module._on_route_cmd(_cmd({"op": "set_mode", "enabled": True}))
    module._on_viewer_click(_click(x=5.0, y=5.0))
    module._on_route_cmd(_cmd({"op": "start"}))

    now[0] = 2.5
    module._on_route_cmd(_cmd({"op": "keepalive"}))
    now[0] = 3.5
    module._maintenance_tick()
    assert _state(captured)["status"] == "running"

    now[0] = 5.5
    module._maintenance_tick()
    assert _state(captured)["status"] == "held"


def test_silence_resets_add_mode_without_a_running_route(
    follower: tuple[RouteFollower, Captured], monkeypatch: pytest.MonkeyPatch
) -> None:
    module, captured = follower
    now = [0.0]
    monkeypatch.setattr(module, "_clock", lambda: now[0])
    module._on_route_cmd(_cmd({"op": "set_mode", "enabled": True}))
    assert _state(captured)["add_mode"] is True

    now[0] = 3.5
    module._maintenance_tick()

    state = _state(captured)
    assert state["add_mode"] is False
    assert state["status"] == "planning"


def test_start_resumes_a_held_route_from_the_first_unreached_waypoint(
    follower: tuple[RouteFollower, Captured], monkeypatch: pytest.MonkeyPatch
) -> None:
    module, captured = follower
    now = [0.0]
    monkeypatch.setattr(module, "_clock", lambda: now[0])
    module._on_route_cmd(_cmd({"op": "set_mode", "enabled": True}))
    module._on_viewer_click(_click(x=0.0, y=0.0))
    module._on_viewer_click(_click(x=2.0, y=0.0))
    module._on_viewer_click(_click(x=5.0, y=5.0))
    module._on_route_cmd(_cmd({"op": "start"}))

    now[0] = 3.5
    module._maintenance_tick()
    module._on_odom(_odom(0.05, 0.0))
    assert len(captured.route_goal) == 1

    now[0] = 4.0
    module._on_route_cmd(_cmd({"op": "start"}))

    assert len(captured.route_goal) == 2
    assert (captured.route_goal[1].x, captured.route_goal[1].y) == (2.0, 0.0)
    assert _state(captured)["status"] == "running"


def test_goal_reached_false_marks_the_current_waypoint_failed(
    follower: tuple[RouteFollower, Captured],
) -> None:
    module, captured = follower
    module._on_route_cmd(_cmd({"op": "set_mode", "enabled": True}))
    module._on_viewer_click(_click(x=0.0, y=0.0))
    module._on_viewer_click(_click(x=5.0, y=5.0))
    module._on_route_cmd(_cmd({"op": "start"}))
    assert len(captured.route_goal) == 1

    module._on_goal_reached(Bool(data=False))

    state = _state(captured)
    assert state["status"] == "failed"
    assert [w["state"] for w in state["waypoints"]] == ["failed", "pending"]
    assert state["message"] == "waypoint 1 could not be reached"
    assert len(captured.route_goal) == 1


def test_failure_on_a_middle_waypoint_behaves_the_same(
    follower: tuple[RouteFollower, Captured],
) -> None:
    module, captured = follower
    module._on_route_cmd(_cmd({"op": "set_mode", "enabled": True}))
    module._on_viewer_click(_click(x=0.0, y=0.0))
    module._on_viewer_click(_click(x=5.0, y=5.0))
    module._on_viewer_click(_click(x=9.0, y=9.0))
    module._on_route_cmd(_cmd({"op": "start"}))
    module._on_odom(_odom(0.05, 0.0))
    assert len(captured.route_goal) == 2

    module._on_goal_reached(Bool(data=False))

    state = _state(captured)
    assert state["status"] == "failed"
    assert [w["state"] for w in state["waypoints"]] == ["reached", "failed", "pending"]
    assert state["message"] == "waypoint 2 could not be reached"
    assert len(captured.route_goal) == 2


def test_failed_state_is_published_within_one_state_publication(
    follower: tuple[RouteFollower, Captured],
) -> None:
    module, captured = follower
    module._on_route_cmd(_cmd({"op": "set_mode", "enabled": True}))
    module._on_viewer_click(_click(x=1.0, y=0.0))
    module._on_route_cmd(_cmd({"op": "start"}))

    module._on_goal_reached(Bool(data=False))

    assert json.loads(captured.route_state[-1].data)["status"] == "failed"
