# Copyright 2025-2026 Dimensional Inc.
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

"""MicroduckSkillContainer with a scripted navigation stub and captured outputs."""

from __future__ import annotations

from collections.abc import Callable, Iterator
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import pickle
import threading
import time
from typing import Any

import pytest

from dimos.core.stream import Stream, Transport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.base import NavigationState
from dimos.robot.pollen.microduck import skills
from dimos.robot.pollen.microduck.places import (
    MICRODUCK_OBJECTS,
    MICRODUCK_ROOMS,
    PlacesMemory,
    RoomSpec,
    scene_id,
)
from dimos.robot.pollen.microduck.skills import MicroduckSkillContainer

FOLLOWING = NavigationState.FOLLOWING_PATH
IDLE = NavigationState.IDLE


class _DirectTransport(Transport):  # type: ignore[type-arg]
    """Synchronous in-process transport so ``start()`` can wire the inputs."""

    def __init__(self) -> None:
        self._subscribers: list[Callable[[Any], Any]] = []

    def broadcast(self, _selfstream: Any, value: Any) -> None:
        for callback in list(self._subscribers):
            callback(value)

    def subscribe(
        self, callback: Callable[[Any], Any], _selfstream: Stream[Any] | None = None
    ) -> Callable[[], None]:
        self._subscribers.append(callback)

        def _unsubscribe() -> None:
            if callback in self._subscribers:
                self._subscribers.remove(callback)

        return _unsubscribe

    def start(self) -> None: ...

    def stop(self) -> None:
        self._subscribers.clear()


class FakeNav:
    """Scripted ``NavigationInterfaceSpec`` stand-in.

    ``get_state`` walks through ``states`` (the last one repeats);
    ``is_goal_reached`` turns true once the script is exhausted when
    ``reached`` is set.
    """

    def __init__(
        self, states: tuple[NavigationState, ...] = (FOLLOWING, FOLLOWING), reached: bool = True
    ) -> None:
        self.states = list(states)
        self.reached = reached
        self.calls = 0
        self.goals: list[PoseStamped] = []
        self.cancel_calls = 0
        self.cancel_result = True
        self.set_goal_result = True

    def script(self, *states: NavigationState, reached: bool) -> None:
        self.states = list(states)
        self.reached = reached
        self.calls = 0

    def set_goal(self, goal: PoseStamped) -> bool:
        self.goals.append(goal)
        return self.set_goal_result

    def get_state(self) -> NavigationState:
        state = self.states[min(self.calls, len(self.states) - 1)]
        self.calls += 1
        return state

    def is_goal_reached(self) -> bool:
        return self.reached and self.calls >= len(self.states)

    def cancel_goal(self) -> bool:
        self.cancel_calls += 1
        return self.cancel_result


def _policy_state(**overrides: Any) -> dict[str, Any]:
    policies = [
        {"name": "walk", "kind": "base", "available": True, "reason": None},
        {"name": "stand", "kind": "base", "available": True, "reason": None},
        {"name": "roller", "kind": "base", "available": False, "reason": "rollers variant only"},
        {
            "name": "roller_crouch",
            "kind": "base",
            "available": False,
            "reason": "rollers variant only",
        },
        {"name": "sitstand", "kind": "posture", "available": True, "reason": None},
        {"name": "kick_left", "kind": "oneshot", "available": True, "reason": None},
        {"name": "kick_right", "kind": "oneshot", "available": True, "reason": None},
        {"name": "roulade", "kind": "oneshot", "available": True, "reason": None},
        {"name": "ground_pick", "kind": "oneshot", "available": True, "reason": None},
    ]
    state: dict[str, Any] = {
        "variant": "default",
        "active": "walk",
        "base": "walk",
        "seated": False,
        "fallen": False,
        "locked": False,
        "oneshot": None,
        "policies": policies,
        "last_error": None,
        "t": time.time(),
    }
    state.update(overrides)
    return state


@dataclass
class Captured:
    goals: list[PoseStamped] = field(default_factory=list)
    policy_requests: list[str] = field(default_factory=list)
    places: list[str] = field(default_factory=list)

    @property
    def last_request(self) -> dict[str, Any]:
        return json.loads(self.policy_requests[-1])

    @property
    def last_places(self) -> dict[str, Any]:
        return json.loads(self.places[-1])


@dataclass
class Harness:
    module: MicroduckSkillContainer
    nav: FakeNav
    captured: Captured
    timers: list[threading.Timer] = field(default_factory=list)

    def feed_odom(self, x: float, y: float, yaw: float = 0.0) -> None:
        pose = PoseStamped(
            ts=time.time(),
            frame_id="world",
            position=Vector3(x, y, 0.0),
            orientation=Quaternion(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)),
        )
        self.module.odom.transport.broadcast(None, pose)

    def feed_state(self, **overrides: Any) -> None:
        self.module.policy_state.transport.broadcast(
            None, json.dumps(_policy_state(**overrides), separators=(",", ":"))
        )

    def feed_state_later(self, delay: float, **overrides: Any) -> None:
        timer = threading.Timer(delay, lambda: self.feed_state(**overrides))
        timer.daemon = True
        self.timers.append(timer)
        timer.start()

    def on_request(self, callback: Callable[[dict[str, Any]], None]) -> Callable[[], None]:
        """Run ``callback`` synchronously whenever a policy_request is published."""
        return self.module.policy_request.subscribe(lambda msg: callback(json.loads(msg)))

    def close(self) -> None:
        for timer in self.timers:
            timer.cancel()
            timer.join(timeout=2.0)


@pytest.fixture
def fast_polling(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(skills, "_NAV_POLL_S", 0.005)
    monkeypatch.setattr(skills, "_NAV_START_TIMEOUT_S", 0.2)
    monkeypatch.setattr(skills, "_NAV_SETTLE_S", 0.05)
    monkeypatch.setattr(skills, "_POLICY_START_GRACE_S", 0.3)
    monkeypatch.setattr(skills, "_POLICY_FIRST_STATE_WAIT_S", 0.2)


def _make_module(tmp_path: Path, **overrides: Any) -> MicroduckSkillContainer:
    config: dict[str, Any] = {
        "rooms": MICRODUCK_ROOMS,
        "objects": MICRODUCK_OBJECTS,
        "places_db": str(tmp_path / "places.db"),
        "nav_timeout_s": 1.0,
        "policy_timeout_s": 1.0,
        "places_republish_s": 0.0,
    }
    config.update(overrides)
    return MicroduckSkillContainer(**config)


@pytest.fixture
def harness(tmp_path: Path, fast_polling: None) -> Iterator[Harness]:
    module = _make_module(tmp_path)
    module.odom.transport = _DirectTransport()
    module.policy_state.transport = _DirectTransport()
    nav = FakeNav()
    module._navigation = nav  # type: ignore[assignment]
    captured = Captured()
    unsubs = [
        module.goal_request.subscribe(captured.goals.append),
        module.policy_request.subscribe(captured.policy_requests.append),
        module.places.subscribe(captured.places.append),
    ]
    module.start()
    h = Harness(module, nav, captured)
    try:
        yield h
    finally:
        h.close()
        for unsub in unsubs:
            unsub()
        module.stop()


# --------------------------------------------------------------------------
# lifecycle / config
# --------------------------------------------------------------------------


def test_blueprint_keeps_objects_kwarg_and_config_pickles(tmp_path: Path) -> None:
    bp = MicroduckSkillContainer.blueprint(objects={"red_box": (1.5, 0.8)})
    assert bp is not None
    module = _make_module(tmp_path)
    try:
        assert module.config.rooms["kitchen"].aliases == ("space A",)
        assert module.config.objects["red_box"] == (1.5, 1.5)
        assert module.config.places_db.endswith("places.db")
        assert module.config.nav_timeout_s == 1.0
        assert module.config.scene == ""
        assert module._scene() == scene_id(MICRODUCK_ROOMS, MICRODUCK_OBJECTS)
        restored = pickle.loads(pickle.dumps(module.config))
        assert restored.rooms == MICRODUCK_ROOMS
    finally:
        module.stop()


def test_blueprints_with_different_tables_share_db_without_leaking(
    tmp_path: Path, fast_polling: None
) -> None:
    """The cockpit (4 rooms) and the old agentic sim (2 objects) use one db file."""
    db = str(tmp_path / "shared.db")
    cockpit = _make_module(tmp_path, places_db=db)
    cockpit.odom.transport = _DirectTransport()
    cockpit._navigation = FakeNav()  # type: ignore[assignment]
    cockpit.start()
    try:
        cockpit.odom.transport.broadcast(
            None,
            PoseStamped(
                ts=time.time(),
                frame_id="world",
                position=Vector3(0.3, -0.2, 0.0),
                orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
            ),
        )
        assert cockpit.remember_place("charger").startswith("Remembered 'charger'")
    finally:
        cockpit.stop()

    legacy_objects = {"red_box": (1.5, 0.8), "blue_box": (-1.5, -0.8)}
    legacy = _make_module(tmp_path, places_db=db, rooms={}, objects=legacy_objects)
    legacy.odom.transport = _DirectTransport()
    legacy._navigation = FakeNav()  # type: ignore[assignment]
    received: list[str] = []
    unsub = legacy.places.subscribe(received.append)
    legacy.start()
    try:
        assert legacy._scene() != scene_id(MICRODUCK_ROOMS, MICRODUCK_OBJECTS)
        payload = json.loads(received[-1])
        assert payload["rooms"] == []
        assert [(o["name"], o["x"], o["y"]) for o in payload["objects"]] == [
            ("red_box", 1.5, 0.8),
            ("blue_box", -1.5, -0.8),
        ]
        assert payload["tagged"] == []
        listing = legacy.list_places()
        assert "Rooms:" not in listing and "charger" not in listing
        assert "- red_box at (1.50, 0.80)" in listing
        assert legacy.go_to_place("charger").startswith("Unknown place 'charger'")
    finally:
        unsub()
        legacy.stop()

    # An explicit scene id overrides the derived one.
    pinned = _make_module(tmp_path, places_db=db, scene="cockpit-a")
    try:
        assert pinned._scene() == "cockpit-a"
    finally:
        pinned.stop()


def test_stream_declarations() -> None:
    module = MicroduckSkillContainer()
    try:
        assert module.odom.type is PoseStamped
        assert module.policy_state.type is str
        assert module.goal_request.type is PoseStamped
        assert module.policy_request.type is str
        assert module.places.type is str
    finally:
        module.stop()


def test_publishes_places_on_start(harness: Harness) -> None:
    assert len(harness.captured.places) == 1
    payload = harness.captured.last_places
    assert payload["frame"] == "world"
    assert [r["name"] for r in payload["rooms"]] == list(MICRODUCK_ROOMS)
    assert [o["name"] for o in payload["objects"]] == list(MICRODUCK_OBJECTS)
    assert payload["tagged"] == []
    assert isinstance(payload["t"], float)
    db = Path(harness.module.config.places_db)
    assert db.is_file()


def test_places_heartbeat_republishes(tmp_path: Path, fast_polling: None) -> None:
    module = _make_module(tmp_path, places_republish_s=0.02)
    module.odom.transport = _DirectTransport()
    module.policy_state.transport = _DirectTransport()
    module._navigation = FakeNav()  # type: ignore[assignment]
    received: list[str] = []
    unsub = module.places.subscribe(received.append)
    module.start()
    try:
        deadline = time.monotonic() + 2.0
        while len(received) < 3 and time.monotonic() < deadline:
            time.sleep(0.01)
        assert len(received) >= 3
        thread = module._republish_thread
        assert thread is not None and thread.is_alive()
    finally:
        unsub()
        module.stop()
    assert module._republish_thread is None
    assert not thread.is_alive()


def test_start_without_policy_state_transport(tmp_path: Path, fast_polling: None) -> None:
    """Old blueprints have no policy_state producer; start must still work."""
    module = _make_module(tmp_path, rooms={}, objects={"red_box": (1.5, 0.8)})
    module.odom.transport = _DirectTransport()
    module._navigation = FakeNav()  # type: ignore[assignment]
    requests: list[str] = []
    unsub = module.policy_request.subscribe(requests.append)
    module.start()
    try:
        assert "red_box" in module.list_places()
        assert "Rooms:" not in module.list_places()
        out = module.perform("kick_left")
        assert out.startswith("Sent 'kick_left' start request")
        assert json.loads(requests[-1])["policy"] == "kick_left"
    finally:
        unsub()
        module.stop()


def test_places_memory_is_not_reopened_after_stop(tmp_path: Path, fast_polling: None) -> None:
    """A skill thread still unwinding after stop() must not reopen the
    database stop() just closed (that handle would leak)."""
    module = _make_module(tmp_path)
    module.odom.transport = _DirectTransport()
    module._navigation = FakeNav()  # type: ignore[assignment]
    module.start()
    assert module._memory is not None
    module.stop()
    assert module._memory is None
    with pytest.raises(RuntimeError, match="stopped"):
        module._places_memory()
    assert module._memory is None
    # Publishing is a no-op once stopped, and the arrival summary degrades
    # instead of raising into the skill's caller.
    module._publish_places()
    assert module._memory is None
    pose = PoseStamped(ts=time.time(), frame_id="world", position=Vector3(0.3, -0.2, 0.0))
    module._on_odom(pose)
    assert module._where_summary() == "Robot was at (0.30, -0.20) (module stopping)."
    assert module._memory is None
    # start() re-arms it.
    module.start()
    try:
        assert module._memory is not None
        assert module._where_summary() == "Robot is now at (0.30, -0.20) in the central hub."
    finally:
        module.stop()


def test_scene_without_rooms_reports_no_hub(tmp_path: Path, fast_polling: None) -> None:
    """The legacy agentic-sim scene has objects but no rooms: no 'central
    hub' phrasing, and move_to's bounds follow the objects."""
    module = _make_module(tmp_path, rooms={}, objects={"red_box": (1.5, 0.8)})
    module.odom.transport = _DirectTransport()
    module._navigation = FakeNav()  # type: ignore[assignment]
    module.start()
    try:
        pose = PoseStamped(ts=time.time(), frame_id="world", position=Vector3(0.1, -0.1, 0.0))
        module._on_odom(pose)
        out = module.where_am_i()
        assert out.startswith("Robot is at (0.10, -0.10), heading 0 deg.")
        assert "hub" not in out and "room" not in out
        assert module.remember_place("charger") == "Remembered 'charger' at (0.10, -0.10)."
        assert module.move_to(0.5, 0.5).endswith("Robot is now at (0.10, -0.10).")
        assert module.go_to_room("kitchen") == "Unknown room 'kitchen'. Known rooms: (none)"
        assert module._arena_half_extent() == pytest.approx(2.0)
    finally:
        module.stop()


def test_move_to_extent_follows_larger_scenes(tmp_path: Path, fast_polling: None) -> None:
    hall = RoomSpec("hall", ("space Z",), (-6.0, 3.0, -1.0, 1.0), (0.0, 0.0, 0.0))
    module = _make_module(tmp_path, rooms={"hall": hall}, objects={"far_box": (2.0, 4.5)})
    module.odom.transport = _DirectTransport()
    module._navigation = FakeNav()  # type: ignore[assignment]
    module.start()
    try:
        assert module._arena_half_extent() == pytest.approx(6.0)
        assert module.move_to(-5.5, 0.0).startswith("Arrived.")
        assert module.move_to(0.0, 4.4).startswith("Arrived.")
        out = module.move_to(6.5, 0.0)
        assert out == "(6.50, 0.00) is outside the arena walls (x and y must be within +/-6 m)."
    finally:
        module.stop()


def test_remember_place_respelled_updates_instead_of_duplicating(harness: Harness) -> None:
    harness.feed_odom(0.3, -0.2)
    assert harness.module.remember_place("charger").startswith("Remembered 'charger'")
    harness.feed_odom(0.5, -0.1)
    assert harness.module.remember_place("Charger").startswith("Remembered 'Charger'")
    tagged = harness.captured.last_places["tagged"]
    assert [(t["name"], t["x"]) for t in tagged] == [("Charger", 0.5)]
    out = harness.module.go_to_place("charger")
    assert out.startswith("Arrived while walking to Charger.")
    assert (harness.captured.goals[-1].position.x, harness.captured.goals[-1].position.y) == (
        pytest.approx(0.5),
        pytest.approx(-0.1),
    )


# --------------------------------------------------------------------------
# navigation skills
# --------------------------------------------------------------------------


def test_go_to_room_by_alias_publishes_goal_and_arrives(harness: Harness) -> None:
    harness.feed_odom(0.0, 0.0)
    out = harness.module.go_to_room("space A")
    assert out.startswith("Arrived while walking to the kitchen.")
    assert "central hub" in out
    assert len(harness.captured.goals) == 1
    goal = harness.captured.goals[0]
    assert goal.frame_id == "world"
    assert (goal.position.x, goal.position.y) == pytest.approx((1.2, 1.0))
    assert goal.yaw == pytest.approx(0.0, abs=1e-6)
    # No transport on goal_request in this harness -> RPC fallback delivers it too.
    assert len(harness.nav.goals) == 1


def test_goal_request_transport_disables_rpc_fallback(harness: Harness) -> None:
    transport = _DirectTransport()
    seen: list[PoseStamped] = []
    transport.subscribe(seen.append)
    harness.module.goal_request.transport = transport
    harness.feed_odom(0.0, 0.0)
    out = harness.module.go_to_room("living room")
    assert out.startswith("Arrived")
    assert len(seen) == 1 and seen[0].position.x == pytest.approx(-1.2)
    assert seen[0].yaw == pytest.approx(3.14159, abs=1e-4)
    assert harness.nav.goals == []


def test_go_to_room_unknown_lists_rooms(harness: Harness) -> None:
    out = harness.module.go_to_room("garage")
    assert out.startswith("Unknown room 'garage'")
    assert "kitchen (space A)" in out
    assert "living (space B, living room, lounge)" in out
    assert harness.captured.goals == []


def test_go_to_room_already_at_entry_point(harness: Harness) -> None:
    harness.feed_odom(1.15, 1.0)
    out = harness.module.go_to_room("kitchen")
    assert out.startswith("Already at the kitchen entry point.")
    assert harness.captured.goals == []


def test_go_to_object_keeps_approach_offset(harness: Harness) -> None:
    harness.feed_odom(0.0, 0.0)
    out = harness.module.go_to_object("red box")
    assert out.startswith("Arrived while walking to red_box.")
    goal = harness.captured.goals[-1]
    heading = math.atan2(1.5, 1.5)
    assert goal.position.x == pytest.approx(1.5 - math.cos(heading) * 0.35)
    assert goal.position.y == pytest.approx(1.5 - math.sin(heading) * 0.35)
    assert goal.yaw == pytest.approx(heading, abs=1e-6)


def test_go_to_object_already_next_to(harness: Harness) -> None:
    harness.feed_odom(1.3, 1.5)
    out = harness.module.go_to_object("red_box")
    assert out.startswith("Already next to red_box (0.20 m away).")
    assert harness.captured.goals == []


def test_go_to_object_needs_odom_and_known_name(harness: Harness) -> None:
    assert harness.module.go_to_object("red_box").startswith("Robot position unknown")
    out = harness.module.go_to_object("purple_sphere")
    assert out.startswith("Unknown object 'purple_sphere'")
    assert "red_box" in out and "orange_crate" in out
    assert harness.captured.goals == []


def test_ambiguous_names_ask_instead_of_guessing(harness: Harness) -> None:
    harness.feed_odom(0.0, 0.0)
    out = harness.module.go_to_object("box")
    assert out == "Ambiguous object 'box': did you mean red_box, blue_box? Use the exact name."
    out = harness.module.go_to_place("box")
    assert out == "Ambiguous place 'box': did you mean red_box, blue_box? Use the exact name."
    # "space" / "room" are filler words: on their own they match nothing.
    assert harness.module.go_to_room("space").startswith("Unknown room 'space'")
    assert harness.module.go_to_room("room").startswith("Unknown room 'room'")
    assert harness.module.go_to_place("space").startswith("Unknown place 'space'")
    assert harness.captured.goals == []
    # A remembered spot literally called "box" is exact and wins.
    harness.feed_odom(0.2, 0.1)
    assert harness.module.remember_place("box").startswith("Remembered 'box'")
    harness.feed_odom(0.0, 0.0)
    assert harness.module.go_to_place("box").startswith("Arrived while walking to box.")
    assert len(harness.captured.goals) == 1


def test_go_to_place_routes_by_kind(harness: Harness) -> None:
    harness.feed_odom(0.0, 0.0)
    assert harness.module.go_to_place("lounge").startswith("Arrived while walking to the living.")
    assert harness.captured.goals[-1].position.x == pytest.approx(-1.2)

    harness.nav.script(FOLLOWING, FOLLOWING, reached=True)
    assert harness.module.go_to_place("blue box").startswith("Arrived while walking to blue_box.")
    goal = harness.captured.goals[-1]
    assert math.hypot(goal.position.x + 1.5, goal.position.y - 1.5) == pytest.approx(0.35)

    harness.feed_odom(0.3, -0.2, 0.7)
    assert harness.module.remember_place("charger").startswith("Remembered 'charger'")
    harness.feed_odom(0.0, 0.0)
    harness.nav.script(FOLLOWING, FOLLOWING, reached=True)
    out = harness.module.go_to_place("charger")
    assert out.startswith("Arrived while walking to charger.")
    goal = harness.captured.goals[-1]
    assert (goal.position.x, goal.position.y) == pytest.approx((0.3, -0.2))
    assert goal.yaw == pytest.approx(0.7, abs=1e-6)

    assert harness.module.go_to_place("nowhere").startswith("Unknown place 'nowhere'")


def test_move_to_validates_and_faces_goal(harness: Harness) -> None:
    harness.feed_odom(0.0, 0.0)
    out = harness.module.move_to(0.5, -0.5)
    assert out.startswith("Arrived.")
    goal = harness.captured.goals[-1]
    assert (goal.position.x, goal.position.y) == pytest.approx((0.5, -0.5))
    assert goal.yaw == pytest.approx(-math.pi / 4, abs=1e-6)

    assert "outside the arena walls" in harness.module.move_to(3.0, 0.0)
    assert "Invalid coordinates" in harness.module.move_to("abc", 0.0)  # type: ignore[arg-type]
    assert "Invalid coordinates" in harness.module.move_to(float("nan"), 0.0)
    assert len(harness.captured.goals) == 1


def test_move_to_without_odom_uses_identity_heading(harness: Harness) -> None:
    out = harness.module.move_to(1.0, 1.0)
    assert out.startswith("Arrived.")
    assert "Robot position unknown." in out
    assert harness.captured.goals[-1].yaw == pytest.approx(0.0, abs=1e-6)


def test_navigation_failure_outcomes(harness: Harness) -> None:
    harness.feed_odom(0.0, 0.0)
    harness.nav.script(IDLE, reached=False)
    assert harness.module.move_to(1.0, 1.0).startswith(
        "Navigation never started following a path (no route found?)"
    )

    harness.nav.script(FOLLOWING, IDLE, reached=False)
    assert harness.module.move_to(1.0, 1.0).startswith(
        "Navigation stopped early (cancelled or no path)"
    )

    harness.nav.script(FOLLOWING, reached=False)
    harness.module.config.nav_timeout_s = 0.15
    t0 = time.monotonic()
    assert harness.module.move_to(1.0, 1.0).startswith("Navigation timed out")
    assert time.monotonic() - t0 < 1.0


def test_rpc_fallback_rejection(harness: Harness) -> None:
    harness.nav.set_goal_result = False
    out = harness.module.move_to(1.0, 1.0)
    assert out.startswith("Navigation rejected the goal")
    assert len(harness.captured.goals) == 1


def test_stop_moving(harness: Harness) -> None:
    assert harness.module.stop_moving() == "Stopped."
    harness.nav.cancel_result = False
    assert harness.module.stop_moving() == "There was no active navigation goal."
    assert harness.nav.cancel_calls == 2


def test_where_am_i(harness: Harness) -> None:
    assert harness.module.where_am_i().startswith("Robot position unknown")
    harness.feed_odom(1.4, 1.4, 0.5)
    out = harness.module.where_am_i()
    assert out.startswith("Robot is at (1.40, 1.40), heading 29 deg, in the kitchen (aka space A).")
    assert "Nearby: red_box (0.14 m)" in out
    assert "kitchen (" not in out.split("Nearby:")[1]
    harness.feed_odom(0.1, -0.1)
    out = harness.module.where_am_i()
    assert "in the central hub (no room)" in out
    assert "Nearby" not in out


def test_remember_place_persists_and_republishes(harness: Harness, tmp_path: Path) -> None:
    assert harness.module.remember_place("charger").startswith("Robot position unknown")
    assert (
        harness.module.remember_place("   ")
        == "Give the place a name, e.g. remember_place('charger')."
    )
    harness.feed_odom(0.3, -0.2, 0.25)
    published_before = len(harness.captured.places)
    out = harness.module.remember_place("charger")
    assert out == "Remembered 'charger' at (0.30, -0.20) in the central hub."
    assert len(harness.captured.places) == published_before + 1
    tagged = harness.captured.last_places["tagged"]
    assert len(tagged) == 1
    assert tagged[0]["name"] == "charger"
    assert (tagged[0]["x"], tagged[0]["y"]) == pytest.approx((0.3, -0.2))
    assert tagged[0]["yaw"] == pytest.approx(0.25, abs=1e-6)
    assert "- charger at (0.30, -0.20)" in harness.module.list_places()

    # Reserved names are refused (compared on the normalised forms find()
    # matches on, so the spot could never shadow a room or object);
    # re-remembering moves the spot.
    assert harness.module.remember_place("kitchen").startswith(
        "'kitchen' is already the name of a room"
    )
    assert harness.module.remember_place("red_box").startswith(
        "'red_box' is already the name of a object"
    )
    assert harness.module.remember_place("red box").startswith(
        "'red box' is already the name of a object"
    )
    assert harness.module.remember_place("Space A").startswith(
        "'Space A' is already the name of a room"
    )
    assert harness.module.remember_place("the lounge").startswith(
        "'the lounge' is already the name of a room"
    )
    assert len(harness.captured.last_places["tagged"]) == 1
    harness.feed_odom(1.6, 1.0)
    assert harness.module.remember_place("charger").endswith("in the kitchen.")
    assert len(harness.captured.last_places["tagged"]) == 1

    # Survives a restart on the same database (same scene) and is invisible
    # to a memory opened on another scene of that database.
    other = PlacesMemory(tmp_path / "places.db", scene=scene_id(MICRODUCK_ROOMS, MICRODUCK_OBJECTS))
    try:
        rec = other.find("charger")
        assert rec is not None and (rec.x, rec.y) == pytest.approx((1.6, 1.0))
    finally:
        other.close()
    foreign = PlacesMemory(tmp_path / "places.db")
    try:
        assert foreign.find("charger") is None
        assert foreign.all() == []
    finally:
        foreign.close()


def test_list_places(harness: Harness) -> None:
    harness.feed_odom(0.0, 0.0)
    out = harness.module.list_places()
    assert out.startswith("Rooms:")
    assert "- kitchen (aka space A): x 0..2, y 0..2, entry point (1.20, 1.00), 1.56 m away" in out
    assert "- living (aka space B, living room, lounge)" in out
    assert "Objects:" in out
    assert "- red_box at (1.50, 1.50) in the kitchen, 2.12 m away" in out
    assert "- orange_crate at (0.60, 1.70) in the kitchen" in out
    assert "Remembered places:" in out
    assert "(none yet" in out
    assert out.endswith("Robot is now at (0.00, 0.00) in the central hub.")


def test_list_objects(harness: Harness) -> None:
    """Kept for the agentic-sim system prompt, which still tells the agent to call it."""
    out = harness.module.list_objects()
    assert out.startswith("Objects in the room:\n")
    assert "- red_box at (1.50, 1.50) in the kitchen" in out
    assert "Rooms:" not in out and "Remembered" not in out
    assert " m away" not in out
    harness.feed_odom(0.0, 0.0)
    assert "- red_box at (1.50, 1.50) in the kitchen, 2.12 m away" in harness.module.list_objects()


def test_list_objects_without_objects(tmp_path: Path, fast_polling: None) -> None:
    module = _make_module(tmp_path, rooms={}, objects={})
    try:
        assert module.list_objects() == "No objects are registered in this scene."
        assert module.list_places() == "No places are registered in this scene."
    finally:
        module.stop()


def test_wait_clamps(harness: Harness) -> None:
    t0 = time.monotonic()
    assert harness.module.wait(-5) == "Waited 0 s."
    assert harness.module.wait(0.05) == "Waited 0 s."
    assert time.monotonic() - t0 < 1.0
    assert "Invalid duration" in harness.module.wait("soon")  # type: ignore[arg-type]


# --------------------------------------------------------------------------
# policy skills
# --------------------------------------------------------------------------


def test_list_policies_without_and_with_state(harness: Harness) -> None:
    out = harness.module.list_policies()
    assert "- kick_left [oneshot]: kick the ball with the left foot" in out
    assert "has not reported its policy state yet" in out

    harness.feed_state(active="stand", base="stand")
    out = harness.module.list_policies()
    assert "- roller [base]" in out and "NOT available: rollers variant only" in out
    assert "- sitstand [posture]" in out
    assert "Now: active policy 'stand' (base 'stand')." in out


def test_perform_oneshot_success(harness: Harness) -> None:
    harness.feed_state()

    def on_request(req: dict[str, Any]) -> None:
        assert req == {"policy": "kick_left", "action": "start", "t": req["t"]}
        harness.feed_state(
            active="kick_left", locked=True, oneshot={"name": "kick_left", "progress": 0.1}
        )
        harness.feed_state_later(0.05, oneshot=None)

    unsub = harness.on_request(on_request)
    try:
        out = harness.module.perform("kick_left")
    finally:
        unsub()
    assert out.startswith("Finished kick_left.")
    assert "Now: active policy 'walk'" in out
    assert len(harness.captured.policy_requests) == 1


def test_perform_reports_fall_and_aliases(harness: Harness) -> None:
    harness.feed_state()

    def on_request(req: dict[str, Any]) -> None:
        assert req["policy"] == "roulade"
        harness.feed_state(
            active="roulade", locked=True, oneshot={"name": "roulade", "progress": 0.5}
        )
        harness.feed_state_later(0.05, oneshot=None, fallen=True, locked=True)

    unsub = harness.on_request(on_request)
    try:
        out = harness.module.perform("somersault")
    finally:
        unsub()
    assert out.startswith("Finished roulade. The duck fell over and is getting back up.")
    assert "FALLEN" in out


def test_perform_rejected_reports_last_error(harness: Harness) -> None:
    harness.feed_state()

    def on_request(req: dict[str, Any]) -> None:
        harness.feed_state(last_error="rejected: robot is locked")

    unsub = harness.on_request(on_request)
    try:
        out = harness.module.perform("ground_pick")
    finally:
        unsub()
    assert out.startswith("Could not start ground_pick: rejected: robot is locked.")


def test_perform_without_ack_times_out_quickly(harness: Harness) -> None:
    harness.feed_state()
    t0 = time.monotonic()
    out = harness.module.perform("kick_right")
    assert out.startswith("Could not start kick_right: no acknowledgement.")
    assert time.monotonic() - t0 < 1.5


def test_perform_precheck_from_state(harness: Harness) -> None:
    harness.feed_state(
        oneshot={"name": "kick_left", "progress": 0.5}, active="kick_left", locked=True
    )
    assert harness.module.perform("roulade").startswith("The duck is busy with kick_left")
    assert harness.module.perform("roulade", "stop") == (
        "roulade is not running. "
        + harness.module._policy_summary(harness.module._current_policy_state())  # type: ignore[arg-type]
    )
    harness.feed_state(fallen=True, locked=True)
    assert harness.module.perform("kick_left").startswith("The duck has fallen")
    harness.feed_state(seated=True, active="sitstand", locked=True)
    assert harness.module.perform("kick_left") == "The duck is sitting; call stand_up first."
    assert harness.module.perform("roller").startswith(
        "'roller' is not available on this robot (rollers variant only)"
    )
    assert harness.captured.policy_requests == []


def test_perform_stop_oneshot(harness: Harness) -> None:
    harness.feed_state(
        oneshot={"name": "kick_left", "progress": 0.5}, active="kick_left", locked=True
    )

    def on_request(req: dict[str, Any]) -> None:
        assert req["policy"] == "kick_left" and req["action"] == "stop"
        harness.feed_state()

    unsub = harness.on_request(on_request)
    try:
        out = harness.module.perform("kick_left", "stop")
    finally:
        unsub()
    assert out.startswith("Stopped kick_left.")


def test_perform_toggle_oneshot_stops_when_running_and_starts_otherwise(
    harness: Harness,
) -> None:
    harness.feed_state(
        oneshot={"name": "kick_left", "progress": 0.5}, active="kick_left", locked=True
    )

    def on_stop(req: dict[str, Any]) -> None:
        assert req["policy"] == "kick_left" and req["action"] == "stop"
        harness.feed_state()

    unsub = harness.on_request(on_stop)
    try:
        out = harness.module.perform("kick_left", "toggle")
    finally:
        unsub()
    assert out.startswith("Stopped kick_left.")

    def on_start(req: dict[str, Any]) -> None:
        assert req["policy"] == "kick_left" and req["action"] == "start"
        harness.feed_state(
            active="kick_left", locked=True, oneshot={"name": "kick_left", "progress": 0.1}
        )
        harness.feed_state_later(0.05, oneshot=None)

    unsub = harness.on_request(on_start)
    try:
        out = harness.module.perform("kick_left", "toggle")
    finally:
        unsub()
    assert out.startswith("Finished kick_left.")
    assert [json.loads(r)["action"] for r in harness.captured.policy_requests] == ["stop", "start"]

    # Toggling a trick that is not running while another one is stays a start,
    # so the busy pre-check applies.
    harness.feed_state(oneshot={"name": "roulade", "progress": 0.5}, active="roulade", locked=True)
    assert harness.module.perform("kick_left", "toggle").startswith("The duck is busy with roulade")
    assert len(harness.captured.policy_requests) == 2


def test_repeated_identical_rejection_fails_fast(harness: Harness) -> None:
    """The scheduler re-issues the same last_error string on every rejected
    request (only ``t`` changes); that must not look like silence."""
    harness.feed_state(seated=False, locked=True, last_error="rejected: robot is locked")

    def reject(req: dict[str, Any]) -> None:
        harness.feed_state(seated=False, locked=True, last_error="rejected: robot is locked")

    unsub = harness.on_request(reject)
    try:
        t0 = time.monotonic()
        out = harness.module.sit()
        sit_elapsed = time.monotonic() - t0
        t0 = time.monotonic()
        base = harness.module.perform("stand")
        base_elapsed = time.monotonic() - t0
    finally:
        unsub()
    assert out == "Could not sit down: rejected: robot is locked."
    assert base == "Could not switch to stand: rejected: robot is locked."
    # Grace period (0.3 s under fast_polling), not policy_timeout_s (1 s).
    assert sit_elapsed < 0.8 and base_elapsed < 0.8
    assert [json.loads(r)["policy"] for r in harness.captured.policy_requests] == [
        "sitstand",
        "stand",
    ]


def test_stand_up_identical_rejection_fails_fast(harness: Harness) -> None:
    harness.feed_state(seated=True, active="sitstand", locked=True, last_error="locked: seated")

    def reject(req: dict[str, Any]) -> None:
        assert req["action"] == "stop"
        harness.feed_state(seated=True, active="sitstand", locked=True, last_error="locked: seated")

    unsub = harness.on_request(reject)
    try:
        t0 = time.monotonic()
        out = harness.module.stand_up()
        elapsed = time.monotonic() - t0
    finally:
        unsub()
    assert out == "Could not stand up: locked: seated."
    assert elapsed < 0.8


def test_posture_heartbeat_then_settles_within_grace(harness: Harness) -> None:
    """A heartbeat with last_error cleared is not by itself an acknowledgement;
    the posture change that follows within the grace period is."""
    harness.feed_state(last_error="rejected: robot is locked")

    def on_sit(req: dict[str, Any]) -> None:
        harness.feed_state(last_error=None)  # heartbeat, not yet seated
        harness.feed_state_later(0.1, seated=True, active="sitstand", locked=True)

    unsub = harness.on_request(on_sit)
    try:
        out = harness.module.sit()
    finally:
        unsub()
    assert out.startswith("The duck is now sitting.")


def test_posture_heartbeats_without_transition_fail_fast(harness: Harness) -> None:
    """Fresh heartbeats with last_error None (a request the scheduler never
    saw) do not count as acknowledgement: 'no acknowledgement' after the
    grace period instead of waiting the full policy_timeout_s."""
    harness.feed_state()
    stop = threading.Event()

    def heartbeat() -> None:
        while not stop.wait(0.02):
            harness.feed_state(last_error=None)

    thread = threading.Thread(target=heartbeat, daemon=True)
    thread.start()
    t0 = time.monotonic()
    try:
        out = harness.module.sit()
    finally:
        stop.set()
        thread.join(timeout=2.0)
    elapsed = time.monotonic() - t0
    assert out == "Could not sit down: no acknowledgement."
    assert 0.25 <= elapsed < 0.8, elapsed


def test_base_switch_heartbeats_without_transition_fail_fast(harness: Harness) -> None:
    harness.feed_state()

    def on_request(req: dict[str, Any]) -> None:
        harness.feed_state(last_error=None)  # heartbeat only, base unchanged

    unsub = harness.on_request(on_request)
    t0 = time.monotonic()
    try:
        out = harness.module.perform("stand")
    finally:
        unsub()
    assert out == "Could not switch to stand: no acknowledgement."
    assert time.monotonic() - t0 < 0.8


def test_short_oneshot_started_and_finished_between_polls(harness: Harness) -> None:
    """Start and end states arriving back-to-back (before the skill polls once)
    still count as the trick having run."""
    harness.feed_state()

    def on_request(req: dict[str, Any]) -> None:
        harness.feed_state(
            active="kick_left", locked=True, oneshot={"name": "kick_left", "progress": 0.5}
        )
        harness.feed_state(oneshot=None)  # already over

    unsub = harness.on_request(on_request)
    t0 = time.monotonic()
    try:
        out = harness.module.perform("kick_left")
    finally:
        unsub()
    assert out.startswith("Finished kick_left.")
    assert "Now: active policy 'walk'" in out
    assert time.monotonic() - t0 < 0.25


def test_posture_settled_between_polls(harness: Harness) -> None:
    """Sit begun and settled in states that arrive before the first poll."""
    harness.feed_state(seated=True, active="sitstand", locked=True)

    def on_stand(req: dict[str, Any]) -> None:
        harness.feed_state(seated=False, active="standing_up", locked=True)
        harness.feed_state(seated=False, active="walk")

    unsub = harness.on_request(on_stand)
    try:
        out = harness.module.stand_up()
    finally:
        unsub()
    assert out.startswith("The duck is now standing.")


def test_policy_history_is_bounded(harness: Harness) -> None:
    for _ in range(skills._POLICY_HISTORY + 20):
        harness.feed_state()
    assert len(harness.module._policy_history) == skills._POLICY_HISTORY
    seqs = [seq for seq, _ in harness.module._policy_history]
    assert seqs == sorted(seqs) and seqs[-1] == harness.module._policy_state_seq


def test_posture_error_after_acknowledgement(harness: Harness) -> None:
    harness.feed_state()

    def on_sit(req: dict[str, Any]) -> None:
        harness.feed_state(active="sitstand", locked=True)  # begun
        harness.feed_state_later(0.05, active="walk", locked=True, last_error="fell during sit")

    unsub = harness.on_request(on_sit)
    try:
        out = harness.module.sit()
    finally:
        unsub()
    assert out == "Could not sit down: fell during sit."


def test_posture_stale_state_only_times_out(harness: Harness) -> None:
    """No state newer than the request at all: the grace period expires with
    'no acknowledgement' rather than the full policy timeout."""
    harness.feed_state()
    t0 = time.monotonic()
    out = harness.module.sit()
    assert out == "Could not sit down: no acknowledgement."
    assert time.monotonic() - t0 < 0.8


def test_base_switch_acknowledged_via_braking(harness: Harness) -> None:
    harness.feed_state()

    def on_request(req: dict[str, Any]) -> None:
        harness.feed_state(active="braking", base="walk", locked=True)
        harness.feed_state_later(0.05, active="stand", base="stand")

    unsub = harness.on_request(on_request)
    try:
        out = harness.module.perform("stand")
    finally:
        unsub()
    assert out.startswith("Base policy is now stand.")


def test_sit_and_stand_up(harness: Harness) -> None:
    harness.feed_state()

    def on_sit(req: dict[str, Any]) -> None:
        assert req == {"policy": "sitstand", "action": "start", "t": req["t"]}
        harness.feed_state(seated=True, active="sitstand", locked=True)

    unsub = harness.on_request(on_sit)
    try:
        out = harness.module.sit()
    finally:
        unsub()
    assert out.startswith("The duck is now sitting.")
    assert "seated" in out
    assert harness.module.sit().startswith("Already sitting.")

    def on_stand(req: dict[str, Any]) -> None:
        assert req == {"policy": "sitstand", "action": "stop", "t": req["t"]}
        harness.feed_state(seated=True, active="standing_up", locked=True)
        harness.feed_state_later(0.05, seated=False, active="walk")

    unsub = harness.on_request(on_stand)
    try:
        out = harness.module.stand_up()
    finally:
        unsub()
    assert out.startswith("The duck is now standing.")
    assert harness.module.stand_up().startswith("Already standing.")
    assert [json.loads(r)["action"] for r in harness.captured.policy_requests] == ["start", "stop"]


def test_perform_aliases_force_posture_action(harness: Harness) -> None:
    harness.feed_state(seated=True, active="sitstand", locked=True)

    def on_request(req: dict[str, Any]) -> None:
        assert req["policy"] == "sitstand" and req["action"] == "stop"
        harness.feed_state(seated=False, active="walk")

    unsub = harness.on_request(on_request)
    try:
        assert harness.module.perform("stand up").startswith("The duck is now standing.")
    finally:
        unsub()


def test_perform_base_switch(harness: Harness) -> None:
    harness.feed_state()

    def on_request(req: dict[str, Any]) -> None:
        assert req == {"policy": "stand", "action": "start", "t": req["t"]}
        harness.feed_state(active="stand", base="stand")

    unsub = harness.on_request(on_request)
    try:
        out = harness.module.perform("stand")
    finally:
        unsub()
    assert out.startswith("Base policy is now stand.")
    assert harness.module.perform("stand").startswith("Already using the stand policy.")

    def on_stop(req: dict[str, Any]) -> None:
        assert req == {"policy": "stand", "action": "stop", "t": req["t"]}
        harness.feed_state()

    unsub = harness.on_request(on_stop)
    try:
        assert harness.module.perform("stand", "stop").startswith("Base policy is now walk.")
    finally:
        unsub()


def test_perform_base_switch_error(harness: Harness) -> None:
    harness.feed_state()

    def on_request(req: dict[str, Any]) -> None:
        harness.feed_state(last_error="cannot switch while locked", locked=True)

    unsub = harness.on_request(on_request)
    try:
        out = harness.module.perform("stand")
    finally:
        unsub()
    assert out == "Could not switch to stand: cannot switch while locked."


def test_perform_bad_inputs(harness: Harness) -> None:
    harness.feed_state()
    out = harness.module.perform("moonwalk")
    assert out.startswith("Unknown policy 'moonwalk'. Known policies: walk, stand, roller")
    assert harness.module.perform("kick_left", "pause").startswith("Unknown action 'pause'")
    assert harness.captured.policy_requests == []


def test_perform_waits_for_first_state(harness: Harness) -> None:
    """policy_state is wired but nothing arrived yet: wait briefly, then give up."""
    t0 = time.monotonic()
    out = harness.module.perform("kick_left")
    assert "has not reported any policy state" in out
    assert time.monotonic() - t0 < 1.0
    assert harness.captured.last_request["policy"] == "kick_left"


def test_malformed_policy_state_is_ignored(harness: Harness) -> None:
    harness.module.policy_state.transport.broadcast(None, "not json")
    harness.module.policy_state.transport.broadcast(None, "[1,2]")
    assert harness.module._current_policy_state() is None
    harness.feed_state(active="stand")
    state = harness.module._current_policy_state()
    assert state is not None and state["active"] == "stand"
