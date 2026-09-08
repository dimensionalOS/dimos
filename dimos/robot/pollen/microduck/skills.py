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

"""Agent skills for the simulated Microduck.

Two families of skills live here:

* **Navigation** over the abstract ``NavigationInterfaceSpec``: rooms,
  landmark objects and remembered spots come from :class:`PlacesMemory`
  (seeded from the blueprint's ground-truth tables of the simulation scene,
  not from a perception pipeline - this is honest about being in sim). Goals
  are published on ``goal_request`` so the planner, the control module and
  the cockpit map all see them, and arrival is awaited through the planner's
  RPCs.
* **Policies** ("tricks"): ``perform`` publishes a ``policy_request`` for the
  simulator's policy scheduler and watches ``policy_state`` until the
  requested one-shot finished, the posture toggled or the base policy
  switched, then reports the outcome (or the scheduler's ``last_error``).

Every skill returns a short human-readable string; the docstrings are what
the LLM sees when it decides which skill to call.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
import json
import math
import threading
import time
from typing import Any

import numpy as np
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.base import NavigationState
from dimos.navigation.navigation_spec import NavigationInterfaceSpec
from dimos.robot.pollen.microduck.places import (
    ARENA_HALF_EXTENT,
    PlaceRecord,
    PlacesMemory,
    RoomSpec,
    normalize_place_query,
    scene_id,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Stop this far from an object's center so the duck ends up next to it
# instead of trying to stand inside it.
_APPROACH_DISTANCE = 0.35

# Smallest arena move_to accepts goals in; a scene whose rooms / objects
# reach further widens it (see ``_arena_half_extent``).
_ARENA_HALF_EXTENT = ARENA_HALF_EXTENT

# Navigation polling (module-level so tests can shrink them).
_NAV_POLL_S = 0.2
_NAV_START_TIMEOUT_S = 15.0
_NAV_SETTLE_S = 3.0

# Policy polling. The simulator publishes policy_state at 5 Hz and on every
# change, and the scheduler applies an accepted request on its next tick, so
# a request whose transition has not begun within the grace window was
# dropped or rejected.
_POLICY_POLL_S = 0.05
_POLICY_START_GRACE_S = 2.5
_POLICY_FIRST_STATE_WAIT_S = 1.0

# Recent policy_state messages kept so a short trick whose start and end
# arrive back-to-back between two polls is still seen to have run.
_POLICY_HISTORY = 256

_POLICY_ACTIONS = ("start", "stop", "toggle")

# Static fallback (display order) used until the simulator has reported its
# own policy table; kinds match dimos.robot.pollen.microduck.policies.
_POLICY_KINDS: dict[str, str] = {
    "walk": "base",
    "stand": "base",
    "roller": "base",
    "roller_crouch": "base",
    "sitstand": "posture",
    "kick_left": "oneshot",
    "kick_right": "oneshot",
    "roulade": "oneshot",
    "ground_pick": "oneshot",
}

_POLICY_DESCRIPTIONS: dict[str, str] = {
    "walk": "default walking gait, follows velocity commands",
    "stand": "stand still and balance (ignores velocity commands)",
    "roller": "drive on the roller wheels (rollers variant only)",
    "roller_crouch": "crouched roller driving (rollers variant only)",
    "sitstand": "sit down / stand back up (posture toggle)",
    "kick_left": "kick the ball with the left foot",
    "kick_right": "kick the ball with the right foot",
    "roulade": "forward roll (somersault) and get back up",
    "ground_pick": "bend down and pick at the ground",
}

# Free-form names people use -> (policy, forced action or None).
_POLICY_ALIASES: dict[str, tuple[str, str | None]] = {
    "walking": ("walk", None),
    "standing": ("stand", None),
    "stand still": ("stand", None),
    "balance": ("stand", None),
    "kick": ("kick_left", None),
    "kick the ball": ("kick_left", None),
    "left kick": ("kick_left", None),
    "kick left": ("kick_left", None),
    "right kick": ("kick_right", None),
    "kick right": ("kick_right", None),
    "roll": ("roulade", None),
    "forward roll": ("roulade", None),
    "somersault": ("roulade", None),
    "flip": ("roulade", None),
    "tumble": ("roulade", None),
    "pick": ("ground_pick", None),
    "pick up": ("ground_pick", None),
    "ground pick": ("ground_pick", None),
    "peck": ("ground_pick", None),
    "crouch": ("roller_crouch", None),
    "roller crouch": ("roller_crouch", None),
    "skate": ("roller", None),
    "wheels": ("roller", None),
    "rollers": ("roller", None),
    "sit": ("sitstand", "start"),
    "sit down": ("sitstand", "start"),
    "sitting": ("sitstand", "start"),
    "stand up": ("sitstand", "stop"),
    "standing up": ("sitstand", "stop"),
    "get up": ("sitstand", "stop"),
    "getting up": ("sitstand", "stop"),
    "rise": ("sitstand", "stop"),
    "sit stand": ("sitstand", None),
}


def _yaw_quaternion(yaw: float) -> Quaternion:
    return Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _normalize_policy_name(name: str) -> str:
    return " ".join(name.strip().lower().replace("_", " ").replace("-", " ").split())


def _now_json(payload: dict[str, Any]) -> str:
    payload.setdefault("t", time.time())
    return json.dumps(payload, separators=(",", ":"))


class MicroduckSkillContainerConfig(ModuleConfig):
    # name -> (x, y) world coordinates of landmark objects in the scene.
    objects: dict[str, tuple[float, float]] = {}
    # name -> RoomSpec of the rooms the scene is divided into.
    rooms: dict[str, RoomSpec] = {}
    # Sqlite file the places memory persists to (``~`` is expanded). Several
    # blueprints share it; records are scoped by ``scene``.
    places_db: str = "~/.cache/dimos/microduck/places.db"
    # Scene id the memory is scoped to. Empty = derive it from the ``rooms``
    # and ``objects`` tables (``places.scene_id``), so blueprints describing
    # different worlds never see each other's rooms, objects or tagged spots.
    scene: str = ""
    # How long a single navigation skill may block waiting for arrival.
    nav_timeout_s: float = 100.0
    # How long ``perform`` waits for a policy to finish / a posture to toggle.
    policy_timeout_s: float = 15.0
    # Re-publish the ``places`` snapshot this often so a consumer that
    # subscribed after start still receives it (0 disables the heartbeat).
    places_republish_s: float = 5.0


class MicroduckSkillContainer(Module):
    """Skills the agent can call to move the Microduck around its rooms and
    make it perform its policies."""

    config: MicroduckSkillContainerConfig

    _navigation: NavigationInterfaceSpec

    odom: In[PoseStamped]
    policy_state: In[str]

    goal_request: Out[PoseStamped]
    policy_request: Out[str]
    places: Out[str]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._latest_odom: PoseStamped | None = None
        self._policy_state: dict[str, Any] | None = None
        # Every policy_state gets a sequence number; a skill remembers the
        # number current when it sent its request and only trusts newer ones.
        self._policy_state_seq: int = 0
        self._policy_history: deque[tuple[int, dict[str, Any]]] = deque(maxlen=_POLICY_HISTORY)
        self._state_cond = threading.Condition()
        self._memory: PlacesMemory | None = None
        self._memory_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._republish_thread: threading.Thread | None = None

    # -- lifecycle -----------------------------------------------------------

    @rpc
    def start(self) -> None:
        super().start()
        self._stop_event.clear()
        self._places_memory()  # open + seed
        if self.odom.transport is not None:
            self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))
        else:
            logger.warning(
                "MicroduckSkillContainer: odom is not connected; navigation skills need it"
            )
        if self.policy_state.transport is not None:
            self.register_disposable(Disposable(self.policy_state.subscribe(self._on_policy_state)))
        self._publish_places()
        if self.config.places_republish_s > 0:
            self._republish_thread = threading.Thread(
                target=self._republish_loop, name="microduck-places", daemon=True
            )
            self._republish_thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        with self._state_cond:
            self._state_cond.notify_all()
        thread = self._republish_thread
        if thread is not None:
            thread.join(timeout=2.0)
            self._republish_thread = None
        super().stop()
        with self._memory_lock:
            if self._memory is not None:
                self._memory.close()
                self._memory = None

    def _scene(self) -> str:
        return self.config.scene or scene_id(self.config.rooms, self.config.objects)

    def _places_memory(self) -> PlacesMemory:
        """The (lazily opened, seeded) places memory.

        Raises ``RuntimeError`` once :meth:`stop` has run, so a skill thread
        still unwinding after shutdown cannot reopen the database that
        ``stop`` just closed; :meth:`start` re-arms it.
        """
        with self._memory_lock:
            if self._stop_event.is_set():
                raise RuntimeError("MicroduckSkillContainer is stopped; places memory is closed")
            if self._memory is None:
                memory = PlacesMemory(self.config.places_db, scene=self._scene())
                memory.seed(self.config.rooms, self.config.objects)
                self._memory = memory
            return self._memory

    def _publish_places(self) -> None:
        if self._stop_event.is_set():
            return
        try:
            self.places.publish(self._places_memory().to_json())
        except RuntimeError:
            pass  # stopped between the check and the publish
        except Exception:
            logger.exception("Failed to publish places")

    def _republish_loop(self) -> None:
        while not self._stop_event.wait(self.config.places_republish_s):
            self._publish_places()

    # -- stream callbacks ----------------------------------------------------

    def _on_odom(self, pose: PoseStamped) -> None:
        self._latest_odom = pose

    def _on_policy_state(self, msg: str) -> None:
        try:
            state = json.loads(msg)
        except (TypeError, ValueError):
            logger.warning("Ignoring malformed policy_state", msg=str(msg)[:120])
            return
        if not isinstance(state, dict):
            return
        with self._state_cond:
            self._policy_state_seq += 1
            self._policy_state = state
            self._policy_history.append((self._policy_state_seq, state))
            self._state_cond.notify_all()

    # -- robot pose helpers --------------------------------------------------

    def _robot_xy(self) -> tuple[float, float] | None:
        odom = self._latest_odom
        if odom is None:
            return None
        return (float(odom.position.x), float(odom.position.y))

    def _robot_yaw(self) -> float | None:
        odom = self._latest_odom
        if odom is None:
            return None
        try:
            return float(odom.yaw)
        except Exception:
            return None

    def _where_summary(self) -> str:
        robot = self._robot_xy()
        if robot is None:
            return "Robot position unknown."
        try:
            memory = self._places_memory()
        except RuntimeError:
            return f"Robot was at ({robot[0]:.2f}, {robot[1]:.2f}) (module stopping)."
        where = self._room_phrase(memory, *robot)
        return f"Robot is now at ({robot[0]:.2f}, {robot[1]:.2f}){where}."

    @staticmethod
    def _room_phrase(memory: PlacesMemory, x: float, y: float) -> str:
        """`` in the kitchen`` / `` in the central hub``; empty when the scene has no rooms."""
        if not memory.rooms():
            return ""
        room = memory.room_at(x, y)
        return f" in the {room.name}" if room is not None else " in the central hub"

    def _arena_half_extent(self) -> float:
        """Goals beyond this (per axis) are refused before reaching the planner.

        The four-room arena is +/-2 m; a scene whose rooms or objects reach
        further widens the limit so move_to stays usable there.
        """
        reach = [_ARENA_HALF_EXTENT]
        reach.extend(abs(b) for room in self.config.rooms.values() for b in room.bounds)
        reach.extend(abs(c) for xy in self.config.objects.values() for c in xy)
        return max(reach)

    def _distance_suffix(self, record: PlaceRecord) -> str:
        robot = self._robot_xy()
        if robot is None:
            return ""
        return f", {record.distance_to(*robot):.2f} m away"

    def _object_lines(self, memory: PlacesMemory, objects: list[PlaceRecord]) -> list[str]:
        lines: list[str] = []
        for r in objects:
            room = memory.room_at(r.x, r.y)
            where = f" in the {room.name}" if room is not None else ""
            lines.append(f"- {r.name} at ({r.x:.2f}, {r.y:.2f}){where}{self._distance_suffix(r)}")
        return lines

    @staticmethod
    def _ambiguous(query: str, candidates: list[PlaceRecord], what: str) -> str:
        names = ", ".join(r.name for r in candidates)
        return f"Ambiguous {what} '{query}': did you mean {names}? Use the exact name."

    # -- places skills -------------------------------------------------------

    @skill
    def list_places(self) -> str:
        """List every place the duck knows: the rooms (with their aliases such
        as "space A"), the landmark objects with their coordinates, and any
        spots remembered with remember_place. Includes the distance from the
        robot when its position is known. Call this when unsure what a room
        or object is called before using go_to_room / go_to_object.
        """
        memory = self._places_memory()
        records = memory.all()
        if not records:
            return "No places are registered in this scene."

        rooms = [r for r in records if r.kind == "room"]
        objects = [r for r in records if r.kind == "object"]
        tagged = [r for r in records if r.kind not in ("room", "object")]
        lines: list[str] = []
        if rooms:
            lines.append("Rooms:")
            for r in rooms:
                aka = f" (aka {', '.join(r.aliases)})" if r.aliases else ""
                xmin, xmax, ymin, ymax = r.bounds or (0.0, 0.0, 0.0, 0.0)
                lines.append(
                    f"- {r.name}{aka}: x {xmin:g}..{xmax:g}, y {ymin:g}..{ymax:g}, "
                    f"entry point ({r.x:.2f}, {r.y:.2f}){self._distance_suffix(r)}"
                )
        if objects:
            lines.append("Objects:")
            lines.extend(self._object_lines(memory, objects))
        lines.append("Remembered places:")
        if tagged:
            for r in tagged:
                lines.append(f"- {r.name} at ({r.x:.2f}, {r.y:.2f}){self._distance_suffix(r)}")
        else:
            lines.append("- (none yet; use remember_place to tag the current spot)")
        lines.append(self._where_summary())
        return "\n".join(lines)

    @skill
    def list_objects(self) -> str:
        """List the known landmark objects with their positions in meters (and
        the distance from the duck when known). list_places additionally
        shows the rooms and remembered spots."""
        memory = self._places_memory()
        objects = [r for r in memory.all() if r.kind == "object"]
        if not objects:
            return "No objects are registered in this scene."
        return "Objects in the room:\n" + "\n".join(self._object_lines(memory, objects))

    @skill
    def go_to_room(self, name: str) -> str:
        """Walk into a room and stop at its entry point, facing into the room.
        Accepts the room's name or any of its aliases as listed by
        list_places (e.g. "kitchen" or "space A"). Blocks until the duck
        arrives (or navigation gives up) and reports where the duck ended up.

        Args:
            name: The room's name or alias, e.g. "space A" or "kitchen".
        """
        memory = self._places_memory()
        candidates = memory.matches(name, kind="room")
        if len(candidates) > 1:
            return self._ambiguous(name, candidates, "room")
        if not candidates:
            known = ", ".join(
                f"{r.name} ({', '.join(r.aliases)})" if r.aliases else r.name
                for r in memory.rooms()
            )
            return f"Unknown room '{name}'. Known rooms: {known or '(none)'}"
        record = candidates[0]
        robot = self._robot_xy()
        if robot is not None and record.distance_to(*robot) <= 0.25:
            return f"Already at the {record.name} entry point. {self._where_summary()}"
        outcome = self._navigate_to(record.x, record.y, record.yaw)
        return f"{outcome} while walking to the {record.name}. {self._where_summary()}"

    @skill
    def go_to_object(self, name: str) -> str:
        """Walk to a named landmark object and stop right next to it (about
        0.35 m away), facing it. Use list_places to see what exists. Blocks
        until arrival and reports where the duck ended up.

        Args:
            name: The object's exact name, e.g. "red_box" or "red box" (a bare
                "box" is ambiguous when several boxes exist).
        """
        memory = self._places_memory()
        candidates = memory.matches(name, kind="object")
        if len(candidates) > 1:
            return self._ambiguous(name, candidates, "object")
        if not candidates:
            known = ", ".join(r.name for r in memory.all() if r.kind == "object") or "(none)"
            return f"Unknown object '{name}'. Known objects: {known}"
        return self._approach(candidates[0])

    @skill
    def go_to_place(self, name: str) -> str:
        """Walk to any known place by name: a room (or alias like "space B"),
        a landmark object, or a spot saved with remember_place. Rooms are
        entered at their entry point, objects are approached to 0.35 m, and
        remembered spots are reached exactly with the saved heading. Blocks
        until arrival and reports where the duck ended up.

        Args:
            name: The place's name or alias (exact enough to be unambiguous).
        """
        memory = self._places_memory()
        candidates = memory.matches(name)
        if len(candidates) > 1:
            return self._ambiguous(name, candidates, "place")
        if not candidates:
            return f"Unknown place '{name}'. Use list_places to see what the duck knows."
        record = candidates[0]
        if record.kind == "room":
            return self.go_to_room(record.name)
        if record.kind == "object":
            return self._approach(record)
        outcome = self._navigate_to(record.x, record.y, record.yaw)
        return f"{outcome} while walking to {record.name}. {self._where_summary()}"

    @skill
    def move_to(self, x: float, y: float) -> str:
        """Walk to world coordinates (x, y) in meters and wait for arrival.
        The world frame is the map shown in the cockpit; list_places gives
        the coordinates of every room, object and remembered spot, which
        bound the reachable area. Prefer go_to_room / go_to_object /
        go_to_place when the user names a place.

        Args:
            x: Target x in meters (world frame).
            y: Target y in meters (world frame).
        """
        try:
            x, y = float(x), float(y)
        except (TypeError, ValueError):
            return f"Invalid coordinates ({x!r}, {y!r}); give numbers in meters."
        if not (math.isfinite(x) and math.isfinite(y)):
            return "Invalid coordinates; give finite numbers in meters."
        half_extent = self._arena_half_extent()
        if abs(x) > half_extent or abs(y) > half_extent:
            return (
                f"({x:.2f}, {y:.2f}) is outside the arena walls "
                f"(x and y must be within +/-{half_extent:g} m)."
            )
        robot = self._robot_xy()
        heading = None
        if robot is not None:
            heading = math.atan2(y - robot[1], x - robot[0])
        outcome = self._navigate_to(x, y, heading)
        return f"{outcome}. {self._where_summary()}"

    @skill
    def stop_moving(self) -> str:
        """Cancel the current navigation goal; the duck stops and stands in
        place. Does not stop a trick in progress (use perform with action
        "stop" for that)."""
        cancelled = self._navigation.cancel_goal()
        return "Stopped." if cancelled else "There was no active navigation goal."

    @skill
    def where_am_i(self) -> str:
        """Report the duck's position, heading, which room it is in (when the
        scene has rooms) and the known places within 1 m."""
        robot = self._robot_xy()
        if robot is None:
            return "Robot position unknown (no odometry yet)."
        memory = self._places_memory()
        yaw = self._robot_yaw()
        heading = "" if yaw is None else f", heading {math.degrees(yaw):.0f} deg"
        where = ""
        if memory.rooms():
            room = memory.room_at(*robot)
            where = (
                f", in the {room.name}" if room is not None else ", in the central hub (no room)"
            )
            if room is not None and room.aliases:
                where += f" (aka {room.aliases[0]})"
        nearby = [r for r in memory.near(robot[0], robot[1], 1.0) if r.kind != "room"]
        near_txt = ""
        if nearby:
            near_txt = (
                " Nearby: "
                + ", ".join(f"{r.name} ({r.distance_to(*robot):.2f} m)" for r in nearby[:4])
                + "."
            )
        return f"Robot is at ({robot[0]:.2f}, {robot[1]:.2f}){heading}{where}.{near_txt}"

    @skill
    def remember_place(self, name: str) -> str:
        """Save the duck's current position and heading under a name so it
        can be revisited later with go_to_place. The place is persisted and
        appears in list_places.

        Args:
            name: A short label for this spot, e.g. "charger" or "front door".
        """
        label = " ".join(str(name).split())
        if not label:
            return "Give the place a name, e.g. remember_place('charger')."
        robot = self._robot_xy()
        if robot is None:
            return "Robot position unknown (no odometry yet); cannot remember this spot."
        memory = self._places_memory()
        clash = self._reserved_name_clash(memory, label)
        if clash is not None:
            return f"'{label}' is already the name of a {clash.kind}; pick another name."
        yaw = self._robot_yaw() or 0.0
        memory.add(label, robot[0], robot[1], yaw)
        self._publish_places()
        where = self._room_phrase(memory, *robot)
        return f"Remembered '{label}' at ({robot[0]:.2f}, {robot[1]:.2f}){where}."

    @skill
    def wait(self, seconds: float) -> str:
        """Wait in place for the given number of seconds (0 to 30), e.g. to
        let a trick finish or to give the user time to look.

        Args:
            seconds: How long to wait, in seconds.
        """
        try:
            seconds = float(np.clip(float(seconds), 0.0, 30.0))
        except (TypeError, ValueError):
            return f"Invalid duration {seconds!r}; give a number of seconds."
        deadline = time.monotonic() + seconds
        while not self._stop_event.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            time.sleep(min(remaining, 0.25))
        return f"Waited {seconds:.0f} s."

    # -- navigation internals ------------------------------------------------

    @staticmethod
    def _reserved_name_clash(memory: PlacesMemory, label: str) -> PlaceRecord | None:
        """The room / object whose name or alias ``label`` would shadow, if any.

        Compared on the normalised forms ``find`` matches on, so a tagged spot
        called "red box" cannot hide the object ``red_box`` (or vice versa).
        """
        forms = set(normalize_place_query(label))
        if not forms:
            return None
        for record in memory.all():
            if record.kind not in ("room", "object"):
                continue
            keys = set(normalize_place_query(record.name))
            for alias in record.aliases:
                keys.update(normalize_place_query(alias))
            if forms & keys:
                return record
        return None

    def _approach(self, record: PlaceRecord) -> str:
        robot = self._robot_xy()
        if robot is None:
            return "Robot position unknown (no odometry yet); try again shortly."
        dx, dy = record.x - robot[0], record.y - robot[1]
        distance = math.hypot(dx, dy)
        if distance <= _APPROACH_DISTANCE + 0.05:
            return f"Already next to {record.name} ({distance:.2f} m away). {self._where_summary()}"
        heading = math.atan2(dy, dx)
        gx = record.x - math.cos(heading) * _APPROACH_DISTANCE
        gy = record.y - math.sin(heading) * _APPROACH_DISTANCE
        outcome = self._navigate_to(gx, gy, heading)
        return f"{outcome} while walking to {record.name}. {self._where_summary()}"

    def _navigate_to(self, x: float, y: float, heading: float | None) -> str:
        goal = PoseStamped(
            ts=time.time(),
            frame_id="world",
            position=Vector3(float(x), float(y), 0.0),
            orientation=_yaw_quaternion(heading) if heading is not None else Quaternion(),
        )
        # Publish so the planner, the control module and the cockpit map all
        # see the goal. Without a wired transport (direct use, unit tests)
        # hand it to the planner RPC instead so the goal is not lost.
        self.goal_request.publish(goal)
        if self.goal_request.transport is None and not self._navigation.set_goal(goal):
            return "Navigation rejected the goal (is the map ready?)"
        return self._wait_for_goal(timeout=self.config.nav_timeout_s)

    def _wait_for_goal(self, timeout: float | None = None, settle: float | None = None) -> str:
        """Block until arrival, cancellation, or timeout.

        Arrival (`is_goal_reached`) only counts after the planner has been
        seen FOLLOWING_PATH for *this* goal - the flag may still be latched
        from a previous goal (e.g. one an exploration cycle just finished).
        The planner also leaves FOLLOWING_PATH briefly on every replan, so a
        pause only counts as the end after `settle` seconds.
        """
        timeout = self.config.nav_timeout_s if timeout is None else timeout
        settle = _NAV_SETTLE_S if settle is None else settle
        deadline = time.monotonic() + timeout
        started = False
        start_deadline = time.monotonic() + min(_NAV_START_TIMEOUT_S, timeout)
        while not started and time.monotonic() < start_deadline:
            if self._stop_event.is_set():
                return "Navigation aborted (module stopping)"
            if self._navigation.get_state() == NavigationState.FOLLOWING_PATH:
                started = True
            else:
                time.sleep(_NAV_POLL_S)
        if not started:
            return "Navigation never started following a path (no route found?)"

        idle_since: float | None = None
        while time.monotonic() < deadline:
            if self._stop_event.is_set():
                return "Navigation aborted (module stopping)"
            if self._navigation.is_goal_reached():
                return "Arrived"
            if self._navigation.get_state() == NavigationState.FOLLOWING_PATH:
                idle_since = None
            elif idle_since is None:
                idle_since = time.monotonic()
            elif time.monotonic() - idle_since > settle:
                return "Navigation stopped early (cancelled or no path)"
            time.sleep(_NAV_POLL_S)
        return "Navigation timed out"

    # -- policy skills -------------------------------------------------------

    @skill
    def list_policies(self) -> str:
        """List the duck's policies (tricks and gaits) with what each does and
        whether it is available on this robot variant, plus what the duck is
        doing right now (active policy, sitting, fallen). Use the listed names
        with perform."""
        state = self._current_policy_state()
        entries: list[dict[str, Any]] = []
        if state is not None and isinstance(state.get("policies"), list):
            entries = [e for e in state["policies"] if isinstance(e, dict) and "name" in e]
        if not entries:
            entries = [
                {"name": n, "kind": k, "available": True, "reason": None}
                for n, k in _POLICY_KINDS.items()
            ]
        lines = ["Policies:"]
        for entry in entries:
            name = str(entry["name"])
            kind = str(entry.get("kind") or _POLICY_KINDS.get(name, "?"))
            desc = _POLICY_DESCRIPTIONS.get(name, "")
            line = f"- {name} [{kind}]"
            if desc:
                line += f": {desc}"
            if not entry.get("available", True):
                line += f" (NOT available: {entry.get('reason') or 'unsupported on this variant'})"
            lines.append(line)
        lines.append(
            "Kinds: oneshot = a trick that runs once and ends by itself; posture = "
            "sitstand toggles sitting; base = the gait used when idle."
        )
        if state is None:
            lines.append("The simulator has not reported its policy state yet.")
        else:
            lines.append(self._policy_summary(state))
        return "\n".join(lines)

    @skill
    def perform(self, policy: str, action: str = "start") -> str:
        """Make the duck perform a policy and wait for the outcome. Tricks
        (kick_left, kick_right, roulade, ground_pick) run once and this
        returns when they finish; "sitstand" sits the duck down (or use sit /
        stand_up); "walk", "stand", "roller", "roller_crouch" switch the base
        gait. Use list_policies for the exact names and availability. The
        duck must be standing still and idle; a running trick can be aborted
        with action "stop". If the request is refused the reason is returned
        (e.g. locked while seated or fallen) - fix that first instead of
        retrying blindly.

        Args:
            policy: Policy name, e.g. "kick_left", "roulade", "sitstand".
            action: "start" (default), "stop" (abort a running trick / stand
                up / back to walking) or "toggle" (start, or abort if that
                trick is already running; sit or stand for sitstand).
        """
        return self._perform(policy, action)

    @skill
    def sit(self) -> str:
        """Make the duck sit down (it stays seated and cannot walk until
        stand_up is called). Returns once it is seated."""
        return self._perform("sitstand", "start")

    @skill
    def stand_up(self) -> str:
        """Make a seated duck stand back up. Returns once it is standing and
        ready to walk again."""
        return self._perform("sitstand", "stop")

    # -- policy internals ----------------------------------------------------

    def _current_policy_state(self) -> dict[str, Any] | None:
        with self._state_cond:
            return self._policy_state

    def _known_policies(self, state: dict[str, Any] | None) -> dict[str, dict[str, Any]]:
        """name -> entry, from the simulator's table when it has one."""
        table: dict[str, dict[str, Any]] = {
            n: {"name": n, "kind": k, "available": True, "reason": None}
            for n, k in _POLICY_KINDS.items()
        }
        if state is not None and isinstance(state.get("policies"), list):
            for entry in state["policies"]:
                if isinstance(entry, dict) and "name" in entry:
                    merged = dict(table.get(str(entry["name"]), {}))
                    merged.update(entry)
                    table[str(entry["name"])] = merged
        return table

    def _resolve_policy(
        self, policy: str, known: dict[str, dict[str, Any]]
    ) -> tuple[str | None, str | None]:
        """Map a free-form name to ``(policy_name, forced_action)``."""
        key = _normalize_policy_name(str(policy))
        if not key:
            return None, None
        by_norm = {_normalize_policy_name(n): n for n in known}
        if key in by_norm:
            return by_norm[key], None
        if key in _POLICY_ALIASES:
            name, forced = _POLICY_ALIASES[key]
            return name, forced
        for alias, (name, forced) in sorted(_POLICY_ALIASES.items(), key=lambda kv: -len(kv[0])):
            if len(alias) >= 4 and (f" {alias} " in f" {key} "):
                return name, forced
        return None, None

    def _policy_summary(self, state: dict[str, Any]) -> str:
        active = state.get("active")
        parts = [f"Now: active policy {active!r} (base {state.get('base')!r})"]
        if state.get("seated"):
            parts.append("seated")
        if state.get("fallen"):
            parts.append("FALLEN (auto-recovering)")
        oneshot = state.get("oneshot")
        if isinstance(oneshot, dict):
            progress = oneshot.get("progress")
            pct = f" {float(progress) * 100:.0f}%" if isinstance(progress, (int, float)) else ""
            parts.append(f"running {oneshot.get('name')}{pct}")
        elif state.get("locked"):
            parts.append("busy")
        if state.get("last_error"):
            parts.append(f"last error: {state['last_error']}")
        return ", ".join(parts) + "."

    def _await_state(
        self, predicate: Callable[[dict[str, Any]], bool], timeout: float
    ) -> dict[str, Any] | None:
        """Block until a policy_state satisfying ``predicate`` arrives."""
        deadline = time.monotonic() + timeout
        with self._state_cond:
            while True:
                state = self._policy_state
                if state is not None and predicate(state):
                    return state
                remaining = deadline - time.monotonic()
                if remaining <= 0 or self._stop_event.is_set():
                    return None
                self._state_cond.wait(min(remaining, _POLICY_POLL_S))

    def _perform(self, policy: str, action: str = "start") -> str:
        action_key = str(action).strip().lower()
        if action_key not in _POLICY_ACTIONS:
            return f"Unknown action '{action}'. Use one of: {', '.join(_POLICY_ACTIONS)}."
        state = self._current_policy_state()
        known = self._known_policies(state)
        name, forced_action = self._resolve_policy(policy, known)
        if name is None:
            return f"Unknown policy '{policy}'. Known policies: {', '.join(known)}."
        if forced_action is not None and action_key == "start":
            action_key = forced_action
        entry = known.get(name, {})
        kind = str(entry.get("kind") or _POLICY_KINDS.get(name, "oneshot"))
        if not entry.get("available", True):
            reason = entry.get("reason") or "unsupported on this robot variant"
            return f"'{name}' is not available on this robot ({reason})."

        def running(s: dict[str, Any]) -> bool:
            one = s.get("oneshot")
            return isinstance(one, dict) and one.get("name") == name

        # Fast paths / pre-checks from the latest state.
        if state is not None:
            seated = bool(state.get("seated"))
            oneshot = state.get("oneshot") if isinstance(state.get("oneshot"), dict) else None
            if kind == "posture":
                if action_key == "start" and seated:
                    return f"Already sitting. {self._policy_summary(state)}"
                if action_key == "stop" and not seated and state.get("active") != "standing_up":
                    return f"Already standing. {self._policy_summary(state)}"
            elif kind == "base" and action_key in ("start", "toggle"):
                if state.get("base") == name and state.get("active") == name:
                    return f"Already using the {name} policy. {self._policy_summary(state)}"
            elif kind == "oneshot":
                if action_key == "toggle":
                    # A toggle means "abort" while this trick runs and "start"
                    # otherwise; decide now so the outcome can be awaited.
                    action_key = "stop" if running(state) else "start"
                if action_key == "stop" and not running(state):
                    return f"{name} is not running. {self._policy_summary(state)}"
                if action_key == "start" and oneshot is not None:
                    return (
                        f"The duck is busy with {oneshot.get('name')}; wait for it to finish "
                        f"or perform('{oneshot.get('name')}', 'stop') first."
                    )
                if action_key == "start" and seated:
                    return "The duck is sitting; call stand_up first."
            if state.get("fallen") and action_key != "stop":
                return "The duck has fallen and is getting back up; try again in a moment."

        prev_error = state.get("last_error") if state is not None else None
        prev_seated = bool(state.get("seated")) if state is not None else False
        with self._state_cond:
            prev_seq = self._policy_state_seq
        self.policy_request.publish(_now_json({"policy": name, "action": action_key}))
        logger.info("Policy request", policy=name, action=action_key)

        if self.policy_state.transport is None and state is None:
            return f"Sent '{name}' {action_key} request (no policy feedback available)."
        if state is None:
            state = self._await_state(lambda _s: True, _POLICY_FIRST_STATE_WAIT_S)
            if state is None:
                return (
                    f"Sent '{name}' {action_key} request but the simulator has not reported "
                    "any policy state; cannot confirm the outcome."
                )

        timeout = float(self.config.policy_timeout_s)

        if kind == "oneshot":
            if action_key == "stop":
                done = self._await_state(lambda s: not running(s), timeout)
                if done is None:
                    return f"Timed out waiting for {name} to stop."
                return f"Stopped {name}. {self._policy_summary(done)}"

            # Only the trick actually running counts as acknowledged; the
            # history scan catches one that already finished before we looked.
            ack, err = self._await_ack(running, prev_seq, prev_error)
            if ack is None:
                latest = self._current_policy_state() or state
                return f"Could not start {name}: {err}. {self._policy_summary(latest)}"
            finished = self._await_state(lambda s: not running(s), timeout)
            if finished is None:
                return f"Timed out waiting for {name} to finish."
            outcome = f"Finished {name}."
            if finished.get("fallen"):
                outcome += " The duck fell over and is getting back up."
            return f"{outcome} {self._policy_summary(finished)}"

        if kind == "posture":
            if action_key == "toggle":
                target_seated = not prev_seated
            else:
                target_seated = action_key == "start"
            what = "sit down" if target_seated else "stand up"

            def settled(s: dict[str, Any]) -> bool:
                return bool(s.get("seated")) == target_seated and s.get("active") != "standing_up"

            def begun(s: dict[str, Any]) -> bool:
                if settled(s):
                    return True
                if target_seated:
                    return bool(s.get("seated")) or s.get("active") == "sitstand"
                return s.get("active") == "standing_up"

            final, err = self._await_transition(begun, settled, prev_seq, prev_error, timeout)
            if err is not None:
                return f"Could not {what}: {err}."
            if final is None:
                return f"Timed out waiting for the duck to {what}."
            return (
                f"The duck is now {'sitting' if target_seated else 'standing'}. "
                f"{self._policy_summary(final)}"
            )

        # base policy switch
        target = "walk" if action_key == "stop" else name

        def switched(s: dict[str, Any]) -> bool:
            return s.get("base") == target and s.get("active") == target

        def base_begun(s: dict[str, Any]) -> bool:
            return switched(s) or s.get("base") == target or s.get("active") == "braking"

        final, err = self._await_transition(base_begun, switched, prev_seq, prev_error, timeout)
        if err is not None:
            return f"Could not switch to {target}: {err}."
        if final is None:
            return f"Timed out waiting for the base policy to switch to {target}."
        return f"Base policy is now {target}. {self._policy_summary(final)}"

    def _await_ack(
        self,
        begun: Callable[[dict[str, Any]], bool],
        prev_seq: int,
        prev_error: str | None,
    ) -> tuple[dict[str, Any] | None, str | None]:
        """Wait up to ``_POLICY_START_GRACE_S`` for the scheduler to take a request.

        Every policy_state received after the request (sequence number above
        ``prev_seq``) is examined, not just the latest one, so a transition
        that begins and ends between two polls - a short trick, a stand-up
        that settles at once - is still seen. Returns ``(state, None)`` for
        the first such state in which ``begun`` holds, ``(None, reason)`` as
        soon as one carries a different ``last_error`` than before the
        request, and otherwise, once the grace period has passed, ``(None,
        reason)`` with the latest ``last_error`` - even when that is the very
        same string as before (the scheduler re-issues identical rejections
        without changing anything but the timestamp) - or "no
        acknowledgement". The scheduler applies an accepted request on its
        next tick, so a request it never saw fails after the grace period
        instead of the full ``policy_timeout_s``.
        """
        deadline = time.monotonic() + _POLICY_START_GRACE_S
        scanned = prev_seq
        with self._state_cond:
            while True:
                for seq, s in self._policy_history:
                    if seq <= scanned:
                        continue
                    scanned = seq
                    if begun(s):
                        return s, None
                    err = s.get("last_error")
                    if err and err != prev_error:
                        return None, str(err)
                remaining = deadline - time.monotonic()
                if remaining <= 0 or self._stop_event.is_set():
                    break
                self._state_cond.wait(min(remaining, _POLICY_POLL_S))
            latest = self._policy_state
        err = (latest.get("last_error") if latest else None) or "no acknowledgement"
        return None, str(err)

    def _await_transition(
        self,
        begun: Callable[[dict[str, Any]], bool],
        settled: Callable[[dict[str, Any]], bool],
        prev_seq: int,
        prev_error: str | None,
        timeout: float,
    ) -> tuple[dict[str, Any] | None, str | None]:
        """Acknowledge a request (see ``_await_ack``) then wait for ``settled``.

        Returns ``(final_state, None)`` on success, ``(None, reason)`` when the
        scheduler rejected the request or reported an error after taking it,
        and ``(None, None)`` on timeout.
        """
        ack, err = self._await_ack(begun, prev_seq, prev_error)
        if ack is None:
            return None, err
        if settled(ack):
            return ack, None
        ack_error = ack.get("last_error")

        def later_error(s: dict[str, Any]) -> str | None:
            e = s.get("last_error")
            return str(e) if e and e != ack_error else None

        final = self._await_state(lambda s: settled(s) or later_error(s) is not None, timeout)
        if final is None:
            return None, None
        if settled(final):
            return final, None
        return None, later_error(final)
