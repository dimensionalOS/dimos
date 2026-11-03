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

"""Route and Waypoint state machine for the multi-waypoint route feature.

Pure logic, no streams and no display. Thread-safe: every public method takes
the lock. The state rules live in specs/001-multi-waypoint-route/data-model.md.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import threading

from dimos.navigation.route_follower.messages import is_valid_click

MSG_STOP_ROUTE_FIRST = "stop the route first"
MSG_ROUTE_FULL = "route is full"
MSG_NO_WAYPOINTS = "no waypoints to follow"
MSG_INVALID_POSITION = "invalid waypoint position"
MSG_EDIT_FIRST = "edit the route first"

DEFAULT_MAX_WAYPOINTS = 20


class WaypointState(str, Enum):
    PENDING = "pending"
    CURRENT = "current"
    REACHED = "reached"
    FAILED = "failed"


class RouteStatus(str, Enum):
    PLANNING = "planning"
    RUNNING = "running"
    COMPLETED = "completed"
    STOPPED = "stopped"
    INTERRUPTED = "interrupted"
    FAILED = "failed"
    HELD = "held"


@dataclass
class Waypoint:
    index: int
    x: float
    y: float
    z: float
    state: WaypointState = WaypointState.PENDING


class Route:
    """One route for the one connected robot.

    A revision bump marks every accepted operation and every refusal (which
    sets `message`); ignored operations change nothing. Accepted operations
    clear `message`, except `fail_current` which names the failed waypoint.
    """

    def __init__(self, max_waypoints: int = DEFAULT_MAX_WAYPOINTS) -> None:
        self._lock = threading.Lock()
        self._max_waypoints = max_waypoints
        self._waypoints: list[Waypoint] = []
        self._status = RouteStatus.PLANNING
        self._add_mode = False
        self._current_index: int | None = None
        self._revision = 0
        self._message: str | None = None

    @property
    def waypoints(self) -> tuple[Waypoint, ...]:
        with self._lock:
            return tuple(
                Waypoint(index=w.index, x=w.x, y=w.y, z=w.z, state=w.state) for w in self._waypoints
            )

    @property
    def status(self) -> RouteStatus:
        with self._lock:
            return self._status

    @property
    def add_mode(self) -> bool:
        with self._lock:
            return self._add_mode

    @property
    def current_index(self) -> int | None:
        with self._lock:
            return self._current_index

    @property
    def revision(self) -> int:
        with self._lock:
            return self._revision

    @property
    def message(self) -> str | None:
        with self._lock:
            return self._message

    def add_waypoint(self, x: float, y: float, z: float) -> bool:
        with self._lock:
            if self._status is RouteStatus.RUNNING:
                return self._refuse(MSG_STOP_ROUTE_FIRST)
            if not is_valid_click(x, y, z):
                return self._refuse(MSG_INVALID_POSITION)
            if len(self._waypoints) >= self._max_waypoints:
                return self._refuse(MSG_ROUTE_FULL)
            self._begin_edit_locked()
            self._waypoints.append(Waypoint(index=len(self._waypoints) + 1, x=x, y=y, z=z))
            self._accept_locked()
            return True

    def remove_waypoint(self, index: int) -> bool:
        with self._lock:
            if self._status is RouteStatus.RUNNING:
                return self._refuse(MSG_STOP_ROUTE_FIRST)
            if not 1 <= index <= len(self._waypoints):
                return self._refuse(f"no waypoint {index}")
            self._begin_edit_locked()
            del self._waypoints[index - 1]
            for i, wp in enumerate(self._waypoints, start=1):
                wp.index = i
            self._accept_locked()
            return True

    def clear(self) -> bool:
        with self._lock:
            if self._status is RouteStatus.RUNNING:
                return self._refuse(MSG_STOP_ROUTE_FIRST)
            self._begin_edit_locked()
            self._waypoints.clear()
            self._accept_locked()
            return True

    def set_add_mode(self, enabled: bool) -> None:
        with self._lock:
            self._add_mode = enabled
            self._accept_locked()

    def set_message(self, text: str | None) -> None:
        """Record a refusal or notice text, e.g. for a dropped map click."""
        with self._lock:
            self._message = text
            self._revision += 1

    def start(self) -> bool:
        with self._lock:
            if self._status is RouteStatus.RUNNING:
                return False
            if self._status not in (RouteStatus.PLANNING, RouteStatus.HELD):
                return self._refuse(MSG_EDIT_FIRST)
            target = next(
                (w for w in self._waypoints if w.state is not WaypointState.REACHED), None
            )
            if target is None:
                return self._refuse(MSG_NO_WAYPOINTS)
            target.state = WaypointState.CURRENT
            self._current_index = target.index
            self._status = RouteStatus.RUNNING
            self._accept_locked()
            return True

    def mark_current_reached(self) -> bool:
        with self._lock:
            if (
                self._status not in (RouteStatus.RUNNING, RouteStatus.HELD)
                or self._current_index is None
            ):
                return False
            current = self._waypoints[self._current_index - 1]
            current.state = WaypointState.REACHED
            nxt = next((w for w in self._waypoints if w.state is WaypointState.PENDING), None)
            if nxt is not None:
                nxt.state = WaypointState.CURRENT
                self._current_index = nxt.index
            elif self._status is RouteStatus.RUNNING:
                self._status = RouteStatus.COMPLETED
                self._current_index = None
            else:
                self._current_index = None
            self._accept_locked()
            return True

    def complete(self) -> bool:
        with self._lock:
            if (
                self._status not in (RouteStatus.RUNNING, RouteStatus.HELD)
                or self._current_index is None
            ):
                return False
            self._waypoints[self._current_index - 1].state = WaypointState.REACHED
            self._status = RouteStatus.COMPLETED
            self._current_index = None
            self._accept_locked()
            return True

    def fail_current(self) -> bool:
        with self._lock:
            if self._status is not RouteStatus.RUNNING or self._current_index is None:
                return False
            index = self._current_index
            self._waypoints[index - 1].state = WaypointState.FAILED
            self._status = RouteStatus.FAILED
            self._current_index = None
            self._message = f"waypoint {index} could not be reached"
            self._revision += 1
            return True

    def stop(self) -> bool:
        with self._lock:
            if self._status not in (RouteStatus.RUNNING, RouteStatus.HELD):
                return False
            self._status = RouteStatus.STOPPED
            self._current_index = None
            self._accept_locked()
            return True

    def interrupt(self) -> bool:
        with self._lock:
            if self._status not in (RouteStatus.RUNNING, RouteStatus.HELD):
                return False
            self._status = RouteStatus.INTERRUPTED
            self._current_index = None
            self._accept_locked()
            return True

    def hold(self) -> bool:
        with self._lock:
            if self._status is not RouteStatus.RUNNING:
                return False
            self._status = RouteStatus.HELD
            self._accept_locked()
            return True

    def _begin_edit_locked(self) -> None:
        if self._status in (RouteStatus.PLANNING, RouteStatus.RUNNING):
            return
        for wp in self._waypoints:
            wp.state = WaypointState.PENDING
        self._current_index = None
        self._status = RouteStatus.PLANNING

    def _accept_locked(self) -> None:
        self._message = None
        self._revision += 1

    def _refuse(self, message: str) -> bool:
        self._message = message
        self._revision += 1
        return False
