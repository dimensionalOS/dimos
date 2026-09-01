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

Navigation-oriented skills over the abstract ``NavigationInterfaceSpec``.
Object targets come from a ground-truth table configured by the blueprint
(the simulation scene's object poses) rather than a perception pipeline -
this is the "super basic" object-walking demo, honest about being in sim.
"""

from __future__ import annotations

import math
import time

import numpy as np
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.navigation.base import NavigationState
from dimos.navigation.navigation_spec import NavigationInterfaceSpec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Stop this far from an object's center so the duck ends up next to it
# instead of trying to stand inside it.
_APPROACH_DISTANCE = 0.35


def _yaw_quaternion(yaw: float) -> Quaternion:
    return Quaternion(0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class MicroduckSkillContainerConfig(ModuleConfig):
    # name -> (x, y) world coordinates of interesting objects in the scene.
    objects: dict[str, tuple[float, float]] = {}


class MicroduckSkillContainer(Module):
    """Skills the agent can call to move the Microduck around its room."""

    config: MicroduckSkillContainerConfig

    _navigation: NavigationInterfaceSpec

    odom: In[PoseStamped]

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._latest_odom: PoseStamped | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_odom(self, pose: PoseStamped) -> None:
        self._latest_odom = pose

    def _robot_xy(self) -> tuple[float, float] | None:
        odom = self._latest_odom
        if odom is None:
            return None
        return (float(odom.position.x), float(odom.position.y))

    @skill
    def list_objects(self) -> str:
        """List the known objects in the room and their positions in meters."""
        if not self.config.objects:
            return "No objects are registered in this scene."
        robot = self._robot_xy()
        lines = []
        for name, (x, y) in self.config.objects.items():
            entry = f"- {name} at ({x:.2f}, {y:.2f})"
            if robot is not None:
                entry += f", {math.hypot(x - robot[0], y - robot[1]):.2f} m away"
            lines.append(entry)
        return "Objects in the room:\n" + "\n".join(lines)

    @skill
    def go_to_object(self, name: str) -> str:
        """Walk to a named object in the room and stop right next to it,
        facing it. Use list_objects to see what exists.

        Args:
            name: The object's name, e.g. "red_ball".
        """
        key = name.strip().lower().replace(" ", "_")
        if key not in self.config.objects:
            known = ", ".join(self.config.objects) or "(none)"
            return f"Unknown object '{name}'. Known objects: {known}"

        ox, oy = self.config.objects[key]
        robot = self._robot_xy()
        if robot is None:
            return "Robot position unknown (no odometry yet); try again shortly."

        dx, dy = ox - robot[0], oy - robot[1]
        distance = math.hypot(dx, dy)
        if distance <= _APPROACH_DISTANCE + 0.05:
            return f"Already next to {key} ({distance:.2f} m away)."

        heading = math.atan2(dy, dx)
        gx = ox - math.cos(heading) * _APPROACH_DISTANCE
        gy = oy - math.sin(heading) * _APPROACH_DISTANCE
        outcome = self._navigate_to(gx, gy, heading)
        robot = self._robot_xy()
        where = "unknown" if robot is None else f"({robot[0]:.2f}, {robot[1]:.2f})"
        return f"{outcome} while walking to {key}. Robot is now at {where}."

    @skill
    def move_to(self, x: float, y: float) -> str:
        """Walk to world coordinates (x, y) in meters and wait for arrival.

        The room spans roughly x in [-2, 2] and y in [-1.5, 1.5].
        """
        robot = self._robot_xy()
        heading = None
        if robot is not None:
            heading = math.atan2(float(y) - robot[1], float(x) - robot[0])
        outcome = self._navigate_to(float(x), float(y), heading)
        robot = self._robot_xy()
        where = "unknown" if robot is None else f"({robot[0]:.2f}, {robot[1]:.2f})"
        return f"{outcome}. Robot is now at {where}."

    @skill
    def stop_moving(self) -> str:
        """Stop the current navigation goal; the robot stands in place."""
        cancelled = self._navigation.cancel_goal()
        return "Stopped." if cancelled else "There was no active navigation goal."

    @skill
    def where_am_i(self) -> str:
        """Report the robot's current position in world coordinates."""
        robot = self._robot_xy()
        if robot is None:
            return "Robot position unknown (no odometry yet)."
        return f"Robot is at ({robot[0]:.2f}, {robot[1]:.2f})."

    @skill
    def wait(self, seconds: float) -> str:
        """Wait in place for the given number of seconds (max 30)."""
        seconds = float(np.clip(seconds, 0.0, 30.0))
        time.sleep(seconds)
        return f"Waited {seconds:.0f} s."

    def _navigate_to(self, x: float, y: float, heading: float | None) -> str:
        goal = PoseStamped(
            ts=time.time(),
            frame_id="world",
            position=Vector3(x, y, 0.0),
            orientation=_yaw_quaternion(heading) if heading is not None else Quaternion(),
        )
        if not self._navigation.set_goal(goal):
            return "Navigation rejected the goal (is the map ready?)"
        return self._wait_for_goal()

    def _wait_for_goal(self, timeout: float = 180.0, settle: float = 3.0) -> str:
        """Block until arrival, cancellation, or timeout.

        Arrival (`is_goal_reached`) only counts after the planner has been
        seen FOLLOWING_PATH for *this* goal - the flag may still be latched
        from a previous goal (e.g. one an exploration cycle just finished).
        The planner also leaves FOLLOWING_PATH briefly on every replan, so a
        pause only counts as the end after `settle` seconds.
        """
        deadline = time.monotonic() + timeout
        started = False
        start_deadline = time.monotonic() + 15.0
        while not started and time.monotonic() < start_deadline:
            if self._navigation.get_state() == NavigationState.FOLLOWING_PATH:
                started = True
            else:
                time.sleep(0.2)
        if not started:
            return "Navigation never started following a path (no route found?)"

        idle_since: float | None = None
        while time.monotonic() < deadline:
            if self._navigation.is_goal_reached():
                return "Arrived"
            if self._navigation.get_state() == NavigationState.FOLLOWING_PATH:
                idle_since = None
            elif idle_since is None:
                idle_since = time.monotonic()
            elif time.monotonic() - idle_since > settle:
                return "Navigation stopped early (cancelled or no path)"
            time.sleep(0.2)
        return "Navigation timed out"
