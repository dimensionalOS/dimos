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

"""Coordinate-based flight skills for the two-drone dimsim demo.

The drone flies at a fixed cruise altitude; the ground navigation stack
(ReplanningAStarPlanner) plans obstacle-free paths in the plane, and the
dimsim flight embodiment holds altitude (planner twists have linear.z = 0).
"""

import threading
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.core.core import rpc
from dimos.core.module import Module
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import make_vector3
from dimos.navigation.base import NavigationState
from dimos.navigation.navigation_spec import NavigationInterfaceSpec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _sweep_waypoints(
    x_min: float, y_min: float, x_max: float, y_max: float, lane_spacing: float
) -> list[tuple[float, float]]:
    """Boustrophedon (lawnmower) sweep of the rectangle, lanes along x."""
    if x_max < x_min:
        x_min, x_max = x_max, x_min
    if y_max < y_min:
        y_min, y_max = y_max, y_min
    lane_spacing = max(0.5, lane_spacing)
    points: list[tuple[float, float]] = []
    y = y_min
    left_to_right = True
    while y <= y_max + 1e-6:
        if left_to_right:
            points += [(x_min, y), (x_max, y)]
        else:
            points += [(x_max, y), (x_min, y)]
        left_to_right = not left_to_right
        y += lane_spacing
    return points


class DroneSkillContainer(Module):
    """Flight + sweep skills bound to the navigation stack."""

    odom: In[PoseStamped]
    # Simulated target sensor (LOS referee publishes here): latest visibility
    # report, e.g. "VISIBLE 4.2 -1.5" or "LOST".
    target_event: In[str]

    _navigation: NavigationInterfaceSpec

    _latest_odom: PoseStamped | None = None
    _latest_target_event: str = "no sensor reports yet"
    _sweep_thread: threading.Thread | None = None
    _sweep_cancel: threading.Event
    _sweep_status: str = "idle"

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._sweep_cancel = threading.Event()

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))
        self.register_disposable(Disposable(self.target_event.subscribe(self._on_target_event)))

    @rpc
    def stop(self) -> None:
        self._sweep_cancel.set()
        if self._sweep_thread and self._sweep_thread.is_alive():
            self._sweep_thread.join(timeout=2.0)
        super().stop()

    def _on_odom(self, odom: PoseStamped) -> None:
        self._latest_odom = odom

    def _on_target_event(self, event: str) -> None:
        self._latest_target_event = event

    # -- skills ---------------------------------------------------------------

    @skill
    def check_target_sensor(self) -> str:
        """Latest reading of your target sensor: whether YOU currently see the
        target (line of sight) and at what coordinates."""
        return f"Target sensor: {self._latest_target_event}"

    @skill(uses=[CAP_MOVEMENT])
    def fly_to(self, x: float, y: float) -> str:
        """Fly to map coordinates (x, y) meters, avoiding obstacles. Non-blocking:
        check progress with get_status. Cancels any running area sweep."""
        self._cancel_sweep()
        accepted = self._navigation.set_goal(
            PoseStamped(
                position=make_vector3(x, y, 0.0),
                orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
                frame_id="map",
            )
        )
        if not accepted:
            return f"Goal ({x:.1f}, {y:.1f}) rejected by the planner (unreachable or in an obstacle)."
        return f"Flying to ({x:.1f}, {y:.1f}). Check with get_status; cancel with stop_moving."

    @skill(uses=[CAP_MOVEMENT])
    def sweep_area(
        self,
        x_min: float,
        y_min: float,
        x_max: float,
        y_max: float,
        lane_spacing: float = 2.5,
    ) -> str:
        """Systematically sweep (lawnmower pattern) the rectangle [x_min..x_max] x
        [y_min..y_max] to search it. Non-blocking: runs until done or cancelled;
        progress via get_status, cancel with stop_moving or a new command."""
        self._cancel_sweep()
        waypoints = _sweep_waypoints(x_min, y_min, x_max, y_max, lane_spacing)
        if not waypoints:
            return "Empty sweep area."
        self._sweep_cancel = threading.Event()
        cancel = self._sweep_cancel

        def run() -> None:
            self._sweep_status = f"sweeping 0/{len(waypoints)}"
            for i, (wx, wy) in enumerate(waypoints):
                if cancel.is_set():
                    self._sweep_status = "cancelled"
                    return
                self._navigation.set_goal(
                    PoseStamped(
                        position=make_vector3(wx, wy, 0.0),
                        orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
                        frame_id="map",
                    )
                )
                # Wait for this waypoint (or timeout and move on).
                t0 = time.time()
                while not cancel.is_set() and time.time() - t0 < 45.0:
                    if self._navigation.is_goal_reached():
                        break
                    time.sleep(0.3)
                self._sweep_status = f"sweeping {i + 1}/{len(waypoints)}"
            self._sweep_status = "sweep complete"

        self._sweep_thread = threading.Thread(target=run, daemon=True)
        self._sweep_thread.start()
        return (
            f"Sweeping rectangle ({x_min:.1f},{y_min:.1f})..({x_max:.1f},{y_max:.1f}) "
            f"in {len(waypoints)} waypoints (lanes every {lane_spacing:.1f} m)."
        )

    @skill
    def get_status(self) -> str:
        """Current position, navigation state, and sweep progress."""
        pos = "unknown"
        if self._latest_odom is not None:
            p = self._latest_odom.position
            pos = f"({p.x:.1f}, {p.y:.1f}) altitude {p.z:.1f} m"
        try:
            nav_state = self._navigation.get_state()
            nav = nav_state.value if isinstance(nav_state, NavigationState) else str(nav_state)
        except Exception:
            nav = "unknown"
        return f"Position {pos}. Navigation: {nav}. Sweep: {self._sweep_status}."

    @skill
    def stop_moving(self) -> str:
        """Stop: cancel the current flight goal and any running sweep."""
        self._cancel_sweep()
        try:
            self._navigation.cancel_goal()
        except Exception:
            logger.warning("cancel_goal failed", exc_info=True)
        return "Stopped. Awaiting next command."

    def _cancel_sweep(self) -> None:
        self._sweep_cancel.set()
        self._sweep_status = "idle"
