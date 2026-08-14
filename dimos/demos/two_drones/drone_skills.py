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

import math
import threading
import time
from typing import Any

from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import make_vector3
from dimos.navigation.base import NavigationState
from dimos.navigation.navigation_spec import NavigationInterfaceSpec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


# The planner only accepts goals in already-mapped space, and the lidar maps a
# ~4 m radius: hops longer than that land in unknown cells and get rejected.
_MAX_HOP = 2.5


def _sweep_waypoints(
    x_min: float,
    y_min: float,
    x_max: float,
    y_max: float,
    lane_spacing: float,
    start: tuple[float, float] | None = None,
) -> list[tuple[float, float]]:
    """Boustrophedon (lawnmower) sweep of the rectangle, lanes along x,
    subdivided into short hops so each next goal is inside mapped space.

    ``start`` (the drone's current position) anchors the path: the approach to
    the first lane corner is subdivided too, otherwise a sector claimed far
    from the drone begins with an unreachable first goal and the whole sweep
    dies on the launch pad.
    """
    if x_max < x_min:
        x_min, x_max = x_max, x_min
    if y_max < y_min:
        y_min, y_max = y_max, y_min
    lane_spacing = max(0.5, lane_spacing)
    corners: list[tuple[float, float]] = []
    y = y_min
    left_to_right = True
    while y <= y_max + 1e-6:
        if left_to_right:
            corners += [(x_min, y), (x_max, y)]
        else:
            corners += [(x_max, y), (x_min, y)]
        left_to_right = not left_to_right
        y += lane_spacing

    if start is not None and corners:
        # Enter the sweep at the nearest end of the nearest lane.
        nearest = min(
            range(0, len(corners), 2),
            key=lambda i: min(
                math.hypot(corners[i][0] - start[0], corners[i][1] - start[1]),
                math.hypot(corners[i + 1][0] - start[0], corners[i + 1][1] - start[1]),
            ),
        )
        head, tail = corners[:nearest], corners[nearest:]
        corners = tail + list(reversed(head))
        if math.hypot(corners[1][0] - start[0], corners[1][1] - start[1]) < math.hypot(
            corners[0][0] - start[0], corners[0][1] - start[1]
        ):
            corners[0], corners[1] = corners[1], corners[0]
        corners = [start, *corners]

    points: list[tuple[float, float]] = []
    for i, (cx, cy) in enumerate(corners):
        if i == 0:
            if start is None:
                points.append((cx, cy))
            continue
        px, py = corners[i - 1]
        dist = math.hypot(cx - px, cy - py)
        hops = max(1, math.ceil(dist / _MAX_HOP))
        for h in range(1, hops + 1):
            points.append((px + (cx - px) * h / hops, py + (cy - py) * h / hops))
    return points


class DroneSkillsConfig(ModuleConfig):
    # Flyable interior of the arena. Goals are clamped inside so a sweep
    # corner claimed at the perimeter wall doesn't wedge the planner into
    # an endless "following_path" crawl against the wall.
    bound_x: float = 10.2
    bound_y: float = 6.2


class DroneSkillContainer(Module):
    """Flight + sweep skills bound to the navigation stack."""

    config: DroneSkillsConfig

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
        """Fly to map coordinates (x, y) meters, avoiding obstacles. Works even
        to far/unexplored coordinates (approaches in incremental hops).
        Non-blocking: check progress with get_status. Cancels any running sweep."""
        self._cancel_sweep()
        if self._latest_odom is None:
            return "No odometry yet — cannot fly."
        x, y = self._clamp(x, y)
        sx, sy = self._latest_odom.position.x, self._latest_odom.position.y
        dist = math.hypot(x - sx, y - sy)
        hops = max(1, math.ceil(dist / _MAX_HOP))
        waypoints = [
            (sx + (x - sx) * h / hops, sy + (y - sy) * h / hops) for h in range(1, hops + 1)
        ]
        self._start_path(waypoints, f"approach ({x:.1f}, {y:.1f})")
        return (
            f"Flying to ({x:.1f}, {y:.1f}) in {hops} hop(s), avoiding obstacles. "
            f"Check with get_status; cancel with stop_moving."
        )

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
        start = None
        if self._latest_odom is not None:
            start = (self._latest_odom.position.x, self._latest_odom.position.y)
        x_min, y_min = self._clamp(x_min, y_min)
        x_max, y_max = self._clamp(x_max, y_max)
        waypoints = _sweep_waypoints(x_min, y_min, x_max, y_max, lane_spacing, start=start)
        if not waypoints:
            return "Empty sweep area."
        self._start_path(
            waypoints,
            f"sweep ({x_min:.0f},{y_min:.0f})..({x_max:.0f},{y_max:.0f})",
        )
        return (
            f"Sweeping rectangle ({x_min:.1f},{y_min:.1f})..({x_max:.1f},{y_max:.1f}) "
            f"in {len(waypoints)} waypoints (lanes every {lane_spacing:.1f} m)."
        )

    def _clamp(self, x: float, y: float) -> tuple[float, float]:
        bx, by = self.config.bound_x, self.config.bound_y
        return max(-bx, min(bx, x)), max(-by, min(by, y))

    def _start_path(self, waypoints: list[tuple[float, float]], label: str) -> None:
        """Follow waypoints in a background thread: hop, retry frontier
        failures once (the map keeps growing as we fly), skip dead hops."""
        self._sweep_cancel = threading.Event()
        cancel = self._sweep_cancel

        def _goto(wx: float, wy: float) -> str:
            """Fly one hop. Returns reached | failed | cancelled."""
            self._navigation.set_goal(
                PoseStamped(
                    position=make_vector3(wx, wy, 0.0),
                    orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
                    frame_id="map",
                )
            )
            t0 = time.time()
            prog_t = t0
            prog_pos = (
                (self._latest_odom.position.x, self._latest_odom.position.y)
                if self._latest_odom
                else None
            )
            while not cancel.is_set() and time.time() - t0 < 30.0:
                if self._navigation.is_goal_reached():
                    return "reached"
                # The planner cancels unreachable goals (unknown space, inside
                # an obstacle) and drops back to idle — detect that quickly
                # instead of stalling the whole path.
                if time.time() - t0 > 4.0:
                    try:
                        state = self._navigation.get_state()
                        state_val = (
                            state.value if isinstance(state, NavigationState) else str(state)
                        )
                        if "idle" in str(state_val).lower():
                            return "failed"
                    except Exception:
                        pass
                # No-progress watchdog: the planner can sit in following_path
                # while grinding against a wall or a degenerate path. If the
                # drone hasn't moved 0.4 m in 10 s, give up on this hop.
                if self._latest_odom is not None:
                    cur = (self._latest_odom.position.x, self._latest_odom.position.y)
                    if prog_pos is None or math.hypot(cur[0] - prog_pos[0], cur[1] - prog_pos[1]) > 0.4:
                        prog_pos = cur
                        prog_t = time.time()
                    elif time.time() - prog_t > 10.0:
                        try:
                            self._navigation.cancel_goal()
                        except Exception:
                            pass
                        return "failed"
                time.sleep(1.0)
            return "cancelled" if cancel.is_set() else "failed"

        def run() -> None:
            self._sweep_status = f"{label}: 0/{len(waypoints)}"
            skipped = 0
            for i, (wx, wy) in enumerate(waypoints):
                if cancel.is_set():
                    self._sweep_status = f"{label}: cancelled"
                    return
                result = _goto(wx, wy)
                if result == "failed" and not cancel.is_set():
                    # The map may still be catching up (hop at the frontier):
                    # give the mapper a moment and retry once before skipping.
                    time.sleep(2.5)
                    result = _goto(wx, wy)
                if result == "failed":
                    skipped += 1
                if cancel.is_set():
                    self._sweep_status = f"{label}: cancelled"
                    return
                self._sweep_status = f"{label}: {i + 1}/{len(waypoints)} ({skipped} hops skipped)"
            self._sweep_status = f"{label}: complete ({skipped} hops unreachable)"

        self._sweep_thread = threading.Thread(target=run, daemon=True)
        self._sweep_thread.start()

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
