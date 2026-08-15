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

import json
import math
import random
import threading
import time
from typing import Any

import numpy as np

from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3, make_vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
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
    # Firmware reflex: intercept target sightings (own sensor or partner's
    # radio report) without waiting for the LLM. The agent keeps strategy
    # (negotiation, exploration, re-search); the reflex keeps pursuit sharp.
    auto_pursuit: bool = True
    # Coordinated-search parameters.
    sensor_range_m: float = 15.0
    # Desired radar-footprint overlap with the partner: separation target is
    # 2*sensor_range - radar_overlap (both circles overlap by ~this much).
    radar_overlap_m: float = 2.0
    # How far each search leg strides.
    search_step_m: float = 11.0
    # Consider the target reached within this distance (mission: < 1 m).
    arrive_radius_m: float = 0.9


class DroneSkillContainer(Module):
    """Flight + sweep skills bound to the navigation stack."""

    config: DroneSkillsConfig

    odom: In[PoseStamped]
    # Simulated target sensor (LOS referee publishes here): latest visibility
    # report, e.g. "VISIBLE 4.2 -1.5" or "LOST".
    target_event: In[str]
    # Every navigation goal this drone commits to, as JSON {x, y, kind} with
    # kind "destination" (an LLM-level fly_to decision) or "hop" (one waypoint
    # of the follower). The referee renders these as ground circles so goal
    # cadence is visible in the video.
    goal_marker: Out[str]
    # Direct velocity override (MovementManager gives teleop priority over the
    # planner) — used by intercept() for straight-line pursuit.
    tele_cmd_vel: Out[Twist]
    # Partner sightings relayed by the radio ("x y") — auto-pursuit input.
    partner_sighting: In[str]
    # Own-sensor sightings pushed to the radio for automatic broadcast, so the
    # partner converges even if this drone's LLM is slow to report.
    auto_sighting: Out[str]
    # This stack's radio beliefs about the partner (JSON from RadioModule) —
    # coordinated search reads the partner's last-known position from here.
    peer_belief: In[str]
    # Own costmap: detected walls/obstacles feed the away-from-walls prior.
    global_costmap: In[OccupancyGrid]

    _navigation: NavigationInterfaceSpec

    _latest_odom: PoseStamped | None = None
    _latest_target_event: str = "no sensor reports yet"
    _sweep_thread: threading.Thread | None = None
    _sweep_cancel: threading.Event
    _sweep_status: str = "idle"
    _last_pursuit: tuple[float, float, float] = (1e9, 1e9, 0.0)

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._sweep_cancel = threading.Event()
        self._stopping = threading.Event()
        self._supervisor: threading.Thread | None = None
        self._search_armed = False  # set once the agent starts searching
        # Coordinated-search memory (all built from LOCAL knowledge + radio):
        self._visited: list[tuple[float, float]] = []  # own covered points
        self._visited_at = 0.0
        self._peer_pos: tuple[float, float] | None = None  # last radio belief
        self._peer_trail: list[tuple[float, float]] = []  # partner coverage
        self._walls: np.ndarray | None = None  # Nx2 occupied points (world)
        self._last_heading = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))
        self.register_disposable(Disposable(self.target_event.subscribe(self._on_target_event)))
        if self.partner_sighting.transport is not None:
            self.register_disposable(
                Disposable(self.partner_sighting.subscribe(self._on_partner_sighting))
            )
        if self.peer_belief.transport is not None:
            self.register_disposable(
                Disposable(self.peer_belief.subscribe(self._on_peer_belief))
            )
        if self.global_costmap.transport is not None:
            self.register_disposable(
                Disposable(self.global_costmap.subscribe(self._on_costmap))
            )
        self._supervisor = threading.Thread(target=self._supervisor_loop, daemon=True)
        self._supervisor.start()

    @rpc
    def stop(self) -> None:
        self._stopping.set()
        self._sweep_cancel.set()
        if self._sweep_thread and self._sweep_thread.is_alive():
            self._sweep_thread.join(timeout=2.0)
        super().stop()

    def _supervisor_loop(self) -> None:
        """Never idle: once search has been commanded, resume it whenever a
        movement task ends (pursuit finished, target lost, leg timed out)."""
        while not self._stopping.wait(4.0):
            if not self.config.auto_pursuit or not self._search_armed:
                continue
            thread = self._sweep_thread
            if thread is not None and thread.is_alive():
                continue
            if self._latest_target_event.startswith("VISIBLE"):
                continue  # the pursuit reflex owns movement right now
            try:
                logger.info("resuming coordinated search (idle after task)")
                self.begin_coordinated_search()
            except Exception:
                logger.warning("search resume failed", exc_info=True)

    def _on_odom(self, odom: PoseStamped) -> None:
        self._latest_odom = odom
        # Coverage memory: sample own position (~1/s, min 2 m apart).
        now = time.time()
        if now - self._visited_at > 1.0:
            p = (odom.position.x, odom.position.y)
            if not self._visited or math.hypot(
                p[0] - self._visited[-1][0], p[1] - self._visited[-1][1]
            ) > 2.0:
                self._visited.append(p)
                if len(self._visited) > 800:
                    self._visited.pop(0)
            self._visited_at = now

    def _on_peer_belief(self, raw: str) -> None:
        try:
            data = json.loads(raw)
            for _, b in data.get("beliefs", {}).items():
                pos = b.get("position")
                if pos is not None:
                    p = (float(pos[0]), float(pos[1]))
                    self._peer_pos = p
                    if not self._peer_trail or math.hypot(
                        p[0] - self._peer_trail[-1][0], p[1] - self._peer_trail[-1][1]
                    ) > 2.0:
                        self._peer_trail.append(p)
                        if len(self._peer_trail) > 800:
                            self._peer_trail.pop(0)
        except (TypeError, ValueError):
            pass

    def _on_costmap(self, grid: OccupancyGrid) -> None:
        try:
            occ = np.argwhere(grid.grid >= 80)
            if occ.size == 0:
                self._walls = None
                return
            if occ.shape[0] > 400:
                occ = occ[:: occ.shape[0] // 400 + 1]
            res = grid.resolution
            ox = grid.origin.position.x
            oy = grid.origin.position.y
            # rows = y, columns = x (see mapping/pointclouds/occupancy.py)
            world = np.stack(
                [occ[:, 1] * res + ox, occ[:, 0] * res + oy], axis=1
            )
            self._walls = world
        except Exception:
            pass

    def _on_target_event(self, event: str) -> None:
        self._latest_target_event = event
        # Firmware reflex: own sensor contact → pursue immediately.
        if self.config.auto_pursuit and event.startswith("VISIBLE"):
            try:
                # "VISIBLE at (x, y)"
                inside = event[event.index("(") + 1 : event.index(")")]
                x_s, y_s = inside.split(",")
                self._auto_pursue(float(x_s), float(y_s), source="own sensor")
            except (ValueError, IndexError):
                pass

    def _on_partner_sighting(self, raw: str) -> None:
        # Partner radioed a sighting → converge unless we have our own contact
        # (own sensor is fresher than a relayed report).
        if not self.config.auto_pursuit:
            return
        if self._latest_target_event.startswith("VISIBLE"):
            return
        try:
            x_s, y_s = raw.split()
            self._auto_pursue(float(x_s), float(y_s), source="partner radio")
        except ValueError:
            pass

    def _auto_pursue(self, x: float, y: float, source: str) -> None:
        """Start/redirect the straight-line intercept, rate-limited so a
        stream of updates doesn't thrash the pursuit thread."""
        now = time.time()
        lx, ly, lt = self._last_pursuit
        if now - lt < 3.0 and math.hypot(x - lx, y - ly) < 2.0:
            return
        self._last_pursuit = (x, y, now)
        logger.info(f"auto-pursuit ({source}) -> ({x:.1f}, {y:.1f})")
        if source == "own sensor":
            try:
                self.auto_sighting.publish(f"{x} {y}")
            except Exception:
                pass
        self.intercept(x, y)

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
        self._mark(x, y, "destination")
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

    def _mark(self, x: float, y: float, kind: str) -> None:
        try:
            self.goal_marker.publish(
                json.dumps({"x": round(x, 2), "y": round(y, 2), "kind": kind})
            )
        except Exception:
            pass

    def _start_path(self, waypoints: list[tuple[float, float]], label: str) -> None:
        """Follow waypoints in a background thread: hop, retry frontier
        failures once (the map keeps growing as we fly), skip dead hops."""
        self._sweep_cancel = threading.Event()
        cancel = self._sweep_cancel

        def _goto(wx: float, wy: float) -> str:
            """Fly one hop. Returns reached | failed | cancelled."""
            self._mark(wx, wy, "hop")
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

    @skill(uses=[CAP_MOVEMENT])
    def intercept(self, x: float, y: float) -> str:
        """INTERCEPT: fly STRAIGHT at map coordinates (x, y) at full speed —
        use this to converge on a target sighting (yours or your partner's).
        Falls back to obstacle-avoiding navigation if the straight line is
        blocked. Non-blocking; re-issue with new coordinates as the target
        moves; cancel with stop_moving."""
        self._cancel_sweep()
        if self._latest_odom is None:
            return "No odometry yet — cannot intercept."
        x, y = self._clamp(x, y)
        self._mark(x, y, "destination")
        self._sweep_cancel = threading.Event()
        cancel = self._sweep_cancel

        def run() -> None:
            self._sweep_status = f"intercept ({x:.1f}, {y:.1f}): direct"
            result = self._direct_fly(
                x, y, cancel, arrive_r=self.config.arrive_radius_m, timeout=150.0
            )
            if result == "blocked":
                od = self._latest_odom
                if od is None or cancel.is_set():
                    return
                px, py = od.position.x, od.position.y
                dist = math.hypot(x - px, y - py)
                self._sweep_status = f"intercept ({x:.1f}, {y:.1f}): blocked, replanning"
                hops = max(1, math.ceil(dist / _MAX_HOP))
                waypoints = [
                    (px + (x - px) * h / hops, py + (y - py) * h / hops)
                    for h in range(1, hops + 1)
                ]
                self._start_path(waypoints, f"intercept ({x:.1f}, {y:.1f})")
                return
            self._sweep_status = f"intercept ({x:.1f}, {y:.1f}): {result}"

        self._sweep_thread = threading.Thread(target=run, daemon=True)
        self._sweep_thread.start()
        return (
            f"INTERCEPTING ({x:.1f}, {y:.1f}) — straight-line at full speed. "
            f"Re-issue intercept with new coordinates as the target moves."
        )

    def _direct_fly(
        self, x: float, y: float, cancel: threading.Event, arrive_r: float, timeout: float
    ) -> str:
        """Straight-line velocity flight to (x, y). Returns arrived | blocked |
        cancelled | timeout. Zeroes the teleop stream on exit."""
        prog_pos = None
        prog_t = time.time()
        t0 = time.time()
        try:
            while not cancel.is_set() and time.time() - t0 < timeout:
                od = self._latest_odom
                if od is None:
                    time.sleep(0.2)
                    continue
                px, py = od.position.x, od.position.y
                dx, dy = x - px, y - py
                dist = math.hypot(dx, dy)
                if dist < arrive_r:
                    return "arrived"
                cur = (px, py)
                if prog_pos is None or math.hypot(cur[0] - prog_pos[0], cur[1] - prog_pos[1]) > 0.4:
                    prog_pos = cur
                    prog_t = time.time()
                elif time.time() - prog_t > 6.0:
                    return "blocked"
                # Point the nose where we are going: the target sensor is a
                # forward cone, so a drone that strafes with a frozen heading
                # searches with its eyes shut. Turn toward the goal, and let
                # forward speed fall off with heading error.
                q = od.orientation
                yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                )
                desired = math.atan2(dy, dx)
                err = (desired - yaw + math.pi) % (2 * math.pi) - math.pi
                turn = max(-1.0, min(1.0, 2.0 * err))
                speed = max(0.0, math.cos(err)) * min(1.0, dist / 2.0)
                # A little strafe keeps progress while the turn completes.
                strafe = 0.35 * math.sin(err) * min(1.0, dist / 4.0)
                self.tele_cmd_vel.publish(
                    Twist(
                        Vector3(speed, strafe, 0.0),
                        Vector3(0.0, 0.0, turn),
                    )
                )
                time.sleep(0.12)
            return "cancelled" if cancel.is_set() else "timeout"
        finally:
            self.tele_cmd_vel.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))

    # -- coordinated search ----------------------------------------------------

    def _min_dist(self, points: list[tuple[float, float]], x: float, y: float) -> float:
        if not points:
            return 1e9
        return min(math.hypot(x - p[0], y - p[1]) for p in points[-300:])

    def _pick_search_goal(self) -> tuple[float, float] | None:
        od = self._latest_odom
        if od is None:
            return None
        px, py = od.position.x, od.position.y
        cfg = self.config
        desired_sep = 2 * cfg.sensor_range_m - cfg.radar_overlap_m
        walls = self._walls
        best, best_score = None, -1e18
        for k in range(16):
            heading = k * math.pi / 8
            cx = px + cfg.search_step_m * math.cos(heading)
            cy = py + cfg.search_step_m * math.sin(heading)
            cx, cy = self._clamp(cx, cy)
            score = random.uniform(0.0, 1.0)
            # Away-from-detected-walls prior: penalize candidates close to any
            # occupied cell this drone's OWN lidar has mapped.
            if walls is not None and walls.size:
                d_wall = float(np.min(np.hypot(walls[:, 0] - cx, walls[:, 1] - cy)))
                if d_wall < 8.0:
                    score -= (8.0 - d_wall) * 3.0
            # Radar-overlap spacing: stay ~(2R - overlap) from the partner's
            # last radioed position; only being too close is penalized.
            if self._peer_pos is not None:
                d_peer = math.hypot(cx - self._peer_pos[0], cy - self._peer_pos[1])
                if d_peer < desired_sep:
                    score -= (desired_sep - d_peer) * 1.5
            # Don't re-cover ground either drone already searched.
            d_self = self._min_dist(self._visited, cx, cy)
            if d_self < cfg.sensor_range_m * 0.75:
                score -= (cfg.sensor_range_m * 0.75 - d_self) * 1.0
            d_peer_trail = self._min_dist(self._peer_trail, cx, cy)
            if d_peer_trail < cfg.sensor_range_m * 0.75:
                score -= (cfg.sensor_range_m * 0.75 - d_peer_trail) * 1.0
            # Mild momentum: keep striding the same way when scores tie.
            score += 1.5 * math.cos(heading - self._last_heading)
            if score > best_score:
                best_score, best = score, (cx, cy, heading)
        if best is None:
            return None
        self._last_heading = best[2]
        return (best[0], best[1])

    @skill(uses=[CAP_MOVEMENT])
    def begin_coordinated_search(self) -> str:
        """Search the arena cooperatively: repeatedly strides toward unexplored
        space, away from detected walls, keeping ~(2*sensor_range - overlap)
        separation from your partner's last known position and avoiding areas
        either of you already covered. Runs until cancelled (stop_moving) or
        until a sighting triggers the intercept reflex."""
        self._cancel_sweep()
        if self._latest_odom is None:
            return "No odometry yet — cannot search."
        self._search_armed = True
        self._sweep_cancel = threading.Event()
        cancel = self._sweep_cancel

        def run() -> None:
            legs = 0
            while not cancel.is_set():
                goal = self._pick_search_goal()
                if goal is None:
                    time.sleep(1.0)
                    continue
                gx, gy = goal
                legs += 1
                self._mark(gx, gy, "hop")
                self._sweep_status = f"coordinated search: leg {legs} -> ({gx:.1f}, {gy:.1f})"
                result = self._direct_fly(gx, gy, cancel, arrive_r=2.5, timeout=45.0)
                if result == "blocked":
                    # Remember the blockage as a virtual wall so scoring learns.
                    od = self._latest_odom
                    if od is not None:
                        p = np.array([[od.position.x, od.position.y]])
                        self._walls = p if self._walls is None else np.vstack([self._walls, p])
                if result == "cancelled":
                    self._sweep_status = "coordinated search: cancelled"
                    return
            self._sweep_status = "coordinated search: stopped"

        self._sweep_thread = threading.Thread(target=run, daemon=True)
        self._sweep_thread.start()
        sep = 2 * self.config.sensor_range_m - self.config.radar_overlap_m
        return (
            f"Coordinated search running: striding {self.config.search_step_m:.0f} m legs "
            f"toward unexplored space, keeping ~{sep:.0f} m from partner "
            f"(radar overlap {self.config.radar_overlap_m:.0f} m), away from mapped walls."
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
        """Stop: cancel the current flight goal and any running search."""
        self._search_armed = False
        self._cancel_sweep()
        self.tele_cmd_vel.publish(Twist(Vector3(0, 0, 0), Vector3(0, 0, 0)))
        try:
            self._navigation.cancel_goal()
        except Exception:
            logger.warning("cancel_goal failed", exc_info=True)
        return "Stopped. Awaiting next command."

    def _cancel_sweep(self) -> None:
        self._sweep_cancel.set()
        self._sweep_status = "idle"
