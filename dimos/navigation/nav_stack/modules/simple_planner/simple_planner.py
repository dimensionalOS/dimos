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

from collections.abc import Callable
from dataclasses import dataclass
import heapq
import math
import threading
import time
from typing import Any

import numpy as np
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_stack.frames import FRAME_BODY, FRAME_MAP, FRAME_ODOM
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class Costmap:
    def __init__(self, cell_size: float, obstacle_height: float, inflation_radius: float) -> None:
        if cell_size <= 0.0:
            raise ValueError(f"cell_size must be positive, got {cell_size}")
        if inflation_radius < 0.0:
            raise ValueError(f"inflation_radius must be non-negative, got {inflation_radius}")
        self.cell_size = float(cell_size)
        self.obstacle_height = float(obstacle_height)
        self.inflation_radius = float(inflation_radius)
        # Raw heights observed per cell (max-ever). Keyed by (ix, iy).
        self._heights: dict[tuple[int, int], float] = {}
        # Inflated blocked set (recomputed lazily).
        self._blocked: set[tuple[int, int]] = set()
        self._blocked_dirty = True

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        return (math.floor(x / self.cell_size), math.floor(y / self.cell_size))

    def cell_to_world(self, ix: int, iy: int) -> tuple[float, float]:
        # Return cell center.
        return ((ix + 0.5) * self.cell_size, (iy + 0.5) * self.cell_size)

    def update(self, x: float, y: float, height: float) -> None:
        """Record an obstacle-candidate point. Height is elevation above ground."""
        key = self.world_to_cell(x, y)
        prev = self._heights.get(key, float("-inf"))
        if height > prev:
            self._heights[key] = height
            self._blocked_dirty = True

    def clear(self) -> None:
        self._heights.clear()
        self._blocked.clear()
        self._blocked_dirty = False

    def is_blocked(self, ix: int, iy: int) -> bool:
        if self._blocked_dirty:
            self._rebuild_blocked()
        return (ix, iy) in self._blocked

    def _rebuild_blocked(self) -> None:
        """Build the inflated obstacle set from the raw height map."""
        blocked: set[tuple[int, int]] = set()
        # Inflation: the number of cells that lie within inflation_radius.
        r_cells = math.ceil(self.inflation_radius / self.cell_size)
        for (ix, iy), h in list(self._heights.items()):
            if h < self.obstacle_height:
                continue
            if r_cells == 0:
                blocked.add((ix, iy))
                continue
            # Circle inflation (squared comparison to avoid sqrt per cell)
            max_sq = (self.inflation_radius / self.cell_size) ** 2
            for dx in range(-r_cells, r_cells + 1):
                for dy in range(-r_cells, r_cells + 1):
                    if dx * dx + dy * dy <= max_sq:
                        blocked.add((ix + dx, iy + dy))
        self._blocked = blocked
        self._blocked_dirty = False

    def blocked_cells(self) -> set[tuple[int, int]]:
        if self._blocked_dirty:
            self._rebuild_blocked()
        return self._blocked


# 8-connected grid neighbourhood: every cell in the 3×3 block around the
# current cell except the cell itself. Diagonals are included (and carry a
# √2 step cost) so that A* can produce near-Euclidean paths through
# doorways and along angled walls — a 4-connected search would force
# staircase paths that don't fit through ~1-cell-wide doorways.
_NEIGHBOURS: tuple[tuple[int, int, float], ...] = tuple(
    (dx, dy, math.hypot(dx, dy)) for dx in (-1, 0, 1) for dy in (-1, 0, 1) if (dx, dy) != (0, 0)
)


@dataclass
class StuckState:
    ref_goal_dist: float
    last_progress_time: float
    effective_inflation: float


def progress_tick(
    state: StuckState,
    goal_dist: float,
    mono_now: float,
    progress_epsilon: float,
    stuck_seconds: float,
    stuck_shrink_factor: float,
    stuck_min_inflation: float,
) -> tuple[StuckState, bool]:
    if goal_dist < state.ref_goal_dist - progress_epsilon:
        return (
            StuckState(
                ref_goal_dist=goal_dist,
                last_progress_time=mono_now,
                effective_inflation=state.effective_inflation,
            ),
            False,
        )
    if (
        mono_now - state.last_progress_time >= stuck_seconds
        and state.effective_inflation > stuck_min_inflation
    ):
        prev = state.effective_inflation
        new_inflation = max(stuck_min_inflation, prev * stuck_shrink_factor)
        if new_inflation < prev:
            return (
                StuckState(
                    ref_goal_dist=goal_dist,
                    last_progress_time=mono_now,
                    effective_inflation=new_inflation,
                ),
                True,
            )
    return (state, False)


def resolve_tf_chain(tf_buffer: Any, queries: list[tuple[str, str]]) -> Any:
    for parent, child in queries:
        tf = tf_buffer.get(parent, child)
        if tf is not None:
            return tf
    return None


def plan_on_costmap(
    costmap: Costmap,
    rx: float,
    ry: float,
    gx: float,
    gy: float,
    max_expansions: int,
    inflation_override: float | None = None,
) -> list[tuple[float, float]] | None:
    cm = costmap
    if inflation_override is not None and inflation_override != cm.inflation_radius:
        blocked = _blocked_at_inflation(cm, inflation_override)
    else:
        blocked = cm.blocked_cells()

    start = cm.world_to_cell(rx, ry)
    goal = cm.world_to_cell(gx, gy)

    # Ignore start/goal cell obstructions so we can plan even if the
    # robot or the goal clip an inflated cell.
    def is_blocked(ix: int, iy: int) -> bool:
        if (ix, iy) == start or (ix, iy) == goal:
            return False
        return (ix, iy) in blocked

    path_cells = astar(start, goal, is_blocked, max_expansions=max_expansions)
    if path_cells is None:
        return None
    return [cm.cell_to_world(ix, iy) for (ix, iy) in path_cells]


def _blocked_at_inflation(cm: Costmap, inflation_radius: float) -> set[tuple[int, int]]:
    if inflation_radius < 0.0:
        raise ValueError(f"inflation_radius must be non-negative, got {inflation_radius}")
    cell = cm.cell_size
    threshold = cm.obstacle_height
    r_cells = math.ceil(inflation_radius / cell)
    max_sq = (inflation_radius / cell) ** 2 if r_cells else 0.0
    blocked: set[tuple[int, int]] = set()
    for (ix, iy), h in list(cm._heights.items()):
        if h < threshold:
            continue
        if r_cells == 0:
            blocked.add((ix, iy))
            continue
        for dx in range(-r_cells, r_cells + 1):
            for dy in range(-r_cells, r_cells + 1):
                if dx * dx + dy * dy <= max_sq:
                    blocked.add((ix + dx, iy + dy))
    return blocked


def astar(
    start: tuple[int, int],
    goal: tuple[int, int],
    is_blocked: Callable[[int, int], bool],
    max_expansions: int = 200_000,
) -> list[tuple[int, int]] | None:
    if start == goal:
        return [start]

    def heuristic(c: tuple[int, int]) -> float:
        dx = abs(c[0] - goal[0])
        dy = abs(c[1] - goal[1])
        # Octile distance
        return (dx + dy) + (math.sqrt(2.0) - 2.0) * min(dx, dy)

    # If start or goal is blocked, try to step off — policy: we let the
    # caller handle that by pre-unblocking those cells.
    open_heap: list[tuple[float, int, tuple[int, int]]] = []
    counter = 0
    heapq.heappush(open_heap, (heuristic(start), counter, start))
    g_score: dict[tuple[int, int], float] = {start: 0.0}
    came_from: dict[tuple[int, int], tuple[int, int]] = {}

    expansions = 0
    while open_heap:
        expansions += 1
        if expansions > max_expansions:
            return None
        _, _, current = heapq.heappop(open_heap)
        if current == goal:
            # Reconstruct
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        cur_g = g_score[current]
        cx, cy = current
        for dx, dy, step in _NEIGHBOURS:
            nb = (cx + dx, cy + dy)
            if is_blocked(nb[0], nb[1]):
                continue
            tentative = cur_g + step
            if tentative < g_score.get(nb, float("inf")):
                came_from[nb] = current
                g_score[nb] = tentative
                counter += 1
                f = tentative + heuristic(nb)
                heapq.heappush(open_heap, (f, counter, nb))

    return None


class SimplePlannerConfig(ModuleConfig):
    # Costmap resolution in metres per cell.
    cell_size: float = 0.3
    # Points above this elevation (height above ground from terrain_map
    # intensity) mark a cell as an obstacle.
    obstacle_height_threshold: float = 0.15
    # Circular inflation radius around each obstacle (metres). Generous
    # by default: for a ~0.5 m diameter robot this keeps the A* path ~0.4 m
    # off every wall. Stuck-detection (below) shrinks this when a
    # doorway would otherwise be unpassable.
    inflation_radius: float = 0.2
    # Look-ahead distance along the planned path to emit as the next
    # waypoint for the local planner.
    lookahead_distance: float = 2.0
    # Replan + publish rate (Hz) — how often the planning loop wakes up.
    replan_rate: float = 5.0
    # Minimum seconds between successive A* searches. Waypoints are
    # still republished at replan_rate using the cached path, but A*
    # only re-runs after this cooldown. This prevents path flicker
    # between near-equivalent A* solutions.
    replan_cooldown: float = 2.0
    # Hard cap on A* node expansions per call.
    max_expansions: int = 200_000
    # Height offset below the robot z-position to estimate ground level (m).
    # Points below this level are ignored; points above become obstacle
    # candidates. Should match or slightly exceed the robot's standing height.
    ground_offset_below_robot: float = 1.3

    # Consider the robot "stuck" if its distance-to-goal hasn't decreased
    # by at least ``progress_epsilon`` metres within ``stuck_seconds``.
    stuck_seconds: float = 5.0
    # Minimum improvement in goal-distance that counts as progress.
    progress_epsilon: float = 0.25
    # When stuck, progressively shrink the inflation_radius by this
    # fraction each escalation step (e.g. 0.5 → half, then quarter, …).
    # Shrinking too aggressively risks clipping obstacles, so we bottom
    # out at ``stuck_min_inflation``.
    stuck_shrink_factor: float = 0.5
    stuck_min_inflation: float = 0.2
    # When the robot is within this distance (m) of the current
    # intermediate waypoint, proactively advance the waypoint along the
    # cached path so the local planner never stops on it. Should be
    # larger than LocalPlanner's goal_reached_threshold.
    waypoint_advance_radius: float = 1.0


class SimplePlanner(Module):
    """Grid-A* global route planner"""

    config: SimplePlannerConfig

    terrain_map_ext: In[PointCloud2]
    terrain_map: In[PointCloud2]
    goal: In[PointStamped]
    way_point: Out[PointStamped]
    goal_path: Out[Path]
    costmap_cloud: Out[PointCloud2]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._running = False
        self._thread: threading.Thread | None = None
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_z = 0.0
        self._has_odom = False
        self._goal_x: float | None = None
        self._goal_y: float | None = None
        self._goal_z = 0.0
        self._last_diag_print = 0.0
        # Progress tracker. ``_ref_goal_dist`` is the distance-to-goal we
        # last clocked as progress; any subsequent drop of at least
        # ``progress_epsilon`` counts as "still making headway" and
        # refreshes ``_last_progress_time``.
        self._ref_goal_dist = float("inf")
        self._last_progress_time = 0.0
        # Current inflation in use — shrunk on stuck escalation, reset
        # to config.inflation_radius on new goal.
        self._effective_inflation = self.config.inflation_radius
        # Cached last-successful A* path and when we planned it, so
        # waypoints can still be republished between replans (cooldown
        # is enforced in the planning loop).
        self._cached_path: list[tuple[float, float]] | None = None
        self._last_plan_time = 0.0
        # Costmap_cloud publish throttle — 2 Hz is plenty for rerun.
        self._last_costmap_pub = 0.0
        # Currently published waypoint — tracked so the odom callback can
        # detect when the robot is about to reach it and advance early.
        self._current_wp: tuple[float, float] | None = None
        self._current_wp_is_goal = False

    @rpc
    def start(self) -> None:
        super().start()
        self._lock = threading.Lock()
        self._costmap = Costmap(
            cell_size=self.config.cell_size,
            obstacle_height=self.config.obstacle_height_threshold,
            inflation_radius=self.config.inflation_radius,
        )
        self.register_disposable(Disposable(self.goal.subscribe(self._on_goal)))
        self.register_disposable(
            Disposable(self.terrain_map_ext.subscribe(self._on_terrain_map_ext))
        )
        self.register_disposable(Disposable(self.terrain_map.subscribe(self._on_terrain_map)))
        self._running = True
        self._thread = threading.Thread(target=self._planning_loop, daemon=True)
        self._thread.start()
        logger.info("SimplePlanner started")

    @rpc
    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None
        super().stop()

    # Ordered list of (parent, child) TF lookups to try for the robot pose.
    # The first successful lookup wins.  ``body`` is the standard REP-105
    # child frame; ``sensor`` is used by the Unity sim bridge.
    _TF_POSE_QUERIES: list[tuple[str, str]] = [
        (FRAME_MAP, FRAME_BODY),
        (FRAME_ODOM, FRAME_BODY),
        (FRAME_MAP, "sensor"),
    ]

    def _query_pose(self) -> bool:
        """Update cached robot position from the TF tree.

        Tries several ``(parent, child)`` pairs in priority order so the
        planner works both on real hardware (``map → body`` via PGO +
        FastLio2) and in simulation (``map → sensor`` from the Unity
        bridge).

        Returns True if a pose was obtained from any chain.
        """
        tf = resolve_tf_chain(self.tf, list(self._TF_POSE_QUERIES))
        if tf is None:
            now = time.monotonic()
            if now - getattr(self, "_last_tf_warn", 0.0) > 5.0:
                self._last_tf_warn = now
                buffers = list(self.tf.buffers.keys()) if hasattr(self.tf, "buffers") else []
                logger.warning(
                    "TF lookup failed — no robot pose available",
                    tried=[(p, c) for p, c in self._TF_POSE_QUERIES],
                    available_frames=buffers,
                )
            return False
        with self._lock:
            self._robot_x = float(tf.translation.x)
            self._robot_y = float(tf.translation.y)
            self._robot_z = float(tf.translation.z)
            self._has_odom = True
        return True

    def _on_goal(self, msg: PointStamped) -> None:
        # NaN sentinel = cancel navigation (e.g. teleop took over).
        if not all(math.isfinite(v) for v in (msg.x, msg.y, msg.z)):
            with self._lock:
                self._goal_x = None
                self._goal_y = None
                self._cached_path = None
                rx, ry, rz = self._robot_x, self._robot_y, self._robot_z
            # Publish robot position as waypoint so LocalPlanner stops
            # tracking the stale waypoint.
            self._current_wp = None
            self._current_wp_is_goal = False
            now = time.time()
            self.way_point.publish(PointStamped(ts=now, frame_id=FRAME_MAP, x=rx, y=ry, z=rz))
            self.goal_path.publish(Path(ts=now, frame_id=FRAME_MAP, poses=[]))
            logger.info("Goal cleared — idle until new goal")
            return
        with self._lock:
            self._goal_x = float(msg.x)
            self._goal_y = float(msg.y)
            self._goal_z = float(msg.z)
            # Fresh goal → fresh progress tracker + restore default
            # inflation + drop cached path so the next tick plans
            # immediately (no cooldown wait for a brand-new goal).
            self._ref_goal_dist = float("inf")
            self._last_progress_time = time.monotonic()
            self._effective_inflation = self.config.inflation_radius
            self._cached_path = None
            self._last_plan_time = 0.0
        logger.info("Goal received", x=round(msg.x, 2), y=round(msg.y, 2), z=round(msg.z, 2))

    # Sensor height assumed for the G1 (m). Points below robot_z minus
    # this offset are interpreted as floor; anything higher is obstacle.

    def _classify_points(self, points: np.ndarray, cm: Costmap) -> None:
        """Add points (Nx3) to ``cm`` using z-relative-to-ground as height.

        The dimos PointCloud2 wrapper drops the intensity field, so we
        can't read elevation-above-ground directly. Instead we classify
        by the point's absolute z relative to the robot's standing
        ground (rz - ``_GROUND_OFFSET_BELOW_ROBOT``). TerrainAnalysis
        only publishes ground/low-height obstacle voxels, so
        z-relative-to-ground is a good elevation proxy.
        """
        if len(points) == 0:
            return
        with self._lock:
            rz = self._robot_z if self._has_odom else 0.0
        ground_z = rz - self.config.ground_offset_below_robot
        heights = points[:, 2] - ground_z
        mask = heights > 0.0
        if not np.any(mask):
            return
        xs = points[mask, 0]
        ys = points[mask, 1]
        hs = heights[mask]
        cell_size = cm.cell_size
        ixs = np.floor(xs / cell_size).astype(np.int64)
        iys = np.floor(ys / cell_size).astype(np.int64)
        # Group by cell and take max height per cell (vectorized).
        keys = np.column_stack((ixs, iys))
        _, inverse, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
        max_h = np.full(len(counts), float("-inf"))
        np.maximum.at(max_h, inverse, hs)
        unique_keys = keys[np.unique(inverse, return_index=True)[1]]
        dirty = False
        for i in range(len(unique_keys)):
            key = (int(unique_keys[i, 0]), int(unique_keys[i, 1]))
            h = float(max_h[i])
            prev = cm._heights.get(key, float("-inf"))
            if h > prev:
                cm._heights[key] = h
                dirty = True
        if dirty:
            cm._blocked_dirty = True

    def _fresh_costmap(self) -> Costmap:
        return Costmap(
            cell_size=self.config.cell_size,
            obstacle_height=self.config.obstacle_height_threshold,
            inflation_radius=self.config.inflation_radius,
        )

    def _on_terrain_map_ext(self, msg: PointCloud2) -> None:
        """Rebuild the costmap from scratch using the persistent world view.

        ``terrain_map_ext`` applies a decay window (8 s by default) on
        the producer side, so each message represents the current world
        state. Resetting here prevents stale obstacles from piling up
        forever.
        """
        points, _ = msg.as_numpy()
        if points is None or len(points) == 0:
            return
        new_cm = self._fresh_costmap()
        self._classify_points(points, new_cm)
        # Hot-swap in one assignment so the planning loop sees either
        # the old or the new map but never a partial one.
        # Note: a concurrent _on_terrain_map may still be writing into the
        # old costmap when we swap; those writes are silently lost. This is
        # acceptable — the next terrain_map_ext rebuild will pick them up.
        self._costmap = new_cm

    def _on_terrain_map(self, msg: PointCloud2) -> None:
        """Layer fresh local terrain on top of the current costmap.

        ``terrain_map`` arrives faster than ``terrain_map_ext`` and
        carries the most recent local view, so dynamic obstacles appear
        here first. We additively merge into the existing costmap;
        these additions are wiped on the next ``terrain_map_ext``
        rebuild.
        """
        points, _ = msg.as_numpy()
        if points is None or len(points) == 0:
            return
        self._classify_points(points, self._costmap)

    def _planning_loop(self) -> None:
        rate = self.config.replan_rate
        period = 1.0 / rate if rate > 0 else 0.2
        while self._running:
            t0 = time.monotonic()
            try:
                self._replan_once()
            except Exception as exc:  # don't let the planning thread die
                logger.error("Replan error", exc_info=exc)
            dt = time.monotonic() - t0
            sleep = period - dt
            if sleep > 0:
                time.sleep(sleep)

    def _publish_costmap_cloud(self, rz: float, now: float) -> None:
        """Publish the blocked-cell centers as a PointCloud2 for rerun.

        Throttled to ~2 Hz. Each cell becomes a 3D point at the cell
        center, lifted slightly above the robot's z for visibility.
        """
        if now - self._last_costmap_pub < 0.5:
            return
        self._last_costmap_pub = now
        cm = self._costmap
        blocked = cm.blocked_cells()
        if not blocked:
            pts = np.zeros((0, 3), dtype=np.float32)
        else:
            pts = np.empty((len(blocked), 3), dtype=np.float32)
            for i, (ix, iy) in enumerate(blocked):
                wx, wy = cm.cell_to_world(ix, iy)
                pts[i, 0] = wx
                pts[i, 1] = wy
                pts[i, 2] = rz - self.config.ground_offset_below_robot + 0.1
        self.costmap_cloud.publish(PointCloud2.from_numpy(pts, frame_id=FRAME_MAP, timestamp=now))

    def _publish_from_cached(self, rx: float, ry: float, gz: float, now: float) -> None:
        """Republish a look-ahead waypoint from the cached path.

        Called while the replan cooldown is in effect — we don't touch
        the goal_path (it's already current in the viewer) but we do
        keep feeding LocalPlanner fresh waypoints so it doesn't treat
        the robot as idle.
        """
        with self._lock:
            cached = self._cached_path
        if not cached:
            return
        wx, wy = self._lookahead(cached, rx, ry, self.config.lookahead_distance)
        last = cached[-1]
        is_goal = (wx, wy) == last
        with self._lock:
            self._current_wp = (wx, wy)
            self._current_wp_is_goal = is_goal
        self.way_point.publish(PointStamped(ts=now, frame_id=FRAME_MAP, x=wx, y=wy, z=gz))

    def _maybe_advance_waypoint(self, rx: float, ry: float, gz: float) -> None:
        """If the robot is close to the current intermediate waypoint, advance it."""
        with self._lock:
            wp = self._current_wp
            is_goal = self._current_wp_is_goal
            cached = self._cached_path
        if wp is None or is_goal or cached is None:
            return
        dist_sq = (wp[0] - rx) ** 2 + (wp[1] - ry) ** 2
        threshold_sq = self.config.waypoint_advance_radius**2
        if dist_sq > threshold_sq:
            return
        extended_lookahead = self.config.lookahead_distance * 1.5
        wx, wy = self._lookahead(cached, rx, ry, extended_lookahead)
        last = cached[-1]
        new_is_goal = (wx, wy) == last
        with self._lock:
            self._current_wp = (wx, wy)
            self._current_wp_is_goal = new_is_goal
        now = time.time()
        self.way_point.publish(PointStamped(ts=now, frame_id=FRAME_MAP, x=wx, y=wy, z=gz))

    def _replan_once(self) -> None:
        # Refresh pose from the TF tree every tick.
        self._query_pose()

        with self._lock:
            if not self._has_odom or self._goal_x is None or self._goal_y is None:
                return
            rx, ry, rz = self._robot_x, self._robot_y, self._robot_z
            gx, gy, gz = self._goal_x, self._goal_y, self._goal_z

        mono_now = time.monotonic()
        goal_dist = math.hypot(gx - rx, gy - ry)
        now = time.time()

        # Keep the next waypoint ahead of the robot so the local planner
        # never stops on an intermediate waypoint.
        self._maybe_advance_waypoint(rx, ry, gz)

        # If it's too soon for a fresh A*, just refresh the waypoint from
        # the cached path using the current pose.
        with self._lock:
            cooldown_active = (
                self._cached_path is not None
                and mono_now - self._last_plan_time < self.config.replan_cooldown
            )
        # Publish the debug costmap every tick (throttled internally).
        self._publish_costmap_cloud(rz, now)

        if cooldown_active:
            self._publish_from_cached(rx, ry, gz, now)
            return

        # Don't bump inflation back up on progress: if we shrank it to clear
        # a tight spot, keep it shrunk until the next goal. Oscillating
        # between wide/narrow inflation was wasting time per cycle on the
        # way through a single doorway.
        with self._lock:
            prev_state = StuckState(
                ref_goal_dist=self._ref_goal_dist,
                last_progress_time=self._last_progress_time,
                effective_inflation=self._effective_inflation,
            )
            new_state, escalated = progress_tick(
                prev_state,
                goal_dist,
                mono_now,
                progress_epsilon=self.config.progress_epsilon,
                stuck_seconds=self.config.stuck_seconds,
                stuck_shrink_factor=self.config.stuck_shrink_factor,
                stuck_min_inflation=self.config.stuck_min_inflation,
            )
            self._ref_goal_dist = new_state.ref_goal_dist
            self._last_progress_time = new_state.last_progress_time
            self._effective_inflation = new_state.effective_inflation
            effective_inflation = new_state.effective_inflation
        if escalated:
            logger.warning(
                "Stuck — shrinking inflation",
                stuck_seconds=self.config.stuck_seconds,
                goal_dist=round(goal_dist, 2),
                ref_dist=round(new_state.ref_goal_dist, 2),
                inflation_from=round(prev_state.effective_inflation, 2),
                inflation_to=round(new_state.effective_inflation, 2),
            )

        path_world = self.plan(rx, ry, gx, gy, inflation_override=effective_inflation)
        with self._lock:
            self._last_plan_time = mono_now  # start cooldown now, success or not
        if path_world is None:
            # A* failed (goal unreachable through the current costmap).
            # Don't drive the robot into a wall: publish the robot's
            # current position so the local planner stops, and wait
            # for the costmap to refresh before the next attempt.
            logger.warning(
                "A* failed; holding position",
                robot=f"({rx:.2f},{ry:.2f})",
                goal=f"({gx:.2f},{gy:.2f})",
            )
            with self._lock:
                self._current_wp = None
                self._current_wp_is_goal = False
            self.way_point.publish(PointStamped(ts=now, frame_id=FRAME_MAP, x=rx, y=ry, z=rz))
            self.goal_path.publish(
                Path(
                    ts=now,
                    frame_id=FRAME_MAP,
                    poses=[
                        PoseStamped(
                            ts=now,
                            frame_id=FRAME_MAP,
                            position=[rx, ry, rz],
                            orientation=[0.0, 0.0, 0.0, 1.0],
                        ),
                        PoseStamped(
                            ts=now,
                            frame_id=FRAME_MAP,
                            position=[gx, gy, gz],
                            orientation=[0.0, 0.0, 0.0, 1.0],
                        ),
                    ],
                )
            )
            return

        # Cache the fresh path for use during the cooldown.
        with self._lock:
            self._cached_path = path_world

        # Publish goal_path
        poses: list[PoseStamped] = []
        for wx, wy in path_world:
            poses.append(
                PoseStamped(
                    ts=now,
                    frame_id=FRAME_MAP,
                    position=[wx, wy, rz],
                    orientation=[0.0, 0.0, 0.0, 1.0],
                )
            )
        self.goal_path.publish(Path(ts=now, frame_id=FRAME_MAP, poses=poses))

        # Pick look-ahead waypoint
        wx, wy = self._lookahead(path_world, rx, ry, self.config.lookahead_distance)
        last = path_world[-1]
        is_goal = (wx, wy) == last
        with self._lock:
            self._current_wp = (wx, wy)
            self._current_wp_is_goal = is_goal
        self.way_point.publish(PointStamped(ts=now, frame_id=FRAME_MAP, x=wx, y=wy, z=gz))

        # 1 Hz diagnostic: cells in costmap, path length, chosen waypoint
        if now - self._last_diag_print >= 1.0:
            self._last_diag_print = now
            blocked = len(self._costmap.blocked_cells())
            logger.info(
                "Replan",
                path_cells=len(path_world),
                blocked_cells=blocked,
                robot=f"({rx:.2f},{ry:.2f})",
                goal=f"({gx:.2f},{gy:.2f})",
                waypoint=f"({wx:.2f},{wy:.2f})",
                inflation=round(effective_inflation, 2),
            )

    def plan(
        self,
        rx: float,
        ry: float,
        gx: float,
        gy: float,
        inflation_override: float | None = None,
    ) -> list[tuple[float, float]] | None:
        """Run A* in world coordinates. Returns [(x, y), ...] or None."""
        return plan_on_costmap(
            self._costmap,
            rx,
            ry,
            gx,
            gy,
            self.config.max_expansions,
            inflation_override=inflation_override,
        )

    @staticmethod
    def _lookahead(
        path: list[tuple[float, float]], rx: float, ry: float, distance: float
    ) -> tuple[float, float]:
        """Pick a look-ahead point at least ``distance`` metres ahead of the
        robot along the path.

        First finds the path index closest to (rx, ry), then walks forward
        until the cumulative distance from that closest point exceeds
        ``distance``. Falls back to the final path node if nothing is far
        enough. Path is ordered start → goal.
        """
        if not path:
            return (rx, ry)
        # Closest path index to the robot
        best_idx = 0
        best_d2 = float("inf")
        for i, (wx, wy) in enumerate(path):
            d2 = (wx - rx) ** 2 + (wy - ry) ** 2
            if d2 < best_d2:
                best_d2 = d2
                best_idx = i
        # Walk forward from there until we've covered `distance`
        d2_target = distance * distance
        for i in range(best_idx, len(path)):
            wx, wy = path[i]
            if (wx - rx) ** 2 + (wy - ry) ** 2 >= d2_target:
                return (wx, wy)
        return path[-1]
