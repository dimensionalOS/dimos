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

"""SmoothLocalPlanner: reactive arc-sampling local planner with EMA curvature.

Python replacement for the native C++ DWA-ish ``LocalPlanner``. The native
planner jitters (sign-flipping yaw rate, snapping paths) because it picks
from 252 discrete bins with branchy scoring. This module instead samples
a small set of constant-curvature arcs, scores them with a single smooth
score (clearance + goal alignment + Gaussian hysteresis on curvature),
and publishes the arc at an EMA-smoothed curvature at a fixed rate.

Drop-in compatible with the native ``LocalPlanner`` port surface minus
``joy_cmd`` and ``registered_scan``, so autoconnect swaps it in without
extra remappings.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any

import numpy as np

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

# ──────────────────────────────────────────────────────────────────────────
# Pure functions (unit-testable)
# ──────────────────────────────────────────────────────────────────────────


def generate_arc(kappa: float, length: float, step: float) -> np.ndarray:
    """Generate an Nx2 array of (x, y) samples along a constant-curvature arc.

    Assumes unit forward speed, starts at the origin heading +x. ``kappa``
    is signed curvature (1/m); ``kappa=0`` produces a straight line along
    +x. ``length`` is arc length in metres, ``step`` is arc-length spacing
    between samples.
    """
    if step <= 0.0:
        raise ValueError(f"step must be positive, got {step}")
    if length <= 0.0:
        return np.zeros((1, 2), dtype=np.float64)
    n = math.floor(length / step) + 1
    s = np.linspace(0.0, length, n)
    if abs(kappa) < 1e-6:
        xs = s
        ys = np.zeros_like(s)
    else:
        r = 1.0 / kappa
        theta = s * kappa
        xs = r * np.sin(theta)
        ys = r * (1.0 - np.cos(theta))
    return np.stack([xs, ys], axis=1)


def crop_obstacles(
    points_vf: np.ndarray,
    obstacle_range: float,
    min_relative_z: float,
    max_relative_z: float,
) -> np.ndarray:
    """Crop vehicle-frame points to the (disk x height-band) region.

    Input is Nx2 or Nx3 in vehicle frame. Output is Nx2 with NaN/inf
    rows, points outside the disk of radius ``obstacle_range``, and
    points outside ``[min_relative_z, max_relative_z]`` removed. Empty
    input returns a (0, 2) array.
    """
    if points_vf is None or len(points_vf) == 0:
        return np.zeros((0, 2), dtype=np.float64)
    pts = np.asarray(points_vf, dtype=np.float64)
    finite = np.all(np.isfinite(pts), axis=1)
    pts = pts[finite]
    if pts.shape[1] >= 3:
        z = pts[:, 2]
        band = (z >= min_relative_z) & (z <= max_relative_z)
        pts = pts[band]
    if len(pts) == 0:
        return np.zeros((0, 2), dtype=np.float64)
    xy = pts[:, :2]
    r2 = xy[:, 0] ** 2 + xy[:, 1] ** 2
    in_disk = r2 <= obstacle_range * obstacle_range
    return xy[in_disk]


def score_candidate(
    arc: np.ndarray,
    obstacles: np.ndarray,
    goal_bearing: float,
    prev_kappa: float,
    kappa: float,
    *,
    robot_radius: float,
    clearance_cap: float,
    clearance_weight: float,
    alignment_weight: float,
    hysteresis_weight: float,
    hysteresis_sigma: float,
) -> tuple[float, bool, float]:
    """Score one candidate arc. Returns (score, blocked, clearance)."""
    if arc.shape[0] == 0:
        return (0.0, True, 0.0)
    if obstacles.shape[0] == 0:
        clearance = clearance_cap
        blocked = False
    else:
        # min over arc samples of min over obstacles of euclidean dist
        dx = arc[:, 0:1] - obstacles[np.newaxis, :, 0]
        dy = arc[:, 1:2] - obstacles[np.newaxis, :, 1]
        d2 = dx * dx + dy * dy
        per_sample_min = np.min(d2, axis=1)
        clearance = float(math.sqrt(per_sample_min.min()))
        blocked = clearance < robot_radius
    clearance_term = min(clearance, clearance_cap)
    heading = math.atan2(float(arc[-1, 1]), float(arc[-1, 0]))
    # cos difference is smooth, no branches
    align = math.cos(heading - goal_bearing)
    dk = (kappa - prev_kappa) / hysteresis_sigma
    hyst = math.exp(-dk * dk)
    score = (
        clearance_weight * (clearance_term / clearance_cap)
        + alignment_weight * 0.5 * (align + 1.0)
        + hysteresis_weight * hyst
    )
    return (score, blocked, clearance)


def select_curvature(
    obstacles: np.ndarray,
    goal_bearing: float,
    prev_kappa: float,
    kappa_candidates: np.ndarray,
    arc_cache: list[np.ndarray],
    *,
    robot_radius: float,
    clearance_cap: float,
    clearance_weight: float,
    alignment_weight: float,
    hysteresis_weight: float,
    hysteresis_sigma: float,
) -> tuple[float, bool, int]:
    """Pick the best-scoring candidate curvature.

    Returns ``(raw_kappa, all_blocked, best_idx)``. If every candidate
    collides, ``raw_kappa`` is ``prev_kappa`` (caller will shrink the
    published arc).
    """
    best_score = -float("inf")
    best_idx = -1
    best_unblocked_score = -float("inf")
    best_unblocked_idx = -1
    for i, kappa in enumerate(kappa_candidates):
        score, blocked, _clr = score_candidate(
            arc_cache[i],
            obstacles,
            goal_bearing,
            prev_kappa,
            float(kappa),
            robot_radius=robot_radius,
            clearance_cap=clearance_cap,
            clearance_weight=clearance_weight,
            alignment_weight=alignment_weight,
            hysteresis_weight=hysteresis_weight,
            hysteresis_sigma=hysteresis_sigma,
        )
        if score > best_score:
            best_score = score
            best_idx = i
        if not blocked and score > best_unblocked_score:
            best_unblocked_score = score
            best_unblocked_idx = i
    if best_unblocked_idx >= 0:
        return (float(kappa_candidates[best_unblocked_idx]), False, best_unblocked_idx)
    return (prev_kappa, True, best_idx)


def update_ema(prev: float, raw: float, alpha: float) -> float:
    """Exponential moving average update."""
    return alpha * raw + (1.0 - alpha) * prev


def goal_in_vehicle_frame(
    gx_w: float, gy_w: float, rx: float, ry: float, ryaw: float
) -> tuple[float, float]:
    """Transform a world-frame goal into the vehicle frame."""
    dx = gx_w - rx
    dy = gy_w - ry
    c = math.cos(ryaw)
    s = math.sin(ryaw)
    # Rotate by -ryaw
    gx_v = c * dx + s * dy
    gy_v = -s * dx + c * dy
    return (gx_v, gy_v)


# ──────────────────────────────────────────────────────────────────────────
# Config + Module
# ──────────────────────────────────────────────────────────────────────────


class SmoothLocalPlannerConfig(ModuleConfig):
    """Config for SmoothLocalPlanner."""

    # Arc sampling
    num_candidates: int = 21
    max_curvature: float = 1.5  # 1/m
    arc_length: float = 3.0  # m simulated forward
    arc_step: float = 0.15  # m between arc samples

    # Scoring weights (scores are pre-normalized to ~[0,1])
    clearance_weight: float = 2.0
    alignment_weight: float = 1.0
    hysteresis_weight: float = 0.4
    hysteresis_sigma: float = 0.4
    clearance_cap: float = 1.5

    # Safety
    robot_radius: float = 0.50
    obstacle_range: float = 4.0
    obstacle_height_threshold: float = 0.15
    max_relative_z: float = 1.5
    min_relative_z: float = -1.5
    # World-frame ground plane offset below the robot origin (m). Matches
    # SimplePlanner's ``_GROUND_OFFSET_BELOW_ROBOT``: the floor sits this
    # far below the robot odometry z. Any terrain point more than
    # ``obstacle_height_threshold`` above the floor is treated as an
    # obstacle; everything below is the floor and ignored.
    ground_offset_below_robot: float = 1.3

    # Temporal filter
    curvature_ema_alpha: float = 0.3
    blocked_speed_scale: float = 0.3

    # Goal proximity
    goal_stop_radius: float = 0.5  # m; publish a stop-pose when this close.
    # When the goal bearing exceeds this angle, the goal is "behind" and
    # we publish a short turn-in-place nudge instead of a forward arc.
    turn_in_place_bearing: float = 1.3  # rad (~75°)
    turn_in_place_length: float = 0.3  # m; tiny nudge arc for pivoting

    # Churn-based speed scaling: track |Δκ| with EMA; when it exceeds
    # ``churn_threshold``, scale down the published arc length toward
    # ``churn_min_scale`` so path_follower naturally slows.
    churn_alpha: float = 0.3  # EMA smoothing for |Δκ|
    churn_threshold: float = 0.15  # κ/tick; above this = high churn
    churn_min_scale: float = 0.3  # floor scale at extreme churn

    # Publishing
    publish_rate: float = 20.0
    publish_length: float = 3.0

    # Housekeeping
    frame_id: str = "vehicle"


class SmoothLocalPlanner(Module[SmoothLocalPlannerConfig]):
    """Reactive arc-sampling local planner with EMA-smoothed curvature.

    Ports:
        terrain_map (In[PointCloud2]): World-frame terrain cloud from
            TerrainAnalysis (short decay, ~2 s). Carries fresh/dynamic
            obstacles within the sensor FOV.
        terrain_map_ext (In[PointCloud2]): World-frame extended terrain
            cloud from TerrainMapExt (~300 s decay, static walls retained
            behind the robot). Provides the persistent obstacle context
            that a pure-reactive planner needs to avoid turning into
            walls that are briefly out-of-FOV.
        odometry (In[Odometry]): Robot pose in world frame. Used to
            transform the goal and obstacle points into vehicle frame.
        way_point (In[PointStamped]): Goal in world frame (from
            FarPlanner / SimplePlanner / ClickToGoal).
        path (Out[Path]): Chosen arc in vehicle frame.
        obstacle_cloud (Out[PointCloud2]): Debug — the vehicle-frame
            obstacle crop we scored against (throttled).
    """

    default_config: type[SmoothLocalPlannerConfig] = SmoothLocalPlannerConfig

    terrain_map: In[PointCloud2]
    terrain_map_ext: In[PointCloud2]
    odometry: In[Odometry]
    way_point: In[PointStamped]
    path: Out[Path]
    obstacle_cloud: Out[PointCloud2]

    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._running = False
        self._thread: threading.Thread | None = None

        self._build_arc_cache()

        self._prev_kappa_ema = 0.0
        # Churn tracker: EMA of |Δκ| between consecutive ticks.
        # High churn → shorten the published arc so path_follower slows.
        self._kappa_delta_ema = 0.0

        # State (world frame unless noted)
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_z = 0.0
        self._robot_yaw = 0.0
        self._has_odom = False
        self._goal_x: float | None = None
        self._goal_y: float | None = None

        # Latest world-frame terrain clouds (Nx3 numpy, may be None).
        self._terrain_pts_world: np.ndarray | None = None
        self._terrain_ext_pts_world: np.ndarray | None = None

        self._last_diag_print = 0.0
        self._last_obstacle_pub = 0.0

    def _build_arc_cache(self) -> None:
        cfg = self.config
        self._kappa_candidates = np.linspace(
            -cfg.max_curvature, cfg.max_curvature, cfg.num_candidates
        )
        self._arc_cache: list[np.ndarray] = [
            generate_arc(float(k), cfg.arc_length, cfg.arc_step) for k in self._kappa_candidates
        ]

    def __getstate__(self) -> dict[str, Any]:
        state = super().__getstate__()
        for k in ("_lock", "_thread", "_arc_cache", "_kappa_candidates"):
            state.pop(k, None)
        return state

    def __setstate__(self, state: dict[str, Any]) -> None:
        super().__setstate__(state)
        self._lock = threading.Lock()
        self._thread = None
        self._build_arc_cache()

    @rpc
    def start(self) -> None:
        self.odometry.subscribe(self._on_odom)
        self.way_point.subscribe(self._on_waypoint)
        self.terrain_map.subscribe(self._on_terrain_map)
        self.terrain_map_ext.subscribe(self._on_terrain_map_ext)
        self._running = True
        self._thread = threading.Thread(target=self._publish_loop, daemon=True)
        self._thread.start()
        print("[smooth_local_planner] Started.")

    @rpc
    def stop(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None
        super().stop()

    # ── Subscription callbacks ────────────────────────────────────────────

    def _on_odom(self, msg: Odometry) -> None:
        with self._lock:
            self._robot_x = float(msg.x)
            self._robot_y = float(msg.y)
            self._robot_z = float(msg.z)
            self._robot_yaw = float(msg.yaw)
            self._has_odom = True

    def _on_waypoint(self, msg: PointStamped) -> None:
        if not all(math.isfinite(v) for v in (msg.x, msg.y)):
            return
        with self._lock:
            self._goal_x = float(msg.x)
            self._goal_y = float(msg.y)

    def _on_terrain_map(self, msg: PointCloud2) -> None:
        points, _ = msg.as_numpy()
        if points is None or len(points) == 0:
            with self._lock:
                self._terrain_pts_world = np.zeros((0, 3), dtype=np.float64)
            return
        with self._lock:
            self._terrain_pts_world = np.asarray(points[:, :3], dtype=np.float64)

    def _on_terrain_map_ext(self, msg: PointCloud2) -> None:
        points, _ = msg.as_numpy()
        if points is None or len(points) == 0:
            with self._lock:
                self._terrain_ext_pts_world = np.zeros((0, 3), dtype=np.float64)
            return
        with self._lock:
            self._terrain_ext_pts_world = np.asarray(points[:, :3], dtype=np.float64)

    # ── Publish loop ──────────────────────────────────────────────────────

    def _publish_loop(self) -> None:
        rate = self.config.publish_rate
        period = 1.0 / rate if rate > 0 else 0.05
        while self._running:
            t0 = time.monotonic()
            try:
                self._plan_once()
            except Exception as exc:
                print(f"[smooth_local_planner] plan error: {exc}")
            dt = time.monotonic() - t0
            sleep = period - dt
            if sleep > 0:
                time.sleep(sleep)

    def _plan_once(self) -> None:
        cfg = self.config
        with self._lock:
            has_odom = self._has_odom
            rx = self._robot_x
            ry = self._robot_y
            rz = self._robot_z
            ryaw = self._robot_yaw
            gx = self._goal_x
            gy = self._goal_y
            terrain_world = self._terrain_pts_world
            terrain_ext_world = self._terrain_ext_pts_world

        if not has_odom or gx is None or gy is None:
            return

        # Merge the two terrain clouds: ``terrain_map`` gives fresh
        # dynamic obstacles, ``terrain_map_ext`` gives static walls that
        # may be out of FOV. Concatenating is fine — clearance is a
        # min-distance so duplicate points don't change the result.
        clouds: list[np.ndarray] = []
        if terrain_world is not None and len(terrain_world) > 0:
            clouds.append(terrain_world)
        if terrain_ext_world is not None and len(terrain_ext_world) > 0:
            clouds.append(terrain_ext_world)
        if clouds:
            terrain_world = np.concatenate(clouds, axis=0)
        else:
            terrain_world = None

        # Transform terrain cloud world → vehicle frame AND drop floor
        # points. terrain_map is world-frame and contains both ground
        # and obstacle points; we classify a point as an obstacle if
        # its world-z is more than obstacle_height_threshold above the
        # floor, where the floor is inferred as
        # (robot_z - ground_offset_below_robot). Anything below that is
        # ignored. Then we express z relative to the robot so the
        # (min_relative_z, max_relative_z) band is meaningful.
        if terrain_world is not None and len(terrain_world) > 0:
            ground_z = rz - cfg.ground_offset_below_robot
            height_above_ground = terrain_world[:, 2] - ground_z
            is_obstacle = height_above_ground > cfg.obstacle_height_threshold
            obs_world = terrain_world[is_obstacle]
            if len(obs_world) > 0:
                dx = obs_world[:, 0] - rx
                dy = obs_world[:, 1] - ry
                c = math.cos(ryaw)
                s = math.sin(ryaw)
                xv = c * dx + s * dy
                yv = -s * dx + c * dy
                zv = obs_world[:, 2] - rz
                pts_vf = np.stack([xv, yv, zv], axis=1)
            else:
                pts_vf = np.zeros((0, 3), dtype=np.float64)
        else:
            pts_vf = np.zeros((0, 3), dtype=np.float64)

        obstacles = crop_obstacles(
            pts_vf,
            obstacle_range=cfg.obstacle_range,
            min_relative_z=cfg.min_relative_z,
            max_relative_z=cfg.max_relative_z,
        )

        gx_v, gy_v = goal_in_vehicle_frame(gx, gy, rx, ry, ryaw)
        goal_bearing = math.atan2(gy_v, gx_v)
        goal_dist = math.hypot(gx_v, gy_v)

        now = time.time()

        # ── Close-to-goal: stop. Publish a single pose at the robot's
        # current position so path_follower decelerates to zero.
        if goal_dist < cfg.goal_stop_radius:
            self._prev_kappa_ema = 0.0
            self._kappa_delta_ema = 0.0
            poses = [
                PoseStamped(
                    ts=now,
                    frame_id=cfg.frame_id,
                    position=[0.0, 0.0, 0.0],
                    orientation=[0.0, 0.0, 0.0, 1.0],
                )
            ]
            self.path.publish(Path(ts=now, frame_id=cfg.frame_id, poses=poses))
            self._maybe_publish_obstacle_cloud(obstacles, now)
            return

        # ── Goal behind: turn-in-place. Arc candidates are all
        # forward-facing and can't represent a 180° pivot. Publish a
        # short nudge arc toward the goal side so path_follower turns
        # the robot without driving forward into a wall.
        if abs(goal_bearing) > cfg.turn_in_place_bearing:
            turn_kappa = math.copysign(cfg.max_curvature, goal_bearing)
            self._prev_kappa_ema = update_ema(
                self._prev_kappa_ema, turn_kappa, cfg.curvature_ema_alpha
            )
            arc = generate_arc(turn_kappa, cfg.turn_in_place_length, cfg.arc_step)
            poses = [
                PoseStamped(
                    ts=now,
                    frame_id=cfg.frame_id,
                    position=[float(arc[i, 0]), float(arc[i, 1]), 0.0],
                    orientation=[0.0, 0.0, 0.0, 1.0],
                )
                for i in range(arc.shape[0])
            ]
            self.path.publish(Path(ts=now, frame_id=cfg.frame_id, poses=poses))
            self._maybe_publish_obstacle_cloud(obstacles, now)
            return

        # ── Normal forward planning ──
        raw_kappa, all_blocked, _best_idx = select_curvature(
            obstacles,
            goal_bearing,
            self._prev_kappa_ema,
            self._kappa_candidates,
            self._arc_cache,
            robot_radius=cfg.robot_radius,
            clearance_cap=cfg.clearance_cap,
            clearance_weight=cfg.clearance_weight,
            alignment_weight=cfg.alignment_weight,
            hysteresis_weight=cfg.hysteresis_weight,
            hysteresis_sigma=cfg.hysteresis_sigma,
        )

        # ── Churn tracking: when the chosen curvature flips rapidly,
        # shorten the published arc so the robot slows down.
        kappa_delta = abs(raw_kappa - self._prev_kappa_ema)
        self._kappa_delta_ema = update_ema(self._kappa_delta_ema, kappa_delta, cfg.churn_alpha)
        if self._kappa_delta_ema > cfg.churn_threshold:
            # Linear ramp from 1.0 → churn_min_scale as churn grows
            excess = (self._kappa_delta_ema - cfg.churn_threshold) / cfg.churn_threshold
            churn_scale = max(cfg.churn_min_scale, 1.0 - excess * (1.0 - cfg.churn_min_scale))
        else:
            churn_scale = 1.0

        self._prev_kappa_ema = update_ema(self._prev_kappa_ema, raw_kappa, cfg.curvature_ema_alpha)

        path_scale_down = cfg.blocked_speed_scale if all_blocked else 1.0
        # Never publish past the goal; apply churn scale.
        final_length = min(cfg.publish_length, goal_dist) * path_scale_down * churn_scale
        # Floor: at least one arc_step so path has 2+ samples
        final_length = max(final_length, cfg.arc_step)
        final_arc = generate_arc(self._prev_kappa_ema, final_length, cfg.arc_step)

        poses: list[PoseStamped] = []
        for i in range(final_arc.shape[0]):
            poses.append(
                PoseStamped(
                    ts=now,
                    frame_id=cfg.frame_id,
                    position=[float(final_arc[i, 0]), float(final_arc[i, 1]), 0.0],
                    orientation=[0.0, 0.0, 0.0, 1.0],
                )
            )
        self.path.publish(Path(ts=now, frame_id=cfg.frame_id, poses=poses))

        self._maybe_publish_obstacle_cloud(obstacles, now)

        if now - self._last_diag_print >= 1.0:
            self._last_diag_print = now
            # Re-score the chosen candidate just for clearance print
            chosen_idx = int(np.argmin(np.abs(self._kappa_candidates - self._prev_kappa_ema)))
            _, _, clr = score_candidate(
                self._arc_cache[chosen_idx],
                obstacles,
                goal_bearing,
                self._prev_kappa_ema,
                float(self._kappa_candidates[chosen_idx]),
                robot_radius=cfg.robot_radius,
                clearance_cap=cfg.clearance_cap,
                clearance_weight=cfg.clearance_weight,
                alignment_weight=cfg.alignment_weight,
                hysteresis_weight=cfg.hysteresis_weight,
                hysteresis_sigma=cfg.hysteresis_sigma,
            )
            print(
                f"[smooth_local_planner] raw_kappa={raw_kappa:+.3f}  "
                f"ema_kappa={self._prev_kappa_ema:+.3f}  "
                f"clearance={clr:.2f}m  all_blocked={all_blocked}  "
                f"obstacles={obstacles.shape[0]}"
            )

    def _maybe_publish_obstacle_cloud(self, obstacles: np.ndarray, now: float) -> None:
        # 2 Hz is plenty for debug
        if now - self._last_obstacle_pub < 0.5:
            return
        self._last_obstacle_pub = now
        if obstacles.shape[0] == 0:
            pts = np.zeros((0, 3), dtype=np.float32)
        else:
            pts = np.zeros((obstacles.shape[0], 3), dtype=np.float32)
            pts[:, 0] = obstacles[:, 0]
            pts[:, 1] = obstacles[:, 1]
        self.obstacle_cloud.publish(
            PointCloud2.from_numpy(pts, frame_id=self.config.frame_id, timestamp=now)
        )
