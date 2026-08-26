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

"""MotionPlanner: the SE(2) local planner as a dimos module.

Bridges the ``PlannerEpisode`` protocol onto module streams:
the raycaster's ``local_map`` is the cloud, leveled body odometry is the
pose, and the goal is a carrot — ``goal_lookahead_m`` of arc along the MLS
global path (``planner_path``), clamped to its end. Ticks on a fixed cadence
but replans only when an input that matters has changed, and publishes the
result as a nav Path. A refusal comes out as the planner made it — a
single-pose stub the follower reads as "hold" — while MLS reroutes globally.
"""

from __future__ import annotations

import math
from threading import Event, RLock, Thread
import time
from typing import Any

import numpy as np
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.motion.adapter.diagnostics import StallReporter
from dimos.navigation.motion.adapter.follower import path_clearance
from dimos.navigation.motion.control.profile import encode_precision
from dimos.navigation.motion.embodiment import EMBODIMENTS, Embodiment
from dimos.navigation.motion.obstacles import ObstacleModel, hard_points, load as load_model
from dimos.navigation.motion.planner.planners.base import PlannerEpisode, load
from dimos.navigation.tf_pose import OdomBasePose
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def annotate(
    ref: Path,
    obstacles: np.ndarray,
    emb: Embodiment,
    ts: float,
    frame_id: str,
    ground_z: float = 0.0,
) -> Path:
    """The planner's route as the follower's path: stamped, grounded, precision profile in the timestamps."""
    nav = stamped(ref, ts=ts, frame_id=frame_id, ground_z=ground_z)
    xy = np.array([[p.position.x, p.position.y] for p in ref.poses]).reshape(-1, 2)
    clearance = path_clearance(xy, obstacles, emb.width / 2.0)
    return encode_precision(nav, clearance, t0=ts)


def stamped(ref: Path, ts: float = 0.0, frame_id: str = "odom", ground_z: float = 0.0) -> Path:
    """The planner's planar route stamped with time, frame and the ground it stands on.

    The search is planar and hands back z = 0, but `odom` z = 0 is wherever the
    LIO frame started -- on this rig, a lidar's height above the floor. Stamping
    the plan with `ground_z` (the surface the feet stand on, which the module
    already tracks for the obstacle model) puts the route on the ground instead
    of floating it over the robot. Every consumer of the path is planar; only a
    viewer reads the z.
    """
    poses = [
        PoseStamped(
            ts=ts,
            frame_id=frame_id,
            position=Vector3(p.position.x, p.position.y, ground_z),
            orientation=Quaternion(
                p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w
            ),
        )
        for p in ref.poses
    ]
    return Path(ts=ts, frame_id=frame_id, poses=poses)


def carrot_along(
    path_xy: np.ndarray, robot_xy: tuple[float, float], lookahead: float
) -> tuple[float, float]:
    """`lookahead` metres of arc along the path from the waypoint closest to
    the robot, clamped to the path end."""
    xy = np.asarray(path_xy, dtype=float).reshape(-1, 2)
    i = int(np.argmin(np.linalg.norm(xy - robot_xy, axis=1)))
    remaining = lookahead
    for j in range(i, len(xy) - 1):
        seg = xy[j + 1] - xy[j]
        seg_len = float(np.linalg.norm(seg))
        if seg_len >= remaining:
            point = xy[j] + (remaining / seg_len) * seg
            return (float(point[0]), float(point[1]))
        remaining -= seg_len
    return (float(xy[-1][0]), float(xy[-1][1]))


REPLAN_CARROT_M = 0.2  # carrot move that earns a replan
RESET_CARROT_M = 1.0  # carrot jump that means a different task


def replan_due(
    planned: tuple[int, tuple[float, float]] | None,
    cloud_seq: int,
    carrot: tuple[float, float],
    carrot_m: float = REPLAN_CARROT_M,
) -> bool:
    """Has an input the plan depends on moved since the plan was made?

    The plan consumes the global route through exactly one quantity — the
    carrot — so that is what the gate compares. Keying on the waypoint array
    instead never dedups anything: MLS trims the route head to the robot on
    every ~1 Hz republish and re-solves with tail wobble, so the array moves
    every time while the carrot does not move at all.
    """
    if planned is None:
        return True
    seq, was = planned
    return seq != cloud_seq or math.dist(was, carrot) > carrot_m


class MotionPlannerConfig(ModuleConfig):
    planner: str = "target"  # registry name or "module:factory"
    embodiment: str = "go2"
    # HOW AGGRESSIVE the search is allowed to be, and the only two numbers that
    # decide whether a gap admits a route. A gap has to be
    # `box_width + 2 * clearance_margin_m` wide before one exists at all.
    #
    # `body_dilate_m` grows (or, negative, shrinks) every planning box PER SIDE.
    # The table's boxes are MEASURED -- the swinging legs, not the trunk, set
    # the width, which is why they read wider than the robot looks -- so a
    # negative value here is a deployment saying "plan me tighter than the legs
    # measured", and the legs are what pays for it.
    #
    # The hard margin the search adds on top stays the embodiment's own
    # precision floor -- what the follower can actually hold -- because a
    # `float | None` cannot cross into the native twin, and one knob that both
    # halves carry beats two that drift.
    body_dilate_m: float = 0.0
    # Plan when an input that MATTERS changed — a new local map, or a carrot
    # that moved — rather than on every tick of the clock. The planner ticks at
    # 5 Hz over a 1 Hz map, so four ticks in five re-solve an unchanged world;
    # between maps the plan is stable to 0.15 m, so those four are work whose
    # only output is jitter. The follower tracks the published path as the robot
    # moves and needs no republish to do it. False replans every tick.
    replan_on_change: bool = True
    # How far the carrot has to move to be worth re-solving for. The route the
    # carrot rides on is republished at ~1 Hz with its head trimmed to the robot
    # and its tail re-solved, so the waypoints move every time and the carrot
    # does not — gating on the array would dedup nothing.
    replan_carrot_m: float = REPLAN_CARROT_M
    # A carrot that jumped this far is a different task, and the episode's warm
    # start and hysteresis are about the old one. Republish noise moves it ~0 m;
    # a real reroute moved it 4.6 m in the door recording.
    reset_carrot_m: float = RESET_CARROT_M
    # Odometry is stamped at the SENSOR (mid360_link on the go2), so the pose it
    # carries is the lidar's, not the robot's -- 0.30 m ahead and 0.16 m above on
    # this rig. tf resolves it into the body; ticks are dropped until it can.
    base_frame: str = "base_link"
    replan_hz: float = 5.0
    goal_lookahead_m: float = 5.0  # carrot arc along the global path
    world_frame: str = "odom"
    # What counts as an obstacle (motion/obstacles.py). "body_band" reads the
    # cloud against the surface the feet stand on, which the embodiment knows
    # the base's height above.
    obstacle_model: str = "body_band"
    # Hold once the local map is this old. The mapper can live across a link,
    # and a dropped link must not leave us replanning on a frozen world at
    # cruise speed — an old map is survivable, an unbounded one is not.
    max_map_age_s: float = 5.0
    # Publish the plan's expected body poses for the viewer (0.0 = off). Drives
    # adapter/viz.py's override too, so drawing and publishing cannot drift.
    viz_publish_hz: float = 2.0
    # Seconds between "still blocked, and here is what on" lines while an input
    # the planner needs has never arrived or has gone away.
    stall_report_s: float = 3.0


class MotionPlanner(Module):
    """Receding-horizon local planning over the live local map."""

    config: MotionPlannerConfig

    local_map: In[PointCloud2]
    odometry: In[Odometry]
    planner_path: In[Path]
    tf: In[TFMessage]

    path: Out[Path]
    plan_body: Out[Path]  # the same plan, subsampled, for the viewer's body boxes

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = RLock()
        self._cloud: PointCloud2 | None = None
        self._cloud_at: float | None = None
        self._stale = False
        # The cloud arrival counter, and the (cloud, carrot) the published plan
        # was made from.
        self._cloud_seq = 0
        self._planned: tuple[int, tuple[float, float]] | None = None
        # ...and the plan itself. The search prefers the route it already
        # published unless a fresh one earns the switch, and this module is
        # where that memory lives: the shell owns it, the planner judges it.
        self._incumbent: Path | None = None
        self._pose: PoseStamped | None = None
        # Where the surface under the robot is, off the body rather than off
        # the scene: the base rides emb.base_height above it.
        self._ground_z: float | None = None
        self._base_pose: OdomBasePose | None = None
        self._global_xy: np.ndarray | None = None
        self._episode: PlannerEpisode | None = None
        self._emb = EMBODIMENTS[self.config.embodiment].dilated(by=self.config.body_dilate_m)
        self._model: ObstacleModel = load_model(self.config.obstacle_model, self._emb)
        self._stop_event = Event()
        self._thread: Thread | None = None
        self._stall = StallReporter("MotionPlanner", self.config.stall_report_s)
        self._viz_at = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        self._episode = load(self.config.planner)(self._emb)
        self._episode.reset()
        self.register_disposable(Disposable(self.local_map.subscribe(self._on_local_map)))
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odometry)))
        self.register_disposable(Disposable(self.planner_path.subscribe(self._on_planner_path)))
        self._thread = Thread(target=self._plan_loop, daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        super().stop()

    def _on_local_map(self, msg: PointCloud2) -> None:
        with self._lock:
            self._cloud = msg
            self._cloud_seq += 1
            # arrival, not msg.ts: the mapper's clock may not be ours, and
            # what this guards is "how long since the mapper was heard from"
            self._cloud_at = time.monotonic()

    def _on_odometry(self, msg: Odometry) -> None:
        if self._base_pose is None:
            self._base_pose = OdomBasePose(self.tfbuffer, self.config.base_frame)
        pose = self._base_pose.resolve(msg)
        if pose is None:
            return
        with self._lock:
            self._pose = pose
            self._ground_z = pose.position.z - self._emb.base_height

    def _on_planner_path(self, msg: Path) -> None:
        # MLS emits an empty path when it finds no route: no carrot, hold the
        # last local plan rather than chase a stale one.
        xy = np.array([[p.position.x, p.position.y] for p in msg.poses]).reshape(-1, 2)
        with self._lock:
            self._global_xy = xy if len(xy) else None

    def _plan_loop(self) -> None:
        period = 1.0 / self.config.replan_hz
        while not self._stop_event.is_set():
            started = time.perf_counter()
            with self._lock:
                cloud, pose, global_xy = self._cloud, self._pose, self._global_xy
                cloud_at, ground_z = self._cloud_at, self._ground_z
                cloud_seq = self._cloud_seq
            age = None if cloud_at is None else time.monotonic() - cloud_at
            # Why a tick did nothing, in the planner's own words. Silence here
            # is the failure that looks like "the robot will not move" from the
            # outside, and it is the one a log has to be able to answer.
            self._stall.check(
                {
                    "local_map": cloud is not None,
                    "odometry": pose is not None,
                    "planner_path (global route/goal)": global_xy is not None,
                }
            )
            if pose is not None and age is not None and age > self.config.max_map_age_s:
                # A hold is not gated: it is a statement about the CLOCK, and
                # nothing arriving is exactly the case it fires on. Forget what
                # was planned so the first live tick plans again -- and what was
                # published with it: a route held across a dead link is a route
                # nothing has re-validated.
                self._planned = None
                self._incumbent = None
                self._hold(pose, age)
            elif cloud is not None and pose is not None and global_xy is not None:
                if self._stale:
                    self._stale = False
                    logger.info("local_map is live again, resuming planning")
                # the carrot is the whole of the route the plan consumes, so it
                # is computed every tick and the gate reads it, not the array
                goal = carrot_along(global_xy, (pose.x, pose.y), self.config.goal_lookahead_m)
                if self._due(cloud_seq, goal):
                    if self._retask(goal) and self._episode is not None:
                        # a new task: warm starts, hysteresis and the route
                        # being held are all about the old one
                        self._episode.reset()
                        self._incumbent = None
                    # a pose implies a ground reference: both come off the same
                    # tf-resolved base, so this cannot be None here
                    if self._plan_once(cloud, pose, goal, 0.0 if ground_z is None else ground_z):
                        self._planned = (cloud_seq, goal)
            elapsed = time.perf_counter() - started
            self._stop_event.wait(max(0.0, period - elapsed))

    def _due(self, cloud_seq: int, carrot: tuple[float, float]) -> bool:
        """Has an input the plan depends on moved since the plan was made?"""
        if not self.config.replan_on_change:
            return True
        return replan_due(self._planned, cloud_seq, carrot, self.config.replan_carrot_m)

    def _retask(self, carrot: tuple[float, float]) -> bool:
        """Did the carrot jump far enough to be a different task?"""
        if self._planned is None:
            return False
        return math.dist(self._planned[1], carrot) > self.config.reset_carrot_m

    def _hold(self, pose: PoseStamped, age: float) -> None:
        """Refuse the way the planner does — a single-pose stub reads as "stop"."""
        # edge-triggered: the loop runs at replan_hz, and a dead link would
        # otherwise warn five times a second for as long as it stays dead
        if not self._stale:
            self._stale = True
            logger.warning(
                "local_map is stale, holding",
                age_s=round(age, 1),
                max_map_age_s=self.config.max_map_age_s,
            )
        ts = time.time()
        stub = PoseStamped(
            ts=ts,
            frame_id=self.config.world_frame,
            position=Vector3(pose.x, pose.y, 0.0),
            orientation=Quaternion.from_euler(Vector3(0.0, 0.0, pose.yaw)),
        )
        held = Path(ts=ts, frame_id=self.config.world_frame, poses=[stub])
        self.path.publish(held)
        self._publish_viz(held)

    def _plan_once(
        self,
        cloud: PointCloud2,
        pose: PoseStamped,
        goal: tuple[float, float],
        ground_z: float,
    ) -> bool:
        """Plan and publish. False when the search raised and nothing went out."""
        assert self._episode is not None
        # The search gets the obstacles, as xy: which returns are obstacles was
        # decided here, by the model, and the search has no z to decide it again
        # with. The follower's room hint is measured off the very same points,
        # so the governor and the stamped profile cannot be pricing different
        # worlds.
        pts = hard_points(self._model, cloud.points_f32(), ground_z)
        try:
            ref = self._episode.plan(pts[:, :2], pose, Pose(goal[0], goal[1], 0.0), self._incumbent)
        except Exception:
            logger.exception("planner failed; keeping the last published path")
            return False
        self._incumbent = ref
        plan = annotate(
            ref,
            pts,
            self._emb,
            ts=time.time(),
            frame_id=self.config.world_frame,
            ground_z=ground_z,
        )
        self.path.publish(plan)
        self._stall.ok(f"planning: {len(plan.poses)} waypoints")
        self._publish_viz(plan)
        return True

    def _publish_viz(self, plan: Path) -> None:
        """Mirror the plan onto the viewer stream, at its own rate."""
        hz = self.config.viz_publish_hz
        if hz <= 0.0:
            return
        now = time.monotonic()
        if now - self._viz_at < 1.0 / hz:
            return
        self._viz_at = now
        self.plan_body.publish(plan)
