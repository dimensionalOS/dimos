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

"""MotionPlanner: the autoresearch planner as a dimos local planner module.

Bridges the referee-side ``PlannerEpisode`` protocol onto module streams:
the raycaster's ``local_map`` is the cloud, leveled body odometry is the
pose, and the goal is a carrot — ``goal_lookahead_m`` of arc along the MLS
global path (``planner_path``), clamped to its end. Ticks on a fixed cadence
but replans only when an input that matters has changed, and publishes the
result as a nav Path. A refusal comes out as the planner made it — a
single-pose stub the follower reads as "hold" — while MLS reroutes globally.
"""

from __future__ import annotations

from threading import Event, RLock, Thread
import time
from typing import Any

import numpy as np
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.motion.adapter.diagnostics import StallReporter
from dimos.navigation.motion.adapter.floor import FloorAnchor
from dimos.navigation.motion.control.profile import encode_precision
from dimos.navigation.motion.control.referee import world as world_bridge
from dimos.navigation.motion.planner.referee.geometry import AvoidanceConfig
from dimos.navigation.motion.planner.referee.planners.base import PlannerEpisode, load
from dimos.navigation.motion.planner.referee.scenarios import EMBODIMENTS, Scenario
from dimos.navigation.motion.planner.referee.types import (
    Path as RefereePath,
    PointCloud2 as RefereeCloud,
)
from dimos.navigation.tf_pose import OdomBasePose
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def annotate(ref: RefereePath, cloud: RefereeCloud, emb: Any, ts: float, frame_id: str) -> Path:
    """Referee path -> stamped nav Path: precision profile in the timestamps."""
    nav = to_nav_path(ref, ts=ts, frame_id=frame_id)
    clearance = world_bridge.path_clearance(ref, cloud, emb)
    return encode_precision(nav, clearance, t0=ts)


def to_nav_path(ref: RefereePath, ts: float = 0.0, frame_id: str = "odom") -> Path:
    """Referee path -> dimos nav_msgs Path (the type the follower consumes)."""
    poses = [
        PoseStamped(
            ts=ts,
            frame_id=frame_id,
            position=Vector3(p.position.x, p.position.y, p.position.z),
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


def route_changed(old: np.ndarray | None, new: np.ndarray | None) -> bool:
    """Is this a different global route, or the same one published again?

    MLS republishes at ~1 Hz and holds still for seconds at a time. Re-solving
    against a route the plan already accounts for is what the gate exists to
    skip, so "changed" has to mean the waypoints moved, not that a message came.
    """
    if old is None or new is None:
        return old is not new
    return old.shape != new.shape or not bool(np.array_equal(old, new))


class MotionPlannerConfig(ModuleConfig):
    planner: str = "target"  # referee registry name or "module:factory"
    embodiment: str = "go2"
    # Plan when an input that MATTERS changed — a new local map, or a global
    # route that is not the one already planned against — rather than on every
    # tick of the clock. The planner ticks at 5 Hz over a 1 Hz map, so four
    # ticks in five re-solve an unchanged world; between maps the plan is stable
    # to 0.15 m, so those four are work whose only output is jitter. The
    # follower tracks the published path as the robot moves and needs no
    # republish to do it. False replans every tick, as it used to.
    replan_on_change: bool = True
    # Odometry is stamped at the SENSOR (mid360_link on the go2), so the pose it
    # carries is the lidar's, not the robot's -- 0.30 m ahead and 0.16 m above on
    # this rig. tf resolves it into the body; ticks are dropped until it can.
    base_frame: str = "base_link"
    replan_hz: float = 5.0  # the control battery's reality default
    goal_lookahead_m: float = 5.0  # carrot arc along the global path
    world_frame: str = "odom"
    # Anchor the cloud to the floor under the robot before planning, so the
    # planner's body z-band (0.05..0.45 ABOVE THE GROUND) lands where the
    # ground actually is. Off leaves the band where the map's z origin puts it,
    # which on a LIO stack is base height — see adapter/floor.py.
    floor_anchor: bool = True
    # Lidar height above the ground while standing. With it, tf gives the base
    # height above ground and hence a prior the floor estimate is bounded
    # against; 0 estimates off the cloud alone. Same knob GoalRelay carries.
    lidar_height: float = 0.0
    # Drop returns within this of the estimated floor before the band is taken.
    # TWO voxel layers, not one: a floor whose true height sits near a voxel
    # boundary quantises into both layers either side of it, and a one-voxel
    # margin leaves the upper one standing as a carpet the search cannot cross.
    # Measured on 20260805-033007: at 0.08 the robot is inside the band on
    # every tick, at 0.16 on 7 % of them.
    ground_margin_m: float = 0.16
    # Manual trim on the map's z ORIGIN, so it lands before the floor is
    # measured off that map. With floor_anchor on it is very nearly inert; the
    # anchoring above is what makes the band right.
    cloud_z_offset: float = 0.0
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
        # Arrival counters, and the pair the published plan was made from.
        self._cloud_seq = 0
        self._route_seq = 0
        self._planned: tuple[int, int] | None = None
        self._pose: tuple[float, float, float] | None = None
        self._floor = FloorAnchor(
            doing="planning",
            enabled=self.config.floor_anchor,
            lidar_height=self.config.lidar_height,
            ground_margin=self.config.ground_margin_m,
            base_frame=self.config.base_frame,
        )
        self._base_pose: OdomBasePose | None = None
        self._global_xy: np.ndarray | None = None
        self._episode: PlannerEpisode | None = None
        self._emb = EMBODIMENTS["go2"]
        self._stop_event = Event()
        self._thread: Thread | None = None
        self._stall = StallReporter("MotionPlanner", self.config.stall_report_s)
        self._viz_at = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        sc = Scenario("live", [], goal=(0.0, 0.0), emb=EMBODIMENTS[self.config.embodiment])
        self._emb = sc.emb
        self._episode = load(self.config.planner)(sc, AvoidanceConfig())
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
        self._floor.observe(self._base_pose, msg.child_frame_id, pose.position.z)
        with self._lock:
            self._pose = (pose.position.x, pose.position.y, pose.orientation.euler[2])

    def _on_planner_path(self, msg: Path) -> None:
        # MLS emits an empty path when it finds no route: no carrot, hold the
        # last local plan rather than chase a stale one.
        xy = np.array([[p.position.x, p.position.y] for p in msg.poses]).reshape(-1, 2)
        route = xy if len(xy) else None
        with self._lock:
            changed = route_changed(self._global_xy, route)
            self._global_xy = route
            if changed:
                self._route_seq += 1
        if changed and self._episode is not None:
            # a new task: warm starts and hysteresis from the old route are
            # stale (no-op for the stateless rust target)
            self._episode.reset()

    def _plan_loop(self) -> None:
        period = 1.0 / self.config.replan_hz
        while not self._stop_event.is_set():
            started = time.perf_counter()
            with self._lock:
                cloud, pose, global_xy = self._cloud, self._pose, self._global_xy
                cloud_at = self._cloud_at
                inputs = (self._cloud_seq, self._route_seq)
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
                # was planned so the first live tick plans again.
                self._planned = None
                self._hold(pose, age)
            elif cloud is not None and pose is not None and global_xy is not None:
                if self._stale:
                    self._stale = False
                    logger.info("local_map is live again, resuming planning")
                if self._due(inputs):
                    goal = carrot_along(global_xy, (pose[0], pose[1]), self.config.goal_lookahead_m)
                    if self._plan_once(cloud, pose, goal):
                        self._planned = inputs
            elapsed = time.perf_counter() - started
            self._stop_event.wait(max(0.0, period - elapsed))

    def _due(self, inputs: tuple[int, int]) -> bool:
        """Has an input the plan depends on arrived since the plan was made?"""
        return not self.config.replan_on_change or inputs != self._planned

    def _hold(self, pose: tuple[float, float, float], age: float) -> None:
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
            position=Vector3(pose[0], pose[1], 0.0),
            orientation=Quaternion.from_euler(Vector3(0.0, 0.0, pose[2])),
        )
        held = Path(ts=ts, frame_id=self.config.world_frame, poses=[stub])
        self.path.publish(held)
        self._publish_viz(held)

    def _plan_once(
        self, cloud: PointCloud2, pose: tuple[float, float, float], goal: tuple[float, float]
    ) -> bool:
        """Plan and publish. False when the search raised and nothing went out."""
        assert self._episode is not None
        # The trim corrects the map's z ORIGIN, so it lands before the floor is
        # measured off that map — which is also where the rust twin applies it
        # (at extraction). With floor_anchor on it is very nearly inert.
        pts = cloud.points_f32()
        if self.config.cloud_z_offset != 0.0:
            pts = pts + np.array([0.0, 0.0, self.config.cloud_z_offset], dtype=np.float32)
        pts = self._floor.anchor(pts, (pose[0], pose[1]))
        ref_cloud = RefereeCloud.from_numpy(pts, frame_id=self.config.world_frame)
        try:
            ref = self._episode.plan(ref_cloud, pose, goal)
        except Exception:
            logger.exception("planner failed; keeping the last published path")
            return False
        plan = annotate(ref, ref_cloud, self._emb, ts=time.time(), frame_id=self.config.world_frame)
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
