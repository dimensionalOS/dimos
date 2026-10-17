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

"""LocalPlanner: the SE(2) local planner as a dimos module.

Cloud = ``local_map``, pose = the ``world_frame -> base_frame`` edge on tf, goal = a carrot ``goal_lookahead_m``
along the MLS ``planner_path``. Replans only when an input changed; a refusal goes out as the planner's single-pose stub.
"""

from __future__ import annotations

from collections.abc import Callable
import math
from threading import Event, RLock, Thread
import time
from typing import Any

import numpy as np
from numpy.typing import NDArray
from pydantic import Field, ImportString
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation import spec
from dimos.navigation.embodiment.base import Embodiment
from dimos.navigation.embodiment.go2 import GO2
from dimos.navigation.local_planner.obstacles import (
    ObstacleModel,
    ground_points,
    hard_points,
    load as load_model,
    path_clearance,
)
from dimos.navigation.local_planner.profile import encode_precision
from dimos.navigation.local_planner.search.base import PlannerEpisode
from dimos.navigation.tf_pose import TfPose
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def annotate(
    ref: Path,
    obstacles: NDArray[np.float32],
    emb: Embodiment,
    ts: float,
    frame_id: str,
    ground_z: float = 0.0,
) -> Path:
    """The planner's route as the follower's path: stamped, grounded, precision profile in the timestamps."""
    nav = stamped(ref, ts=ts, frame_id=frame_id, ground_z=ground_z)
    xy = np.array([[p.position.x, p.position.y] for p in ref.poses]).reshape(-1, 2)
    clearance = path_clearance(xy, obstacles, emb.width / 2.0)
    return encode_precision(nav, clearance, emb, t0=ts)


def stamped(ref: Path, ts: float = 0.0, frame_id: str = "odom", ground_z: float = 0.0) -> Path:
    """The route stamped with time, frame and ground: the search's z = 0 is not the floor, `ground_z` is."""
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
    path_xy: NDArray[np.float64], robot_xy: tuple[float, float], lookahead: float
) -> tuple[float, float]:
    """`lookahead` metres of arc from the waypoint nearest the robot, clamped to the path end."""
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


REPLAN_CARROT_M = 0.3  # carrot move that earns a replan
RESET_CARROT_M = 1.0  # carrot jump that means a different task


def replan_due(
    planned: tuple[int, tuple[float, float]] | None,
    cloud_seq: int,
    carrot: tuple[float, float],
    carrot_m: float = REPLAN_CARROT_M,
) -> bool:
    """Has an input the plan depends on moved since the plan was made?

    Keyed on the carrot, not the waypoint array: MLS re-solves the array on every ~1 Hz republish.
    """
    if planned is None:
        return True
    seq, was = planned
    return seq != cloud_seq or math.dist(was, carrot) > carrot_m


def retask_due(
    planned: tuple[int, tuple[float, float]] | None,
    carrot: tuple[float, float],
    reset_m: float = RESET_CARROT_M,
) -> bool:
    """Did the carrot jump far enough from the planned one to be a different task?"""
    return planned is not None and math.dist(planned[1], carrot) > reset_m


class LocalPlannerConfig(ModuleConfig):
    planner: ImportString[Callable[..., Any]] = Field(
        "dimos.navigation.local_planner.search.target:make_py", validate_default=True
    )
    embodiment: Embodiment = GO2
    # Grows (negative: shrinks) every planning box per side; the hard margin stays the embodiment's precision floor.
    body_dilate_m: float = 0.0
    # Price multiplier per metre over lattice cells the cloud saw no floor under; <= 1 turns it off.
    unseen_cost: float = 5.0
    # Replan only when the map or the carrot (by replan_carrot_m) changed; False replans every tick.
    replan_on_change: bool = True
    replan_carrot_m: float = REPLAN_CARROT_M
    # A carrot jump this far is a new task: the episode's warm start and hysteresis are reset.
    reset_carrot_m: float = RESET_CARROT_M
    # The pose is the `world_frame -> base_frame` edge on tf, read each tick.
    world_frame: str = "odom"
    base_frame: str = "base_link"
    replan_hz: float = 5.0
    goal_lookahead_m: float = 5.0  # carrot arc along the global path
    # What counts as an obstacle (obstacles.py); "body_band" reads the cloud against the surface the feet stand on.
    obstacle_model: str = "body_band"
    # Hold once the local map is this old: a dropped link must not leave us replanning on a frozen world.
    max_map_age_s: float = 5.0


class LocalPlanner(Module, spec.MapLocalPlanner):
    """Receding-horizon local planning over the live local map."""

    config: LocalPlannerConfig

    local_map: In[PointCloud2]
    planner_path: In[Path]
    tf: IO[TFMessage]

    path: Out[Path]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = RLock()
        self._cloud: PointCloud2 | None = None
        self._cloud_at: float | None = None
        self._stale = False
        self._cloud_seq = 0
        # (cloud_seq, carrot) the published plan was made from
        self._planned: tuple[int, tuple[float, float]] | None = None
        # the published route; the search prefers it unless a fresh one earns the switch
        self._incumbent: Path | None = None
        # so a route that vanishes is cleared once
        self._published = False
        # Built in start(): the tf buffer needs the port's transport.
        self._pose_src: TfPose | None = None
        self._global_xy: NDArray[np.float64] | None = None
        self._emb = self.config.embodiment.dilated(by=self.config.body_dilate_m)
        self._model: ObstacleModel = load_model(self.config.obstacle_model, self._emb)
        self._episode: PlannerEpisode = self.config.planner(self._emb)
        self._stop_event = Event()
        self._thread: Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._episode.reset()
        self._pose_src = TfPose(self.tfbuffer, self.config.base_frame, self.config.max_map_age_s)
        self.register_disposable(Disposable(self.local_map.subscribe(self._on_local_map)))
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
            # arrival, not msg.ts: this measures how long since the mapper was heard from
            self._cloud_at = time.monotonic()

    def _on_planner_path(self, msg: Path) -> None:
        # an empty path is MLS finding no route: no carrot, hold the last local plan
        xy = np.array([[p.position.x, p.position.y] for p in msg.poses]).reshape(-1, 2)
        with self._lock:
            self._global_xy = xy if len(xy) else None

    def _plan_loop(self) -> None:
        period = 1.0 / self.config.replan_hz
        while not self._stop_event.is_set():
            started = time.perf_counter()
            self.tick()
            elapsed = time.perf_counter() - started
            self._stop_event.wait(max(0.0, period - elapsed))

    def tick(self) -> None:
        """One replan tick: plan, hold, or say what is missing."""
        pose = None if self._pose_src is None else self._pose_src.get(self.config.world_frame)
        with self._lock:
            cloud, global_xy = self._cloud, self._global_xy
            cloud_at, cloud_seq = self._cloud_at, self._cloud_seq
        age = None if cloud_at is None else time.monotonic() - cloud_at
        if pose is not None and age is not None and age > self.config.max_map_age_s:
            # forget the plan and the incumbent: a route held across a dead link is unvalidated
            self._planned = None
            self._incumbent = None
            self.hold(pose, age)
        elif global_xy is None:
            self.clear()
        elif pose is not None and cloud is not None:
            if self._stale:
                self._stale = False
                logger.info("local_map is live again, resuming planning")
            # the gate reads the carrot, not the array (replan_due)
            goal = carrot_along(global_xy, (pose.x, pose.y), self.config.goal_lookahead_m)
            if self.due(cloud_seq, goal):
                if self.retask(goal):
                    # a new task: warm start, hysteresis and the incumbent are about the old one
                    self._episode.reset()
                    self._incumbent = None
                # the base rides emb.base_height above the surface
                ground_z = pose.position.z - self._emb.base_height
                if self.plan_once(cloud, pose, goal, ground_z):
                    self._planned = (cloud_seq, goal)

    def due(self, cloud_seq: int, carrot: tuple[float, float]) -> bool:
        """Has an input the plan depends on moved since the plan was made?"""
        if not self.config.replan_on_change:
            return True
        return replan_due(self._planned, cloud_seq, carrot, self.config.replan_carrot_m)

    def retask(self, carrot: tuple[float, float]) -> bool:
        """Did the carrot jump far enough to be a different task?"""
        return retask_due(self._planned, carrot, self.config.reset_carrot_m)

    def clear(self) -> None:
        """The global route is gone: publish one empty path (the follower's stop) and forget the plan."""
        if not self._published:
            return
        self._published = False
        self._planned = None
        self._incumbent = None
        self._episode.reset()
        logger.info("planner_path is empty, clearing the local plan")
        self.path.publish(Path(ts=time.time(), frame_id=self.config.world_frame, poses=[]))

    def hold(self, pose: PoseStamped, age: float) -> None:
        """Refuse the way the planner does: a single-pose stub reads as "stop"."""
        # edge-triggered, or a dead link warns at replan_hz
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
            position=Vector3(pose.x, pose.y, pose.position.z - self._emb.base_height),
            orientation=Quaternion.from_euler(Vector3(0.0, 0.0, pose.yaw)),
        )
        held = Path(ts=ts, frame_id=self.config.world_frame, poses=[stub])
        self._published = True
        self.path.publish(held)

    def plan_once(
        self,
        cloud: PointCloud2,
        pose: PoseStamped,
        goal: tuple[float, float],
        ground_z: float,
    ) -> bool:
        """Plan and publish. False when the search raised and nothing went out."""
        # the model decides the obstacles here; the follower's room hint is measured off the same points
        raw = cloud.points_f32()
        pts = hard_points(self._model, raw, ground_z)
        try:
            ref = self._episode.plan(
                pts[:, :2],
                pose,
                Pose(goal[0], goal[1], 0.0),
                self._incumbent,
                ground=ground_points(raw, ground_z),
                unseen_cost=self.config.unseen_cost,
            )
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
        self._published = True
        self.path.publish(plan)
        return True
