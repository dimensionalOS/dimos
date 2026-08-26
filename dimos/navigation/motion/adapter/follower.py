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

"""TrajectoryFollower: the motion controller as a dimos module.

A thin transport shell around the pluggable ``TrajectoryController`` — the
controller stays a pure pose+path -> twist law (the piece that later ports
to rust); this module owns subscriptions, the control clock, the on-robot
clearance annotation and goal arrival. Clearance is recomputed from the local
map per (path, map) pair.

It reads the map through the planner's own obstacle model
(``motion/obstacles.py``), because the governor and the planner's stamped
precision profile have to be talking about the same slice of the world.
"""

from __future__ import annotations

import math
from threading import Event, RLock, Thread
import time
from typing import Any

from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]
import numpy as np
from pydantic import Field
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.motion.adapter.diagnostics import StallReporter
from dimos.navigation.motion.control.controller import (
    ControllerConfig,
    TrajectoryController,
    load,
)
from dimos.navigation.motion.control.profile import ceilings_to_clearance, decode_ceilings
from dimos.navigation.motion.control.tracks import TRACKS
from dimos.navigation.motion.embodiment.registry import EMBODIMENTS
from dimos.navigation.motion.obstacles import ObstacleModel, hard_points, load as load_model
from dimos.navigation.tf_pose import OdomBasePose
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def path_clearance(xy: np.ndarray, points: np.ndarray, half_width: float) -> np.ndarray:
    """Per-waypoint room hint (m): nearest obstacle minus the half-width.

    A speed hint for the controller, not a safety contract. Nothing to hit or
    an empty path = infinite room. `points` is an obstacle model's hard set
    (motion/obstacles.py) — every row is something the body can hit, and z
    rides along unread. Deciding that again here would price a different world
    than the plan was made for, and would truncate any body taller than
    whatever band this function happened to carry.
    """
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    band = np.asarray(points, dtype=np.float32).reshape(-1, 3)[:, :2]
    if not len(band) or not len(xy):
        return np.full(len(xy), np.inf)
    from scipy.spatial import cKDTree

    d, _ = cKDTree(band).query(xy)
    return np.asarray(d, dtype=float) - half_width


class GoalLatch:
    """Arrival edge detector: fires once per goal, then holds until it moves."""

    def __init__(self, tolerance: float) -> None:
        self.tolerance = tolerance
        self._goal: tuple[float, float] | None = None
        self._reached = False

    @property
    def reached(self) -> bool:
        return self._reached

    def set_goal(self, xy: tuple[float, float]) -> None:
        # moves under the arrival tolerance are the same goal — replans snap
        # the path end to the search grid, and re-chasing that is jitter
        if self._goal is None or math.dist(xy, self._goal) > self.tolerance:
            self._goal = xy
            self._reached = False

    def arrive(self, xy: tuple[float, float]) -> bool:
        """True exactly once: the tick this position first reaches the goal."""
        if self._goal is None or self._reached:
            return False
        if math.dist(xy, self._goal) < self.tolerance:
            self._reached = True
            return True
        return False


class TrajectoryFollowerConfig(ModuleConfig):
    # The track fixes the law AND whether the controller is handed the path
    # room hint (control/tracks.py). "hinted" is right wherever the raycaster's
    # local map is actually live, which on the go2-zenoh stack it is; "blind"
    # runs the law that recovers required precision from the path stamps alone.
    track: str = "hinted"
    controller: str | None = None  # registry name or "module:factory"; None = the track's law
    controller_config: ControllerConfig = Field(default_factory=ControllerConfig)
    control_frequency: float = 10.0
    goal_tolerance: float = 0.20  # planar distance that counts as arrival (m)
    # The clearance hint this module recomputes on the robot has to be the same
    # quantity the planner stamped into the path, or the follower's governor is
    # reading a different world than the one that was planned. The referee's
    # control/world.py takes `emb.width / 2`, so this does too -- naming the
    # EMBODIMENT rather than a number, so body dimensions live in exactly one
    # place (planner/planners/se2.py). There is deliberately no
    # half_width override: a `float | None` cannot cross into the native module
    # (`to_config_dict` drops None and `#[native_config]` bans Option), and a
    # knob the deployed twin cannot carry is a knob that drifts.
    embodiment: str = "go2"
    # Must equal the planner's `body_dilate_m`: the room hint has to price the
    # body the plan was made for, or the governor creeps through gaps the plan
    # calls fine.
    body_dilate_m: float = 0.0
    # Odometry is stamped at the SENSOR (mid360_link on the go2), so the pose it
    # carries is the lidar's, not the robot's -- 0.30 m ahead and 0.16 m above on
    # this rig. tf resolves it into the body; ticks are dropped until it can.
    base_frame: str = "base_link"
    # Which returns are obstacles (motion/obstacles.py). It has to be the
    # planner's model, or the room hint measured here is a different world than
    # the one the plan was priced in — MotionPlannerConfig carries the twin.
    obstacle_model: str = "body_band"
    # Seconds between "still not moving, and here is why" lines.
    stall_report_s: float = 3.0
    # A commanded speed at or under this is standing still, whatever the reason.
    idle_speed: float = 0.02


class TrajectoryFollower(Module):
    """Track the planned path; stop and latch goal_reached on arrival."""

    config: TrajectoryFollowerConfig

    path: In[Path]
    odometry: In[Odometry]
    local_map: In[PointCloud2]
    stop_movement: In[Bool]
    tf: In[TFMessage]

    nav_cmd_vel: Out[Twist]
    goal_reached: Out[Bool]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = RLock()
        self._pose: PoseStamped | None = None
        self._base_pose: OdomBasePose | None = None
        self._path: Path | None = None
        self._cloud: PointCloud2 | None = None
        self._clearance: np.ndarray | None = None
        # The very (path, cloud) the hint was measured from, held rather than
        # keyed by id(): CPython recycles addresses, so a cache that remembers
        # only an id compares equal to a NEW path that landed in the freed one's
        # slot and serves the previous plan's room. Holding the refs pins them,
        # which makes `is` exact and costs one extra cloud alive.
        self._clearance_path: Path | None = None
        self._clearance_cloud: PointCloud2 | None = None
        self._latch = GoalLatch(self.config.goal_tolerance)
        self._track = TRACKS[self.config.track]
        # The same dilation the planner used, or the governor prices a body
        # the plan was not made for and creeps through gaps the plan calls fine.
        self._emb = EMBODIMENTS[self.config.embodiment].dilated(by=self.config.body_dilate_m)
        self._model: ObstacleModel = load_model(self.config.obstacle_model, self._emb)
        self._half_width = self._emb.width / 2.0
        self._controller: TrajectoryController | None = None
        self._stop_event = Event()
        self._thread: Thread | None = None
        self._stall = StallReporter("TrajectoryFollower", self.config.stall_report_s)
        # Edge trigger for the hinted track running without its map (follower.rs::Gate).
        self._blind = False

    @rpc
    def start(self) -> None:
        super().start()
        self._controller = load(self.config.controller or self._track.controller)(
            self.config.controller_config, self._emb
        )
        self._controller.reset()
        self.register_disposable(Disposable(self.path.subscribe(self._on_path)))
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odometry)))
        self.register_disposable(Disposable(self.local_map.subscribe(self._on_local_map)))
        if self.stop_movement.transport is not None:
            self.register_disposable(Disposable(self.stop_movement.subscribe(self._on_stop)))
        self._thread = Thread(target=self._control_loop, daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self.nav_cmd_vel.publish(Twist())
        super().stop()

    def _on_path(self, msg: Path) -> None:
        with self._lock:
            self._path = msg
            if len(msg.poses) >= 2:
                # the plan ends at the goal; a single-pose stub is a refusal,
                # never an arrival target
                self._latch.set_goal((msg.poses[-1].position.x, msg.poses[-1].position.y))

    def _on_odometry(self, msg: Odometry) -> None:
        if self._base_pose is None:
            self._base_pose = OdomBasePose(self.tfbuffer, self.config.base_frame)
        pose = self._base_pose.resolve(msg)
        if pose is None:
            return
        with self._lock:
            self._pose = pose

    def _on_local_map(self, msg: PointCloud2) -> None:
        with self._lock:
            self._cloud = msg

    def _on_stop(self, msg: Bool) -> None:
        if msg.data:
            with self._lock:
                self._path = None
            self.nav_cmd_vel.publish(Twist())

    def _control_loop(self) -> None:
        period = 1.0 / self.config.control_frequency
        while not self._stop_event.is_set():
            started = time.perf_counter()
            with self._lock:
                pose, path = self._pose, self._path
            # A follower with no pose or no plan is not "not moving", it is not
            # RUNNING, and those want different fixes. `path` going None is also
            # how stop_movement lands here, so it is named as such.
            if self._stall.check(
                {"odometry": pose is not None, "path (local plan)": path is not None}
            ):
                assert pose is not None and path is not None
                self._step(pose, path)
            elapsed = time.perf_counter() - started
            self._stop_event.wait(max(0.0, period - elapsed))

    def _step(self, pose: PoseStamped, path: Path) -> None:
        assert self._controller is not None
        xy = (pose.position.x, pose.position.y)
        if self._latch.arrive(xy):
            self.nav_cmd_vel.publish(Twist())
            self.goal_reached.publish(Bool(True))
            logger.info("Goal reached")
            return
        if self._latch.reached:
            self.nav_cmd_vel.publish(Twist())
            self._stall.blocked("a new goal -- the last one is reached and latched")
            return
        tw = self._controller.update(pose, path, time.monotonic(), self._clearance_for(path, pose))
        self.nav_cmd_vel.publish(tw)

        # Standing still with a plan in hand is the ambiguous case, and the two
        # causes want opposite fixes: a one-pose plan is the PLANNER refusing
        # (look upstream -- map, clearance, goal), while a real plan the law
        # answers with ~zero is the FOLLOWER's own governor or gait envelope.
        speed = math.hypot(tw.linear.x, tw.linear.y)
        if speed <= self.config.idle_speed and abs(tw.angular.z) <= self.config.idle_speed:
            if len(path.poses) < 2:
                self._stall.blocked(
                    "the planner: it published a single-pose stub, i.e. no safe route"
                )
            else:
                self._stall.blocked(
                    f"nothing -- the law commands ~0 on a {len(path.poses)}-waypoint plan "
                    f"(track={self.config.track}); suspect the speed governor or the gait envelope"
                )
        else:
            self._stall.ok(f"driving: |v|={speed:.2f} m/s wz={tw.angular.z:+.2f} rad/s")

    def _clearance_for(self, path: Path, pose: PoseStamped) -> np.ndarray | None:
        if not self._track.annotate_clearance:
            # the blind track: the law reads the path's own stamps instead
            return None
        with self._lock:
            cloud = self._cloud
        if cloud is None:
            # no local map: fall back to the precision the planner stamped
            # into the path's own timestamps (control/profile.py dialect).
            # That is the hinted track running blind in all but name, and the
            # twist it commands looks healthy either way, so it is said out
            # loud once per outage. No ceilings is the worse case: the law gets
            # no room at all and drives on the governor's floor.
            ceilings = decode_ceilings(path, self._emb.min_speed, self._emb.max_speed)
            if not self._blind:
                self._blind = True
                logger.warning(
                    "no local_map on the hinted track: driving on the path's stamped precision",
                    stamped=ceilings is not None,
                )
            return ceilings_to_clearance(ceilings, self._emb) if ceilings is not None else None
        if self._blind:
            self._blind = False
            logger.info("local_map is back, the room hint is measured again")
        if path is not self._clearance_path or cloud is not self._clearance_cloud:
            wp = np.array([[p.position.x, p.position.y] for p in path.poses]).reshape(-1, 2)
            # Re-referenced per (path, map) pair like the hint itself: the
            # surface under the robot moves far slower than the pair it is
            # cached with.
            ground_z = pose.position.z - self._emb.base_height
            pts = hard_points(self._model, cloud.points_f32(), ground_z)
            self._clearance = path_clearance(wp, pts, self._half_width)
            self._clearance_path, self._clearance_cloud = path, cloud
        return self._clearance
