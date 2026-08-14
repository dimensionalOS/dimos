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

"""Stream boundary for the existing regulated-pure-pursuit controller."""

from __future__ import annotations

import math
from threading import Event, RLock, Thread
import time
from typing import Any

from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.benchmarking.velocity_profile import VelocityProfileConfig
from dimos.control.tasks.path_follower_task.path_follower_task import PathFollowerTaskConfig
from dimos.control.tasks.rpp_path_follower_task.rpp_path_follower_task import RPPPathFollowerTask
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class RPPPathFollowerConfig(ModuleConfig):
    speed: float = 0.55
    control_frequency: float = 10.0
    goal_tolerance: float = 0.2
    orientation_tolerance: float = math.radians(10.0)
    k_angular: float = 1.5
    lookahead_dist: float = 0.5
    lookahead_min: float = 0.3
    lookahead_max: float = 0.9
    lookahead_speed_scale: float = 0.0
    max_yaw_rate: float | None = None
    forward_only: bool = True
    min_linear_speed: float = 0.2
    min_angular_speed: float = 0.2
    rotation_threshold: float = math.pi / 2
    slowdown_distance: float = 0.0
    pose_timeout: float = 0.5
    # Curvature regulation is controller-side and robot independent when these
    # limits are supplied by the robot blueprint. It does not load another
    # robot's feed-forward plant model.
    curvature_regulation: bool = True
    max_centripetal_accel: float = 1.0
    max_linear_accel: float = 1.0
    max_linear_decel: float = 2.0
    profile_lookahead_pts: int = 8
    # Position-only planners need tangent headings synthesized. Proper pose
    # paths with a distinct final orientation are detected and left untouched.
    synthesize_tangent_headings: bool = True


class RPPPathFollower(Module):
    """Convert ``Path + PoseStamped`` into autonomy ``Twist`` commands.

    The module owns controller timing and pose freshness only. Its output is an
    input to the ControlCoordinator, which remains the sole command arbiter.
    """

    config: RPPPathFollowerConfig

    path: In[Path]
    base_pose: In[PoseStamped]
    stop_movement: In[Bool]

    nav_cmd_vel: Out[Twist]
    goal_reached: Out[Bool]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        profile = None
        if self.config.curvature_regulation:
            profile = VelocityProfileConfig(
                max_linear_speed=self.config.speed,
                max_angular_speed=(
                    self.config.max_yaw_rate
                    if self.config.max_yaw_rate is not None
                    else self.config.speed
                ),
                max_centripetal_accel=self.config.max_centripetal_accel,
                max_linear_accel=self.config.max_linear_accel,
                max_linear_decel=self.config.max_linear_decel,
                min_speed=self.config.min_linear_speed,
                lookahead_pts=self.config.profile_lookahead_pts,
            )
        self._controller = RPPPathFollowerTask(
            "rpp_path_follower",
            PathFollowerTaskConfig(
                speed=self.config.speed,
                control_frequency=self.config.control_frequency,
                goal_tolerance=self.config.goal_tolerance,
                orientation_tolerance=self.config.orientation_tolerance,
                k_angular=self.config.k_angular,
                lookahead_dist=self.config.lookahead_dist,
                lookahead_min=self.config.lookahead_min,
                lookahead_max=self.config.lookahead_max,
                lookahead_speed_scale=self.config.lookahead_speed_scale,
                max_yaw_rate=self.config.max_yaw_rate,
                forward_only=self.config.forward_only,
                min_linear_speed=self.config.min_linear_speed,
                min_angular_speed=self.config.min_angular_speed,
                rotation_threshold=self.config.rotation_threshold,
                slowdown_distance=self.config.slowdown_distance,
                velocity_profile_config=profile,
            ),
            global_config=self.config.g,
            artifact_path="",
            use_artifact_feedforward=False,
            use_artifact_velocity_profile=False,
            use_artifact_yaw_limit=False,
            synthesize_tangent_headings=self.config.synthesize_tangent_headings,
        )
        self._lock = RLock()
        self._current_pose: PoseStamped | None = None
        self._pose_received_at: float | None = None
        self._pending_path: Path | None = None
        self._pose_stale_stopped = False
        self._goal_signaled = False
        self._stop_event = Event()
        self._thread: Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.base_pose.subscribe(self._on_base_pose)))
        self.register_disposable(Disposable(self.path.subscribe(self._on_path)))
        if self.stop_movement.transport is not None:
            self.register_disposable(Disposable(self.stop_movement.subscribe(self._on_stop)))
        self._thread = Thread(target=self._follow, name="rpp-path-follower", daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self.nav_cmd_vel.publish(Twist())
        super().stop()

    def _on_base_pose(self, pose: PoseStamped) -> None:
        with self._lock:
            self._current_pose = pose
            self._pose_received_at = time.monotonic()
            self._pose_stale_stopped = False

    def _on_path(self, path: Path) -> None:
        with self._lock:
            self._clear_controller_locked()
            self._pending_path = path if len(path.poses) >= 2 else None
            self._goal_signaled = False
        self.nav_cmd_vel.publish(Twist())
        if len(path.poses) < 2:
            logger.warning("RPP path follower rejected a path with fewer than two poses")

    def _on_stop(self, msg: Bool) -> None:
        if not msg.data:
            return
        with self._lock:
            self._pending_path = None
            self._clear_controller_locked()
        self.nav_cmd_vel.publish(Twist())

    def _clear_controller_locked(self) -> None:
        if self._controller.is_active():
            self._controller.cancel()
        self._controller.reset()

    def _follow(self) -> None:
        period = 1.0 / self.config.control_frequency
        while not self._stop_event.is_set():
            started_at = time.perf_counter()
            self._control_once(time.monotonic())
            elapsed = time.perf_counter() - started_at
            self._stop_event.wait(max(0.0, period - elapsed))

    def _control_once(self, now: float) -> None:
        command: Twist | None = None
        reached = False
        stale_stop = False
        with self._lock:
            pose = self._current_pose
            pose_is_fresh = (
                pose is not None
                and self._pose_received_at is not None
                and now - self._pose_received_at <= self.config.pose_timeout
            )
            if self._pending_path is not None and pose_is_fresh:
                path, self._pending_path = self._pending_path, None
                assert pose is not None
                self._controller.start_path(path, pose)

            if self._controller.is_active() and pose_is_fresh:
                assert pose is not None
                command = self._controller.compute_twist(pose)
                if self._controller.get_state() == "arrived" and not self._goal_signaled:
                    self._goal_signaled = True
                    reached = True
            elif self._controller.is_active() and not self._pose_stale_stopped:
                self._pose_stale_stopped = True
                stale_stop = True

        if command is not None:
            self.nav_cmd_vel.publish(command)
        elif stale_stop:
            self.nav_cmd_vel.publish(Twist())
            logger.warning("Base pose is stale; stopping RPP path follower")
        if reached:
            self.goal_reached.publish(Bool(True))
            logger.info("RPP path follower reached its goal")
