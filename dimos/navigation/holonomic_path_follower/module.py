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

"""Stream boundary for the canonical holonomic full-pose controller task."""

from __future__ import annotations

import math
from threading import Event, RLock, Thread
import time
from typing import Any

from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.control.task import CoordinatorState, JointStateSnapshot
from dimos.control.tasks.holonomic_pose_follower_task.holonomic_pose_follower_task import (
    HolonomicPoseFollowerTask,
    HolonomicPoseFollowerTaskConfig,
)
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_BASE_JOINTS = ["base/vx", "base/vy", "base/wz"]


class HolonomicPathFollowerConfig(ModuleConfig):
    speed: float = 0.18
    control_frequency: float = 10.0
    lookahead: float = 0.25
    regulate_horizon: float = 0.6
    goal_tolerance: float = 0.06
    orientation_tolerance: float = math.radians(5.0)
    approach_decel: float = 0.5
    stop_hold_s: float = 0.5
    pose_timeout: float = 0.5
    max_yaw_rate: float = 0.18
    min_linear_speed: float = 0.10
    min_angular_speed: float = 0.08
    position_gain: float = 0.7
    yaw_gain: float = 0.5


class HolonomicPathFollower(Module):
    """Convert a full-pose ``Path`` and world pose into an autonomy ``Twist``.

    The underlying controller tracks translation and commanded yaw together.
    This module owns timing and stale-pose stopping; ControlCoordinator remains
    the sole authority that arbitrates autonomy against operator commands.
    """

    config: HolonomicPathFollowerConfig

    path: In[Path]
    base_pose: In[PoseStamped]
    stop_movement: In[Bool]

    nav_cmd_vel: Out[Twist]
    goal_reached: Out[Bool]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._controller = HolonomicPoseFollowerTask(
            "holonomic_pose_follower",
            HolonomicPoseFollowerTaskConfig(
                joint_names=_BASE_JOINTS,
                speed=self.config.speed,
                lookahead=self.config.lookahead,
                regulate_horizon=self.config.regulate_horizon,
                goal_tolerance=self.config.goal_tolerance,
                orientation_tolerance=self.config.orientation_tolerance,
                approach_decel=self.config.approach_decel,
                stop_hold_s=self.config.stop_hold_s,
                artifact_path="",
                stale_pose_timeout=self.config.pose_timeout,
                max_linear_speed=self.config.speed,
                max_yaw_rate=self.config.max_yaw_rate,
                min_linear_speed=self.config.min_linear_speed,
                min_angular_speed=self.config.min_angular_speed,
                position_gain=self.config.position_gain,
                yaw_gain=self.config.yaw_gain,
            ),
        )
        self._lock = RLock()
        self._current_pose: PoseStamped | None = None
        self._pose_received_at: float | None = None
        self._pending_path: Path | None = None
        self._pose_stale_stopped = False
        self._goal_signaled = False
        self._stop_event = Event()
        self._thread: Thread | None = None
        self._last_control_at: float | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.base_pose.subscribe(self._on_base_pose)))
        self.register_disposable(Disposable(self.path.subscribe(self._on_path)))
        if self.stop_movement.transport is not None:
            self.register_disposable(Disposable(self.stop_movement.subscribe(self._on_stop)))
        self._thread = Thread(target=self._follow, name="holonomic-path-follower", daemon=True)
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
            self._last_control_at = None
        self.nav_cmd_vel.publish(Twist())
        if len(path.poses) < 2:
            logger.warning("Holonomic path follower rejected a path with fewer than two poses")

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
                yaw = float(pose.orientation.euler[2])
                last = self._last_control_at
                state = CoordinatorState(
                    joints=JointStateSnapshot(
                        joint_positions={
                            _BASE_JOINTS[0]: float(pose.position.x),
                            _BASE_JOINTS[1]: float(pose.position.y),
                            _BASE_JOINTS[2]: yaw,
                        }
                    ),
                    t_now=now,
                    dt=0.0 if last is None else now - last,
                )
                output = self._controller.compute(state)
                self._last_control_at = now
                if output is not None and output.velocities is not None:
                    vx, vy, wz = output.velocities
                    command = Twist(
                        linear=Vector3(vx, vy, 0.0),
                        angular=Vector3(0.0, 0.0, wz),
                    )
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
            logger.warning("Base pose is stale; stopping holonomic path follower")
        if reached:
            self.goal_reached.publish(Bool(True))
            logger.info("Holonomic path follower reached its goal")
