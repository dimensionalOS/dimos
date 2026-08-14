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

import math
from threading import Event, RLock, Thread
import time
from typing import Any

from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]
import numpy as np
from numpy.typing import NDArray
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.tf_pose import OdomBasePose
from dimos.utils.logging_config import setup_logger
from dimos.utils.trigonometry import angle_diff

logger = setup_logger()


class BasicPathFollowerConfig(ModuleConfig):
    base_frame: str = "base_link"
    speed: float = 0.5
    min_linear_speed: float = 0.0
    slowdown_distance: float = 0.0
    control_frequency: float = 10.0
    goal_tolerance: float = 0.3
    orientation_tolerance: float = math.radians(10.0)
    align_goal_yaw: bool = False
    rotate_before_drive: bool = False
    drive_heading_tolerance: float = math.radians(20.0)
    min_angular: float = 0.0
    pose_timeout: float = 0.5
    # Lookahead grows with speed (time_s * speed), clamped to these bounds.
    lookahead_time_s: float = 1.5
    min_lookahead_m: float = 0.4
    max_lookahead_m: float = 1.5
    heading_gain: float = 1.5
    max_angular: float = 1.0


def lookahead_distance(speed: float, time_s: float, lo: float, hi: float) -> float:
    return min(hi, max(lo, time_s * speed))


class BasicPathFollower(Module):
    """Follow a planned path by chasing a lookahead point with P-controlled heading.

    Publishes nav_cmd_vel until the last waypoint is within goal tolerance, then
    goal_reached. stop_movement cancels the current path.
    """

    config: BasicPathFollowerConfig

    path: In[Path]
    odometry: In[Odometry]
    base_pose: In[PoseStamped]
    stop_movement: In[Bool]
    tf: In[TFMessage]

    nav_cmd_vel: Out[Twist]
    goal_reached: Out[Bool]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = RLock()
        self._base_pose: OdomBasePose | None = None
        self._current_pose: PoseStamped | None = None
        self._pose_received_at: float | None = None
        self._waypoints: NDArray[np.float32] | None = None
        self._goal_yaw: float | None = None
        self._pose_stale_stopped = False
        self._stop_event = Event()
        self._thread: Thread | None = None

    @rpc
    def start(self) -> None:
        super().start()
        if self.odometry.transport is not None:
            self.register_disposable(Disposable(self.odometry.subscribe(self._on_odometry)))
        if self.base_pose.transport is not None:
            self.register_disposable(Disposable(self.base_pose.subscribe(self._on_base_pose)))
        self.register_disposable(Disposable(self.path.subscribe(self._on_path)))
        if self.stop_movement.transport is not None:
            self.register_disposable(Disposable(self.stop_movement.subscribe(self._on_stop)))
        self._thread = Thread(target=self._follow, daemon=True)
        self._thread.start()

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
        self.nav_cmd_vel.publish(Twist())
        super().stop()

    def _on_odometry(self, msg: Odometry) -> None:
        if self._base_pose is None:
            self._base_pose = OdomBasePose(self.tfbuffer, self.config.base_frame)
        pose = self._base_pose.resolve(msg)
        if pose is None:
            return
        with self._lock:
            self._current_pose = pose
            self._pose_received_at = time.monotonic()
            self._pose_stale_stopped = False

    def _on_base_pose(self, pose: PoseStamped) -> None:
        """Accept an already-resolved world-to-base pose, such as Point-LIO."""
        with self._lock:
            self._current_pose = PoseStamped(
                ts=pose.ts,
                frame_id=pose.frame_id,
                position=pose.position,
                orientation=pose.orientation,
            )
            self._pose_received_at = time.monotonic()
            self._pose_stale_stopped = False

    def _on_path(self, path: Path) -> None:
        # The planner owns path safety: it sends the route as far as it is safe,
        # or an empty path when nothing ahead is traversable. Follow what we get.
        if len(path.poses) == 0:
            with self._lock:
                self._waypoints = None
                self._goal_yaw = None
            self.nav_cmd_vel.publish(Twist())
            return
        waypoints = np.array([[p.position.x, p.position.y] for p in path.poses], dtype=np.float32)
        with self._lock:
            self._waypoints = waypoints
            self._goal_yaw = float(path.poses[-1].orientation.to_euler().z)

    def _on_stop(self, msg: Bool) -> None:
        if msg.data:
            with self._lock:
                self._waypoints = None
                self._goal_yaw = None
            self.nav_cmd_vel.publish(Twist())

    def _follow(self) -> None:
        period = 1.0 / self.config.control_frequency
        while not self._stop_event.is_set():
            start_time = time.perf_counter()
            now = time.monotonic()
            with self._lock:
                pose = self._current_pose
                waypoints = self._waypoints
                goal_yaw = self._goal_yaw
                pose_received_at = self._pose_received_at
                pose_stale_stopped = self._pose_stale_stopped
            pose_is_fresh = (
                pose_received_at is not None and now - pose_received_at <= self.config.pose_timeout
            )
            if pose is not None and waypoints is not None and pose_is_fresh:
                self._step(pose, waypoints, goal_yaw)
            elif waypoints is not None and not pose_is_fresh and not pose_stale_stopped:
                self.nav_cmd_vel.publish(Twist())
                with self._lock:
                    self._pose_stale_stopped = True
                logger.warning("Base pose is stale; stopping path follower")
            elapsed = time.perf_counter() - start_time
            self._stop_event.wait(max(0.0, period - elapsed))

    def _step(
        self,
        pose: PoseStamped,
        waypoints: NDArray[np.float32],
        goal_yaw: float | None,
    ) -> None:
        position = np.array([pose.position.x, pose.position.y], dtype=np.float32)
        distance_to_goal = float(np.linalg.norm(waypoints[-1] - position))
        if distance_to_goal < self.config.goal_tolerance:
            if self.config.align_goal_yaw and goal_yaw is not None:
                yaw_error = angle_diff(goal_yaw, pose.orientation.euler[2])
                if abs(yaw_error) >= self.config.orientation_tolerance:
                    self.nav_cmd_vel.publish(
                        Twist(
                            Vector3(),
                            Vector3(0.0, 0.0, self._angular_command(yaw_error)),
                        )
                    )
                    return
            self.nav_cmd_vel.publish(Twist())
            with self._lock:
                if self._waypoints is waypoints:
                    self._waypoints = None
                    self._goal_yaw = None
            self.goal_reached.publish(Bool(True))
            logger.info("Goal reached")
            return

        target = self._lookahead_point(waypoints, position)
        yaw_error = angle_diff(
            math.atan2(target[1] - position[1], target[0] - position[0]),
            pose.orientation.euler[2],
        )

        angular = self._angular_command(yaw_error)
        if self.config.rotate_before_drive and abs(yaw_error) > self.config.drive_heading_tolerance:
            self.nav_cmd_vel.publish(Twist(Vector3(), Vector3(0.0, 0.0, angular)))
            return

        target_speed = self.config.speed
        if self.config.slowdown_distance > 0.0:
            target_speed = self.config.speed * min(
                1.0,
                distance_to_goal / self.config.slowdown_distance,
            )
        linear = target_speed * max(0.0, math.cos(yaw_error))
        if linear > 0.0:
            linear = max(self.config.min_linear_speed, linear)
            linear = min(self.config.speed, linear)
        self.nav_cmd_vel.publish(Twist(Vector3(linear, 0, 0), Vector3(0, 0, angular)))

    def _angular_command(self, yaw_error: float) -> float:
        angular = max(
            -self.config.max_angular,
            min(self.config.max_angular, self.config.heading_gain * yaw_error),
        )
        if abs(yaw_error) > 0.0 and abs(angular) < self.config.min_angular:
            angular = math.copysign(self.config.min_angular, yaw_error)
        return angular

    def _lookahead_point(
        self, waypoints: NDArray[np.float32], position: NDArray[np.float32]
    ) -> NDArray[np.float32]:
        if len(waypoints) == 1:
            return np.asarray(waypoints[0])

        # Interpolate along the path rather than snap to a waypoint, which would
        # cut corners.
        seg_idx, start = self._project_onto_path(waypoints, position)
        remaining = lookahead_distance(
            self.config.speed,
            self.config.lookahead_time_s,
            self.config.min_lookahead_m,
            self.config.max_lookahead_m,
        )
        for i in range(seg_idx, len(waypoints) - 1):
            end = waypoints[i + 1]
            seg = end - start
            seg_len = float(np.linalg.norm(seg))
            if seg_len >= remaining:
                return np.asarray(start + (remaining / seg_len) * seg)
            remaining -= seg_len
            start = end
        return np.asarray(waypoints[-1])

    def _project_onto_path(
        self, waypoints: NDArray[np.float32], position: NDArray[np.float32]
    ) -> tuple[int, NDArray[np.float32]]:
        best_idx = 0
        best_point = np.asarray(waypoints[0])
        best_dist = math.inf
        for i in range(len(waypoints) - 1):
            a = waypoints[i]
            ab = waypoints[i + 1] - a
            denom = float(np.dot(ab, ab))
            t = 0.0 if denom == 0 else float(np.clip(np.dot(position - a, ab) / denom, 0.0, 1.0))
            proj = a + t * ab
            dist = float(np.linalg.norm(position - proj))
            if dist < best_dist:
                best_dist = dist
                best_idx = i
                best_point = proj
        return best_idx, best_point
