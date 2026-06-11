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
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path
from dimos.utils.logging_config import setup_logger
from dimos.utils.trigonometry import angle_diff

logger = setup_logger()


class BasicPathFollowerConfig(ModuleConfig):
    speed: float = 0.5
    control_frequency: float = 10.0
    goal_tolerance: float = 0.3
    lookahead_m: float = 0.6
    heading_gain: float = 1.5
    max_angular: float = 1.0
    # Rotate in place instead of advancing when heading error exceeds this.
    rotate_threshold: float = 0.6


class BasicPathFollower(Module):
    """Follow a planned path by chasing a lookahead point with P-controlled heading.

    Consumes world-frame paths and odometry. Publishes nav_cmd_vel until the
    last waypoint is within goal tolerance, then publishes goal_reached.
    An empty path or a stop_movement message cancels the current path.
    """

    config: BasicPathFollowerConfig

    path: In[Path]
    odometry: In[Odometry]
    stop_movement: In[Bool]

    nav_cmd_vel: Out[Twist]
    goal_reached: Out[Bool]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = RLock()
        self._current_odom: PoseStamped | None = None
        self._thread: Thread | None = None
        self._stop_event = Event()

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odometry)))
        self.register_disposable(Disposable(self.path.subscribe(self._on_path)))
        if self.stop_movement.transport is not None:
            self.register_disposable(Disposable(self.stop_movement.subscribe(self._on_stop)))

    @rpc
    def stop(self) -> None:
        self._cancel()
        super().stop()

    def _on_odometry(self, msg: Odometry) -> None:
        with self._lock:
            self._current_odom = msg.to_pose_stamped()

    def _on_path(self, path: Path) -> None:
        self._cancel()
        waypoints = np.array([[p.position.x, p.position.y] for p in path.poses])
        if len(waypoints) == 0:
            return
        logger.info("Following new path", waypoints=len(waypoints))
        stop_event = Event()
        thread = Thread(target=self._follow, args=(waypoints, stop_event), daemon=True)
        with self._lock:
            self._stop_event = stop_event
            self._thread = thread
        thread.start()

    def _on_stop(self, msg: Bool) -> None:
        if msg.data:
            self._cancel()

    def _cancel(self) -> None:
        with self._lock:
            self._stop_event.set()
            self._thread = None
        self.nav_cmd_vel.publish(Twist())

    def _follow(self, waypoints: np.ndarray, stop_event: Event) -> None:
        period = 1.0 / self.config.control_frequency
        while not stop_event.is_set():
            start_time = time.perf_counter()

            with self._lock:
                odom = self._current_odom

            if odom is None:
                stop_event.wait(period)
                continue

            position = np.array([odom.position.x, odom.position.y])
            if float(np.linalg.norm(waypoints[-1] - position)) < self.config.goal_tolerance:
                self.nav_cmd_vel.publish(Twist())
                self.goal_reached.publish(Bool(True))
                logger.info("Goal reached")
                return

            target = self._lookahead_point(waypoints, position)
            yaw_error = angle_diff(
                math.atan2(target[1] - position[1], target[0] - position[0]),
                odom.orientation.euler[2],
            )

            angular = max(
                -self.config.max_angular,
                min(self.config.max_angular, self.config.heading_gain * yaw_error),
            )
            linear = 0.0
            if abs(yaw_error) <= self.config.rotate_threshold:
                linear = self.config.speed * max(0.0, math.cos(yaw_error))

            self.nav_cmd_vel.publish(Twist(Vector3(linear, 0, 0), Vector3(0, 0, angular)))

            elapsed = time.perf_counter() - start_time
            stop_event.wait(max(0.0, period - elapsed))

    def _lookahead_point(self, waypoints: np.ndarray, position: np.ndarray) -> np.ndarray:
        closest = int(np.argmin(np.linalg.norm(waypoints - position, axis=1)))
        travelled = 0.0
        for i in range(closest + 1, len(waypoints)):
            travelled += float(np.linalg.norm(waypoints[i] - waypoints[i - 1]))
            if travelled >= self.config.lookahead_m:
                return np.asarray(waypoints[i])
        return np.asarray(waypoints[-1])
