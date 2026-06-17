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

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
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
    # Lookahead grows with speed (time_s * speed) within these bounds, so the
    # robot tracks tightly when slow and stays stable when fast.
    lookahead_time_s: float = 1.5
    min_lookahead_m: float = 0.4
    max_lookahead_m: float = 1.5
    heading_gain: float = 1.5
    max_angular: float = 1.0


def lookahead_distance(speed: float, time_s: float, lo: float, hi: float) -> float:
    return min(hi, max(lo, time_s * speed))


class BasicPathFollower(Module):
    """Follow a planned path by chasing a lookahead point with P-controlled heading.

    Consumes world-frame paths and odometry. Publishes nav_cmd_vel until the
    last waypoint is within goal tolerance, then publishes goal_reached.
    A stop_movement message cancels the current path. Empty paths are ignored
    so continuous replanning does not stutter the follower.
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
        self._waypoints: np.ndarray | None = None
        self._stop_event = Event()
        self._thread: Thread | None = None
        self._path_count = 0
        self._stats_last = 0.0
        self._last_path_t = 0.0
        self._max_gap = 0.0
        self._cmd_log_last = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(Disposable(self.odometry.subscribe(self._on_odometry)))
        self.register_disposable(Disposable(self.path.subscribe(self._on_path)))
        if self.stop_movement.transport is not None:
            self.register_disposable(Disposable(self.stop_movement.subscribe(self._on_stop)))
        self._stats_last = time.perf_counter()
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
        with self._lock:
            self._current_odom = msg.to_pose_stamped()

    def _on_path(self, path: Path) -> None:
        # The planner clears its plan with an empty path on every start-pose
        # change and on a failed replan. Under continuous replanning that is not
        # a stop, so keep the last path. Stops arrive via stop_movement.
        if len(path.poses) == 0:
            return
        waypoints = np.array([[p.position.x, p.position.y] for p in path.poses])
        now = time.perf_counter()
        with self._lock:
            if self._last_path_t:
                self._max_gap = max(self._max_gap, now - self._last_path_t)
            self._last_path_t = now
            self._waypoints = waypoints
            self._path_count += 1

    def _on_stop(self, msg: Bool) -> None:
        if msg.data:
            with self._lock:
                self._waypoints = None
            self.nav_cmd_vel.publish(Twist())

    def _follow(self) -> None:
        period = 1.0 / self.config.control_frequency
        while not self._stop_event.is_set():
            start_time = time.perf_counter()
            with self._lock:
                odom = self._current_odom
                waypoints = self._waypoints
            if odom is not None and waypoints is not None:
                self._step(odom, waypoints)
            self._maybe_log_stats(odom, waypoints)
            elapsed = time.perf_counter() - start_time
            self._stop_event.wait(max(0.0, period - elapsed))

    # DEBUG: replanning telemetry, remove before merge
    def _maybe_log_stats(self, odom: PoseStamped | None, waypoints: np.ndarray | None) -> None:
        now = time.perf_counter()
        elapsed = now - self._stats_last
        if elapsed < 2.0:
            return
        self._stats_last = now
        with self._lock:
            count = self._path_count
            gap = self._max_gap
            self._path_count = 0
            self._max_gap = 0.0
        if count == 0:
            return
        rate = count / elapsed
        lag = float("nan")
        if odom is not None and waypoints is not None:
            position = np.array([odom.position.x, odom.position.y])
            lag = float(np.linalg.norm(waypoints[0] - position))
        logger.debug(
            "path follower stats",
            replan_hz=round(rate, 1),
            max_gap_ms=round(gap * 1000),
            path_lag_m=round(lag, 2),
        )

    def _step(self, odom: PoseStamped, waypoints: np.ndarray) -> None:
        position = np.array([odom.position.x, odom.position.y])
        if float(np.linalg.norm(waypoints[-1] - position)) < self.config.goal_tolerance:
            self.nav_cmd_vel.publish(Twist())
            with self._lock:
                if self._waypoints is waypoints:
                    self._waypoints = None
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
        # Taper forward speed with heading error instead of a hard stop: the
        # robot decelerates into a turn and pivots in place only near 90 degrees
        # (cos -> 0). Slower speed at sharp corners means a tighter turn radius,
        # so it tracks the safe path instead of stopping and lurching.
        linear = self.config.speed * max(0.0, math.cos(yaw_error))

        # DEBUG: command trace, remove before merge
        now = time.perf_counter()
        if now - self._cmd_log_last >= 0.2:
            self._cmd_log_last = now
            logger.debug(
                "follower cmd",
                lin=round(linear, 3),
                ang=round(angular, 3),
                yaw_err_deg=round(math.degrees(yaw_error), 1),
                target_dx=round(float(target[0] - position[0]), 2),
                target_dy=round(float(target[1] - position[1]), 2),
                goal_dist=round(float(np.linalg.norm(waypoints[-1] - position)), 2),
            )

        self.nav_cmd_vel.publish(Twist(Vector3(linear, 0, 0), Vector3(0, 0, angular)))

    def _lookahead_point(self, waypoints: np.ndarray, position: np.ndarray) -> np.ndarray:
        if len(waypoints) == 1:
            return np.asarray(waypoints[0])

        # Project onto the path, then march the lookahead along it and
        # interpolate. Returning an actual waypoint would let the target jump to
        # a distant vertex and cut the corner.
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
        self, waypoints: np.ndarray, position: np.ndarray
    ) -> tuple[int, np.ndarray]:
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
