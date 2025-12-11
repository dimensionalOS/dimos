# Copyright 2025 Dimensional Inc.
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

from threading import Event, RLock, Thread
import time
from typing import Literal, TypeAlias

import numpy as np
from numpy.typing import NDArray
from reactivex import Subject

from dimos.core.resource import Resource
from dimos.msgs.geometry_msgs import Twist, Vector3
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs import Path
from dimos.navigation.base import NavigationState
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import normalize_angle, quaternion_to_euler

logger = setup_logger()

PlannerState: TypeAlias = Literal["idle", "initial_rotation", "path_following", "final_rotation"]


class LocalPlanner(Resource):
    cmd_vel: Subject[Twist]

    _thread: Thread | None = None
    _path: Path | None = None
    _np_path: NDArray[np.float64] | None = None
    _current_odom: PoseStamped | None = None
    _goal_reached: bool = False
    _lock: RLock
    _stop_planning_event: Event
    _state: PlannerState

    speed: float = 1.0
    k_angular: float = 2.0
    lookahead_dist: float = 0.5
    goal_tolerance: float = 0.3
    orientation_tolerance: float = 0.2
    control_frequency: float = 20.0

    def __init__(self) -> None:
        self.cmd_vel = Subject()
        self._lock = RLock()
        self._stop_planning_event = Event()
        self._state = "idle"

    def start(self) -> None:
        pass

    def stop(self) -> None:
        self.stop_planning()

    def handle_odom(self, msg: PoseStamped) -> None:
        self._current_odom = msg

    def start_planning(self, path: Path) -> None:
        self.stop_planning()

        self._stop_planning_event = Event()

        with self._lock:
            self._goal_reached = False
            self._path = path
            self._np_path = np.array([[p.position.x, p.position.y] for p in self._path.poses])
            self._thread = Thread(target=self._loop, daemon=True)
            self._thread.start()

    def stop_planning(self) -> None:
        self._stop_planning_event.set()

        with self._lock:
            if self._thread is not None:
                self._thread.join(2)
                self._thread = None

    def _loop(self) -> None:
        stop_event = self._stop_planning_event

        with self._lock:
            path = self._path
            self._state = "initial_rotation"

        if path is None:
            raise RuntimeError("No path set for local planner.")

        while not stop_event.is_set():
            start_time = time.perf_counter()

            with self._lock:
                state: PlannerState = self._state

            if state == "initial_rotation":
                cmd_vel = self._compute_initial_rotation()
            elif state == "path_following":
                cmd_vel = self._compute_path_following()
            elif state == "final_rotation":
                cmd_vel = self._compute_final_rotation()
            elif state == "idle":
                with self._lock:
                    self._goal_reached = True
                break

            self.cmd_vel.on_next(cmd_vel)

            elapsed = time.perf_counter() - start_time
            sleep_time = max(0.0, (1.0 / self.control_frequency) - elapsed)
            stop_event.wait(sleep_time)

        self.cmd_vel.on_next(Twist())

        with self._lock:
            self._state = "idle"

    def _compute_initial_rotation(self) -> Twist:
        assert self._path is not None
        assert self._current_odom is not None
        first_pose = self._path.poses[0]
        first_yaw = quaternion_to_euler(first_pose.orientation).z
        robot_yaw = self._current_odom.orientation.euler[2]
        yaw_error = normalize_angle(first_yaw - robot_yaw)

        if abs(yaw_error) < self.orientation_tolerance:
            with self._lock:
                self._state = "path_following"
            return self._compute_path_following()

        max_angular_speed = self.speed
        angular_velocity = np.clip(
            self.k_angular * yaw_error, -max_angular_speed, max_angular_speed
        )

        return Twist(
            linear=Vector3(0.0, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, angular_velocity),
        )

    def _compute_path_following(self) -> Twist:
        assert self._np_path is not None
        assert self._current_odom is not None
        current_pose = np.array([self._current_odom.position.x, self._current_odom.position.y])

        # Check if we've reached the final position
        goal_position = self._np_path[-1]
        distance_to_goal = np.linalg.norm(goal_position - current_pose)

        if distance_to_goal < self.goal_tolerance:
            logger.info("Reached goal position, starting final rotation")
            self._state = "final_rotation"
            return self._compute_final_rotation()

        closest_idx = self._find_closest_point_idx(current_pose, self._np_path)
        lookahead_point = self._find_lookahead_point(self._np_path, closest_idx)

        direction = lookahead_point - current_pose
        distance = np.linalg.norm(direction)

        if distance < 1e-6:
            return Twist()

        robot_yaw = self._current_odom.orientation.euler[2]
        desired_yaw = np.arctan2(direction[1], direction[0])
        yaw_error = normalize_angle(desired_yaw - robot_yaw)

        max_angular_speed = 0.5
        angular_velocity = np.clip(
            self.k_angular * yaw_error, -max_angular_speed, max_angular_speed
        )

        # Linear velocity - reduce speed when turning
        angular_slowdown = 1.0 - 0.7 * min(abs(angular_velocity) / max_angular_speed, 1.0)
        linear_velocity = self.speed * angular_slowdown

        return Twist(
            linear=Vector3(linear_velocity, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, angular_velocity),
        )

    def _compute_final_rotation(self) -> Twist:
        assert self._path is not None
        assert self._current_odom is not None
        last_pose = self._path.poses[-1]
        goal_yaw = quaternion_to_euler(last_pose.orientation).z
        robot_yaw = self._current_odom.orientation.euler[2]
        yaw_error = normalize_angle(goal_yaw - robot_yaw)

        if abs(yaw_error) < self.orientation_tolerance:
            logger.info("Final rotation complete, goal reached")
            with self._lock:
                self._state = "idle"
            return Twist()

        max_angular_speed = self.speed
        angular_velocity = np.clip(
            self.k_angular * yaw_error, -max_angular_speed, max_angular_speed
        )

        return Twist(
            linear=Vector3(0.0, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, angular_velocity),
        )

    def _find_closest_point_idx(self, pose: NDArray[np.float64], path: NDArray[np.float64]) -> int:
        """Find the index of the closest point on the path."""
        distances = np.linalg.norm(path - pose, axis=1)
        return int(np.argmin(distances))

    def _find_lookahead_point(
        self, path: NDArray[np.float64], start_idx: int
    ) -> NDArray[np.float64]:
        """Find a lookahead point on the path at the specified distance."""
        accumulated_dist: float = 0.0

        for i in range(start_idx, len(path) - 1):
            segment_dist = float(np.linalg.norm(path[i + 1] - path[i]))

            if accumulated_dist + segment_dist >= self.lookahead_dist:
                remaining_dist = self.lookahead_dist - accumulated_dist
                if segment_dist > 0:
                    t = remaining_dist / segment_dist
                    result: NDArray[np.float64] = path[i] + t * (path[i + 1] - path[i])
                    return result
                result_i: NDArray[np.float64] = path[i]
                return result_i

            accumulated_dist += segment_dist

        # Return the last point if we've traversed the whole path
        result_last: NDArray[np.float64] = path[-1]
        return result_last

    def get_state(self) -> NavigationState:
        with self._lock:
            state = self._state

        match state:
            case "idle":
                return NavigationState.IDLE
            case "initial_rotation" | "path_following" | "final_rotation":
                return NavigationState.FOLLOWING_PATH
            case _:
                raise ValueError(f"Unknown planner state: {state}")

    def is_goal_reached(self) -> bool:
        with self._lock:
            return self._goal_reached
