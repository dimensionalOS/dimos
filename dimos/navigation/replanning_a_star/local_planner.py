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

import os
from threading import Event, RLock, Thread
import time
import traceback
from typing import Literal, TypeAlias

import numpy as np
from numpy.typing import NDArray
from reactivex import Subject

from dimos.core.global_config import GlobalConfig
from dimos.core.resource import Resource
from dimos.mapping.occupancy.visualizations import visualize_occupancy_grid
from dimos.mapping.occupancy.visualize_path import visualize_path
from dimos.msgs.geometry_msgs import Twist, Vector3
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs import Path
from dimos.msgs.sensor_msgs import Image
from dimos.navigation.base import NavigationState
from dimos.navigation.replanning_a_star.navigation_map import NavigationMap
from dimos.navigation.replanning_a_star.path_clearance import PathClearance
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import normalize_angle, quaternion_to_euler

PlannerState: TypeAlias = Literal["idle", "initial_rotation", "path_following", "final_rotation"]
StopMessage: TypeAlias = Literal["arrived", "obstacle_found", "error"]

logger = setup_logger()


class LocalPlanner(Resource):
    cmd_vel: Subject[Twist]
    stopped_navigating: Subject[StopMessage]
    debug_navigation: Subject[Image]

    _thread: Thread | None = None
    _path: Path | None = None
    _np_path: NDArray[np.float64] | None = None
    _path_clearance: PathClearance | None
    _current_odom: PoseStamped | None = None
    _pose_index: int
    _lock: RLock
    _stop_planning_event: Event
    _state: PlannerState
    _global_config: GlobalConfig
    _navigation_map: NavigationMap

    _speed: float = 1.0
    _k_angular: float = 2.0
    _lookahead_dist: float = 0.5
    _goal_tolerance: float = 0.3
    _orientation_tolerance: float = 0.2
    _control_frequency: float = 20.0

    def __init__(self, global_config: GlobalConfig, navigation_map: NavigationMap) -> None:
        self.cmd_vel = Subject()
        self.stopped_navigating = Subject()
        self.debug_navigation = Subject()
        self._pose_index = 0
        self._lock = RLock()
        self._stop_planning_event = Event()
        self._state = "idle"
        self._global_config = global_config
        self._navigation_map = navigation_map

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
            self._path = path
            self._np_path = np.array([[p.position.x, p.position.y] for p in self._path.poses])
            self._path_clearance = PathClearance(self._global_config, self._path)
            self._pose_index = 0
            self._thread = Thread(target=self._thread_entrypoint, daemon=True)
            self._thread.start()

    def _reset_state(self) -> None:
        with self._lock:
            self._state = "idle"
            self._path = None
            self._np_path = None
            self._path_clearance = None
            self._pose_index = 0

    def stop_planning(self) -> None:
        self.cmd_vel.on_next(Twist())
        self._stop_planning_event.set()

        with self._lock:
            if self._thread is not None:
                self._thread.join(2)
                self._thread = None

        self._reset_state()

    def _make_debug_navigation_image(self, path: Path) -> Image:
        scale = 8
        image = visualize_path(
            self._navigation_map.gradient_costmap,
            path,
            self._global_config.robot_width,
            self._global_config.robot_rotation_diameter,
            2,
            scale,
        )
        image.data = np.flipud(image.data)
        if self._path_clearance is not None:
            mask = self._path_clearance.mask
            scaled_mask = np.repeat(np.repeat(mask, scale, axis=0), scale, axis=1)
            scaled_mask = np.flipud(scaled_mask)
            white = np.array([255, 255, 255], dtype=np.int16)
            image.data[scaled_mask] = (image.data[scaled_mask].astype(np.int16) * 3 + white * 7) // 10
        if self._current_odom is not None:
            grid_pos = self._navigation_map.gradient_costmap.world_to_grid(
                self._current_odom.position
            )
            x = int(grid_pos.x * scale)
            y = image.data.shape[0] - 1 - int(grid_pos.y * scale)
            radius = 8
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if dx * dx + dy * dy <= radius * radius:
                        py, px = y + dy, x + dx
                        if 0 <= py < image.data.shape[0] and 0 <= px < image.data.shape[1]:
                            image.data[py, px] = [255, 255, 255]
        return image

    def _thread_entrypoint(self) -> None:
        try:
            self._loop()
        except Exception as e:
            traceback.print_exc()
            logger.exception("Error in local planning", exc_info=e)
            self.stopped_navigating.on_next("error")
        finally:
            self._reset_state()
            self.cmd_vel.on_next(Twist())

    def _loop(self) -> None:
        stop_event = self._stop_planning_event

        with self._lock:
            path = self._path
            path_clearance = self._path_clearance
            self._state = "initial_rotation"

        if path is None or path_clearance is None:
            raise RuntimeError("No path set for local planner.")

        while not stop_event.is_set():
            start_time = time.perf_counter()

            path_clearance.update_costmap(self._navigation_map.binary_costmap)
            path_clearance.update_pose_index(self._pose_index)

            if "DEBUG_NAVIGATION" in os.environ:
                self.debug_navigation.on_next(self._make_debug_navigation_image(path))

            if path_clearance.is_obstacle_ahead():
                self.stopped_navigating.on_next("obstacle_found")
                break

            with self._lock:
                state: PlannerState = self._state

            if state == "initial_rotation":
                cmd_vel = self._compute_initial_rotation()
            elif state == "path_following":
                cmd_vel = self._compute_path_following()
            elif state == "final_rotation":
                cmd_vel = self._compute_final_rotation()
            elif state == "idle":
                self.stopped_navigating.on_next("arrived")
                break

            self.cmd_vel.on_next(cmd_vel)

            elapsed = time.perf_counter() - start_time
            sleep_time = max(0.0, (1.0 / self._control_frequency) - elapsed)
            stop_event.wait(sleep_time)

    def _compute_initial_rotation(self) -> Twist:
        assert self._path is not None
        assert self._current_odom is not None
        first_pose = self._path.poses[0]
        first_yaw = quaternion_to_euler(first_pose.orientation).z
        robot_yaw = self._current_odom.orientation.euler[2]
        yaw_error = normalize_angle(first_yaw - robot_yaw)

        if abs(yaw_error) < self._orientation_tolerance:
            with self._lock:
                self._state = "path_following"
            return self._compute_path_following()

        angular_velocity = np.clip(self._k_angular * yaw_error, -self._speed, self._speed)

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

        if distance_to_goal < self._goal_tolerance:
            logger.info("Reached goal position, starting final rotation")
            self._state = "final_rotation"
            return self._compute_final_rotation()

        closest_idx = self._find_closest_point_idx(current_pose, self._np_path)
        self._pose_index = closest_idx
        lookahead_point = self._find_lookahead_point(self._np_path, closest_idx)

        direction = lookahead_point - current_pose
        distance = np.linalg.norm(direction)

        if distance < 1e-6:
            return Twist()

        robot_yaw = self._current_odom.orientation.euler[2]
        desired_yaw = np.arctan2(direction[1], direction[0])
        yaw_error = normalize_angle(desired_yaw - robot_yaw)

        # Rotate-then-drive: if heading error is large, rotate in place first
        rotation_threshold = 0.5  # ~30 degrees
        if abs(yaw_error) > rotation_threshold:
            angular_velocity = np.clip(self._k_angular * yaw_error, -self._speed, self._speed)
            return Twist(
                linear=Vector3(0.0, 0.0, 0.0),
                angular=Vector3(0.0, 0.0, angular_velocity),
            )

        # When aligned, drive forward with proportional angular correction
        angular_velocity = self._k_angular * yaw_error
        linear_velocity = self._speed * (1.0 - abs(yaw_error) / rotation_threshold)

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

        if abs(yaw_error) < self._orientation_tolerance:
            logger.info("Final rotation complete, goal reached")
            with self._lock:
                self._state = "idle"
            return Twist()

        max_angular_speed = self._speed
        angular_velocity = np.clip(
            self._k_angular * yaw_error, -max_angular_speed, max_angular_speed
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

            if accumulated_dist + segment_dist >= self._lookahead_dist:
                remaining_dist = self._lookahead_dist - accumulated_dist
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
