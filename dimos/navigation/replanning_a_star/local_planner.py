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

import math
import os
from threading import Event, RLock, Thread
import time
import traceback
from typing import Literal, TypeAlias

import numpy as np
from reactivex import Subject

from dimos.core.global_config import GlobalConfig
from dimos.core.resource import Resource
from dimos.mapping.occupancy.visualize_path import visualize_path
from dimos.msgs.geometry_msgs import Twist, Vector3
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs import Path
from dimos.msgs.sensor_msgs import Image
from dimos.navigation.base import NavigationState
from dimos.navigation.replanning_a_star.navigation_map import NavigationMap
from dimos.navigation.replanning_a_star.path_clearance import PathClearance
from dimos.navigation.replanning_a_star.path_distancer import PathDistancer
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import normalize_angle, quaternion_to_euler

PlannerState: TypeAlias = Literal[
    "idle", "initial_rotation", "path_following", "final_rotation", "arrived"
]
StopMessage: TypeAlias = Literal["arrived", "obstacle_found", "error"]

logger = setup_logger()


class LocalPlanner(Resource):
    cmd_vel: Subject[Twist]
    stopped_navigating: Subject[StopMessage]
    debug_navigation: Subject[Image]

    _thread: Thread | None = None
    _path: Path | None = None
    _path_clearance: PathClearance | None = None
    _path_distancer: PathDistancer | None = None
    _current_odom: PoseStamped | None = None

    _pose_index: int
    _lock: RLock
    _stop_planning_event: Event
    _state: PlannerState
    _state_unique_id: int
    _global_config: GlobalConfig
    _navigation_map: NavigationMap

    _speed: float = 0.55
    _min_linear_velocity: float = 0.2
    _min_angular_velocity: float = 0.2
    _k_angular: float = 0.5
    _k_derivative: float = 0.15
    _max_angular_accel: float = 2.0
    _goal_tolerance: float = 0.3
    _orientation_tolerance: float = 0.35
    _control_frequency: float = 10.0
    _rotation_threshold: float = 90

    _prev_yaw_error: float
    _prev_angular_velocity: float

    def __init__(self, global_config: GlobalConfig, navigation_map: NavigationMap) -> None:
        self.cmd_vel = Subject()
        self.stopped_navigating = Subject()
        self.debug_navigation = Subject()

        self._pose_index = 0
        self._lock = RLock()
        self._stop_planning_event = Event()
        self._state = "idle"
        self._state_unique_id = 0
        self._global_config = global_config
        self._navigation_map = navigation_map

        self._prev_yaw_error = 0.0
        self._prev_angular_velocity = 0.0

    def start(self) -> None:
        pass

    def stop(self) -> None:
        self.stop_planning()

    def handle_odom(self, msg: PoseStamped) -> None:
        with self._lock:
            self._current_odom = msg

    def start_planning(self, path: Path) -> None:
        self.stop_planning()

        self._stop_planning_event = Event()

        with self._lock:
            self._path = path
            self._path_clearance = PathClearance(self._global_config, self._path)
            self._path_distancer = PathDistancer(self._path)
            self._pose_index = 0
            self._thread = Thread(target=self._thread_entrypoint, daemon=True)
            self._thread.start()

    def stop_planning(self) -> None:
        self.cmd_vel.on_next(Twist())
        self._stop_planning_event.set()

        with self._lock:
            self._thread = None

        self._reset_state()

    def get_state(self) -> NavigationState:
        with self._lock:
            state = self._state

        match state:
            case "idle" | "arrived":
                return NavigationState.IDLE
            case "initial_rotation" | "path_following" | "final_rotation":
                return NavigationState.FOLLOWING_PATH
            case _:
                raise ValueError(f"Unknown planner state: {state}")

    def get_unique_state(self) -> tuple[PlannerState, int]:
        with self._lock:
            return (self._state, self._state_unique_id)

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

    def _change_state(self, new_state: PlannerState) -> None:
        self._state = new_state
        self._state_unique_id += 1
        logger.info("changed state", state=new_state)

    def _loop(self) -> None:
        stop_event = self._stop_planning_event

        with self._lock:
            path = self._path
            path_clearance = self._path_clearance
            current_odom = self._current_odom

        if path is None or path_clearance is None:
            raise RuntimeError("No path set for local planner.")

        # Determine initial state: skip initial_rotation if already aligned.
        new_state: PlannerState = "initial_rotation"
        if current_odom is not None and len(path.poses) > 0:
            first_yaw = quaternion_to_euler(path.poses[0].orientation).z
            robot_yaw = current_odom.orientation.euler[2]
            initial_yaw_error = normalize_angle(first_yaw - robot_yaw)
            self._prev_yaw_error = initial_yaw_error
            if abs(initial_yaw_error) < self._orientation_tolerance:
                new_state = "path_following"

        with self._lock:
            self._change_state(new_state)

        while not stop_event.is_set():
            start_time = time.perf_counter()

            with self._lock:
                path_clearance.update_costmap(self._navigation_map.binary_costmap)
                path_clearance.update_pose_index(self._pose_index)

            if "DEBUG_NAVIGATION" in os.environ:
                self.debug_navigation.on_next(
                    self._make_debug_navigation_image(path, path_clearance)
                )

            if path_clearance.is_obstacle_ahead():
                logger.info("Obstacle detected ahead, stopping local planner.")
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
            elif state == "arrived":
                self.stopped_navigating.on_next("arrived")
                break
            elif state == "idle":
                cmd_vel = None

            if cmd_vel is not None:
                self.cmd_vel.on_next(cmd_vel)

            elapsed = time.perf_counter() - start_time
            sleep_time = max(0.0, (1.0 / self._control_frequency) - elapsed)
            stop_event.wait(sleep_time)

        if stop_event.is_set():
            logger.info("Local planner loop exited due to stop event.")

    def _apply_min_velocity(self, velocity: float, min_velocity: float) -> float:
        """Apply minimum velocity threshold, preserving sign. Returns 0 if velocity is 0."""
        if velocity == 0.0:
            return 0.0
        if abs(velocity) < min_velocity:
            return min_velocity if velocity > 0 else -min_velocity
        return velocity

    def _compute_angular_velocity(self, yaw_error: float, max_speed: float) -> float:
        dt = 1.0 / self._control_frequency

        # PD control: proportional + derivative damping
        yaw_error_derivative = (yaw_error - self._prev_yaw_error) / dt
        angular_velocity = self._k_angular * yaw_error - self._k_derivative * yaw_error_derivative

        # Rate limiting: limit angular acceleration to prevent jerky corrections
        max_delta = self._max_angular_accel * dt
        angular_velocity = np.clip(
            angular_velocity,
            self._prev_angular_velocity - max_delta,
            self._prev_angular_velocity + max_delta,
        )

        # Clamp to max speed
        angular_velocity = np.clip(angular_velocity, -max_speed, max_speed)

        # Apply minimum velocity threshold
        angular_velocity = self._apply_min_velocity(angular_velocity, self._min_angular_velocity)

        # Update state for next iteration
        self._prev_yaw_error = yaw_error
        self._prev_angular_velocity = angular_velocity

        return float(angular_velocity)

    def _compute_initial_rotation(self) -> Twist:
        with self._lock:
            path = self._path
            current_odom = self._current_odom

        assert path is not None
        assert current_odom is not None

        first_pose = path.poses[0]
        first_yaw = quaternion_to_euler(first_pose.orientation).z
        robot_yaw = current_odom.orientation.euler[2]
        yaw_error = normalize_angle(first_yaw - robot_yaw)

        if abs(yaw_error) < self._orientation_tolerance:
            with self._lock:
                self._change_state("path_following")
            return self._compute_path_following()

        angular_velocity = self._compute_angular_velocity(yaw_error, self._speed)
        return self._angular_twist(angular_velocity)

    def get_distance_to_path(self) -> float | None:
        with self._lock:
            path_distancer = self._path_distancer
            current_odom = self._current_odom

        if path_distancer is None or current_odom is None:
            return None

        current_pos = np.array([current_odom.position.x, current_odom.position.y])

        return path_distancer.get_distance_to_path(current_pos)

    def _compute_path_following(self) -> Twist:
        with self._lock:
            path_distancer = self._path_distancer
            current_odom = self._current_odom

        assert path_distancer is not None
        assert current_odom is not None

        current_pos = np.array([current_odom.position.x, current_odom.position.y])

        if path_distancer.distance_to_goal(current_pos) < self._goal_tolerance:
            logger.info("Reached goal position, starting final rotation")
            with self._lock:
                self._change_state("final_rotation")
            return self._compute_final_rotation()

        closest_index = path_distancer.find_closest_point_index(current_pos)

        with self._lock:
            self._pose_index = closest_index

        lookahead_point = path_distancer.find_lookahead_point(closest_index)

        direction = lookahead_point - current_pos
        distance = np.linalg.norm(direction)

        if distance < 1e-6:
            # Robot is coincidentally at the lookahead point; skip this cycle.
            return Twist()

        robot_yaw = current_odom.orientation.euler[2]
        desired_yaw = np.arctan2(direction[1], direction[0])
        yaw_error = normalize_angle(desired_yaw - robot_yaw)

        # Rotate-then-drive: if heading error is large, rotate in place first
        rotation_threshold = self._rotation_threshold * math.pi / 180
        if abs(yaw_error) > rotation_threshold:
            angular_velocity = self._compute_angular_velocity(yaw_error, self._speed)
            return Twist(
                linear=Vector3(0.0, 0.0, 0.0),
                angular=Vector3(0.0, 0.0, angular_velocity),
            )

        # When aligned, drive forward with proportional angular correction
        angular_velocity = self._compute_angular_velocity(yaw_error, self._speed)
        linear_velocity = self._speed * (1.0 - abs(yaw_error) / rotation_threshold)
        linear_velocity = self._apply_min_velocity(linear_velocity, self._min_linear_velocity)

        return Twist(
            linear=Vector3(linear_velocity, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, angular_velocity),
        )

    def _compute_final_rotation(self) -> Twist:
        with self._lock:
            path = self._path
            current_odom = self._current_odom

        assert path is not None
        assert current_odom is not None

        goal_yaw = quaternion_to_euler(path.poses[-1].orientation).z
        robot_yaw = current_odom.orientation.euler[2]
        yaw_error = normalize_angle(goal_yaw - robot_yaw)

        if abs(yaw_error) < self._orientation_tolerance:
            logger.info("Final rotation complete, goal reached")
            with self._lock:
                self._change_state("arrived")
            return Twist()

        angular_velocity = self._compute_angular_velocity(yaw_error, self._speed)
        return self._angular_twist(angular_velocity)

    def _reset_state(self) -> None:
        with self._lock:
            self._change_state("idle")
            self._path = None
            self._path_clearance = None
            self._path_distancer = None
            self._pose_index = 0
            self._prev_yaw_error = 0.0
            self._prev_angular_velocity = 0.0

    def _make_debug_navigation_image(self, path: Path, path_clearance: PathClearance) -> Image:
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

        # Add path mask.
        mask = path_clearance.mask
        scaled_mask = np.repeat(np.repeat(mask, scale, axis=0), scale, axis=1)
        scaled_mask = np.flipud(scaled_mask)
        white = np.array([255, 255, 255], dtype=np.int16)
        image.data[scaled_mask] = (image.data[scaled_mask].astype(np.int16) * 3 + white * 7) // 10

        with self._lock:
            current_odom = self._current_odom

        # Draw robot position.
        if current_odom is not None:
            grid_pos = self._navigation_map.gradient_costmap.world_to_grid(current_odom.position)
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

    def _angular_twist(self, angular_velocity: float) -> Twist:
        # In simulation, add a small forward velocity to help the locomotion
        # policy execute rotation (some policies don't handle pure in-place rotation).
        linear_x = self._min_linear_velocity if self._global_config.simulation else 0.0

        return Twist(
            linear=Vector3(linear_x, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, angular_velocity),
        )
