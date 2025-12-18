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

from threading import Event, RLock, Thread, current_thread

from reactivex import Subject
from reactivex.disposable import CompositeDisposable

from dimos.core.global_config import GlobalConfig
from dimos.core.resource import Resource
from dimos.mapping.occupancy.path_resampling import smooth_resample_path
from dimos.msgs.geometry_msgs import Twist
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs import Image
from dimos.navigation.base import NavigationState
from dimos.navigation.bt_navigator.goal_validator import find_safe_goal
from dimos.navigation.global_planner.astar import astar
from dimos.navigation.replanning_a_star.local_planner import LocalPlanner, StopMessage
from dimos.navigation.replanning_a_star.navigation_map import NavigationMap
from dimos.navigation.replanning_a_star.position_tracker import PositionTracker
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class GlobalPlanner(Resource):
    path: Subject[Path]

    _current_odom: PoseStamped | None = None
    _current_goal: PoseStamped | None = None
    _goal_reached: bool = False
    _thread: Thread | None = None
    _global_config: GlobalConfig
    _navigation_map: NavigationMap
    _local_planner: LocalPlanner
    _disposables: CompositeDisposable
    _stop_planner: Event
    _lock: RLock
    _replan_attempt: int

    _safe_goal_tolerance: float = 4.0
    _replan_goal_tolerance: float = 0.5
    _max_replan_attempts: int = 10

    def __init__(self, global_config: GlobalConfig) -> None:
        self.path = Subject()
        self._global_config = global_config
        self._navigation_map = NavigationMap(self._global_config)
        self._local_planner = LocalPlanner(self._global_config, self._navigation_map)
        self._position_tracker = PositionTracker()
        self._disposables = CompositeDisposable()
        self._lock = RLock()
        self._stop_planner = Event()

    def start(self) -> None:
        self._local_planner.start()
        self._disposables.add(
            self._local_planner.stopped_navigating.subscribe(self._on_stopped_navigating)
        )
        self._stop_planner.clear()
        self._thread = Thread(target=self._thread_entrypoint, daemon=True)
        self._thread.start()
        self._replan_attempt = 0

    def stop(self) -> None:
        self.cancel_goal()
        self._local_planner.stop()
        self._disposables.dispose()
        self._stop_planner.set()
        if self._thread is not None:
            if self._thread is not current_thread():
                self._thread.join(2)
            self._thread = None

    def handle_odom(self, msg: PoseStamped) -> None:
        with self._lock:
            self._current_odom = msg

        self._local_planner.handle_odom(msg)
        self._position_tracker.add_position(msg)

    def handle_global_costmap(self, msg: OccupancyGrid) -> None:
        self._navigation_map.update(msg)

    def handle_goal_request(self, goal: PoseStamped) -> None:
        with self._lock:
            self._current_goal = goal
            self._goal_reached = False
            self._replan_attempt = 0
        self._plan_path()

    def cancel_goal(self, *, but_will_try_again: bool = False, arrived: bool = False) -> None:
        with self._lock:
            self._position_tracker.reset_data()

            if not but_will_try_again:
                self._current_goal = None
                self._goal_reached = arrived
                self._replan_attempt = 0

        self.path.on_next(Path())
        self._local_planner.stop_planning()

    def get_state(self) -> NavigationState:
        return self._local_planner.get_state()

    def is_goal_reached(self) -> bool:
        with self._lock:
            return self._goal_reached

    @property
    def cmd_vel(self) -> Subject[Twist]:
        return self._local_planner.cmd_vel

    @property
    def debug_navigation(self) -> Subject[Image]:
        return self._local_planner.debug_navigation

    def _thread_entrypoint(self) -> None:
        while not self._stop_planner.is_set():
            with self._lock:
                current_goal = self._current_goal

            if current_goal:
                if self._position_tracker.is_stuck():
                    self._replan_path()

            self._stop_planner.wait(0.1)

    def _on_stopped_navigating(self, stop_message: StopMessage) -> None:
        self.path.on_next(Path())
        if stop_message == "obstacle_found":
            logger.info("Replanning path due to obstacle found.")
            self._replan_path()
        elif stop_message == "arrived":
            logger.info("Arrived at goal.")
            self.cancel_goal(arrived=True)
        elif stop_message == "error":
            logger.info("Failure in navigation.")
            self._replan_path()
        else:
            logger.error(f"No code to handle '{stop_message}'.")
            self.cancel_goal()

    def _replan_path(self) -> None:
        with self._lock:
            current_odom = self._current_odom
            current_goal = self._current_goal
            replan_attempt = self._replan_attempt

        assert current_odom is not None
        assert current_goal is not None

        if current_goal.position.distance(current_odom.position) < self._replan_goal_tolerance:
            self.cancel_goal(arrived=True)
            return

        if replan_attempt + 1 > self._max_replan_attempts:
            self.cancel_goal()
            return

        with self._lock:
            self._replan_attempt += 1

        self._plan_path()

    def _plan_path(self) -> None:
        self.cancel_goal(but_will_try_again=True)

        with self._lock:
            current_odom = self._current_odom
            current_goal = self._current_goal

        assert current_goal is not None

        if current_odom is None:
            logger.warning("Cannot handle goal request: missing odometry")
            return

        safe_goal = find_safe_goal(
            self._navigation_map.binary_costmap,
            current_goal.position,
            algorithm="bfs_contiguous",
            cost_threshold=CostValues.OCCUPIED,
            min_clearance=self._global_config.robot_rotation_diameter / 2,
            max_search_distance=self._safe_goal_tolerance,
        )

        if safe_goal is None:
            logger.warning("No safe goal found near requested target")
            return

        goals_distance = safe_goal.distance(current_goal.position)
        if goals_distance > 0.2:
            logger.warning(f"Travelling to goal {goals_distance}m away from requested goal.")

        logger.info("Found safe goal", x=round(safe_goal.x, 2), y=round(safe_goal.y, 2))

        robot_pos = current_odom.position
        costmap = self._navigation_map.gradient_costmap
        path = astar(self._global_config.astar_algorithm, costmap, safe_goal, robot_pos)

        if not path:
            logger.warning(
                "No path found to the goal.", x=round(safe_goal.x, 3), y=round(safe_goal.y, 3)
            )
            return

        resampled_path = smooth_resample_path(path, current_goal, 0.1)

        self.path.on_next(resampled_path)

        self._local_planner.start_planning(resampled_path)
