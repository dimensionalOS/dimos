# Copyright 2025-2026 Dimensional Inc.
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

from dataclasses import dataclass
import os
from threading import Event, RLock, Thread
import time
import traceback
from typing import Literal, TypeAlias

import numpy as np
from reactivex import Subject

from dimos.core.global_config import GlobalConfig
from dimos.core.resource import Resource
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.base import NavigationState
from dimos.navigation.replanning_a_star.controllers import Controller, PController
from dimos.navigation.replanning_a_star.navigation_map import NavigationMap
from dimos.navigation.replanning_a_star.path_clearance import PathClearance
from dimos.navigation.replanning_a_star.path_distancer import PathDistancer
from dimos.utils.logging_config import setup_logger
from dimos.utils.trigonometry import angle_diff

PlannerState: TypeAlias = Literal[
    "idle", "initial_rotation", "path_following", "final_rotation", "arrived"
]
StopMessage: TypeAlias = Literal["arrived", "obstacle_found", "error"]
PlanEpochs: TypeAlias = tuple[int, int]


@dataclass(frozen=True)
class PlanStopEvent:
    reason: StopMessage
    plan_epochs: PlanEpochs | None = None


logger = setup_logger()


class LocalPlanner(Resource):
    cmd_vel: Subject[Twist]
    stopped_navigating: Subject[StopMessage]
    _plan_stopped: Subject[PlanStopEvent]
    navigation_costmap: Subject[OccupancyGrid]

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
    _goal_tolerance: float
    _controller: Controller

    _speed: float = 0.55
    _control_frequency: float = 10
    _orientation_tolerance: float = 0.35
    _navigation_costmap_interval: float = 1.0
    _navigation_costmap_last: float = 0.0

    def __init__(
        self, global_config: GlobalConfig, navigation_map: NavigationMap, goal_tolerance: float
    ) -> None:
        self.cmd_vel = Subject()
        self.stopped_navigating = Subject()
        self._plan_stopped = Subject()
        self.navigation_costmap = Subject()

        self._pose_index = 0
        self._lock = RLock()
        self._stop_planning_event = Event()
        self._state = "idle"
        self._state_unique_id = 0
        self._global_config = global_config
        self._navigation_map = navigation_map
        self._goal_tolerance = goal_tolerance

        speed = self._speed
        if global_config.nerf_speed < 1.0:
            speed *= global_config.nerf_speed

        self._controller = PController(
            self._global_config,
            speed,
            self._control_frequency,
        )

    def start(self) -> None:
        pass

    def stop(self) -> None:
        self.stop_planning()

    def handle_odom(self, msg: PoseStamped) -> None:
        with self._lock:
            self._current_odom = msg

    def start_planning(self, path: Path) -> None:
        self.stop_planning()
        self._start_planning(path, None)

    @property
    def plan_stopped(self) -> Subject[PlanStopEvent]:
        return self._plan_stopped

    def start_tagged_planning(self, path: Path, plan_epochs: PlanEpochs) -> None:
        self._deactivate_planning()
        self._start_planning(path, plan_epochs)

    def deactivate_planning(self) -> None:
        self._deactivate_planning()

    def publish_stop_command(self) -> None:
        self._publish_stop_command()

    def _start_planning(self, path: Path, plan_epochs: PlanEpochs | None) -> None:
        stop_event = Event()

        with self._lock:
            self._stop_planning_event.set()
            self._stop_planning_event = stop_event
            self._path = path
            self._path_clearance = PathClearance(self._global_config, self._path)
            self._path_distancer = PathDistancer(self._path)
            self._pose_index = 0
            self._thread = Thread(
                target=self._thread_entrypoint,
                args=(stop_event, plan_epochs),
                daemon=True,
            )
            self._thread.start()

    def stop_planning(self) -> None:
        try:
            self._deactivate_planning()
        finally:
            self._publish_stop_command()

    def _deactivate_planning(self) -> None:
        with self._lock:
            self._stop_planning_event.set()
            self._thread = None
            self._reset_state()

    def _publish_stop_command(self) -> None:
        try:
            self.cmd_vel.on_next(Twist())
        except Exception as error:
            logger.exception("Error publishing local planner stop command", exc_info=error)

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

    def _thread_entrypoint(
        self,
        stop_event: Event,
        plan_epochs: PlanEpochs | None,
    ) -> None:
        try:
            self._loop(stop_event, plan_epochs)
        except Exception as e:
            traceback.print_exc()
            logger.exception("Error in local planning", exc_info=e)
            self._publish_stop_event("error", plan_epochs)
        finally:
            is_current = False
            with self._lock:
                is_current = self._stop_planning_event is stop_event
                if is_current:
                    self._reset_state()
            if is_current:
                self._publish_stop_command()

    def _publish_stop_event(self, reason: StopMessage, plan_epochs: PlanEpochs | None) -> None:
        try:
            self._plan_stopped.on_next(PlanStopEvent(reason, plan_epochs))
        except Exception as error:
            logger.exception("Error notifying tagged local planner stop observers", exc_info=error)
        try:
            self.stopped_navigating.on_next(reason)
        except Exception as error:
            logger.exception("Error notifying local planner stop observers", exc_info=error)

    def _change_state(self, new_state: PlannerState) -> None:
        if new_state == self._state:
            return
        self._state = new_state
        self._state_unique_id += 1
        logger.info("changed state", state=new_state)

    def _worker_is_current(self, stop_event: Event) -> bool:
        return self._stop_planning_event is stop_event and not stop_event.is_set()

    def _loop(self, stop_event: Event, plan_epochs: PlanEpochs | None) -> None:
        with self._lock:
            if not self._worker_is_current(stop_event):
                return
            path = self._path
            path_clearance = self._path_clearance
            current_odom = self._current_odom

        if path is None or path_clearance is None:
            raise RuntimeError("No path set for local planner.")

        # Determine initial state: skip initial_rotation if already aligned.
        new_state: PlannerState = "initial_rotation"
        initial_yaw_error: float | None = None
        if current_odom is not None and len(path.poses) > 0:
            first_yaw = path.poses[0].orientation.euler[2]
            robot_yaw = current_odom.orientation.euler[2]
            initial_yaw_error = angle_diff(first_yaw, robot_yaw)
            angle_in_tolerance = abs(initial_yaw_error) < self._orientation_tolerance
            if angle_in_tolerance:
                position_in_tolerance = (
                    path.poses[0].position.distance(current_odom.position) < 0.01
                )
                if position_in_tolerance:
                    new_state = "final_rotation"
                else:
                    new_state = "path_following"

        with self._lock:
            if not self._worker_is_current(stop_event):
                return
            if initial_yaw_error is not None:
                self._controller.reset_yaw_error(initial_yaw_error)
            self._change_state(new_state)

        while True:
            start_time = time.perf_counter()

            with self._lock:
                if not self._worker_is_current(stop_event):
                    break
                path_clearance.update_costmap(self._navigation_map.binary_costmap)
                path_clearance.update_pose_index(self._pose_index)

            self._send_navigation_costmap(path, path_clearance)

            if path_clearance.is_obstacle_ahead():
                with self._lock:
                    if not self._worker_is_current(stop_event):
                        break
                logger.info("Obstacle detected ahead, stopping local planner.")
                self._publish_stop_event("obstacle_found", plan_epochs)
                break

            with self._lock:
                if not self._worker_is_current(stop_event):
                    break
                state: PlannerState = self._state

            cmd_vel: Twist | None
            match state:
                case "initial_rotation":
                    cmd_vel = self._compute_initial_rotation(stop_event)
                case "path_following":
                    cmd_vel = self._compute_path_following(stop_event)
                case "final_rotation":
                    cmd_vel = self._compute_final_rotation(stop_event)
                case "arrived":
                    self._publish_stop_event("arrived", plan_epochs)
                    break
                case "idle":
                    cmd_vel = None
                case _:
                    raise ValueError(f"Unknown planner state: {state}")

            if cmd_vel is not None:
                with self._lock:
                    publish_command = self._worker_is_current(stop_event)
                if publish_command:
                    self.cmd_vel.on_next(cmd_vel)

            elapsed = time.perf_counter() - start_time
            sleep_time = max(0.0, (1.0 / self._control_frequency) - elapsed)
            stop_event.wait(sleep_time)

        if stop_event.is_set():
            logger.info("Local planner loop exited due to stop event.")

    def _compute_initial_rotation(self, stop_event: Event) -> Twist | None:
        with self._lock:
            if not self._worker_is_current(stop_event):
                return None
            path = self._path
            current_odom = self._current_odom

        assert path is not None
        assert current_odom is not None

        first_pose = path.poses[0]
        first_yaw = first_pose.orientation.euler[2]
        robot_yaw = current_odom.orientation.euler[2]
        yaw_error = angle_diff(first_yaw, robot_yaw)

        if abs(yaw_error) < self._orientation_tolerance:
            with self._lock:
                if not self._worker_is_current(stop_event):
                    return None
                self._change_state("path_following")
            return self._compute_path_following(stop_event)

        with self._lock:
            if not self._worker_is_current(stop_event):
                return None
            return self._controller.rotate(yaw_error)

    def get_distance_to_path(self) -> float | None:
        with self._lock:
            path_distancer = self._path_distancer
            current_odom = self._current_odom

        if path_distancer is None or current_odom is None:
            return None

        current_pos = np.array([current_odom.position.x, current_odom.position.y])

        return path_distancer.get_distance_to_path(current_pos)

    def _compute_path_following(self, stop_event: Event) -> Twist | None:
        with self._lock:
            if not self._worker_is_current(stop_event):
                return None
            path_distancer = self._path_distancer
            current_odom = self._current_odom

        assert path_distancer is not None
        assert current_odom is not None

        current_pos = np.array([current_odom.position.x, current_odom.position.y])

        if path_distancer.distance_to_goal(current_pos) < self._goal_tolerance:
            logger.info("Reached goal position, starting final rotation")
            with self._lock:
                if not self._worker_is_current(stop_event):
                    return None
                self._change_state("final_rotation")
            return self._compute_final_rotation(stop_event)

        closest_index = path_distancer.find_closest_point_index(current_pos)

        with self._lock:
            if not self._worker_is_current(stop_event):
                return None
            self._pose_index = closest_index

        lookahead_point = path_distancer.find_lookahead_point(closest_index)

        with self._lock:
            if not self._worker_is_current(stop_event):
                return None
            return self._controller.advance(lookahead_point, current_odom)

    def _compute_final_rotation(self, stop_event: Event) -> Twist | None:
        with self._lock:
            if not self._worker_is_current(stop_event):
                return None
            path = self._path
            current_odom = self._current_odom

        assert path is not None
        assert current_odom is not None

        goal_yaw = path.poses[-1].orientation.euler[2]
        robot_yaw = current_odom.orientation.euler[2]
        yaw_error = angle_diff(goal_yaw, robot_yaw)

        if abs(yaw_error) < self._orientation_tolerance:
            logger.info("Final rotation complete, goal reached")
            with self._lock:
                if not self._worker_is_current(stop_event):
                    return None
                self._change_state("arrived")
            return Twist()

        with self._lock:
            if not self._worker_is_current(stop_event):
                return None
            return self._controller.rotate(yaw_error)

    def _reset_state(self) -> None:
        with self._lock:
            self._change_state("idle")
            self._path = None
            self._path_clearance = None
            self._path_distancer = None
            self._pose_index = 0
            self._controller.reset_errors()

    def _send_navigation_costmap(self, path: Path, path_clearance: PathClearance) -> None:
        if "DEBUG_NAVIGATION" not in os.environ:
            return

        now = time.time()
        if now - self._navigation_costmap_last < self._navigation_costmap_interval:
            return

        self._navigation_costmap_last = now

        self.navigation_costmap.on_next(self._navigation_map.gradient_costmap)
