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

import math
from threading import Event, RLock, Thread, current_thread
import time

from dimos_lcm.std_msgs import Bool
import numpy as np
from reactivex import Subject
from reactivex.disposable import CompositeDisposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.global_config import GlobalConfig
from dimos.core.resource import Resource
from dimos.mapping.occupancy.path_resampling import smooth_resample_path
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.base import NavigationState
from dimos.navigation.replanning_a_star.goal_validator import find_safe_goal
from dimos.navigation.replanning_a_star.local_planner import LocalPlanner, StopMessage
from dimos.navigation.replanning_a_star.min_cost_astar import min_cost_astar
from dimos.navigation.replanning_a_star.navigation_map import NavigationMap
from dimos.navigation.replanning_a_star.position_tracker import PositionTracker
from dimos.navigation.replanning_a_star.recovery_supervisor import (
    RecoveryAction,
    RecoveryCapabilities,
    RecoveryCause,
    RecoveryEvent,
    RecoveryOutcome,
    RecoverySupervisor,
)
from dimos.navigation.replanning_a_star.replan_limiter import ReplanLimiter
from dimos.utils.logging_config import setup_logger
from dimos.utils.trigonometry import angle_diff

logger = setup_logger()


class GlobalPlanner(Resource):
    path: Subject[Path]
    goal_reached: Subject[Bool]
    recovery_event: Subject[RecoveryEvent]

    _current_odom: PoseStamped | None = None
    _current_goal: PoseStamped | None = None
    _goal_reached: bool = False
    _thread: Thread | None = None

    _global_config: GlobalConfig
    _navigation_map: NavigationMap
    _navigation_map_near: NavigationMap
    _local_planner: LocalPlanner
    _position_tracker: PositionTracker
    _replan_limiter: ReplanLimiter
    _disposables: CompositeDisposable
    _stop_planner: Event
    _replan_event: Event
    _replan_reason: StopMessage | None
    _lock: RLock
    _safe_goal_clearance: float
    _recovery_supervisor: RecoverySupervisor
    _recovery_active: bool
    _active_recovery: tuple[int, RecoveryCause, RecoveryAction] | None
    _goal_revision: int

    _safe_goal_tolerance: float = 4.0
    _goal_tolerance: float = 0.2
    _rotation_tolerance: float = math.radians(15)
    _replan_goal_tolerance: float = 0.5
    # Recover sooner when commanded motion produces no meaningful translation.
    # Six seconds still leaves normal initial/final rotation enough time to
    # complete before the position-only stuck detector requests a replan.
    _stuck_time_window: float = 6.0
    _stuck_threshold: float = 0.4
    _max_path_deviation: float = 0.9
    _replanning_enabled: bool = True

    def __init__(self, global_config: GlobalConfig) -> None:
        self.path = Subject()
        self.goal_reached = Subject()
        self.recovery_event = Subject()

        self._global_config = global_config
        self._navigation_map = NavigationMap(self._global_config, "voronoi")
        self._navigation_map_near = NavigationMap(self._global_config, "gradient")
        self._local_planner = LocalPlanner(
            self._global_config, self._navigation_map, self._goal_tolerance
        )

        stuck_threshold = self._stuck_threshold
        if global_config.simulation:
            stuck_threshold = 1.0

        self._position_tracker = PositionTracker(self._stuck_time_window, stuck_threshold)
        self._replan_limiter = ReplanLimiter()
        self._disposables = CompositeDisposable()
        self._stop_planner = Event()
        self._replan_event = Event()
        self._replan_reason = None
        self._lock = RLock()
        self._recovery_supervisor = RecoverySupervisor()
        self._recovery_active = False
        self._active_recovery = None
        self._goal_revision = 0
        self._reset_safe_goal_clearance()

    def start(self) -> None:
        self._local_planner.start()
        self._disposables.add(
            self._local_planner.stopped_navigating.subscribe(self._on_stopped_navigating)
        )
        self._stop_planner.clear()
        self._thread = Thread(target=self._thread_entrypoint, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self.cancel_goal()
        self._local_planner.stop()
        self._disposables.dispose()
        self._stop_planner.set()
        self._replan_event.set()

        if self._thread is not None and self._thread is not current_thread():
            self._thread.join(DEFAULT_THREAD_JOIN_TIMEOUT)
            if self._thread.is_alive():
                logger.error("GlobalPlanner thread did not stop in time.")
            self._thread = None

    def handle_odom(self, msg: PoseStamped) -> None:
        with self._lock:
            self._current_odom = msg

        self._local_planner.handle_odom(msg)
        self._position_tracker.add_position(msg)

    def handle_global_costmap(self, msg: OccupancyGrid) -> None:
        self._navigation_map.update(msg)
        self._navigation_map_near.update(msg)

    def handle_goal_request(self, goal: PoseStamped) -> None:
        logger.info("Got new goal", goal=str(goal))
        self._finish_recovery(RecoveryOutcome.CANCELLED, "superseded_by_new_goal")
        with self._lock:
            self._current_goal = goal
            self._goal_reached = False
            self._goal_revision += 1
        self._replan_limiter.reset()
        self._plan_path()

    def set_safe_goal_clearance(self, clearance: float) -> None:
        with self._lock:
            self._safe_goal_clearance = clearance

    def reset_safe_goal_clearance(self) -> None:
        self._reset_safe_goal_clearance()

    def cancel_goal(self, *, but_will_try_again: bool = False, arrived: bool = False) -> None:
        # return silently so we don't flood the logs.
        with self._lock:
            no_goal = self._current_goal is None
            recovery_active = self._recovery_active
        if (
            no_goal
            and not recovery_active
            and self._local_planner.get_state() == NavigationState.IDLE
        ):
            return

        logger.info("Cancelling goal.", but_will_try_again=but_will_try_again, arrived=arrived)

        with self._lock:
            self._position_tracker.reset_data()

            if not but_will_try_again:
                self._current_goal = None
                self._goal_reached = arrived
                self._replan_limiter.reset()
                self._goal_revision += 1

        if not but_will_try_again:
            outcome = RecoveryOutcome.SUCCEEDED if arrived else RecoveryOutcome.CANCELLED
            reason = "goal_reached" if arrived else "goal_cancelled"
            self._finish_recovery(outcome, reason)

        self.path.on_next(Path())
        self._local_planner.stop_planning()

        if not but_will_try_again:
            self.goal_reached.on_next(Bool(arrived))

    def set_replanning_enabled(self, enabled: bool) -> None:
        with self._lock:
            self._replanning_enabled = enabled

    def get_state(self) -> NavigationState:
        with self._lock:
            if self._recovery_active:
                return NavigationState.RECOVERY
        return self._local_planner.get_state()

    def is_goal_reached(self) -> bool:
        with self._lock:
            return self._goal_reached

    @property
    def cmd_vel(self) -> Subject[Twist]:
        return self._local_planner.cmd_vel

    @property
    def navigation_costmap(self) -> Subject[OccupancyGrid]:
        return self._local_planner.navigation_costmap

    def _thread_entrypoint(self) -> None:
        """Monitor if the robot is stuck, veers off track, or stopped navigating."""

        last_id = -1
        last_stuck_check = time.perf_counter()

        while not self._stop_planner.is_set():
            # Wait for either timeout or replan signal from local planner.
            replanning_wanted = self._replan_event.wait(timeout=0.1)

            if self._stop_planner.is_set():
                break

            # Handle stop message from local planner (priority)
            if replanning_wanted:
                self._replan_event.clear()
                with self._lock:
                    reason = self._replan_reason
                    self._replan_reason = None

                if reason is not None:
                    self._handle_stop_message(reason)
                    last_stuck_check = time.perf_counter()
                    continue

            with self._lock:
                current_goal = self._current_goal
                current_odom = self._current_odom

            if not current_goal or not current_odom:
                continue

            if (
                current_goal.position.distance(current_odom.position) < self._goal_tolerance
                and abs(
                    angle_diff(current_goal.orientation.euler[2], current_odom.orientation.euler[2])
                )
                < self._rotation_tolerance
            ):
                logger.info("Close enough to goal. Accepting as arrived.")
                self.cancel_goal(arrived=True)
                continue

            # Check if robot has veered too far off the path
            deviation = self._local_planner.get_distance_to_path()
            if deviation is not None and deviation > self._max_path_deviation:
                logger.info(
                    "Robot veered off track. Replanning.",
                    deviation=round(deviation, 2),
                    threshold=self._max_path_deviation,
                )
                self._replan_path(RecoveryCause.PATH_DEVIATION)
                last_stuck_check = time.perf_counter()
                continue

            local_state, new_id = self._local_planner.get_unique_state()

            if new_id != last_id:
                last_id = new_id
                last_stuck_check = time.perf_counter()
                if local_state in {"path_following", "final_rotation", "arrived"}:
                    self._finish_recovery(
                        RecoveryOutcome.SUCCEEDED,
                        "local_planner_resumed",
                    )
                continue

            if (
                time.perf_counter() - last_stuck_check > self._stuck_time_window
                and self._position_tracker.is_stuck()
            ):
                logger.info("Robot is stuck. Replanning.")
                self._replan_path(RecoveryCause.PROGRESS_TIMEOUT)
                last_stuck_check = time.perf_counter()

    def _on_stopped_navigating(self, stop_message: StopMessage) -> None:
        with self._lock:
            self._replan_reason = stop_message
        # Signal the monitoring thread to do the replanning. This is so we don't have two
        # threads which could be replanning at the same time.
        self._replan_event.set()

    def _handle_stop_message(self, stop_message: StopMessage) -> None:
        # Note, this runs in the monitoring thread.

        self.path.on_next(Path())

        if stop_message == "arrived":
            logger.info("Arrived at goal.")
            self.cancel_goal(arrived=True)
        elif stop_message == "obstacle_found":
            logger.info("Replanning path due to obstacle found.")
            self._replan_path(RecoveryCause.OBSTACLE)
        elif stop_message == "error":
            logger.info("Failure in navigation.")
            self._replan_path(RecoveryCause.PLANNER_ERROR)
        else:
            logger.error(f"No code to handle '{stop_message}'.")
            self.cancel_goal()

    def _replan_path(self, cause: RecoveryCause) -> None:
        with self._lock:
            current_odom = self._current_odom
            current_goal = self._current_goal
            goal_revision = self._goal_revision

        if current_odom is None or current_goal is None:
            return

        if current_goal.position.distance(current_odom.position) < self._replan_goal_tolerance:
            self.cancel_goal(arrived=True)
            return

        if not self._replanning_enabled:
            self._emit_terminal_recovery_failure(
                cause=cause,
                reason="replanning_disabled",
            )
            self.cancel_goal()
            return

        can_retry = self._replan_limiter.can_retry(current_odom.position)
        attempt = self._replan_limiter.get_attempt()
        decision = self._recovery_supervisor.decide(
            attempt=attempt,
            cause=cause,
            capabilities=RecoveryCapabilities(),
        )
        logger.info(
            "Recovery decision.",
            attempt=attempt + 1,
            cause=cause.value,
            action=decision.action.value,
            skipped=[action.value for action in decision.skipped_actions],
        )

        if not can_retry or decision.action is RecoveryAction.FAIL:
            self._emit_terminal_recovery_failure(
                cause=cause,
                reason=decision.failure_reason or "replan_limit_reached",
                attempt=attempt,
            )
            self.cancel_goal()
            return

        if decision.action not in {RecoveryAction.REPLAN, RecoveryAction.ROTATE_RESCAN}:
            self._emit_terminal_recovery_failure(
                cause=cause,
                reason=f"unsupported_recovery_action:{decision.action.value}",
                attempt=attempt,
            )
            self.cancel_goal()
            return

        self._finish_recovery(RecoveryOutcome.FAILED, "obstruction_persisted")
        self._replan_limiter.will_retry()
        event_attempt = attempt + 1
        with self._lock:
            self._recovery_active = True
            self._active_recovery = (event_attempt, cause, decision.action)

        self._emit_recovery_event(
            attempt=event_attempt,
            cause=cause,
            action=decision.action,
            outcome=RecoveryOutcome.DISPATCHED,
            reason=decision.reason,
        )
        planned = self._plan_path(
            recovery_heading_offset=decision.heading_offset_radians,
            cancel_on_failure=False,
        )
        if not planned:
            with self._lock:
                goal_still_active = (
                    self._goal_revision == goal_revision and self._current_goal is not None
                )
            if goal_still_active:
                self._finish_recovery(RecoveryOutcome.FAILED, "path_unavailable")
                self.cancel_goal()
            else:
                self._finish_recovery(RecoveryOutcome.CANCELLED, "goal_cancelled")
            return

    def _plan_path(
        self,
        *,
        recovery_heading_offset: float | None = None,
        cancel_on_failure: bool = True,
    ) -> bool:
        self.cancel_goal(but_will_try_again=True)

        with self._lock:
            current_odom = self._current_odom
            current_goal = self._current_goal
            goal_revision = self._goal_revision

        if current_goal is None or current_odom is None:
            logger.warning("Cannot handle goal request: missing goal or odometry.")
            return False

        safe_goal = self._find_safe_goal(current_goal.position)

        if not safe_goal:
            logger.warning(
                "No safe goal found.", x=round(current_goal.x, 3), y=round(current_goal.y, 3)
            )
            if cancel_on_failure:
                self.cancel_goal()
            return False

        path = self._find_wide_path(safe_goal, current_odom.position)

        if not path:
            logger.warning(
                "No path found to the goal.", x=round(safe_goal.x, 3), y=round(safe_goal.y, 3)
            )
            if cancel_on_failure:
                self.cancel_goal()
            return False

        resampled_path = smooth_resample_path(path, current_goal, 0.1)
        if recovery_heading_offset is not None:
            self._apply_recovery_heading(
                resampled_path,
                current_odom,
                recovery_heading_offset,
            )

        # A cancel or a newer goal can arrive while A* is computing. Hold the
        # planner lock across dispatch so cancellation can only happen before
        # this block (making the result stale) or after it (and then stop it).
        with self._lock:
            if (
                self._goal_revision != goal_revision
                or self._current_goal is not current_goal
            ):
                logger.info("Discarding stale path after goal changed.")
                return False
            self.path.on_next(resampled_path)
            self._local_planner.start_planning(resampled_path)
        return True

    def _apply_recovery_heading(
        self,
        path: Path,
        current_odom: PoseStamped,
        heading_offset: float,
    ) -> Path:
        """Force the existing local planner to rotate before translating.

        Only the freshly generated path's first orientation is changed. The
        route positions and final goal orientation remain untouched.
        """
        if not path.poses:
            return path

        current_euler = current_odom.orientation.euler
        path.poses[0].orientation = Quaternion.from_euler(
            Vector3(
                current_euler.x,
                current_euler.y,
                current_euler.z + heading_offset,
            )
        )
        return path

    def _emit_terminal_recovery_failure(
        self,
        *,
        cause: RecoveryCause,
        reason: str,
        attempt: int | None = None,
    ) -> None:
        self._finish_recovery(RecoveryOutcome.FAILED, reason)
        self._emit_recovery_event(
            attempt=self._replan_limiter.get_attempt() if attempt is None else attempt,
            cause=cause,
            action=RecoveryAction.FAIL,
            outcome=RecoveryOutcome.FAILED,
            reason=reason,
        )

    def _finish_recovery(self, outcome: RecoveryOutcome, reason: str) -> None:
        with self._lock:
            active = self._active_recovery
            self._active_recovery = None
            self._recovery_active = False

        if active is None:
            return

        attempt, cause, action = active
        self._emit_recovery_event(
            attempt=attempt,
            cause=cause,
            action=action,
            outcome=outcome,
            reason=reason,
        )

    def _emit_recovery_event(
        self,
        *,
        attempt: int,
        cause: RecoveryCause,
        action: RecoveryAction,
        outcome: RecoveryOutcome,
        reason: str,
    ) -> None:
        event = RecoveryEvent(
            attempt=attempt,
            cause=cause,
            action=action,
            outcome=outcome,
            reason=reason,
            timestamp=time.time(),
        )
        logger.info(
            "Recovery event.",
            attempt=attempt,
            cause=cause.value,
            action=action.value,
            outcome=outcome.value,
            reason=reason,
        )
        self.recovery_event.on_next(event)

    def _find_wide_path(self, goal: Vector3, robot_pos: Vector3) -> Path | None:
        #        sizes_to_try: list[float] = [2.2, 1.7, 1.3, 1]
        sizes_to_try: list[float] = [1.1]

        for size in sizes_to_try:
            distance = robot_pos.distance(goal)
            navigation_map = self._navigation_map if distance > 1.5 else self._navigation_map_near
            costmap = navigation_map.make_gradient_costmap(size)
            self._clear_robot_footprint(costmap, navigation_map.binary_costmap, robot_pos)
            path = min_cost_astar(costmap, goal, robot_pos)
            if path and path.poses:
                logger.info(f"Found path {size}x robot width.")
                return path

        return None

    def _clear_robot_footprint(
        self, costmap: OccupancyGrid, binary: OccupancyGrid, robot_pos: Vector3
    ) -> None:
        """Make the cells under the robot passable for planning.

        A new obstacle can be observed for the first time when the robot is
        already inside its inflation envelope (it drove there before the
        costmap caught up). Inflation then covers the robot's cell, A* has no
        passable start neighbors, and every replan fails no matter where the
        goal is. The space the robot stands on is traversable by definition,
        so drop inflated costs there to high-but-passable. Cells occupied in
        the raw binary costmap (actually observed obstacle points) stay
        blocked.
        """
        if binary.grid.shape != costmap.grid.shape or binary.origin != costmap.origin:
            # A newer map update raced in between building the two grids;
            # skip clearing rather than misalign cells.
            return

        center = costmap.world_to_grid(robot_pos)
        center_x, center_y = int(center.x), int(center.y)
        cells = int(self._global_config.robot_rotation_diameter / 2 / costmap.resolution) + 1

        height, width = costmap.grid.shape
        y0, y1 = max(0, center_y - cells), min(height, center_y + cells + 1)
        x0, x1 = max(0, center_x - cells), min(width, center_x + cells + 1)
        if y0 >= y1 or x0 >= x1:
            return

        region = costmap.grid[y0:y1, x0:x1]
        binary_region = binary.grid[y0:y1, x0:x1]
        rows, columns = np.ogrid[y0:y1, x0:x1]
        disc = (rows - center_y) ** 2 + (columns - center_x) ** 2 <= cells**2
        clearable = disc & (region >= CostValues.OCCUPIED) & (binary_region < CostValues.OCCUPIED)
        region[clearable] = CostValues.OCCUPIED - 1

    def _find_safe_goal(self, goal: Vector3) -> Vector3 | None:
        costmap = self._navigation_map.binary_costmap

        if costmap.cell_value(goal) == CostValues.UNKNOWN:
            return goal

        safe_goal = find_safe_goal(
            costmap,
            goal,
            algorithm="bfs_contiguous",
            cost_threshold=CostValues.OCCUPIED,
            min_clearance=self._safe_goal_clearance,
            max_search_distance=self._safe_goal_tolerance,
        )

        if safe_goal is None:
            logger.warning("No safe goal found near requested target.")
            return None

        goals_distance = safe_goal.distance(goal)
        if goals_distance > 0.2:
            logger.warning(f"Travelling to goal {goals_distance}m away from requested goal.")

        logger.info("Found safe goal.", x=round(safe_goal.x, 2), y=round(safe_goal.y, 2))

        return safe_goal

    def _reset_safe_goal_clearance(self) -> None:
        with self._lock:
            self._safe_goal_clearance = self._global_config.robot_rotation_diameter / 2
