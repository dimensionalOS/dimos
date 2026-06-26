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
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.base import NavigationState
from dimos.navigation.dannav.controllers import (
    CommandEnvelopeOverrides,
    Controller,
    HolonomicPathController,
    command_envelope_overrides_for_profile,
    make_local_path_controller,
)
from dimos.navigation.replanning_a_star.navigation_map import NavigationMap
from dimos.navigation.dannav.path_clearance import PathClearance
from dimos.navigation.dannav.path_distancer import PathDistancer
from dimos.navigation.dannav.rotation_clearance import can_rotate_in_place
from dimos.navigation.holonomic_trajectory_controller.trajectory_control_tick_export import JsonlTrajectoryControlTickSink
from dimos.navigation.holonomic_trajectory_controller.trajectory_control_tick_log import (
    TrajectoryControlTickSink,
    append_trajectory_control_tick,
)
from dimos.navigation.holonomic_trajectory_controller.trajectory_path_speed_profile import (
    PathSpeedProfileLimits,
    profile_speed_along_polyline,
    speed_at_progress_m,
)
from dimos.navigation.holonomic_trajectory_controller.trajectory_run_profiles import (
    GO2_RUN_PROFILES,
    RunProfile,
    RunProfileError,
)
from dimos.navigation.holonomic_trajectory_controller.trajectory_types import (
    TrajectoryMeasuredSample,
    TrajectoryReferenceSample,
)
from dimos.utils.logging_config import setup_logger
from dimos.utils.trigonometry import angle_diff

PlannerState: TypeAlias = Literal[
    "idle", "initial_rotation", "path_following", "final_rotation", "arrived"
]
StopMessage: TypeAlias = Literal[
    "arrived",
    "obstacle_found",
    "error",
    "run_envelope_rejected",
    "map_updated",
]

logger = setup_logger()


@dataclass(frozen=True)
class ActiveRunEnvelope:
    """The movement envelope governing the current goal.

    ``profile_name`` is ``None`` for the default walking behavior, where the
    speed and limits come from ``planner_robot_speed`` and the
    ``local_planner_*`` config fields exactly as before run profiles existed.
    For a named run profile, the speed is the profile's requested speed after
    any global slowdown scaling, and the limits come from the profile.
    """

    profile_name: str | None
    speed_m_s: float
    path_limits: PathSpeedProfileLimits
    goal_decel_m_s2: float
    command_overrides: CommandEnvelopeOverrides | None


class LocalPlanner(Resource):
    cmd_vel: Subject[Twist]
    stopped_navigating: Subject[StopMessage]
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
    _trajectory_tick_sink: TrajectoryControlTickSink | None
    _previous_odom_for_velocity: PoseStamped | None

    _speed: float = 0.55
    _active_envelope: ActiveRunEnvelope
    _path_speed_profile_s: list[float] | None
    _path_speed_profile_v: list[float] | None
    _path_speed_profile_path_id: int | None
    _control_frequency: float
    _orientation_tolerance: float = 0.35
    _navigation_costmap_interval: float = 1.0
    _navigation_costmap_last: float = 0.0
    _hot_replan: bool

    def __init__(
        self,
        global_config: GlobalConfig,
        navigation_map: NavigationMap,
        goal_tolerance: float,
        *,
        hot_replan: bool = False,
    ) -> None:
        self.cmd_vel = Subject()
        self.stopped_navigating = Subject()
        self.navigation_costmap = Subject()

        self._pose_index = 0
        self._lock = RLock()
        self._stop_planning_event = Event()
        self._state = "idle"
        self._state_unique_id = 0
        self._global_config = global_config
        self._navigation_map = navigation_map
        self._goal_tolerance = goal_tolerance
        self._hot_replan = hot_replan
        self._control_frequency = float(global_config.local_planner_control_rate_hz)
        self._trajectory_tick_sink = self._make_trajectory_tick_sink(global_config)
        self._previous_odom_for_velocity = None

        speed = (
            float(global_config.planner_robot_speed)
            if global_config.planner_robot_speed is not None
            else self._speed
        )
        if not math.isfinite(speed) or speed <= 0.0:
            raise ValueError(f"planner speed must be a positive finite float, got {speed!r}")
        if global_config.nerf_speed < 1.0:
            speed *= global_config.nerf_speed
        self._speed = speed
        self._active_envelope = self._default_run_envelope()
        self._path_speed_profile_s = None
        self._path_speed_profile_v = None
        self._path_speed_profile_path_id = None

        self._controller = make_local_path_controller(
            self._global_config,
            speed,
            self._control_frequency,
        )

    def start(self) -> None:
        pass

    def stop(self) -> None:
        self.stop_planning()
        self._close_trajectory_tick_sink()

    def handle_odom(self, msg: PoseStamped) -> None:
        with self._lock:
            self._current_odom = msg

    def start_planning(self, path: Path, run_profile_name: str | None = None) -> None:
        self.stop_planning()

        envelope = self._resolve_run_envelope(run_profile_name)
        if envelope is None:
            self.stopped_navigating.on_next("run_envelope_rejected")
            return
        self._apply_run_envelope(envelope)

        self._stop_planning_event = Event()

        with self._lock:
            self._path = path
            self._path_clearance = PathClearance(self._global_config, self._path)
            self._path_distancer = PathDistancer(self._path)
            self._pose_index = 0
            self._previous_odom_for_velocity = None
            self._rebuild_path_speed_profile(self._path_distancer)
            self._thread = Thread(target=self._thread_entrypoint, daemon=True)
            self._thread.start()

    def update_path(self, path: Path) -> bool:
        """Swap the active path without stopping the local planner thread."""
        if not path.poses:
            return False

        with self._lock:
            if self._path is None or self._thread is None:
                return False

            self._path = path
            self._path_clearance = PathClearance(self._global_config, self._path)
            self._path_distancer = PathDistancer(self._path)
            current_odom = self._current_odom
            if current_odom is not None:
                current_pos = np.array([current_odom.position.x, current_odom.position.y])
                self._pose_index = self._path_distancer.find_closest_point_index(current_pos)
            self._rebuild_path_speed_profile(self._path_distancer)

        return True

    def _default_run_envelope(self) -> ActiveRunEnvelope:
        """Walking behavior: speed and limits exactly as configured today."""
        return ActiveRunEnvelope(
            profile_name=None,
            speed_m_s=self._speed,
            path_limits=PathSpeedProfileLimits(
                max_speed_m_s=self._speed,
                max_tangent_accel_m_s2=self._global_config.local_planner_max_tangent_accel_m_s2,
                max_normal_accel_m_s2=self._global_config.local_planner_max_normal_accel_m_s2,
            ),
            goal_decel_m_s2=self._global_config.local_planner_goal_decel_m_s2,
            command_overrides=None,
        )

    def _profile_run_envelope(self, profile: RunProfile) -> ActiveRunEnvelope:
        speed = profile.requested_planner_speed_m_s
        if self._global_config.nerf_speed < 1.0:
            speed *= self._global_config.nerf_speed
        return ActiveRunEnvelope(
            profile_name=profile.name,
            speed_m_s=speed,
            path_limits=profile.path_speed_profile_limits_at(speed),
            goal_decel_m_s2=profile.goal_decel_m_s2,
            command_overrides=command_envelope_overrides_for_profile(profile),
        )

    def _resolve_run_envelope(self, run_profile_name: str | None) -> ActiveRunEnvelope | None:
        """Resolve the movement envelope for one goal, before motion.

        A per-goal profile name wins over the session default
        (``GlobalConfig.go2_run_profile``). The registry default profile keeps
        the unchanged walking behavior. Any other named profile resolves its
        speed and limits from the registry.
        """
        name = (
            run_profile_name
            if run_profile_name is not None
            else self._global_config.go2_run_profile
        )
        if name == GO2_RUN_PROFILES.default_profile_name:
            return self._default_run_envelope()

        try:
            profile = GO2_RUN_PROFILES.get(name)
        except RunProfileError as exc:
            logger.warning(
                "run profile rejected",
                profile=name,
                reason=str(exc),
            )
            return None

        envelope = self._profile_run_envelope(profile)

        logger.info(
            "run envelope applied",
            profile=profile.name,
            speed_m_s=round(envelope.speed_m_s, 3),
            goal_decel_m_s2=envelope.goal_decel_m_s2,
            max_yaw_rate_rad_s=profile.max_yaw_rate_rad_s,
        )
        return envelope

    def _apply_run_envelope(self, envelope: ActiveRunEnvelope) -> None:
        self._active_envelope = envelope
        controller = self._controller
        if isinstance(controller, HolonomicPathController):
            controller.set_command_envelope(envelope.command_overrides)
        controller.set_speed(envelope.speed_m_s)
        with self._lock:
            path_distancer = self._path_distancer
        if path_distancer is not None:
            self._rebuild_path_speed_profile(path_distancer)

    def stop_planning(self) -> None:
        self.cmd_vel.on_next(Twist())
        self._stop_planning_event.set()

        with self._lock:
            self._thread = None

        self._reset_state()

    def _make_trajectory_tick_sink(
        self, global_config: GlobalConfig
    ) -> TrajectoryControlTickSink | None:
        path = global_config.local_planner_trajectory_tick_log_path
        if path is None or str(path).strip() == "":
            return None
        return JsonlTrajectoryControlTickSink(path)

    def _close_trajectory_tick_sink(self) -> None:
        sink = self._trajectory_tick_sink
        close = getattr(sink, "close", None)
        if callable(close):
            close()
        self._trajectory_tick_sink = None

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

    def planner_speed_m_s(self) -> float:
        with self._lock:
            return self._active_envelope.speed_m_s

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
        if new_state == self._state:
            return
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
            first_yaw = path.poses[0].orientation.euler[2]
            robot_yaw = current_odom.orientation.euler[2]
            initial_yaw_error = angle_diff(first_yaw, robot_yaw)
            self._controller.reset_yaw_error(initial_yaw_error)
            angle_in_tolerance = abs(initial_yaw_error) < self._orientation_tolerance
            if angle_in_tolerance:
                position_in_tolerance = (
                    path.poses[0].position.distance(current_odom.position) < 0.01
                )
                if position_in_tolerance:
                    new_state = "final_rotation"
                else:
                    new_state = "path_following"
            elif self._holonomic_yaw_lock_active(current_odom):
                # Holonomic can translate without aligning first; in narrow
                # corridors an initial spin would sweep the body into walls.
                new_state = "path_following"

        with self._lock:
            self._change_state(new_state)

        while not stop_event.is_set():
            start_time = time.perf_counter()

            with self._lock:
                path_clearance.set_planner_speed(self._active_envelope.speed_m_s)
                path_clearance.update_costmap(self._navigation_map.binary_costmap)
                path_clearance.update_pose_index(self._pose_index)

            self._send_navigation_costmap(path, path_clearance)

            if path_clearance.is_obstacle_ahead():
                if self._hot_replan:
                    logger.info("Obstacle detected ahead, requesting hot replan.")
                    self.stopped_navigating.on_next("obstacle_found")
                    self.cmd_vel.on_next(Twist())
                else:
                    logger.info("Obstacle detected ahead, stopping local planner.")
                    self.stopped_navigating.on_next("obstacle_found")
                    break
                elapsed = time.perf_counter() - start_time
                sleep_time = max(0.0, (1.0 / self._control_frequency) - elapsed)
                stop_event.wait(sleep_time)
                continue

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

    def _compute_initial_rotation(self) -> Twist:
        with self._lock:
            path = self._path
            current_odom = self._current_odom

        assert path is not None
        assert current_odom is not None

        if self._holonomic_yaw_lock_active(current_odom):
            with self._lock:
                self._change_state("path_following")
            return self._compute_path_following()

        first_pose = path.poses[0]
        first_yaw = first_pose.orientation.euler[2]
        robot_yaw = current_odom.orientation.euler[2]
        yaw_error = angle_diff(first_yaw, robot_yaw)

        if abs(yaw_error) < self._orientation_tolerance:
            with self._lock:
                self._change_state("path_following")
            return self._compute_path_following()

        self._controller.set_speed(self._active_envelope.speed_m_s)
        measured_body_twist = self._estimate_measured_body_twist(current_odom)
        cmd = self._controller.rotate(yaw_error, current_odom, measured_body_twist)
        ref_pose = _pose_from_xy_yaw(
            float(current_odom.position.x),
            float(current_odom.position.y),
            float(first_yaw),
        )
        self._append_trajectory_control_tick(
            ref_pose,
            Twist(),
            current_odom,
            measured_body_twist,
            cmd,
        )
        return cmd

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

        path_speed = self._path_speed_for_index(path_distancer, closest_index, current_pos)
        self._controller.set_speed(path_speed)
        reference_sample = self._lookahead_reference_sample(
            path_distancer,
            current_odom,
            current_pos,
            path_speed,
            yaw_lock_rad=self._holonomic_yaw_lock_yaw_rad(current_odom),
        )
        measured_body_twist = self._estimate_measured_body_twist(current_odom)
        cmd = self._controller.advance_reference(
            reference_sample,
            current_odom,
            measured_body_twist,
        )
        self._append_trajectory_control_sample(
            reference_sample,
            current_odom,
            measured_body_twist,
            cmd,
        )
        return cmd

    def _lookahead_reference_sample(
        self,
        path_distancer: PathDistancer,
        current_odom: PoseStamped,
        current_pos: np.ndarray,
        path_speed: float,
        *,
        yaw_lock_rad: float | None = None,
    ) -> TrajectoryReferenceSample:
        projection = path_distancer.project(current_pos)
        s_start = float(projection.s_along_path_m)
        s_end = min(
            path_distancer.path_length_m,
            s_start + path_distancer.lookahead_distance_m,
        )
        now_s = float(current_odom.ts)
        if not math.isfinite(now_s):
            now_s = 0.0
        travel_s = max(0.0, s_end - s_start)
        dt_s = 1.0 / self._control_frequency
        duration_s = max(dt_s, travel_s / max(path_speed, 1e-6))
        return self._reference_sample_at_progress(
            path_distancer,
            s_end,
            now_s + duration_s,
            path_speed,
            yaw_lock_rad=yaw_lock_rad,
        )

    def _reference_sample_at_progress(
        self,
        path_distancer: PathDistancer,
        progress_m: float,
        time_s: float,
        path_speed: float,
        *,
        yaw_lock_rad: float | None = None,
    ) -> TrajectoryReferenceSample:
        point = path_distancer.point_at_progress(progress_m)
        path_yaw = path_distancer.yaw_at_progress(progress_m)
        ref_yaw = path_yaw if yaw_lock_rad is None else float(yaw_lock_rad)
        if yaw_lock_rad is None:
            feedforward = Twist(
                linear=Vector3(path_speed, 0.0, 0.0),
                angular=Vector3(0.0, 0.0, 0.0),
            )
        else:
            delta = angle_diff(path_yaw, ref_yaw)
            feedforward = Twist(
                linear=Vector3(
                    path_speed * math.cos(delta),
                    path_speed * math.sin(delta),
                    0.0,
                ),
                angular=Vector3(0.0, 0.0, 0.0),
            )
        return TrajectoryReferenceSample(
            time_s=time_s,
            pose_plan=_pose_from_xy_yaw(float(point[0]), float(point[1]), ref_yaw),
            twist_body=feedforward,
        )

    def _path_speed_for_index(
        self,
        path_distancer: PathDistancer,
        closest_index: int,
        current_pos: np.ndarray,
    ) -> float:
        del closest_index
        self._ensure_path_speed_profile(path_distancer)
        envelope = self._active_envelope
        progress_m = float(path_distancer.project(current_pos).s_along_path_m)
        profile_speed = self._profiled_path_speed_m_s(progress_m)
        distance_cap = math.sqrt(
            max(
                0.0,
                2.0 * envelope.goal_decel_m_s2 * path_distancer.distance_to_goal(current_pos),
            )
        )
        capped = min(envelope.speed_m_s, profile_speed, distance_cap)
        return min(envelope.speed_m_s, max(0.05, capped))

    def _profiled_path_speed_m_s(self, progress_m: float) -> float:
        s_profile = self._path_speed_profile_s
        v_profile = self._path_speed_profile_v
        if s_profile is None or v_profile is None:
            return self._active_envelope.speed_m_s
        return speed_at_progress_m(progress_m, s_profile, v_profile)

    def _ensure_path_speed_profile(self, path_distancer: PathDistancer) -> None:
        path_id = id(path_distancer._path)
        if (
            self._path_speed_profile_s is None
            or self._path_speed_profile_path_id != path_id
        ):
            self._rebuild_path_speed_profile(path_distancer)
            self._path_speed_profile_path_id = path_id

    def _rebuild_path_speed_profile(self, path_distancer: PathDistancer) -> None:
        envelope = self._active_envelope
        s_profile, v_profile = profile_speed_along_polyline(
            path_distancer._path,
            path_distancer._cumulative_dists,
            envelope.path_limits,
            envelope.goal_decel_m_s2,
        )
        self._path_speed_profile_s = s_profile
        self._path_speed_profile_v = v_profile
        self._path_speed_profile_path_id = id(path_distancer._path)

    def _compute_final_rotation(self) -> Twist:
        with self._lock:
            path = self._path
            current_odom = self._current_odom

        assert path is not None
        assert current_odom is not None

        if self._holonomic_yaw_lock_active(current_odom):
            logger.info(
                "Final rotation deferred: position goal reached in narrow corridor",
            )
            with self._lock:
                self._change_state("arrived")
            return Twist()

        goal_yaw = path.poses[-1].orientation.euler[2]
        robot_yaw = current_odom.orientation.euler[2]
        yaw_error = angle_diff(goal_yaw, robot_yaw)

        if abs(yaw_error) < self._orientation_tolerance:
            logger.info("Final rotation complete, goal reached")
            with self._lock:
                self._change_state("arrived")
            return Twist()

        self._controller.set_speed(self._active_envelope.speed_m_s)
        measured_body_twist = self._estimate_measured_body_twist(current_odom)
        cmd = self._controller.rotate(yaw_error, current_odom, measured_body_twist)
        ref_pose = _pose_from_xy_yaw(
            float(current_odom.position.x),
            float(current_odom.position.y),
            float(goal_yaw),
        )
        self._append_trajectory_control_tick(
            ref_pose,
            Twist(),
            current_odom,
            measured_body_twist,
            cmd,
        )
        return cmd

    def _reset_state(self) -> None:
        with self._lock:
            self._change_state("idle")
            self._path = None
            self._path_clearance = None
            self._path_distancer = None
            self._pose_index = 0
            self._previous_odom_for_velocity = None
            self._controller.set_speed(self._active_envelope.speed_m_s)
            self._controller.reset_errors()

    def _append_trajectory_control_tick(
        self,
        reference_pose: Pose,
        reference_twist: Twist,
        current_odom: PoseStamped,
        measured_body_twist: Twist,
        command: Twist,
    ) -> None:
        reference = TrajectoryReferenceSample(
            time_s=float(current_odom.ts),
            pose_plan=reference_pose,
            twist_body=reference_twist,
        )
        self._append_trajectory_control_sample(
            reference,
            current_odom,
            measured_body_twist,
            command,
        )

    def _append_trajectory_control_sample(
        self,
        reference: TrajectoryReferenceSample,
        current_odom: PoseStamped,
        measured_body_twist: Twist,
        command: Twist,
    ) -> None:
        sink = self._trajectory_tick_sink
        if sink is None:
            return
        measurement = TrajectoryMeasuredSample(
            time_s=float(current_odom.ts),
            pose_plan=Pose(current_odom.position, current_odom.orientation),
            twist_body=measured_body_twist,
        )
        append_trajectory_control_tick(
            sink,
            reference,
            measurement,
            command,
            1.0 / self._control_frequency,
            wall_time_s=time.time(),
        )

    def _estimate_measured_body_twist(self, current_odom: PoseStamped) -> Twist:
        previous = self._previous_odom_for_velocity
        self._previous_odom_for_velocity = current_odom
        if previous is None:
            return Twist()
        dt = float(current_odom.ts) - float(previous.ts)
        if not math.isfinite(dt) or dt <= 0.0:
            return Twist()
        vx_w = (float(current_odom.position.x) - float(previous.position.x)) / dt
        vy_w = (float(current_odom.position.y) - float(previous.position.y)) / dt
        yaw = float(current_odom.orientation.euler[2])
        c = math.cos(yaw)
        s = math.sin(yaw)
        vx_b = c * vx_w + s * vy_w
        vy_b = -s * vx_w + c * vy_w
        wz = (
            angle_diff(
                float(current_odom.orientation.euler[2]),
                float(previous.orientation.euler[2]),
            )
            / dt
        )
        return Twist(
            linear=Vector3(vx_b, vy_b, 0.0),
            angular=Vector3(0.0, 0.0, wz),
        )

    def _holonomic_yaw_lock_active(self, current_odom: PoseStamped) -> bool:
        if self._global_config.local_planner_path_controller != "holonomic":
            return False
        return self._holonomic_yaw_lock_yaw_rad(current_odom) is not None

    def _holonomic_yaw_lock_yaw_rad(self, current_odom: PoseStamped) -> float | None:
        if self._global_config.local_planner_path_controller != "holonomic":
            return None
        try:
            costmap = self._navigation_map.binary_costmap
        except ValueError:
            return float(current_odom.orientation.euler[2])
        rotation_radius_m = self._global_config.robot_rotation_diameter / 2.0
        if can_rotate_in_place(costmap, current_odom, rotation_radius_m):
            return None
        return float(current_odom.orientation.euler[2])

    def _send_navigation_costmap(self, path: Path, path_clearance: PathClearance) -> None:
        if "DEBUG_NAVIGATION" not in os.environ:
            return

        now = time.time()
        if now - self._navigation_costmap_last < self._navigation_costmap_interval:
            return

        self._navigation_costmap_last = now

        self.navigation_costmap.on_next(self._navigation_map.gradient_costmap)


def _pose_from_xy_yaw(x: float, y: float, yaw: float) -> Pose:
    return Pose(
        position=Vector3(x, y, 0.0),
        orientation=Quaternion.from_euler(Vector3(0.0, 0.0, float(yaw))),
    )
