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

"""Holonomic trajectory controller.

``DanHolonomicTC`` follows a planned ``Path`` with the holonomic tracking law.
It owns trajectory control only: the planner (``MLSPlannerNative``) owns route
safety and emits the path, sending an empty ``Path`` when nothing ahead is
traversable. The costmap, obstacle, and replanning concerns of the old
``LocalPlanner`` are gone; the stripped control core (``_HolonomicPathFollower``)
keeps the state machine, holonomic tracking, and run-profile envelope.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from threading import Event, RLock, Thread, current_thread
import time
import traceback
from typing import Any, Literal, TypeAlias

from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]
import numpy as np
from reactivex import Subject
from reactivex.disposable import Disposable

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.base import NavigationState
from dimos.navigation.dannav.geometry.path_distancer import PathDistancer
from dimos.navigation.dannav.geometry.path_speed_profile import (
    PathSpeedProfileLimits,
    profile_speed_along_polyline,
    speed_at_progress_m,
)
from dimos.navigation.dannav.holonomic_tc.holonomic_path_controller import (
    HolonomicPathController,
    _pose_from_xy_yaw,
)
from dimos.navigation.dannav.holonomic_tc.run_profiles import (
    GO2_RUN_PROFILES,
    RunProfile,
    RunProfileError,
)
from dimos.navigation.dannav.holonomic_tc.types import (
    TrajectoryReferenceSample,
)
from dimos.utils.logging_config import setup_logger
from dimos.utils.trigonometry import angle_diff

PlannerState: TypeAlias = Literal[
    "idle", "initial_rotation", "path_following", "final_rotation", "arrived"
]
# Only the two terminal reasons the controller still owns survive; the
# planner-oriented reasons (obstacle_found, map_updated, run_envelope_rejected)
# are gone with the costmap.
StopMessage: TypeAlias = Literal["arrived", "error"]

logger = setup_logger()


@dataclass(frozen=True)
class ActiveRunEnvelope:
    """Resolved movement envelope for the current session profile."""

    profile: RunProfile
    speed_m_s: float
    path_limits: PathSpeedProfileLimits
    goal_decel_m_s2: float


class _HolonomicPathFollower:
    """Constructible control core for :class:`DanHolonomicTC`.

    Owns the follow state machine, the holonomic tracking law, and the
    run-profile speed/accel envelope. Has no transport: ``DanHolonomicTC`` wires
    the ``path`` / ``odometry`` / ``stop_movement`` streams to its methods and
    forwards ``cmd_vel`` / ``stopped_navigating``.
    """

    cmd_vel: Subject[Twist]
    stopped_navigating: Subject[StopMessage]

    _thread: Thread | None = None
    _path: Path | None = None
    _path_distancer: PathDistancer | None = None
    _current_odom: PoseStamped | None = None

    _lock: RLock
    _stop_planning_event: Event
    _state: PlannerState
    _global_config: GlobalConfig
    _goal_tolerance: float
    _controller: HolonomicPathController
    _previous_odom_for_velocity: PoseStamped | None

    _run_profile: str
    _cruise_speed_override: float | None
    _active_envelope: ActiveRunEnvelope
    _path_speed_profile_s: list[float] | None
    _path_speed_profile_v: list[float] | None
    _path_speed_profile_path_id: int | None
    _control_frequency: float
    _orientation_tolerance: float
    _align_heading_before_move: bool
    _align_goal_yaw: bool

    def __init__(self, config: DanHolonomicTCConfig) -> None:
        self.cmd_vel = Subject()
        self.stopped_navigating = Subject()

        self._config = config
        self._global_config = config.g
        self._lock = RLock()
        self._stop_planning_event = Event()
        self._state = "idle"
        self._goal_tolerance = float(config.goal_tolerance)
        self._orientation_tolerance = float(config.orientation_tolerance)
        self._control_frequency = float(config.control_frequency)
        self._align_heading_before_move = bool(config.align_heading_before_move)
        self._align_goal_yaw = bool(config.align_goal_yaw)
        self._run_profile = config.run_profile
        self._previous_odom_for_velocity = None
        self._path_speed_profile_s = None
        self._path_speed_profile_v = None
        self._path_speed_profile_path_id = None

        override = config.speed_m_s
        if override is not None and (not math.isfinite(override) or override <= 0.0):
            raise ValueError(f"speed_m_s must be a positive finite float, got {override!r}")
        self._cruise_speed_override = override

        envelope = self._resolve_run_envelope()
        if envelope is None:
            raise ValueError(f"invalid run profile {self._run_profile!r}")
        self._active_envelope = envelope

        self._controller = HolonomicPathController(
            self._global_config,
            envelope.profile,
            envelope.speed_m_s,
            self._control_frequency,
            k_position_per_s=config.k_position_per_s,
            k_yaw_per_s=config.k_yaw_per_s,
            k_velocity_per_s=config.k_velocity_per_s,
            k_yaw_rate_per_s=config.k_yaw_rate_per_s,
        )
        self._apply_run_envelope(envelope)

    # lifecycle

    def close(self) -> None:
        self.stop_planning()

    def handle_odom(self, msg: PoseStamped) -> None:
        with self._lock:
            self._current_odom = msg

    def start_planning(self, path: Path) -> None:
        self.stop_planning()

        with self._lock:
            self._stop_planning_event = Event()
            self._path = path
            self._path_distancer = PathDistancer(path)
            self._previous_odom_for_velocity = None
            self._rebuild_path_speed_profile(self._path_distancer)
            self._thread = Thread(
                target=self._thread_entrypoint,
                args=(self._stop_planning_event,),
                daemon=True,
            )
            self._thread.start()

    def update_path(self, path: Path) -> bool:
        """Swap the active path without stopping the control thread."""
        if not path.poses:
            return False

        with self._lock:
            if self._path is None or self._thread is None:
                return False

            self._path = path
            self._path_distancer = PathDistancer(path)
            self._rebuild_path_speed_profile(self._path_distancer)

        return True

    def stop_planning(self) -> None:
        with self._lock:
            thread = self._thread
            self._thread = None
            self._stop_planning_event.set()

        if thread is not None and thread is not current_thread():
            thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            if thread.is_alive():
                logger.warning("Holonomic control thread did not exit within join timeout.")

        self.cmd_vel.on_next(Twist())
        self._reset_state()

    # run-profile envelope

    def set_run_profile(self, profile: str) -> bool:
        """Set the session-default movement envelope and apply it live.

        Validated against the run-profile registry so a bad name cannot poison
        the controller. The new envelope takes effect mid-follow.
        """
        try:
            GO2_RUN_PROFILES.get(profile)
        except RunProfileError as exc:
            logger.warning("Rejected run profile.", profile=profile, reason=str(exc))
            return False
        self._run_profile = profile
        envelope = self._resolve_run_envelope()
        if envelope is None:
            return False
        self._apply_run_envelope(envelope)
        return True

    def _profile_run_envelope(self, profile: RunProfile) -> ActiveRunEnvelope:
        speed = (
            self._cruise_speed_override
            if self._cruise_speed_override is not None
            else profile.requested_planner_speed_m_s
        )
        return ActiveRunEnvelope(
            profile=profile,
            speed_m_s=speed,
            path_limits=profile.path_speed_profile_limits_at(speed),
            goal_decel_m_s2=profile.goal_decel_m_s2,
        )

    def _resolve_run_envelope(self) -> ActiveRunEnvelope | None:
        """Resolve the movement envelope from the current session profile."""
        name = self._run_profile
        try:
            profile = GO2_RUN_PROFILES.get(name)
        except RunProfileError as exc:
            logger.warning("run profile rejected", profile=name, reason=str(exc))
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
        self._controller.set_profile(envelope.profile)
        self._controller.set_speed(envelope.speed_m_s)
        with self._lock:
            if self._path_distancer is not None:
                self._rebuild_path_speed_profile(self._path_distancer)

    # introspection

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

    # control loop

    def _thread_entrypoint(self, stop_event: Event) -> None:
        try:
            self._loop(stop_event)
        except Exception as e:
            traceback.print_exc()
            logger.exception("Error in holonomic trajectory control", exc_info=e)
            self.stopped_navigating.on_next("error")
        finally:
            with self._lock:
                owns_state = self._thread is current_thread()
                if owns_state:
                    self._thread = None
                    self._reset_state()
            if owns_state:
                self.cmd_vel.on_next(Twist())

    def _change_state(self, new_state: PlannerState) -> None:
        if new_state == self._state:
            return
        self._state = new_state
        logger.info("changed state", state=new_state)

    def _initial_state(self, path: Path, current_odom: PoseStamped | None) -> PlannerState:
        """Decide where the follow starts.

        A holonomic base translates while turning toward the path tangent, so by
        default a non-empty path goes straight to ``path_following``. Rotate
        first only when ``align_heading_before_move`` is set and the start is
        misaligned with the first path tangent.
        """
        if not self._align_heading_before_move or current_odom is None or len(path.poses) == 0:
            return "path_following"

        first_yaw = path.poses[0].orientation.euler[2]
        robot_yaw = current_odom.orientation.euler[2]
        initial_yaw_error = angle_diff(first_yaw, robot_yaw)
        if abs(initial_yaw_error) < self._orientation_tolerance:
            return "path_following"
        return "initial_rotation"

    def _loop(self, stop_event: Event) -> None:
        with self._lock:
            path = self._path
            current_odom = self._current_odom

        if path is None:
            raise RuntimeError("No path set for holonomic path follower.")

        with self._lock:
            self._change_state(self._initial_state(path, current_odom))

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
            elif state == "arrived":
                # Stop motion before signalling arrival, matching the path
                # followers downstream consumers expect.
                self.cmd_vel.on_next(Twist())
                self.stopped_navigating.on_next("arrived")
                break
            else:  # idle
                cmd_vel = None

            if cmd_vel is not None:
                self.cmd_vel.on_next(cmd_vel)

            elapsed = time.perf_counter() - start_time
            sleep_time = max(0.0, (1.0 / self._control_frequency) - elapsed)
            stop_event.wait(sleep_time)

        if stop_event.is_set():
            logger.info("Holonomic path follower loop exited due to stop event.")

    def _compute_initial_rotation(self) -> Twist:
        with self._lock:
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
                self._change_state("path_following")
            return self._compute_path_following()

        self._controller.set_speed(self._active_envelope.speed_m_s)
        measured_body_twist = self._estimate_measured_body_twist(current_odom)
        return self._controller.rotate(yaw_error, current_odom, measured_body_twist)

    def _compute_path_following(self) -> Twist:
        with self._lock:
            path_distancer = self._path_distancer
            current_odom = self._current_odom

        assert path_distancer is not None
        assert current_odom is not None

        current_pos = np.array([current_odom.position.x, current_odom.position.y])

        if path_distancer.distance_to_goal(current_pos) < self._goal_tolerance:
            if self._align_goal_yaw:
                logger.info("Reached goal position, starting final rotation")
                with self._lock:
                    self._change_state("final_rotation")
                return self._compute_final_rotation()
            logger.info("Reached goal position")
            with self._lock:
                self._change_state("arrived")
            return Twist()

        path_speed = self._path_speed_at_position(path_distancer, current_pos)
        self._controller.set_speed(path_speed)
        reference_sample = self._lookahead_reference_sample(
            path_distancer,
            current_odom,
            current_pos,
            path_speed,
        )
        measured_body_twist = self._estimate_measured_body_twist(current_odom)
        return self._controller.advance_reference(
            reference_sample,
            current_odom,
            measured_body_twist,
        )

    def _lookahead_reference_sample(
        self,
        path_distancer: PathDistancer,
        current_odom: PoseStamped,
        current_pos: np.ndarray,
        path_speed: float,
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
        )

    def _reference_sample_at_progress(
        self,
        path_distancer: PathDistancer,
        progress_m: float,
        time_s: float,
        path_speed: float,
    ) -> TrajectoryReferenceSample:
        point = path_distancer.point_at_progress(progress_m)
        # Body yaw tracks the path tangent; the holonomic law translates toward
        # the reference while turning. No costmap yaw-lock in this stack.
        path_yaw = path_distancer.yaw_at_progress(progress_m)
        feedforward = Twist(
            linear=Vector3(path_speed, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, 0.0),
        )
        return TrajectoryReferenceSample(
            time_s=time_s,
            pose_plan=_pose_from_xy_yaw(float(point[0]), float(point[1]), path_yaw),
            twist_body=feedforward,
        )

    def _path_speed_at_position(
        self,
        path_distancer: PathDistancer,
        current_pos: np.ndarray,
    ) -> float:
        envelope = self._active_envelope
        progress_m = float(path_distancer.project(current_pos).s_along_path_m)
        with self._lock:
            self._ensure_path_speed_profile(path_distancer)
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
        if self._path_speed_profile_s is None or self._path_speed_profile_path_id != path_id:
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
        return self._controller.rotate(yaw_error, current_odom, measured_body_twist)

    def _reset_state(self) -> None:
        with self._lock:
            self._change_state("idle")
            self._path = None
            self._path_distancer = None
            self._previous_odom_for_velocity = None
            self._controller.set_speed(self._active_envelope.speed_m_s)
            self._controller.reset_errors()

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


class DanHolonomicTCConfig(ModuleConfig):
    control_frequency: float = 10.0
    run_profile: str = "walk"
    speed_m_s: float | None = None
    goal_tolerance: float = 0.2
    orientation_tolerance: float = 0.35
    k_position_per_s: float = 2.0
    k_yaw_per_s: float = 1.0
    k_velocity_per_s: float = 0.5
    k_yaw_rate_per_s: float = 1.0
    align_heading_before_move: bool = False
    align_goal_yaw: bool = False


class DanHolonomicTC(Module):
    """Follow a planned ``Path`` with the holonomic tracking law.

    Consumes the robot's ``PoseStamped`` odom directly. The planner owns route
    safety and emits the ``Path``: an empty path stops the follow, a non-empty
    path updates the active route or starts a new follow. Publishes
    ``nav_cmd_vel`` until the goal is within tolerance, then ``goal_reached``.
    ``stop_movement`` cancels the current path.
    """

    config: DanHolonomicTCConfig

    path: In[Path]
    odom: In[PoseStamped]
    stop_movement: In[Bool]

    nav_cmd_vel: Out[Twist]
    goal_reached: Out[Bool]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._core = _HolonomicPathFollower(self.config)

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(self._core.cmd_vel.subscribe(self.nav_cmd_vel.publish))
        self.register_disposable(self._core.stopped_navigating.subscribe(self._on_core_stopped))
        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))
        self.register_disposable(Disposable(self.path.subscribe(self._on_path)))
        if self.stop_movement.transport is not None:
            self.register_disposable(Disposable(self.stop_movement.subscribe(self._on_stop)))

    @rpc
    def stop(self) -> None:
        self._core.close()
        self.nav_cmd_vel.publish(Twist())
        super().stop()

    def _on_odom(self, msg: PoseStamped) -> None:
        self._core.handle_odom(msg)

    def _on_path(self, path: Path) -> None:
        # The planner owns path safety: it sends the route as far as it is safe,
        # or an empty path when nothing ahead is traversable.
        if len(path.poses) == 0:
            self._core.stop_planning()
            return
        if not self._core.update_path(path):
            self._core.start_planning(path)

    def _on_stop(self, msg: Bool) -> None:
        if msg.data:
            self._core.stop_planning()

    def _on_core_stopped(self, msg: StopMessage) -> None:
        if msg == "arrived":
            self.goal_reached.publish(Bool(True))
            logger.info("Goal reached")
        # On "error" the core has already published a zero Twist and cleared the
        # route; there is no planner here to ask for a replan.

    @rpc
    def set_run_profile(self, profile: str) -> bool:
        """Set the session-default movement envelope, applied live."""
        return self._core.set_run_profile(profile)

    @rpc
    def get_state(self) -> NavigationState:
        return self._core.get_state()
