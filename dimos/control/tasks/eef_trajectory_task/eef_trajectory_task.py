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

"""Coordinator-clock EEF trajectory playback using the shared bounded Pink solver."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from copy import deepcopy
import math
from typing import TYPE_CHECKING, Any

import attrs
import numpy as np

from dimos.control.task import ControlMode, CoordinatorState, JointCommandOutput
from dimos.control.tasks.eef_trajectory_task.trajectory import EefTrajectory, EefTrajectoryPoint
from dimos.control.tasks.pose_target_ik import (
    FrameTargetSnapshot,
    PinkPoseTargetSolver,
    PoseTargetIKTask,
    PoseTargetIKTaskConfig,
    PoseTargetIKTaskParams,
    string_tuple_converter,
)
from dimos.control.tasks.trajectory_clock import TrajectoryClock
from dimos.control.tasks.trajectory_task.trajectory_task import (
    TrajectoryCancellationResult,
    TrajectoryCancellationStatus,
    TrajectoryExecutionResult,
    TrajectoryExecutionStatus,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState, TrajectoryStatus
from dimos.utils.transform_utils import matrix_to_pose, pose_to_matrix

if TYPE_CHECKING:
    from dimos.control.coordinator import TaskConfig
    from dimos.manipulation.planning.spec.config import RobotModelConfig

EEF_TRAJECTORY_TASK_NAME = "eef_trajectory"


def _positive_finite(_instance: Any, attribute: attrs.Attribute[float], value: float) -> None:
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{attribute.name} must be positive and finite")


@attrs.frozen(slots=False)
class EefTrajectoryTaskConfig(PoseTargetIKTaskConfig):
    """Tool tracking bounds and final measured-feedback completion tolerances."""

    timeout: float = 0.0
    gripper_joint_names: tuple[str, ...] = attrs.field(default=(), converter=string_tuple_converter)
    position_tolerance_m: float = attrs.field(default=0.005, validator=_positive_finite)
    orientation_tolerance_rad: float = attrs.field(default=0.05, validator=_positive_finite)
    gripper_tolerance: float = attrs.field(default=0.02, validator=_positive_finite)
    max_position_error_m: float = attrs.field(default=0.15, validator=_positive_finite)
    max_orientation_error_rad: float = attrs.field(default=1.0, validator=_positive_finite)
    settling_timeout_s: float = attrs.field(default=2.0, validator=_positive_finite)
    start_position_tolerance_m: float = attrs.field(default=0.05, validator=_positive_finite)
    start_orientation_tolerance_rad: float = attrs.field(default=0.35, validator=_positive_finite)
    max_gripper_velocity: float = attrs.field(default=1.0, validator=_positive_finite)


class EefTrajectoryTask(PoseTargetIKTask):
    """Sample all tool targets atomically, solve once and complete from measured FK.

    The coordinator serializes execute/cancel/compute with its task lock. No
    per-task thread, hardware reads or independent wall clock are owned here.
    """

    _config: EefTrajectoryTaskConfig

    def __init__(
        self, config: EefTrajectoryTaskConfig, *, solver: PinkPoseTargetSolver | None = None
    ) -> None:
        self._trajectory: EefTrajectory | None = None
        self._state = TrajectoryState.IDLE
        self._clock = TrajectoryClock()
        self._duration = 0.0
        self._error = ""
        self._estopped = False
        self._gripper_commands: dict[str, float] = {}
        super().__init__(
            EEF_TRAJECTORY_TASK_NAME,
            config,
            additional_claimed_joints=config.gripper_joint_names,
            solver=solver,
        )

    def is_active(self) -> bool:
        return self._state == TrajectoryState.EXECUTING

    def execute(
        self,
        trajectory: EefTrajectory,
        deadman_timeout: float | None = None,
    ) -> TrajectoryExecutionResult:
        """Accept one complete chunk, rejecting replacement while a chunk is active."""
        try:
            clock = TrajectoryClock(deadman_timeout=deadman_timeout)
        except ValueError as exc:
            return TrajectoryExecutionResult(TrajectoryExecutionStatus.INVALID_TRAJECTORY, str(exc))
        if self._estopped:
            return TrajectoryExecutionResult(
                TrajectoryExecutionStatus.ESTOPPED, "EEF task is emergency-stopped"
            )
        if self.is_active():
            return TrajectoryExecutionResult(
                TrajectoryExecutionStatus.ALREADY_EXECUTING, "EEF trajectory is still executing"
            )
        try:
            trajectory = EefTrajectory(deepcopy(trajectory.points), trajectory.timestamp)
            reference_frame = next(iter(trajectory.points[0].poses.values())).frame_id
            if reference_frame != "world":
                if reference_frame != self._config.robot_model.base_link:
                    raise ValueError(
                        "EEF reference frame must be world or the configured robot model base_link"
                    )
                base_world = pose_to_matrix(self._config.robot_model.base_pose)
                points = []
                for point in trajectory.points:
                    poses = {}
                    for name, target in point.poses.items():
                        world = matrix_to_pose(base_world @ pose_to_matrix(target))
                        poses[name] = PoseStamped(
                            ts=0.0,
                            frame_id="world",
                            position=world.position,
                            orientation=world.orientation,
                        )
                    points.append(EefTrajectoryPoint(point.time_from_start, poses, point.grippers))
                trajectory = EefTrajectory(points, trajectory.timestamp)
            first = trajectory.points[0]
            if set(first.poses) != set(self._config.target_frames):
                raise ValueError("EEF frames must match configured target frames")
            if set(first.grippers) != set(self._additional_claimed_joints):
                raise ValueError("EEF grippers must match configured normalized gripper joints")
        except (AttributeError, TypeError, ValueError) as exc:
            return TrajectoryExecutionResult(TrajectoryExecutionStatus.INVALID_TRAJECTORY, str(exc))
        state = self.context.get_state()
        if state is None:
            return TrajectoryExecutionResult(
                TrajectoryExecutionStatus.START_STATE_UNAVAILABLE,
                "EEF start requires a complete tick observation",
            )
        current_positions = state.joints.joint_positions
        names = self.claim().joints
        if any(
            name not in current_positions or not math.isfinite(current_positions[name])
            for name in names
        ):
            return TrajectoryExecutionResult(
                TrajectoryExecutionStatus.START_STATE_UNAVAILABLE,
                "EEF start requires complete finite tick feedback",
            )
        if any(not 0 <= current_positions[name] <= 1 for name in self._additional_claimed_joints):
            return TrajectoryExecutionResult(
                TrajectoryExecutionStatus.START_STATE_UNAVAILABLE,
                "EEF start gripper feedback must be normalized",
            )
        try:
            poses = self.measured_frame_poses(state)
            position_error, rotation_error = self._pose_errors(first.poses, poses)
        except Exception as exc:
            return TrajectoryExecutionResult(
                TrajectoryExecutionStatus.START_STATE_UNAVAILABLE, str(exc)
            )
        if (
            position_error > self._config.start_position_tolerance_m
            or rotation_error > self._config.start_orientation_tolerance_rad
        ):
            return TrajectoryExecutionResult(
                TrajectoryExecutionStatus.START_STATE_MISMATCH,
                "EEF anchor differs from measured tool pose",
            )
        if any(
            abs(first.grippers[name] - current_positions[name]) > self._config.gripper_tolerance
            for name in first.grippers
        ):
            return TrajectoryExecutionResult(
                TrajectoryExecutionStatus.START_STATE_MISMATCH,
                "EEF gripper anchor differs from measured opening",
            )
        self._reset_command_state()
        self._trajectory = trajectory
        self._clock = clock
        self._duration = trajectory.duration
        self._gripper_commands = {name: current_positions[name] for name in first.grippers}
        self._error = ""
        self._state = TrajectoryState.EXECUTING
        return TrajectoryExecutionResult(TrajectoryExecutionStatus.ACCEPTED)

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        if not self.is_active():
            return None
        if not math.isfinite(state.t_now) or not math.isfinite(state.dt) or state.dt <= 0:
            self._finish(TrajectoryState.FAULT, "Invalid coordinator tick time")
            return None
        if any(
            state.joints.get_position(name) is None
            or not math.isfinite(state.joints.joint_positions[name])
            for name in self.claim().joints
        ):
            self._finish(TrajectoryState.FAULT, "Missing or non-finite joint feedback")
            return None
        first_tick = self._clock.start_time is None
        if (
            self._clock.start_time is not None
            and state.t_now < self._clock.start_time + self._clock.elapsed
        ):
            self._finish(TrajectoryState.FAULT, "Coordinator time moved backwards")
            return None
        self._clock.update(state.t_now)
        if self._clock.expired(state.t_now):
            self._finish(TrajectoryState.ABORTED, "Trajectory deadman expired")
            return None
        trajectory = self._trajectory
        assert trajectory is not None
        desired = trajectory.sample(self._clock.elapsed)
        try:
            measured = self.measured_frame_poses(state)
            position_error, rotation_error = self._pose_errors(desired.poses, measured)
            if first_tick and (
                position_error > self._config.start_position_tolerance_m
                or rotation_error > self._config.start_orientation_tolerance_rad
            ):
                raise ValueError("EEF anchor changed before the first execution tick")
            if (
                position_error > self._config.max_position_error_m
                or rotation_error > self._config.max_orientation_error_rad
            ):
                raise ValueError("EEF tracking error exceeds configured bound")
            gripper_errors = []
            for name, opening in desired.grippers.items():
                value = state.joints.get_position(name)
                if value is None or not math.isfinite(value) or not 0 <= value <= 1:
                    raise ValueError(f"Missing or invalid normalized feedback for {name}")
                error = abs(value - opening)
                if first_tick and error > self._config.gripper_tolerance:
                    raise ValueError("EEF gripper anchor changed before the first execution tick")
                gripper_errors.append(error)
            if self._clock.elapsed >= trajectory.duration:
                reached = (
                    position_error <= self._config.position_tolerance_m
                    and rotation_error <= self._config.orientation_tolerance_rad
                    and all(error <= self._config.gripper_tolerance for error in gripper_errors)
                )
                if reached:
                    # End at measured feedback rather than emit a lagging solver
                    # command after feedback has already reached the target.
                    names = [*self._joint_names, *self._additional_claimed_joints]
                    hold = JointCommandOutput(
                        joint_names=names,
                        positions=[state.joints.joint_positions[name] for name in names],
                        mode=ControlMode.SERVO_POSITION,
                    )
                    self._finish(TrajectoryState.COMPLETED)
                    return hold
                if self._clock.elapsed > trajectory.duration + self._config.settling_timeout_s:
                    raise ValueError(
                        "EEF final target did not reach measured completion tolerances"
                    )
            snapshot = self._frame_target_snapshot(state, desired)
            assert snapshot is not None
            output = self._compute_snapshot(state, snapshot)
            if output is None:
                raise ValueError("Bounded Pink IK produced no valid command")
            return output
        except Exception as exc:
            self._finish(TrajectoryState.FAULT, str(exc))
            return None

    def _frame_target_snapshot(
        self, state: CoordinatorState, point: EefTrajectoryPoint | None = None
    ) -> FrameTargetSnapshot | None:
        if not self.is_active() or self._trajectory is None:
            return None
        if point is None:
            point = self._trajectory.sample(self._clock.elapsed)
        openings = {}
        for name, target in point.grippers.items():
            previous = self._gripper_commands[name]
            limit = self._config.max_gripper_velocity * state.dt
            openings[name] = previous + max(-limit, min(limit, target - previous))
        self._gripper_commands = openings
        return FrameTargetSnapshot(point.poses, state.t_now, openings)

    @staticmethod
    def _pose_errors(
        targets: Mapping[str, PoseStamped], measured: Mapping[str, PoseStamped]
    ) -> tuple[float, float]:
        if set(targets) != set(measured):
            raise ValueError("Missing measured tool frame poses")
        max_position = max_rotation = 0.0
        for frame, target in targets.items():
            actual = measured[frame]
            values = [
                actual.x,
                actual.y,
                actual.z,
                actual.orientation.x,
                actual.orientation.y,
                actual.orientation.z,
                actual.orientation.w,
            ]
            if not np.isfinite(values).all():
                raise ValueError("Measured EEF pose is not finite")
            max_position = max(
                max_position,
                math.dist([target.x, target.y, target.z], [actual.x, actual.y, actual.z]),
            )
            a, b = target.orientation, actual.orientation
            dot = abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w)
            angle = 2 * math.acos(max(-1.0, min(1.0, dot)))
            if not math.isfinite(max_position) or not math.isfinite(angle):
                raise ValueError("Measured EEF pose is not finite")
            max_rotation = max(max_rotation, angle)
        return max_position, max_rotation

    def _finish(self, state: TrajectoryState, error: str = "") -> None:
        self._state = state
        self._error = error
        self._trajectory = None
        self._gripper_commands.clear()
        self._reset_command_state()

    def cancel(self) -> TrajectoryCancellationResult:
        if not self.is_active():
            return TrajectoryCancellationResult(TrajectoryCancellationStatus.ALREADY_STOPPED)
        self._finish(TrajectoryState.ABORTED)
        return TrajectoryCancellationResult(TrajectoryCancellationStatus.CANCELLED)

    def set_estop(self, estopped: bool) -> None:
        self._estopped = estopped
        if estopped:
            self.cancel()

    def _on_pose_target_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        self._finish(TrajectoryState.ABORTED, f"Preempted by {by_task}")

    def reset_runtime_state(self, reactivate: bool | None = None) -> bool:
        had_state = self.is_active() or self._state == TrajectoryState.FAULT
        self._finish(TrajectoryState.IDLE)
        self._clock = TrajectoryClock()
        self._duration = 0.0
        return had_state

    def keepalive(self) -> TrajectoryStatus:
        """Refresh optional caller supervision and return last-tick execution status."""
        if self.is_active():
            self._clock.keepalive()
        return self.get_status()

    def get_state(self) -> TrajectoryState:
        return self._state

    def get_status(self, t_now: float | None = None) -> TrajectoryStatus:
        """Report progress of the last compute tick; the caller cannot advance time."""
        progress, elapsed, remaining = self._clock.sample(self._duration)
        return TrajectoryStatus(
            state=self._state,
            progress=progress,
            time_elapsed=elapsed,
            time_remaining=remaining if self.is_active() else 0.0,
            error=self._error,
            timestamp=0.0,
        )


class EefTrajectoryTaskParams(PoseTargetIKTaskParams):
    target_frames: list[str]
    timeout: float = 0.0
    gripper_joint_names: list[str] = []
    position_tolerance_m: float = 0.005
    orientation_tolerance_rad: float = 0.05
    gripper_tolerance: float = 0.02
    max_position_error_m: float = 0.15
    max_orientation_error_rad: float = 1.0
    settling_timeout_s: float = 2.0
    start_position_tolerance_m: float = 0.05
    start_orientation_tolerance_rad: float = 0.35
    max_gripper_velocity: float = 1.0


def eef_trajectory_task(
    joint_names: Sequence[str],
    robot_model: RobotModelConfig,
    target_frames: Sequence[str],
    gripper_joint_names: Sequence[str] = (),
    priority: int = 10,
    **params: Any,
) -> TaskConfig:
    """Configure the coordinator's canonical EEF chunk playback task."""
    from dimos.control.coordinator import TaskConfig

    return TaskConfig(
        name=EEF_TRAJECTORY_TASK_NAME,
        type=EEF_TRAJECTORY_TASK_NAME,
        joint_names=list(joint_names),
        priority=priority,
        params={
            "robot_model": robot_model,
            "target_frames": list(target_frames),
            "gripper_joint_names": list(gripper_joint_names),
            **params,
        },
    )


def create_task(cfg: TaskConfig, hardware: Any) -> EefTrajectoryTask:
    if cfg.name != EEF_TRAJECTORY_TASK_NAME:
        raise ValueError("EEF trajectory task must use canonical name eef_trajectory")
    params = EefTrajectoryTaskParams.model_validate(cfg.params)
    fields = params.model_dump(exclude={"robot_model", "pink"})
    return EefTrajectoryTask(
        EefTrajectoryTaskConfig(
            joint_names=tuple(cfg.joint_names),
            robot_model=params.robot_model,
            pink=params.pink,
            priority=cfg.priority,
            **fields,
        )
    )
