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

"""Joint trajectory task for the ControlCoordinator.

Passive trajectory execution - called by coordinator each tick.
Unlike JointTrajectoryController which owns a thread, this task
is compute-only and relies on the coordinator for timing.

"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from enum import Enum, auto
import math
from typing import TYPE_CHECKING, Annotated, Any

from pydantic import BeforeValidator, ConfigDict, Field
from pydantic.dataclasses import dataclass as pydantic_dataclass

from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint
from dimos.msgs.trajectory_msgs.TrajectoryStatus import TrajectoryState, TrajectoryStatus
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.control.coordinator import TaskConfig

logger = setup_logger()

JOINT_TRAJECTORY_TASK_NAME = "joint_trajectory"


def joint_trajectory_task(
    joint_names: Sequence[str],
    priority: int = 10,
    start_position_tolerance: float = 0.05,
    velocity_limits: Mapping[str, float] | None = None,
) -> TaskConfig:
    """Build the coordinator's single canonical joint-trajectory task."""
    # The coordinator imports this module to recognize the canonical JTT.
    from dimos.control.coordinator import TaskConfig

    params: dict[str, Any] = {"start_position_tolerance": start_position_tolerance}
    if velocity_limits is not None:
        params["velocity_limits"] = dict(velocity_limits)
    return TaskConfig(
        name=JOINT_TRAJECTORY_TASK_NAME,
        type="trajectory",
        joint_names=list(joint_names),
        priority=priority,
        params=params,
    )


class TrajectoryExecutionStatus(Enum):
    """Semantic outcome of a trajectory execution request."""

    ACCEPTED = auto()
    NO_TRAJECTORY_TASK = auto()
    INVALID_TRAJECTORY = auto()
    START_STATE_UNAVAILABLE = auto()
    START_STATE_MISMATCH = auto()
    ALREADY_EXECUTING = auto()


@dataclass(frozen=True)
class TrajectoryExecutionResult:
    """Result returned by the coordinator trajectory execution RPC."""

    status: TrajectoryExecutionStatus
    message: str = ""


class TrajectoryCancellationStatus(Enum):
    """Semantic outcome of a trajectory cancellation request."""

    CANCELLED = auto()
    ALREADY_STOPPED = auto()
    NO_TRAJECTORY_TASK = auto()
    UNCERTAIN = auto()


@dataclass(frozen=True)
class TrajectoryCancellationResult:
    """Result returned by the coordinator trajectory cancellation RPC."""

    status: TrajectoryCancellationStatus
    message: str = ""

    @property
    def safe(self) -> bool:
        """Return whether cancellation reached a deterministic coordinator state."""
        return self.status is not TrajectoryCancellationStatus.UNCERTAIN

    @property
    def cancelled(self) -> bool:
        """Return whether an active trajectory was cancelled."""
        return self.status is TrajectoryCancellationStatus.CANCELLED


def _to_joint_names(value: Sequence[str]) -> tuple[str, ...]:
    return tuple(value)


@pydantic_dataclass(
    frozen=True,
    config=ConfigDict(extra="forbid", validate_default=True),
)
class JointTrajectoryTaskConfig:
    """Configuration for trajectory task.

    Attributes:
        joint_names: List of joint names this task controls
        priority: Priority for arbitration (higher wins)
        start_position_tolerance: Maximum difference between current joint
            position and the first trajectory point.
        velocity_limits: Optional positive velocity limit for every configured
            joint. Defaults to 1 rad/s per joint.
    """

    joint_names: Annotated[
        tuple[Annotated[str, Field(min_length=1)], ...],
        BeforeValidator(_to_joint_names),
    ] = Field(min_length=1)
    priority: int = Field(default=10, strict=True)
    start_position_tolerance: float = Field(
        default=0.05,
        ge=0.0,
        allow_inf_nan=False,
    )
    velocity_limits: dict[str, float] | None = None


@dataclass
class _TrajectoryRun:
    trajectory: JointTrajectory
    start_time: float | None = None


class JointTrajectoryTask(BaseControlTask):
    """Passive trajectory execution task.

    Unlike JointTrajectoryController which owns a thread, this task
    is called by the coordinator at each tick.

    State Machine:
        IDLE ──execute()──► EXECUTING ──done──► COMPLETED
          ▲                     │                    │
          │                  cancel()             reset()
          │                     ▼                    │
          └─────reset()───── ABORTED ◄──────────────┘

    Example:
        >>> task = JointTrajectoryTask(
        ...     config=JointTrajectoryTaskConfig(
        ...         joint_names=["left/joint1", "left/joint2"],
        ...         priority=10,
        ...     ),
        ... )
        >>> coordinator.add_task(task)
        >>> task.execute(my_trajectory, current_positions)
    """

    def __init__(self, config: JointTrajectoryTaskConfig) -> None:
        """Initialize trajectory task.

        Args:
            config: Task configuration
        """
        self._name = JOINT_TRAJECTORY_TASK_NAME
        self._config = config
        self._joint_names = frozenset(config.joint_names)
        self._joint_names_list = list(config.joint_names)

        # State machine
        self._state = TrajectoryState.IDLE
        self._trajectory: JointTrajectory | None = None
        self._motions: dict[str, tuple[_TrajectoryRun, int]] = {}
        self._commanded_positions: dict[str, float] = {}
        self._start_time: float = 0.0
        self._pending_start: bool = False  # Defer start time to first compute()
        self._last_duration: float = 0.0
        self._last_elapsed: float = 0.0

        configured_limits = config.velocity_limits
        if configured_limits is None:
            self._velocity_limits = {name: 1.0 for name in config.joint_names}
        else:
            if set(configured_limits) != self._joint_names:
                raise ValueError("velocity_limits must name every configured trajectory joint")
            if any(not math.isfinite(value) or value <= 0.0 for value in configured_limits.values()):
                raise ValueError("velocity_limits must be finite and positive")
            self._velocity_limits = dict(configured_limits)

        logger.info(
            f"JointTrajectoryTask {self._name} initialized for joints: {config.joint_names}"
        )

    def claim(self) -> ResourceClaim:
        """Declare resource requirements."""
        return ResourceClaim(
            joints=self._joint_names,
            priority=self._config.priority,
            mode=ControlMode.SERVO_POSITION,
        )

    def is_active(self) -> bool:
        """Check if task should run this tick."""
        return self._state == TrajectoryState.EXECUTING

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        """Compute trajectory output for this tick.

        CRITICAL: Uses state.t_now for timing, NOT time.time()!

        Args:
            state: Current coordinator state

        Returns:
            JointCommandOutput with positions, or None if not executing
        """
        if not self._motions:
            return None

        output_names = [name for name in self._joint_names_list if name in self._motions]
        all_complete = bool(self._motions)
        for joint_name, (run, index) in list(self._motions.items()):
            if run.start_time is None:
                run.start_time = state.t_now
                if run.trajectory is self._trajectory:
                    self._start_time = state.t_now
                    self._pending_start = False
            elapsed = max(0.0, state.t_now - run.start_time)
            self._last_elapsed = max(self._last_elapsed, elapsed)
            desired = run.trajectory.sample(elapsed)[0][index]
            current = self._commanded_positions.get(joint_name)
            if current is None:
                current = state.joints.get_position(joint_name)
                if current is None or not math.isfinite(current):
                    all_complete = False
                    continue
            max_delta = self._velocity_limits[joint_name] * max(0.0, state.dt)
            delta = max(-max_delta, min(max_delta, desired - current))
            commanded = current + delta
            self._commanded_positions[joint_name] = commanded

            final_position = run.trajectory.points[-1].positions[index]
            nominal_complete = elapsed >= run.trajectory.duration
            reached = math.isclose(commanded, final_position, abs_tol=1e-9)
            if nominal_complete and reached:
                del self._motions[joint_name]
            else:
                all_complete = False

        if all_complete and not self._motions and self._state == TrajectoryState.EXECUTING:
            self._state = TrajectoryState.COMPLETED
            self._trajectory = None
            self._pending_start = False
            logger.info("Trajectory completed", task_name=self._name)

        emitted_names = [name for name in output_names if name in self._commanded_positions]
        if not emitted_names:
            return None
        return JointCommandOutput(
            joint_names=emitted_names,
            positions=[self._commanded_positions[name] for name in emitted_names],
            mode=ControlMode.SERVO_POSITION,
        )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        """Handle preemption by higher-priority task.

        Args:
            by_task: Name of preempting task
            joints: Joints that were preempted
        """
        logger.warning(f"Trajectory {self._name} preempted by {by_task} on joints {joints}")
        # Abort if any of our joints were preempted
        if joints & self._joint_names:
            self._state = TrajectoryState.ABORTED
            self._clear_active_trajectory()

    def _clear_active_trajectory(self) -> None:
        """Clear stored trajectory-specific execution state."""
        self._trajectory = None
        self._motions.clear()
        self._commanded_positions.clear()
        self._pending_start = False
        self._start_time = 0.0

    def _validate_trajectory(self, trajectory: JointTrajectory) -> bool:
        """Validate a trajectory before execution."""
        joint_names = list(trajectory.joint_names)
        if not joint_names:
            logger.warning("Trajectory for %s has empty joint names", self._name)
            return False
        if len(set(joint_names)) != len(joint_names):
            logger.warning("Trajectory for %s has duplicate joint names", self._name)
            return False
        unknown = [name for name in joint_names if name not in self._joint_names]
        if unknown:
            logger.warning("Trajectory for %s has unknown joints: %s", self._name, unknown)
            return False
        if not trajectory.points:
            logger.warning("Empty trajectory for %s", self._name)
            return False
        width = len(joint_names)
        previous_time: float | None = None
        for index, point in enumerate(trajectory.points):
            if len(point.positions) != width or len(point.velocities) != width:
                logger.warning("Trajectory point %d for %s has invalid width", index, self._name)
                return False
            if not all(math.isfinite(value) for value in point.positions):
                logger.warning(
                    "Trajectory point %d for %s has non-finite positions", index, self._name
                )
                return False
            if not all(math.isfinite(value) for value in point.velocities):
                logger.warning(
                    "Trajectory point %d for %s has non-finite velocities", index, self._name
                )
                return False
            if not math.isfinite(point.time_from_start):
                logger.warning("Trajectory point %d for %s has non-finite time", index, self._name)
                return False
            if index == 0 and point.time_from_start != 0.0:
                logger.warning("Trajectory for %s must start at t=0", self._name)
                return False
            if previous_time is not None and point.time_from_start <= previous_time:
                logger.warning("Trajectory for %s has non-increasing timestamps", self._name)
                return False
            previous_time = point.time_from_start
        if len(trajectory.points) > 1 and trajectory.duration <= 0.0:
            logger.warning("Trajectory for %s has nonpositive duration", self._name)
            return False
        return True

    def execute(
        self,
        trajectory: JointTrajectory,
        current_positions: Mapping[str, float],
    ) -> TrajectoryExecutionResult:
        """Start executing a trajectory.

        Args:
            trajectory: Trajectory to execute
            current_positions: Authoritative positions from the coordinator.

        Returns:
            Semantic execution acceptance result.
        """
        if self._state == TrajectoryState.FAULT:
            logger.warning(f"Cannot execute: {self._name} in FAULT state")
            return TrajectoryExecutionResult(
                TrajectoryExecutionStatus.INVALID_TRAJECTORY,
                f"Trajectory task '{self._name}' is in FAULT state",
            )

        if trajectory is None:
            logger.warning(f"Invalid trajectory for {self._name}")
            return TrajectoryExecutionResult(
                TrajectoryExecutionStatus.INVALID_TRAJECTORY,
                "Trajectory is missing",
            )

        if not self._validate_trajectory(trajectory):
            return TrajectoryExecutionResult(
                TrajectoryExecutionStatus.INVALID_TRAJECTORY,
                "Trajectory structure or joints are invalid",
            )

        first_positions = list(trajectory.points[0].positions)
        anchored = False
        if len(trajectory.points) > 1:
            for index, (joint_name, planned_position) in enumerate(
                zip(trajectory.joint_names, first_positions, strict=True)
            ):
                commanded_position = self._commanded_positions.get(joint_name)
                if commanded_position is not None:
                    first_positions[index] = commanded_position
                    anchored = True
                    continue
                current_position = current_positions.get(joint_name)
                if current_position is None or not math.isfinite(current_position):
                    return TrajectoryExecutionResult(
                        TrajectoryExecutionStatus.START_STATE_UNAVAILABLE,
                        f"Current position for joint '{joint_name}' is unavailable",
                    )
                error = abs(current_position - planned_position)
                if error > self._config.start_position_tolerance:
                    return TrajectoryExecutionResult(
                        TrajectoryExecutionStatus.START_STATE_MISMATCH,
                        f"Trajectory start for joint '{joint_name}' differs from current "
                        f"position by {error:.6f}",
                    )

        if anchored:
            trajectory = JointTrajectory(
                joint_names=list(trajectory.joint_names),
                points=[
                    TrajectoryPoint(
                        time_from_start=trajectory.points[0].time_from_start,
                        positions=first_positions,
                        velocities=list(trajectory.points[0].velocities),
                    ),
                    *trajectory.points[1:],
                ],
                timestamp=trajectory.timestamp,
            )

        run = _TrajectoryRun(trajectory)
        for index, joint_name in enumerate(trajectory.joint_names):
            self._motions[joint_name] = (run, index)
            if len(trajectory.points) > 1 and joint_name not in self._commanded_positions:
                self._commanded_positions[joint_name] = current_positions[joint_name]
        self._trajectory = trajectory
        self._last_duration = trajectory.duration
        self._last_elapsed = 0.0
        self._pending_start = True  # Start time set on first compute()
        self._state = TrajectoryState.EXECUTING

        logger.info(
            f"Executing trajectory on {self._name}: "
            f"{len(trajectory.points)} points, duration={trajectory.duration:.3f}s"
        )
        return TrajectoryExecutionResult(TrajectoryExecutionStatus.ACCEPTED)

    def cancel(self) -> TrajectoryCancellationResult:
        """Cancel current trajectory.

        Returns:
            Semantic cancellation result.
        """
        if self._state != TrajectoryState.EXECUTING:
            return TrajectoryCancellationResult(TrajectoryCancellationStatus.ALREADY_STOPPED)
        self._state = TrajectoryState.ABORTED
        self._clear_active_trajectory()
        logger.info(f"Trajectory {self._name} cancelled")
        return TrajectoryCancellationResult(TrajectoryCancellationStatus.CANCELLED)

    def reset(self) -> bool:
        """Reset to idle state.

        Returns:
            True if reset, False if currently executing
        """
        if self._state == TrajectoryState.EXECUTING:
            logger.warning(f"Cannot reset {self._name} while executing")
            return False
        self._state = TrajectoryState.IDLE
        self._clear_active_trajectory()
        logger.info(f"Trajectory {self._name} reset to IDLE")
        return True

    def get_state(self) -> TrajectoryState:
        """Get current state."""
        return self._state

    def get_progress(self, t_now: float) -> float:
        """Get execution progress (0.0 to 1.0).

        Args:
            t_now: Current coordinator time

        Returns:
            Progress as fraction, or 0.0 if not executing
        """
        if self._state != TrajectoryState.EXECUTING or self._trajectory is None:
            return 0.0
        if self._trajectory.duration <= 0.0:
            return 0.0
        t_elapsed = t_now - self._start_time
        return min(1.0, t_elapsed / self._trajectory.duration)

    def get_status(self, t_now: float) -> TrajectoryStatus:
        """Return a non-destructive snapshot of the current execution state."""

        if self._state == TrajectoryState.EXECUTING:
            progress = self.get_progress(t_now)
            elapsed = 0.0 if self._pending_start else max(0.0, t_now - self._start_time)
            return TrajectoryStatus(
                state=self._state,
                progress=progress,
                time_elapsed=elapsed,
                time_remaining=max(0.0, self._last_duration - elapsed),
            )
        completed = self._state == TrajectoryState.COMPLETED
        return TrajectoryStatus(
            state=self._state,
            progress=1.0 if completed else 0.0,
            time_elapsed=self._last_duration if completed else self._last_elapsed,
            time_remaining=0.0,
        )


class JointTrajectoryTaskParams(BaseConfig):
    """Task-specific trajectory execution parameters."""

    start_position_tolerance: float = Field(
        default=0.05,
        ge=0.0,
        allow_inf_nan=False,
    )
    velocity_limits: dict[str, float] | None = None


def create_task(cfg: Any, hardware: Any) -> JointTrajectoryTask:
    if cfg.name != JOINT_TRAJECTORY_TASK_NAME:
        raise ValueError(
            f"trajectory task must be named {JOINT_TRAJECTORY_TASK_NAME!r}, got {cfg.name!r}"
        )
    params = JointTrajectoryTaskParams.model_validate(cfg.params)
    return JointTrajectoryTask(
        JointTrajectoryTaskConfig(
            joint_names=cfg.joint_names,
            priority=cfg.priority,
            start_position_tolerance=params.start_position_tolerance,
            velocity_limits=params.velocity_limits,
        ),
    )
