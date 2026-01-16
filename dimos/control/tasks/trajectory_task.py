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

CRITICAL: Uses t_now from CoordinatorState, never calls time.time()
"""

from __future__ import annotations

from dataclasses import dataclass

from dimos.control.task import (
    ControlMode,
    ControlTask,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.msgs.trajectory_msgs import JointTrajectory, TrajectoryState
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class JointTrajectoryTaskConfig:
    """Configuration for trajectory task.

    Attributes:
        joint_names: List of joint names this task controls
        priority: Priority for arbitration (higher wins)
    """

    joint_names: list[str]
    priority: int = 10


class JointTrajectoryTask(ControlTask):
    """Passive trajectory execution task.

    Unlike JointTrajectoryController which owns a thread, this task
    is called by the coordinator at each tick.

    CRITICAL: Uses t_now from CoordinatorState, never calls time.time()

    State Machine:
        IDLE ──execute()──► EXECUTING ──done──► COMPLETED
          ▲                     │                    │
          │                  cancel()             reset()
          │                     ▼                    │
          └─────reset()───── ABORTED ◄──────────────┘

    Example:
        >>> task = JointTrajectoryTask(
        ...     name="traj_left",
        ...     config=JointTrajectoryTaskConfig(
        ...         joint_names=["left_joint1", "left_joint2"],
        ...         priority=10,
        ...     ),
        ... )
        >>> coordinator.add_task(task)
        >>> task.execute(my_trajectory, t_now=coordinator_t_now)
    """

    def __init__(self, name: str, config: JointTrajectoryTaskConfig) -> None:
        """Initialize trajectory task.

        Args:
            name: Unique task name
            config: Task configuration
        """
        if not config.joint_names:
            raise ValueError(f"JointTrajectoryTask '{name}' requires at least one joint")
        self._name = name
        self._config = config
        self._joint_names = frozenset(config.joint_names)
        self._joint_names_list = list(config.joint_names)

        # State machine
        self._state = TrajectoryState.IDLE
        self._trajectory: JointTrajectory | None = None
        self._start_time: float = 0.0
        self._pending_start: bool = False  # Defer start time to first compute()

        logger.info(f"JointTrajectoryTask {name} initialized for joints: {config.joint_names}")

    @property
    def name(self) -> str:
        """Unique task identifier."""
        return self._name

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
        if self._trajectory is None:
            return None

        # Set start time on first compute() for consistent timing
        if self._pending_start:
            self._start_time = state.t_now
            self._pending_start = False

        t_elapsed = state.t_now - self._start_time

        # Check completion - clamp to final position to ensure we reach goal
        if t_elapsed >= self._trajectory.duration:
            self._state = TrajectoryState.COMPLETED
            logger.info(f"Trajectory {self._name} completed after {t_elapsed:.3f}s")
            # Return final position to hold at goal
            q_ref, _ = self._trajectory.sample(self._trajectory.duration)
            return JointCommandOutput(
                joint_names=self._joint_names_list,
                positions=list(q_ref),
                mode=ControlMode.SERVO_POSITION,
            )

        # Sample trajectory
        q_ref, _ = self._trajectory.sample(t_elapsed)

        return JointCommandOutput(
            joint_names=self._joint_names_list,
            positions=list(q_ref),
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

    # =========================================================================
    # Task-specific methods
    # =========================================================================

    def execute(self, trajectory: JointTrajectory) -> bool:
        """Start executing a trajectory.

        Args:
            trajectory: Trajectory to execute

        Returns:
            True if accepted, False if invalid or in FAULT state
        """
        if self._state == TrajectoryState.FAULT:
            logger.warning(f"Cannot execute: {self._name} in FAULT state")
            return False

        if trajectory is None or trajectory.duration <= 0:
            logger.warning(f"Invalid trajectory for {self._name}")
            return False

        if not trajectory.points:
            logger.warning(f"Empty trajectory for {self._name}")
            return False

        # Preempt any active trajectory
        if self._state == TrajectoryState.EXECUTING:
            logger.info(f"Preempting active trajectory on {self._name}")

        self._trajectory = trajectory
        self._pending_start = True  # Start time set on first compute()
        self._state = TrajectoryState.EXECUTING

        logger.info(
            f"Executing trajectory on {self._name}: "
            f"{len(trajectory.points)} points, duration={trajectory.duration:.3f}s"
        )
        return True

    def cancel(self) -> bool:
        """Cancel current trajectory.

        Returns:
            True if cancelled, False if not executing
        """
        if self._state != TrajectoryState.EXECUTING:
            return False
        self._state = TrajectoryState.ABORTED
        logger.info(f"Trajectory {self._name} cancelled")
        return True

    def reset(self) -> bool:
        """Reset to idle state.

        Returns:
            True if reset, False if currently executing
        """
        if self._state == TrajectoryState.EXECUTING:
            logger.warning(f"Cannot reset {self._name} while executing")
            return False
        self._state = TrajectoryState.IDLE
        self._trajectory = None
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
        t_elapsed = t_now - self._start_time
        return min(1.0, t_elapsed / self._trajectory.duration)


__all__ = [
    "JointTrajectoryTask",
    "JointTrajectoryTaskConfig",
]
