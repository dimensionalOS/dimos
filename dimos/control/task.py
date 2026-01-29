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

"""ControlTask protocol and types for the ControlCoordinator.

This module defines:
- Data types used by tasks and the coordinator (ResourceClaim, JointStateSnapshot, etc.)
- ControlTask protocol that all tasks must implement

Tasks are "passive" - they don't own threads. The coordinator calls
compute() at each tick, passing current state and time.

CRITICAL: Tasks must NEVER call time.time() directly.
Use the t_now passed in CoordinatorState.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol, runtime_checkable

from dimos.control.components import JointName
from dimos.hardware.manipulators.spec import ControlMode

# =============================================================================
# Data Types
# =============================================================================


@dataclass(frozen=True)
class ResourceClaim:
    """Declares which joints a task wants to control.

    Used by the coordinator to determine resource ownership and
    resolve conflicts between competing tasks.

    Attributes:
        joints: Set of joint names this task wants to control.
                Example: frozenset({"left_joint1", "left_joint2"})
        priority: Priority level for conflict resolution. Higher wins.
                  Typical values: 10 (trajectory), 50 (WBC), 100 (safety)
        mode: Control mode (POSITION, VELOCITY, TORQUE)
    """

    joints: frozenset[JointName]
    priority: int = 0
    mode: ControlMode = ControlMode.POSITION

    def conflicts_with(self, other: ResourceClaim) -> bool:
        """Check if two claims compete for the same joints."""
        return bool(self.joints & other.joints)


@dataclass
class JointStateSnapshot:
    """Aggregated joint states from all hardware.

    Provides a unified view of all joint states across all hardware
    interfaces, indexed by fully-qualified joint name.

    Attributes:
        joint_positions: Joint name -> position (radians)
        joint_velocities: Joint name -> velocity (rad/s)
        joint_efforts: Joint name -> effort (Nm)
        timestamp: Unix timestamp when state was read
    """

    joint_positions: dict[JointName, float] = field(default_factory=dict)
    joint_velocities: dict[JointName, float] = field(default_factory=dict)
    joint_efforts: dict[JointName, float] = field(default_factory=dict)
    timestamp: float = 0.0

    def get_position(self, joint_name: JointName) -> float | None:
        """Get position for a specific joint."""
        return self.joint_positions.get(joint_name)

    def get_velocity(self, joint_name: JointName) -> float | None:
        """Get velocity for a specific joint."""
        return self.joint_velocities.get(joint_name)

    def get_effort(self, joint_name: JointName) -> float | None:
        """Get effort for a specific joint."""
        return self.joint_efforts.get(joint_name)


@dataclass
class CoordinatorState:
    """Complete state snapshot for tasks to read.

    Passed to each task's compute() method every tick. Contains
    all information a task needs to compute its output.

    CRITICAL: Tasks should use t_now for timing, never time.time()!

    Attributes:
        joints: Aggregated joint states from all hardware
        t_now: Current tick time (time.perf_counter())
        dt: Time since last tick (seconds)
    """

    joints: JointStateSnapshot
    t_now: float  # Coordinator time (perf_counter) - USE THIS, NOT time.time()!
    dt: float  # Time since last tick


@dataclass
class JointCommandOutput:
    """Joint-centric command output from a task.

    Commands are addressed by joint name, NOT by hardware ID.
    The coordinator routes commands to the appropriate hardware.

    This design enables:
    - WBC spanning multiple hardware interfaces
    - Partial joint ownership
    - Per-joint arbitration

    Attributes:
        joint_names: Which joints this command is for
        positions: Position commands (radians), or None
        velocities: Velocity commands (rad/s), or None
        efforts: Effort commands (Nm), or None
        mode: Control mode - must match which field is populated
    """

    joint_names: list[JointName]
    positions: list[float] | None = None
    velocities: list[float] | None = None
    efforts: list[float] | None = None
    mode: ControlMode = ControlMode.POSITION

    def __post_init__(self) -> None:
        """Validate that lengths match and at least one value field is set."""
        n = len(self.joint_names)

        if self.positions is not None and len(self.positions) != n:
            raise ValueError(f"positions length {len(self.positions)} != joint_names length {n}")
        if self.velocities is not None and len(self.velocities) != n:
            raise ValueError(f"velocities length {len(self.velocities)} != joint_names length {n}")
        if self.efforts is not None and len(self.efforts) != n:
            raise ValueError(f"efforts length {len(self.efforts)} != joint_names length {n}")

    def get_values(self) -> list[float] | None:
        """Get the active values based on mode."""
        match self.mode:
            case ControlMode.POSITION | ControlMode.SERVO_POSITION:
                return self.positions
            case ControlMode.VELOCITY:
                return self.velocities
            case ControlMode.TORQUE:
                return self.efforts
            case _:
                return None


# =============================================================================
# ControlTask Protocol
# =============================================================================


@runtime_checkable
class ControlTask(Protocol):
    """Protocol for passive tasks that run within the coordinator.

    Tasks are "passive" - they don't own threads. The coordinator
    calls compute() at each tick, passing current state and time.

    Lifecycle:
    1. Task is added to coordinator via add_task()
    2. Coordinator calls claim() to understand resource needs
    3. Each tick: is_active() → compute() → output merged via arbitration
    4. Task removed via remove_task() or transitions to inactive

    CRITICAL: Tasks must NEVER call time.time() directly.
    Use state.t_now passed to compute() for all timing.

    Example:
        >>> class MyTask:
        ...     @property
        ...     def name(self) -> str:
        ...         return "my_task"
        ...
        ...     def claim(self) -> ResourceClaim:
        ...         return ResourceClaim(
        ...             joints=frozenset(["left_joint1", "left_joint2"]),
        ...             priority=10,
        ...         )
        ...
        ...     def is_active(self) -> bool:
        ...         return self._executing
        ...
        ...     def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        ...         # Use state.t_now, NOT time.time()!
        ...         t_elapsed = state.t_now - self._start_time
        ...         positions = self._trajectory.sample(t_elapsed)
        ...         return JointCommandOutput(
        ...             joint_names=["left_joint1", "left_joint2"],
        ...             positions=positions,
        ...         )
        ...
        ...     def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        ...         print(f"Preempted by {by_task} on {joints}")
    """

    @property
    def name(self) -> str:
        """Unique identifier for this task instance.

        Used for logging, debugging, and task management.
        Must be unique across all tasks in the coordinator.
        """
        ...

    def claim(self) -> ResourceClaim:
        """Declare resource requirements.

        Called by coordinator to determine:
        - Which joints this task wants to control
        - Priority for conflict resolution
        - Control mode (position/velocity/effort)

        Returns:
            ResourceClaim with joints (frozenset) and priority (int)

        Note:
            The claim can change dynamically - coordinator calls this
            every tick for active tasks.
        """
        ...

    def is_active(self) -> bool:
        """Check if task should run this tick.

        Inactive tasks:
        - Skip compute() call
        - Don't participate in arbitration
        - Don't consume resources

        Returns:
            True if task should execute this tick
        """
        ...

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        """Compute output command given current state.

        Called by coordinator for active tasks each tick.

        CRITICAL: Use state.t_now for timing, NEVER time.time()!
        This ensures deterministic behavior and enables simulation.

        Args:
            state: CoordinatorState containing:
                   - joints: JointStateSnapshot with all joint states
                   - t_now: Current tick time (use this for all timing!)
                   - dt: Time since last tick

        Returns:
            JointCommandOutput with joint_names and values, or None if
            no command should be sent this tick.
        """
        ...

    def on_preempted(self, by_task: str, joints: frozenset[JointName]) -> None:
        """Called ONCE per tick with ALL preempted joints aggregated.

        Called when a higher-priority task takes control of some of this
        task's joints. Allows task to gracefully handle being overridden.

        This is called ONCE per tick with ALL preempted joints, not once
        per joint. This reduces noise and improves performance.

        Args:
            by_task: Name of the preempting task (or "arbitration" if multiple)
            joints: All joints that were preempted this tick
        """
        ...


__all__ = [
    # Types
    "ControlMode",
    # Protocol
    "ControlTask",
    "CoordinatorState",
    "JointCommandOutput",
    "JointName",
    "JointStateSnapshot",
    "ResourceClaim",
]
