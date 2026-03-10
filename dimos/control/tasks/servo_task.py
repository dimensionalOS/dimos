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

"""Streaming joint servo task for real-time position control.

Accepts streaming joint positions (e.g., from teleoperation) and outputs them
directly to hardware each tick. Useful for teleoperation, visual servoing,
or any real-time control where you don't want trajectory planning overhead.

CRITICAL: Uses t_now from CoordinatorState, never calls time.time()
"""

from __future__ import annotations

from dataclasses import dataclass
import threading

from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class JointServoTaskConfig:
    """Configuration for servo task.

    Attributes:
        joint_names: List of joint names this task controls
        priority: Priority for arbitration (higher wins)
        timeout: If no command received for this many seconds, go inactive (0 = never timeout)
    """

    joint_names: list[str]
    priority: int = 10
    timeout: float = 0.5  # 500ms default timeout


class JointServoTask(BaseControlTask):
    """Streaming joint position control for teleoperation/visual servoing.

    Accepts target positions via set_target() or set_target_by_name() and
    outputs them each tick. Uses SERVO_POSITION mode for high-frequency control.

    No trajectory planning - just pass-through with optional timeout.

    Example:
        >>> task = JointServoTask(
        ...     name="servo_arm",
        ...     config=JointServoTaskConfig(
        ...         joint_names=["arm_joint1", "arm_joint2", "arm_joint3"],
        ...         priority=10,
        ...         timeout=0.5,
        ...     ),
        ... )
        >>> coordinator.add_task(task)
        >>> task.start()
        >>>
        >>> # From teleop callback or other source:
        >>> task.set_target([0.1, 0.2, 0.3], t_now=time.perf_counter())
    """

    def __init__(self, name: str, config: JointServoTaskConfig) -> None:
        """Initialize servo task.

        Args:
            name: Unique task name
            config: Task configuration
        """
        if not config.joint_names:
            raise ValueError(f"JointServoTask '{name}' requires at least one joint")

        self._name = name
        self._config = config
        self._joint_names = frozenset(config.joint_names)
        self._joint_names_list = list(config.joint_names)
        self._num_joints = len(config.joint_names)

        # Current target (thread-safe)
        self._lock = threading.Lock()
        self._target: list[float] | None = None
        self._last_update_time: float = 0.0
        self._active = False

        logger.info(f"JointServoTask {name} initialized for joints: {config.joint_names}")

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
        with self._lock:
            return self._active and self._target is not None

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        """Output current target positions.

        Args:
            state: Current coordinator state

        Returns:
            JointCommandOutput with positions, or None if inactive/timed out
        """
        with self._lock:
            if not self._active or self._target is None:
                return None

            # Check timeout
            if self._config.timeout > 0:
                time_since_update = state.t_now - self._last_update_time
                if time_since_update > self._config.timeout:
                    logger.warning(
                        f"JointServoTask {self._name} timed out "
                        f"(no update for {time_since_update:.3f}s)"
                    )
                    self._active = False
                    return None

            return JointCommandOutput(
                joint_names=self._joint_names_list,
                positions=list(self._target),
                mode=ControlMode.SERVO_POSITION,
            )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        """Handle preemption by higher-priority task.

        Args:
            by_task: Name of preempting task
            joints: Joints that were preempted
        """
        if joints & self._joint_names:
            logger.warning(f"JointServoTask {self._name} preempted by {by_task} on joints {joints}")

    # =========================================================================
    # Task-specific methods
    # =========================================================================

    def set_target(self, positions: list[float], t_now: float) -> bool:
        """Set target joint positions.

        Call this from your teleop callback or other data source.

        Args:
            positions: Joint positions in radians (must match joint_names length)
            t_now: Current time (from coordinator or time.perf_counter())

        Returns:
            True if accepted, False if wrong number of joints
        """
        if len(positions) != self._num_joints:
            logger.warning(
                f"JointServoTask {self._name}: expected {self._num_joints} "
                f"positions, got {len(positions)}"
            )
            return False

        with self._lock:
            self._target = list(positions)
            self._last_update_time = t_now
            self._active = True

        return True

    def set_target_by_name(self, positions: dict[str, float], t_now: float) -> bool:
        """Set target positions by joint name.

        Extracts only the joints this task controls from the dict.
        Useful for routing when multiple tasks share an input stream.

        Args:
            positions: {joint_name: position} dict (can contain extra joints)
            t_now: Current time

        Returns:
            True if all required joints found, False if any missing
        """
        ordered = []
        for name in self._joint_names_list:
            if name not in positions:
                # Missing joint - don't update
                return False
            ordered.append(positions[name])

        return self.set_target(ordered, t_now)

    def start(self) -> None:
        """Activate the task (start accepting and outputting commands)."""
        with self._lock:
            self._active = True
        logger.info(f"JointServoTask {self._name} started")

    def stop(self) -> None:
        """Deactivate the task (stop outputting commands)."""
        with self._lock:
            self._active = False
        logger.info(f"JointServoTask {self._name} stopped")

    def clear(self) -> None:
        """Clear current target and deactivate."""
        with self._lock:
            self._target = None
            self._active = False
        logger.info(f"JointServoTask {self._name} cleared")

    def is_streaming(self) -> bool:
        """Check if actively receiving and outputting commands."""
        with self._lock:
            return self._active and self._target is not None


__all__ = [
    "JointServoTask",
    "JointServoTaskConfig",
]
