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

"""Streaming joint velocity task for real-time velocity control.

Accepts streaming joint velocities (e.g., from joystick) and outputs them
directly to hardware each tick. Useful for joystick control, force feedback,
or any velocity-mode real-time control.

SAFETY: On timeout, sends zero velocities to stop motion (configurable).

CRITICAL: Uses t_now from CoordinatorState, never calls time.time()
"""

from __future__ import annotations

from dataclasses import dataclass
import threading

from dimos.control.task import (
    ControlMode,
    ControlTask,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class JointVelocityTaskConfig:
    """Configuration for velocity task.

    Attributes:
        joint_names: List of joint names this task controls
        priority: Priority for arbitration (higher wins)
        timeout: If no command received for this many seconds, trigger timeout behavior
        zero_on_timeout: If True, send zero velocities on timeout (safety). If False, go inactive.
    """

    joint_names: list[str]
    priority: int = 10
    timeout: float = 0.2  # 200ms default - shorter for safety
    zero_on_timeout: bool = True  # Send zeros to stop motion


class JointVelocityTask(ControlTask):
    """Streaming joint velocity control for joystick/force feedback.

    Accepts target velocities via set_velocities() or set_velocities_by_name()
    and outputs them each tick. Uses VELOCITY mode for direct velocity control.

    SAFETY: On timeout (no update for timeout seconds):
    - If zero_on_timeout=True: sends zero velocities to stop motion
    - If zero_on_timeout=False: goes inactive (hardware may coast)

    Example:
        >>> task = JointVelocityTask(
        ...     name="velocity_arm",
        ...     config=JointVelocityTaskConfig(
        ...         joint_names=["arm_joint1", "arm_joint2", "arm_joint3"],
        ...         priority=10,
        ...         timeout=0.2,
        ...         zero_on_timeout=True,
        ...     ),
        ... )
        >>> coordinator.add_task(task)
        >>> task.start()
        >>>
        >>> # From joystick callback:
        >>> task.set_velocities([0.1, -0.05, 0.0], t_now=time.perf_counter())
    """

    def __init__(self, name: str, config: JointVelocityTaskConfig) -> None:
        """Initialize velocity task.

        Args:
            name: Unique task name
            config: Task configuration
        """
        if not config.joint_names:
            raise ValueError(f"JointVelocityTask '{name}' requires at least one joint")

        self._name = name
        self._config = config
        self._joint_names = frozenset(config.joint_names)
        self._joint_names_list = list(config.joint_names)
        self._num_joints = len(config.joint_names)

        # Current velocities (thread-safe)
        self._lock = threading.Lock()
        self._velocities: list[float] | None = None
        self._last_update_time: float = 0.0
        self._active = False
        self._timed_out = False  # Track timeout state for logging

        logger.info(f"JointVelocityTask {name} initialized for joints: {config.joint_names}")

    @property
    def name(self) -> str:
        """Unique task identifier."""
        return self._name

    def claim(self) -> ResourceClaim:
        """Declare resource requirements."""
        return ResourceClaim(
            joints=self._joint_names,
            priority=self._config.priority,
            mode=ControlMode.VELOCITY,
        )

    def is_active(self) -> bool:
        """Check if task should run this tick."""
        with self._lock:
            # Active if started, even if timed out (we still send zeros)
            if self._config.zero_on_timeout:
                return self._active
            else:
                return self._active and self._velocities is not None

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        """Output current target velocities.

        Args:
            state: Current coordinator state

        Returns:
            JointCommandOutput with velocities, or None if inactive
        """
        with self._lock:
            if not self._active:
                return None

            # Check timeout
            if self._config.timeout > 0 and self._velocities is not None:
                time_since_update = state.t_now - self._last_update_time
                if time_since_update > self._config.timeout:
                    if not self._timed_out:
                        logger.warning(
                            f"JointVelocityTask {self._name} timed out "
                            f"(no update for {time_since_update:.3f}s)"
                        )
                        self._timed_out = True

                    if self._config.zero_on_timeout:
                        # SAFETY: Send zeros to stop motion
                        return JointCommandOutput(
                            joint_names=self._joint_names_list,
                            velocities=[0.0] * self._num_joints,
                            mode=ControlMode.VELOCITY,
                        )
                    else:
                        # Go inactive
                        self._active = False
                        return None

            if self._velocities is None:
                return None

            # Reset timeout flag on successful output
            self._timed_out = False

            return JointCommandOutput(
                joint_names=self._joint_names_list,
                velocities=list(self._velocities),
                mode=ControlMode.VELOCITY,
            )

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        """Handle preemption by higher-priority task.

        Args:
            by_task: Name of preempting task
            joints: Joints that were preempted
        """
        if joints & self._joint_names:
            logger.warning(
                f"JointVelocityTask {self._name} preempted by {by_task} on joints {joints}"
            )

    # =========================================================================
    # Task-specific methods
    # =========================================================================

    def set_velocities(self, velocities: list[float], t_now: float) -> bool:
        """Set target joint velocities.

        Call this from your joystick callback or other data source.

        Args:
            velocities: Joint velocities in rad/s (must match joint_names length)
            t_now: Current time (from coordinator or time.perf_counter())

        Returns:
            True if accepted, False if wrong number of joints
        """
        if len(velocities) != self._num_joints:
            logger.warning(
                f"JointVelocityTask {self._name}: expected {self._num_joints} "
                f"velocities, got {len(velocities)}"
            )
            return False

        with self._lock:
            self._velocities = list(velocities)
            self._last_update_time = t_now
            self._active = True
            self._timed_out = False

        return True

    def set_velocities_by_name(self, velocities: dict[str, float], t_now: float) -> bool:
        """Set target velocities by joint name.

        Extracts only the joints this task controls from the dict.
        Useful for routing when multiple tasks share an input stream.

        Args:
            velocities: {joint_name: velocity} dict (can contain extra joints)
            t_now: Current time

        Returns:
            True if all required joints found, False if any missing
        """
        ordered = []
        for name in self._joint_names_list:
            if name not in velocities:
                # Missing joint - don't update
                return False
            ordered.append(velocities[name])

        return self.set_velocities(ordered, t_now)

    def start(self) -> None:
        """Activate the task (start accepting and outputting commands)."""
        with self._lock:
            self._active = True
            self._timed_out = False
        logger.info(f"JointVelocityTask {self._name} started")

    def stop(self) -> None:
        """Deactivate the task (stop outputting commands)."""
        with self._lock:
            self._active = False
        logger.info(f"JointVelocityTask {self._name} stopped")

    def clear(self) -> None:
        """Clear current velocities and deactivate."""
        with self._lock:
            self._velocities = None
            self._active = False
            self._timed_out = False
        logger.info(f"JointVelocityTask {self._name} cleared")

    def is_streaming(self) -> bool:
        """Check if actively receiving and outputting commands."""
        with self._lock:
            return self._active and self._velocities is not None and not self._timed_out


__all__ = [
    "JointVelocityTask",
    "JointVelocityTaskConfig",
]
