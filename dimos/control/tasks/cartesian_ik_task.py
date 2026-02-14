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

"""Cartesian control task with internal Pinocchio IK solver.

Accepts streaming cartesian poses (e.g., from teleoperation, visual servoing)
and computes inverse kinematics internally to output joint commands.
Participates in joint-level arbitration.

CRITICAL: Uses t_now from CoordinatorState, never calls time.time()
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.manipulation.planning.kinematics.pinocchio_ik import (
    PinocchioIK,
    check_joint_delta,
    get_worst_joint_delta,
    pose_to_se3,
)
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from pathlib import Path

    from numpy.typing import NDArray
    import pinocchio  # type: ignore[import-untyped]

    from dimos.msgs.geometry_msgs import Pose, PoseStamped

logger = setup_logger()


@dataclass
class CartesianIKTaskConfig:
    """Configuration for cartesian IK task.

    Attributes:
        joint_names: List of joint names this task controls (must match model DOF)
        model_path: Path to URDF or MJCF file for IK solver
        ee_joint_id: End-effector joint ID in the kinematic chain
        priority: Priority for arbitration (higher wins)
        timeout: If no command received for this many seconds, go inactive (0 = never)
        max_joint_delta_deg: Maximum allowed joint change per tick (safety limit)
    """

    joint_names: list[str]
    model_path: str | Path
    ee_joint_id: int
    priority: int = 10
    timeout: float = 0.5
    max_joint_delta_deg: float = 15.0  # ~1500°/s at 100Hz


class CartesianIKTask(BaseControlTask):
    """Cartesian control task with internal Pinocchio IK solver.

    Accepts streaming cartesian poses via on_cartesian_command() and computes IK
    internally to output joint commands. Uses current joint state from
    CoordinatorState as IK warm-start for fast convergence.

    Unlike CartesianServoTask (which bypasses joint arbitration), this task
    outputs JointCommandOutput and participates in joint-level arbitration.

    Example:
        >>> from dimos.utils.data import get_data
        >>> piper_path = get_data("piper_description")
        >>> task = CartesianIKTask(
        ...     name="cartesian_arm",
        ...     config=CartesianIKTaskConfig(
        ...         joint_names=["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"],
        ...         model_path=piper_path / "mujoco_model" / "piper_no_gripper_description.xml",
        ...         ee_joint_id=6,
        ...         priority=10,
        ...         timeout=0.5,
        ...     ),
        ... )
        >>> coordinator.add_task(task)
        >>> task.start()
        >>>
        >>> # From teleop callback or other source:
        >>> task.on_cartesian_command(pose_stamped, t_now=time.perf_counter())
    """

    def __init__(self, name: str, config: CartesianIKTaskConfig) -> None:
        """Initialize cartesian IK task.

        Args:
            name: Unique task name
            config: Task configuration
        """
        if not config.joint_names:
            raise ValueError(f"CartesianIKTask '{name}' requires at least one joint")
        if not config.model_path:
            raise ValueError(f"CartesianIKTask '{name}' requires model_path for IK solver")

        self._name = name
        self._config = config
        self._joint_names = frozenset(config.joint_names)
        self._joint_names_list = list(config.joint_names)
        self._num_joints = len(config.joint_names)

        # Create IK solver from model
        self._ik = PinocchioIK.from_model_path(config.model_path, config.ee_joint_id)

        # Validate DOF matches joint names
        if self._ik.nq != self._num_joints:
            logger.warning(
                f"CartesianIKTask {name}: model DOF ({self._ik.nq}) != "
                f"joint_names count ({self._num_joints})"
            )

        # Thread-safe target state
        self._lock = threading.Lock()
        self._target_pose: Pose | PoseStamped | None = None
        self._last_update_time: float = 0.0
        self._active = False

        # Cache last successful IK solution for warm-starting
        self._last_q_solution: NDArray[np.floating[Any]] | None = None

        logger.info(
            f"CartesianIKTask {name} initialized with model: {config.model_path}, "
            f"ee_joint_id={config.ee_joint_id}, joints={config.joint_names}"
        )

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
            return self._active and self._target_pose is not None

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        """Compute IK and output joint positions.

        Args:
            state: Current coordinator state (contains joint positions for IK warm-start)

        Returns:
            JointCommandOutput with positions, or None if inactive/timed out/IK failed
        """
        with self._lock:
            if not self._active or self._target_pose is None:
                return None
            # Check timeout
            if self._config.timeout > 0:
                time_since_update = state.t_now - self._last_update_time
                if time_since_update > self._config.timeout:
                    logger.warning(
                        f"CartesianIKTask {self._name} timed out "
                        f"(no update for {time_since_update:.3f}s)"
                    )
                    self._active = False
                    return None
            raw_pose = self._target_pose

        # Convert to SE3 right before use
        target_pose = pose_to_se3(raw_pose)
        # Get current joint positions for IK warm-start
        q_current = self._get_current_joints(state)
        if q_current is None:
            logger.debug(f"CartesianIKTask {self._name}: missing joint state for IK warm-start")
            return None

        # Compute IK
        q_solution, converged, final_error = self._ik.solve(target_pose, q_current)
        # Use the solution even if it didn't fully converge
        if not converged:
            logger.debug(
                f"CartesianIKTask {self._name}: IK did not converge "
                f"(error={final_error:.4f}), using partial solution"
            )

        # Safety check: reject if any joint delta exceeds limit
        if not check_joint_delta(q_solution, q_current, self._config.max_joint_delta_deg):
            worst_idx, worst_deg = get_worst_joint_delta(q_solution, q_current)
            logger.warning(
                f"CartesianIKTask {self._name}: rejecting motion - "
                f"joint {self._joint_names_list[worst_idx]} delta "
                f"{worst_deg:.1f}° exceeds limit {self._config.max_joint_delta_deg}°"
            )
            return None

        # Cache solution for next warm-start
        with self._lock:
            self._last_q_solution = q_solution.copy()
        return JointCommandOutput(
            joint_names=self._joint_names_list,
            positions=q_solution.flatten().tolist(),
            mode=ControlMode.SERVO_POSITION,
        )

    def _get_current_joints(self, state: CoordinatorState) -> NDArray[np.floating[Any]] | None:
        """Get current joint positions from coordinator state.

        Falls back to last IK solution if joint state unavailable.
        """
        positions = []
        for joint_name in self._joint_names_list:
            pos = state.joints.get_position(joint_name)
            if pos is None:
                # Fallback to last solution
                if self._last_q_solution is not None:
                    result: NDArray[np.floating[Any]] = self._last_q_solution.copy()
                    return result
                return None
            positions.append(pos)
        return np.array(positions, dtype=np.float64)

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        """Handle preemption by higher-priority task.

        Args:
            by_task: Name of preempting task
            joints: Joints that were preempted
        """
        if joints & self._joint_names:
            logger.warning(
                f"CartesianIKTask {self._name} preempted by {by_task} on joints {joints}"
            )

    # =========================================================================
    # Task-specific methods
    # =========================================================================

    def on_cartesian_command(self, pose: Pose | PoseStamped, t_now: float) -> bool:
        """Handle incoming cartesian command (target EE pose).

        Args:
            pose: Target end-effector pose (Pose or PoseStamped)
            t_now: Current time (from coordinator or time.perf_counter())

        Returns:
            True if accepted
        """
        with self._lock:
            self._target_pose = pose  # Store raw, convert to SE3 in compute()
            self._last_update_time = t_now
            self._active = True

        return True

    def start(self) -> None:
        """Activate the task (start accepting and outputting commands)."""
        with self._lock:
            self._active = True
        logger.info(f"CartesianIKTask {self._name} started")

    def stop(self) -> None:
        """Deactivate the task (stop outputting commands)."""
        with self._lock:
            self._active = False
        logger.info(f"CartesianIKTask {self._name} stopped")

    def clear(self) -> None:
        """Clear current target and deactivate."""
        with self._lock:
            self._target_pose = None
            self._active = False
        logger.info(f"CartesianIKTask {self._name} cleared")

    def is_tracking(self) -> bool:
        """Check if actively receiving and outputting commands."""
        with self._lock:
            return self._active and self._target_pose is not None

    def get_current_ee_pose(self, state: CoordinatorState) -> pinocchio.SE3 | None:
        """Get current end-effector pose via forward kinematics.

        Useful for getting initial pose before starting tracking.

        Args:
            state: Current coordinator state

        Returns:
            Current EE pose as SE3, or None if joint state unavailable
        """
        q_current = self._get_current_joints(state)
        if q_current is None:
            return None

        return self._ik.forward_kinematics(q_current)

    def forward_kinematics(self, joint_positions: NDArray[np.floating[Any]]) -> pinocchio.SE3:
        """Compute end-effector pose from joint positions.

        Args:
            joint_positions: Joint angles in radians

        Returns:
            End-effector pose as SE3
        """
        return self._ik.forward_kinematics(joint_positions)


__all__ = [
    "CartesianIKTask",
    "CartesianIKTaskConfig",
]
