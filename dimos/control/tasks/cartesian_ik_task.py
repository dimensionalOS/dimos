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
from pathlib import Path
import threading
from typing import TYPE_CHECKING, Any

import numpy as np
from numpy.linalg import norm, solve
import pinocchio  # type: ignore[import-untyped]

from dimos.control.task import (
    ControlMode,
    ControlTask,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

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
        ik_max_iter: Maximum IK solver iterations
        ik_eps: IK convergence threshold (error norm in meters)
        ik_damp: IK damping factor for singularity handling (higher = more stable)
        ik_dt: IK integration step size
        max_joint_delta_deg: Maximum allowed joint change per tick (safety limit)
        max_velocity: Max joint velocity per IK iteration (rad/s)
    """

    joint_names: list[str]
    model_path: str | Path
    ee_joint_id: int
    priority: int = 10
    timeout: float = 0.5
    ik_max_iter: int = 100
    ik_eps: float = 1e-4
    ik_damp: float = 1e-2
    ik_dt: float = 1.0
    max_joint_delta_deg: float = 15.0  # ~1500°/s at 100Hz
    max_velocity: float = 2.0


class CartesianIKTask(ControlTask):
    """Cartesian control task with internal Pinocchio IK solver.

    Accepts streaming cartesian poses via set_target_pose() and computes IK
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
        >>> task.set_target_pose(pose_stamped, t_now=time.perf_counter())
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

        # Load Pinocchio model
        model_path = Path(config.model_path)
        if not model_path.exists():
            raise FileNotFoundError(f"Model file not found: {model_path}")

        if model_path.suffix == ".xml":
            self._model = pinocchio.buildModelFromMJCF(str(model_path))
        else:
            self._model = pinocchio.buildModelFromUrdf(str(model_path))

        self._data = self._model.createData()
        self._ee_joint_id = config.ee_joint_id

        # Validate DOF matches joint names
        if self._model.nq != self._num_joints:
            logger.warning(
                f"CartesianIKTask {name}: model DOF ({self._model.nq}) != "
                f"joint_names count ({self._num_joints})"
            )

        # Thread-safe target state
        self._lock = threading.Lock()
        self._target_pose: pinocchio.SE3 | None = None
        self._last_update_time: float = 0.0
        self._active = False

        # Cache last successful IK solution for warm-starting
        self._last_q_solution: NDArray[np.floating[Any]] | None = None

        logger.info(
            f"CartesianIKTask {name} initialized with model: {model_path}, "
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

            target_pose = self._target_pose

        # Get current joint positions for IK warm-start
        q_current = self._get_current_joints(state)
        if q_current is None:
            logger.debug(f"CartesianIKTask {self._name}: missing joint state for IK warm-start")
            return None

        # Compute IK
        q_solution, converged, final_error = self._solve_ik(target_pose, q_current)

        # Use the solution even if it didn't fully converge - the safety clamp
        # will handle any large jumps. This prevents the arm from "sticking"
        # when near singularities or workspace boundaries.
        if not converged:
            logger.debug(
                f"CartesianIKTask {self._name}: IK did not converge "
                f"(error={final_error:.4f}), using partial solution"
            )

        # Safety check: clamp large joint jumps for smooth motion
        _is_exact, q_clamped = self._safety_check(q_solution, q_current)

        # Cache solution for next warm-start
        with self._lock:
            self._last_q_solution = q_clamped.copy()

        return JointCommandOutput(
            joint_names=self._joint_names_list,
            positions=q_clamped.flatten().tolist(),
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
                    return self._last_q_solution.copy()
                return None
            positions.append(pos)
        return np.array(positions)

    def _solve_ik(
        self,
        target_pose: pinocchio.SE3,
        q_init: NDArray[np.floating[Any]],
    ) -> tuple[NDArray[np.floating[Any]], bool, float]:
        """Solve IK using damped least-squares (Levenberg-Marquardt).

        Args:
            target_pose: Target end-effector pose as SE3
            q_init: Initial joint configuration (warm-start)

        Returns:
            Tuple of (joint_angles, converged, final_error)
        """
        q = q_init.copy()
        final_err = float("inf")

        for _ in range(self._config.ik_max_iter):
            pinocchio.forwardKinematics(self._model, self._data, q)
            iMd = self._data.oMi[self._ee_joint_id].actInv(target_pose)

            err = pinocchio.log(iMd).vector
            final_err = float(norm(err))
            if final_err < self._config.ik_eps:
                return q, True, final_err

            J = pinocchio.computeJointJacobian(self._model, self._data, q, self._ee_joint_id)
            J = -np.dot(pinocchio.Jlog6(iMd.inverse()), J)
            v = -J.T.dot(solve(J.dot(J.T) + self._config.ik_damp * np.eye(6), err))

            # Clamp velocity to prevent explosion near singularities
            v_norm = norm(v)
            if v_norm > self._config.max_velocity:
                v = v * (self._config.max_velocity / v_norm)

            q = pinocchio.integrate(self._model, q, v * self._config.ik_dt)

        return q, False, final_err

    def _safety_check(
        self, q_new: NDArray[np.floating[Any]], q_current: NDArray[np.floating[Any]]
    ) -> tuple[bool, NDArray[np.floating[Any]]]:
        """Check IK solution and clamp if needed.

        Args:
            q_new: Proposed joint positions from IK
            q_current: Current joint positions

        Returns:
            Tuple of (is_safe, clamped_q) - if not safe, returns interpolated position
        """
        max_delta_rad = np.radians(self._config.max_joint_delta_deg)
        joint_deltas = q_new - q_current

        # Check if any joint exceeds the limit
        if np.any(np.abs(joint_deltas) > max_delta_rad):
            # Clamp the delta to the max allowed
            clamped_delta = np.clip(joint_deltas, -max_delta_rad, max_delta_rad)
            q_clamped = q_current + clamped_delta

            worst_joint = int(np.argmax(np.abs(joint_deltas)))
            logger.debug(
                f"CartesianIKTask {self._name}: clamping joint motion - "
                f"joint {self._joint_names_list[worst_joint]} delta "
                f"{np.degrees(joint_deltas[worst_joint]):.1f}° -> "
                f"{np.degrees(clamped_delta[worst_joint]):.1f}°"
            )
            return False, q_clamped

        return True, q_new

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

    def set_target_pose(self, pose: Pose | PoseStamped, t_now: float) -> bool:
        """Set target end-effector pose.

        Call this from your teleop callback, visual servoing, or other input source.

        Args:
            pose: Target end-effector pose (Pose or PoseStamped)
            t_now: Current time (from coordinator or time.perf_counter())

        Returns:
            True if accepted
        """
        target_se3 = self._pose_to_se3(pose)

        with self._lock:
            self._target_pose = target_se3
            self._last_update_time = t_now
            self._active = True

        return True

    def set_target_pose_dict(
        self,
        pose: dict[str, float],
        t_now: float,
    ) -> bool:
        """Set target from pose dict with position and RPY orientation.

        Args:
            pose: {x, y, z, roll, pitch, yaw} in meters/radians
            t_now: Current time

        Returns:
            True if accepted, False if missing required keys
        """
        required_keys = {"x", "y", "z", "roll", "pitch", "yaw"}
        if not required_keys.issubset(pose.keys()):
            missing = required_keys - set(pose.keys())
            logger.warning(f"CartesianIKTask {self._name}: missing pose keys {missing}")
            return False

        position = np.array([pose["x"], pose["y"], pose["z"]])
        rotation = pinocchio.rpy.rpyToMatrix(pose["roll"], pose["pitch"], pose["yaw"])
        target_se3 = pinocchio.SE3(rotation, position)

        with self._lock:
            self._target_pose = target_se3
            self._last_update_time = t_now
            self._active = True

        return True

    def _pose_to_se3(self, pose: Pose | PoseStamped) -> pinocchio.SE3:
        """Convert a Pose message to pinocchio SE3.

        Uses quaternion directly to avoid Euler angle conversion issues.
        """
        # Handle both Pose and PoseStamped
        if hasattr(pose, "position"):
            # Assume Pose or PoseStamped with position/orientation attributes
            position = np.array([pose.x, pose.y, pose.z])
            quat = pose.orientation
            rotation = pinocchio.Quaternion(quat.w, quat.x, quat.y, quat.z).toRotationMatrix()
        else:
            # Assume it has x, y, z directly
            position = np.array([pose.x, pose.y, pose.z])
            quat = pose.orientation
            rotation = pinocchio.Quaternion(quat.w, quat.x, quat.y, quat.z).toRotationMatrix()

        return pinocchio.SE3(rotation, position)

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

        pinocchio.forwardKinematics(self._model, self._data, q_current)
        return self._data.oMi[self._ee_joint_id].copy()

    def forward_kinematics(self, joint_positions: NDArray[np.floating[Any]]) -> pinocchio.SE3:
        """Compute end-effector pose from joint positions.

        Args:
            joint_positions: Joint angles in radians

        Returns:
            End-effector pose as SE3
        """
        pinocchio.forwardKinematics(self._model, self._data, joint_positions)
        return self._data.oMi[self._ee_joint_id].copy()


__all__ = [
    "CartesianIKTask",
    "CartesianIKTaskConfig",
]
