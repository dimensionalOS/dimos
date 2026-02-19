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

"""Pinocchio-based inverse kinematics solver.

Standalone IK solver using Pinocchio for forward kinematics and Jacobian
computation. Uses damped least-squares (Levenberg-Marquardt) for robust
convergence near singularities.

Unlike JacobianIK (which uses the WorldSpec interface), this solver operates
directly on a Pinocchio model. This makes it suitable for lightweight,
real-time IK in control tasks where a full WorldSpec is not needed.

Usage:
    >>> from dimos.manipulation.planning.kinematics.pinocchio_ik import PinocchioIK
    >>> ik = PinocchioIK.from_model_path("robot.urdf", ee_joint_id=6)
    >>> q_solution, converged, error = ik.solve(target_se3, q_init)
    >>> ee_pose = ik.forward_kinematics(q_solution)
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np
from numpy.linalg import norm, solve
import pinocchio  # type: ignore[import-untyped]

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.msgs.geometry_msgs import Pose, PoseStamped

logger = setup_logger()


# =============================================================================
# Configuration
# =============================================================================


@dataclass
class PinocchioIKConfig:
    """Configuration for the Pinocchio IK solver.

    Attributes:
        max_iter: Maximum IK solver iterations
        eps: Convergence threshold (SE3 log-error norm)
        damp: Damping factor for singularity handling (higher = more stable)
        dt: Integration step size
        max_velocity: Max joint velocity per iteration (rad/s), clamps near singularities
    """

    max_iter: int = 100
    eps: float = 1e-4
    damp: float = 1e-2
    dt: float = 1.0
    max_velocity: float = 10.0


# =============================================================================
# PinocchioIK Solver
# =============================================================================


class PinocchioIK:
    """Pinocchio-based damped least-squares IK solver.

    Loads a URDF or MJCF model and provides:
    - solve(): Damped least-squares IK from SE3 target
    - forward_kinematics(): FK from joint angles to EE pose

    Thread safety: NOT thread-safe. Each caller should use its own instance
    or protect calls with an external lock. Control tasks typically hold a
    lock around compute() which covers IK calls.

    Example:
        >>> ik = PinocchioIK.from_model_path("robot.urdf", ee_joint_id=6)
        >>> target = pose_to_se3(pose_stamped)
        >>> q, converged, err = ik.solve(target, q_current)
        >>> if converged:
        ...     ee = ik.forward_kinematics(q)
    """

    def __init__(
        self,
        model: pinocchio.Model,
        data: pinocchio.Data,
        ee_joint_id: int,
        config: PinocchioIKConfig | None = None,
    ) -> None:
        """Initialize solver with an existing Pinocchio model.

        Args:
            model: Pinocchio model
            data: Pinocchio data (created from model)
            ee_joint_id: End-effector joint ID in the kinematic chain
            config: Solver configuration (uses defaults if None)
        """
        self._model = model
        self._data = data
        self._ee_joint_id = ee_joint_id
        self._config = config or PinocchioIKConfig()

    @classmethod
    def from_model_path(
        cls,
        model_path: str | Path,
        ee_joint_id: int,
    ) -> PinocchioIK:
        """Create solver by loading a URDF or MJCF file.

        Args:
            model_path: Path to URDF (.urdf) or MJCF (.xml) file
            ee_joint_id: End-effector joint ID in the kinematic chain

        Returns:
            Configured PinocchioIK instance

        Raises:
            FileNotFoundError: If model file doesn't exist
        """
        path = Path(str(model_path))
        if not path.exists():
            raise FileNotFoundError(f"Model file not found: {path}")

        if path.suffix == ".xml":
            model = pinocchio.buildModelFromMJCF(str(path))
        else:
            model = pinocchio.buildModelFromUrdf(str(path))

        data = model.createData()
        return cls(model, data, ee_joint_id)

    @property
    def model(self) -> pinocchio.Model:
        """The Pinocchio model."""
        return self._model

    @property
    def nq(self) -> int:
        """Number of configuration variables (DOF)."""
        return int(self._model.nq)

    @property
    def ee_joint_id(self) -> int:
        """End-effector joint ID."""
        return self._ee_joint_id

    # =========================================================================
    # Core IK
    # =========================================================================

    def solve(
        self,
        target_pose: pinocchio.SE3,
        q_init: NDArray[np.floating[Any]],
        config: PinocchioIKConfig | None = None,
    ) -> tuple[NDArray[np.floating[Any]], bool, float]:
        """Solve IK using damped least-squares (Levenberg-Marquardt).

        Args:
            target_pose: Target end-effector pose as SE3
            q_init: Initial joint configuration (warm-start)
            config: Override solver config for this call (uses instance config if None)

        Returns:
            Tuple of (joint_angles, converged, final_error)
        """
        cfg = config or self._config
        q = q_init.copy()
        final_err = float("inf")

        for _ in range(cfg.max_iter):
            pinocchio.forwardKinematics(self._model, self._data, q)
            iMd = self._data.oMi[self._ee_joint_id].actInv(target_pose)

            err = pinocchio.log(iMd).vector
            final_err = float(norm(err))
            if final_err < cfg.eps:
                return q, True, final_err

            J = pinocchio.computeJointJacobian(self._model, self._data, q, self._ee_joint_id)
            J = -np.dot(pinocchio.Jlog6(iMd.inverse()), J)
            v = -J.T.dot(solve(J.dot(J.T) + cfg.damp * np.eye(6), err))

            # Clamp velocity to prevent explosion near singularities
            v_norm = norm(v)
            if v_norm > cfg.max_velocity:
                v = v * (cfg.max_velocity / v_norm)

            q = pinocchio.integrate(self._model, q, v * cfg.dt)

        return q, False, final_err

    # =========================================================================
    # Forward Kinematics
    # =========================================================================

    def forward_kinematics(self, joint_positions: NDArray[np.floating[Any]]) -> pinocchio.SE3:
        """Compute end-effector pose from joint positions.

        Args:
            joint_positions: Joint angles in radians

        Returns:
            End-effector pose as SE3
        """
        pinocchio.forwardKinematics(self._model, self._data, joint_positions)
        return self._data.oMi[self._ee_joint_id].copy()


# =============================================================================
# Pose Conversion Helpers
# =============================================================================


def pose_to_se3(pose: Pose | PoseStamped) -> pinocchio.SE3:
    """Convert Pose or PoseStamped to pinocchio SE3"""

    position = np.array([pose.x, pose.y, pose.z])
    quat = pose.orientation
    rotation = pinocchio.Quaternion(quat.w, quat.x, quat.y, quat.z).toRotationMatrix()
    return pinocchio.SE3(rotation, position)


# =============================================================================
# Safety Utilities
# =============================================================================


def check_joint_delta(
    q_new: NDArray[np.floating[Any]],
    q_current: NDArray[np.floating[Any]],
    max_delta_deg: float,
) -> bool:
    """Check if joint position change is within safety limits.

    Args:
        q_new: Proposed joint positions (radians)
        q_current: Current joint positions (radians)
        max_delta_deg: Maximum allowed change per joint (degrees)

    Returns:
        True if all joint deltas are within limits
    """
    max_delta_rad = np.radians(max_delta_deg)
    joint_deltas = np.abs(q_new - q_current)
    return bool(np.all(joint_deltas <= max_delta_rad))


def get_worst_joint_delta(
    q_new: NDArray[np.floating[Any]],
    q_current: NDArray[np.floating[Any]],
) -> tuple[int, float]:
    """Find the joint with the largest position change.

    Args:
        q_new: Proposed joint positions (radians)
        q_current: Current joint positions (radians)

    Returns:
        Tuple of (joint_index, delta_in_degrees)
    """
    joint_deltas = np.abs(q_new - q_current)
    worst_idx = int(np.argmax(joint_deltas))
    return worst_idx, float(np.degrees(joint_deltas[worst_idx]))


__all__ = [
    "PinocchioIK",
    "PinocchioIKConfig",
    "check_joint_delta",
    "get_worst_joint_delta",
    "pose_to_se3",
]
