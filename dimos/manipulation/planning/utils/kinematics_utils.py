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

"""
Kinematics Utilities

Standalone utility functions for inverse kinematics operations.
These functions are stateless and can be used by any IK solver implementation.

## Functions

- damped_pseudoinverse(): Compute damped pseudoinverse of Jacobian
- check_singularity(): Check if Jacobian is near singularity
- get_manipulability(): Compute manipulability measure
- compute_pose_error(): Compute position/orientation error between poses
- compute_error_twist(): Compute error twist for differential IK
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from numpy.typing import NDArray

    from dimos.manipulation.planning.spec import Jacobian


def damped_pseudoinverse(
    J: Jacobian,
    damping: float = 0.01,
) -> NDArray[np.float64]:
    """Compute damped pseudoinverse of Jacobian.

    Uses the damped least-squares formula:
        J_pinv = J^T @ (J @ J^T + λ²I)^(-1)

    This avoids numerical issues near singularities where J @ J^T becomes
    ill-conditioned. The damping factor λ controls the trade-off between
    accuracy and stability.

    Args:
        J: 6 x n Jacobian matrix (rows: [vx, vy, vz, wx, wy, wz])
        damping: Damping factor λ (higher = more regularization, more stable)

    Returns:
        n x 6 pseudoinverse matrix

    Example:
        J = world.get_jacobian(ctx, robot_id)
        J_pinv = damped_pseudoinverse(J, damping=0.01)
        q_dot = J_pinv @ twist
    """
    JJT = J @ J.T
    I = np.eye(JJT.shape[0])
    result: NDArray[np.float64] = J.T @ np.linalg.inv(JJT + damping**2 * I)
    return result


def check_singularity(
    J: Jacobian,
    threshold: float = 0.01,
) -> bool:
    """Check if Jacobian is near singularity.

    Computes the manipulability measure (sqrt(det(J @ J^T))) and checks
    if it's below the threshold. Near singularities, the manipulability
    approaches zero.

    Args:
        J: 6 x n Jacobian matrix
        threshold: Manipulability threshold (default 0.01)

    Returns:
        True if near singularity (manipulability < threshold)

    Example:
        J = world.get_jacobian(ctx, robot_id)
        if check_singularity(J, threshold=0.001):
            logger.warning("Near singularity, using damped IK")
    """
    return get_manipulability(J) < threshold


def get_manipulability(J: Jacobian) -> float:
    """Compute manipulability measure.

    The manipulability measure w = sqrt(det(J @ J^T)) represents the
    volume of the velocity ellipsoid - how well the robot can move
    in all directions.

    Values:
    - Higher = better manipulability
    - Zero = singularity
    - Lower = near singularity

    Args:
        J: 6 x n Jacobian matrix

    Returns:
        Manipulability measure (non-negative)

    Example:
        J = world.get_jacobian(ctx, robot_id)
        w = get_manipulability(J)
        print(f"Manipulability: {w:.4f}")
    """
    JJT = J @ J.T
    det = np.linalg.det(JJT)
    return float(np.sqrt(max(0, det)))


def compute_pose_error(
    current_pose: NDArray[np.float64],
    target_pose: NDArray[np.float64],
) -> tuple[float, float]:
    """Compute position and orientation error between two poses.

    Position error is the Euclidean distance between origins.
    Orientation error is the angle of the rotation matrix relating the two frames.

    Args:
        current_pose: Current 4x4 homogeneous transform
        target_pose: Target 4x4 homogeneous transform

    Returns:
        Tuple of (position_error, orientation_error) in meters and radians

    Example:
        current = world.get_ee_pose(ctx, robot_id)
        pos_err, ori_err = compute_pose_error(current, target)
        converged = pos_err < 0.001 and ori_err < 0.01
    """
    # Position error (Euclidean distance)
    position_error = float(np.linalg.norm(target_pose[:3, 3] - current_pose[:3, 3]))

    # Orientation error using rotation matrices
    R_current = current_pose[:3, :3]
    R_target = target_pose[:3, :3]
    R_error = R_target @ R_current.T

    # Convert to axis-angle for scalar error
    trace = np.trace(R_error)
    # Clamp to valid range for arccos (numerical stability)
    cos_angle = (trace - 1) / 2
    cos_angle = np.clip(cos_angle, -1, 1)
    orientation_error = float(np.arccos(cos_angle))

    return position_error, orientation_error


def compute_error_twist(
    current_pose: NDArray[np.float64],
    target_pose: NDArray[np.float64],
    gain: float = 1.0,
) -> NDArray[np.float64]:
    """Compute error twist for differential IK.

    Computes the 6D twist (linear + angular velocity) that would move
    from the current pose toward the target pose. Used in iterative IK.

    The twist is expressed in the world frame:
        twist = [vx, vy, vz, wx, wy, wz]

    Args:
        current_pose: Current 4x4 homogeneous transform
        target_pose: Target 4x4 homogeneous transform
        gain: Proportional gain (higher = faster convergence, less stable)

    Returns:
        6D twist vector [vx, vy, vz, wx, wy, wz]

    Example:
        twist = compute_error_twist(current_pose, target_pose, gain=0.5)
        q_dot = damped_pseudoinverse(J) @ twist
        q_new = q + q_dot * dt
    """
    # Position error (linear velocity direction)
    pos_error = target_pose[:3, 3] - current_pose[:3, 3]

    # Orientation error -> angular velocity
    R_current = current_pose[:3, :3]
    R_target = target_pose[:3, :3]
    R_error = R_target @ R_current.T

    # Extract axis-angle from rotation matrix
    # Using Rodrigues' formula inverse
    trace = np.trace(R_error)
    cos_angle = (trace - 1) / 2
    cos_angle = np.clip(cos_angle, -1, 1)
    angle = np.arccos(cos_angle)

    if angle < 1e-6:
        # No rotation needed
        angular_error = np.zeros(3)
    elif angle > np.pi - 1e-6:
        # 180 degree rotation - extract axis from diagonal
        diag = np.diag(R_error)
        idx = np.argmax(diag)
        axis = np.zeros(3)
        axis[idx] = 1.0
        # Refine axis
        axis = axis * np.sqrt((diag[idx] + 1) / 2)
        angular_error = axis * angle
    else:
        # General case: axis from skew-symmetric part
        sin_angle = np.sin(angle)
        axis = np.array(
            [
                R_error[2, 1] - R_error[1, 2],
                R_error[0, 2] - R_error[2, 0],
                R_error[1, 0] - R_error[0, 1],
            ]
        ) / (2 * sin_angle)
        angular_error = axis * angle

    # Combine into twist with gain
    twist: NDArray[np.float64] = np.concatenate([pos_error * gain, angular_error * gain])

    return twist


def skew_symmetric(v: NDArray[np.float64]) -> NDArray[np.float64]:
    """Create skew-symmetric matrix from 3D vector.

    The skew-symmetric matrix [v]_x satisfies: [v]_x @ w = v cross w

    Args:
        v: 3D vector

    Returns:
        3x3 skew-symmetric matrix
    """
    return np.array(
        [
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0],
        ]
    )


def rotation_matrix_to_axis_angle(R: NDArray[np.float64]) -> tuple[NDArray[np.float64], float]:
    """Convert rotation matrix to axis-angle representation.

    Args:
        R: 3x3 rotation matrix

    Returns:
        Tuple of (axis, angle) where axis is unit vector and angle is radians
    """
    trace = np.trace(R)
    cos_angle = (trace - 1) / 2
    cos_angle = np.clip(cos_angle, -1, 1)
    angle = float(np.arccos(cos_angle))

    if angle < 1e-6:
        # Identity rotation
        return np.array([1.0, 0.0, 0.0]), 0.0

    if angle > np.pi - 1e-6:
        # 180 degree rotation
        diag = np.diag(R)
        idx = int(np.argmax(diag))
        axis = np.zeros(3)
        axis[idx] = np.sqrt((diag[idx] + 1) / 2)
        if axis[idx] > 1e-12:
            for j in range(3):
                if j != idx:
                    axis[j] = R[idx, j] / (2 * axis[idx])
        return axis, angle

    # General case
    sin_angle = np.sin(angle)
    axis = np.array(
        [
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1],
        ]
    ) / (2 * sin_angle)

    return axis, angle
