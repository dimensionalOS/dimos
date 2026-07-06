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

"""Backend-agnostic Jacobian-based inverse kinematics.

JacobianIK provides iterative and differential IK methods that work with any
WorldSpec implementation. It only uses the standard WorldSpec interface methods
(get_jacobian, get_ee_pose, get_joint_limits) and doesn't depend on any specific
physics backend.

For full nonlinear optimization IK with Drake, use DrakeOptimizationIK.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import TYPE_CHECKING

import numpy as np

from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupSelection
from dimos.manipulation.planning.spec.enums import IKStatus
from dimos.manipulation.planning.spec.models import IKResult, RobotName, WorldRobotID
from dimos.manipulation.planning.spec.protocols import WorldSpec
from dimos.manipulation.planning.utils.kinematics_utils import (
    check_singularity,
    compute_error_twist,
    compute_pose_error,
    damped_pseudoinverse,
)
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import pose_to_matrix

if TYPE_CHECKING:
    from numpy.typing import NDArray

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState

logger = setup_logger()


class JacobianIK:
    """Backend-agnostic Jacobian-based IK solver.

    This class provides iterative and differential IK methods using only
    the standard WorldSpec interface. It works with any physics backend
    (Drake, MuJoCo, PyBullet, etc.).

    Methods:
        - solve_iterative(): Iterative Jacobian-based IK until convergence
        - solve_differential(): Single Jacobian step for velocity control
        - solve_differential_position_only(): Position-only differential IK
        - solve(): Wrapper for solve_iterative with multiple random restarts

    Example:
        ik = JacobianIK(damping=0.01)
        result = ik.solve_iterative(
            world, robot_id,
            target_pose=target,
            seed=current_joints,
        )
        if result.is_success():
            print(f"Solution: {result.joint_positions}")
    """

    def __init__(
        self,
        damping: float = 0.05,
        max_iterations: int = 200,
        singularity_threshold: float = 1e-6,
    ) -> None:
        """Create Jacobian IK solver.

        Args:
            damping: Damping factor for pseudoinverse (higher = more stable near singularities)
            max_iterations: Default maximum iterations for iterative IK
            singularity_threshold: Manipulability threshold for singularity detection
        """
        self._damping = damping
        self._max_iterations = max_iterations
        self._singularity_threshold = singularity_threshold

    def solve(
        self,
        world: WorldSpec,
        robot_id: WorldRobotID,
        target_pose: PoseStamped,
        seed: JointState | None = None,
        position_tolerance: float = 0.001,
        orientation_tolerance: float = 0.01,
        check_collision: bool = True,
        max_attempts: int = 10,
    ) -> IKResult:
        """Solve IK with multiple random restarts.

        Tries iterative IK from multiple starting configurations to find
        a collision-free solution.

        Args:
            world: World for FK/collision checking
            robot_id: Robot to solve IK for
            target_pose: Target end-effector pose
            seed: Initial guess (uses current state if None)
            position_tolerance: Required position accuracy (meters)
            orientation_tolerance: Required orientation accuracy (radians)
            check_collision: Whether to check collision of solution
            max_attempts: Maximum random restart attempts

        Returns:
            IKResult with solution or failure status
        """
        if not world.is_finalized:
            return _create_failure_result(IKStatus.NO_SOLUTION, "World must be finalized before IK")

        lower_limits, upper_limits = world.get_joint_limits(robot_id)

        # Get seed from current state if not provided
        if seed is None:
            with world.scratch_context() as ctx:
                seed = world.get_joint_state(ctx, robot_id)

        # Extract joint names for creating random seeds
        joint_names = seed.name

        best_result: IKResult | None = None
        best_error = float("inf")

        for attempt in range(max_attempts):
            # Generate seed JointState
            if attempt == 0:
                current_seed = seed
            else:
                # Random seed within joint limits
                random_positions = np.random.uniform(lower_limits, upper_limits)
                current_seed = JointState(
                    {"name": joint_names, "position": random_positions.tolist()}
                )

            # Solve iterative IK
            result = self.solve_iterative(
                world=world,
                robot_id=robot_id,
                target_pose=target_pose,
                seed=current_seed,
                max_iterations=self._max_iterations,
                position_tolerance=position_tolerance,
                orientation_tolerance=orientation_tolerance,
            )

            if result.is_success() and result.joint_state is not None:
                # Check collision if requested
                if check_collision:
                    if not world.check_config_collision_free(robot_id, result.joint_state):
                        continue  # Try another seed

                # Check error
                total_error = result.position_error + result.orientation_error
                if total_error < best_error:
                    best_error = total_error
                    best_result = result

                # If error is within tolerance, we're done
                if (
                    result.position_error <= position_tolerance
                    and result.orientation_error <= orientation_tolerance
                ):
                    return result

        if best_result is not None:
            return best_result

        return _create_failure_result(
            IKStatus.NO_SOLUTION,
            f"IK failed after {max_attempts} attempts",
        )

    def solve_pose_targets(
        self,
        world: WorldSpec,
        pose_targets: Mapping[PlanningGroup, PoseStamped],
        auxiliary_groups: Sequence[PlanningGroup] = (),
        seed: JointState | None = None,
        position_tolerance: float = 0.001,
        orientation_tolerance: float = 0.01,
        check_collision: bool = True,
        max_attempts: int = 10,
    ) -> IKResult:
        """Solve a planning-group pose target using group FK/Jacobian."""
        if not world.is_finalized:
            return _create_failure_result(IKStatus.NO_SOLUTION, "World must be finalized before IK")
        if not pose_targets:
            return _create_failure_result(
                IKStatus.NO_SOLUTION, "At least one pose target is required"
            )
        if len(pose_targets) != 1 or auxiliary_groups:
            return _create_failure_result(
                IKStatus.NO_SOLUTION,
                "JacobianIK supports exactly one pose target and no auxiliary planning groups",
            )

        target_group = next(iter(pose_targets.keys()))
        if not target_group.has_pose_target:
            return _create_failure_result(
                IKStatus.NO_SOLUTION,
                f"Planning group '{target_group.id}' has no pose target frame",
            )

        try:
            selection = PlanningGroupSelection.from_groups((target_group,))
            robot_ids_by_name = _robot_ids_by_name(world, selection.robot_names)
        except ValueError as exc:
            return _create_failure_result(IKStatus.NO_SOLUTION, str(exc))

        if len(selection.robot_names) != 1:
            return _create_failure_result(
                IKStatus.NO_SOLUTION,
                "JacobianIK does not support cross-robot pose IK",
            )

        robot_id = robot_ids_by_name[target_group.robot_name]
        config = world.get_robot_config(robot_id)
        try:
            seed_positions = _seed_positions_with_world_fallback(
                world,
                robot_id,
                config.name,
                list(config.joint_names),
                seed,
            )
            group_indices = [
                list(config.joint_names).index(name) for name in target_group.local_joint_names
            ]
        except ValueError as exc:
            return _create_failure_result(IKStatus.NO_SOLUTION, str(exc))

        lower_limits, upper_limits = world.get_joint_limits(robot_id)
        group_lower = lower_limits[group_indices]
        group_upper = upper_limits[group_indices]

        result = self._solve_group_iterative(
            world=world,
            robot_id=robot_id,
            group=target_group,
            target_pose=pose_targets[target_group],
            base_positions=seed_positions,
            group_indices=group_indices,
            lower_limits=group_lower,
            upper_limits=group_upper,
            max_iterations=self._max_iterations * max(1, max_attempts),
            position_tolerance=position_tolerance,
            orientation_tolerance=orientation_tolerance,
        )
        if not result.is_success() or result.joint_state is None:
            return result

        if check_collision:
            full_positions = seed_positions.copy()
            full_positions[group_indices] = np.asarray(
                result.joint_state.position, dtype=np.float64
            )
            full_state = JointState(
                {"name": list(config.joint_names), "position": full_positions.tolist()}
            )
            if not world.check_config_collision_free(robot_id, full_state):
                return _create_failure_result(IKStatus.COLLISION, "IK solution is in collision")
        return result

    def _solve_group_iterative(
        self,
        world: WorldSpec,
        robot_id: WorldRobotID,
        group: PlanningGroup,
        target_pose: PoseStamped,
        base_positions: NDArray[np.float64],
        group_indices: list[int],
        lower_limits: NDArray[np.float64],
        upper_limits: NDArray[np.float64],
        max_iterations: int,
        position_tolerance: float,
        orientation_tolerance: float,
    ) -> IKResult:
        target_matrix = Transform(
            translation=target_pose.position,
            rotation=target_pose.orientation,
        ).to_matrix()
        current_group_positions = base_positions[group_indices].copy()

        for iteration in range(max_iterations):
            full_positions = base_positions.copy()
            full_positions[group_indices] = current_group_positions
            current_state = JointState(
                {
                    "name": list(world.get_robot_config(robot_id).joint_names),
                    "position": full_positions.tolist(),
                }
            )
            with world.scratch_context() as ctx:
                world.set_joint_state(ctx, robot_id, current_state)
                current_pose = pose_to_matrix(world.get_group_ee_pose(ctx, group.id))
                pos_error, ori_error = compute_pose_error(current_pose, target_matrix)
                if pos_error <= position_tolerance and ori_error <= orientation_tolerance:
                    return _create_success_result(
                        joint_names=list(group.joint_names),
                        joint_positions=current_group_positions,
                        position_error=pos_error,
                        orientation_error=ori_error,
                        iterations=iteration + 1,
                    )
                twist = compute_error_twist(current_pose, target_matrix, gain=0.5)
                jacobian = world.get_group_jacobian(ctx, group.id)

            effective_damping = (
                self._damping * 10.0
                if check_singularity(jacobian, threshold=self._singularity_threshold)
                else self._damping
            )
            q_dot = damped_pseudoinverse(jacobian, effective_damping) @ twist
            max_delta = 0.1
            max_change = np.max(np.abs(q_dot)) if q_dot.size else 0.0
            if max_change > max_delta:
                q_dot = q_dot * (max_delta / max_change)
            current_group_positions = np.clip(
                current_group_positions + q_dot,
                lower_limits,
                upper_limits,
            )

        full_positions = base_positions.copy()
        full_positions[group_indices] = current_group_positions
        final_state = JointState(
            {
                "name": list(world.get_robot_config(robot_id).joint_names),
                "position": full_positions.tolist(),
            }
        )
        with world.scratch_context() as ctx:
            world.set_joint_state(ctx, robot_id, final_state)
            final_pose = pose_to_matrix(world.get_group_ee_pose(ctx, group.id))
            pos_error, ori_error = compute_pose_error(final_pose, target_matrix)

        return _create_failure_result(
            IKStatus.NO_SOLUTION,
            f"Did not converge after {max_iterations} iterations (pos_err={pos_error:.4f}, ori_err={ori_error:.4f})",
            iterations=max_iterations,
        )

    def solve_iterative(
        self,
        world: WorldSpec,
        robot_id: WorldRobotID,
        target_pose: PoseStamped,
        seed: JointState,
        max_iterations: int = 100,
        position_tolerance: float = 0.001,
        orientation_tolerance: float = 0.01,
    ) -> IKResult:
        """Iterative Jacobian-based IK until convergence.

        Uses the damped pseudoinverse method with adaptive step size.
        Converges when both position and orientation errors are within tolerance.

        Args:
            world: World for FK/Jacobian computation
            robot_id: Robot to solve IK for
            target_pose: Target end-effector pose
            seed: Initial joint configuration
            max_iterations: Maximum iterations before giving up
            position_tolerance: Required position accuracy (meters)
            orientation_tolerance: Required orientation accuracy (radians)

        Returns:
            IKResult with solution or failure status
        """
        # Convert to internal representation
        target_matrix = Transform(
            translation=target_pose.position,
            rotation=target_pose.orientation,
        ).to_matrix()
        current_joints = np.array(seed.position, dtype=np.float64)
        joint_names = seed.name

        max_iterations = max_iterations or self._max_iterations
        lower_limits, upper_limits = world.get_joint_limits(robot_id)

        for iteration in range(max_iterations):
            with world.scratch_context() as ctx:
                # Set current position (convert to JointState for API)
                current_state = JointState(
                    {"name": joint_names, "position": current_joints.tolist()}
                )
                world.set_joint_state(ctx, robot_id, current_state)

                # Get current pose (as matrix for error computation)
                current_pose = pose_to_matrix(world.get_ee_pose(ctx, robot_id))

                # Compute error
                pos_error, ori_error = compute_pose_error(current_pose, target_matrix)

                # Check convergence
                if pos_error <= position_tolerance and ori_error <= orientation_tolerance:
                    return _create_success_result(
                        joint_names=joint_names,
                        joint_positions=current_joints,
                        position_error=pos_error,
                        orientation_error=ori_error,
                        iterations=iteration + 1,
                    )

                # Compute twist to reduce error
                twist = compute_error_twist(current_pose, target_matrix, gain=0.5)

                # Get Jacobian
                J = world.get_jacobian(ctx, robot_id)

            # Adaptive damping near singularities
            if check_singularity(J, threshold=self._singularity_threshold):
                # Increase damping near singularity instead of failing
                effective_damping = self._damping * 10.0
            else:
                effective_damping = self._damping

            # Compute joint velocities
            J_pinv = damped_pseudoinverse(J, effective_damping)
            q_dot = J_pinv @ twist

            # Clamp maximum joint change per iteration (like reference implementations)
            max_delta = 0.1  # radians per iteration
            max_change = np.max(np.abs(q_dot))
            if max_change > max_delta:
                q_dot = q_dot * (max_delta / max_change)

            current_joints = current_joints + q_dot

            # Clip to limits
            current_joints = np.clip(current_joints, lower_limits, upper_limits)

        # Compute final error
        with world.scratch_context() as ctx:
            final_state = JointState({"name": joint_names, "position": current_joints.tolist()})
            world.set_joint_state(ctx, robot_id, final_state)
            final_pose = pose_to_matrix(world.get_ee_pose(ctx, robot_id))
            pos_error, ori_error = compute_pose_error(final_pose, target_matrix)

        return _create_failure_result(
            IKStatus.NO_SOLUTION,
            f"Did not converge after {max_iterations} iterations (pos_err={pos_error:.4f}, ori_err={ori_error:.4f})",
            iterations=max_iterations,
        )

    def solve_differential(
        self,
        world: WorldSpec,
        robot_id: WorldRobotID,
        current_joints: JointState,
        twist: Twist,
        dt: float,
    ) -> JointState | None:
        """Single Jacobian step for velocity control.

        Computes joint velocities from desired end-effector twist using
        the damped pseudoinverse method. Returns None if near singularity.

        Args:
            world: World for Jacobian computation
            robot_id: Robot to compute for
            current_joints: Current joint configuration
            twist: Desired end-effector twist (linear + angular velocity)
            dt: Time step (not used, but kept for interface compatibility)

        Returns:
            JointState with velocities, or None if near singularity
        """
        # Convert Twist to 6D array [vx, vy, vz, wx, wy, wz]
        twist_array = np.array(
            [
                twist.linear.x,
                twist.linear.y,
                twist.linear.z,
                twist.angular.x,
                twist.angular.y,
                twist.angular.z,
            ],
            dtype=np.float64,
        )

        joint_names = current_joints.name
        with world.scratch_context() as ctx:
            world.set_joint_state(ctx, robot_id, current_joints)
            J = world.get_jacobian(ctx, robot_id)

        # Check for singularity
        if check_singularity(J, threshold=self._singularity_threshold):
            logger.warning("Near singularity in differential IK")
            return None

        # Compute damped pseudoinverse
        J_pinv = damped_pseudoinverse(J, self._damping)

        # Compute joint velocities
        q_dot = J_pinv @ twist_array

        # Apply velocity limits if available
        config = world.get_robot_config(robot_id)
        if config.velocity_limits is not None:
            velocity_limits = np.array(config.velocity_limits)
            # Only consider joints with non-zero velocity limits
            nonzero_mask = velocity_limits > 0
            if np.any(nonzero_mask):
                max_ratio = np.max(np.abs(q_dot[nonzero_mask]) / velocity_limits[nonzero_mask])
                if max_ratio > 1.0:
                    q_dot = q_dot / max_ratio

        return JointState({"name": joint_names, "velocity": q_dot.tolist()})

    def solve_differential_position_only(
        self,
        world: WorldSpec,
        robot_id: WorldRobotID,
        current_joints: JointState,
        linear_velocity: Vector3,
    ) -> JointState | None:
        """Position-only differential IK using linear Jacobian.

        Computes joint velocities from desired linear velocity, ignoring
        orientation. Returns None if near singularity.

        Args:
            world: World for Jacobian computation
            robot_id: Robot to compute for
            current_joints: Current joint configuration
            linear_velocity: Desired linear velocity

        Returns:
            JointState with velocities, or None if singular
        """
        # Convert Vector3 to array
        vel_array = np.array(
            [linear_velocity.x, linear_velocity.y, linear_velocity.z], dtype=np.float64
        )

        joint_names = current_joints.name
        with world.scratch_context() as ctx:
            world.set_joint_state(ctx, robot_id, current_joints)
            J = world.get_jacobian(ctx, robot_id)

        # Extract linear part (first 3 rows)
        J_linear = J[:3, :]

        # Check for singularity
        JJT = J_linear @ J_linear.T
        manipulability = np.sqrt(max(0, np.linalg.det(JJT)))
        if manipulability < self._singularity_threshold:
            return None

        # Compute damped pseudoinverse
        I = np.eye(3)
        J_pinv = J_linear.T @ np.linalg.inv(JJT + self._damping**2 * I)

        # Compute joint velocities
        q_dot = J_pinv @ vel_array
        return JointState({"name": joint_names, "velocity": q_dot.tolist()})


# Result Helpers


def _create_success_result(
    joint_names: list[str],
    joint_positions: NDArray[np.float64],
    position_error: float,
    orientation_error: float,
    iterations: int,
) -> IKResult:
    """Create a successful IK result."""
    return IKResult(
        status=IKStatus.SUCCESS,
        joint_state=JointState({"name": joint_names, "position": joint_positions.tolist()}),
        position_error=position_error,
        orientation_error=orientation_error,
        iterations=iterations,
        message="IK solution found",
    )


def _create_failure_result(
    status: IKStatus,
    message: str,
    iterations: int = 0,
) -> IKResult:
    """Create a failed IK result."""
    return IKResult(
        status=status,
        joint_state=None,
        iterations=iterations,
        message=message,
    )


def _robot_ids_by_name(
    world: WorldSpec,
    robot_names: tuple[RobotName, ...],
) -> dict[RobotName, WorldRobotID]:
    robot_ids_by_name: dict[RobotName, WorldRobotID] = {}
    for robot_name in robot_names:
        matches = [
            robot_id
            for robot_id in world.get_robot_ids()
            if world.get_robot_config(robot_id).name == robot_name
        ]
        if not matches:
            raise ValueError(f"Robot '{robot_name}' not found")
        if len(matches) > 1:
            raise ValueError(f"Robot name '{robot_name}' is not unique in planning world")
        robot_ids_by_name[robot_name] = matches[0]
    return robot_ids_by_name


def _seed_positions_with_world_fallback(
    world: WorldSpec,
    robot_id: WorldRobotID,
    robot_name: RobotName,
    local_joint_names: list[str],
    seed: JointState | None,
) -> NDArray[np.float64]:
    """Return full robot positions, reading world only for missing seed joints."""
    if seed is None:
        with world.scratch_context() as ctx:
            current = world.get_joint_state(ctx, robot_id)
        return _positions_by_local_name(current, robot_name, local_joint_names)

    try:
        return _positions_by_local_name(seed, robot_name, local_joint_names)
    except ValueError:
        with world.scratch_context() as ctx:
            current = world.get_joint_state(ctx, robot_id)
        fallback_positions = _positions_by_local_name(current, robot_name, local_joint_names)
        seed_positions = _partial_positions_by_local_name(seed, robot_name, local_joint_names)
        for local_name, position in seed_positions.items():
            fallback_positions[local_joint_names.index(local_name)] = position
        return fallback_positions


def _positions_by_local_name(
    joint_state: JointState,
    robot_name: RobotName,
    local_joint_names: list[str],
) -> NDArray[np.float64]:
    if not joint_state.name:
        if len(joint_state.position) != len(local_joint_names):
            raise ValueError(
                f"JointState has {len(joint_state.position)} positions for "
                f"{len(local_joint_names)} joints"
            )
        return np.asarray(joint_state.position, dtype=np.float64)

    positions_by_name = dict(zip(joint_state.name, joint_state.position, strict=True))
    positions: list[float] = []
    missing: list[str] = []
    for local_name in local_joint_names:
        global_name = f"{robot_name}/{local_name}"
        if local_name in positions_by_name:
            positions.append(float(positions_by_name[local_name]))
        elif global_name in positions_by_name:
            positions.append(float(positions_by_name[global_name]))
        else:
            missing.append(local_name)
    if missing:
        raise ValueError(f"JointState missing joints: {missing}")
    return np.asarray(positions, dtype=np.float64)


def _partial_positions_by_local_name(
    joint_state: JointState,
    robot_name: RobotName,
    local_joint_names: list[str],
) -> dict[str, float]:
    positions_by_name = dict(zip(joint_state.name, joint_state.position, strict=True))
    known_local_names = set(local_joint_names)
    positions: dict[str, float] = {}
    for name, position in positions_by_name.items():
        if name in known_local_names:
            positions[name] = float(position)
            continue
        prefix = f"{robot_name}/"
        if name.startswith(prefix):
            local_name = name[len(prefix) :]
            if local_name in known_local_names:
                positions[local_name] = float(position)
                continue
        raise ValueError(f"Unrecognized seed joint '{name}'")
    return positions
