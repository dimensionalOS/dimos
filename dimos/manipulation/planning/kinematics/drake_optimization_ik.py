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

"""Drake optimization-based IK using SNOPT/IPOPT. Requires DrakeWorld."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from dimos.manipulation.planning.spec import IKResult, IKStatus, WorldRobotID, WorldSpec
from dimos.manipulation.planning.utils.kinematics_utils import compute_pose_error
from dimos.msgs.geometry_msgs import PoseStamped, Transform
from dimos.msgs.sensor_msgs import JointState
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import pose_to_matrix

if TYPE_CHECKING:
    from numpy.typing import NDArray

try:
    from pydrake.math import RigidTransform, RotationMatrix  # type: ignore[import-not-found]
    from pydrake.multibody.inverse_kinematics import (  # type: ignore[import-not-found]
        InverseKinematics,
    )
    from pydrake.solvers import Solve  # type: ignore[import-not-found]

    DRAKE_AVAILABLE = True
except ImportError:
    DRAKE_AVAILABLE = False

logger = setup_logger()


class DrakeOptimizationIK:
    """Drake optimization-based IK solver using constrained nonlinear optimization.

    Requires DrakeWorld. For backend-agnostic IK, use JacobianIK.
    """

    def __init__(self) -> None:
        if not DRAKE_AVAILABLE:
            raise ImportError("Drake is not installed. Install with: pip install drake")

    def _validate_world(self, world: WorldSpec) -> IKResult | None:
        from dimos.manipulation.planning.world.drake_world import DrakeWorld

        if not isinstance(world, DrakeWorld):
            return _create_failure_result(
                IKStatus.NO_SOLUTION, "DrakeOptimizationIK requires DrakeWorld"
            )
        if not world.is_finalized:
            return _create_failure_result(IKStatus.NO_SOLUTION, "World must be finalized before IK")
        return None

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
        """Solve IK with multiple random restarts, returning the best collision-free solution."""
        error = self._validate_world(world)
        if error is not None:
            return error

        # Convert PoseStamped to 4x4 matrix via Transform
        target_matrix = Transform(
            translation=target_pose.position,
            rotation=target_pose.orientation,
        ).to_matrix()

        # Get joint limits
        lower_limits, upper_limits = world.get_joint_limits(robot_id)

        # Get seed from current state if not provided
        if seed is None:
            with world.scratch_context() as ctx:
                seed = world.get_joint_state(ctx, robot_id)

        # Extract joint names and seed positions
        joint_names = seed.name
        seed_positions = np.array(seed.position, dtype=np.float64)

        # Target transform
        target_transform = RigidTransform(target_matrix)

        best_result: IKResult | None = None
        best_error = float("inf")

        for attempt in range(max_attempts):
            # Generate seed positions
            if attempt == 0:
                current_seed = seed_positions
            else:
                # Random seed within joint limits
                current_seed = np.random.uniform(lower_limits, upper_limits)

            # Solve IK
            result = self._solve_single(
                world=world,
                robot_id=robot_id,
                target_transform=target_transform,
                seed=current_seed,
                joint_names=joint_names,
                position_tolerance=position_tolerance,
                orientation_tolerance=orientation_tolerance,
                lower_limits=lower_limits,
                upper_limits=upper_limits,
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

    def _solve_single(
        self,
        world: WorldSpec,
        robot_id: WorldRobotID,
        target_transform: RigidTransform,
        seed: NDArray[np.float64],
        joint_names: list[str],
        position_tolerance: float,
        orientation_tolerance: float,
        lower_limits: NDArray[np.float64],
        upper_limits: NDArray[np.float64],
    ) -> IKResult:
        # Get robot data from world internals (Drake-specific access)
        robot_data = world._robots[robot_id]  # type: ignore[attr-defined]
        plant = world.plant  # type: ignore[attr-defined]

        # Create IK problem
        ik = InverseKinematics(plant)

        # Get end-effector frame
        ee_frame = robot_data.ee_frame

        # Add position constraint
        ik.AddPositionConstraint(
            frameB=ee_frame,
            p_BQ=np.array([0.0, 0.0, 0.0]),
            frameA=plant.world_frame(),
            p_AQ_lower=target_transform.translation() - np.array([position_tolerance] * 3),
            p_AQ_upper=target_transform.translation() + np.array([position_tolerance] * 3),
        )

        # Add orientation constraint
        ik.AddOrientationConstraint(
            frameAbar=plant.world_frame(),
            R_AbarA=target_transform.rotation(),
            frameBbar=ee_frame,
            R_BbarB=RotationMatrix(),
            theta_bound=orientation_tolerance,
        )

        # Get program and set initial guess
        prog = ik.get_mutable_prog()
        q = ik.q()

        # Set initial guess (full positions vector)
        full_seed = np.zeros(plant.num_positions())
        for i, joint_idx in enumerate(robot_data.joint_indices):
            full_seed[joint_idx] = seed[i]
        prog.SetInitialGuess(q, full_seed)

        # Solve
        result = Solve(prog)

        if not result.is_success():
            return _create_failure_result(
                IKStatus.NO_SOLUTION,
                f"Optimization failed: {result.get_solution_result()}",
            )

        # Extract solution for this robot's joints
        full_solution = result.GetSolution(q)
        joint_solution = np.array([full_solution[idx] for idx in robot_data.joint_indices])

        # Clip to limits
        joint_solution = np.clip(joint_solution, lower_limits, upper_limits)

        # Compute actual error using FK
        solution_state = JointState(name=joint_names, position=joint_solution.tolist())
        with world.scratch_context() as ctx:
            world.set_joint_state(ctx, robot_id, solution_state)
            actual_pose = world.get_ee_pose(ctx, robot_id)

        position_error, orientation_error = compute_pose_error(
            pose_to_matrix(actual_pose),
            target_transform.GetAsMatrix4(),
        )

        return _create_success_result(
            joint_names=joint_names,
            joint_positions=joint_solution,
            position_error=position_error,
            orientation_error=orientation_error,
            iterations=1,
        )


def _create_success_result(
    joint_names: list[str],
    joint_positions: NDArray[np.float64],
    position_error: float,
    orientation_error: float,
    iterations: int,
) -> IKResult:
    return IKResult(
        status=IKStatus.SUCCESS,
        joint_state=JointState(name=joint_names, position=joint_positions.tolist()),
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
    return IKResult(
        status=status,
        joint_state=None,
        iterations=iterations,
        message=message,
    )
