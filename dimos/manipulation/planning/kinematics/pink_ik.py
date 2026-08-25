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

"""Pink implementation of the manipulation planning kinematics contract."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from typing import TYPE_CHECKING

import numpy as np
import pink
from pink.exceptions import NoSolutionFound
import pinocchio

from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupSelection
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.kinematics.pink_solver import (
    _PinkRobotContext,
    _PinkSolverCore,
    _seed_positions_for_mapping,
)
from dimos.manipulation.planning.kinematics.utils import (
    groups_by_robot as _groups_by_robot,
    robot_ids_by_name as _robot_ids_by_name,
    seed_positions_with_world_fallback as _seed_positions_with_world_fallback,
    unique_pose_target_frame_for_robot as _unique_pose_target_frame_for_robot,
)
from dimos.manipulation.planning.spec.enums import IKStatus
from dimos.manipulation.planning.spec.models import IKResult, RobotName, WorldRobotID
from dimos.manipulation.planning.spec.protocols import WorldSpec
from dimos.manipulation.planning.utils.kinematics_utils import compute_pose_error
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState

if TYPE_CHECKING:
    from numpy.typing import NDArray

PinkIKConfig = PinkKinematicsConfig


class PinkIK(_PinkSolverCore):
    """Planning-only Pink backend implementing ``KinematicsSpec``."""

    def __init__(
        self,
        config: PinkKinematicsConfig | None = None,
        **overrides: bool | float | int | str,
    ) -> None:
        super().__init__(config, **overrides)
        self._robot_contexts: dict[tuple[str, str], _PinkRobotContext] = {}

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
        """Solve one planning pose target."""
        return self._solve(
            world=world,
            robot_id=robot_id,
            target_pose=target_pose,
            seed=seed,
            position_tolerance=position_tolerance,
            orientation_tolerance=orientation_tolerance,
            check_collision=check_collision,
            max_attempts=max_attempts,
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
        """Solve planning-group-scoped pose targets."""
        return self._solve_pose_targets(
            world=world,
            pose_targets=pose_targets,
            auxiliary_groups=auxiliary_groups,
            seed=seed,
            position_tolerance=position_tolerance,
            orientation_tolerance=orientation_tolerance,
            check_collision=check_collision,
            max_attempts=max_attempts,
        )

    def _solve(
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
        """Solve IK with Pink, returning the standard planning ``IKResult``."""
        if not world.is_finalized:
            return _failure(IKStatus.NO_SOLUTION, "World must be finalized before IK")

        target_frame_name = _unique_pose_target_frame_for_robot(world, robot_id)
        if target_frame_name is None:
            return _failure(
                IKStatus.NO_SOLUTION,
                "PinkIK requires exactly one pose-targetable planning group for legacy solve()",
            )

        try:
            robot_context = self._get_robot_context(world, robot_id, target_frame_name)
        except (FileNotFoundError, ImportError, ValueError) as exc:
            return _failure(IKStatus.NO_SOLUTION, f"Pink IK model setup failed: {exc}")

        if seed is None:
            with world.scratch_context() as ctx:
                seed = world.get_joint_state(ctx, robot_id)

        lower_limits, upper_limits = world.get_joint_limits(robot_id)
        target_model = self._target_in_model_frame(world.get_robot_config(robot_id), target_pose)

        fallback_result: IKResult | None = None

        for attempt in range(max_attempts):
            try:
                q0 = self._initial_q(robot_context, seed, lower_limits, upper_limits, attempt)
                result = self._solve_targets(
                    targets=[(robot_context, target_model)],
                    seed_q=q0,
                    lower_limits=lower_limits,
                    upper_limits=upper_limits,
                    position_tolerance=position_tolerance,
                    orientation_tolerance=orientation_tolerance,
                )
            except ValueError as exc:
                # Mapping failures are seed-independent: the target or seed cannot be
                # expressed in the model frame at all, so perturbed retries cannot help.
                return _failure(IKStatus.NO_SOLUTION, f"Pink IK mapping failed: {exc}")
            except NoSolutionFound as exc:
                # QP infeasibility can be seed-specific, so try the next perturbed seed.
                if fallback_result is None:
                    fallback_result = _failure(
                        IKStatus.NO_SOLUTION, f"Pink IK solver failed: {exc}"
                    )
                continue
            except Exception as exc:
                return _failure(IKStatus.NO_SOLUTION, f"Pink IK solver failed: {exc}")

            if not result.is_success() or result.joint_state is None:
                if fallback_result is None:
                    fallback_result = result
                continue

            if check_collision and not world.check_config_collision_free(
                robot_id, result.joint_state
            ):
                fallback_result = _collision_failure(result)
                continue

            return result

        if fallback_result is not None:
            return fallback_result

        return _failure(IKStatus.NO_SOLUTION, f"Pink IK failed after {max_attempts} attempts")

    def _solve_pose_targets(
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
        """Solve planning-group-scoped pose targets with Pink IK."""
        if not world.is_finalized:
            return _failure(IKStatus.NO_SOLUTION, "World must be finalized before IK")
        all_groups = tuple(pose_targets.keys()) + tuple(auxiliary_groups)
        if not all_groups:
            return _failure(
                IKStatus.NO_SOLUTION, "At least one pose target or auxiliary group is required"
            )
        bad_groups = [
            group.id
            for group in pose_targets
            if not group.has_pose_target or group.tip_link is None
        ]
        if bad_groups:
            return _failure(
                IKStatus.UNSUPPORTED,
                f"Planning groups have no pose target frame: {bad_groups}",
            )

        try:
            selection = PlanningGroupSelection.from_groups(all_groups)
            robot_ids_by_name = _robot_ids_by_name(world, selection.robot_names)
        except ValueError as exc:
            return _failure(IKStatus.NO_SOLUTION, str(exc))

        results_by_robot: dict[RobotName, IKResult] = {}
        for robot_name, groups in _groups_by_robot(all_groups).items():
            robot_id = robot_ids_by_name[robot_name]
            config = world.get_robot_config(robot_id)
            joint_names = list(config.joint_names)
            try:
                selected_indices = [
                    joint_names.index(name) for group in groups for name in group.local_joint_names
                ]
                seed_positions = _seed_positions_with_world_fallback(
                    world, robot_id, config.name, joint_names, seed
                )
            except ValueError as exc:
                return _failure(IKStatus.NO_SOLUTION, f"Pink IK mapping failed: {exc}")
            robot_pose_targets = [group for group in groups if group in pose_targets]
            if not robot_pose_targets:
                robot_result = _success(joint_names, seed_positions, 0.0, 0.0, 0)
                results_by_robot[robot_name] = robot_result
                continue

            lower_limits, upper_limits = world.get_joint_limits(robot_id)
            locked_positions = {
                index: float(seed_positions[index])
                for index in range(len(joint_names))
                if index not in set(selected_indices)
            }
            targets: list[tuple[_PinkRobotContext, NDArray[np.float64]]] = []
            try:
                for group in robot_pose_targets:
                    if group.tip_link is None:
                        raise ValueError(f"Planning group '{group.id}' has no pose target frame")
                    targets.append(
                        (
                            self._get_robot_context(world, robot_id, group.tip_link),
                            self._target_in_model_frame(config, pose_targets[group]),
                        )
                    )
            except (FileNotFoundError, ImportError, ValueError) as exc:
                return _failure(IKStatus.NO_SOLUTION, f"Pink IK model setup failed: {exc}")

            fallback_result: IKResult | None = None
            for attempt in range(max_attempts):
                current_positions = seed_positions.copy()
                if attempt > 0:
                    current_positions[selected_indices] = np.random.uniform(
                        lower_limits[selected_indices], upper_limits[selected_indices]
                    )
                try:
                    q0 = self._q_from_dimos_positions(targets[0][0], current_positions)
                    result = self._solve_targets(
                        targets=targets,
                        seed_q=q0,
                        lower_limits=lower_limits,
                        upper_limits=upper_limits,
                        position_tolerance=position_tolerance,
                        orientation_tolerance=orientation_tolerance,
                        locked_joint_positions=locked_positions,
                    )
                except ValueError as exc:
                    # Seed-independent, as in _solve: mapping cannot succeed on a retry.
                    return _failure(IKStatus.NO_SOLUTION, f"Pink IK mapping failed: {exc}")
                except NoSolutionFound as exc:
                    # QP infeasibility can be seed-specific, so try the next perturbed seed.
                    if fallback_result is None:
                        fallback_result = _failure(
                            IKStatus.NO_SOLUTION, f"Pink IK solver failed: {exc}"
                        )
                    continue
                except Exception as exc:
                    return _failure(IKStatus.NO_SOLUTION, f"Pink IK solver failed: {exc}")

                if not result.is_success() or result.joint_state is None:
                    if fallback_result is None:
                        fallback_result = result
                    continue
                results_by_robot[robot_name] = result
                break
            else:
                if fallback_result is not None:
                    return fallback_result
                return _failure(
                    IKStatus.NO_SOLUTION, f"Pink IK failed after {max_attempts} attempts"
                )

        positions_by_robot: dict[RobotName, dict[str, float]] = {}
        max_position_error = 0.0
        max_orientation_error = 0.0
        iterations = 0
        for robot_name, result in results_by_robot.items():
            if not result.is_success() or result.joint_state is None:
                return result
            positions_by_robot[robot_name] = dict(
                zip(result.joint_state.name, result.joint_state.position, strict=True)
            )
            max_position_error = max(max_position_error, result.position_error)
            max_orientation_error = max(max_orientation_error, result.orientation_error)
            iterations = max(iterations, result.iterations)

        selected_names: list[str] = []
        selected_positions: list[float] = []
        for group in selection.groups:
            robot_positions = positions_by_robot[group.robot_name]
            for global_name, local_name in zip(
                group.joint_names,
                group.local_joint_names,
                strict=True,
            ):
                if global_name in robot_positions:
                    position = robot_positions[global_name]
                elif local_name in robot_positions:
                    position = robot_positions[local_name]
                else:
                    return _failure(
                        IKStatus.NO_SOLUTION,
                        f"Pink IK result is missing selected joint '{global_name}'",
                    )
                selected_names.append(global_name)
                selected_positions.append(float(position))

        combined = IKResult(
            status=IKStatus.SUCCESS,
            joint_state=JointState(
                {
                    "name": selected_names,
                    "position": selected_positions,
                }
            ),
            position_error=max_position_error,
            orientation_error=max_orientation_error,
            iterations=iterations,
            message="Pink IK solution found",
        )
        if check_collision and not _combined_robot_results_collision_free(
            world,
            robot_ids_by_name,
            results_by_robot,
        ):
            return _collision_failure(combined)
        return combined

    def _solve_targets(
        self,
        targets: Sequence[tuple[_PinkRobotContext, NDArray[np.float64]]],
        seed_q: NDArray[np.float64],
        lower_limits: NDArray[np.float64],
        upper_limits: NDArray[np.float64],
        position_tolerance: float,
        orientation_tolerance: float,
        locked_joint_positions: Mapping[int, float] | None = None,
    ) -> IKResult:
        robot_context = targets[0][0]
        configuration, tasks = self._configuration_and_tasks(targets, seed_q)
        final_position_error = float("inf")
        final_orientation_error = float("inf")
        for iteration in range(self.config.max_iterations):
            errors = [
                compute_pose_error(self._current_frame_matrix(ctx, configuration.q), target_model)
                for ctx, target_model in targets
            ]
            final_position_error = max(error[0] for error in errors)
            final_orientation_error = max(error[1] for error in errors)
            if (
                final_position_error <= position_tolerance
                and final_orientation_error <= orientation_tolerance
            ):
                return _success(
                    robot_context.mapping.dimos_joint_names,
                    self._q_to_dimos_positions(robot_context, configuration.q),
                    final_position_error,
                    final_orientation_error,
                    iteration + 1,
                )
            self._step_configuration(
                robot_context=robot_context,
                configuration=configuration,
                tasks=tasks,
                dt=self.config.dt,
                locked_joint_positions=locked_joint_positions,
            )
            joint_positions = self._q_to_dimos_positions(robot_context, configuration.q)
            if not _within_limits(joint_positions, lower_limits, upper_limits):
                return IKResult(
                    status=IKStatus.JOINT_LIMITS,
                    joint_state=None,
                    position_error=final_position_error,
                    orientation_error=final_orientation_error,
                    iterations=iteration + 1,
                    message="Pink IK candidate violates DimOS joint limits",
                )
        return IKResult(
            status=IKStatus.NO_SOLUTION,
            joint_state=None,
            position_error=final_position_error,
            orientation_error=final_orientation_error,
            iterations=self.config.max_iterations,
            message="Pink IK did not converge within the iteration budget",
        )

    def _configuration_and_tasks(
        self,
        targets: Sequence[tuple[_PinkRobotContext, NDArray[np.float64]]],
        seed_q: NDArray[np.float64],
    ) -> tuple[pink.Configuration, Mapping[str, pink.Task]]:
        robot_context = targets[0][0]
        configuration = pink.Configuration(robot_context.model, robot_context.data, seed_q.copy())
        frame_names = tuple(context.frame_name for context, _target in targets)
        tasks = self._build_task_stack(configuration, frame_names)
        self._update_frame_task_targets(
            tasks,
            {context.frame_name: target for context, target in targets},
        )
        self._update_current_posture_target(tasks, configuration)
        return configuration, tasks

    def _get_robot_context(
        self,
        world: WorldSpec,
        robot_id: WorldRobotID,
        frame_name: str,
    ) -> _PinkRobotContext:
        cache_key = (str(robot_id), frame_name)
        if cache_key not in self._robot_contexts:
            self._robot_contexts[cache_key] = self._build_robot_context(
                world.get_robot_config(robot_id), frame_name
            )
        return self._robot_contexts[cache_key]

    def _initial_q(
        self,
        context: _PinkRobotContext,
        seed: JointState,
        lower_limits: NDArray[np.float64],
        upper_limits: NDArray[np.float64],
        attempt: int,
    ) -> NDArray[np.float64]:
        neutral = pinocchio.neutral(context.model)
        q = np.array(neutral, dtype=np.float64)

        if attempt == 0:
            positions = _seed_positions_for_mapping(seed, context.mapping)
        else:
            positions = np.random.uniform(lower_limits, upper_limits)

        for value, idx_q in zip(positions, context.mapping.idx_q, strict=True):
            q[idx_q] = value
        return q


def _within_limits(
    positions: NDArray[np.float64],
    lower_limits: NDArray[np.float64],
    upper_limits: NDArray[np.float64],
    tolerance: float = 1e-8,
) -> bool:
    return bool(
        np.all(positions >= lower_limits - tolerance)
        and np.all(positions <= upper_limits + tolerance)
    )


def _combined_robot_results_collision_free(
    world: WorldSpec,
    robot_ids_by_name: Mapping[RobotName, WorldRobotID],
    results_by_robot: Mapping[RobotName, IKResult],
) -> bool:
    with world.scratch_context() as ctx:
        for robot_name, result in results_by_robot.items():
            if result.joint_state is None:
                return False
            world.set_joint_state(ctx, robot_ids_by_name[robot_name], result.joint_state)
        return all(
            world.is_collision_free(ctx, robot_id)
            for robot_name, robot_id in robot_ids_by_name.items()
            if robot_name in results_by_robot
        )


def _success(
    joint_names: list[str],
    joint_positions: NDArray[np.float64],
    position_error: float,
    orientation_error: float,
    iterations: int,
) -> IKResult:
    return IKResult(
        status=IKStatus.SUCCESS,
        joint_state=JointState({"name": joint_names, "position": joint_positions.tolist()}),
        position_error=position_error,
        orientation_error=orientation_error,
        iterations=iterations,
        message="Pink IK solution found",
    )


def _failure(status: IKStatus, message: str, iterations: int = 0) -> IKResult:
    return IKResult(status=status, joint_state=None, iterations=iterations, message=message)


def _collision_failure(result: IKResult) -> IKResult:
    return IKResult(
        status=IKStatus.COLLISION,
        joint_state=None,
        position_error=result.position_error,
        orientation_error=result.orientation_error,
        iterations=result.iterations,
        message="Pink IK solution rejected by collision check",
    )
