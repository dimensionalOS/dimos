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
from dataclasses import replace
from typing import TYPE_CHECKING

import numpy as np
import pink
from pink.exceptions import NoSolutionFound

from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupSelection
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.kinematics.pink_solver import (
    _get_frame_id,
    _PinkRobotContext,
    _PinkSolverCore,
    _seed_positions_for_mapping,
)
from dimos.manipulation.planning.kinematics.utils import (
    seed_positions_with_world_fallback as _seed_positions_with_world_fallback,
    unique_pose_target_frame as _unique_pose_target_frame,
)
from dimos.manipulation.planning.spec.enums import IKStatus
from dimos.manipulation.planning.spec.joint_space import JointSpace
from dimos.manipulation.planning.spec.models import IKResult
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
        self._model_context: _PinkRobotContext | None = None

    def solve(
        self,
        world: WorldSpec,
        target_pose: PoseStamped,
        seed: JointState | None = None,
        position_tolerance: float = 0.001,
        orientation_tolerance: float = 0.01,
        check_collision: bool = True,
        max_attempts: int = 10,
    ) -> IKResult:
        """Solve one planning pose target."""
        if not world.is_finalized:
            return _failure(IKStatus.NO_SOLUTION, "World must be finalized before IK")

        target_frame_name = _unique_pose_target_frame(world)
        if target_frame_name is None:
            return _failure(
                IKStatus.NO_SOLUTION,
                "PinkIK requires exactly one pose-targetable planning group for legacy solve()",
            )

        try:
            model_context = self._get_model_context(world, target_frame_name)
        except (FileNotFoundError, ImportError, ValueError) as exc:
            return _failure(IKStatus.NO_SOLUTION, f"Pink IK model setup failed: {exc}")

        if seed is None:
            with world.scratch_context() as ctx:
                seed = world.get_joint_state(ctx)

        prepared = world.get_prepared_model()
        lower_limits, upper_limits = prepared.joint_space.position_limits()
        target_model = self._target_in_model_frame(prepared.config, target_pose)
        fallback_result: IKResult | None = None

        for attempt in range(max_attempts):
            try:
                q0 = self._initial_q(model_context, seed, lower_limits, upper_limits, attempt)
                result = self._solve_targets(
                    targets=[(model_context, target_model)],
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
            if check_collision and not world.check_config_collision_free(result.joint_state):
                fallback_result = _collision_failure(result)
                continue
            return result

        if fallback_result is not None:
            return fallback_result
        return _failure(IKStatus.NO_SOLUTION, f"Pink IK failed after {max_attempts} attempts")

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
            prepared = world.get_prepared_model()
            config = prepared.config
            joint_names = list(config.joint_names)
            selected_indices = [joint_names.index(name) for name in selection.joint_names]
            seed_positions = _seed_positions_with_world_fallback(world, joint_names, seed)
        except ValueError as exc:
            return _failure(IKStatus.NO_SOLUTION, f"Pink IK mapping failed: {exc}")

        if not pose_targets:
            return _selected_result(
                _success(joint_names, seed_positions, 0.0, 0.0, 0), selection.joint_names
            )

        lower_limits, upper_limits = prepared.joint_space.position_limits()
        selected_index_set = set(selected_indices)
        locked_positions = {
            index: float(seed_positions[index])
            for index in range(len(joint_names))
            if index not in selected_index_set
        }
        targets: list[tuple[_PinkRobotContext, NDArray[np.float64]]] = []
        try:
            for group, target_pose in pose_targets.items():
                if group.tip_link is None:
                    raise ValueError(f"Planning group '{group.id}' has no pose target frame")
                targets.append(
                    (
                        self._get_model_context(world, group.tip_link),
                        self._target_in_model_frame(config, target_pose),
                    )
                )
        except (FileNotFoundError, ImportError, ValueError) as exc:
            return _failure(IKStatus.NO_SOLUTION, f"Pink IK model setup failed: {exc}")

        fallback_result: IKResult | None = None
        for attempt in range(max_attempts):
            current_positions = seed_positions.copy()
            if attempt > 0:
                retry_lower, retry_upper = _finite_retry_limits(
                    targets[0][0].joint_space,
                    seed_positions,
                    lower_limits,
                    upper_limits,
                    selected_indices,
                    attempt,
                )
                current_positions[selected_indices] = np.random.uniform(
                    retry_lower[selected_indices], retry_upper[selected_indices]
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
                return _failure(IKStatus.NO_SOLUTION, f"Pink IK mapping failed: {exc}")
            except NoSolutionFound as exc:
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
            if check_collision and not world.check_config_collision_free(result.joint_state):
                fallback_result = _collision_failure(result)
                continue
            return _selected_result(result, selection.joint_names)

        if fallback_result is not None:
            return fallback_result
        return _failure(IKStatus.NO_SOLUTION, f"Pink IK failed after {max_attempts} attempts")

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
        constraints = self._locked_joint_constraints(
            robot_context,
            seed_q,
            locked_joint_positions,
        )
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
                configuration=configuration,
                tasks=tasks,
                dt=self.config.dt,
                constraints=constraints,
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

    def _get_model_context(self, world: WorldSpec, frame_name: str) -> _PinkRobotContext:
        if self._model_context is None:
            self._model_context = self._build_robot_context(
                world.get_prepared_model(), frame_name
            )
            return self._model_context
        if self._model_context.frame_name == frame_name:
            return self._model_context
        return replace(
            self._model_context,
            frame_id=_get_frame_id(self._model_context.model, frame_name),
            frame_name=frame_name,
        )

    def _initial_q(
        self,
        context: _PinkRobotContext,
        seed: JointState,
        lower_limits: NDArray[np.float64],
        upper_limits: NDArray[np.float64],
        attempt: int,
    ) -> NDArray[np.float64]:
        seed_positions = _seed_positions_for_mapping(seed, context.mapping)
        positions = seed_positions
        if attempt > 0:
            lower, upper = _finite_retry_limits(
                context.joint_space,
                seed_positions,
                lower_limits,
                upper_limits,
                range(len(seed_positions)),
                attempt,
            )
            positions = np.random.uniform(lower, upper)
        return self._q_from_dimos_positions(context, positions)


def _finite_retry_limits(
    joint_space: JointSpace,
    seed_positions: NDArray[np.float64],
    lower_limits: NDArray[np.float64],
    upper_limits: NDArray[np.float64],
    movable_indices: Sequence[int],
    attempt: int,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Build finite retry bounds through the canonical joint-space policy."""
    margin = float(2 ** (attempt - 1))
    seed = joint_space.configuration(seed_positions)
    request_lower, request_upper = joint_space.finite_sampling_domain(seed, seed, margin)
    lower = lower_limits.copy()
    upper = upper_limits.copy()
    for index in movable_indices:
        lower[index] = request_lower[index]
        upper[index] = request_upper[index]
        if not np.isfinite(lower[index]) or not np.isfinite(upper[index]):
            raise ValueError(
                f"Cannot sample retry seed for unbounded joint '{joint_space.names[index]}'"
            )
    return lower, upper


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


def _selected_result(result: IKResult, selected_names: Sequence[str]) -> IKResult:
    if result.joint_state is None:
        return result
    positions = dict(zip(result.joint_state.name, result.joint_state.position, strict=True))
    missing_names = [name for name in selected_names if name not in positions]
    if missing_names:
        return _failure(
            IKStatus.NO_SOLUTION,
            f"Pink IK result is missing selected joints: {missing_names}",
            iterations=result.iterations,
        )
    return IKResult(
        status=result.status,
        joint_state=JointState(
            {
                "name": list(selected_names),
                "position": [float(positions[name]) for name in selected_names],
            }
        ),
        position_error=result.position_error,
        orientation_error=result.orientation_error,
        iterations=result.iterations,
        message=result.message,
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
