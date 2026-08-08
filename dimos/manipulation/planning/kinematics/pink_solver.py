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

"""Shared Pink model, task-stack, and QP implementation."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import TYPE_CHECKING

import numpy as np

try:
    import pink
    import pinocchio
    import qpsolvers
except ImportError as exc:
    msg = "Pink IK dependencies not found; install them with: uv sync --extra manipulation."
    raise ImportError(msg) from exc

from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupSelection
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.kinematics.utils import (
    groups_by_robot as _groups_by_robot,
    robot_ids_by_name as _robot_ids_by_name,
    seed_positions_with_world_fallback as _seed_positions_with_world_fallback,
    unique_pose_target_frame_for_robot as _unique_pose_target_frame_for_robot,
)
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import IKStatus
from dimos.manipulation.planning.spec.models import IKResult, RobotName, WorldRobotID
from dimos.manipulation.planning.spec.protocols import WorldSpec
from dimos.manipulation.planning.utils.kinematics_utils import compute_pose_error
from dimos.manipulation.planning.utils.mesh_utils import prepare_urdf_for_drake
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import matrix_to_pose, pose_to_matrix

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = setup_logger()


class PinkJointLimitError(ValueError):
    """Raised when a joint value is outside its allowed limit tolerance."""

    def __init__(
        self,
        *,
        joint_name: str,
        value: float,
        lower: float,
        upper: float,
        tolerance: float,
    ) -> None:
        self.joint_name = joint_name
        self.value = value
        self.lower = lower
        self.upper = upper
        self.tolerance = tolerance
        self.boundary = "lower" if value < lower else "upper"
        limit = lower if self.boundary == "lower" else upper
        super().__init__(
            f"Measured joint '{joint_name}' value {value} exceeds {self.boundary} limit "
            f"{limit} beyond feedback tolerance {tolerance}"
        )


_MANIPULATION_EXTRA_HINT = "Install manipulation dependencies with: uv sync --extra manipulation."


@dataclass(frozen=True)
class _JointMapping:
    dimos_joint_names: list[str]
    model_joint_names: list[str]
    idx_q: list[int]
    idx_v: list[int]


@dataclass
class _PinkRobotContext:
    model: pinocchio.Model
    data: pinocchio.Data
    frame_id: int
    frame_name: str
    mapping: _JointMapping


@dataclass
class _PinkControlContext:
    robot: _PinkRobotContext
    frames: Mapping[str, _PinkRobotContext]
    tasks: Mapping[str, pink.Task] | None = None


_CURRENT_POSTURE_TASK = "posture/current"


def _frame_task_key(frame_name: str) -> str:
    return f"frame/{frame_name}"


class _PinkSolverCore:
    """Private Pink mechanics shared by planning and pose-target control."""

    def __init__(
        self,
        config: PinkKinematicsConfig | None = None,
        **overrides: bool | float | int | str,
    ) -> None:
        """Create a Pink IK backend.

        Args:
            config: Optional Pink IK configuration object.
            **overrides: Per-field overrides applied to ``config`` for factory/CLI use.
        """
        config_values = (config or PinkKinematicsConfig()).model_dump()
        config_values.update(overrides)
        self.config = PinkKinematicsConfig(**config_values)
        if self.config.solver not in qpsolvers.available_solvers:
            raise ImportError(
                f"Pink IK solver '{self.config.solver}' is unavailable. "
                f"Available solvers: {sorted(qpsolvers.available_solvers)}. "
                f"{_MANIPULATION_EXTRA_HINT}"
            )
        self._robot_contexts: dict[tuple[str, str], _PinkRobotContext] = {}
        self._control_contexts: dict[
            tuple[str, tuple[str, ...], tuple[str, ...]], _PinkControlContext
        ] = {}

    def _step_frame_targets(
        self,
        robot_model: RobotModelConfig,
        frame_targets: Mapping[str, PoseStamped],
        controlled_joints: Sequence[str],
        command_state: JointState,
        measured_state: JointState,
        max_command_tracking_error_rad: float,
        feedback_limit_tolerance: float,
        command_limit_margin: float,
        dt: float | None = None,
        max_joint_velocity_rad_s: float = 5.0,
        joint_velocity_limits_rad_s: Mapping[str, float] | None = None,
        joint_command_filter_cutoff_hz: float | None = None,
    ) -> JointState:
        """Perform one bounded Pink update for one robot's frame targets.

        ``command_state`` advances the generated trajectory while
        ``measured_state`` independently bounds it against hardware feedback.
        Unlike the planning methods, this performs no convergence loop, random
        restart, collision query, or world lookup.
        """
        if not frame_targets:
            raise ValueError("Pink frame-target step requires at least one target")
        joint_names = tuple(controlled_joints)
        if not joint_names:
            raise ValueError("Pink frame-target step requires at least one controlled joint")
        if len(set(joint_names)) != len(joint_names):
            raise ValueError("Pink controlled joint names must be unique")

        frame_names = tuple(frame_targets)
        control_context = self._get_control_context(robot_model, frame_names, joint_names)
        robot_context = control_context.robot
        targets = {
            frame_name: self._target_in_model_frame(robot_model, frame_targets[frame_name])
            for frame_name in frame_names
        }
        if not np.isfinite(max_command_tracking_error_rad) or max_command_tracking_error_rad <= 0.0:
            raise ValueError("Pink command tracking error must be positive and finite")
        step_dt = self.config.dt if dt is None else dt
        if not np.isfinite(step_dt) or step_dt <= 0.0:
            raise ValueError("Pink streaming timestep must be positive and finite")
        if not np.isfinite(max_joint_velocity_rad_s) or max_joint_velocity_rad_s <= 0.0:
            raise ValueError("Pink streaming joint velocity limit must be positive and finite")
        if joint_command_filter_cutoff_hz is not None and (
            not np.isfinite(joint_command_filter_cutoff_hz) or joint_command_filter_cutoff_hz <= 0.0
        ):
            raise ValueError("Pink streaming command filter cutoff must be positive and finite")
        joint_velocity_limits = joint_velocity_limits_rad_s or {}

        previous_positions = _seed_positions_for_mapping(command_state, robot_context.mapping)
        measured_positions = _seed_positions_for_mapping(measured_state, robot_context.mapping)
        raw_command_q = self._q_from_dimos_positions(robot_context, previous_positions)
        measured_q = self._q_from_dimos_positions(robot_context, measured_positions)
        self._validate_streaming_feedback(robot_context, measured_q, feedback_limit_tolerance)
        command_q = self._clamp_streaming_configuration(
            robot_context, raw_command_q, command_limit_margin
        )
        configuration = pink.Configuration(
            robot_context.model,
            robot_context.data,
            command_q.copy(),
        )
        if control_context.tasks is None:
            control_context.tasks = self._build_task_stack(configuration, frame_names)
        tasks = control_context.tasks
        self._update_frame_task_targets(tasks, targets)
        self._update_current_posture_target(tasks, configuration)
        self._step_configuration(
            robot_context=robot_context,
            configuration=configuration,
            tasks=tasks,
            dt=step_dt,
        )
        candidate_positions = self._q_to_dimos_positions(robot_context, configuration.q)
        if joint_command_filter_cutoff_hz is not None:
            alpha = -np.expm1(-2.0 * np.pi * joint_command_filter_cutoff_hz * step_dt)
            candidate_positions = previous_positions + alpha * (
                candidate_positions - previous_positions
            )
        command_positions = self._apply_streaming_command_envelope(
            context=robot_context,
            candidate=candidate_positions,
            previous=previous_positions,
            measured=measured_positions,
            dt=step_dt,
            max_joint_velocity=max_joint_velocity_rad_s,
            joint_velocity_limits=joint_velocity_limits,
            max_tracking_error=max_command_tracking_error_rad,
            command_limit_margin=command_limit_margin,
        )
        return JointState(
            {
                "name": list(joint_names),
                "position": command_positions.tolist(),
            }
        )

    def _validate_frame_targets(
        self,
        robot_model: RobotModelConfig,
        frame_names: Sequence[str],
        controlled_joints: Sequence[str],
        command_limit_margin: float,
    ) -> None:
        """Build and validate the model, controlled joints, and target frames."""
        frames = tuple(frame_names)
        joints = tuple(controlled_joints)
        if not frames or len(set(frames)) != len(frames):
            raise ValueError("Pink target frame names must be non-empty and unique")
        if not joints or len(set(joints)) != len(joints):
            raise ValueError("Pink controlled joint names must be non-empty and unique")
        context = self._get_control_context(robot_model, frames, joints)
        self._validate_streaming_limit_margin(context.robot, command_limit_margin)

    def _frame_poses(
        self,
        robot_model: RobotModelConfig,
        frame_names: Sequence[str],
        controlled_joints: Sequence[str],
        seed: JointState,
    ) -> dict[str, PoseStamped]:
        """Return current world poses for named frames at a joint seed."""
        if not frame_names:
            raise ValueError("Pink frame pose query requires at least one frame")
        joint_names = tuple(controlled_joints)
        context = self._get_control_context(robot_model, tuple(frame_names), joint_names)
        positions = _seed_positions_for_mapping(seed, context.robot.mapping)
        q = self._q_from_dimos_positions(context.robot, positions)
        base_world = pose_to_matrix(robot_model.base_pose)
        poses: dict[str, PoseStamped] = {}
        for frame_name, frame_context in context.frames.items():
            pose = matrix_to_pose(base_world @ self._current_frame_matrix(frame_context, q))
            poses[frame_name] = PoseStamped(
                frame_id=robot_model.base_link,
                position=pose.position,
                orientation=pose.orientation,
            )
        return poses

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
                result = self._solve_single(
                    robot_context=robot_context,
                    target_model=target_model,
                    seed_q=q0,
                    lower_limits=lower_limits,
                    upper_limits=upper_limits,
                    position_tolerance=position_tolerance,
                    orientation_tolerance=orientation_tolerance,
                )
            except ValueError as exc:
                return _failure(IKStatus.NO_SOLUTION, f"Pink IK mapping failed: {exc}")
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
                    if len(targets) == 1:
                        result = self._solve_single(
                            robot_context=targets[0][0],
                            target_model=targets[0][1],
                            seed_q=q0,
                            lower_limits=lower_limits,
                            upper_limits=upper_limits,
                            position_tolerance=position_tolerance,
                            orientation_tolerance=orientation_tolerance,
                            locked_joint_positions=locked_positions,
                        )
                    else:
                        result = self._solve_multi(
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

    def _solve_multi(
        self,
        targets: Sequence[tuple[_PinkRobotContext, NDArray[np.float64]]],
        seed_q: NDArray[np.float64],
        lower_limits: NDArray[np.float64],
        upper_limits: NDArray[np.float64],
        position_tolerance: float,
        orientation_tolerance: float,
        locked_joint_positions: Mapping[int, float] | None = None,
    ) -> IKResult:
        return self._solve_targets(
            targets=targets,
            seed_q=seed_q,
            lower_limits=lower_limits,
            upper_limits=upper_limits,
            position_tolerance=position_tolerance,
            orientation_tolerance=orientation_tolerance,
            locked_joint_positions=locked_joint_positions,
        )

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

    def _solve_single(
        self,
        robot_context: _PinkRobotContext,
        target_model: NDArray[np.float64],
        seed_q: NDArray[np.float64],
        lower_limits: NDArray[np.float64],
        upper_limits: NDArray[np.float64],
        position_tolerance: float,
        orientation_tolerance: float,
        locked_joint_positions: Mapping[int, float] | None = None,
    ) -> IKResult:
        return self._solve_targets(
            targets=[(robot_context, target_model)],
            seed_q=seed_q,
            lower_limits=lower_limits,
            upper_limits=upper_limits,
            position_tolerance=position_tolerance,
            orientation_tolerance=orientation_tolerance,
            locked_joint_positions=locked_joint_positions,
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

    def _create_tasks(
        self,
        configuration: pink.Configuration,
        target_frames: tuple[str, ...],
    ) -> dict[str, pink.Task]:
        """Create the ordered Pink task stack for a solve context.

        Subclasses should call ``super()``, then tune or replace named tasks
        and add auxiliary entries. The returned structure is validated and
        frozen before it is used by the solver.
        """
        tasks = {
            _frame_task_key(frame_name): pink.tasks.FrameTask(
                frame_name,
                position_cost=self.config.position_cost,
                orientation_cost=self.config.orientation_cost,
                lm_damping=self.config.lm_damping,
                gain=self.config.gain,
            )
            for frame_name in target_frames
        }
        if self.config.posture_cost > 0.0:
            tasks[_CURRENT_POSTURE_TASK] = pink.tasks.PostureTask(cost=self.config.posture_cost)
        return tasks

    def _before_solve(
        self,
        tasks: Mapping[str, pink.Task],
        configuration: pink.Configuration,
        dt: float,
    ) -> None:
        """Update dynamic auxiliary task inputs before a Pink solve."""

    def _after_solve(
        self,
        tasks: Mapping[str, pink.Task],
        velocity: NDArray[np.float64],
        dt: float,
    ) -> None:
        """Record successful Pink output for explicitly temporal tasks."""

    def _build_task_stack(
        self,
        configuration: pink.Configuration,
        target_frames: tuple[str, ...],
    ) -> Mapping[str, pink.Task]:
        tasks = self._create_tasks(configuration, target_frames)
        if not isinstance(tasks, dict):
            raise TypeError("Pink _create_tasks() must return a dict")
        for name, task in tasks.items():
            if not isinstance(name, str) or not name:
                raise ValueError("Pink task names must be non-empty strings")
            if task is None:
                raise ValueError(f"Pink task '{name}' cannot be None")
        for frame_name in target_frames:
            key = _frame_task_key(frame_name)
            task = tasks.get(key)
            if task is None or not callable(getattr(task, "set_target", None)):
                raise ValueError(f"Pink task stack requires '{key}' to be a frame-target task")
            task_frame = getattr(task, "frame", None)
            if task_frame != frame_name:
                raise ValueError(
                    f"Pink task '{key}' targets frame '{task_frame}', expected '{frame_name}'"
                )
        return MappingProxyType(dict(tasks))

    def _update_frame_task_targets(
        self,
        tasks: Mapping[str, pink.Task],
        targets: Mapping[str, NDArray[np.float64]],
    ) -> None:
        for frame_name, target_model in targets.items():
            tasks[_frame_task_key(frame_name)].set_target(_matrix_to_se3(target_model))

    def _update_current_posture_target(
        self,
        tasks: Mapping[str, pink.Task],
        configuration: pink.Configuration,
    ) -> None:
        posture_task = tasks.get(_CURRENT_POSTURE_TASK)
        if posture_task is None:
            return
        if self.config.joint_limit_posture_margin > 0.0:
            posture_task.set_target(
                _inward_joint_limit_posture(
                    configuration,
                    self.config.joint_limit_posture_margin,
                )
            )
        else:
            posture_task.set_target_from_configuration(configuration)

    def _step_configuration(
        self,
        robot_context: _PinkRobotContext,
        configuration: pink.Configuration,
        tasks: Mapping[str, pink.Task],
        dt: float,
        locked_joint_positions: Mapping[int, float] | None = None,
    ) -> None:
        self._before_solve(tasks, configuration, dt)
        velocity = pink.solve_ik(
            configuration,
            list(tasks.values()),
            dt,
            solver=self.config.solver,
            damping=self.config.damping,
            safety_break=self.config.safety_break,
        )
        self._after_solve(tasks, velocity, dt)
        configuration.integrate_inplace(velocity, dt)
        if locked_joint_positions:
            locked_q = configuration.q.copy()
            for local_index, value in locked_joint_positions.items():
                locked_q[robot_context.mapping.idx_q[local_index]] = value
            configuration.update(locked_q)

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

    def _get_control_context(
        self,
        config: RobotModelConfig,
        frame_names: Sequence[str],
        controlled_joints: Sequence[str],
    ) -> _PinkControlContext:
        frames = tuple(frame_names)
        if not frames:
            raise ValueError("Pink control context requires at least one frame")
        cache_key = (
            str(Path(config.model_path).resolve()),
            frames,
            tuple(controlled_joints),
        )
        if cache_key not in self._control_contexts:
            robot_context = self._build_robot_context(
                config,
                frames[0],
                controlled_joints,
            )
            contexts = {frames[0]: robot_context}
            for frame_name in frames[1:]:
                contexts[frame_name] = _PinkRobotContext(
                    model=robot_context.model,
                    data=robot_context.data,
                    frame_id=_get_frame_id(robot_context.model, frame_name),
                    frame_name=frame_name,
                    mapping=robot_context.mapping,
                )
            self._control_contexts[cache_key] = _PinkControlContext(
                robot=robot_context,
                frames=MappingProxyType(contexts),
            )
        return self._control_contexts[cache_key]

    def _build_robot_context(
        self,
        config: RobotModelConfig,
        frame_name: str,
        controlled_joints: Sequence[str] | None = None,
    ) -> _PinkRobotContext:
        model_path = Path(config.model_path).resolve()
        if not model_path.exists():
            raise FileNotFoundError(f"Robot model not found: {model_path}")

        if model_path.suffix == ".xml":
            model = pinocchio.buildModelFromMJCF(str(model_path))
        else:
            prepared_path = prepare_urdf_for_drake(
                urdf_path=model_path,
                package_paths=config.package_paths,
                xacro_args=config.xacro_args,
                convert_meshes=config.auto_convert_meshes,
            )
            model = pinocchio.buildModelFromUrdf(str(prepared_path))

        data = model.createData()
        _assert_base_link_is_model_root(model, config.base_link)
        frame_id = _get_frame_id(model, frame_name)
        mapping = _build_joint_mapping(model, config, controlled_joints)
        return _PinkRobotContext(
            model=model,
            data=data,
            frame_id=frame_id,
            frame_name=frame_name,
            mapping=mapping,
        )

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

    def _q_from_dimos_positions(
        self,
        context: _PinkRobotContext,
        positions: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        q = np.array(pinocchio.neutral(context.model), dtype=np.float64)
        if len(positions) != len(context.mapping.idx_q):
            raise ValueError(
                f"Seed has {len(positions)} positions for {len(context.mapping.idx_q)} joints"
            )
        for value, idx_q in zip(positions, context.mapping.idx_q, strict=True):
            q[idx_q] = value
        return q

    def _q_to_dimos_positions(
        self, context: _PinkRobotContext, q: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        return np.array([q[idx_q] for idx_q in context.mapping.idx_q], dtype=np.float64)

    def _validate_streaming_feedback(
        self,
        context: _PinkRobotContext,
        measured_q: NDArray[np.float64],
        tolerance: float,
    ) -> None:
        for joint_name, q_index, lower, upper in _bounded_controlled_joint_limits(context):
            value = float(measured_q[q_index])
            if value < lower - tolerance or value > upper + tolerance:
                raise PinkJointLimitError(
                    joint_name=joint_name,
                    value=value,
                    lower=lower,
                    upper=upper,
                    tolerance=tolerance,
                )

    def _clamp_streaming_configuration(
        self,
        context: _PinkRobotContext,
        q: NDArray[np.float64],
        margin: float,
    ) -> NDArray[np.float64]:
        normalized = q.copy()
        for _joint_name, q_index, lower, upper in _bounded_controlled_joint_limits(context):
            value = float(q[q_index])
            normalized[q_index] = np.clip(value, lower + margin, upper - margin)
        return normalized

    def _apply_streaming_command_envelope(
        self,
        *,
        context: _PinkRobotContext,
        candidate: NDArray[np.float64],
        previous: NDArray[np.float64],
        measured: NDArray[np.float64],
        dt: float,
        max_joint_velocity: float,
        joint_velocity_limits: Mapping[str, float],
        max_tracking_error: float,
        command_limit_margin: float,
    ) -> NDArray[np.float64]:
        """Clamp one Pink candidate to the complete streaming safety envelope."""
        joint_count = len(context.mapping.idx_q)
        expected_shape = (joint_count,)
        if any(values.shape != expected_shape for values in (candidate, previous, measured)):
            raise ValueError("Pink streaming states do not match the controlled joint count")

        model_velocity = np.asarray(context.model.velocityLimit, dtype=np.float64)
        if model_velocity.shape != (context.model.nv,):
            raise ValueError("Pink model velocity limits do not match its tangent dimension")
        velocity_limits = np.full(joint_count, max_joint_velocity, dtype=np.float64)
        for index, joint_name in enumerate(context.mapping.dimos_joint_names):
            override = joint_velocity_limits.get(joint_name)
            if override is not None:
                velocity_limits[index] = override
        for index, v_index in enumerate(context.mapping.idx_v):
            urdf_velocity = float(model_velocity[v_index])
            if np.isfinite(urdf_velocity) and 1e-10 < urdf_velocity < 1e20:
                velocity_limits[index] = min(velocity_limits[index], urdf_velocity)

        step_limits = velocity_limits * dt
        lower = np.maximum(previous - step_limits, measured - max_tracking_error)
        upper = np.minimum(previous + step_limits, measured + max_tracking_error)

        controlled_index_by_q = {
            q_index: controlled_index
            for controlled_index, q_index in enumerate(context.mapping.idx_q)
        }
        margin = command_limit_margin
        for _joint_name, q_index, urdf_lower, urdf_upper in _bounded_controlled_joint_limits(
            context
        ):
            index = controlled_index_by_q[q_index]
            lower[index] = max(lower[index], urdf_lower + margin)
            upper[index] = min(upper[index], urdf_upper - margin)

        invalid = np.flatnonzero(lower > upper)
        if invalid.size:
            joint_name = context.mapping.dimos_joint_names[int(invalid[0])]
            raise ValueError(f"Pink streaming command envelope is empty for '{joint_name}'")
        return np.clip(candidate, lower, upper)

    def _validate_streaming_limit_margin(self, context: _PinkRobotContext, margin: float) -> None:
        for joint_name, _q_index, lower, upper in _bounded_controlled_joint_limits(context):
            if lower + margin > upper - margin:
                raise ValueError(
                    f"Pink command limit margin {margin} leaves no valid range for "
                    f"controlled joint '{joint_name}' with limits [{lower}, {upper}]"
                )

    def _current_frame_matrix(
        self, context: _PinkRobotContext, q: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        pinocchio.forwardKinematics(context.model, context.data, q)
        pinocchio.updateFramePlacements(context.model, context.data)
        placement = context.data.oMf[context.frame_id]
        matrix: NDArray[np.float64] = np.eye(4)
        matrix[:3, :3] = np.asarray(placement.rotation, dtype=np.float64)
        matrix[:3, 3] = np.asarray(placement.translation, dtype=np.float64)
        return matrix

    def _target_in_model_frame(
        self, config: RobotModelConfig, target_pose: PoseStamped
    ) -> NDArray[np.float64]:
        target_world = pose_to_matrix(target_pose)
        base_world = pose_to_matrix(config.base_pose)
        target_model: NDArray[np.float64] = np.asarray(
            np.linalg.inv(base_world) @ target_world, dtype=np.float64
        )
        return target_model


def _build_joint_mapping(
    model: pinocchio.Model,
    config: RobotModelConfig,
    controlled_joints: Sequence[str] | None = None,
) -> _JointMapping:
    idx_q: list[int] = []
    idx_v: list[int] = []
    model_joint_names: list[str] = []
    dimos_joint_names = list(controlled_joints or config.joint_names)

    for dimos_name in dimos_joint_names:
        model_joint_name = config.get_urdf_joint_name(dimos_name)
        joint_id = _get_joint_id(model, model_joint_name)
        joint = model.joints[joint_id]
        nq = int(getattr(joint, "nq", 1))
        if nq != 1:
            raise ValueError(
                f"PinkIK currently supports one-DoF controlled joints; "
                f"joint '{model_joint_name}' has nq={nq}"
            )
        nv = int(getattr(joint, "nv", 1))
        if nv != 1:
            raise ValueError(
                f"PinkIK currently supports one-DoF controlled joints; "
                f"joint '{model_joint_name}' has nv={nv}"
            )
        idx_q.append(int(joint.idx_q))
        idx_v.append(int(joint.idx_v))
        model_joint_names.append(model_joint_name)

    return _JointMapping(
        dimos_joint_names=dimos_joint_names,
        model_joint_names=model_joint_names,
        idx_q=idx_q,
        idx_v=idx_v,
    )


def _get_joint_id(model: pinocchio.Model, joint_name: str) -> int:
    if hasattr(model, "existJointName") and not model.existJointName(joint_name):
        raise ValueError(_missing_joint_message(model, joint_name))
    joint_id = int(model.getJointId(joint_name))
    if joint_id >= len(model.joints):
        raise ValueError(_missing_joint_message(model, joint_name))
    return joint_id


def _inward_joint_limit_posture(
    configuration: pink.Configuration, margin: float
) -> NDArray[np.float64]:
    """Return the seed posture with near-limit coordinates moved inward."""
    target: NDArray[np.float64] = np.asarray(
        configuration.q,
        dtype=np.float64,
    ).copy()
    lower = np.asarray(configuration.model.lowerPositionLimit, dtype=np.float64)
    upper = np.asarray(configuration.model.upperPositionLimit, dtype=np.float64)
    for index, (value, lower_limit, upper_limit) in enumerate(
        zip(target, lower, upper, strict=True)
    ):
        lower_target = lower_limit + margin
        upper_target = upper_limit - margin
        if lower_target > upper_target:
            target[index] = 0.5 * lower_limit + 0.5 * upper_limit
        elif value < lower_target:
            target[index] = lower_target
        elif value > upper_target:
            target[index] = upper_target
    return target


def _bounded_controlled_joint_limits(
    context: _PinkRobotContext,
) -> list[tuple[str, int, float, float]]:
    lower_limits = np.asarray(context.model.lowerPositionLimit, dtype=np.float64)
    upper_limits = np.asarray(context.model.upperPositionLimit, dtype=np.float64)
    if lower_limits.shape != (context.model.nq,) or upper_limits.shape != (context.model.nq,):
        raise ValueError("Pink model position limits do not match its configuration dimension")

    bounded: list[tuple[str, int, float, float]] = []
    for joint_name, q_index in zip(
        context.mapping.dimos_joint_names, context.mapping.idx_q, strict=True
    ):
        lower = float(lower_limits[q_index])
        upper = float(upper_limits[q_index])
        if (
            np.isfinite(lower)
            and np.isfinite(upper)
            and lower > -1e20
            and upper < 1e20
            and upper > lower + 1e-10
        ):
            bounded.append((joint_name, q_index, lower, upper))
    return bounded


def _get_frame_id(model: pinocchio.Model, frame_name: str) -> int:
    if hasattr(model, "existFrame") and not model.existFrame(frame_name):
        raise ValueError(_missing_frame_message(model, frame_name))
    frame_id = int(model.getFrameId(frame_name))
    if frame_id >= len(model.frames):
        raise ValueError(_missing_frame_message(model, frame_name))
    return frame_id


def _assert_base_link_is_model_root(model: pinocchio.Model, base_link: str) -> None:
    """Validate that the configured base link is fixed at the Pinocchio model root."""
    frame_id = _get_frame_id(model, base_link)
    frame = model.frames[frame_id]
    parent_joint = int(getattr(frame, "parentJoint", 0))
    if parent_joint != 0:
        raise ValueError(
            f"PinkIK expects RobotModelConfig.base_link '{base_link}' to be the model root; "
            f"Pinocchio frame parentJoint is {parent_joint}"
        )


def _missing_joint_message(model: pinocchio.Model, joint_name: str) -> str:
    available = [str(name) for name in getattr(model, "names", [])]
    return f"Joint '{joint_name}' not found in Pinocchio model. Available joints: {available}"


def _missing_frame_message(model: pinocchio.Model, frame_name: str) -> str:
    frames = getattr(model, "frames", [])
    available = [str(getattr(frame, "name", frame)) for frame in frames]
    return f"Frame '{frame_name}' not found in Pinocchio model. Available frames: {available}"


def _seed_positions_for_mapping(seed: JointState, mapping: _JointMapping) -> NDArray[np.float64]:
    if len(seed.name) == len(seed.position) and seed.name:
        positions_by_name = dict(zip(seed.name, seed.position, strict=True))
        values: list[float] = []
        for dimos_name, model_name in zip(
            mapping.dimos_joint_names, mapping.model_joint_names, strict=True
        ):
            if dimos_name in positions_by_name:
                values.append(float(positions_by_name[dimos_name]))
            elif model_name in positions_by_name:
                values.append(float(positions_by_name[model_name]))
            else:
                raise ValueError(f"Seed is missing joint '{dimos_name}' (URDF name '{model_name}')")
        return np.array(values, dtype=np.float64)

    if len(seed.position) != len(mapping.dimos_joint_names):
        raise ValueError(
            f"Seed has {len(seed.position)} positions for {len(mapping.dimos_joint_names)} joints"
        )
    return np.array(seed.position, dtype=np.float64)


def _matrix_to_se3(matrix: NDArray[np.float64]) -> pinocchio.SE3:
    return pinocchio.SE3(matrix[:3, :3], matrix[:3, 3])


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
