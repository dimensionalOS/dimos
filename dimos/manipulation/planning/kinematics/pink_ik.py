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

"""Pink-based manipulation-planning inverse kinematics backend."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, replace
import importlib
from tempfile import NamedTemporaryFile
from types import ModuleType
from typing import TYPE_CHECKING, Any

import numpy as np
import pink
from pink.exceptions import NoSolutionFound
import pinocchio

from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupSelection
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.kinematics.utils import (
    seed_positions_with_world_fallback as _seed_positions_with_world_fallback,
    unique_pose_target_frame as _unique_pose_target_frame,
)
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import IKStatus
from dimos.manipulation.planning.spec.models import IKResult
from dimos.manipulation.planning.spec.protocols import WorldSpec
from dimos.manipulation.planning.utils.kinematics_utils import compute_pose_error
from dimos.manipulation.planning.utils.mesh_utils import prepare_urdf_for_drake
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import pose_to_matrix

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = setup_logger()


class PinkIKDependencyError(ImportError):
    """Raised when Pink or its QP solver dependencies are unavailable."""


PinkIKConfig = PinkKinematicsConfig


@dataclass(frozen=True)
class _PinkModules:
    pink: ModuleType
    pinocchio: ModuleType


_MANIPULATION_EXTRA_HINT = "Install manipulation dependencies with: uv sync --extra manipulation."


@dataclass(frozen=True)
class _JointMapping:
    dimos_joint_names: list[str]
    model_joint_names: list[str]
    idx_q: list[int]


@dataclass
class _PinkRobotContext:
    model: Any
    data: Any
    frame_id: int
    frame_name: str
    mapping: _JointMapping


class PinkIK:
    """Pink task/QP IK solver implementing the planning ``KinematicsSpec`` contract.

    Pink is a local differential IK library. This backend builds a Pinocchio model
    from ``RobotModelConfig``, maps DimOS joint-state ordering to Pinocchio q
    indices by joint name, then iterates ``pink.solve_ik`` until pose tolerances
    are met or the iteration budget is exhausted.
    """

    def __init__(
        self,
        config: PinkKinematicsConfig | None = None,
        **overrides: Any,
    ) -> None:
        """Create a Pink IK backend.

        Args:
            config: Optional Pink IK configuration object.
            **overrides: Per-field overrides applied to ``config`` for factory/CLI use.
        """
        config_values = (config or PinkKinematicsConfig()).model_dump()
        config_values.update(overrides)
        self.config = PinkKinematicsConfig(**config_values)
        self._modules = _load_optional_dependencies(self.config.solver)
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
        """Solve IK with Pink, returning the standard planning ``IKResult``."""
        if not world.is_finalized:
            return _failure(IKStatus.NO_SOLUTION, "World must be finalized before IK")

        target_frame_name = _unique_pose_target_frame(world)
        if target_frame_name is None:
            return _failure(
                IKStatus.NO_SOLUTION,
                "PinkIK requires exactly one pose-targetable planning group for legacy solve()",
            )

        try:
            robot_context = self._get_model_context(world, target_frame_name)
        except (FileNotFoundError, ImportError, ValueError) as exc:
            return _failure(IKStatus.NO_SOLUTION, f"Pink IK model setup failed: {exc}")

        if seed is None:
            with world.scratch_context() as ctx:
                seed = world.get_joint_state(ctx)

        lower_limits, upper_limits = world.get_joint_limits()
        target_model = self._target_in_model_frame(world.get_model_config(), target_pose)

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
            config = world.get_model_config()
            joint_names = list(config.joint_names)
            selected_indices = [joint_names.index(name) for name in selection.joint_names]
            seed_positions = _seed_positions_with_world_fallback(world, joint_names, seed)
            lower_limits, upper_limits = world.get_joint_limits()
            locked_positions = {
                index: float(seed_positions[index])
                for index in range(len(joint_names))
                if index not in set(selected_indices)
            }
            targets = [
                (
                    self._get_model_context(world, group.tip_link or ""),
                    self._target_in_model_frame(config, target),
                )
                for group, target in pose_targets.items()
            ]
        except (FileNotFoundError, ImportError, ValueError) as exc:
            return _failure(IKStatus.NO_SOLUTION, f"Pink IK model setup failed: {exc}")

        if not targets:
            positions = seed_positions
            result = _success(joint_names, positions, 0.0, 0.0, 0)
        else:
            fallback: IKResult | None = None
            result = _failure(IKStatus.NO_SOLUTION, "Pink IK did not produce a solution")
            for attempt in range(max_attempts):
                current = seed_positions.copy()
                if attempt:
                    current[selected_indices] = np.random.uniform(
                        lower_limits[selected_indices], upper_limits[selected_indices]
                    )
                try:
                    q0 = self._q_from_dimos_positions(targets[0][0], current)
                    result = (
                        self._solve_single(
                            robot_context=targets[0][0],
                            target_model=targets[0][1],
                            seed_q=q0,
                            lower_limits=lower_limits,
                            upper_limits=upper_limits,
                            position_tolerance=position_tolerance,
                            orientation_tolerance=orientation_tolerance,
                            locked_joint_positions=locked_positions,
                        )
                        if len(targets) == 1
                        else self._solve_multi(
                            targets=targets,
                            seed_q=q0,
                            lower_limits=lower_limits,
                            upper_limits=upper_limits,
                            position_tolerance=position_tolerance,
                            orientation_tolerance=orientation_tolerance,
                            locked_joint_positions=locked_positions,
                        )
                    )
                except ValueError as exc:
                    # Seed-independent, as in _solve: mapping cannot succeed on a retry.
                    return _failure(IKStatus.NO_SOLUTION, f"Pink IK mapping failed: {exc}")
                except NoSolutionFound as exc:
                    # QP infeasibility can be seed-specific, so try the next perturbed seed.
                    if fallback is None:
                        fallback = _failure(
                            IKStatus.NO_SOLUTION, f"Pink IK solver failed: {exc}"
                        )
                    continue
                except Exception as exc:
                    return _failure(IKStatus.NO_SOLUTION, f"Pink IK solver failed: {exc}")
                if result.is_success() and result.joint_state is not None:
                    break
                fallback = fallback or result
            else:
                return fallback or result

        assert result.joint_state is not None
        if check_collision and not world.check_config_collision_free(result.joint_state):
            return _collision_failure(result)
        positions_by_name = dict(
            zip(result.joint_state.name, result.joint_state.position, strict=True)
        )
        return IKResult(
            status=IKStatus.SUCCESS,
            joint_state=JointState(
                name=list(selection.joint_names),
                position=[positions_by_name[name] for name in selection.joint_names],
            ),
            position_error=result.position_error,
            orientation_error=result.orientation_error,
            iterations=result.iterations,
            message=result.message,
        )

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
        robot_context = targets[0][0]
        pink = self._modules.pink
        pinocchio = self._modules.pinocchio
        configuration = pink.Configuration(robot_context.model, robot_context.data, seed_q.copy())
        tasks: list[Any] = []
        for target_context, target_model in targets:
            frame_task = pink.tasks.FrameTask(
                target_context.frame_name,
                position_cost=self.config.position_cost,
                orientation_cost=self.config.orientation_cost,
                lm_damping=self.config.lm_damping,
                gain=self.config.gain,
            )
            frame_task.set_target(_matrix_to_se3(pinocchio, target_model))
            tasks.append(frame_task)
        if self.config.posture_cost > 0.0:
            posture_task = pink.tasks.PostureTask(cost=self.config.posture_cost)
            posture_task.set_target_from_configuration(configuration)
            tasks.append(posture_task)
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
            velocity = pink.solve_ik(
                configuration,
                tasks,
                self.config.dt,
                solver=self.config.solver,
                damping=self.config.damping,
                safety_break=self.config.safety_break,
            )
            configuration.integrate_inplace(velocity, self.config.dt)
            for local_index, value in (locked_joint_positions or {}).items():
                configuration.q[robot_context.mapping.idx_q[local_index]] = value
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
        pink = self._modules.pink
        pinocchio = self._modules.pinocchio

        configuration = pink.Configuration(robot_context.model, robot_context.data, seed_q.copy())
        target_se3 = _matrix_to_se3(pinocchio, target_model)

        frame_task = pink.tasks.FrameTask(
            robot_context.frame_name,
            position_cost=self.config.position_cost,
            orientation_cost=self.config.orientation_cost,
            lm_damping=self.config.lm_damping,
            gain=self.config.gain,
        )
        frame_task.set_target(target_se3)
        tasks: list[Any] = [frame_task]

        if self.config.posture_cost > 0.0:
            posture_task = pink.tasks.PostureTask(cost=self.config.posture_cost)
            posture_task.set_target_from_configuration(configuration)
            tasks.append(posture_task)

        final_position_error = float("inf")
        final_orientation_error = float("inf")

        for iteration in range(self.config.max_iterations):
            current_pose = self._current_frame_matrix(robot_context, configuration.q)
            final_position_error, final_orientation_error = compute_pose_error(
                current_pose, target_model
            )
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

            velocity = pink.solve_ik(
                configuration,
                tasks,
                self.config.dt,
                solver=self.config.solver,
                damping=self.config.damping,
                safety_break=self.config.safety_break,
            )
            configuration.integrate_inplace(velocity, self.config.dt)
            for local_index, value in (locked_joint_positions or {}).items():
                configuration.q[robot_context.mapping.idx_q[local_index]] = value

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

    def _get_model_context(self, world: WorldSpec, frame_name: str) -> _PinkRobotContext:
        if self._model_context is None:
            self._model_context = self._build_robot_context(world.get_model_config(), frame_name)
        if self._model_context.frame_name == frame_name:
            return self._model_context
        return replace(
            self._model_context,
            frame_id=_get_frame_id(self._model_context.model, frame_name),
            frame_name=frame_name,
        )

    def _build_robot_context(self, config: RobotModelConfig, frame_name: str) -> _PinkRobotContext:
        pinocchio = self._modules.pinocchio
        description = prepare_urdf_for_drake(
            config.model.load(),
            convert_meshes=config.auto_convert_meshes,
        )
        # Pinocchio's supported Python API loads URDFs by filename. Materialize
        # the already-expanded model only for the duration of model creation.
        with NamedTemporaryFile(mode="w", suffix=".urdf") as urdf_file:
            urdf_file.write(description.xml)
            urdf_file.flush()
            model = pinocchio.buildModelFromUrdf(urdf_file.name)

        data = model.createData()
        _assert_base_link_is_model_root(model, config.base_link)
        frame_id = _get_frame_id(model, frame_name)
        mapping = _build_joint_mapping(model, config)
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
        pinocchio = self._modules.pinocchio
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
        pinocchio = self._modules.pinocchio
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

    def _current_frame_matrix(
        self, context: _PinkRobotContext, q: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        pinocchio = self._modules.pinocchio
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


def _load_optional_dependencies(solver: str) -> _PinkModules:
    pink = _import_required_module(
        "pink",
        "Pink IK backend requires Pink. "
        f"{_MANIPULATION_EXTRA_HINT} PyPI package: pin-pink; import name: pink.",
    )
    pinocchio = _import_required_module(
        "pinocchio",
        f"Pink IK backend requires Pinocchio (import name 'pinocchio'). {_MANIPULATION_EXTRA_HINT}",
    )
    qpsolvers = _import_required_module(
        "qpsolvers",
        "Pink IK backend requires qpsolvers plus a QP backend such as proxqp. "
        f"{_MANIPULATION_EXTRA_HINT}",
    )

    available_solvers = set(getattr(qpsolvers, "available_solvers", []))
    if solver not in available_solvers:
        raise PinkIKDependencyError(
            f"Pink IK solver '{solver}' is not available from qpsolvers. "
            f"Available solvers: {sorted(available_solvers)}. "
            "Install manipulation dependencies with uv sync --extra manipulation, "
            "which includes qpsolvers[proxqp]."
        )

    return _PinkModules(pink=pink, pinocchio=pinocchio)


def _import_required_module(name: str, message: str) -> ModuleType:
    try:
        return importlib.import_module(name)
    except ImportError as exc:
        raise PinkIKDependencyError(message) from exc


def _build_joint_mapping(model: Any, config: RobotModelConfig) -> _JointMapping:
    idx_q: list[int] = []
    model_joint_names: list[str] = []

    for dimos_name in config.joint_names:
        model_joint_name = dimos_name
        joint_id = _get_joint_id(model, model_joint_name)
        joint = model.joints[joint_id]
        nq = int(getattr(joint, "nq", 1))
        if nq != 1:
            raise ValueError(
                f"PinkIK currently supports one-DoF controlled joints; "
                f"joint '{model_joint_name}' has nq={nq}"
            )
        idx_q.append(int(joint.idx_q))
        model_joint_names.append(model_joint_name)

    return _JointMapping(
        dimos_joint_names=list(config.joint_names),
        model_joint_names=model_joint_names,
        idx_q=idx_q,
    )


def _get_joint_id(model: Any, joint_name: str) -> int:
    if hasattr(model, "existJointName") and not model.existJointName(joint_name):
        raise ValueError(_missing_joint_message(model, joint_name))
    joint_id = int(model.getJointId(joint_name))
    if joint_id >= len(model.joints):
        raise ValueError(_missing_joint_message(model, joint_name))
    return joint_id


def _get_frame_id(model: Any, frame_name: str) -> int:
    if hasattr(model, "existFrame") and not model.existFrame(frame_name):
        raise ValueError(_missing_frame_message(model, frame_name))
    frame_id = int(model.getFrameId(frame_name))
    if frame_id >= len(model.frames):
        raise ValueError(_missing_frame_message(model, frame_name))
    return frame_id


def _assert_base_link_is_model_root(model: Any, base_link: str) -> None:
    """Validate that the configured base link is fixed at the Pinocchio model root."""
    frame_id = _get_frame_id(model, base_link)
    frame = model.frames[frame_id]
    parent_joint = int(getattr(frame, "parentJoint", 0))
    if parent_joint != 0:
        raise ValueError(
            f"PinkIK expects RobotModelConfig.base_link '{base_link}' to be the model root; "
            f"Pinocchio frame parentJoint is {parent_joint}"
        )


def _missing_joint_message(model: Any, joint_name: str) -> str:
    available = [str(name) for name in getattr(model, "names", [])]
    return f"Joint '{joint_name}' not found in Pinocchio model. Available joints: {available}"


def _missing_frame_message(model: Any, frame_name: str) -> str:
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


def _matrix_to_se3(pinocchio: ModuleType, matrix: NDArray[np.float64]) -> Any:
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
