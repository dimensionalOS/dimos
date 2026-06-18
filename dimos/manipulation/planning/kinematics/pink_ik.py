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
from dataclasses import dataclass
import importlib
from pathlib import Path
from types import ModuleType
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.planning_group_utils import (
    filter_joint_state_to_selected_joints,
    matching_resolved_joint_name,
    planning_group_id_from_selector,
)
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import IKStatus
from dimos.manipulation.planning.spec.models import (
    IKResult,
    PlanningGroupDescriptor,
    PlanningGroupID,
    WorldRobotID,
)
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
        self._robot_contexts: dict[str, _PinkRobotContext] = {}

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
        """Solve IK with Pink, returning the standard planning ``IKResult``."""
        if not world.is_finalized:
            return _failure(IKStatus.NO_SOLUTION, "World must be finalized before IK")

        try:
            robot_context = self._get_robot_context(world, robot_id)
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

    def solve_pose_targets(
        self,
        world: WorldSpec,
        pose_targets: Mapping[PlanningGroupID | PlanningGroupDescriptor, PoseStamped],
        auxiliary_groups: Sequence[PlanningGroupID | PlanningGroupDescriptor] = (),
        seed: JointState | None = None,
        position_tolerance: float = 0.001,
        orientation_tolerance: float = 0.01,
        check_collision: bool = True,
        max_attempts: int = 10,
    ) -> IKResult:
        """Solve a planning-group pose target and return only selected resolved joints."""
        if not pose_targets:
            return _failure(IKStatus.NO_SOLUTION, "At least one pose target is required")

        pose_group_ids = tuple(
            planning_group_id_from_selector(group) for group in pose_targets.keys()
        )
        auxiliary_group_ids = tuple(
            planning_group_id_from_selector(group) for group in auxiliary_groups
        )
        selected_group_ids = pose_group_ids + auxiliary_group_ids
        try:
            resolved_groups = world.resolve_planning_groups(selected_group_ids)
        except (KeyError, ValueError) as exc:
            return _failure(IKStatus.NO_SOLUTION, str(exc))

        if len(pose_group_ids) != 1:
            return _failure(
                IKStatus.NO_SOLUTION,
                "PinkIK supports exactly one pose target per request",
            )

        target_group = next(group for group in resolved_groups if group.id == pose_group_ids[0])
        if not target_group.has_pose_target or target_group.tip_link is None:
            return _failure(
                IKStatus.NO_SOLUTION,
                f"Planning group '{target_group.id}' has no pose target frame",
            )

        robot_ids = {group.robot_id for group in resolved_groups}
        if len(robot_ids) != 1:
            return _failure(IKStatus.NO_SOLUTION, "PinkIK does not support cross-robot pose IK")

        robot_id = target_group.robot_id
        if seed is None:
            with world.scratch_context() as ctx:
                seed = world.get_joint_state(ctx, robot_id)

        try:
            robot_context = self._get_robot_context(world, robot_id, target_group.tip_link)
        except (FileNotFoundError, ImportError, ValueError) as exc:
            return _failure(IKStatus.NO_SOLUTION, f"Pink IK model setup failed: {exc}")

        lower_limits, upper_limits = world.get_joint_limits(robot_id)
        target_pose = pose_targets[next(iter(pose_targets.keys()))]
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

            selected_joint_names: list[str] = []
            selected_local_names: list[str] = []
            for group in resolved_groups:
                selected_joint_names.extend(group.joint_names)
                selected_local_names.extend(group.local_joint_names)
            try:
                result.joint_state = filter_joint_state_to_selected_joints(
                    result.joint_state,
                    selected_joint_names,
                    selected_local_names,
                )
            except ValueError as exc:
                return _failure(IKStatus.NO_SOLUTION, str(exc))
            return result

        if fallback_result is not None:
            return fallback_result
        return _failure(IKStatus.NO_SOLUTION, f"Pink IK failed after {max_attempts} attempts")

    def _solve_single(
        self,
        robot_context: _PinkRobotContext,
        target_model: NDArray[np.float64],
        seed_q: NDArray[np.float64],
        lower_limits: NDArray[np.float64],
        upper_limits: NDArray[np.float64],
        position_tolerance: float,
        orientation_tolerance: float,
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

    def _get_robot_context(
        self, world: WorldSpec, robot_id: WorldRobotID, frame_name: str | None = None
    ) -> _PinkRobotContext:
        config = world.get_robot_config(robot_id)
        target_frame = frame_name or config.end_effector_link
        if target_frame is None:
            raise ValueError(f"Robot '{robot_id}' has no end-effector frame configured")
        cache_key = f"{robot_id}:{target_frame}"
        if (
            cache_key not in self._robot_contexts
            and frame_name is None
            and str(robot_id) in self._robot_contexts
        ):
            return self._robot_contexts[str(robot_id)]
        if cache_key not in self._robot_contexts:
            self._robot_contexts[cache_key] = self._build_robot_context(config, target_frame)
        return self._robot_contexts[cache_key]

    def _build_robot_context(
        self, config: RobotModelConfig, frame_name: str | None = None
    ) -> _PinkRobotContext:
        pinocchio = self._modules.pinocchio
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
        target_frame = frame_name or config.end_effector_link
        if target_frame is None:
            raise ValueError("Robot model has no end-effector frame configured")
        frame_id = _get_frame_id(model, target_frame)
        mapping = _build_joint_mapping(model, config)
        return _PinkRobotContext(
            model=model,
            data=data,
            frame_id=frame_id,
            frame_name=target_frame,
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
        model_joint_name = config.get_urdf_joint_name(dimos_name)
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
            elif (
                resolved_name := matching_resolved_joint_name(positions_by_name, dimos_name)
            ) is not None:
                values.append(float(positions_by_name[resolved_name]))
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
        joint_state=JointState(name=joint_names, position=joint_positions.tolist()),
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


__all__ = ["PinkIK", "PinkIKConfig", "PinkIKDependencyError"]
