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
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.manipulation.planning.groups.identifiers import make_global_joint_name
from dimos.manipulation.planning.groups.joints import matching_global_joint_name
from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupSelection
from dimos.manipulation.planning.groups.registry import PlanningGroupRegistry
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import IKStatus
from dimos.manipulation.planning.spec.models import (
    IKResult,
    RobotName,
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

try:
    import pink  # type: ignore[import-not-found, import-untyped]
    import pinocchio  # type: ignore[import-not-found]
    import qpsolvers  # type: ignore[import-not-found]
except ImportError as exc:
    pink = None  # type: ignore[assignment]
    pinocchio = None  # type: ignore[assignment]
    qpsolvers = None  # type: ignore[assignment]
    _PINK_IMPORT_ERROR: ImportError | None = exc
else:
    _PINK_IMPORT_ERROR = None

logger = setup_logger()


class PinkIKDependencyError(ImportError):
    """Raised when Pink or its QP solver dependencies are unavailable."""


PinkIKConfig = PinkKinematicsConfig


_MANIPULATION_EXTRA_HINT = "Install manipulation dependencies with: uv sync --extra manipulation."


@dataclass(frozen=True)
class _JointMapping:
    dimos_joint_names: list[str]
    model_joint_names: list[str]
    idx_q: list[int]
    idx_q_array: NDArray[np.int64]


@dataclass
class _PinkRobotModelContext:
    model: Any
    mapping: _JointMapping
    neutral_q: NDArray[np.float64]
    frame_ids: dict[str, int]


@dataclass
class _PinkRobotContext:
    model: Any
    data: Any
    frame_id: int
    frame_name: str
    mapping: _JointMapping
    neutral_q: NDArray[np.float64] | None = None


class _CurrentStateRequiredError(ValueError):
    """Raised when normalizing a seed requires the world's current state."""


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
        _check_optional_dependencies(self.config.solver)
        self._robot_model_contexts: dict[tuple[object, ...], _PinkRobotModelContext] = {}

    def solve(
        self,
        world: WorldSpec,
        robot_id: WorldRobotID,
        target_pose: PoseStamped,
        seed: JointState | None = None,
        position_tolerance: float = 0.001,
        orientation_tolerance: float = 0.01,
        max_attempts: int = 10,
    ) -> IKResult:
        """Solve IK with Pink, returning the standard planning ``IKResult``."""
        try:
            config = world.get_robot_config(robot_id)
            group = _single_pose_group_for_robot(world, config.name)
        except (KeyError, ValueError) as exc:
            return _failure(IKStatus.NO_SOLUTION, str(exc))
        return self.solve_pose_targets(
            world=world,
            pose_targets={group: target_pose},
            seed=seed,
            position_tolerance=position_tolerance,
            orientation_tolerance=orientation_tolerance,
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
        max_attempts: int = 10,
    ) -> IKResult:
        """Solve planning-group pose targets and return selected global joints."""
        if not pose_targets:
            return _failure(IKStatus.NO_SOLUTION, "At least one pose target is required")

        pose_groups = tuple(pose_targets.keys())
        try:
            selection = PlanningGroupSelection.from_groups(pose_groups + tuple(auxiliary_groups))
            robot_ids_by_name = _robot_ids_by_name(world, selection.robot_names)
        except (KeyError, ValueError) as exc:
            return _failure(IKStatus.NO_SOLUTION, str(exc))

        groups_by_robot: dict[RobotName, list[PlanningGroup]] = {}
        pose_groups_by_robot: dict[RobotName, list[PlanningGroup]] = {}
        for group in selection.groups:
            groups_by_robot.setdefault(group.robot_name, []).append(group)
        for group in pose_groups:
            if not group.has_pose_target or group.tip_link is None:
                return _failure(
                    IKStatus.NO_SOLUTION,
                    f"Planning group '{group.id}' has no pose target frame",
                )
            pose_groups_by_robot.setdefault(group.robot_name, []).append(group)

        selected_positions_by_name: dict[str, float] = {}
        max_position_error = 0.0
        max_orientation_error = 0.0
        total_iterations = 0
        for robot_name, groups in groups_by_robot.items():
            robot_id = robot_ids_by_name[robot_name]
            robot_pose_groups = pose_groups_by_robot.get(robot_name, [])
            robot_pose_targets = {group: pose_targets[group] for group in robot_pose_groups}
            config = world.get_robot_config(robot_id)
            seed_for_robot = _seed_for_robot_with_world_fallback(world, robot_id, seed)
            if robot_pose_targets:
                lower_limits, upper_limits = world.get_joint_limits(robot_id)
                result = self._solve_pose_targets_for_robot(
                    world=world,
                    robot_id=robot_id,
                    pose_targets=robot_pose_targets,
                    seed=seed_for_robot,
                    position_tolerance=position_tolerance,
                    orientation_tolerance=orientation_tolerance,
                    max_attempts=max_attempts,
                    config=config,
                    lower_limits=lower_limits,
                    upper_limits=upper_limits,
                    target_models=self._targets_in_model_frame(config, robot_pose_targets),
                )
                if not result.is_success() or result.joint_state is None:
                    return result
            else:
                result = IKResult(
                    status=IKStatus.SUCCESS,
                    joint_state=seed_for_robot,
                    message="Auxiliary group retained seed state",
                )
            joint_state = result.joint_state
            if joint_state is None:
                return _failure(
                    IKStatus.NO_SOLUTION,
                    f"Pink IK result for robot '{robot_name}' has no joint state",
                )

            max_position_error = max(max_position_error, result.position_error)
            max_orientation_error = max(max_orientation_error, result.orientation_error)
            total_iterations += result.iterations
            local_positions = dict(zip(joint_state.name, joint_state.position, strict=True))
            for group in groups:
                for global_name, local_name in zip(
                    group.joint_names, group.local_joint_names, strict=True
                ):
                    if local_name not in local_positions:
                        return _failure(
                            IKStatus.NO_SOLUTION,
                            f"Pink IK result for robot '{robot_name}' is missing joint '{local_name}'",
                        )
                    selected_positions_by_name[global_name] = float(local_positions[local_name])

        selected_positions = [selected_positions_by_name[name] for name in selection.joint_names]
        return IKResult(
            status=IKStatus.SUCCESS,
            joint_state=JointState(
                {"name": list(selection.joint_names), "position": selected_positions}
            ),
            position_error=max_position_error,
            orientation_error=max_orientation_error,
            iterations=total_iterations,
            message="Pink IK target set solution found",
        )

    def _solve_pose_targets_for_robot(
        self,
        world: WorldSpec,
        robot_id: WorldRobotID,
        pose_targets: Mapping[PlanningGroup, PoseStamped],
        seed: JointState,
        position_tolerance: float,
        orientation_tolerance: float,
        max_attempts: int,
        config: RobotModelConfig | None = None,
        lower_limits: NDArray[np.float64] | None = None,
        upper_limits: NDArray[np.float64] | None = None,
        target_models: Mapping[PlanningGroup, NDArray[np.float64]] | None = None,
    ) -> IKResult:
        """Solve one robot's one-or-more frame targets."""
        try:
            contexts = [
                self._get_robot_context(world, robot_id, group.tip_link, config)
                for group in pose_targets
                if group.tip_link is not None
            ]
        except (FileNotFoundError, ImportError, ValueError) as exc:
            return _failure(IKStatus.NO_SOLUTION, f"Pink IK model setup failed: {exc}")

        config = config or world.get_robot_config(robot_id)
        if lower_limits is None or upper_limits is None:
            lower_limits, upper_limits = world.get_joint_limits(robot_id)
        target_models_by_group = target_models or self._targets_in_model_frame(config, pose_targets)
        target_model_list = [target_models_by_group[group] for group in pose_targets]
        fallback_result: IKResult | None = None

        for attempt in range(max_attempts):
            try:
                q0 = self._initial_q(contexts[0], seed, lower_limits, upper_limits, attempt)
                if len(contexts) == 1:
                    result = self._solve_single(
                        robot_context=contexts[0],
                        target_model=target_model_list[0],
                        seed_q=q0,
                        lower_limits=lower_limits,
                        upper_limits=upper_limits,
                        position_tolerance=position_tolerance,
                        orientation_tolerance=orientation_tolerance,
                    )
                else:
                    result = self._solve_multi_frame(
                        robot_contexts=contexts,
                        target_models=target_model_list,
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
        return self._solve_frame_targets(
            robot_contexts=[robot_context],
            target_models=[target_model],
            seed_q=seed_q,
            lower_limits=lower_limits,
            upper_limits=upper_limits,
            position_tolerance=position_tolerance,
            orientation_tolerance=orientation_tolerance,
        )

    def _solve_multi_frame(
        self,
        robot_contexts: Sequence[_PinkRobotContext],
        target_models: Sequence[NDArray[np.float64]],
        seed_q: NDArray[np.float64],
        lower_limits: NDArray[np.float64],
        upper_limits: NDArray[np.float64],
        position_tolerance: float,
        orientation_tolerance: float,
    ) -> IKResult:
        """Solve multiple frame tasks for one robot model."""
        return self._solve_frame_targets(
            robot_contexts=robot_contexts,
            target_models=target_models,
            seed_q=seed_q,
            lower_limits=lower_limits,
            upper_limits=upper_limits,
            position_tolerance=position_tolerance,
            orientation_tolerance=orientation_tolerance,
        )

    def _solve_frame_targets(
        self,
        robot_contexts: Sequence[_PinkRobotContext],
        target_models: Sequence[NDArray[np.float64]],
        seed_q: NDArray[np.float64],
        lower_limits: NDArray[np.float64],
        upper_limits: NDArray[np.float64],
        position_tolerance: float,
        orientation_tolerance: float,
    ) -> IKResult:
        """Solve one robot model against one or more frame targets."""
        assert pink is not None
        assert pinocchio is not None
        primary_context = robot_contexts[0]
        configuration = pink.Configuration(
            primary_context.model, primary_context.data, seed_q.copy()
        )

        tasks: list[Any] = []
        for context, target_model in zip(robot_contexts, target_models, strict=True):
            frame_task = pink.tasks.FrameTask(
                context.frame_name,
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
            position_errors: list[float] = []
            orientation_errors: list[float] = []
            current_poses = self._current_frame_matrices(robot_contexts, configuration.q)
            for current_pose, target_model in zip(current_poses, target_models, strict=True):
                position_error, orientation_error = compute_pose_error(current_pose, target_model)
                position_errors.append(position_error)
                orientation_errors.append(orientation_error)
            final_position_error = max(position_errors)
            final_orientation_error = max(orientation_errors)
            if (
                final_position_error <= position_tolerance
                and final_orientation_error <= orientation_tolerance
            ):
                return _success(
                    primary_context.mapping.dimos_joint_names,
                    self._q_to_dimos_positions(primary_context, configuration.q),
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

            joint_positions = self._q_to_dimos_positions(primary_context, configuration.q)
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
        self,
        world: WorldSpec,
        robot_id: WorldRobotID,
        frame_name: str | None = None,
        config: RobotModelConfig | None = None,
    ) -> _PinkRobotContext:
        config = config or world.get_robot_config(robot_id)
        target_frame = frame_name or config.end_effector_link
        if target_frame is None:
            raise ValueError(f"Robot '{robot_id}' has no end-effector frame configured")
        model_context = self._get_robot_model_context(robot_id, config)
        frame_id = self._frame_id_for_model_context(model_context, target_frame)
        return _PinkRobotContext(
            model=model_context.model,
            data=model_context.model.createData(),
            frame_id=frame_id,
            frame_name=target_frame,
            mapping=model_context.mapping,
            neutral_q=model_context.neutral_q,
        )

    def _get_robot_model_context(
        self, robot_id: WorldRobotID, config: RobotModelConfig
    ) -> _PinkRobotModelContext:
        cache_key = _robot_model_cache_key(robot_id, config)
        if cache_key not in self._robot_model_contexts:
            self._robot_model_contexts[cache_key] = self._build_robot_model_context(config)
        return self._robot_model_contexts[cache_key]

    def _frame_id_for_model_context(
        self, model_context: _PinkRobotModelContext, frame_name: str
    ) -> int:
        if frame_name not in model_context.frame_ids:
            model_context.frame_ids[frame_name] = _get_frame_id(model_context.model, frame_name)
        return model_context.frame_ids[frame_name]

    def _build_robot_model_context(self, config: RobotModelConfig) -> _PinkRobotModelContext:
        assert pinocchio is not None
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
                strip_world_joint_child_link=config.base_link
                if config.strip_model_world_joint
                else None,
            )
            model = pinocchio.buildModelFromUrdf(str(prepared_path))
        model = _lock_uncontrolled_model_joints(pinocchio, model, config)
        mapping = _build_joint_mapping(model, config)
        neutral_q = np.asarray(pinocchio.neutral(model), dtype=np.float64)
        return _PinkRobotModelContext(
            model=model,
            mapping=mapping,
            neutral_q=neutral_q,
            frame_ids={},
        )

    def _initial_q(
        self,
        context: _PinkRobotContext,
        seed: JointState,
        lower_limits: NDArray[np.float64],
        upper_limits: NDArray[np.float64],
        attempt: int,
    ) -> NDArray[np.float64]:
        assert pinocchio is not None
        neutral = context.neutral_q
        if neutral is None:
            neutral = np.asarray(pinocchio.neutral(context.model), dtype=np.float64)
        q = np.array(neutral, dtype=np.float64, copy=True)

        if attempt == 0:
            positions = _seed_positions_for_mapping(seed, context.mapping)
        else:
            positions = np.random.uniform(lower_limits, upper_limits)

        q[context.mapping.idx_q_array] = positions
        return q

    def _q_to_dimos_positions(
        self, context: _PinkRobotContext, q: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        return np.asarray(q[context.mapping.idx_q_array], dtype=np.float64)

    def _current_frame_matrices(
        self, contexts: Sequence[_PinkRobotContext], q: NDArray[np.float64]
    ) -> list[NDArray[np.float64]]:
        assert pinocchio is not None
        primary_context = contexts[0]
        pinocchio.forwardKinematics(primary_context.model, primary_context.data, q)
        pinocchio.updateFramePlacements(primary_context.model, primary_context.data)
        return [
            _placement_to_matrix(primary_context.data.oMf[context.frame_id]) for context in contexts
        ]

    def _target_in_model_frame(
        self, config: RobotModelConfig, target_pose: PoseStamped
    ) -> NDArray[np.float64]:
        base_world_inverse = np.linalg.inv(pose_to_matrix(config.base_pose))
        return self._target_in_model_frame_with_base_inverse(target_pose, base_world_inverse)

    def _targets_in_model_frame(
        self,
        config: RobotModelConfig,
        pose_targets: Mapping[PlanningGroup, PoseStamped],
    ) -> dict[PlanningGroup, NDArray[np.float64]]:
        base_world_inverse = np.linalg.inv(pose_to_matrix(config.base_pose))
        return {
            group: self._target_in_model_frame_with_base_inverse(pose, base_world_inverse)
            for group, pose in pose_targets.items()
        }

    def _target_in_model_frame_with_base_inverse(
        self, target_pose: PoseStamped, base_world_inverse: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        target_world = pose_to_matrix(target_pose)
        target_model: NDArray[np.float64] = np.asarray(
            base_world_inverse @ target_world, dtype=np.float64
        )
        return target_model


def _check_optional_dependencies(solver: str) -> None:
    if _PINK_IMPORT_ERROR is not None or pink is None or pinocchio is None or qpsolvers is None:
        raise PinkIKDependencyError(
            "Pink IK backend requires Pink, Pinocchio, and qpsolvers plus a QP backend "
            f"such as proxqp. {_MANIPULATION_EXTRA_HINT} PyPI package: pin-pink; "
            "import names: pink, pinocchio, qpsolvers."
        ) from _PINK_IMPORT_ERROR
    available_solvers = set(getattr(qpsolvers, "available_solvers", []))
    if solver not in available_solvers:
        raise PinkIKDependencyError(
            f"Pink IK solver '{solver}' is not available from qpsolvers. "
            f"Available solvers: {sorted(available_solvers)}. "
            "Install manipulation dependencies with uv sync --extra manipulation, "
            "which includes qpsolvers[proxqp]."
        )


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
        idx_q_array=np.asarray(idx_q, dtype=np.int64),
    )


def _robot_model_cache_key(robot_id: WorldRobotID, config: RobotModelConfig) -> tuple[object, ...]:
    return (
        str(robot_id),
        str(Path(config.model_path).resolve()),
        tuple(config.joint_names),
        config.base_link,
        tuple(sorted((name, str(path.resolve())) for name, path in config.package_paths.items())),
        tuple(sorted(config.xacro_args.items())),
        config.auto_convert_meshes,
        config.strip_model_world_joint,
    )


def _placement_to_matrix(placement: Any) -> NDArray[np.float64]:
    matrix: NDArray[np.float64] = np.eye(4)
    matrix[:3, :3] = np.asarray(placement.rotation, dtype=np.float64)
    matrix[:3, 3] = np.asarray(placement.translation, dtype=np.float64)
    return matrix


def _lock_uncontrolled_model_joints(pinocchio: Any, model: Any, config: RobotModelConfig) -> Any:
    """Return a Pinocchio model reduced to the joints controlled by config."""
    controlled_joint_names = set(config.joint_names)
    lock_joint_ids: list[int] = []
    for joint_id, model_joint_name in enumerate(model.names):
        if joint_id == 0 or model_joint_name in controlled_joint_names:
            continue
        joint = model.joints[joint_id]
        if int(getattr(joint, "nq", 1)) > 0:
            lock_joint_ids.append(joint_id)

    if not lock_joint_ids:
        return model

    logger.debug(
        "Reducing Pink IK model '%s' by locking uncontrolled joints: %s",
        config.name,
        [model.names[joint_id] for joint_id in lock_joint_ids],
    )
    return pinocchio.buildReducedModel(model, lock_joint_ids, pinocchio.neutral(model))


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
                global_name := matching_global_joint_name(positions_by_name, dimos_name)
            ) is not None:
                values.append(float(positions_by_name[global_name]))
            else:
                raise ValueError(f"Seed is missing joint '{dimos_name}' (URDF name '{model_name}')")
        return np.array(values, dtype=np.float64)

    if len(seed.position) != len(mapping.dimos_joint_names):
        raise ValueError(
            f"Seed has {len(seed.position)} positions for {len(mapping.dimos_joint_names)} joints"
        )
    return np.array(seed.position, dtype=np.float64)


def _matrix_to_se3(pinocchio: Any, matrix: NDArray[np.float64]) -> Any:
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


def _seed_for_robot_config(
    config: RobotModelConfig,
    seed: JointState | None,
    current_state: JointState | None = None,
) -> JointState:
    """Return a full local seed state for one robot from local/global seed input."""
    if seed is None:
        if current_state is None:
            raise _CurrentStateRequiredError("Current joint state is required when seed is absent")
        return JointState(current_state)
    if not seed.name:
        if len(seed.position) == len(config.joint_names):
            return JointState({"name": list(config.joint_names), "position": list(seed.position)})
        raise ValueError(
            f"Seed has {len(seed.position)} positions for robot '{config.name}', "
            f"expected {len(config.joint_names)}"
        )
    if len(seed.name) != len(seed.position):
        raise ValueError(f"Seed has {len(seed.name)} names but {len(seed.position)} positions")
    seed_by_name = dict(zip(seed.name, seed.position, strict=True))
    positions: list[float] = []
    missing_local_names: list[str] = []
    for local_name in config.joint_names:
        global_name = make_global_joint_name(config.name, local_name)
        if local_name in seed_by_name:
            positions.append(float(seed_by_name[local_name]))
        elif global_name in seed_by_name:
            positions.append(float(seed_by_name[global_name]))
        else:
            positions.append(0.0)
            missing_local_names.append(local_name)
    if missing_local_names:
        if current_state is None:
            missing = ", ".join(repr(name) for name in missing_local_names)
            raise _CurrentStateRequiredError(
                f"Current joint state is required for missing joints: {missing}"
            )
        current = current_state
        current_by_name = dict(zip(current.name, current.position, strict=True))
        for index, local_name in enumerate(config.joint_names):
            if local_name not in missing_local_names:
                continue
            if local_name not in current_by_name:
                raise ValueError(f"Seed/current state is missing joint '{local_name}'")
            positions[index] = float(current_by_name[local_name])
    return JointState({"name": list(config.joint_names), "position": positions})


def _seed_for_robot_with_world_fallback(
    world: WorldSpec, robot_id: WorldRobotID, seed: JointState | None
) -> JointState:
    """Normalize a robot seed, reading world state only when the seed is incomplete."""
    config = world.get_robot_config(robot_id)
    try:
        return _seed_for_robot_config(config, seed)
    except _CurrentStateRequiredError:
        with world.scratch_context() as ctx:
            current = world.get_joint_state(ctx, robot_id)
        return _seed_for_robot_config(config, seed, current)


def _robot_ids_by_name(
    world: WorldSpec, robot_names: tuple[RobotName, ...]
) -> dict[RobotName, WorldRobotID]:
    robot_ids_by_name: dict[RobotName, WorldRobotID] = {}
    for robot_name in robot_names:
        matches = [
            robot_id
            for robot_id in world.get_robot_ids()
            if world.get_robot_config(robot_id).name == robot_name
        ]
        if not matches:
            raise KeyError(f"Robot '{robot_name}' not found")
        if len(matches) > 1:
            raise ValueError(f"Robot name '{robot_name}' is not unique in planning world")
        robot_ids_by_name[robot_name] = matches[0]
    return robot_ids_by_name


def _single_pose_group_for_robot(world: WorldSpec, robot_name: RobotName) -> PlanningGroup:
    configs = [world.get_robot_config(robot_id) for robot_id in world.get_robot_ids()]
    registry = PlanningGroupRegistry(configs)
    pose_groups = [
        group for group in registry.groups_for_robot(robot_name) if group.has_pose_target
    ]
    if len(pose_groups) != 1:
        raise ValueError(
            f"Robot '{robot_name}' has {len(pose_groups)} pose-targetable planning groups"
        )
    return pose_groups[0]
