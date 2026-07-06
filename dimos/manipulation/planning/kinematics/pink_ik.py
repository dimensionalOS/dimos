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

from dimos.manipulation.planning.groups.models import PlanningGroup, PlanningGroupSelection
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import IKStatus
from dimos.manipulation.planning.spec.models import IKResult, RobotName, WorldRobotID
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
        """Solve a planning-group-scoped pose target with Pink IK."""
        if not world.is_finalized:
            return _failure(IKStatus.NO_SOLUTION, "World must be finalized before IK")
        if not pose_targets:
            return _failure(IKStatus.NO_SOLUTION, "At least one pose target is required")
        if len(pose_targets) != 1 or auxiliary_groups:
            return _failure(
                IKStatus.NO_SOLUTION,
                "PinkIK supports exactly one pose target and no auxiliary planning groups",
            )

        target_group = next(iter(pose_targets.keys()))
        if not target_group.has_pose_target or target_group.tip_link is None:
            return _failure(
                IKStatus.NO_SOLUTION,
                f"Planning group '{target_group.id}' has no pose target frame",
            )

        try:
            selection = PlanningGroupSelection.from_groups((target_group,))
            robot_ids_by_name = _robot_ids_by_name(world, selection.robot_names)
            robot_id = robot_ids_by_name[target_group.robot_name]
            config = world.get_robot_config(robot_id)
            joint_names = list(config.joint_names)
            group_indices = [joint_names.index(name) for name in target_group.local_joint_names]
            seed_positions = _seed_positions_with_world_fallback(
                world, robot_id, config.name, joint_names, seed
            )
            robot_context = self._get_robot_context(world, robot_id, target_group.tip_link)
        except (FileNotFoundError, ImportError, ValueError) as exc:
            return _failure(IKStatus.NO_SOLUTION, f"Pink IK model setup failed: {exc}")

        lower_limits, upper_limits = world.get_joint_limits(robot_id)
        target_model = self._target_in_model_frame(config, pose_targets[target_group])
        locked_positions = {
            index: float(seed_positions[index])
            for index in range(len(joint_names))
            if index not in set(group_indices)
        }
        fallback_result: IKResult | None = None

        for attempt in range(max_attempts):
            current_positions = seed_positions.copy()
            if attempt > 0:
                current_positions[group_indices] = np.random.uniform(
                    lower_limits[group_indices], upper_limits[group_indices]
                )
            try:
                q0 = self._q_from_dimos_positions(robot_context, current_positions)
                result = self._solve_single(
                    robot_context=robot_context,
                    target_model=target_model,
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
            if check_collision and not world.check_config_collision_free(
                robot_id, result.joint_state
            ):
                fallback_result = _collision_failure(result)
                continue
            return _filter_result_to_group(result, target_group)

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

    def _build_robot_context(self, config: RobotModelConfig, frame_name: str) -> _PinkRobotContext:
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


def _unique_pose_target_frame_for_robot(
    world: WorldSpec,
    robot_id: WorldRobotID,
) -> str | None:
    config = world.get_robot_config(robot_id)
    pose_target_frames = [
        group.tip_link for group in config.planning_groups if group.tip_link is not None
    ]
    unique_frames = list(dict.fromkeys(pose_target_frames))
    if len(unique_frames) != 1:
        return None
    return unique_frames[0]


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
    """Return full robot positions, reading world only for absent seed joints."""
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
        local_indices = {name: index for index, name in enumerate(local_joint_names)}
        for local_name, position in seed_positions.items():
            fallback_positions[local_indices[local_name]] = position
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
    if len(joint_state.name) != len(joint_state.position):
        raise ValueError(
            f"Seed has {len(joint_state.name)} names but {len(joint_state.position)} positions"
        )
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


def _filter_result_to_group(result: IKResult, group: PlanningGroup) -> IKResult:
    if result.joint_state is None:
        return result
    positions_by_name = dict(zip(result.joint_state.name, result.joint_state.position, strict=True))
    positions: list[float] = []
    for global_name, local_name in zip(group.joint_names, group.local_joint_names, strict=True):
        if global_name in positions_by_name:
            positions.append(float(positions_by_name[global_name]))
        elif local_name in positions_by_name:
            positions.append(float(positions_by_name[local_name]))
        else:
            raise ValueError(f"IK result is missing selected joint '{global_name}'")
    return IKResult(
        status=result.status,
        joint_state=JointState({"name": list(group.joint_names), "position": positions}),
        position_error=result.position_error,
        orientation_error=result.orientation_error,
        iterations=result.iterations,
        message=result.message,
    )


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
