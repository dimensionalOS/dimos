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

"""Mink-based manipulation-planning inverse kinematics backend."""

from __future__ import annotations

from dataclasses import dataclass
import importlib
from pathlib import Path
from types import ModuleType
from typing import TYPE_CHECKING, Any
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.manipulation.planning.kinematics.config import MinkKinematicsConfig
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.spec.enums import IKStatus
from dimos.manipulation.planning.spec.models import IKResult, WorldRobotID
from dimos.manipulation.planning.spec.protocols import IKStepCallback, WorldSpec
from dimos.manipulation.planning.utils.kinematics_utils import compute_pose_error
from dimos.manipulation.planning.world.mujoco_world import (
    compile_mujoco_model_from_config,
    prepare_urdf_for_mujoco,
)
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import pose_to_matrix

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = setup_logger()


class MinkIKDependencyError(ImportError):
    """Raised when Mink or its solver dependencies are unavailable."""


@dataclass(frozen=True)
class _MinkModules:
    mink: ModuleType
    mujoco: ModuleType


@dataclass(frozen=True)
class _JointMapping:
    dimos_joint_names: list[str]
    model_joint_names: list[str]
    qpos_adr: NDArray[np.intp]
    dof_adr: NDArray[np.intp]


@dataclass
class _MinkRobotContext:
    config: RobotModelConfig
    model: Any
    data: Any
    ee_body_name: str
    ee_body_id: int
    body_to_ee: NDArray[np.float64]
    q_base: NDArray[np.float64]
    mapping: _JointMapping
    arm_velocity_mask: NDArray[np.float64]
    frozen_dofs: list[int]
    # Warm-start seed: the last collision-free solution's full qpos. Seeding
    # the next solve from here keeps interactive dragging continuous (the
    # elbow configuration evolves instead of re-deriving from scratch). Only
    # collision-free results may be stored -- warming from a colliding or
    # stuck configuration poisons every following solve.
    q_warm: NDArray[np.float64] | None = None


_MANIPULATION_EXTRA_HINT = "Install manipulation dependencies with: uv sync --extra manipulation."

# on_step cadence during descent: frequent enough that the animated search
# reads as continuous, rare enough that the callback doesn't dominate solve
# time (tuned in the original reachability demo viewer).
_ON_STEP_STRIDE = 12


class MinkIK:
    """Mink task/QP IK solver implementing the planning ``KinematicsSpec`` contract.

    Mink operates on a MuJoCo model, but this class keeps the public solver
    contract backend-oriented: robot setup comes from ``RobotModelConfig`` and
    final collision validation is delegated to the provided ``WorldSpec``.
    """

    def __init__(self, config: MinkKinematicsConfig | None = None, **overrides: Any) -> None:
        """Create a Mink IK backend."""
        config_values = (config or MinkKinematicsConfig()).model_dump()
        config_values.update(overrides)
        self.config = MinkKinematicsConfig(**config_values)
        self._modules = _load_optional_dependencies(self.config.solver)
        self._robot_contexts: dict[str, _MinkRobotContext] = {}
        # Persistent across solves: a failed solve retried with the same RNG
        # state would re-attempt the exact same restart configurations and
        # fail identically forever.
        self._rng = np.random.default_rng(0)

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
        on_step: IKStepCallback | None = None,
    ) -> IKResult:
        """Solve IK with Mink, returning the standard planning ``IKResult``.

        Attempt 0 is warm-started from the previous collision-free solution
        so interactive dragging feels continuous; the caller's seed, the home
        pose, and random restarts follow as recovery basins. Collision-free
        solutions are preferred over closer ones that penetrate; a converged
        but colliding best candidate is returned with COLLISION status and
        its joint state intact so callers can visualize it.

        ``on_step(joint_state, position_error, attempt)`` is called every few
        descent iterations with the solver's current (possibly wrong) guess so
        a caller can animate the search. A truthy return aborts the search
        (the target is stale) and the best result so far is returned.
        """
        if not world.is_finalized:
            return _failure(IKStatus.NO_SOLUTION, "World must be finalized before IK")

        try:
            context = self._get_robot_context(world, robot_id)
        except (FileNotFoundError, ImportError, ValueError) as exc:
            return _failure(IKStatus.NO_SOLUTION, f"Mink IK model setup failed: {exc}")

        if seed is None:
            with world.scratch_context() as ctx:
                seed = world.get_joint_state(ctx, robot_id)

        lower_limits, upper_limits = world.get_joint_limits(robot_id)
        target_matrix = pose_to_matrix(target_pose)
        target_body_matrix = target_matrix @ np.linalg.inv(context.body_to_ee)
        best_result: IKResult | None = None
        # (collision-free success, -total error): free solutions beat closer
        # ones that penetrate.
        best_key = (-1, -float("inf"))

        for attempt in range(max_attempts):
            try:
                q0 = self._initial_q(context, seed, lower_limits, upper_limits, attempt)
                result, aborted = self._solve_single(
                    context=context,
                    target_matrix=target_body_matrix,
                    target_ee_matrix=target_matrix,
                    seed_q=q0,
                    lower_limits=lower_limits,
                    upper_limits=upper_limits,
                    position_tolerance=position_tolerance,
                    orientation_tolerance=orientation_tolerance,
                    attempt=attempt,
                    on_step=on_step,
                )
            except ValueError as exc:
                return _failure(IKStatus.NO_SOLUTION, f"Mink IK mapping failed: {exc}")
            except Exception as exc:
                return _failure(IKStatus.NO_SOLUTION, f"Mink IK solver failed: {exc}")

            converged = result.is_success() and result.joint_state is not None
            collision_free = converged
            if converged and check_collision:
                assert result.joint_state is not None
                collision_free = world.check_config_collision_free(robot_id, result.joint_state)
                if not collision_free:
                    result = _collision_result(result)

            key = (
                1 if converged and collision_free else 0,
                -(result.position_error + result.orientation_error),
            )
            if key > best_key:
                best_key = key
                best_result = result

            if converged and collision_free:
                assert result.joint_state is not None
                context.q_warm = self._full_q_for_positions(
                    context, np.asarray(result.joint_state.position, dtype=np.float64)
                )
                return result

            if aborted:
                break

        if best_result is not None:
            return best_result

        return _failure(IKStatus.NO_SOLUTION, f"Mink IK failed after {max_attempts} attempts")

    def _solve_single(
        self,
        context: _MinkRobotContext,
        target_matrix: NDArray[np.float64],
        target_ee_matrix: NDArray[np.float64],
        seed_q: NDArray[np.float64],
        lower_limits: NDArray[np.float64],
        upper_limits: NDArray[np.float64],
        position_tolerance: float,
        orientation_tolerance: float,
        attempt: int = 0,
        on_step: IKStepCallback | None = None,
    ) -> tuple[IKResult, bool]:
        """One descent from ``seed_q``. Returns (result, aborted-by-on_step)."""
        mink = self._modules.mink
        configuration = mink.Configuration(context.model, seed_q.copy())
        frame_task = mink.FrameTask(
            frame_name=context.ee_body_name,
            frame_type="body",
            position_cost=self.config.position_cost,
            orientation_cost=self.config.orientation_cost,
            gain=self.config.gain,
            lm_damping=self.config.lm_damping,
        )
        frame_task.set_target(_matrix_to_se3(mink, self._modules.mujoco, target_matrix))
        tasks: list[Any] = [frame_task]
        if context.frozen_dofs:
            tasks.append(mink.DofFreezingTask(context.model, context.frozen_dofs))
        limits = [mink.ConfigurationLimit(context.model)]

        final_position_error = float("inf")
        final_orientation_error = float("inf")
        aborted = False

        for iteration in range(self.config.max_iterations):
            current_pose = self._current_ee_matrix(context, configuration.q)
            final_position_error, final_orientation_error = compute_pose_error(
                current_pose, target_ee_matrix
            )
            if (
                final_position_error <= position_tolerance
                and final_orientation_error <= orientation_tolerance
            ):
                joint_positions = self._q_to_dimos_positions(context, configuration.q)
                if not _within_limits(joint_positions, lower_limits, upper_limits):
                    return (
                        _joint_limit_failure(
                            final_position_error, final_orientation_error, iteration + 1
                        ),
                        aborted,
                    )
                return (
                    _success(
                        context.mapping.dimos_joint_names,
                        joint_positions,
                        final_position_error,
                        final_orientation_error,
                        iteration + 1,
                    ),
                    aborted,
                )

            if on_step is not None and iteration % _ON_STEP_STRIDE == 0:
                guess = JointState(
                    name=context.mapping.dimos_joint_names,
                    position=self._q_to_dimos_positions(context, configuration.q).tolist(),
                )
                if on_step(guess, final_position_error, attempt):
                    aborted = True
                    break

            velocity = (
                mink.solve_ik(
                    configuration,
                    tasks,
                    self.config.dt,
                    self.config.solver,
                    damping=self.config.damping,
                    safety_break=self.config.safety_break,
                    limits=limits,
                )
                * context.arm_velocity_mask
            )
            configuration.integrate_inplace(velocity, self.config.dt)
            if float(np.linalg.norm(velocity)) < self.config.velocity_tolerance:
                break

        joint_positions = self._q_to_dimos_positions(context, configuration.q)
        if not _within_limits(joint_positions, lower_limits, upper_limits):
            return (
                _joint_limit_failure(
                    final_position_error, final_orientation_error, self.config.max_iterations
                ),
                aborted,
            )
        current_pose = self._current_ee_matrix(context, configuration.q)
        final_position_error, final_orientation_error = compute_pose_error(
            current_pose, target_ee_matrix
        )
        if (
            final_position_error <= position_tolerance
            and final_orientation_error <= orientation_tolerance
        ):
            return (
                _success(
                    context.mapping.dimos_joint_names,
                    joint_positions,
                    final_position_error,
                    final_orientation_error,
                    self.config.max_iterations,
                ),
                aborted,
            )
        return (
            IKResult(
                status=IKStatus.NO_SOLUTION,
                joint_state=JointState(
                    name=context.mapping.dimos_joint_names,
                    position=joint_positions.tolist(),
                ),
                position_error=final_position_error,
                orientation_error=final_orientation_error,
                iterations=self.config.max_iterations,
                message="Mink IK did not converge within the iteration budget",
            ),
            aborted,
        )

    def _get_robot_context(self, world: WorldSpec, robot_id: WorldRobotID) -> _MinkRobotContext:
        cache_key = str(robot_id)
        if cache_key not in self._robot_contexts:
            self._robot_contexts[cache_key] = self._build_robot_context(
                world.get_robot_config(robot_id)
            )
        return self._robot_contexts[cache_key]

    def _build_robot_context(self, config: RobotModelConfig) -> _MinkRobotContext:
        mujoco = self._modules.mujoco
        model = compile_mujoco_model_from_config(config)
        data = mujoco.MjData(model)
        mapping = _build_joint_mapping(mujoco, model, config)
        ee_body_name, ee_body_id, body_to_ee = _resolve_end_effector_frame(mujoco, model, config)
        claimed_dofs = {int(idx) for idx in mapping.dof_adr}
        arm_velocity_mask = np.zeros(model.nv, dtype=np.float64)
        arm_velocity_mask[list(claimed_dofs)] = 1.0
        frozen_dofs = [dof for dof in range(model.nv) if dof not in claimed_dofs]
        return _MinkRobotContext(
            config=config,
            model=model,
            data=data,
            ee_body_name=ee_body_name,
            ee_body_id=ee_body_id,
            body_to_ee=body_to_ee,
            q_base=_base_qpos(mujoco, model, config),
            mapping=mapping,
            arm_velocity_mask=arm_velocity_mask,
            frozen_dofs=frozen_dofs,
        )

    def _initial_q(
        self,
        context: _MinkRobotContext,
        seed: JointState,
        lower_limits: NDArray[np.float64],
        upper_limits: NDArray[np.float64],
        attempt: int,
    ) -> NDArray[np.float64]:
        """Attempt schedule: warm start, caller's seed, home pose, random.

        Without a warm seed the schedule shifts up (seed, home, random...).
        The home pose is a deterministic recovery basin in case the warm
        start is stuck somewhere bad.
        """
        if context.q_warm is None:
            attempt += 1
        if attempt == 0:
            return context.q_warm.copy()  # type: ignore[union-attr]
        q = context.q_base.copy()
        if attempt == 1:
            positions = _seed_positions_for_mapping(seed, context.mapping)
        elif attempt == 2:
            positions = context.q_base[context.mapping.qpos_adr]
        else:
            positions = self._rng.uniform(lower_limits, upper_limits)
        q[context.mapping.qpos_adr] = positions
        return q

    def _full_q_for_positions(
        self, context: _MinkRobotContext, positions: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        q = context.q_base.copy()
        q[context.mapping.qpos_adr] = positions
        return q

    def _q_to_dimos_positions(
        self, context: _MinkRobotContext, q: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        return np.array([q[adr] for adr in context.mapping.qpos_adr], dtype=np.float64)

    def _current_ee_matrix(
        self, context: _MinkRobotContext, q: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        mujoco = self._modules.mujoco
        context.data.qpos[:] = q
        mujoco.mj_forward(context.model, context.data)
        matrix = np.eye(4, dtype=np.float64)
        matrix[:3, :3] = context.data.xmat[context.ee_body_id].reshape(3, 3)
        matrix[:3, 3] = context.data.xpos[context.ee_body_id]
        return matrix @ context.body_to_ee


def _load_optional_dependencies(solver: str) -> _MinkModules:
    mink = _import_required_module(
        "mink",
        "Mink IK backend requires Mink. "
        f"{_MANIPULATION_EXTRA_HINT} PyPI package and import name: mink.",
    )
    mujoco = _import_required_module(
        "mujoco",
        f"Mink IK backend requires MuJoCo. {_MANIPULATION_EXTRA_HINT}",
    )
    qpsolvers = _import_required_module(
        "qpsolvers",
        "Mink IK backend requires qpsolvers plus a QP backend such as daqp. "
        f"{_MANIPULATION_EXTRA_HINT}",
    )
    available_solvers = set(getattr(qpsolvers, "available_solvers", []))
    if solver not in available_solvers:
        raise MinkIKDependencyError(
            f"Mink IK solver '{solver}' is not available from qpsolvers. "
            f"Available solvers: {sorted(available_solvers)}. "
            "Install manipulation dependencies with uv sync --extra manipulation, "
            "which includes qpsolvers[daqp]."
        )
    return _MinkModules(mink=mink, mujoco=mujoco)


def _import_required_module(name: str, message: str) -> ModuleType:
    try:
        return importlib.import_module(name)
    except ImportError as exc:
        raise MinkIKDependencyError(message) from exc


def _build_joint_mapping(mujoco: ModuleType, model: Any, config: RobotModelConfig) -> _JointMapping:
    qpos_adr: list[int] = []
    dof_adr: list[int] = []
    model_joint_names: list[str] = []
    for dimos_name in config.joint_names:
        model_name = config.get_urdf_joint_name(dimos_name)
        joint_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, model_name))
        if joint_id < 0:
            raise ValueError(f"Joint '{model_name}' not found in MuJoCo model")
        if int(model.jnt_type[joint_id]) == int(mujoco.mjtJoint.mjJNT_FREE):
            raise ValueError(f"Controlled joint '{model_name}' is a free joint")
        qpos_adr.append(int(model.jnt_qposadr[joint_id]))
        dof_adr.append(int(model.jnt_dofadr[joint_id]))
        model_joint_names.append(model_name)
    return _JointMapping(
        dimos_joint_names=list(config.joint_names),
        model_joint_names=model_joint_names,
        qpos_adr=np.asarray(qpos_adr, dtype=np.intp),
        dof_adr=np.asarray(dof_adr, dtype=np.intp),
    )


def _resolve_end_effector_frame(
    mujoco: ModuleType, model: Any, config: RobotModelConfig
) -> tuple[str, int, NDArray[np.float64]]:
    ee_body_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, config.end_effector_link))
    if ee_body_id >= 0:
        return config.end_effector_link, ee_body_id, np.eye(4, dtype=np.float64)

    if Path(str(config.model_path)).resolve().suffix == ".xml":
        raise ValueError(f"End-effector body '{config.end_effector_link}' not in MuJoCo model")

    fixed_joints = _fixed_joint_map(prepare_urdf_for_mujoco(config))
    link_name = config.end_effector_link
    body_to_ee = np.eye(4, dtype=np.float64)
    visited: set[str] = set()

    while link_name not in visited and link_name in fixed_joints:
        visited.add(link_name)
        parent, parent_to_child = fixed_joints[link_name]
        body_to_ee = parent_to_child @ body_to_ee
        body_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, parent))
        if body_id >= 0:
            return parent, body_id, body_to_ee
        link_name = parent

    raise ValueError(
        f"End-effector frame '{config.end_effector_link}' is not a MuJoCo body "
        "and could not be resolved through fixed URDF joints"
    )


def _fixed_joint_map(urdf_path: Path) -> dict[str, tuple[str, NDArray[np.float64]]]:
    root = ET.parse(urdf_path).getroot()
    fixed: dict[str, tuple[str, NDArray[np.float64]]] = {}
    for joint in root.findall("joint"):
        if joint.get("type") != "fixed":
            continue
        parent_node = joint.find("parent")
        child_node = joint.find("child")
        if parent_node is None or child_node is None:
            continue
        parent = parent_node.get("link")
        child = child_node.get("link")
        if parent is None or child is None:
            continue
        fixed[child] = (parent, _origin_matrix(joint.find("origin")))
    return fixed


def _origin_matrix(origin: ET.Element | None) -> NDArray[np.float64]:
    matrix = np.eye(4, dtype=np.float64)
    if origin is None:
        return matrix
    xyz = _parse_vector(origin.get("xyz"), 3)
    rpy = _parse_vector(origin.get("rpy"), 3)
    matrix[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    matrix[:3, 3] = xyz
    return matrix


def _parse_vector(value: str | None, size: int) -> NDArray[np.float64]:
    if value is None:
        return np.zeros(size, dtype=np.float64)
    parts = [float(part) for part in value.split()]
    if len(parts) != size:
        raise ValueError(f"Expected {size} values, got {len(parts)} in '{value}'")
    return np.asarray(parts, dtype=np.float64)


def _base_qpos(mujoco: ModuleType, model: Any, config: RobotModelConfig) -> NDArray[np.float64]:
    q = model.qpos0.copy()
    for joint_id in range(model.njnt):
        if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_FREE:
            continue
        adr = int(model.jnt_qposadr[joint_id])
        q[adr : adr + 3] = (
            config.base_pose.position.x,
            config.base_pose.position.y,
            config.base_pose.position.z,
        )
        q[adr + 3 : adr + 7] = (
            config.base_pose.orientation.w,
            config.base_pose.orientation.x,
            config.base_pose.orientation.y,
            config.base_pose.orientation.z,
        )
    return q


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
                raise ValueError(
                    f"Seed is missing joint '{dimos_name}' (model name '{model_name}')"
                )
        return np.asarray(values, dtype=np.float64)

    if len(seed.position) != len(mapping.dimos_joint_names):
        raise ValueError(
            f"Seed has {len(seed.position)} positions for {len(mapping.dimos_joint_names)} joints"
        )
    return np.asarray(seed.position, dtype=np.float64)


def _matrix_to_se3(mink: ModuleType, mujoco: ModuleType, matrix: NDArray[np.float64]) -> Any:
    quat = np.empty(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quat, np.ascontiguousarray(matrix[:3, :3]).reshape(9))
    return mink.SE3.from_rotation_and_translation(mink.SO3(quat), matrix[:3, 3])


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
        message="Mink IK solution found",
    )


def _failure(status: IKStatus, message: str, iterations: int = 0) -> IKResult:
    return IKResult(status=status, joint_state=None, iterations=iterations, message=message)


def _joint_limit_failure(
    position_error: float, orientation_error: float, iterations: int
) -> IKResult:
    return IKResult(
        status=IKStatus.JOINT_LIMITS,
        joint_state=None,
        position_error=position_error,
        orientation_error=orientation_error,
        iterations=iterations,
        message="Mink IK candidate violates DimOS joint limits",
    )


def _collision_result(result: IKResult) -> IKResult:
    """Converged-but-colliding candidate: keep the joint state for visualization."""
    return IKResult(
        status=IKStatus.COLLISION,
        joint_state=result.joint_state,
        position_error=result.position_error,
        orientation_error=result.orientation_error,
        iterations=result.iterations,
        message="Mink IK solution rejected by collision check",
    )


__all__ = ["MinkIK", "MinkIKDependencyError"]
