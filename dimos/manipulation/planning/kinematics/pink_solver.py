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
from types import MappingProxyType
from typing import TYPE_CHECKING

import numpy as np

try:
    import pink
    import pinocchio
    import qpsolvers
except ImportError as exc:
    msg = (
        "Pink IK dependencies not found; install them with: uv sync --extra manipulation --inexact."
    )
    raise ImportError(msg) from exc

from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.utils.mesh_utils import prepare_urdf_for_drake
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import pose_to_matrix

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = setup_logger()


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
                "Install manipulation dependencies with: "
                "uv sync --extra manipulation --inexact."
            )

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

    def _build_robot_context(
        self,
        config: RobotModelConfig,
        frame_name: str,
        controlled_joints: Sequence[str] | None = None,
    ) -> _PinkRobotContext:
        description = prepare_urdf_for_drake(config.model.load(), convert_meshes=False)
        model = pinocchio.buildModelFromXML(description.xml)

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
        model_joint_name = dimos_name
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
