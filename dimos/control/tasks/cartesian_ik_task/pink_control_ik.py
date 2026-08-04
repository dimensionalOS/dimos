# Copyright 2026 Dimensional Inc.
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

"""Pink differential IK for coordinator Cartesian control."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import NDArray
import pinocchio
from pydantic import Field, FiniteFloat

_PINK_INSTALL_ERROR = "Pink control tasks require the 'pink' dependency. Install it with `uv sync`."

try:
    from pink import Configuration, solve_ik
    from pink.limits import ConfigurationLimit, VelocityLimit
    from pink.tasks import FrameTask, PostureTask
except ModuleNotFoundError as exc:
    raise ModuleNotFoundError(
        f"{_PINK_INSTALL_ERROR} Missing module: {exc.name}",
        name=exc.name,
    ) from exc

from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.utils.mesh_utils import prepare_urdf_for_drake
from dimos.protocol.service.spec import BaseConfig

# Pink's integration/QP boundary tolerance is small but larger than machine epsilon.
_POSITION_LIMIT_EPSILON_RAD = 1e-5


class PinkControlIKConfig(BaseConfig):
    """Typed configuration for the control IK backend."""

    robot_model: RobotModelConfig
    solver: str = "proxqp"
    max_velocity: FiniteFloat = Field(10.0, gt=0.0)
    lm_damping: FiniteFloat = Field(1e-4, gt=0.0)
    task_gain: FiniteFloat = Field(1.0, gt=0.0)
    position_cost: FiniteFloat = Field(1.0, ge=0.0)
    orientation_cost: FiniteFloat = Field(1.0, ge=0.0)
    posture_cost: FiniteFloat = Field(1e-3, ge=0.0)
    reference_q: list[float] | None = None
    qpsolver_options: dict[str, FiniteFloat] = Field(default_factory=dict)


@dataclass(frozen=True)
class ControlIKResult:
    positions: NDArray[np.float64]
    velocity: NDArray[np.float64]


class IKControlRuntimeError(RuntimeError):
    """A runtime solver/model failure that should produce a bounded hold."""


@dataclass(frozen=True)
class _CoordinateMapping:
    joint_names: tuple[str, ...]
    q_indices: tuple[int, ...]
    v_indices: tuple[int, ...]
    q_widths: tuple[int, ...]
    joint_ids: frozenset[int]


@dataclass(frozen=True)
class _PinkRuntime:
    config: PinkControlIKConfig
    model: pinocchio.Model
    data: pinocchio.Data
    mapping: _CoordinateMapping
    ee_frame_id: int
    reference_q: NDArray[np.float64]
    configuration: Configuration
    frame_task: FrameTask
    posture_task: PostureTask | None
    tasks: list[object]
    limits: list[object]


class _PinkControlIKBuilder:
    """Assemble model and Pink state before creating the runtime solver."""

    def __init__(self, config: PinkControlIKConfig) -> None:
        self._config = config

    def build(self) -> _PinkRuntime:
        config = self._config
        robot = config.robot_model
        prepared_path = Path(
            prepare_urdf_for_drake(
                robot.model_path,
                package_paths=robot.package_paths,
                xacro_args=robot.xacro_args,
                convert_meshes=False,
            )
        )
        if not prepared_path.exists():
            raise FileNotFoundError(f"prepared Pink control URDF not found: {prepared_path}")

        model = pinocchio.buildModelFromUrdf(str(prepared_path))
        mapping = self._build_mapping(model, robot)
        ee_frame_id = self._validate_frame(model, robot.end_effector_link)
        limits = self._apply_limits(model, mapping, robot)
        full_reference_q = self._build_reference_q(model, config.reference_q)
        locked_joint_ids = [
            joint_id
            for joint_id in range(1, len(model.joints))
            if joint_id not in mapping.joint_ids
        ]
        if locked_joint_ids:
            if config.reference_q is None and self._uncontrolled_ee_chain(
                model, ee_frame_id, mapping.joint_ids
            ):
                raise ValueError(
                    "Pink requires reference_q for an uncontrolled joint on the end-effector chain"
                )
            model = pinocchio.buildReducedModel(model, locked_joint_ids, full_reference_q)
            mapping = self._build_mapping(model, robot)
            ee_frame_id = self._validate_frame(model, robot.end_effector_link)
            limits = self._apply_limits(model, mapping, robot)

        data = model.createData()
        reference_q = self._build_reference_q(model, None)
        configuration = Configuration(
            model,
            data,
            reference_q.copy(),
        )
        frame_task = FrameTask(
            robot.end_effector_link,
            position_cost=config.position_cost,
            orientation_cost=config.orientation_cost,
            lm_damping=config.lm_damping,
            gain=config.task_gain,
        )
        posture_task = PostureTask(cost=config.posture_cost) if config.posture_cost > 0.0 else None
        tasks: list[object] = [frame_task]
        if posture_task is not None:
            tasks.append(posture_task)

        return _PinkRuntime(
            config=config,
            model=model,
            data=data,
            mapping=mapping,
            ee_frame_id=ee_frame_id,
            reference_q=reference_q,
            configuration=configuration,
            frame_task=frame_task,
            posture_task=posture_task,
            tasks=tasks,
            limits=limits,
        )

    @staticmethod
    def _build_mapping(
        model: pinocchio.Model,
        robot: RobotModelConfig,
    ) -> _CoordinateMapping:
        joint_names = tuple(robot.get_coordinator_joint_names())
        if not joint_names or len(set(joint_names)) != len(joint_names):
            raise ValueError("control task joints must be unique and non-empty")

        q_indices: list[int] = []
        v_indices: list[int] = []
        q_widths: list[int] = []
        joint_ids: set[int] = set()
        for urdf_name in (robot.get_urdf_joint_name(name) for name in joint_names):
            if not model.existJointName(urdf_name):
                raise ValueError(f"control joint mapping references unknown joint: {urdf_name}")
            joint_id = int(model.getJointId(urdf_name))
            if joint_id <= 0 or joint_id >= len(model.joints):
                raise ValueError(f"invalid control joint index for {urdf_name}")
            joint = model.joints[joint_id]
            if int(joint.nv) != 1 or int(joint.nq) not in (1, 2):
                raise ValueError(f"control joint must be one-DoF: {urdf_name}")
            q_indices.append(int(joint.idx_q))
            v_indices.append(int(joint.idx_v))
            q_widths.append(int(joint.nq))
            joint_ids.add(joint_id)

        return _CoordinateMapping(
            joint_names=joint_names,
            q_indices=tuple(q_indices),
            v_indices=tuple(v_indices),
            q_widths=tuple(q_widths),
            joint_ids=frozenset(joint_ids),
        )

    @staticmethod
    def _build_reference_q(
        model: pinocchio.Model,
        configured_reference_q: list[float] | None,
    ) -> NDArray[np.float64]:
        if configured_reference_q is not None:
            q = np.asarray(configured_reference_q, dtype=np.float64).reshape(-1)
            if q.size != model.nq or not np.all(np.isfinite(q)):
                raise ValueError("Pink reference_q must match model nq and be finite")
        else:
            q = np.asarray(pinocchio.neutral(model), dtype=np.float64)
            for joint_id in range(1, len(model.joints)):
                joint = model.joints[joint_id]
                start = int(joint.idx_q)
                width = int(joint.nq)
                if width == 2 and int(joint.nv) == 1:
                    q[start : start + 2] = (1.0, 0.0)
                    continue
                if width != 1:
                    continue
                lower = model.lowerPositionLimit[start]
                upper = model.upperPositionLimit[start]
                if np.isfinite(lower) and np.isfinite(upper):
                    q[start] = (lower + upper) / 2.0
                elif np.isfinite(lower):
                    q[start] = max(0.0, lower)
                elif np.isfinite(upper):
                    q[start] = min(0.0, upper)
                else:
                    q[start] = 0.0
        if not np.all(np.isfinite(q)):
            raise ValueError("Pink reference configuration is not finite")
        bounded = np.isfinite(model.lowerPositionLimit) & np.isfinite(model.upperPositionLimit)
        if np.any(q[bounded] < model.lowerPositionLimit[bounded]) or np.any(
            q[bounded] > model.upperPositionLimit[bounded]
        ):
            raise ValueError("Pink reference configuration violates model limits")
        return q

    @staticmethod
    def _uncontrolled_ee_chain(
        model: pinocchio.Model,
        frame_id: int,
        controlled_joint_ids: frozenset[int],
    ) -> bool:
        joint_id = int(model.frames[frame_id].parentJoint)
        while joint_id > 0:
            if joint_id not in controlled_joint_ids:
                return True
            joint_id = int(model.parents[joint_id])
        return False

    @staticmethod
    def _validate_frame(model: pinocchio.Model, frame_name: str) -> int:
        if not model.existFrame(frame_name):
            raise ValueError(f"unknown control end-effector frame: {frame_name}")
        frame_id = int(model.getFrameId(frame_name))
        if frame_id < 0 or frame_id >= len(model.frames):
            raise ValueError(f"invalid control end-effector frame: {frame_name}")
        return frame_id

    def _apply_limits(
        self,
        model: pinocchio.Model,
        mapping: _CoordinateMapping,
        robot: RobotModelConfig,
    ) -> list[object]:
        if robot.joint_limits_lower is not None or robot.joint_limits_upper is not None:
            if robot.joint_limits_lower is None or robot.joint_limits_upper is None:
                raise ValueError("both configured joint limit bounds are required")
            if len(robot.joint_limits_lower) != len(mapping.joint_names) or len(
                robot.joint_limits_upper
            ) != len(mapping.joint_names):
                raise ValueError("configured joint limits do not match control joints")
            for index, width, lower, upper in zip(
                mapping.q_indices,
                mapping.q_widths,
                robot.joint_limits_lower,
                robot.joint_limits_upper,
                strict=True,
            ):
                if not np.isfinite(lower) or not np.isfinite(upper) or lower >= upper:
                    raise ValueError("configured joint limits must be finite and ordered")
                if width == 2:
                    raise ValueError(
                        "configured position limits for continuous joints require "
                        "tangent-space angular limit handling"
                    )
                model.lowerPositionLimit[index] = lower
                model.upperPositionLimit[index] = upper
        if robot.velocity_limits is not None:
            if len(robot.velocity_limits) != len(mapping.joint_names) or any(
                not np.isfinite(value) or value <= 0.0 for value in robot.velocity_limits
            ):
                raise ValueError("configured velocity limits are invalid")
            for index, limit in zip(mapping.v_indices, robot.velocity_limits, strict=True):
                model.velocityLimit[index] = limit
        for index in mapping.v_indices:
            model.velocityLimit[index] = min(model.velocityLimit[index], self._config.max_velocity)
        return [ConfigurationLimit(model), VelocityLimit(model)]


class PinkControlIK:
    """One-step Pink control IK assembled by :func:`create_pink_control_ik`."""

    def __init__(self, runtime: _PinkRuntime) -> None:
        self._runtime = runtime

    @property
    def nq(self) -> int:
        """Number of controlled coordinates, matching the task contract."""
        return len(self._runtime.mapping.joint_names)

    def forward_kinematics(self, q: NDArray[np.float64]) -> pinocchio.SE3:
        runtime = self._runtime
        full_q = self._full_q(q)
        pinocchio.forwardKinematics(runtime.model, runtime.data, full_q)
        pinocchio.updateFramePlacements(runtime.model, runtime.data)
        return runtime.data.oMf[runtime.ee_frame_id].copy()

    def solve(
        self,
        target: pinocchio.SE3,
        measured: NDArray[np.float64],
        dt: float,
    ) -> ControlIKResult:
        runtime = self._runtime
        measured = np.asarray(measured, dtype=np.float64).reshape(-1)
        if measured.size != self.nq or not np.all(np.isfinite(measured)):
            raise ValueError("measured joint state is invalid")
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError("control IK dt must be finite and positive")

        configuration = runtime.configuration
        frame_task = runtime.frame_task
        try:
            configuration.update(self._full_q(measured))
            frame_task.set_target(target)
            if runtime.posture_task is not None:
                runtime.posture_task.set_target(configuration.q.copy())
            velocity = solve_ik(
                configuration,
                runtime.tasks,
                dt,
                solver=runtime.config.solver,
                damping=runtime.config.lm_damping,
                limits=runtime.limits,
                **runtime.config.qpsolver_options,
            )
            velocity = np.asarray(velocity, dtype=np.float64).reshape(-1)
            if velocity.size != runtime.model.nv or not np.all(np.isfinite(velocity)):
                raise IKControlRuntimeError("Pink produced an invalid velocity")
            configuration.integrate_inplace(velocity, dt)
            candidate = self._project_controlled_positions(configuration.q, measured)
            if candidate.size != measured.size or not np.all(np.isfinite(candidate)):
                raise IKControlRuntimeError("Pink produced an invalid joint candidate")
            candidate = self._clamp_position_limits(candidate)
            return ControlIKResult(candidate, self._controlled_velocity(velocity))
        except IKControlRuntimeError:
            raise
        except Exception as exc:
            raise IKControlRuntimeError(f"Pink control solve failed: {exc}") from exc

    def _full_q(self, controlled: NDArray[np.float64]) -> NDArray[np.float64]:
        runtime = self._runtime
        mapping = runtime.mapping
        q = runtime.reference_q.copy()
        for value, index, width in zip(
            controlled, mapping.q_indices, mapping.q_widths, strict=True
        ):
            if width == 2:
                q[index] = np.cos(value)
                q[index + 1] = np.sin(value)
            else:
                q[index] = value
        return q

    def _project_controlled_positions(
        self, full_q: NDArray[np.float64], reference: NDArray[np.float64] | None = None
    ) -> NDArray[np.float64]:
        """Project model coordinates to coordinator joints and unwrap continuous angles."""
        mapping = self._runtime.mapping
        positions = np.array(
            [
                np.arctan2(full_q[index + 1], full_q[index]) if width == 2 else full_q[index]
                for index, width in zip(mapping.q_indices, mapping.q_widths, strict=True)
            ],
            dtype=np.float64,
        )
        if reference is not None:
            for index, width in enumerate(mapping.q_widths):
                if width == 2:
                    positions[index] = reference[index] + float(
                        (positions[index] - reference[index] + np.pi) % (2.0 * np.pi) - np.pi
                    )
        return positions

    def _controlled_velocity(self, velocity: NDArray[np.float64]) -> NDArray[np.float64]:
        return np.array(
            [velocity[index] for index in self._runtime.mapping.v_indices], dtype=np.float64
        )

    def _clamp_position_limits(self, candidate: NDArray[np.float64]) -> NDArray[np.float64]:
        runtime = self._runtime
        mapping = runtime.mapping
        bounded = candidate.copy()
        for index, width in enumerate(mapping.q_widths):
            if width != 1:
                continue
            q_index = mapping.q_indices[index]
            lower = runtime.model.lowerPositionLimit[q_index]
            upper = runtime.model.upperPositionLimit[q_index]
            value = bounded[index]
            if value < lower:
                if lower - value <= _POSITION_LIMIT_EPSILON_RAD:
                    bounded[index] = lower
                else:
                    raise IKControlRuntimeError("Pink produced an out-of-bounds joint candidate")
            elif value > upper:
                if value - upper <= _POSITION_LIMIT_EPSILON_RAD:
                    bounded[index] = upper
                else:
                    raise IKControlRuntimeError("Pink produced an out-of-bounds joint candidate")
        return bounded


def create_pink_control_ik(config: PinkControlIKConfig) -> PinkControlIK:
    """Construct the default Cartesian control IK backend."""
    return PinkControlIK(_PinkControlIKBuilder(config).build())
