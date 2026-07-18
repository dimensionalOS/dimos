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

"""Pink and legacy Pinocchio backends for coordinator Cartesian control."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path
from typing import Literal

import numpy as np
from numpy.typing import NDArray
import pink
from pink.limits import ConfigurationLimit, Limit, VelocityLimit
import pinocchio
from pydantic import Field, field_validator

from dimos.manipulation.planning.kinematics.pinocchio_ik import PinocchioIK
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.manipulation.planning.utils.mesh_utils import prepare_urdf_for_drake
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.protocol.service.spec import BaseConfig

# Pink's integration/QP boundary tolerance is small but larger than machine epsilon.
_POSITION_LIMIT_EPSILON_RAD = 1e-5


class PinkControlIKConfig(BaseConfig):
    """Typed configuration for the control IK backend."""

    backend: Literal["pink", "pinocchio"] = "pink"
    robot_model: RobotModelConfig | None = None
    solver: str = "proxqp"
    max_velocity: float = 10.0
    lm_damping: float = 1e-4
    task_gain: float = 1.0
    position_cost: float = 1.0
    orientation_cost: float = 1.0
    min_dt: float = 1e-4
    max_dt: float = 0.05
    reference_q: list[float] | None = None
    qpsolver_options: dict[str, float] = Field(default_factory=dict)

    @field_validator("robot_model", mode="before")
    @classmethod
    def _rebuild_robot_model(cls, value: object) -> RobotModelConfig | None:
        if value is None or isinstance(value, RobotModelConfig):
            return value
        if not isinstance(value, Mapping):
            raise ValueError("Pink robot_model must be a serialized RobotModelConfig")
        payload = dict(value)
        base_pose = payload.get("base_pose")
        if isinstance(base_pose, Mapping):
            position = base_pose.get("position")
            orientation = base_pose.get("orientation")
            if not isinstance(position, list) or not isinstance(orientation, list):
                raise ValueError("serialized RobotModelConfig base_pose is invalid")
            payload["base_pose"] = PoseStamped(
                ts=float(base_pose.get("ts", 0.0)),
                frame_id=str(base_pose.get("frame_id", "")),
                position=position,
                orientation=orientation,
            )
        return RobotModelConfig.model_validate(payload)

    def validate_settings(
        self,
        joint_count: int,
        ee_joint_id: int | None,
        model_path: str | Path | None = None,
    ) -> None:
        numeric = (
            self.max_velocity,
            self.lm_damping,
            self.task_gain,
            self.position_cost,
            self.orientation_cost,
            self.min_dt,
            self.max_dt,
        )
        if not all(np.isfinite(value) for value in numeric):
            raise ValueError("control IK numeric settings must be finite")
        if self.max_velocity <= 0.0 or self.lm_damping <= 0.0 or self.task_gain <= 0.0:
            raise ValueError("control IK velocity, damping, and gain must be positive")
        if self.position_cost < 0.0 or self.orientation_cost < 0.0:
            raise ValueError("control IK task costs must not be negative")
        if self.min_dt <= 0.0 or self.max_dt < self.min_dt:
            raise ValueError("control IK dt bounds must be positive and ordered")
        if any(not np.isfinite(value) for value in self.qpsolver_options.values()):
            raise ValueError("control IK QP options must be finite")
        if self.backend == "pink":
            if self.robot_model is None:
                raise ValueError("Pink control requires a RobotModelConfig")
            if not self.robot_model.end_effector_link:
                raise ValueError("Pink control requires a named end-effector frame")
            if len(self.robot_model.joint_names) != joint_count:
                raise ValueError("RobotModelConfig and control task joint counts differ")
            if (
                model_path is not None
                and Path(self.robot_model.model_path).resolve() != Path(model_path).resolve()
            ):
                raise ValueError("Pink RobotModelConfig must use the authoritative model path")
        elif not isinstance(ee_joint_id, int) or isinstance(ee_joint_id, bool):
            raise ValueError("Pinocchio control requires a numeric ee_joint_id")


@dataclass(frozen=True)
class ControlIKResult:
    positions: NDArray[np.float64]
    velocity: NDArray[np.float64]


class PinkControlRuntimeError(RuntimeError):
    """A runtime solver/model failure that should produce a bounded hold."""


class PinkControlIK:
    """One-step Pink control IK with explicit legacy Pinocchio compatibility."""

    def __init__(
        self,
        model_path: str | Path,
        ee_joint_id: int | None,
        joint_names: list[str],
        config: PinkControlIKConfig,
    ) -> None:
        self._config = config
        self._joint_names = list(joint_names)
        self._config.validate_settings(len(self._joint_names), ee_joint_id, model_path)
        self._is_pinocchio = config.backend == "pinocchio"
        self._legacy_ik: PinocchioIK | None = None

        if self._is_pinocchio:
            if ee_joint_id is None:
                raise ValueError("Pinocchio control requires an explicit ee_joint_id")
            self._legacy_ik = PinocchioIK.from_model_path(model_path, ee_joint_id)
            self._model = self._legacy_ik.model
            self._data = self._model.createData()
            self._q_indices: list[int] = []
            self._v_indices: list[int] = []
            self._configuration = None
            self._frame_task = None
            self._limits: list[Limit] = []
            return

        robot = config.robot_model
        if robot is None:  # guarded by validate_settings; retained for narrowing
            raise ValueError("Pink control requires a RobotModelConfig")
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

        self._model = pinocchio.buildModelFromUrdf(str(prepared_path))
        self._data = self._model.createData()
        self._q_indices, self._v_indices = self._build_mapping(robot)
        self._ee_frame_id = self._validate_frame(robot.end_effector_link)
        self._apply_limits(robot)
        full_reference_q = self._build_reference_q()
        controlled_joint_ids = set(self._controlled_joint_ids)
        locked_joint_ids = [
            joint_id
            for joint_id in range(1, len(self._model.joints))
            if joint_id not in controlled_joint_ids
        ]
        if locked_joint_ids:
            if self._config.reference_q is None and self._uncontrolled_ee_chain(
                self._ee_frame_id, controlled_joint_ids
            ):
                raise ValueError(
                    "Pink requires reference_q for an uncontrolled joint on the end-effector chain"
                )
            self._model = pinocchio.buildReducedModel(
                self._model, locked_joint_ids, full_reference_q
            )
            self._data = self._model.createData()
            self._q_indices, self._v_indices = self._build_mapping(robot)
            self._ee_frame_id = self._validate_frame(robot.end_effector_link)
            self._apply_limits(robot)
        self._reference_q = self._build_reference_q(use_config_reference=False)
        self._configuration = pink.Configuration(
            self._model,
            self._data,
            self._reference_q.copy(),
        )
        self._frame_task = pink.tasks.FrameTask(
            robot.end_effector_link,
            position_cost=config.position_cost,
            orientation_cost=config.orientation_cost,
            lm_damping=config.lm_damping,
            gain=config.task_gain,
        )

    @property
    def nq(self) -> int:
        """Number of controlled coordinates, matching the task contract."""
        if self._is_pinocchio:
            if self._legacy_ik is None:
                raise PinkControlRuntimeError("Pinocchio control backend is unavailable")
            return self._legacy_ik.nq
        return len(self._joint_names)

    def forward_kinematics(self, q: NDArray[np.float64]) -> pinocchio.SE3:
        if self._is_pinocchio:
            if self._legacy_ik is None:
                raise PinkControlRuntimeError("Pinocchio control backend is unavailable")
            return self._legacy_ik.forward_kinematics(q)
        full_q = self._full_q(q)
        pinocchio.forwardKinematics(self._model, self._data, full_q)
        pinocchio.updateFramePlacements(self._model, self._data)
        return self._data.oMf[self._ee_frame_id].copy()

    def solve(
        self,
        target: pinocchio.SE3,
        measured: NDArray[np.float64],
        dt: float,
    ) -> ControlIKResult:
        measured = np.asarray(measured, dtype=np.float64).reshape(-1)
        if measured.size != len(self._joint_names) or not np.all(np.isfinite(measured)):
            raise ValueError("measured joint state is invalid")
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError("control IK dt must be finite and positive")
        dt = min(max(dt, self._config.min_dt), self._config.max_dt)
        if self._is_pinocchio:
            if self._legacy_ik is None:
                raise PinkControlRuntimeError("Pinocchio control backend is unavailable")
            positions, _, _ = self._legacy_ik.solve(target, measured)
            return ControlIKResult(np.asarray(positions, dtype=np.float64), positions - measured)

        configuration = self._configuration
        frame_task = self._frame_task
        if configuration is None or frame_task is None:
            raise PinkControlRuntimeError("Pink control backend is unavailable")
        try:
            configuration.update(self._full_q(measured))
            frame_task.set_target(target)
            velocity = pink.solve_ik(
                configuration,
                [frame_task],
                dt,
                solver=self._config.solver,
                damping=self._config.lm_damping,
                limits=self._limits,
                **self._config.qpsolver_options,
            )
            velocity = np.asarray(velocity, dtype=np.float64).reshape(-1)
            if velocity.size != self._model.nv or not np.all(np.isfinite(velocity)):
                raise PinkControlRuntimeError("Pink produced an invalid velocity")
            configuration.integrate_inplace(velocity, dt)
            candidate = self._controlled_q(configuration.q, measured)
            if candidate.size != measured.size or not np.all(np.isfinite(candidate)):
                raise PinkControlRuntimeError("Pink produced an invalid joint candidate")
            candidate = self._clamp_position_limits(candidate)
            return ControlIKResult(candidate, self._controlled_velocity(velocity))
        except PinkControlRuntimeError:
            raise
        except Exception as exc:
            raise PinkControlRuntimeError(f"Pink control solve failed: {exc}") from exc

    def _full_q(self, controlled: NDArray[np.float64]) -> NDArray[np.float64]:
        q = self._reference_q.copy()
        for value, index, width in zip(controlled, self._q_indices, self._q_widths, strict=True):
            if width == 2:
                q[index] = np.cos(value)
                q[index + 1] = np.sin(value)
            else:
                q[index] = value
        return q

    def _controlled_q(
        self, full_q: NDArray[np.float64], reference: NDArray[np.float64] | None = None
    ) -> NDArray[np.float64]:
        positions = np.array(
            [
                np.arctan2(full_q[index + 1], full_q[index]) if width == 2 else full_q[index]
                for index, width in zip(self._q_indices, self._q_widths, strict=True)
            ],
            dtype=np.float64,
        )
        if reference is not None:
            for index, width in enumerate(self._q_widths):
                if width == 2:
                    positions[index] = reference[index] + float(
                        (positions[index] - reference[index] + np.pi) % (2.0 * np.pi) - np.pi
                    )
        return positions

    def _controlled_velocity(self, velocity: NDArray[np.float64]) -> NDArray[np.float64]:
        return np.array([velocity[index] for index in self._v_indices], dtype=np.float64)

    def _clamp_position_limits(self, candidate: NDArray[np.float64]) -> NDArray[np.float64]:
        bounded = candidate.copy()
        for index, width in enumerate(self._q_widths):
            if width != 1:
                continue
            q_index = self._q_indices[index]
            lower = self._model.lowerPositionLimit[q_index]
            upper = self._model.upperPositionLimit[q_index]
            value = bounded[index]
            if np.isfinite(lower) and value < lower:
                if lower - value <= _POSITION_LIMIT_EPSILON_RAD:
                    bounded[index] = lower
                else:
                    raise PinkControlRuntimeError("Pink produced an out-of-bounds joint candidate")
            elif np.isfinite(upper) and value > upper:
                if value - upper <= _POSITION_LIMIT_EPSILON_RAD:
                    bounded[index] = upper
                else:
                    raise PinkControlRuntimeError("Pink produced an out-of-bounds joint candidate")
        return bounded

    def _build_mapping(self, robot: RobotModelConfig) -> tuple[list[int], list[int]]:
        coordinator_names = robot.get_coordinator_joint_names()
        if coordinator_names != self._joint_names or len(set(coordinator_names)) != len(
            coordinator_names
        ):
            raise ValueError(
                "control task joints must exactly match ordered RobotModelConfig joints"
            )
        indices: list[int] = []
        velocity_indices: list[int] = []
        self._q_widths: list[int] = []
        self._controlled_joint_ids: list[int] = []
        for urdf_name in (robot.get_urdf_joint_name(name) for name in coordinator_names):
            if not self._model.existJointName(urdf_name):
                raise ValueError(f"control joint mapping references unknown joint: {urdf_name}")
            joint_id = self._model.getJointId(urdf_name)
            if joint_id <= 0 or joint_id >= len(self._model.joints):
                raise ValueError(f"invalid control joint index for {urdf_name}")
            joint = self._model.joints[joint_id]
            if int(joint.nv) != 1 or int(joint.nq) not in (1, 2):
                raise ValueError(f"control joint must be one-DoF: {urdf_name}")
            indices.append(int(joint.idx_q))
            velocity_indices.append(int(joint.idx_v))
            self._q_widths.append(int(joint.nq))
            self._controlled_joint_ids.append(joint_id)
        return indices, velocity_indices

    def _build_reference_q(self, use_config_reference: bool = True) -> NDArray[np.float64]:
        if use_config_reference and self._config.reference_q is not None:
            q = np.asarray(self._config.reference_q, dtype=np.float64).reshape(-1)
            if q.size != self._model.nq or not np.all(np.isfinite(q)):
                raise ValueError("Pink reference_q must match model nq and be finite")
        else:
            q = np.asarray(pinocchio.neutral(self._model), dtype=np.float64)
        if not (use_config_reference and self._config.reference_q is not None):
            for joint_id in range(1, len(self._model.joints)):
                joint = self._model.joints[joint_id]
                start = int(joint.idx_q)
                width = int(joint.nq)
                if width == 2 and int(joint.nv) == 1:
                    q[start : start + 2] = (1.0, 0.0)
                    continue
                if width != 1:
                    continue
                for index in range(start, start + width):
                    lower = self._model.lowerPositionLimit[index]
                    upper = self._model.upperPositionLimit[index]
                    if np.isfinite(lower) and np.isfinite(upper):
                        q[index] = (lower + upper) / 2.0
                    elif np.isfinite(lower):
                        q[index] = max(0.0, lower)
                    elif np.isfinite(upper):
                        q[index] = min(0.0, upper)
                    else:
                        q[index] = 0.0
        if not np.all(np.isfinite(q)):
            raise ValueError("Pink reference configuration is not finite")
        bounded = np.isfinite(self._model.lowerPositionLimit) & np.isfinite(
            self._model.upperPositionLimit
        )
        if np.any(q[bounded] < self._model.lowerPositionLimit[bounded]) or np.any(
            q[bounded] > self._model.upperPositionLimit[bounded]
        ):
            raise ValueError("Pink reference configuration violates model limits")
        return q

    def _uncontrolled_ee_chain(self, frame_id: int, controlled_joint_ids: set[int]) -> bool:
        joint_id = int(self._model.frames[frame_id].parentJoint)
        while joint_id > 0:
            if joint_id not in controlled_joint_ids:
                return True
            joint_id = int(self._model.parents[joint_id])
        return False

    def _validate_frame(self, frame_name: str) -> int:
        if not self._model.existFrame(frame_name):
            raise ValueError(f"unknown control end-effector frame: {frame_name}")
        frame_id = int(self._model.getFrameId(frame_name))
        if frame_id < 0 or frame_id >= len(self._model.frames):
            raise ValueError(f"invalid control end-effector frame: {frame_name}")
        return frame_id

    def _apply_limits(self, robot: RobotModelConfig) -> None:
        if robot.joint_limits_lower is not None or robot.joint_limits_upper is not None:
            if robot.joint_limits_lower is None or robot.joint_limits_upper is None:
                raise ValueError("both configured joint limit bounds are required")
            if len(robot.joint_limits_lower) != len(self._joint_names) or len(
                robot.joint_limits_upper
            ) != len(self._joint_names):
                raise ValueError("configured joint limits do not match control joints")
            for index, width, lower, upper in zip(
                self._q_indices,
                self._q_widths,
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
                self._model.lowerPositionLimit[index] = lower
                self._model.upperPositionLimit[index] = upper
        if robot.velocity_limits is not None:
            if len(robot.velocity_limits) != len(self._joint_names) or any(
                not np.isfinite(value) or value <= 0.0 for value in robot.velocity_limits
            ):
                raise ValueError("configured velocity limits are invalid")
            for index, limit in zip(self._v_indices, robot.velocity_limits, strict=True):
                self._model.velocityLimit[index] = limit
        for index in self._v_indices:
            self._model.velocityLimit[index] = min(
                self._model.velocityLimit[index], self._config.max_velocity
            )
        self._limits = [ConfigurationLimit(self._model), VelocityLimit(self._model)]
