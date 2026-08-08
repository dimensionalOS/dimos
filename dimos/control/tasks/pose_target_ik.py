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

"""Shared bounded Pink IK machinery for streaming pose-target tasks."""

from __future__ import annotations

from abc import abstractmethod
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
import math
import threading
from typing import TYPE_CHECKING

import attrs
import numpy as np

from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.kinematics.pink_solver import (
    PinkJointLimitError,
    _PinkSolverCore,
)
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped

logger = setup_logger()

_FEEDBACK_LIMIT_WARNING_INTERVAL_S = 1.0


def _to_tuple(value: Sequence[str]) -> tuple[str, ...]:
    return tuple(value)


def _validate_unique_names(
    _instance: object,
    attribute: attrs.Attribute[tuple[str, ...]],
    value: tuple[str, ...],
) -> None:
    singular = "joint" if attribute.name == "joint_names" else "target frame"
    plural = "joint names" if attribute.name == "joint_names" else "target frames"
    if not value:
        raise ValueError(f"PoseTargetIKTask requires at least one {singular}")
    if len(set(value)) != len(value):
        raise ValueError(f"PoseTargetIKTask requires unique {plural}")


def _positive_finite(
    _instance: object,
    attribute: attrs.Attribute[float],
    value: float,
) -> None:
    if not math.isfinite(value) or value <= 0.0:
        label = (
            "joint velocity limit"
            if attribute.name == "max_joint_velocity_rad_s"
            else "command tracking error"
        )
        raise ValueError(f"PoseTargetIKTask requires a positive finite {label}")


def _optional_positive_finite(
    _instance: object,
    _attribute: attrs.Attribute[float],
    value: float | None,
) -> None:
    if value is not None and (not math.isfinite(value) or value <= 0.0):
        raise ValueError("PoseTargetIKTask requires a positive finite joint command filter cutoff")


def _to_optional_float(value: float | None) -> float | None:
    return None if value is None else float(value)


def _to_joint_velocity_limits(value: Mapping[str, float]) -> dict[str, float]:
    return {name: float(limit) for name, limit in value.items()}


def _validate_joint_velocity_limits(
    instance: PoseTargetIKTaskConfig,
    _attribute: attrs.Attribute[dict[str, float]],
    value: dict[str, float],
) -> None:
    unknown_joints = sorted(set(value) - set(instance.joint_names))
    if unknown_joints:
        raise ValueError(
            f"PoseTargetIKTask joint velocity limits contain unknown joints: {unknown_joints}"
        )
    invalid_joints = sorted(
        name for name, limit in value.items() if not math.isfinite(limit) or limit <= 0.0
    )
    if invalid_joints:
        raise ValueError(
            "PoseTargetIKTask requires a positive finite velocity limit per joint; "
            f"invalid joints: {invalid_joints}"
        )


def _nonnegative_finite(
    _instance: object,
    attribute: attrs.Attribute[float],
    value: float,
) -> None:
    if not math.isfinite(value) or value < 0.0:
        label = (
            "feedback tolerance"
            if attribute.name == "feedback_limit_tolerance"
            else "command limit margin"
        )
        raise ValueError(f"PoseTargetIKTask requires a non-negative finite {label}")


@attrs.frozen(slots=False)
class PoseTargetIKTaskConfig:
    """Configuration shared by absolute and Quest pose-target tasks."""

    joint_names: tuple[str, ...] = attrs.field(
        converter=_to_tuple,
        validator=_validate_unique_names,
    )
    robot_model: RobotModelConfig = attrs.field(
        validator=attrs.validators.instance_of(RobotModelConfig)
    )
    target_frames: tuple[str, ...] = attrs.field(
        converter=_to_tuple,
        validator=_validate_unique_names,
    )
    pink: PinkKinematicsConfig = attrs.field(factory=PinkKinematicsConfig)
    priority: int = attrs.field(default=10, converter=int)
    timeout: float = attrs.field(default=0.5, converter=float)
    max_joint_velocity_rad_s: float = attrs.field(
        default=5.0,
        converter=float,
        validator=_positive_finite,
    )
    joint_velocity_limits_rad_s: dict[str, float] = attrs.field(
        factory=dict,
        converter=_to_joint_velocity_limits,
        validator=_validate_joint_velocity_limits,
    )
    joint_command_filter_cutoff_hz: float | None = attrs.field(
        default=5.0,
        converter=_to_optional_float,
        validator=_optional_positive_finite,
    )
    max_command_tracking_error_deg: float = attrs.field(
        default=10.0,
        converter=float,
        validator=_positive_finite,
    )
    feedback_limit_tolerance: float = attrs.field(
        default=1e-3,
        converter=float,
        validator=_nonnegative_finite,
    )
    command_limit_margin: float = attrs.field(
        default=1e-4,
        converter=float,
        validator=_nonnegative_finite,
    )


@dataclass(frozen=True)
class FrameTargetSnapshot:
    """One atomic control-tick snapshot produced by a task leaf."""

    targets: Mapping[str, PoseStamped]
    last_update_time: float
    extra_joint_positions: Mapping[str, float] = field(default_factory=dict)


class PinkPoseTargetSolver(_PinkSolverCore):
    """Stateful Pink solver owned by one pose-target control task."""

    def __init__(self, config: PoseTargetIKTaskConfig) -> None:
        super().__init__(config.pink)
        self._control_config = config
        self._command_state_lock = threading.Lock()
        self._command_state: JointState | None = None
        self._command_state_generation = 0
        self._validate_frame_targets(
            config.robot_model,
            config.target_frames,
            config.joint_names,
            config.command_limit_margin,
        )

    def step(
        self,
        frame_targets: Mapping[str, PoseStamped],
        measured_state: JointState,
        dt: float,
    ) -> JointState | None:
        """Advance the persistent command trajectory by one bounded QP step."""
        with self._command_state_lock:
            command_state = _copy_joint_state(self._command_state or measured_state)
            generation = self._command_state_generation
        result = self._step_frame_targets(
            robot_model=self._control_config.robot_model,
            frame_targets=frame_targets,
            controlled_joints=self._control_config.joint_names,
            command_state=command_state,
            measured_state=measured_state,
            max_command_tracking_error_rad=float(
                np.deg2rad(self._control_config.max_command_tracking_error_deg)
            ),
            feedback_limit_tolerance=self._control_config.feedback_limit_tolerance,
            command_limit_margin=self._control_config.command_limit_margin,
            dt=dt,
            max_joint_velocity_rad_s=self._control_config.max_joint_velocity_rad_s,
            joint_velocity_limits_rad_s=self._control_config.joint_velocity_limits_rad_s,
            joint_command_filter_cutoff_hz=self._control_config.joint_command_filter_cutoff_hz,
        )
        with self._command_state_lock:
            if self._command_state_generation != generation:
                return None
            self._command_state = _copy_joint_state(result)
        return result

    def frame_poses(
        self,
        measured_state: JointState,
        frame_names: Sequence[str],
    ) -> dict[str, PoseStamped]:
        """Return current world poses for controlled frames."""
        return self._frame_poses(
            self._control_config.robot_model,
            frame_names,
            self._control_config.joint_names,
            measured_state,
        )

    def reset(self) -> None:
        """Discard the persistent command trajectory."""
        with self._command_state_lock:
            self._command_state = None
            self._command_state_generation += 1


class PoseTargetIKTask(BaseControlTask):
    """Turn frame targets into a feedback-bounded persistent command trajectory."""

    def __init__(
        self,
        name: str,
        config: PoseTargetIKTaskConfig,
        *,
        additional_claimed_joints: Sequence[str] = (),
        solver: PinkPoseTargetSolver | None = None,
        solver_type: type[PinkPoseTargetSolver] | None = None,
    ) -> None:
        additional_joints = tuple(additional_claimed_joints)
        if set(config.joint_names) & set(additional_joints):
            raise ValueError(
                f"PoseTargetIKTask '{name}' has duplicate IK and additional claimed joints"
            )
        if len(set(additional_joints)) != len(additional_joints):
            raise ValueError(f"PoseTargetIKTask '{name}' requires unique additional joints")

        self._name = name
        self._config = config
        self._joint_names = config.joint_names
        self._additional_claimed_joints = additional_joints
        self._feedback_limit_warning_times: dict[tuple[str, str], float] = {}
        if solver is not None and solver_type is not None:
            raise ValueError("PoseTargetIKTask accepts either solver or solver_type, not both")
        self._solver = solver or (solver_type or PinkPoseTargetSolver)(config)

    def claim(self) -> ResourceClaim:
        """Claim IK-controlled and leaf-provided additional joints."""
        return ResourceClaim(
            joints=frozenset((*self._joint_names, *self._additional_claimed_joints)),
            priority=self._config.priority,
            mode=ControlMode.SERVO_POSITION,
        )

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        """Perform at most one bounded Pink update for this coordinator tick."""
        snapshot = self._frame_target_snapshot(state)
        if snapshot is None:
            return None
        if self._config.timeout > 0.0:
            age = state.t_now - snapshot.last_update_time
            if age > self._config.timeout:
                self._reset_command_state()
                self._on_target_timeout()
                return None

        measured_state = self._measured_joint_state(state)
        if measured_state is None:
            self._reset_command_state()
            return None
        try:
            result = self._solver.step(snapshot.targets, measured_state, state.dt)
        except PinkJointLimitError as exc:
            warning_key = (exc.joint_name, exc.boundary)
            last_warning = self._feedback_limit_warning_times.get(warning_key)
            if (
                last_warning is None
                or state.t_now - last_warning >= _FEEDBACK_LIMIT_WARNING_INTERVAL_S
            ):
                logger.warning(
                    "Measured joint feedback exceeds Pink limit tolerance",
                    task=self._name,
                    joint=exc.joint_name,
                    value=exc.value,
                    lower=exc.lower,
                    upper=exc.upper,
                    tolerance=exc.tolerance,
                )
                self._feedback_limit_warning_times[warning_key] = state.t_now
            self._reset_command_state()
            return None
        # Pink/QP failures derive directly from Exception rather than a stable
        # built-in subtype. A failed streaming tick must not stop the coordinator.
        except Exception as exc:
            logger.warning("Pink control step failed", task=self._name, error=str(exc))
            return None
        if result is None:
            return None
        if result.name != list(self._joint_names) or len(result.position) != len(self._joint_names):
            logger.warning("Pink control step returned an invalid joint result", task=self._name)
            return None

        positions = np.asarray(result.position, dtype=np.float64)
        if not np.all(np.isfinite(positions)):
            logger.warning("Pink control step returned non-finite positions", task=self._name)
            return None

        output_names = list(self._joint_names)
        output_positions = positions.tolist()
        for joint_name in self._additional_claimed_joints:
            if joint_name not in snapshot.extra_joint_positions:
                logger.warning(
                    "Pose-target task snapshot omitted an additional claimed joint",
                    task=self._name,
                    joint=joint_name,
                )
                return None
            output_names.append(joint_name)
            output_positions.append(float(snapshot.extra_joint_positions[joint_name]))
        return JointCommandOutput(
            joint_names=output_names,
            positions=output_positions,
            mode=ControlMode.SERVO_POSITION,
        )

    def current_frame_poses(
        self, state: CoordinatorState, frame_names: Sequence[str]
    ) -> dict[str, PoseStamped] | None:
        """Return live poses for task frames, or ``None`` without full feedback."""
        measured_state = self._measured_joint_state(state)
        if measured_state is None:
            return None
        return self._solver.frame_poses(measured_state, frame_names)

    def on_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        """Notify the leaf when any of its claimed joints are preempted."""
        if joints & self.claim().joints:
            self._reset_command_state()
            self._on_pose_target_preempted(by_task, joints)

    def _measured_joint_state(self, state: CoordinatorState) -> JointState | None:
        positions: list[float] = []
        for joint_name in self._joint_names:
            position = state.joints.get_position(joint_name)
            if position is None:
                return None
            positions.append(position)
        return JointState(name=list(self._joint_names), position=positions)

    def _reset_command_state(self) -> None:
        """Discard the active command trajectory and clear a tracking fault."""
        self._solver.reset()

    @abstractmethod
    def _frame_target_snapshot(self, state: CoordinatorState) -> FrameTargetSnapshot | None:
        """Return the leaf's atomic target snapshot for this tick."""

    def _on_target_timeout(self) -> None:
        """Allow a leaf to clear state after a stale snapshot."""

    def _on_pose_target_preempted(self, by_task: str, joints: frozenset[str]) -> None:
        """Allow a leaf to reset semantics after preemption."""


def _copy_joint_state(state: JointState) -> JointState:
    return JointState(name=list(state.name), position=list(state.position))
