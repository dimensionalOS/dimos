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
from collections import deque
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, field
import math
import threading
from types import MappingProxyType
from typing import TYPE_CHECKING

import attrs
import numpy as np
import pink
from pydantic import Field

from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.manipulation.planning.kinematics.config import PinkKinematicsConfig
from dimos.manipulation.planning.kinematics.pink_solver import (
    _get_frame_id,
    _PinkRobotContext,
    _PinkSolverCore,
    _seed_positions_for_mapping,
)
from dimos.manipulation.planning.spec.config import RobotModelConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.protocol.service.spec import BaseConfig
from dimos.robot.assets.model import RobotModel
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import matrix_to_pose, pose_to_matrix

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = setup_logger()

_FEEDBACK_LIMIT_WARNING_INTERVAL_S = 1.0


def string_tuple_converter(values: Iterable[str]) -> tuple[str, ...]:
    """Give attrs a typed constructor for immutable sequence fields."""
    return tuple(values)


class PinkJointLimitError(ValueError):
    """Raised when measured feedback exceeds its model limit tolerance."""

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
        converter=string_tuple_converter,
        validator=_validate_unique_names,
    )
    robot_model: RobotModelConfig = attrs.field(
        validator=attrs.validators.instance_of(RobotModelConfig)
    )
    target_frames: tuple[str, ...] = attrs.field(
        converter=string_tuple_converter,
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
        converter=attrs.converters.optional(float),
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


class PoseTargetIKTaskParams(BaseConfig):
    """Serialized configuration shared by streaming pose-target tasks.

    The velocity fields bound each generated command step. The command filter
    suppresses high-frequency QP motion, while the tracking-error and limit
    margins keep the generated trajectory close to measured hardware state and
    inside the robot model's position limits.
    """

    robot_model: RobotModelConfig
    pink: PinkKinematicsConfig = Field(default_factory=PinkKinematicsConfig)
    timeout: float = 0.5
    max_joint_velocity_rad_s: float = 5.0
    joint_velocity_limits_rad_s: dict[str, float] = Field(default_factory=dict)
    joint_command_filter_cutoff_hz: float | None = 5.0
    max_command_tracking_error_deg: float = 10.0
    feedback_limit_tolerance: float = 1e-3
    command_limit_margin: float = 1e-4


@dataclass(frozen=True)
class FrameTargetSnapshot:
    """One atomic control-tick snapshot produced by a task leaf."""

    targets: Mapping[str, PoseStamped]
    last_update_time: float
    extra_joint_positions: Mapping[str, float] = field(default_factory=dict)


@dataclass
class _PinkControlContext:
    robot: _PinkRobotContext
    frames: Mapping[str, _PinkRobotContext]
    tasks: Mapping[str, pink.Task] | None = None


@dataclass(frozen=True)
class _StreamingStepResult:
    command: JointState
    bounded_increment: NDArray[np.float64]


class PinkPoseTargetSolver(_PinkSolverCore):
    """Stateful Pink solver owned by one pose-target control task."""

    joint_increment_filter_weights: tuple[float, ...] = (0.1, 0.3, 0.6)

    def __init__(self, config: PoseTargetIKTaskConfig) -> None:
        super().__init__(config.pink)
        self._control_config = config
        self._control_contexts: dict[
            tuple[RobotModel, tuple[str, ...], tuple[str, ...]], _PinkControlContext
        ] = {}
        self._command_state_lock = threading.Lock()
        self._command_state: JointState | None = None
        self._joint_increment_filter_weights = self._validated_increment_filter_weights()
        self._command_increment_history: deque[NDArray[np.float64]] = deque(
            maxlen=max(0, len(self._joint_increment_filter_weights) - 1)
        )
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
            command_state = JointState(self._command_state or measured_state)
            command_increment_history = tuple(
                increment.copy() for increment in self._command_increment_history
            )
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
            command_increment_history=command_increment_history,
        )
        with self._command_state_lock:
            if self._command_state_generation != generation:
                return None
            self._command_state = JointState(result.command)
            self._command_increment_history.append(result.bounded_increment.copy())
        return result.command

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
            self._command_increment_history.clear()
            self._command_state_generation += 1

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
        command_increment_history: Sequence[NDArray[np.float64]] = (),
    ) -> _StreamingStepResult:
        """Perform one feedback-bounded Pink update for the control loop."""
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
        instantaneous_positions = self._apply_streaming_command_envelope(
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
        bounded_increment = instantaneous_positions - previous_positions
        filtered_increment = self._weighted_joint_increment(
            bounded_increment,
            command_increment_history,
        )
        command_positions = self._apply_streaming_command_envelope(
            context=robot_context,
            candidate=previous_positions + filtered_increment,
            previous=previous_positions,
            measured=measured_positions,
            dt=step_dt,
            max_joint_velocity=max_joint_velocity_rad_s,
            joint_velocity_limits=joint_velocity_limits,
            max_tracking_error=max_command_tracking_error_rad,
            command_limit_margin=command_limit_margin,
        )
        return _StreamingStepResult(
            command=JointState(name=list(joint_names), position=command_positions.tolist()),
            bounded_increment=bounded_increment,
        )

    def _validated_increment_filter_weights(self) -> NDArray[np.float64]:
        weights = np.asarray(self.joint_increment_filter_weights, dtype=np.float64)
        if weights.ndim != 1 or len(weights) == 0:
            raise ValueError("Pink joint increment filter weights must be a non-empty sequence")
        if not np.all(np.isfinite(weights)) or np.any(weights <= 0.0):
            raise ValueError("Pink joint increment filter weights must be positive and finite")
        if np.any(np.diff(weights) <= 0.0):
            raise ValueError("Pink joint increment filter weights must increase toward newer steps")
        return weights

    def _weighted_joint_increment(
        self,
        bounded_increment: NDArray[np.float64],
        history: Sequence[NDArray[np.float64]],
    ) -> NDArray[np.float64]:
        history_length = len(self._joint_increment_filter_weights) - 1
        recent_history = tuple(history[-history_length:]) if history_length else ()
        samples = (*recent_history, bounded_increment)
        if any(sample.shape != bounded_increment.shape for sample in samples):
            raise ValueError(
                "Pink joint increment history does not match the controlled joint count"
            )
        weights = self._joint_increment_filter_weights[-len(samples) :]
        normalized_weights = weights / np.sum(weights)
        return np.asarray(
            np.average(np.stack(samples), axis=0, weights=normalized_weights),
            dtype=np.float64,
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
        context = self._get_control_context(
            robot_model,
            tuple(frame_names),
            tuple(controlled_joints),
        )
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
            config.model,
            frames,
            tuple(controlled_joints),
        )
        if cache_key not in self._control_contexts:
            robot_context = self._build_robot_context(config, frames[0], controlled_joints)
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
            normalized[q_index] = np.clip(float(q[q_index]), lower + margin, upper - margin)
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
        for _joint_name, q_index, urdf_lower, urdf_upper in _bounded_controlled_joint_limits(
            context
        ):
            index = controlled_index_by_q[q_index]
            lower[index] = max(lower[index], urdf_lower + command_limit_margin)
            upper[index] = min(upper[index], urdf_upper - command_limit_margin)

        invalid = np.flatnonzero(lower > upper)
        if invalid.size:
            joint_name = context.mapping.dimos_joint_names[int(invalid[0])]
            raise ValueError(f"Pink streaming command envelope is empty for '{joint_name}'")
        return np.clip(candidate, lower, upper)

    def _validate_streaming_limit_margin(
        self,
        context: _PinkRobotContext,
        margin: float,
    ) -> None:
        for joint_name, _q_index, lower, upper in _bounded_controlled_joint_limits(context):
            if lower + margin > upper - margin:
                raise ValueError(
                    f"Pink command limit margin {margin} leaves no valid range for "
                    f"controlled joint '{joint_name}' with limits [{lower}, {upper}]"
                )


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


def _bounded_controlled_joint_limits(
    context: _PinkRobotContext,
) -> list[tuple[str, int, float, float]]:
    lower_limits = np.asarray(context.model.lowerPositionLimit, dtype=np.float64)
    upper_limits = np.asarray(context.model.upperPositionLimit, dtype=np.float64)
    if lower_limits.shape != (context.model.nq,) or upper_limits.shape != (context.model.nq,):
        raise ValueError("Pink model position limits do not match its configuration dimension")

    bounded: list[tuple[str, int, float, float]] = []
    for joint_name, q_index in zip(
        context.mapping.dimos_joint_names,
        context.mapping.idx_q,
        strict=True,
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
