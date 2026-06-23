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

"""Joint-space compliance transform for position-servo references."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any

from dimos.control.components import JointName
from dimos.control.task import (
    BaseControlTask,
    ControlMode,
    CoordinatorState,
    JointCommandOutput,
    ResourceClaim,
)
from dimos.protocol.service.spec import BaseConfig
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class JointComplianceDiagnostics:
    """Inspectable compliance state for tests and runtime diagnostics."""

    offsets: dict[JointName, float] = field(default_factory=dict)
    offset_velocities: dict[JointName, float] = field(default_factory=dict)
    saturated: dict[JointName, bool] = field(default_factory=dict)
    feedback_mode: str = "position_error"
    last_safe_action: str = "none"


@dataclass
class JointComplianceTaskConfig:
    """Configuration for a joint-space compliance task.

    Scalar gains apply to all joints. Dict gains override individual joints.
    Effort feedback is opt-in because several adapters expose placeholder zeros.
    """

    joint_names: list[JointName]
    priority: int = 10
    mass: float | dict[JointName, float] = 1.0
    damping: float | dict[JointName, float] = 8.0
    stiffness: float | dict[JointName, float] = 30.0
    max_offset: float | dict[JointName, float] = 0.15
    max_offset_velocity: float | dict[JointName, float] = 0.5
    deadband: float | dict[JointName, float] = 0.0
    use_effort_feedback: bool = False
    effort_gain: float | dict[JointName, float] = 1.0


class JointComplianceTask(BaseControlTask):
    """Transform nominal joint references into bounded compliant commands."""

    def __init__(self, name: str, config: JointComplianceTaskConfig) -> None:
        if not config.joint_names:
            raise ValueError(f"JointComplianceTask '{name}' requires at least one joint")
        self._name = name
        self._config = config
        self._joint_names = frozenset(config.joint_names)
        self._joint_names_list = list(config.joint_names)
        self._active = False
        self._offset = {joint: 0.0 for joint in self._joint_names_list}
        self._offset_velocity = {joint: 0.0 for joint in self._joint_names_list}
        self._diagnostics = JointComplianceDiagnostics(
            offsets=dict(self._offset),
            offset_velocities=dict(self._offset_velocity),
            saturated={joint: False for joint in self._joint_names_list},
            feedback_mode="effort" if config.use_effort_feedback else "position_error",
        )

        self._mass = self._resolve_positive_values(config.mass, "mass")
        self._damping = self._resolve_nonnegative_values(config.damping, "damping")
        self._stiffness = self._resolve_nonnegative_values(config.stiffness, "stiffness")
        self._max_offset = self._resolve_nonnegative_values(config.max_offset, "max_offset")
        self._max_offset_velocity = self._resolve_nonnegative_values(
            config.max_offset_velocity,
            "max_offset_velocity",
        )
        self._deadband = self._resolve_nonnegative_values(config.deadband, "deadband")
        self._effort_gain = self._resolve_values(config.effort_gain, "effort_gain")

        logger.info(f"JointComplianceTask {name} initialized for joints: {config.joint_names}")

    @property
    def name(self) -> str:
        """Unique task identifier."""
        return self._name

    def claim(self) -> ResourceClaim:
        """Declare position-servo resources for standalone or composed use."""
        return ResourceClaim(
            joints=self._joint_names,
            priority=self._config.priority,
            mode=ControlMode.SERVO_POSITION,
        )

    def is_active(self) -> bool:
        """Standalone compliance does not emit commands until explicitly started."""
        return self._active

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        """Standalone mode holds current joint positions with compliance reset."""
        if not self._active:
            return None
        positions: list[float] = []
        for joint in self._joint_names_list:
            position = state.joints.get_position(joint)
            if position is None:
                self._set_safe_action("missing_state")
                return None
            positions.append(position)
        reference = JointCommandOutput(
            joint_names=self._joint_names_list,
            positions=positions,
            mode=ControlMode.SERVO_POSITION,
        )
        return self.compute_from_reference(state, reference)

    def compute_from_reference(
        self,
        state: CoordinatorState,
        reference: JointCommandOutput,
    ) -> JointCommandOutput | None:
        """Apply bounded admittance-style shaping to a position reference."""
        if reference.mode != ControlMode.SERVO_POSITION or reference.positions is None:
            self._set_safe_action("incompatible_reference")
            return None
        if reference.joint_names != self._joint_names_list:
            self._set_safe_action("incompatible_joints")
            return None
        if not math.isfinite(state.dt) or state.dt <= 0.0:
            self._set_safe_action("invalid_dt")
            return reference
        if any(not math.isfinite(q_ref) for q_ref in reference.positions):
            self._set_safe_action("invalid_reference")
            return None

        q_cmd: list[float] = []
        next_offset: dict[JointName, float] = {}
        next_offset_velocity: dict[JointName, float] = {}
        saturated: dict[JointName, bool] = {}
        for joint, q_ref in zip(self._joint_names_list, reference.positions, strict=True):
            q_actual = state.joints.get_position(joint)
            if q_actual is None:
                self._set_safe_action("missing_state")
                return None
            if not math.isfinite(q_actual):
                self._set_safe_action("invalid_state")
                return None
            signal = self._feedback_signal(joint, q_actual, q_ref, state)
            if signal is None:
                return None

            velocity = self._offset_velocity[joint]
            offset = self._offset[joint]
            if not math.isfinite(velocity) or not math.isfinite(offset):
                self.reset()
                self._set_safe_action("invalid_compliance_state")
                return None
            acceleration = (
                signal / self._mass[joint]
                - self._damping[joint] * velocity
                - self._stiffness[joint] * offset
            )
            if not math.isfinite(acceleration):
                self._set_safe_action("invalid_acceleration")
                return None

            velocity = self._clamp(
                velocity + acceleration * state.dt,
                -self._max_offset_velocity[joint],
                self._max_offset_velocity[joint],
            )
            offset = offset + velocity * state.dt
            if not math.isfinite(velocity) or not math.isfinite(offset):
                self._set_safe_action("invalid_offset")
                return None
            clamped_offset = self._clamp(
                offset,
                -self._max_offset[joint],
                self._max_offset[joint],
            )
            is_saturated = (
                clamped_offset != offset or abs(velocity) >= self._max_offset_velocity[joint]
            )

            next_offset[joint] = clamped_offset
            next_offset_velocity[joint] = velocity
            saturated[joint] = is_saturated
            command = q_ref + clamped_offset
            if not math.isfinite(command):
                self._set_safe_action("invalid_output")
                return None
            q_cmd.append(command)

        self._offset.update(next_offset)
        self._offset_velocity.update(next_offset_velocity)
        self._diagnostics = JointComplianceDiagnostics(
            offsets=dict(self._offset),
            offset_velocities=dict(self._offset_velocity),
            saturated=saturated,
            feedback_mode="effort" if self._config.use_effort_feedback else "position_error",
            last_safe_action="none",
        )
        return JointCommandOutput(
            joint_names=list(reference.joint_names),
            positions=q_cmd,
            mode=ControlMode.SERVO_POSITION,
        )

    def on_preempted(self, by_task: str, joints: frozenset[JointName]) -> None:
        """Reset compliance state if a higher-priority task takes any joint."""
        if joints & self._joint_names:
            logger.warning(
                f"JointComplianceTask {self._name} preempted by {by_task} on joints {joints}"
            )
            self.reset()

    def start(self) -> None:
        """Activate standalone hold behavior."""
        self._active = True

    def stop(self) -> None:
        """Deactivate standalone behavior and reset offsets."""
        self._active = False
        self.reset()

    def reset(self) -> None:
        """Reset all offsets, velocities, and saturation diagnostics."""
        for joint in self._joint_names_list:
            self._offset[joint] = 0.0
            self._offset_velocity[joint] = 0.0
        self._diagnostics = JointComplianceDiagnostics(
            offsets=dict(self._offset),
            offset_velocities=dict(self._offset_velocity),
            saturated={joint: False for joint in self._joint_names_list},
            feedback_mode="effort" if self._config.use_effort_feedback else "position_error",
            last_safe_action="reset",
        )

    def get_diagnostics(self) -> JointComplianceDiagnostics:
        """Return a copy of the latest diagnostics."""
        return JointComplianceDiagnostics(
            offsets=dict(self._diagnostics.offsets),
            offset_velocities=dict(self._diagnostics.offset_velocities),
            saturated=dict(self._diagnostics.saturated),
            feedback_mode=self._diagnostics.feedback_mode,
            last_safe_action=self._diagnostics.last_safe_action,
        )

    def _feedback_signal(
        self,
        joint: JointName,
        q_actual: float,
        q_ref: float,
        state: CoordinatorState,
    ) -> float | None:
        if self._config.use_effort_feedback:
            effort = state.joints.get_effort(joint)
            if effort is None:
                return 0.0
            if not math.isfinite(effort):
                self._set_safe_action("invalid_effort")
                return None
            signal = self._effort_gain[joint] * effort
        else:
            signal = q_actual - q_ref
        if not math.isfinite(signal):
            self._set_safe_action("invalid_feedback")
            return None

        deadband = self._deadband[joint]
        if abs(signal) <= deadband:
            return 0.0
        return signal - math.copysign(deadband, signal)

    def _set_safe_action(self, action: str) -> None:
        self._diagnostics = JointComplianceDiagnostics(
            offsets=dict(self._offset),
            offset_velocities=dict(self._offset_velocity),
            saturated=dict(self._diagnostics.saturated),
            feedback_mode="effort" if self._config.use_effort_feedback else "position_error",
            last_safe_action=action,
        )

    def _resolve_positive_values(
        self,
        values: float | dict[JointName, float],
        label: str,
    ) -> dict[JointName, float]:
        resolved = self._resolve_values(values, label)
        for joint, value in resolved.items():
            if value <= 0.0:
                raise ValueError(
                    f"JointComplianceTask '{self._name}' {label} for {joint} must be > 0"
                )
        return resolved

    def _resolve_nonnegative_values(
        self,
        values: float | dict[JointName, float],
        label: str,
    ) -> dict[JointName, float]:
        resolved = self._resolve_values(values, label)
        for joint, value in resolved.items():
            if value < 0.0:
                raise ValueError(
                    f"JointComplianceTask '{self._name}' {label} for {joint} must be >= 0"
                )
        return resolved

    def _resolve_values(
        self,
        values: float | dict[JointName, float],
        label: str,
    ) -> dict[JointName, float]:
        if isinstance(values, dict):
            missing = [joint for joint in self._joint_names_list if joint not in values]
            if missing:
                raise ValueError(
                    f"JointComplianceTask '{self._name}' {label} missing joints: {missing}"
                )
            resolved = {joint: float(values[joint]) for joint in self._joint_names_list}
        else:
            resolved = {joint: float(values) for joint in self._joint_names_list}
        for joint, value in resolved.items():
            if not math.isfinite(value):
                raise ValueError(
                    f"JointComplianceTask '{self._name}' {label} for {joint} must be finite"
                )
        return resolved

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return min(max(value, lower), upper)


class JointComplianceTaskParams(BaseConfig):
    mass: float | dict[str, float] | None = None
    damping: float | dict[str, float] | None = None
    stiffness: float | dict[str, float] | None = None
    max_offset: float | dict[str, float] | None = None
    max_offset_velocity: float | dict[str, float] | None = None
    deadband: float | dict[str, float] | None = None
    use_effort_feedback: bool = False
    effort_gain: float | dict[str, float] | None = None


def create_task(cfg: Any, hardware: Any) -> JointComplianceTask:
    params = JointComplianceTaskParams.model_validate(cfg.params)
    kwargs: dict[str, object] = {
        "joint_names": cfg.joint_names,
        "priority": cfg.priority,
        "use_effort_feedback": params.use_effort_feedback,
    }
    for key in (
        "mass",
        "damping",
        "stiffness",
        "max_offset",
        "max_offset_velocity",
        "deadband",
        "effort_gain",
    ):
        value = getattr(params, key)
        if value is not None:
            kwargs[key] = value
    return JointComplianceTask(cfg.name, JointComplianceTaskConfig(**kwargs))  # type: ignore[arg-type]
