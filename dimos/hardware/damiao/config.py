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

from __future__ import annotations

from collections.abc import Sequence
import math
from pathlib import Path

import attrs

from dimos.hardware.manipulators.spec import ControlMode

_NON_EMPTY_STRING = attrs.validators.and_(
    attrs.validators.instance_of(str),
    attrs.validators.min_len(1),
)
_NON_NEGATIVE_INT = attrs.validators.and_(
    attrs.validators.instance_of(int),
    attrs.validators.ge(0),
)
_POSITIVE_INT = attrs.validators.and_(
    attrs.validators.instance_of(int),
    attrs.validators.ge(1),
)


def _to_floats(values: Sequence[float]) -> tuple[float, ...]:
    return tuple(float(value) for value in values)


def _to_optional_floats(values: Sequence[float] | None) -> tuple[float, ...] | None:
    return None if values is None else _to_floats(values)


def _to_motors(values: Sequence[DamiaoMotorConfig]) -> tuple[DamiaoMotorConfig, ...]:
    return tuple(values)


def _to_control_modes(values: Sequence[ControlMode]) -> tuple[ControlMode, ...]:
    return tuple(values)


def _to_optional_path(value: str | Path | None) -> Path | None:
    return None if value is None else Path(value)


def _finite_non_negative(
    _instance: object, attribute: attrs.Attribute[float], value: float
) -> None:
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(f"{attribute.name} must be finite and non-negative")


@attrs.frozen(slots=False)
class DamiaoMotorConfig:
    """Physical identity for one Damiao motor in command-vector order."""

    name: str = attrs.field(validator=_NON_EMPTY_STRING)
    type: str | int = attrs.field(validator=attrs.validators.instance_of((str, int)))
    send_id: int = attrs.field(validator=_NON_NEGATIVE_INT)
    recv_id: int | None = attrs.field(
        default=None,
        validator=attrs.validators.optional(_NON_NEGATIVE_INT),
    )

    @property
    def effective_recv_id(self) -> int:
        """Return the explicit receive CAN ID, or Damiao's default response ID."""

        return self.recv_id if self.recv_id is not None else (self.send_id | 0x10)


@attrs.frozen(slots=False)
class DamiaoArmConfig:
    """Immutable physical definition and capabilities for one Damiao arm."""

    name: str = attrs.field(validator=_NON_EMPTY_STRING)
    vendor: str = attrs.field(validator=_NON_EMPTY_STRING)
    model: str = attrs.field(validator=_NON_EMPTY_STRING)
    motors: tuple[DamiaoMotorConfig, ...] = attrs.field(
        converter=_to_motors,
        validator=attrs.validators.deep_iterable(
            member_validator=attrs.validators.instance_of(DamiaoMotorConfig),
            iterable_validator=attrs.validators.min_len(1),
        ),
    )
    position_lower: tuple[float, ...] = attrs.field(converter=_to_floats)
    position_upper: tuple[float, ...] = attrs.field(converter=_to_floats)
    velocity_max: tuple[float, ...] = attrs.field(converter=_to_floats)
    kp: tuple[float, ...] = attrs.field(converter=_to_floats)
    kd: tuple[float, ...] = attrs.field(converter=_to_floats)
    gravity_torque_limits: tuple[float, ...] | None = attrs.field(
        default=None,
        converter=_to_optional_floats,
    )
    fd: bool = attrs.field(default=False, validator=attrs.validators.instance_of(bool))
    supported_control_modes: tuple[ControlMode, ...] = attrs.field(
        factory=lambda: (
            ControlMode.POSITION,
            ControlMode.SERVO_POSITION,
            ControlMode.TORQUE,
        ),
        converter=_to_control_modes,
        validator=attrs.validators.deep_iterable(
            member_validator=attrs.validators.instance_of(ControlMode),
            iterable_validator=attrs.validators.min_len(1),
        ),
    )

    @property
    def dof(self) -> int:
        """Return the number of joints described by this arm."""

        return len(self.motors)

    @property
    def joint_names(self) -> tuple[str, ...]:
        """Return joint names in adapter and command-vector order."""

        return tuple(motor.name for motor in self.motors)

    @motors.validator
    def _validate_motor_identity(
        self,
        _attribute: attrs.Attribute[tuple[DamiaoMotorConfig, ...]],
        motors: tuple[DamiaoMotorConfig, ...],
    ) -> None:
        identities: dict[str, Sequence[str | int]] = {
            "joint names": [motor.name for motor in motors],
            "send IDs": [motor.send_id for motor in motors],
            "receive IDs": [motor.effective_recv_id for motor in motors],
        }
        for label, values in identities.items():
            if len(set(values)) != len(values):
                raise ValueError(f"Damiao arm {self.name!r} has duplicate {label}: {values}")

    @position_lower.validator
    def _validate_joint_vectors(
        self,
        _attribute: attrs.Attribute[tuple[float, ...]],
        _value: tuple[float, ...],
    ) -> None:
        vectors = {
            "position_lower": self.position_lower,
            "position_upper": self.position_upper,
            "velocity_max": self.velocity_max,
            "kp": self.kp,
            "kd": self.kd,
        }
        if self.gravity_torque_limits is not None:
            vectors["gravity_torque_limits"] = self.gravity_torque_limits
        for label, values in vectors.items():
            if len(values) != self.dof:
                raise ValueError(f"{label} length {len(values)} does not match arm DOF {self.dof}")
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f"{label} values must be finite")
        if any(
            lower > upper
            for lower, upper in zip(self.position_lower, self.position_upper, strict=True)
        ):
            raise ValueError("position lower limits must not exceed upper limits")
        if any(value <= 0.0 for value in self.velocity_max):
            raise ValueError("velocity limits must be greater than zero")
        if any(value < 0.0 for value in (*self.kp, *self.kd)):
            raise ValueError("default gains must be non-negative")
        if self.gravity_torque_limits is not None and any(
            value < 0.0 for value in self.gravity_torque_limits
        ):
            raise ValueError("gravity torque limits must be non-negative")

    @supported_control_modes.validator
    def _validate_control_modes(
        self,
        _attribute: attrs.Attribute[tuple[ControlMode, ...]],
        modes: tuple[ControlMode, ...],
    ) -> None:
        if len(set(modes)) != len(modes):
            raise ValueError("supported control modes must be unique")


@attrs.frozen(slots=False)
class DamiaoRuntimeConfig:
    """Deployment-specific settings and optional overrides for a Damiao arm."""

    address: str = attrs.field(default="can0", converter=str, validator=_NON_EMPTY_STRING)
    gravity_comp: bool = attrs.field(default=True, validator=attrs.validators.instance_of(bool))
    gravity_model_path: Path | None = attrs.field(default=None, converter=_to_optional_path)
    kp_override: tuple[float, ...] | None = attrs.field(
        default=None,
        converter=_to_optional_floats,
    )
    kd_override: tuple[float, ...] | None = attrs.field(
        default=None,
        converter=_to_optional_floats,
    )
    use_mock_bus: bool = attrs.field(default=False, validator=attrs.validators.instance_of(bool))
    config_path: Path | None = attrs.field(default=None, converter=_to_optional_path)
    tick_deadline_us: int = attrs.field(default=1_000, validator=_POSITIVE_INT)
    state_cache_ttl_s: float = attrs.field(
        default=0.002,
        converter=float,
        validator=_finite_non_negative,
    )

    @kp_override.validator
    @kd_override.validator
    def _validate_gain_override(
        self,
        attribute: attrs.Attribute[tuple[float, ...] | None],
        values: tuple[float, ...] | None,
    ) -> None:
        if values is not None and any(not math.isfinite(value) or value < 0.0 for value in values):
            raise ValueError(f"{attribute.name} values must be finite and non-negative")
