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

"""Validated configuration for the Galaxea A1Z hardware adapter."""

from __future__ import annotations

import math
from pathlib import Path

import attrs

_A1Z_DOF = 6
_A1Z_DEFAULT_KP = (80.0, 80.0, 80.0, 50.0, 10.0, 10.0)
_A1Z_DEFAULT_KD = (3.0, 3.0, 3.0, 0.7, 0.2, 0.2)


def _joint_gains(
    value: tuple[float, ...] | list[float],
    *,
    name: str,
    maximum: float,
) -> tuple[float, ...]:
    gains = tuple(float(gain) for gain in value)
    if len(gains) != _A1Z_DOF:
        raise ValueError(f"{name} must contain {_A1Z_DOF} values")
    if any(not math.isfinite(gain) or not 0.0 <= gain <= maximum for gain in gains):
        raise ValueError(f"{name} gains must be finite and within [0, {maximum}]")
    return gains


def _kp_gains(value: tuple[float, ...] | list[float]) -> tuple[float, ...]:
    return _joint_gains(value, name="default_kp", maximum=200.0)


def _kd_gains(value: tuple[float, ...] | list[float]) -> tuple[float, ...]:
    return _joint_gains(value, name="default_kd", maximum=5.0)


def _validate_optional_path(
    _instance: object,
    attribute: attrs.Attribute[str | Path | None],
    value: str | Path | None,
) -> None:
    """Validate path config without resolving lazy Path subclasses."""
    if value is not None and not issubclass(type(value), (str, Path)):
        raise TypeError(
            f"'{attribute.name}' must be str, Path, or None (got {type(value).__name__})"
        )


@attrs.frozen(slots=False)
class A1ZGripperConfig:
    """G1Z gripper configuration."""

    max_torque: float = attrs.field(
        default=0.5,
        converter=float,
        validator=attrs.validators.gt(0.0),
    )
    max_opening_m: float = attrs.field(
        default=0.1,
        converter=float,
        validator=attrs.validators.gt(0.0),
    )
    max_velocity_rad_s: float = attrs.field(
        default=10.0,
        converter=float,
        validator=attrs.validators.and_(
            attrs.validators.gt(0.0),
            attrs.validators.le(100.0),
        ),
    )


@attrs.frozen(slots=False)
class A1ZTeachingConfig:
    """Free-drive behavior used while hand-teaching the arm."""

    gripper_free_drive: bool = attrs.field(
        default=False,
        validator=attrs.validators.instance_of(bool),
    )


@attrs.frozen(slots=False)
class A1ZConfig:
    """Runtime configuration passed to the six-axis A1Z adapter."""

    gravity_comp_factor: float = attrs.field(
        default=1.0,
        converter=float,
        validator=attrs.validators.and_(
            attrs.validators.ge(0.0),
            attrs.validators.le(1.0),
        ),
    )
    default_kp: tuple[float, ...] = attrs.field(
        default=_A1Z_DEFAULT_KP,
        converter=_kp_gains,
    )
    default_kd: tuple[float, ...] = attrs.field(
        default=_A1Z_DEFAULT_KD,
        converter=_kd_gains,
    )
    urdf_path: str | Path | None = attrs.field(
        default=None,
        validator=_validate_optional_path,
    )
    gripper: A1ZGripperConfig | None = attrs.field(
        default=None,
        validator=attrs.validators.optional(
            attrs.validators.instance_of(A1ZGripperConfig),
        ),
    )
    teaching: A1ZTeachingConfig | None = attrs.field(
        default=None,
        validator=attrs.validators.optional(
            attrs.validators.instance_of(A1ZTeachingConfig),
        ),
    )

    def __attrs_post_init__(self) -> None:
        if self.teaching and self.teaching.gripper_free_drive and self.gripper is None:
            raise ValueError("teaching.gripper_free_drive requires a configured gripper")
