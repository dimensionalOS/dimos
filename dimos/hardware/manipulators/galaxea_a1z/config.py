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

from pathlib import Path

from pydantic import ConfigDict, Field, field_validator, model_validator
from pydantic.dataclasses import dataclass as pydantic_dataclass
from typing_extensions import Self

_CONFIG = ConfigDict(extra="forbid", validate_default=True)


@pydantic_dataclass(frozen=True, config=_CONFIG)
class A1ZGripperConfig:
    """A1Z gripper configuration."""

    max_torque: float = Field(default=0.5, gt=0.0)
    max_opening_m: float = Field(default=0.1, gt=0.0)


@pydantic_dataclass(frozen=True, config=_CONFIG)
class A1ZTeachingConfig:
    """Free-drive behavior used while hand-teaching the arm."""

    gripper_free_drive: bool = Field(default=False, strict=True)


@pydantic_dataclass(frozen=True, config=_CONFIG)
class A1ZConfig:
    """Runtime configuration passed to the six-axis A1Z adapter."""

    gravity_comp_factor: float = Field(default=1.0, ge=0.0, le=1.0)
    default_kp: tuple[float, ...] = (80.0, 80.0, 80.0, 50.0, 20.0, 20.0)
    default_kd: tuple[float, ...] = (3.0, 3.0, 3.0, 0.7, 0.4, 0.4)
    urdf_path: str | Path | None = None
    gripper: A1ZGripperConfig | None = None
    teaching: A1ZTeachingConfig | None = None

    @field_validator("urdf_path", mode="before")
    @classmethod
    def _validate_optional_path(cls, value: object) -> object:
        """Validate path config without resolving lazy Path subclasses."""
        if value is not None and not issubclass(type(value), (str, Path)):
            raise TypeError(f"'urdf_path' must be str, Path, or None (got {type(value).__name__})")
        return value

    @model_validator(mode="after")
    def _validate_teaching(self) -> Self:
        if self.teaching and self.teaching.gripper_free_drive and self.gripper is None:
            raise ValueError("teaching.gripper_free_drive requires a configured gripper")
        return self
