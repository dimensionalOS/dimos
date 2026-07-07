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

"""Configuration helpers for OpenArm Mini leader teleoperation."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

from dimos.constants import STATE_DIR
from dimos.robot.manipulators.openarm.config import openarm_joints

OPENARM_MINI_TELEOP_EXTRA = "openarm-mini-teleop"
OPENARM_MINI_STATE_DIR = STATE_DIR / "teleop" / "openarm_mini"
OPENARM_MINI_SIDES = ("left", "right")


def default_calibration_path(side: str) -> Path:
    """Return the default persistent calibration directory for an OpenArm Mini side."""
    validate_side(side)
    return OPENARM_MINI_STATE_DIR / side


def validate_side(side: str) -> None:
    """Validate an OpenArm Mini side string."""
    if side not in OPENARM_MINI_SIDES:
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")


@dataclass(frozen=True)
class OpenArmMiniTeleopConfig:
    """Runtime configuration for a bimanual OpenArm Mini leader.

    Runtime startup is intentionally non-interactive: calibration paths point to
    pre-existing side-specific calibration directories created by the manual
    calibration demo.
    """

    port_left: str = "/dev/ttyUSB1"
    port_right: str = "/dev/ttyUSB0"
    left_calibration_path: Path | None = None
    right_calibration_path: Path | None = None
    baudrate: int = 1_000_000
    max_joint_jump_radians: float = 0.75
    authority_active: bool = True
    enabled_sides: tuple[str, ...] = OPENARM_MINI_SIDES
    target_joint_names_by_side: Mapping[str, Sequence[str]] | None = None

    def __post_init__(self) -> None:
        """Validate selected OpenArm Mini leader sides."""
        if not self.enabled_sides:
            raise ValueError("enabled_sides must include at least one side")
        for side in self.enabled_sides:
            validate_side(side)
        if len(set(self.enabled_sides)) != len(self.enabled_sides):
            raise ValueError("enabled_sides must not contain duplicate sides")
        if self.target_joint_names_by_side is None:
            return
        for side, target_joint_names in self.target_joint_names_by_side.items():
            validate_side(side)
            if len(target_joint_names) != 7:
                raise ValueError(
                    f"target_joint_names_by_side[{side!r}] must contain exactly 7 names"
                )

    def calibration_path(self, side: str) -> Path:
        """Return the configured or default calibration directory for a side."""
        validate_side(side)
        if side == "left" and self.left_calibration_path is not None:
            return self.left_calibration_path
        if side == "right" and self.right_calibration_path is not None:
            return self.right_calibration_path
        return default_calibration_path(side)

    def port(self, side: str) -> str:
        """Return the configured serial port for a side."""
        validate_side(side)
        return self.port_left if side == "left" else self.port_right

    def sides(self) -> tuple[str, ...]:
        """Return the selected leader sides in runtime order."""
        return self.enabled_sides

    def target_joint_names(self, side: str) -> tuple[str, ...]:
        """Return the follower joint names emitted for a leader side."""
        validate_side(side)
        if self.target_joint_names_by_side is None:
            return tuple(openarm_joints(side))
        configured = self.target_joint_names_by_side.get(side)
        if configured is None:
            return tuple(openarm_joints(side))
        return tuple(configured)


class OpenArmMiniDependencyError(ImportError):
    """Raised when the optional Feetech SDK dependency is unavailable."""


class OpenArmMiniCalibrationError(RuntimeError):
    """Raised when OpenArm Mini calibration is missing or invalid."""


def missing_dependency_error() -> OpenArmMiniDependencyError:
    """Build the localized missing dependency error for OpenArm Mini teleop."""
    return OpenArmMiniDependencyError(
        "OpenArm Mini teleop requires the Feetech SDK. Install it with "
        "`uv sync --extra openarm`, "
        f"`uv sync --extra {OPENARM_MINI_TELEOP_EXTRA}`, or "
        f"`pip install 'dimos[{OPENARM_MINI_TELEOP_EXTRA}]'`."
    )
