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

"""Calibration artifacts for Feetech leader arms: one file per leader side."""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path
import sys
from typing import Literal

if sys.version_info >= (3, 11):
    from typing import Self
else:
    from typing_extensions import Self

from pydantic import StrictBool, StrictInt, ValidationError, model_validator

from dimos.constants import STATE_DIR
from dimos.protocol.service.spec import BaseConfig

CALIBRATION_FILENAME = "calibration.json"
FEETECH_RAW_MIN = 0
FEETECH_RAW_MAX = 4095
FEETECH_POSITION_SPAN = FEETECH_RAW_MAX - FEETECH_RAW_MIN
LEADER_STATE_DIR = STATE_DIR / "teleop" / "leader"
LeaderSide = Literal["left", "right"]
LEADER_SIDES: tuple[LeaderSide, ...] = ("left", "right")
# Both leader arms built so far carry seven Feetech motors, numbered along the arm.
DEFAULT_LEADER_MOTOR_NAMES: tuple[str, ...] = tuple(f"joint_{index}" for index in range(1, 8))
# The trigger servo a leader may carry for its gripper; absent on arm-only leaders.
LEADER_GRIPPER_MOTOR_NAME = "gripper"


def default_calibration_path(side: LeaderSide, root: Path = LEADER_STATE_DIR) -> Path:
    """The calibration directory for a leader side under a per-leader root."""
    return root / side


class LeaderCalibrationError(RuntimeError):
    """Raised when a leader calibration is missing or invalid."""


class LeaderMotorCalibration(BaseConfig):
    """Calibration values for one Feetech motor."""

    id: StrictInt
    homing_offset: StrictInt
    flip: StrictBool = False

    @model_validator(mode="after")
    def _validate_motor(self) -> Self:
        if self.id <= 0:
            raise LeaderCalibrationError(f"motor has invalid id {self.id}")
        return self


class LeaderCalibration(BaseConfig):
    """Side-specific leader calibration artifact."""

    side: LeaderSide
    motors: dict[str, LeaderMotorCalibration]
    schema_version: Literal[1] = 1

    def require_motors(self, motor_names: Sequence[str]) -> Self:
        """Check that exactly the given motors are calibrated."""
        missing = set(motor_names) - set(self.motors)
        extra = set(self.motors) - set(motor_names)
        if missing or extra:
            raise LeaderCalibrationError(
                f"leader calibration must contain exactly the motors {list(motor_names)}; "
                f"missing={sorted(missing)}, extra={sorted(extra)}"
            )
        return self


def calibration_file(path: Path) -> Path:
    """Resolve a calibration directory or file to the artifact file path."""
    if path.suffix == ".json":
        return path
    return path / CALIBRATION_FILENAME


def load_calibration(
    path: Path, side: LeaderSide, motor_names: Sequence[str] = DEFAULT_LEADER_MOTOR_NAMES
) -> LeaderCalibration:
    """Load and validate a side-specific calibration artifact."""
    artifact_path = calibration_file(path)
    if not artifact_path.exists():
        raise LeaderCalibrationError(
            f"Missing {side} leader calibration at {artifact_path}. "
            "Run `dimos hardware feetech-leader calibrate` first."
        )
    try:
        calibration = LeaderCalibration.model_validate_json(artifact_path.read_text())
        calibration.require_motors(motor_names)
    except (LeaderCalibrationError, ValidationError, ValueError) as exc:
        raise LeaderCalibrationError(
            f"Invalid {side} leader calibration at {artifact_path}: {exc}"
        ) from exc
    if calibration.side != side:
        raise LeaderCalibrationError(
            f"Leader calibration side mismatch at {artifact_path}: "
            f"expected {side!r}, got {calibration.side!r}"
        )
    return calibration


def save_calibration(path: Path, calibration: LeaderCalibration) -> Path:
    """Write a side-specific calibration artifact and return its file path."""
    artifact_path = calibration_file(path)
    artifact_path.parent.mkdir(parents=True, exist_ok=True)
    artifact_path.write_text(calibration.model_dump_json(indent=2) + "\n")
    return artifact_path
