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

"""OpenArm Mini calibration artifact loading and validation."""

from __future__ import annotations

from pathlib import Path
from typing import Literal, Self

from pydantic import StrictBool, StrictInt, ValidationError, model_validator

from dimos.protocol.service.spec import BaseConfig
from dimos.teleop.openarm_mini.config import OpenArmMiniCalibrationError, OpenArmMiniSide

CALIBRATION_FILENAME = "calibration.json"
FEETECH_RAW_MIN = 0
FEETECH_RAW_MAX = 4095
FEETECH_POSITION_SPAN = FEETECH_RAW_MAX - FEETECH_RAW_MIN
OPENARM_MINI_ARM_JOINT_NAMES = (
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
    "joint_7",
)
OPENARM_MINI_MOTOR_NAMES = OPENARM_MINI_ARM_JOINT_NAMES


class OpenArmMiniMotorCalibration(BaseConfig):
    """Calibration values for one arm-joint Feetech motor."""

    id: StrictInt
    homing_offset: StrictInt
    flip: StrictBool = False

    @model_validator(mode="after")
    def _validate_motor(self) -> Self:
        if self.id <= 0:
            raise OpenArmMiniCalibrationError(f"motor has invalid id {self.id}")
        return self


class OpenArmMiniCalibration(BaseConfig):
    """Side-specific OpenArm Mini calibration artifact."""

    side: OpenArmMiniSide
    motors: dict[str, OpenArmMiniMotorCalibration]
    schema_version: Literal[1] = 1

    @model_validator(mode="after")
    def _validate_calibration(self) -> Self:
        missing = set(OPENARM_MINI_ARM_JOINT_NAMES) - set(self.motors)
        extra = set(self.motors) - set(OPENARM_MINI_ARM_JOINT_NAMES)
        if missing or extra:
            raise OpenArmMiniCalibrationError(
                "OpenArm Mini calibration must contain exactly arm joints "
                f"{list(OPENARM_MINI_ARM_JOINT_NAMES)}; missing={sorted(missing)}, extra={sorted(extra)}"
            )
        for motor_name, motor in self.motors.items():
            if motor.id <= 0:
                raise OpenArmMiniCalibrationError(f"{motor_name} has invalid id {motor.id}")
        return self


def calibration_file(path: Path) -> Path:
    """Resolve a calibration directory or file to the artifact file path."""
    if path.suffix == ".json":
        return path
    return path / CALIBRATION_FILENAME


def load_calibration(path: Path, side: OpenArmMiniSide) -> OpenArmMiniCalibration:
    """Load and validate a side-specific calibration artifact."""
    artifact_path = calibration_file(path)
    if not artifact_path.exists():
        raise OpenArmMiniCalibrationError(
            f"Missing OpenArm Mini {side} calibration at {artifact_path}. "
            "Run `python -m dimos.teleop.openarm_mini.tools.calibrate` "
            "to create calibration artifacts before starting teleop."
        )
    try:
        calibration = OpenArmMiniCalibration.model_validate_json(artifact_path.read_text())
    except (OpenArmMiniCalibrationError, ValidationError, ValueError) as exc:
        raise OpenArmMiniCalibrationError(
            f"Invalid OpenArm Mini {side} calibration at {artifact_path}: {exc}"
        ) from exc
    if calibration.side != side:
        raise OpenArmMiniCalibrationError(
            f"OpenArm Mini calibration side mismatch at {artifact_path}: "
            f"expected {side!r}, got {calibration.side!r}"
        )
    return calibration


def save_calibration(path: Path, calibration: OpenArmMiniCalibration) -> Path:
    """Write a side-specific calibration artifact and return its file path."""
    artifact_path = calibration_file(path)
    artifact_path.parent.mkdir(parents=True, exist_ok=True)
    artifact_path.write_text(calibration.model_dump_json(indent=2) + "\n")
    return artifact_path
