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

from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import Any

from dimos.teleop.openarm_mini.config import OpenArmMiniCalibrationError, validate_side

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


@dataclass(frozen=True)
class OpenArmMiniMotorCalibration:
    """Calibration values for one arm-joint Feetech motor."""

    id: int
    homing_offset: int
    flip: bool = False


@dataclass(frozen=True)
class OpenArmMiniCalibration:
    """Side-specific OpenArm Mini calibration artifact."""

    side: str
    motors: dict[str, OpenArmMiniMotorCalibration]
    schema_version: int = 1

    def __post_init__(self) -> None:
        validate_side(self.side)
        if self.schema_version != 1:
            raise OpenArmMiniCalibrationError(
                f"unsupported OpenArm Mini calibration schema_version {self.schema_version}"
            )
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

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> OpenArmMiniCalibration:
        """Build a calibration artifact from decoded JSON data."""
        side = data.get("side")
        motors_data = data.get("motors")
        schema_version = data.get("schema_version", 1)
        if not isinstance(side, str):
            raise OpenArmMiniCalibrationError("calibration side must be a string")
        if not isinstance(schema_version, int):
            raise OpenArmMiniCalibrationError("calibration schema_version must be an integer")
        if not isinstance(motors_data, dict):
            raise OpenArmMiniCalibrationError("calibration motors must be an object")

        motors: dict[str, OpenArmMiniMotorCalibration] = {}
        for motor_name, motor_data in motors_data.items():
            if not isinstance(motor_name, str) or not isinstance(motor_data, dict):
                raise OpenArmMiniCalibrationError("calibration motor entries must be objects")
            motors[motor_name] = _motor_calibration_from_dict(motor_name, motor_data)
        return cls(side=side, motors=motors, schema_version=schema_version)

    def to_dict(self) -> dict[str, object]:
        """Return a JSON-serializable representation."""
        return asdict(self)


def _motor_calibration_from_dict(
    motor_name: str, data: dict[Any, Any]
) -> OpenArmMiniMotorCalibration:
    required_fields = {"id", "homing_offset", "flip"}
    missing = required_fields - set(data)
    extra = set(data) - required_fields
    if missing or extra:
        raise OpenArmMiniCalibrationError(
            f"{motor_name} must contain only {sorted(required_fields)}; "
            f"missing={sorted(missing)}, extra={sorted(extra)}"
        )
    motor_id = data["id"]
    homing_offset = data["homing_offset"]
    flip = data["flip"]
    if not isinstance(motor_id, int) or isinstance(motor_id, bool):
        raise OpenArmMiniCalibrationError(f"{motor_name}.id must be an integer")
    if not isinstance(homing_offset, int) or isinstance(homing_offset, bool):
        raise OpenArmMiniCalibrationError(f"{motor_name}.homing_offset must be an integer")
    if not isinstance(flip, bool):
        raise OpenArmMiniCalibrationError(f"{motor_name}.flip must be a boolean")
    return OpenArmMiniMotorCalibration(
        id=motor_id,
        homing_offset=homing_offset,
        flip=flip,
    )


def calibration_file(path: Path) -> Path:
    """Resolve a calibration directory or file to the artifact file path."""
    if path.suffix == ".json":
        return path
    return path / CALIBRATION_FILENAME


def load_calibration(path: Path, side: str) -> OpenArmMiniCalibration:
    """Load and validate a side-specific calibration artifact."""
    validate_side(side)
    artifact_path = calibration_file(path)
    if not artifact_path.exists():
        raise OpenArmMiniCalibrationError(
            f"Missing OpenArm Mini {side} calibration at {artifact_path}. "
            "Run `python -m dimos.teleop.openarm_mini.demo_calibrate_openarm_mini` "
            "to create calibration artifacts before starting teleop."
        )
    try:
        raw_data = json.loads(artifact_path.read_text())
    except json.JSONDecodeError as exc:
        raise OpenArmMiniCalibrationError(
            f"Invalid OpenArm Mini {side} calibration JSON at {artifact_path}: {exc}"
        ) from exc
    if not isinstance(raw_data, dict):
        raise OpenArmMiniCalibrationError(
            f"Invalid OpenArm Mini {side} calibration at {artifact_path}: expected JSON object"
        )
    calibration = OpenArmMiniCalibration.from_dict(raw_data)
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
    artifact_path.write_text(json.dumps(calibration.to_dict(), indent=2, sort_keys=True) + "\n")
    return artifact_path
