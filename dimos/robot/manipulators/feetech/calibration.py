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

"""Feetech calibration files: each motor's ID, zero offset and direction, one file per bus."""

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
# Where the leader arms' calibration files have always lived.
FEETECH_CALIBRATION_ROOT = STATE_DIR / "teleop" / "leader"


def default_calibration_path(bus: str, root: Path = FEETECH_CALIBRATION_ROOT) -> Path:
    """The calibration directory for one bus, e.g. ``<root>/left``."""
    return root / bus


class FeetechCalibrationError(RuntimeError):
    """Raised when a calibration file is missing or invalid."""


class FeetechMotorCalibration(BaseConfig):
    """Calibration values for one Feetech motor.

    Attributes:
        id: The motor's ID on its bus.
        homing_offset: The raw reading at the joint's zero, in encoder ticks
            (0 to 4095 per turn).
        flip: True for a motor that reads backwards on this arm.
    """

    id: StrictInt
    homing_offset: StrictInt
    flip: StrictBool = False

    @model_validator(mode="after")
    def _validate_motor(self) -> Self:
        if self.id <= 0:
            raise FeetechCalibrationError(f"motor has invalid id {self.id}")
        return self


class FeetechCalibration(BaseConfig):
    """One bus's calibration file.

    Attributes:
        side: The bus name the file belongs to, e.g. "left".
        motors: Calibration per motor name.
    """

    side: str
    motors: dict[str, FeetechMotorCalibration]
    schema_version: Literal[1] = 1

    def require_motors(self, motor_names: Sequence[str]) -> Self:
        """Check that exactly the given motors are calibrated."""
        missing = set(motor_names) - set(self.motors)
        extra = set(self.motors) - set(motor_names)
        if missing or extra:
            raise FeetechCalibrationError(
                f"Feetech calibration must contain exactly the motors {list(motor_names)}; "
                f"missing={sorted(missing)}, extra={sorted(extra)}"
            )
        return self


def calibration_file(path: Path) -> Path:
    """Resolve a calibration directory or file to the artifact file path."""
    if path.suffix == ".json":
        return path
    return path / CALIBRATION_FILENAME


def load_calibration(path: Path, side: str, motor_names: Sequence[str]) -> FeetechCalibration:
    """Read one bus's calibration file and check it covers exactly ``motor_names``.

    Args:
        path: The calibration file, or the directory holding ``calibration.json``.
        side: The bus name the file must be for, e.g. "left".
        motor_names: The motors the bus reads.
    """
    artifact_path = calibration_file(path)
    if not artifact_path.exists():
        raise FeetechCalibrationError(f"Missing {side} Feetech calibration at {artifact_path}.")
    try:
        calibration = FeetechCalibration.model_validate_json(artifact_path.read_text())
        calibration.require_motors(motor_names)
    except (FeetechCalibrationError, ValidationError, ValueError) as exc:
        raise FeetechCalibrationError(
            f"Invalid {side} Feetech calibration at {artifact_path}: {exc}"
        ) from exc
    if calibration.side != side:
        raise FeetechCalibrationError(
            f"Feetech calibration side mismatch at {artifact_path}: "
            f"expected {side!r}, got {calibration.side!r}"
        )
    return calibration


def save_calibration(path: Path, calibration: FeetechCalibration) -> Path:
    """Write one bus's calibration file and return its path."""
    artifact_path = calibration_file(path)
    artifact_path.parent.mkdir(parents=True, exist_ok=True)
    artifact_path.write_text(calibration.model_dump_json(indent=2) + "\n")
    return artifact_path
