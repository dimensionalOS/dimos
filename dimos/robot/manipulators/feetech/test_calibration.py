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

from pathlib import Path

from pydantic import ValidationError
import pytest

from dimos.constants import STATE_DIR
from dimos.robot.manipulators.feetech.calibration import (
    FeetechCalibration,
    FeetechCalibrationError,
    FeetechMotorCalibration,
    default_calibration_path,
    load_calibration,
    save_calibration,
)

MOTORS = tuple(f"joint_{index}" for index in range(1, 8))


def _valid_calibration(side: str = "left") -> FeetechCalibration:
    return FeetechCalibration(
        side=side,
        motors={
            motor_name: FeetechMotorCalibration(
                id=index + 1, homing_offset=100 + index, flip=index % 2 == 0
            )
            for index, motor_name in enumerate(MOTORS)
        },
    )


def test_default_calibration_paths_live_under_the_leader_state_dir(tmp_path: Path) -> None:
    assert default_calibration_path("left") == STATE_DIR / "teleop" / "leader" / "left"
    assert default_calibration_path("right", tmp_path / "yam") == tmp_path / "yam" / "right"


def test_save_and_load_side_specific_calibration(tmp_path: Path) -> None:
    calibration = _valid_calibration("right")
    artifact_path = save_calibration(tmp_path / "right", calibration)

    assert artifact_path == tmp_path / "right" / "calibration.json"
    loaded = load_calibration(tmp_path / "right", "right", MOTORS)
    assert loaded == calibration
    assert loaded.motors["joint_2"].homing_offset == 101
    assert load_calibration(artifact_path, "right", MOTORS) == calibration


def test_load_accepts_the_motor_set_the_mapping_names(tmp_path: Path) -> None:
    save_calibration(tmp_path / "left", _valid_calibration())
    loaded = load_calibration(tmp_path / "left", "left", MOTORS)
    assert set(loaded.motors) == set(MOTORS)
    with pytest.raises(FeetechCalibrationError, match="extra=\\['joint_7'\\]"):
        load_calibration(tmp_path / "left", "left", MOTORS[:6])
    with pytest.raises(FeetechCalibrationError, match="missing=\\['joint_8'\\]"):
        load_calibration(tmp_path / "left", "left", (*MOTORS, "joint_8"))


def test_invalid_calibration_rejects_unknown_fields_and_bad_values() -> None:
    data = _valid_calibration().model_dump(mode="json")
    data["motors"]["joint_1"]["drive_mode"] = 0  # type: ignore[index]
    with pytest.raises(ValidationError, match="extra"):
        FeetechCalibration.model_validate(data)

    data = _valid_calibration().model_dump(mode="json")
    data["motors"]["joint_1"]["flip"] = 0  # type: ignore[index]
    with pytest.raises(ValidationError, match="flip"):
        FeetechCalibration.model_validate(data)

    with pytest.raises(FeetechCalibrationError, match="invalid id 0"):
        FeetechMotorCalibration(id=0, homing_offset=0)


def test_invalid_calibration_rejects_side_mismatch(tmp_path: Path) -> None:
    save_calibration(tmp_path / "left", _valid_calibration("right"))

    with pytest.raises(FeetechCalibrationError, match="side mismatch"):
        load_calibration(tmp_path / "left", "left", MOTORS)
