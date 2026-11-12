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
from dimos.teleop.leader.calibration import (
    DEFAULT_LEADER_MOTOR_NAMES,
    LeaderCalibration,
    LeaderCalibrationError,
    LeaderMotorCalibration,
    default_calibration_path,
    load_calibration,
    save_calibration,
)
from dimos.teleop.leader.feetech import LEADER_TELEOP_EXTRA, missing_dependency_error


def _valid_calibration(side: str = "left") -> LeaderCalibration:
    return LeaderCalibration(
        side=side,
        motors={
            motor_name: LeaderMotorCalibration(
                id=index + 1, homing_offset=100 + index, flip=index % 2 == 0
            )
            for index, motor_name in enumerate(DEFAULT_LEADER_MOTOR_NAMES)
        },
    )


def test_default_calibration_paths_live_under_the_leader_state_dir(tmp_path: Path) -> None:
    assert default_calibration_path("left") == STATE_DIR / "teleop" / "leader" / "left"
    assert default_calibration_path("right", tmp_path / "yam") == tmp_path / "yam" / "right"


def test_save_and_load_side_specific_calibration(tmp_path: Path) -> None:
    calibration = _valid_calibration("right")
    artifact_path = save_calibration(tmp_path / "right", calibration)

    assert artifact_path == tmp_path / "right" / "calibration.json"
    loaded = load_calibration(tmp_path / "right", "right")
    assert loaded == calibration
    assert loaded.motors["joint_2"].homing_offset == 101
    assert load_calibration(artifact_path, "right") == calibration


def test_load_accepts_the_motor_set_the_mapping_names(tmp_path: Path) -> None:
    save_calibration(tmp_path / "left", _valid_calibration())
    loaded = load_calibration(tmp_path / "left", "left", DEFAULT_LEADER_MOTOR_NAMES)
    assert set(loaded.motors) == set(DEFAULT_LEADER_MOTOR_NAMES)
    with pytest.raises(LeaderCalibrationError, match="extra=\\['joint_7'\\]"):
        load_calibration(tmp_path / "left", "left", DEFAULT_LEADER_MOTOR_NAMES[:6])
    with pytest.raises(LeaderCalibrationError, match="missing=\\['joint_8'\\]"):
        load_calibration(tmp_path / "left", "left", (*DEFAULT_LEADER_MOTOR_NAMES, "joint_8"))


def test_missing_calibration_error_mentions_calibration_utility(tmp_path: Path) -> None:
    with pytest.raises(LeaderCalibrationError, match="hardware feetech-leader calibrate"):
        load_calibration(tmp_path / "missing", "left")


def test_invalid_calibration_rejects_unknown_fields_and_bad_values() -> None:
    data = _valid_calibration().model_dump(mode="json")
    data["motors"]["joint_1"]["drive_mode"] = 0  # type: ignore[index]
    with pytest.raises(ValidationError, match="extra"):
        LeaderCalibration.model_validate(data)

    data = _valid_calibration().model_dump(mode="json")
    data["motors"]["joint_1"]["flip"] = 0  # type: ignore[index]
    with pytest.raises(ValidationError, match="flip"):
        LeaderCalibration.model_validate(data)

    with pytest.raises(LeaderCalibrationError, match="invalid id 0"):
        LeaderMotorCalibration(id=0, homing_offset=0)


def test_invalid_calibration_rejects_side_mismatch(tmp_path: Path) -> None:
    save_calibration(tmp_path / "left", _valid_calibration("right"))

    with pytest.raises(LeaderCalibrationError, match="side mismatch"):
        load_calibration(tmp_path / "left", "left")


def test_missing_dependency_error_names_optional_extra() -> None:
    error = missing_dependency_error()

    assert f"--extra {LEADER_TELEOP_EXTRA}" in str(error)
    assert "Feetech" in str(error)
