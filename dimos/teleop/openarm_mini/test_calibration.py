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

from __future__ import annotations

from pathlib import Path

import pytest

from dimos.constants import STATE_DIR
from dimos.teleop.openarm_mini.calibration import (
    OPENARM_MINI_ARM_JOINT_NAMES,
    OpenArmMiniCalibration,
    OpenArmMiniMotorCalibration,
    load_calibration,
    save_calibration,
)
from dimos.teleop.openarm_mini.config import (
    OpenArmMiniCalibrationError,
    OpenArmMiniTeleopConfig,
    default_calibration_path,
    missing_dependency_error,
)


def _valid_calibration(side: str = "left") -> OpenArmMiniCalibration:
    return OpenArmMiniCalibration(
        side=side,
        motors={
            motor_name: OpenArmMiniMotorCalibration(
                id=index + 1,
                homing_offset=100 + index,
                flip=index % 2 == 0,
            )
            for index, motor_name in enumerate(OPENARM_MINI_ARM_JOINT_NAMES)
        },
    )


def test_default_calibration_paths_use_dimos_state_dir() -> None:
    config = OpenArmMiniTeleopConfig()

    assert default_calibration_path("left") == STATE_DIR / "teleop" / "openarm_mini" / "left"
    assert config.calibration_path("right") == STATE_DIR / "teleop" / "openarm_mini" / "right"


def test_explicit_calibration_paths_override_defaults(tmp_path: Path) -> None:
    left_path = tmp_path / "left-cal"
    config = OpenArmMiniTeleopConfig(left_calibration_path=left_path)

    assert config.calibration_path("left") == left_path
    assert config.calibration_path("right") == STATE_DIR / "teleop" / "openarm_mini" / "right"


def test_save_and_load_side_specific_calibration(tmp_path: Path) -> None:
    calibration = _valid_calibration("right")

    artifact_path = save_calibration(tmp_path / "right", calibration)
    loaded = load_calibration(tmp_path / "right", "right")

    assert artifact_path == tmp_path / "right" / "calibration.json"
    assert loaded == calibration
    assert set(loaded.motors) == set(OPENARM_MINI_ARM_JOINT_NAMES)
    assert "gripper" not in loaded.motors


def test_missing_calibration_error_mentions_demo_script(tmp_path: Path) -> None:
    with pytest.raises(OpenArmMiniCalibrationError, match="demo_calibrate_openarm_mini"):
        load_calibration(tmp_path / "missing", "left")


def test_invalid_calibration_rejects_missing_motor() -> None:
    motors = _valid_calibration().motors.copy()
    del motors["joint_7"]

    with pytest.raises(OpenArmMiniCalibrationError, match="missing"):
        OpenArmMiniCalibration(side="left", motors=motors)


def test_invalid_calibration_rejects_gripper_or_legacy_fields() -> None:
    motors = _valid_calibration().motors.copy()
    motors["gripper"] = OpenArmMiniMotorCalibration(id=8, homing_offset=2048, flip=False)

    with pytest.raises(OpenArmMiniCalibrationError, match="extra"):
        OpenArmMiniCalibration(side="left", motors=motors)

    data = _valid_calibration().to_dict()
    data["motors"]["joint_1"]["drive_mode"] = 0  # type: ignore[index]

    with pytest.raises(OpenArmMiniCalibrationError, match="extra"):
        OpenArmMiniCalibration.from_dict(data)  # type: ignore[arg-type]


def test_invalid_calibration_rejects_non_bool_flip() -> None:
    data = _valid_calibration().to_dict()
    data["motors"]["joint_1"]["flip"] = 0  # type: ignore[index]

    with pytest.raises(OpenArmMiniCalibrationError, match="flip"):
        OpenArmMiniCalibration.from_dict(data)  # type: ignore[arg-type]


def test_invalid_calibration_rejects_side_mismatch(tmp_path: Path) -> None:
    save_calibration(tmp_path / "left", _valid_calibration("right"))

    with pytest.raises(OpenArmMiniCalibrationError, match="side mismatch"):
        load_calibration(tmp_path / "left", "left")


def test_missing_dependency_error_names_optional_extra() -> None:
    error = missing_dependency_error()

    assert "openarm-mini-teleop" in str(error)
    assert "Feetech" in str(error)
