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

from dimos.teleop.openarm_mini.calibration import OPENARM_MINI_ARM_JOINT_NAMES, load_calibration
from dimos.teleop.openarm_mini.tools.calibrate import (
    DEFAULT_FLIPS_BY_SIDE,
    _calibrate_side,
    _capture_zero_calibration,
    _format_calibration_confirmation,
    _parse_flip_overrides,
)


def _raw_positions(value: int) -> dict[str, int]:
    return {joint: value for joint in OPENARM_MINI_ARM_JOINT_NAMES}


def test_zero_capture_records_offsets_and_flips() -> None:
    raw_positions = _raw_positions(2048)
    raw_positions["joint_6"] = 1234

    calibration = _capture_zero_calibration("left", raw_positions, {"joint_1", "joint_6"})

    assert set(calibration.motors) == set(OPENARM_MINI_ARM_JOINT_NAMES)
    assert calibration.motors["joint_6"].homing_offset == 1234
    assert calibration.motors["joint_1"].flip is True
    assert "gripper" not in calibration.motors


def test_confirmation_table_shows_zero_offsets_not_limits() -> None:
    rendered = _format_calibration_confirmation(
        _capture_zero_calibration("right", _raw_positions(100), {"joint_2"})
    )

    assert "Zero Raw" in rendered
    assert "Flip" in rendered
    assert "Max" not in rendered
    assert "gripper" not in rendered


def test_parse_flip_overrides_defaults_none_and_validation() -> None:
    assert _parse_flip_overrides(None, "left") == set(DEFAULT_FLIPS_BY_SIDE["left"])
    assert _parse_flip_overrides("none", "left") == set()
    assert _parse_flip_overrides("joint_1,joint_7", "right") == {"joint_1", "joint_7"}

    with pytest.raises(RuntimeError, match="unknown"):
        _parse_flip_overrides("gripper", "right")


def test_calibrate_side_writes_artifact_and_disconnects(tmp_path: Path) -> None:
    reader = _FakeRawReader(_raw_positions(2048))

    _calibrate_side(
        "left",
        "/dev/fake-left",
        tmp_path / "left",
        1_000_000,
        flips={"joint_3"},
        reader_factory=lambda _port, _baudrate: reader,
    )

    calibration = load_calibration(tmp_path / "left", "left")
    assert reader.connected and reader.disconnected
    assert calibration.motors["joint_1"].homing_offset == 2048
    assert calibration.motors["joint_3"].flip is True
    assert calibration.motors["joint_4"].flip is False


def test_zero_capture_rejects_extra_or_missing_joint() -> None:
    with_extra = _raw_positions(2048) | {"gripper": 1}
    without_joint = _raw_positions(2048)
    del without_joint["joint_7"]

    with pytest.raises(RuntimeError, match="extra"):
        _capture_zero_calibration("left", with_extra, set())
    with pytest.raises(RuntimeError, match="missing"):
        _capture_zero_calibration("left", without_joint, set())


class _FakeRawReader:
    def __init__(self, snapshot: dict[str, int]) -> None:
        self._snapshot = snapshot
        self.connected = False
        self.disconnected = False

    def connect(self) -> None:
        self.connected = True

    def disconnect(self) -> None:
        self.disconnected = True

    def read_raw_positions(self, _motor_ids_by_name: object) -> dict[str, int]:
        return self._snapshot
