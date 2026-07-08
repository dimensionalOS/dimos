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

from io import StringIO
import math
from pathlib import Path
import re

import pytest
from rich.console import Console
import typer
from typer.testing import CliRunner

from dimos.teleop.openarm_mini.calibration import (
    FEETECH_POSITION_SPAN,
    OPENARM_MINI_ARM_JOINT_NAMES,
    OpenArmMiniCalibration,
    OpenArmMiniMotorCalibration,
    save_calibration,
)
from dimos.teleop.openarm_mini.tools.joint_tui import (
    _build_joint_dashboard,
    _load_tui_calibration,
    _read_side_rows,
    _resolve_calibration_path,
    main,
)


def _joint_tui_app() -> typer.Typer:
    app = typer.Typer()
    app.command()(main)
    return app


def test_read_side_rows_displays_raw_radians_clamped_and_flip(tmp_path: Path) -> None:
    calibration = OpenArmMiniCalibration(
        side="left",
        motors={
            joint_name: OpenArmMiniMotorCalibration(
                id=index + 1,
                homing_offset=2048,
                flip=joint_name == "joint_1",
            )
            for index, joint_name in enumerate(OPENARM_MINI_ARM_JOINT_NAMES)
        },
    )
    calibration_path = tmp_path / "left"
    save_calibration(calibration_path, calibration)
    raw_positions = {joint_name: 2048 for joint_name in OPENARM_MINI_ARM_JOINT_NAMES}
    raw_positions["joint_1"] = 2049
    raw_positions["joint_4"] = 0

    rows = _read_side_rows(_load_tui_calibration("left", calibration_path), raw_positions)

    assert rows[0].side == "left"
    assert rows[0].joint == "joint_1"
    assert rows[0].follower_joint == "openarm_left_joint1"
    assert rows[0].motor_id == 1
    assert rows[0].raw == 2049
    assert rows[0].flip is True
    assert rows[0].radians == -(math.tau / (FEETECH_POSITION_SPAN + 1))
    assert rows[3].clamped_radians == 2.4


def test_build_joint_dashboard_contains_key_columns(tmp_path: Path) -> None:
    calibration = OpenArmMiniCalibration(
        side="right",
        motors={
            joint_name: OpenArmMiniMotorCalibration(id=index + 1, homing_offset=100, flip=False)
            for index, joint_name in enumerate(OPENARM_MINI_ARM_JOINT_NAMES)
        },
    )
    calibration_path = tmp_path / "right"
    save_calibration(calibration_path, calibration)
    rows = _read_side_rows(
        _load_tui_calibration("right", calibration_path),
        {joint: 100 for joint in OPENARM_MINI_ARM_JOINT_NAMES},
    )
    console = Console(record=True, width=140, file=StringIO())

    console.print(_build_joint_dashboard(rows))
    rendered = console.export_text()

    assert "OpenArm Mini leader joint readout" in rendered
    assert "Follower Joint" in rendered
    assert "Clamped Rad" in rendered
    assert "openarm_right_joint7" in rendered


def test_resolve_calibration_path_uses_side_default(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    state_dir = tmp_path / "openarm_mini"
    left_path = state_dir / "left"
    right_path = state_dir / "right"
    save_calibration(
        left_path,
        OpenArmMiniCalibration(
            side="left",
            motors={
                joint_name: OpenArmMiniMotorCalibration(id=index + 1, homing_offset=0)
                for index, joint_name in enumerate(OPENARM_MINI_ARM_JOINT_NAMES)
            },
        ),
    )

    def fake_default_calibration_path(side: str) -> Path:
        return left_path if side == "left" else right_path

    monkeypatch.setattr(
        "dimos.teleop.openarm_mini.tools.joint_tui.default_calibration_path",
        fake_default_calibration_path,
    )

    assert _resolve_calibration_path("left", None) == left_path
    assert _resolve_calibration_path("right", None) == right_path


def test_joint_tui_cli_uses_side_and_single_port_without_side_specific_ports() -> None:
    runner = CliRunner()
    result = runner.invoke(_joint_tui_app(), ["--help"])
    output = re.sub(r"\x1b\[[0-9;]*m", "", result.output)

    assert result.exit_code == 0
    assert "--side" in output
    assert "--port" in output
    assert "--calibration-path" in output
    assert "--port-left" not in output
    assert "--port-right" not in output
