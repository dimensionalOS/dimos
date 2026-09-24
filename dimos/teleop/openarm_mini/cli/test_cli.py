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

import subprocess
import sys

import pytest
from typer.testing import CliRunner

from dimos.teleop.openarm_mini.calibration import default_calibration_path
from dimos.teleop.openarm_mini.cli import calibrate, joint_tui, setup_motor_id
from dimos.teleop.openarm_mini.cli.app import app
from dimos.teleop.openarm_mini.feetech import (
    OPENARM_MINI_DEFAULT_BAUDRATE,
    OpenArmMiniDependencyError,
)

runner = CliRunner()


def test_openarm_mini_cli_lists_every_operator_command() -> None:
    result = runner.invoke(app, ["--help"])

    assert result.exit_code == 0, result.output
    assert "calibrate" in result.output
    assert "joint-tui" in result.output
    assert "setup-motor-id" in result.output


@pytest.mark.parametrize("command", ["calibrate", "joint-tui", "setup-motor-id"])
def test_openarm_mini_command_help_needs_no_hardware(command: str) -> None:
    result = runner.invoke(app, [command, "--help"])

    assert result.exit_code == 0, result.output


def test_calibrate_delegates_parsed_options(mocker) -> None:
    run = mocker.patch.object(calibrate, "_run")

    result = runner.invoke(
        app,
        [
            "calibrate",
            "--side",
            "left",
            "--port-left",
            "/dev/left",
            "--port-right",
            "/dev/right",
            "--baudrate",
            "1000000",
            "--live-readout",
        ],
    )

    assert result.exit_code == 0, result.output
    run.assert_called_once_with(
        side="left",
        port_left="/dev/left",
        port_right="/dev/right",
        baudrate=1_000_000,
        left_calibration_path=default_calibration_path("left"),
        right_calibration_path=default_calibration_path("right"),
        left_flips=None,
        right_flips=None,
        live_readout=True,
    )


def test_joint_tui_delegates_parsed_options(mocker) -> None:
    run = mocker.patch.object(joint_tui, "_run")

    result = runner.invoke(
        app,
        ["joint-tui", "--side", "right", "--port", "/dev/right"],
    )

    assert result.exit_code == 0, result.output
    run.assert_called_once_with(
        side="right",
        port="/dev/right",
        baudrate=OPENARM_MINI_DEFAULT_BAUDRATE,
        calibration_path=None,
        refresh_hz=10.0,
    )


def test_setup_motor_id_delegates_parsed_options(mocker) -> None:
    run = mocker.patch.object(setup_motor_id, "_run")

    result = runner.invoke(
        app,
        [
            "setup-motor-id",
            "--port",
            "/dev/motor",
            "--new-id",
            "3",
            "--old-id",
            "1",
            "--baudrate",
            "1000000",
            "--yes",
        ],
    )

    assert result.exit_code == 0, result.output
    run.assert_called_once_with(
        port="/dev/motor",
        new_id=3,
        old_id=1,
        baudrate=1_000_000,
        yes=True,
    )


def test_missing_sdk_is_a_clean_actionable_cli_error(mocker) -> None:
    mocker.patch.object(
        setup_motor_id,
        "_run",
        side_effect=OpenArmMiniDependencyError("Install the OpenArm Mini extra."),
    )

    result = runner.invoke(
        app,
        [
            "setup-motor-id",
            "--port",
            "/dev/motor",
            "--new-id",
            "3",
            "--baudrate",
            "1000000",
            "--yes",
        ],
    )

    assert result.exit_code == 1
    assert result.output == "Install the OpenArm Mini extra.\n"
    assert result.exception is not None
    assert "Traceback" not in result.output


def test_importing_openarm_mini_cli_app_is_lightweight() -> None:
    script = (
        "import sys; "
        "import dimos.teleop.openarm_mini.cli.app; "
        "bad = [m for m in "
        "('scservo_sdk', 'numpy', 'rich', 'dimos.control', 'dimos.manipulation') "
        "if m in sys.modules]; "
        "assert not bad, f'Heavy imports: {bad}'"
    )

    result = subprocess.run(
        [sys.executable, "-c", script],
        capture_output=True,
        text=True,
        timeout=30,
    )

    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize(
    "module",
    [
        "dimos.teleop.openarm_mini.cli.calibrate",
        "dimos.teleop.openarm_mini.cli.joint_tui",
        "dimos.teleop.openarm_mini.cli.setup_motor_id",
    ],
)
def test_direct_module_help_remains_supported(module: str) -> None:
    result = subprocess.run(
        [sys.executable, "-m", module, "--help"],
        capture_output=True,
        text=True,
        timeout=30,
    )

    assert result.returncode == 0, result.stderr


def test_help_does_not_load_openarm_execution_dependencies() -> None:
    script = (
        "import sys; "
        "from typer.testing import CliRunner; "
        "from dimos.teleop.openarm_mini.cli.app import app; "
        "result = CliRunner().invoke(app, ['--help']); "
        "assert result.exit_code == 0, result.output; "
        "bad = [m for m in "
        "('scservo_sdk', 'numpy', 'dimos.control', 'dimos.manipulation') "
        "if m in sys.modules]; "
        "assert not bad, f'Heavy imports: {bad}'"
    )

    result = subprocess.run(
        [sys.executable, "-c", script],
        capture_output=True,
        text=True,
        timeout=30,
    )

    assert result.returncode == 0, result.stderr
