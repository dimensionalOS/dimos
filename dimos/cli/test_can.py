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

import subprocess
import sys
from unittest.mock import Mock

from click.testing import Result
import pytest
from pytest_mock import MockerFixture
from typer.testing import CliRunner

from dimos.cli.dimos import main


def _subprocess_argv(run: Mock) -> list[list[str]]:
    return [call.args[0] for call in run.call_args_list]


def _invoke_can(args: list[str]) -> Result:
    return CliRunner().invoke(main, ["hardware", "can", *args])


def test_list_linux_can_interfaces(mocker: MockerFixture) -> None:
    mocker.patch("dimos.cli.can.sys.platform", "linux")
    run = mocker.patch(
        "dimos.cli.can.subprocess.run",
        return_value=subprocess.CompletedProcess(
            [], 0, stdout="can0             UP\ncan1             DOWN\n", stderr=""
        ),
    )

    result = _invoke_can(["list"])

    assert result.exit_code == 0, result.output
    assert "can0" in result.stdout
    assert "can1" in result.stdout
    run.assert_called_once_with(
        ["ip", "-brief", "link", "show", "type", "can"],
        check=True,
        capture_output=True,
        text=True,
    )


def test_list_macos_gs_usb_serials(mocker: MockerFixture) -> None:
    mocker.patch("dimos.cli.can.sys.platform", "darwin")
    list_gs_usb = mocker.patch.object(
        sys.modules["can_motor_control"],
        "list_gs_usb_devices",
        create=True,
        return_value=[
            Mock(index=0, serial_number="LEFT123"),
            Mock(index=1, serial_number="RIGHT456"),
        ],
    )

    result = _invoke_can(["list"])

    assert result.exit_code == 0, result.output
    assert "INDEX  SERIAL" in result.stdout
    assert "0      LEFT123" in result.stdout
    assert "1      RIGHT456" in result.stdout
    list_gs_usb.assert_called_once_with(vendor_id=0x1D50, product_id=0x606F)


def test_list_macos_warns_about_missing_serial(mocker: MockerFixture) -> None:
    mocker.patch("dimos.cli.can.sys.platform", "darwin")
    mocker.patch.object(
        sys.modules["can_motor_control"],
        "list_gs_usb_devices",
        create=True,
        return_value=[Mock(index=0, serial_number=None)],
    )

    result = _invoke_can(["list"])

    assert result.exit_code == 0, result.output
    assert "<missing>" in result.stdout
    assert "unique, non-empty USB serials" in result.stderr


def test_list_macos_reports_discovery_error(mocker: MockerFixture) -> None:
    mocker.patch("dimos.cli.can.sys.platform", "darwin")
    mocker.patch.object(
        sys.modules["can_motor_control"],
        "list_gs_usb_devices",
        create=True,
        side_effect=sys.modules["can_motor_control"].TransportError("USB unavailable"),
    )

    result = _invoke_can(["list"])

    assert result.exit_code == 1
    assert "CAN device discovery failed: USB unavailable" in result.stderr


def test_setup_valid_options_configures_and_verifies_can_interface(
    mocker: MockerFixture,
) -> None:
    mocker.patch("dimos.cli.can.os.geteuid", return_value=1000)
    run = mocker.patch(
        "dimos.cli.can.subprocess.run",
        return_value=subprocess.CompletedProcess(
            [],
            0,
            stdout="4: follower_l: UP qlen 1000\n",
            stderr="",
        ),
    )

    result = _invoke_can(["setup", "follower_l"])

    assert result.exit_code == 0, result.output
    assert "Running: sudo -- ip link set dev follower_l down" in result.stdout
    assert "bitrate=1000000, txqueuelen=1000" in result.stdout
    assert _subprocess_argv(run) == [
        ["ip", "link", "show", "dev", "follower_l"],
        ["sudo", "--", "ip", "link", "set", "dev", "follower_l", "down"],
        [
            "sudo",
            "--",
            "ip",
            "link",
            "set",
            "dev",
            "follower_l",
            "type",
            "can",
            "bitrate",
            "1000000",
        ],
        [
            "sudo",
            "--",
            "ip",
            "link",
            "set",
            "dev",
            "follower_l",
            "txqueuelen",
            "1000",
        ],
        ["sudo", "--", "ip", "link", "set", "dev", "follower_l", "up"],
        ["ip", "-details", "-statistics", "link", "show", "dev", "follower_l"],
    ]


def test_setup_nonpositive_queue_length_returns_usage_error() -> None:
    result = _invoke_can(["setup", "can0", "--txqueuelen", "0"])

    assert result.exit_code == 2
    assert "x>=1" in result.output


def test_setup_bitrate_below_minimum_returns_usage_error() -> None:
    result = _invoke_can(["setup", "can0", "--bitrate", "9999"])

    assert result.exit_code == 2
    assert "x>=10000" in result.output


def test_status_existing_interface_prints_detailed_state(mocker: MockerFixture) -> None:
    run = mocker.patch(
        "dimos.cli.can.subprocess.run",
        return_value=subprocess.CompletedProcess([], 0, stdout="can0: UP\n", stderr=""),
    )

    result = _invoke_can(["status", "can0"])

    assert result.exit_code == 0, result.output
    assert result.stdout == "can0: UP\n"
    run.assert_called_once_with(
        ["ip", "-details", "-statistics", "link", "show", "dev", "can0"],
        check=True,
        capture_output=True,
        text=True,
    )


def test_down_nonroot_user_runs_privileged_command_with_sudo(
    mocker: MockerFixture,
) -> None:
    mocker.patch("dimos.cli.can.os.geteuid", return_value=1000)
    run = mocker.patch(
        "dimos.cli.can.subprocess.run",
        return_value=subprocess.CompletedProcess([], 0),
    )

    result = _invoke_can(["down", "can1"])

    assert result.exit_code == 0, result.output
    assert "CAN interface can1 is down" in result.stdout
    run.assert_called_once_with(
        ["sudo", "--", "ip", "link", "set", "dev", "can1", "down"],
        check=True,
        capture_output=False,
        text=True,
    )


def test_up_root_user_runs_ip_without_sudo(mocker: MockerFixture) -> None:
    mocker.patch("dimos.cli.can.os.geteuid", return_value=0)
    run = mocker.patch(
        "dimos.cli.can.subprocess.run",
        return_value=subprocess.CompletedProcess([], 0),
    )

    result = _invoke_can(["up", "can2"])

    assert result.exit_code == 0, result.output
    assert "CAN interface can2 is up" in result.stdout
    run.assert_called_once_with(
        ["ip", "link", "set", "dev", "can2", "up"],
        check=True,
        capture_output=False,
        text=True,
    )


def test_status_missing_ip_command_returns_usage_error(mocker: MockerFixture) -> None:
    mocker.patch("dimos.cli.can.subprocess.run", side_effect=FileNotFoundError)

    result = _invoke_can(["status", "can0"])

    assert result.exit_code == 2
    assert "the 'ip' command is not installed" in result.output


@pytest.mark.parametrize(
    ("stderr", "stdout", "expected_detail"),
    [
        ("permission denied\n", "ignored\n", "permission denied"),
        ("", "device not found\n", "device not found"),
        ("", "", "exit code 7"),
    ],
)
def test_status_failed_ip_command_reports_available_detail(
    mocker: MockerFixture,
    stderr: str,
    stdout: str,
    expected_detail: str,
) -> None:
    mocker.patch(
        "dimos.cli.can.subprocess.run",
        side_effect=subprocess.CalledProcessError(
            7,
            ["ip"],
            output=stdout,
            stderr=stderr,
        ),
    )

    result = _invoke_can(["status", "can0"])

    assert result.exit_code == 1
    assert f"CAN interface command failed: {expected_detail}" in result.output
