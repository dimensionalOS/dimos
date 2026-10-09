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

import time
from unittest.mock import MagicMock

import pytest
from pytest_mock import MockerFixture
from typer.testing import CliRunner

from dimos.cli import hardware_cli
from dimos.cli.hardware import seeedstudio as seeed_cli
from dimos.hardware.manipulators.seeedstudio.protocol import Feedback, MotorParameters

runner = CliRunner()


@pytest.fixture
def bus(mocker: MockerFixture) -> MagicMock:
    transport: MagicMock = mocker.patch(
        "dimos.cli.hardware.seeedstudio.DmSerialTransport"
    ).return_value
    transport.parameters.side_effect = lambda mid, fid: MotorParameters(
        mid, fid, 2, 12.5, 10, 28, 0
    )
    transport.feedback.side_effect = lambda p: Feedback(-0.1, 0.0, 0.0, 0, 25, 25, time.monotonic())
    return transport


def test_hardware_namespace_exposes_seeedstudio_doctor() -> None:
    result = runner.invoke(hardware_cli.app, ["seeedstudio", "--help"])
    assert result.exit_code == 0, result.output
    assert "doctor" in result.output


def test_inspect_reports_every_motor_without_control_writes(bus: MagicMock) -> None:
    def partial(mid: int, fid: int) -> MotorParameters:
        if mid != 1:
            raise TimeoutError(f"motor {mid} missing")
        return MotorParameters(mid, fid, 2, 12.5, 10, 28, 0)

    bus.parameters.side_effect = partial
    reports = seeed_cli.inspect_motors("test-only")
    assert [r.motor_id for r in reports] == list(range(1, 8))
    assert [r.error is None for r in reports] == [True] + [False] * 6
    bus.send.assert_not_called()
    bus.close.assert_called_once()


def test_failed_open_still_closes_transport(bus: MagicMock) -> None:
    bus.open.side_effect = OSError("USB unplugged")
    with pytest.raises(OSError, match="unplugged"):
        seeed_cli.inspect_motors("test-only")
    bus.close.assert_called_once()


def test_doctor_passes_healthy_arm(bus: MagicMock) -> None:
    result = runner.invoke(seeed_cli.app, ["test-only"])
    assert result.exit_code == 0, result.output
    assert result.output.count("PASS") == 7
    bus.send.assert_not_called()


def test_doctor_flags_wrong_mode_and_out_of_range_joint(bus: MagicMock) -> None:
    bus.parameters.side_effect = lambda mid, fid: MotorParameters(
        mid, fid, 1 if mid == 3 else 2, 12.5, 10, 28, 0
    )
    bus.feedback.side_effect = lambda p: Feedback(
        0.5 if p.motor_id == 2 else -0.1, 0.0, 0.0, 0, 25, 25, time.monotonic()
    )
    result = runner.invoke(seeed_cli.app, ["test-only"])
    assert result.exit_code == 1
    assert "FAIL joint2 (motor 2): position outside" in result.output
    assert "FAIL joint3 (motor 3): control mode 1" in result.output


def test_doctor_requires_exactly_one_bridge_when_port_is_omitted(mocker: MockerFixture) -> None:
    mocker.patch.object(seeed_cli, "find_bridge_ports", return_value=[])
    result = runner.invoke(seeed_cli.app, [])
    assert result.exit_code == 1
    assert "found 0" in result.output


def test_doctor_reports_wrapped_gripper_in_travel(bus: MagicMock) -> None:
    bus.feedback.side_effect = lambda p: Feedback(
        2.8513 if p.motor_id == 7 else -0.1, 0.0, 0.0, 0, 25, 25, time.monotonic()
    )
    result = runner.invoke(seeed_cli.app, ["test-only"])
    assert result.exit_code == 0, result.output
    assert "-3.4319 rad in travel (reported +2.8513)" in result.output
