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

from unittest.mock import Mock, call

from typer.testing import CliRunner

from dimos.robot.cli import piper

runner = CliRunner()


def test_can_activate_confirms_before_spawning(monkeypatch):
    confirm = Mock(return_value=True)
    run = Mock()
    monkeypatch.setattr(piper.typer, "confirm", confirm)
    monkeypatch.setattr(piper.subprocess, "run", run)

    result = runner.invoke(piper.app, ["can1", "--bitrate", "500000"])

    assert result.exit_code == 0, result.output
    confirm.assert_called_once()
    assert run.call_args_list == [
        call(["sudo", "ip", "link", "set", "can1", "down"], check=True),
        call(
            ["sudo", "ip", "link", "set", "can1", "type", "can", "bitrate", "500000"],
            check=True,
        ),
        call(["sudo", "ip", "link", "set", "can1", "up"], check=True),
    ]


def test_can_activate_rejection_does_not_spawn(monkeypatch):
    confirm = Mock(return_value=False)
    run = Mock()
    monkeypatch.setattr(piper.typer, "confirm", confirm)
    monkeypatch.setattr(piper.subprocess, "run", run)

    result = runner.invoke(piper.app, ["can0"])

    assert result.exit_code == 1
    assert "Aborted." in result.output
    run.assert_not_called()


def test_can_activate_uses_default_bitrate(monkeypatch):
    monkeypatch.setattr(piper.typer, "confirm", Mock(return_value=True))
    run = Mock()
    monkeypatch.setattr(piper.subprocess, "run", run)

    result = runner.invoke(piper.app, ["can0"])

    assert result.exit_code == 0, result.output
    assert run.call_args_list[1] == call(
        ["sudo", "ip", "link", "set", "can0", "type", "can", "bitrate", "1000000"],
        check=True,
    )
