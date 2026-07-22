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

from unittest.mock import Mock

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
    run.assert_called_once()
    command = run.call_args.args[0]
    assert command[0].endswith("can_activate.sh")
    assert command[1:] == ["can1", "500000"]
    assert run.call_args.kwargs == {"check": True}


def test_can_activate_rejection_does_not_spawn(monkeypatch):
    confirm = Mock(return_value=False)
    run = Mock()
    monkeypatch.setattr(piper.typer, "confirm", confirm)
    monkeypatch.setattr(piper.subprocess, "run", run)

    result = runner.invoke(piper.app, ["can0"])

    assert result.exit_code == 1
    assert "Aborted." in result.output
    run.assert_not_called()


def test_bundled_helper_and_license_are_available():
    package = piper.resources.files("dimos.robot.manipulators.piper.scripts")

    with piper._can_activation_helper() as helper:
        assert helper.name == "can_activate.sh"
        assert helper.is_file()
    assert (package / "LICENSE").is_file()
