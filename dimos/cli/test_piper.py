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

from typer.testing import CliRunner

from dimos.cli.dimos import main
import dimos.cli.piper as piper

runner = CliRunner()


def test_can_activate_confirms_before_generic_setup(mocker) -> None:
    confirm = mocker.patch.object(piper.typer, "confirm", return_value=True)
    setup_interface = mocker.patch.object(piper, "setup_interface")

    result = runner.invoke(main, ["piper", "can-activate", "can1", "--bitrate", "500000"])

    assert result.exit_code == 0, result.output
    confirm.assert_called_once()
    setup_interface.assert_called_once_with("can1", bitrate=500000)


def test_can_activate_rejection_does_not_configure_interface(mocker) -> None:
    mocker.patch.object(piper.typer, "confirm", return_value=False)
    setup_interface = mocker.patch.object(piper, "setup_interface")

    result = runner.invoke(main, ["piper", "can-activate", "can0"])

    assert result.exit_code == 1
    assert "Aborted." in result.output
    setup_interface.assert_not_called()


def test_can_activate_uses_default_bitrate(mocker) -> None:
    mocker.patch.object(piper.typer, "confirm", return_value=True)
    setup_interface = mocker.patch.object(piper, "setup_interface")

    result = runner.invoke(main, ["piper", "can-activate", "can0"])

    assert result.exit_code == 0, result.output
    setup_interface.assert_called_once_with("can0", bitrate=1_000_000)


def test_can_activate_rejects_nonpositive_bitrate_before_confirmation(mocker) -> None:
    confirm = mocker.patch.object(piper.typer, "confirm")

    result = runner.invoke(main, ["piper", "can-activate", "can0", "--bitrate", "0"])

    assert result.exit_code == 2
    confirm.assert_not_called()
