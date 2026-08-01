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

from pathlib import Path
from unittest.mock import Mock

from typer.testing import CliRunner

from dimos.cli.hardware import a1z as a1z_cli

runner = CliRunner()


def test_a1z_help_lists_learning_commands_without_importing_lerobot() -> None:
    result = runner.invoke(a1z_cli.app, ["--help"])

    assert result.exit_code == 0, result.output
    assert "teach" in result.output
    assert "replay" in result.output
    assert "run-policy" in result.output


def test_teach_refuses_to_overwrite_recording(tmp_path: Path) -> None:
    recording = tmp_path / "existing.db"
    recording.touch()

    result = runner.invoke(a1z_cli.app, ["teach", str(recording)])

    assert result.exit_code == 2
    assert "refusing to overwrite existing recording" in result.output


def test_run_policy_loads_isolated_contract_without_host_lerobot(monkeypatch) -> None:
    monkeypatch.setattr(a1z_cli.typer, "confirm", Mock(return_value=False))

    result = runner.invoke(a1z_cli.app, ["run-policy", "checkpoint"])

    assert result.exit_code == 0, result.output
    assert "Policy execution cancelled" in result.output
