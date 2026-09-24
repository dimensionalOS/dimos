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

from dimos.cli import hardware_cli

runner = CliRunner()


def test_hardware_namespace_mounts_openarm_mini_commands() -> None:
    result = runner.invoke(hardware_cli.app, ["openarm-mini", "--help"])

    assert result.exit_code == 0, result.output
    assert "calibrate" in result.output
    assert "joint-tui" in result.output
    assert "setup-motor-id" in result.output
