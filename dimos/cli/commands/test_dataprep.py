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

from dimos.cli.commands.dataprep import dataprep_app


def test_dataprep_build_exposes_only_profile_based_configuration() -> None:
    result = CliRunner().invoke(dataprep_app, ["build", "--help"])

    assert result.exit_code == 0
    assert "--profile" in result.output
    assert "--source" in result.output
    assert "--config" not in result.output
    assert "--format" not in result.output


def test_dataprep_inspect_auto_detects_format() -> None:
    result = CliRunner().invoke(dataprep_app, ["inspect", "--help"])

    assert result.exit_code == 0
    assert "--profile" in result.output
    assert "--config" not in result.output
    assert "--format" not in result.output
