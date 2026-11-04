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

"""The old entry point stays usable until its replacement is introduced."""

from typer.testing import CliRunner

from dimos.cli.dimos import main


def test_dataprep_remains_registered_before_the_imitation_cli():
    result = CliRunner().invoke(main, ["dataprep", "--help"])

    assert result.exit_code == 0, result.output
    assert "build" in result.output
    assert "inspect" in result.output
