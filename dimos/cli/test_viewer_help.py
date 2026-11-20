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

import pytest
from typer.testing import CliRunner

from dimos.cli.commands.map import map_app
from dimos.memory.cli.app import mem_app


@pytest.mark.parametrize(
    ("app", "command"),
    [
        (mem_app, "rerun"),
        (map_app, "global"),
        (map_app, "replay"),
        (map_app, "replay-marker"),
        (map_app, "view"),
    ],
)
def test_help_explains_detached_viewer(app, command):
    result = CliRunner().invoke(app, [command, "--help"], terminal_width=200)

    assert result.exit_code == 0, result.output
    assert "detached viewer" in result.output
    assert "Close the viewer window separately" in result.output
    assert "dimos stop" in result.output
