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

import pytest
from typer.testing import CliRunner

from dimos.cli.commands import agent
from dimos.cli.dimos import main


@pytest.mark.parametrize(
    "words",
    [["run", "--json", "--help"], ["--cwd", ".", "help"]],
)
def test_forwarded_help_without_node(words: list[str], monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(agent, "_node_argv", Mock(side_effect=AssertionError("started Node")))

    result = CliRunner().invoke(main, ["agent", *words], catch_exceptions=False)

    assert result.exit_code == 0
    assert "conversations" in result.stdout


@pytest.mark.parametrize(
    "words",
    [
        ["run", "--", "--help"],
        ["run", "--cwd", "--help"],
        ["run", "--typo", "--help"],
        ["run", "--json=true", "--help"],
        ["run", "--conversation", "help"],
    ],
)
def test_non_help_requests_are_forwarded_verbatim(
    words: list[str], monkeypatch: pytest.MonkeyPatch
) -> None:
    argv = Mock(side_effect=SystemExit(0))
    monkeypatch.setattr(agent, "_node_argv", argv)

    result = CliRunner().invoke(main, ["agent", *words], catch_exceptions=False)

    assert result.exit_code == 0
    argv.assert_called_once_with(words)
