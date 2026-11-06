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

"""Python-version CLI regression: production parsing, fake final MCP transport."""

from typing import Any

import pytest
from typer.testing import CliRunner

import dimos.cli.commands.mcp as command
from dimos.cli.dimos import main


@pytest.mark.parametrize("args", [[], ["mem"], ["mcp"], ["mcp", "call"]])
def test_help_constructs_without_transport(args: list[str]) -> None:
    result = CliRunner().invoke(main, [*args, "--help"])
    assert result.exit_code == 0, result.output + repr(result.exception)
    assert "Usage:" in result.output


@pytest.mark.parametrize(
    ("args", "expected"),
    [
        ([], {}),
        (["--arg", "x=0.5", "--arg", "enabled=true"], {"x": 0.5, "enabled": True}),
        (
            ["-a", "n=3", "-a", "name=sample", "-a", "missing=null"],
            {"n": 3, "name": "sample", "missing": None},
        ),
        (["--json-args", '{"x":0.5,"enabled":true}'], {"x": 0.5, "enabled": True}),
    ],
)
def test_values_reach_adapter(
    monkeypatch: pytest.MonkeyPatch, args: list[str], expected: dict[str, Any]
) -> None:
    calls = []

    class Adapter:
        def call_tool(self, name: str, arguments: dict[str, Any]) -> dict[str, Any]:
            calls.append((name, arguments))
            return {"content": [{"text": "fixture result"}]}

    monkeypatch.setattr(command, "_get_adapter", lambda timeout: Adapter())
    result = CliRunner().invoke(main, ["mcp", "call", "inspect", *args])
    assert result.exit_code == 0, result.output + repr(result.exception)
    assert calls == [("inspect", expected)]
    assert "fixture result" in result.output


def test_malformed_argument_rejected_before_adapter(monkeypatch: pytest.MonkeyPatch) -> None:
    def forbidden(*args: Any) -> None:
        raise AssertionError("invalid argument reached adapter")

    monkeypatch.setattr(command, "_get_adapter", forbidden)
    result = CliRunner().invoke(main, ["mcp", "call", "inspect", "--arg", "invalid"])
    assert result.exit_code == 2, result.output + repr(result.exception)
    assert "expected KEY=VALUE" in result.output
