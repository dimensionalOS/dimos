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

import os
from pathlib import Path
from unittest.mock import MagicMock

import pytest
from pytest_mock import MockerFixture
import requests
from typer.testing import CliRunner

from dimos.cli.dimos import main
from dimos.core.run_registry import RunEntry


@pytest.fixture(autouse=True)
def isolated_registry(tmp_path: Path, monkeypatch: pytest.MonkeyPatch, mcp_url: str) -> None:
    monkeypatch.setattr("dimos.core.run_registry.REGISTRY_DIR", tmp_path / "runs")
    monkeypatch.setattr("dimos.agents.mcp.mcp_adapter.DEFAULT_MCP_URL", mcp_url)


@pytest.fixture(
    params=[
        ["mcp", "list-tools"],
        ["mcp", "call", "ping"],
        ["mcp", "status"],
        ["mcp", "modules"],
        ["agent-send", "hello"],
    ],
    ids=["list-tools", "call", "status", "modules", "agent-send"],
)
def command(request: pytest.FixtureRequest) -> list[str]:
    return list(request.param)


@pytest.fixture
def run_entry(tmp_path: Path) -> RunEntry:
    entry = RunEntry(
        run_id="20261115-000001-aaaa-test",
        pid=os.getpid(),
        blueprint="coordinator-mock",
        started_at="2026-11-15T00:00:01+00:00",
        log_dir=str(tmp_path / "logs"),
    )
    entry.save()
    return entry


@pytest.fixture
def post(mocker: MockerFixture) -> MagicMock:
    return mocker.patch(
        "dimos.agents.mcp.mcp_adapter.requests.post", side_effect=requests.ConnectionError
    )


def test_missing_endpoint_reports_uncertainty_and_relaunch_steps(
    command: list[str], run_entry: RunEntry, post: MagicMock, mcp_url: str
) -> None:
    result = CliRunner().invoke(main, command)

    assert result.exit_code == 1
    assert result.output == (
        f"Error: could not reach MCP server at {mcp_url}. "
        "coordinator-mock is running, but no MCP endpoint is recorded. "
        "Check the server's address if MCP is already configured. "
        "If the blueprint lacks McpServer, stop the existing run with `dimos stop`, "
        "then relaunch with `dimos run <blueprint> mcp-server`, preserving the original "
        "blueprints and options.\n"
    )
    assert post.call_args.args == (mcp_url,)


def test_connection_failure_without_a_registered_run(
    command: list[str], post: MagicMock, mcp_url: str
) -> None:
    result = CliRunner().invoke(main, command)

    assert result.exit_code == 1
    assert result.output == (
        f"Error: could not reach MCP server at {mcp_url}. No running DimOS instance found.\n"
    )


def test_stale_run_is_not_reported_as_running(
    command: list[str], run_entry: RunEntry, post: MagicMock, mocker: MockerFixture, mcp_url: str
) -> None:
    mocker.patch("dimos.core.run_registry.is_pid_alive", return_value=False)

    result = CliRunner().invoke(main, command)

    assert result.exit_code == 1
    assert result.output == (
        f"Error: could not reach MCP server at {mcp_url}. No running DimOS instance found.\n"
    )


def test_unreachable_recorded_endpoint_names_its_run(
    command: list[str], run_entry: RunEntry, post: MagicMock, tmp_path: Path
) -> None:
    run_entry.mcp_url = "http://127.0.0.1:12345/mcp"
    run_entry.save()
    RunEntry(
        run_id="20261115-000002-bbbb-newer",
        pid=os.getpid(),
        blueprint="newer-without-mcp",
        started_at="2026-11-15T00:00:02+00:00",
        log_dir=str(tmp_path / "newer-logs"),
    ).save()

    result = CliRunner().invoke(main, command)

    assert result.exit_code == 1
    assert result.output == (
        f"Error: could not reach MCP server at {run_entry.mcp_url}. "
        "coordinator-mock is running and records this endpoint. "
        "Check its MCP server and run logs.\n"
    )
    assert post.call_args.args == (run_entry.mcp_url,)


@pytest.mark.parametrize("error", [requests.ReadTimeout, requests.ConnectTimeout])
def test_timeout_is_reported_without_claiming_the_server_is_missing(
    command: list[str], run_entry: RunEntry, post: MagicMock, mcp_url: str, error: type[Exception]
) -> None:
    post.side_effect = error

    result = CliRunner().invoke(main, command)

    assert result.exit_code == 1
    assert result.output == f"Error: MCP request to {mcp_url} timed out.\n"


def test_default_endpoint_is_tried_even_without_a_recorded_endpoint(
    command: list[str], run_entry: RunEntry, post: MagicMock, mcp_url: str
) -> None:
    post.side_effect = None
    post.return_value.json.return_value = {
        "result": {"tools": [{"name": "ping"}], "content": [{"type": "text", "text": "ping"}]}
    }

    result = CliRunner().invoke(main, command)

    assert result.exit_code == 0, result.output
    assert "ping" in result.output
    assert post.call_args.args == (mcp_url,)


def test_mcp_protocol_errors_keep_the_server_message(command: list[str], post: MagicMock) -> None:
    post.side_effect = None
    post.return_value.json.return_value = {"error": {"code": -32601, "message": "Unknown method"}}

    result = CliRunner().invoke(main, command)

    assert result.exit_code == 1
    assert result.output == "Error: Unknown method\n"


@pytest.mark.parametrize(
    "args",
    [
        ["mcp", "call", "ping"],
        ["mcp", "status"],
        ["mcp", "modules"],
        ["agent-send", "hello"],
    ],
)
def test_tool_failures_reach_stderr_and_set_exit_status(args: list[str], post: MagicMock) -> None:
    post.side_effect = None
    post.return_value.json.return_value = {
        "result": {
            "content": [{"type": "text", "text": t} for t in ("refused", "busy")],
            "isError": True,
        }
    }

    result = CliRunner().invoke(main, args)

    assert result.exit_code == 1
    assert "busy" in result.stderr


def test_tool_status_comes_from_flag_independently_of_text(post: MagicMock) -> None:
    post.side_effect = None
    post.return_value.json.return_value = {
        "result": {
            "content": [
                {"type": "text", "text": "Error: last run failed"},
                {"type": "text", "text": "Retry succeeded"},
            ],
            "isError": False,
        }
    }

    result = CliRunner().invoke(main, ["mcp", "call", "history"])

    assert result.exit_code == 0
    assert result.stdout == "Error: last run failed\nRetry succeeded\n"
