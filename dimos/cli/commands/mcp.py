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

"""`dimos mcp` commands plus agent-send, talking to the running MCP server."""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
import json
from typing import Any

import requests
import typer

from dimos.agents.mcp.mcp_adapter import McpAdapter, McpError
from dimos.core.run_registry import list_runs

mcp_app = typer.Typer(help="Interact with the running MCP server")


def _get_adapter(timeout: int | None = None) -> McpAdapter:
    """Get an McpAdapter from the latest RunEntry or default URL."""
    return McpAdapter.from_run_entry(timeout=timeout)


def _connection_error(url: str) -> str:
    """Describe a failed connection without inferring capabilities from missing metadata."""
    error = f"Error: could not reach MCP server at {url}."
    runs = list_runs()
    if not runs:
        return f"{error} No running DimOS instance found."
    for run in reversed(runs):
        if run.mcp_url == url:
            return (
                f"{error} {run.blueprint} is running and records this endpoint. "
                "Check its MCP server and run logs."
            )
    if not any(run.mcp_url for run in runs):
        return (
            f"{error} {runs[-1].blueprint} is running, but no MCP endpoint is recorded. "
            "Check the server's address if MCP is already configured. "
            "If the blueprint lacks McpServer, stop the existing run with `dimos stop`, "
            "then relaunch with `dimos run <blueprint> mcp-server`, preserving the original "
            "blueprints and options."
        )
    return f"{error} DimOS is running. Check its MCP endpoint and run logs."


@contextmanager
def _mcp_request(timeout: int | None = None) -> Iterator[McpAdapter]:
    adapter = _get_adapter(timeout)
    try:
        yield adapter
    except requests.Timeout:
        typer.echo(f"Error: MCP request to {adapter.url} timed out.", err=True)
        raise typer.Exit(1) from None
    except requests.ConnectionError:
        typer.echo(_connection_error(adapter.url), err=True)
        raise typer.Exit(1) from None
    except McpError as e:
        typer.echo(f"Error: {e}", err=True)
        raise typer.Exit(1) from None


@mcp_app.command("list-tools")
def mcp_list_tools() -> None:
    """List available MCP tools (skills)."""
    with _mcp_request() as adapter:
        tools = adapter.list_tools()
    typer.echo(json.dumps(tools, indent=2))


def _parse_key_value_arg(value: str) -> tuple[str, Any]:
    """Parse a KEY=VALUE argument, auto-converting JSON values."""
    if "=" not in value:
        raise ValueError(f"expected KEY=VALUE, got: {value}")
    key, val = value.split("=", 1)
    try:
        return (key, json.loads(val))
    except (json.JSONDecodeError, ValueError):
        return (key, val)


def _validate_key_value_args(values: list[str]) -> list[str]:
    """Validate KEY=VALUE arguments during CLI parsing."""
    for value in values:
        if "=" not in value:
            raise typer.BadParameter(f"expected KEY=VALUE, got: {value}")
    return values


@mcp_app.command("call")
def mcp_call_tool(
    tool_name: str = typer.Argument(..., help="Tool name to call"),
    args: list[str] = typer.Option(
        [],
        "--arg",
        "-a",
        # Avoid Typer's inspection of list[str] callback annotations on Python 3.10.
        callback=lambda values: _validate_key_value_args(values),
        help="Arguments as key=value",
    ),
    json_args: str = typer.Option("", "--json-args", "-j", help="Arguments as JSON string"),
    timeout: int = typer.Option(
        None, "--timeout", "-t", help="Seconds to wait for the tool (default: 30 seconds)"
    ),
) -> None:
    """Call an MCP tool by name."""
    arguments: dict[str, Any] = {}
    if json_args:
        try:
            arguments = json.loads(json_args)
        except json.JSONDecodeError as e:
            typer.echo(f"Error: invalid JSON in --json-args: {e}", err=True)
            raise typer.Exit(1)
    else:
        try:
            arguments = dict(_parse_key_value_arg(arg) for arg in args)
        except ValueError as e:
            typer.echo(f"Error: invalid --arg: {e}", err=True)
            raise typer.Exit(1)

    with _mcp_request(timeout) as adapter:
        result = adapter.call_tool(tool_name, arguments)

    content = result.get("content", [])
    if not content:
        typer.echo("(no output)")
        return
    for item in content:
        typer.echo(item.get("text", str(item)))


@mcp_app.command("status")
def mcp_status() -> None:
    """Show MCP server status (modules, skills)."""
    with _mcp_request() as adapter:
        data = adapter.call_tool_text("server_status")
    # server_status returns JSON string -- pretty-print it
    try:
        typer.echo(json.dumps(json.loads(data), indent=2))
    except (json.JSONDecodeError, ValueError):
        typer.echo(data)


@mcp_app.command("modules")
def mcp_modules() -> None:
    """List deployed modules and their skills."""
    with _mcp_request() as adapter:
        data = adapter.call_tool_text("list_modules")
    try:
        typer.echo(json.dumps(json.loads(data), indent=2))
    except (json.JSONDecodeError, ValueError):
        typer.echo(data)


def agent_send_cmd(
    message: str = typer.Argument(..., help="Message to send to the running agent"),
) -> None:
    """Send a message to the running DimOS agent via MCP."""
    with _mcp_request() as adapter:
        text = adapter.call_tool_text("agent_send", {"message": message})
    typer.echo(text)
