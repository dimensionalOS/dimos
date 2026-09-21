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

import json
from typing import Any

import requests
import typer

from dimos.agents.mcp.mcp_adapter import McpAdapter, McpError

mcp_app = typer.Typer(help="Interact with the running MCP server")


def _get_adapter(timeout: int | None = None) -> McpAdapter:
    """Get an McpAdapter from the latest RunEntry or default URL."""
    return McpAdapter.from_run_entry(timeout=timeout)


@mcp_app.command("list-tools")
def mcp_list_tools() -> None:
    """List available MCP tools (skills)."""
    try:
        tools = _get_adapter().list_tools()
    except requests.ConnectionError:
        typer.echo("Error: no running MCP server (is DimOS running?)", err=True)
        raise typer.Exit(1)
    except McpError as e:
        typer.echo(f"Error: {e}", err=True)
        raise typer.Exit(1)
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
        [], "--arg", "-a", callback=_validate_key_value_args, help="Arguments as key=value"
    ),
    json_args: str = typer.Option("", "--json-args", "-j", help="Arguments as JSON string"),
    timeout: int = typer.Option(
        None, "--timeout", "-t", help="Seconds to wait for the tool (default: mcp_timeout)"
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

    try:
        result = _get_adapter(timeout).call_tool(tool_name, arguments)
    except requests.ConnectionError:
        typer.echo("Error: no running MCP server (is DimOS running?)", err=True)
        raise typer.Exit(1)
    except McpError as e:
        typer.echo(f"Error: {e}", err=True)
        raise typer.Exit(1)

    content = result.get("content", [])
    if not content:
        typer.echo("(no output)")
        return
    for item in content:
        typer.echo(item.get("text", str(item)))


@mcp_app.command("status")
def mcp_status() -> None:
    """Show MCP server status (modules, skills)."""
    try:
        data = _get_adapter().call_tool_text("server_status")
    except requests.ConnectionError:
        typer.echo("Error: no running MCP server (is DimOS running?)", err=True)
        raise typer.Exit(1)
    except McpError as e:
        typer.echo(f"Error: {e}", err=True)
        raise typer.Exit(1)
    # server_status returns JSON string -- pretty-print it
    try:
        typer.echo(json.dumps(json.loads(data), indent=2))
    except (json.JSONDecodeError, ValueError):
        typer.echo(data)


@mcp_app.command("modules")
def mcp_modules() -> None:
    """List deployed modules and their skills."""
    try:
        data = _get_adapter().call_tool_text("list_modules")
    except requests.ConnectionError:
        typer.echo("Error: no running MCP server (is DimOS running?)", err=True)
        raise typer.Exit(1)
    except McpError as e:
        typer.echo(f"Error: {e}", err=True)
        raise typer.Exit(1)
    try:
        typer.echo(json.dumps(json.loads(data), indent=2))
    except (json.JSONDecodeError, ValueError):
        typer.echo(data)


def agent_send_cmd(
    message: str = typer.Argument(..., help="Message to send to the running agent"),
) -> None:
    """Send a message to the running DimOS agent via MCP."""
    try:
        text = _get_adapter().call_tool_text("agent_send", {"message": message})
    except requests.ConnectionError:
        typer.echo("Error: no running MCP server (is DimOS running?)", err=True)
        raise typer.Exit(1)
    except McpError as e:
        typer.echo(f"Error: {e}", err=True)
        raise typer.Exit(1)
    typer.echo(text)
