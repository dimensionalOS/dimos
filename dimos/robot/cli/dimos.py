# Copyright 2025-2026 Dimensional Inc.
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

from __future__ import annotations

import inspect
import json
import sys
from typing import Any, get_args, get_origin

import click
from dotenv import load_dotenv
import requests
import typer

from dimos.agents.mcp.mcp_adapter import McpAdapter, McpError
from dimos.core.global_config import GlobalConfig, global_config
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

main = typer.Typer(
    help="Dimensional CLI",
    no_args_is_help=True,
)

load_dotenv()


def create_dynamic_callback():  # type: ignore[no-untyped-def]
    fields = GlobalConfig.model_fields

    # Build the function signature dynamically
    params = [
        inspect.Parameter("ctx", inspect.Parameter.POSITIONAL_OR_KEYWORD, annotation=typer.Context),
    ]

    # Create parameters for each field in GlobalConfig
    for field_name, field_info in fields.items():
        field_type = field_info.annotation

        # Handle Optional types
        # Check for Optional/Union with None
        if get_origin(field_type) is type(str | None):
            inner_types = get_args(field_type)
            if len(inner_types) == 2 and type(None) in inner_types:
                # It's Optional[T], get the actual type T
                actual_type = next(t for t in inner_types if t != type(None))
            else:
                actual_type = field_type
        else:
            actual_type = field_type

        # Convert field name from snake_case to kebab-case for CLI
        cli_option_name = field_name.replace("_", "-")

        # Special handling for boolean fields
        if actual_type is bool:
            # For boolean fields, create --flag/--no-flag pattern
            param = inspect.Parameter(
                field_name,
                inspect.Parameter.KEYWORD_ONLY,
                default=typer.Option(
                    None,  # None means use the model's default if not provided
                    f"--{cli_option_name}/--no-{cli_option_name}",
                    help=f"Override {field_name} in GlobalConfig",
                ),
                annotation=bool | None,
            )
        else:
            # For non-boolean fields, use regular option
            param = inspect.Parameter(
                field_name,
                inspect.Parameter.KEYWORD_ONLY,
                default=typer.Option(
                    None,  # None means use the model's default if not provided
                    f"--{cli_option_name}",
                    help=f"Override {field_name} in GlobalConfig",
                ),
                annotation=actual_type | None,
            )
        params.append(param)

    def callback(**kwargs) -> None:  # type: ignore[no-untyped-def]
        ctx = kwargs.pop("ctx")
        ctx.obj = {k: v for k, v in kwargs.items() if v is not None}

    callback.__signature__ = inspect.Signature(params)  # type: ignore[attr-defined]

    return callback


main.callback()(create_dynamic_callback())  # type: ignore[no-untyped-call]


@main.command()
def run(
    ctx: typer.Context,
    robot_types: list[str] = typer.Argument(..., help="Blueprints or modules to run"),
    daemon: bool = typer.Option(False, "--daemon", "-d", help="Run in background"),
) -> None:
    """Start a robot blueprint"""
    from datetime import datetime, timezone
    import os

    logger.info("Starting DimOS")

    from dimos.core.blueprints import autoconnect
    from dimos.core.run_registry import (
        LOG_BASE_DIR,
        RunEntry,
        check_port_conflicts,
        cleanup_stale,
        generate_run_id,
    )
    from dimos.robot.get_all_blueprints import get_by_name
    from dimos.utils.logging_config import set_run_log_dir, setup_exception_handler

    setup_exception_handler()

    cli_config_overrides: dict[str, Any] = ctx.obj
    global_config.update(**cli_config_overrides)

    # Clean stale registry entries
    stale = cleanup_stale()
    if stale:
        logger.info(f"Cleaned {stale} stale run entries")

    # Port conflict check
    conflict = check_port_conflicts()
    if conflict:
        typer.echo(
            f"Error: Ports in use by {conflict.run_id} (PID {conflict.pid}). "
            f"Run 'dimos stop' first.",
            err=True,
        )
        raise typer.Exit(1)

    blueprint_name = "-".join(robot_types)
    run_id = generate_run_id(blueprint_name)
    log_dir = LOG_BASE_DIR / run_id

    # Route structured logs (main.jsonl) to the per-run directory.
    # Workers inherit DIMOS_RUN_LOG_DIR env var via forkserver.
    set_run_log_dir(log_dir)

    blueprint = autoconnect(*map(get_by_name, robot_types))
    coordinator = blueprint.build(cli_config_overrides=cli_config_overrides)

    if daemon:
        from dimos.core.daemon import (
            daemonize,
            install_signal_handlers,
        )

        # Health check before daemonizing — catch early crashes
        if not coordinator.health_check():
            typer.echo("Error: health check failed — a worker process died.", err=True)
            coordinator.stop()
            raise typer.Exit(1)

        n_workers = coordinator.n_workers
        n_modules = coordinator.n_modules
        typer.echo(f"✓ All modules started ({n_modules} modules, {n_workers} workers)")
        typer.echo("✓ Health check passed")
        typer.echo("✓ DimOS running in background\n")
        typer.echo(f"  Run ID:    {run_id}")
        typer.echo(f"  Log:       {log_dir}")
        typer.echo("  Stop:      dimos stop")
        typer.echo("  Status:    dimos status")

        daemonize(log_dir)

        entry = RunEntry(
            run_id=run_id,
            pid=os.getpid(),
            blueprint=blueprint_name,
            started_at=datetime.now(timezone.utc).isoformat(),
            log_dir=str(log_dir),
            cli_args=list(robot_types),
            config_overrides=cli_config_overrides,
        )
        entry.save()
        install_signal_handlers(entry, coordinator)
        coordinator.loop()
    else:
        entry = RunEntry(
            run_id=run_id,
            pid=os.getpid(),
            blueprint=blueprint_name,
            started_at=datetime.now(timezone.utc).isoformat(),
            log_dir=str(log_dir),
            cli_args=list(robot_types),
            config_overrides=cli_config_overrides,
        )
        entry.save()
        try:
            coordinator.loop()
        finally:
            entry.remove()


@main.command()
def status() -> None:
    """Show the running DimOS instance."""
    from datetime import datetime, timezone

    from dimos.core.run_registry import get_most_recent

    entry = get_most_recent(alive_only=True)
    if not entry:
        typer.echo("No running DimOS instance")
        return

    try:
        started = datetime.fromisoformat(entry.started_at)
        age = datetime.now(timezone.utc) - started
        hours, remainder = divmod(int(age.total_seconds()), 3600)
        minutes, seconds = divmod(remainder, 60)
        uptime = f"{hours}h {minutes}m" if hours > 0 else f"{minutes}m {seconds}s"
    except Exception:
        uptime = "unknown"

    typer.echo(f"  Run ID:    {entry.run_id}")
    typer.echo(f"  PID:       {entry.pid}")
    typer.echo(f"  Blueprint: {entry.blueprint}")
    typer.echo(f"  Uptime:    {uptime}")
    typer.echo(f"  Log:       {entry.log_dir}")


@main.command()
def stop(
    force: bool = typer.Option(False, "--force", "-f", help="Force kill (SIGKILL)"),
) -> None:
    """Stop the running DimOS instance."""

    from dimos.core.run_registry import get_most_recent

    entry = get_most_recent(alive_only=True)
    if not entry:
        typer.echo("No running DimOS instance", err=True)
        raise typer.Exit(1)

    from dimos.core.run_registry import stop_entry

    sig_name = "SIGKILL" if force else "SIGTERM"
    typer.echo(f"Stopping {entry.run_id} (PID {entry.pid}) with {sig_name}...")
    msg, _ok = stop_entry(entry, force=force)
    typer.echo(f"  {msg}")


# ---------------------------------------------------------------------------
# MCP subcommands
# ---------------------------------------------------------------------------

mcp_app = typer.Typer(help="Interact with the running MCP server")
main.add_typer(mcp_app, name="mcp")


def _get_adapter() -> McpAdapter:
    """Get an McpAdapter from the latest RunEntry or default URL."""
    from dimos.agents.mcp.mcp_adapter import McpAdapter

    return McpAdapter.from_run_entry()


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


class _KeyValueType(click.ParamType):
    """Parse KEY=VALUE arguments, auto-converting JSON values."""

    name = "KEY=VALUE"

    def convert(
        self, value: str, param: click.Parameter | None, ctx: click.Context | None
    ) -> tuple[str, Any]:
        if "=" not in value:
            self.fail(f"expected KEY=VALUE, got: {value}", param, ctx)
        key, val = value.split("=", 1)
        try:
            return (key, json.loads(val))
        except (json.JSONDecodeError, ValueError):
            return (key, val)


@mcp_app.command("call")
def mcp_call_tool(
    tool_name: str = typer.Argument(..., help="Tool name to call"),
    args: list[str] = typer.Option(
        [], "--arg", "-a", click_type=_KeyValueType(), help="Arguments as key=value"
    ),
    json_args: str = typer.Option("", "--json-args", "-j", help="Arguments as JSON string"),
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
        # _KeyValueType.convert() returns (key, val) tuples at runtime
        arguments = dict(args)  # type: ignore[arg-type]

    try:
        result = _get_adapter().call_tool(tool_name, arguments)
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


@main.command("agent-send")
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


@main.command()
def show_config(ctx: typer.Context) -> None:
    """Show current config settings and their values."""

    cli_config_overrides: dict[str, Any] = ctx.obj
    global_config.update(**cli_config_overrides)

    for field_name, value in global_config.model_dump().items():
        typer.echo(f"{field_name}: {value}")


@main.command(name="list")
def list_blueprints() -> None:
    """List all available blueprints."""
    from dimos.robot.all_blueprints import all_blueprints

    blueprints = [name for name in all_blueprints.keys() if not name.startswith("demo-")]
    for blueprint_name in sorted(blueprints):
        typer.echo(blueprint_name)


@main.command(context_settings={"allow_extra_args": True, "ignore_unknown_options": True})
def lcmspy(ctx: typer.Context) -> None:
    """LCM spy tool for monitoring LCM messages."""
    from dimos.utils.cli.lcmspy.run_lcmspy import main as lcmspy_main

    sys.argv = ["lcmspy", *ctx.args]
    lcmspy_main()


@main.command(context_settings={"allow_extra_args": True, "ignore_unknown_options": True})
def agentspy(ctx: typer.Context) -> None:
    """Agent spy tool for monitoring agents."""
    from dimos.utils.cli.agentspy.agentspy import main as agentspy_main

    sys.argv = ["agentspy", *ctx.args]
    agentspy_main()


@main.command(context_settings={"allow_extra_args": True, "ignore_unknown_options": True})
def humancli(ctx: typer.Context) -> None:
    """Interface interacting with agents."""
    from dimos.utils.cli.human.humanclianim import main as humancli_main

    sys.argv = ["humancli", *ctx.args]
    humancli_main()


@main.command(context_settings={"allow_extra_args": True, "ignore_unknown_options": True})
def top(ctx: typer.Context) -> None:
    """Live resource monitor TUI."""
    from dimos.utils.cli.dtop import main as dtop_main

    sys.argv = ["dtop", *ctx.args]
    dtop_main()


topic_app = typer.Typer(help="Topic commands for pub/sub")
main.add_typer(topic_app, name="topic")


@topic_app.command()
def echo(
    topic: str = typer.Argument(..., help="Topic name to listen on (e.g., /goal_request)"),
    type_name: str | None = typer.Argument(
        None,
        help="Optional message type (e.g., PoseStamped). If omitted, infer from '/topic#pkg.Msg'.",
    ),
) -> None:
    from dimos.robot.cli.topic import topic_echo

    topic_echo(topic, type_name)


@topic_app.command()
def send(
    topic: str = typer.Argument(..., help="Topic name to send to (e.g., /goal_request)"),
    message_expr: str = typer.Argument(..., help="Python expression for the message"),
) -> None:
    from dimos.robot.cli.topic import topic_send

    topic_send(topic, message_expr)


@main.command(name="rerun-bridge")
def rerun_bridge_cmd(
    viewer_mode: str = typer.Option(
        "native", help="Viewer mode: native (desktop), web (browser), none (headless)"
    ),
    memory_limit: str = typer.Option(
        "25%", help="Memory limit for Rerun viewer (e.g., '4GB', '16GB', '25%')"
    ),
) -> None:
    """Launch the Rerun visualization bridge."""
    from dimos.visualization.rerun.bridge import run_bridge

    run_bridge(viewer_mode=viewer_mode, memory_limit=memory_limit)


if __name__ == "__main__":
    main()
