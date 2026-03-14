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

from datetime import datetime, timezone
import inspect
import json
import os
import sys
import time
from typing import Any, get_args, get_origin

import click
from dotenv import load_dotenv
import requests
import typer

from dimos.agents.mcp.mcp_adapter import McpAdapter, McpError
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.instance_registry import (
    InstanceInfo,
    get,
    get_sole_running,
    is_pid_alive,
    list_running,
    make_run_dir,
    register,
    stop as registry_stop,
    unregister,
)
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


def _resolve_name(name: str | None) -> InstanceInfo:
    """Resolve an instance name.  If None, use sole running instance."""
    if name:
        info = get(name)
        if info is None:
            typer.echo(f"No running instance named '{name}'", err=True)
            raise typer.Exit(1)
        return info
    info = get_sole_running()
    if info is None:
        typer.echo("No running DimOS instance", err=True)
        raise typer.Exit(1)
    return info


@main.command()
def run(
    ctx: typer.Context,
    robot_types: list[str] = typer.Argument(..., help="Blueprints or modules to run"),
    daemon: bool = typer.Option(
        False, "--daemon", "-d", help="Run as daemon (always daemonizes; without -d stays attached)"
    ),
    detach: bool = typer.Option(
        False, "--detach", help="Exit CLI after successful build (implies --daemon)"
    ),
    name: str = typer.Option("", "--name", help="Global instance name (default: blueprint name)"),
    force_replace: bool = typer.Option(
        False, "--force-replace", help="Auto-stop existing instance with same name"
    ),
    disable: list[str] = typer.Option([], "--disable", help="Module names to disable"),
) -> None:
    """Start a robot blueprint."""
    logger.info("Starting DimOS")

    cli_config_overrides: dict[str, Any] = ctx.obj

    if daemon or detach:
        # Daemon path: spawn + double-fork
        # launch_blueprint handles everything: config, existing instance,
        # run_dir creation, config snapshot, and spawning the daemon.
        from dimos.core.daemon import attached_tail, launch_blueprint

        result = launch_blueprint(
            robot_types=list(robot_types),
            config_overrides=cli_config_overrides,
            instance_name=name or None,
            force_replace=force_replace,
            disable=list(disable) if disable else None,
        )

        if not detach:
            # Attached mode: tail stdout.log, forward ctrl+c as SIGTERM
            exit_code = attached_tail(result.run_dir / "stdout.log", result.instance_name)
            raise typer.Exit(exit_code)

        typer.echo(f"Launched {result.instance_name}")
        typer.echo(f"  Run dir: {result.run_dir}")
        typer.echo("  Stop:    dimos stop")
        raise typer.Exit(0)
    else:
        # Foreground path
        from dimos.core.blueprints import autoconnect
        from dimos.robot.get_all_blueprints import get_by_name, get_module_by_name
        from dimos.utils.logging_config import set_run_log_dir, setup_exception_handler

        setup_exception_handler()
        global_config.update(**cli_config_overrides)

        blueprint_name = "-".join(robot_types)
        instance_name = name or blueprint_name

        # Check for existing instance
        existing = get(instance_name)
        if existing is not None:
            if force_replace:
                typer.echo(f"Stopping existing instance '{instance_name}' (PID {existing.pid})...")
                msg, _ok = registry_stop(instance_name)
                typer.echo(f"  {msg}")
                for _ in range(20):
                    if not is_pid_alive(existing.pid):
                        break
                    time.sleep(0.1)
            elif sys.stdin.isatty():
                typer.echo(
                    f"Instance '{instance_name}' is already running (PID {existing.pid}). "
                    f"Stop it and launch new? [y/N] ",
                    nl=False,
                )
                answer = input().strip().lower()
                if answer not in ("y", "yes"):
                    raise typer.Exit(0)
                msg, _ok = registry_stop(instance_name)
                typer.echo(f"  {msg}")
                for _ in range(20):
                    if not is_pid_alive(existing.pid):
                        break
                    time.sleep(0.1)
            else:
                typer.echo(
                    f"Error: Instance '{instance_name}' already running (PID {existing.pid}). "
                    f"Use --force-replace to auto-stop.",
                    err=True,
                )
                raise typer.Exit(1)

        run_dir = make_run_dir(instance_name)
        set_run_log_dir(run_dir)

        config_snapshot = run_dir / "config.json"
        config_snapshot.write_text(json.dumps(global_config.model_dump(mode="json"), indent=2))

        blueprint = autoconnect(*map(get_by_name, robot_types))

        if disable:
            disabled_classes = tuple(get_module_by_name(d).blueprints[0].module for d in disable)
            blueprint = blueprint.disabled_modules(*disabled_classes)

        coordinator = blueprint.build(cli_config_overrides=cli_config_overrides)

        info = InstanceInfo(
            name=instance_name,
            pid=os.getpid(),
            blueprint=blueprint_name,
            started_at=datetime.now(timezone.utc).isoformat(),
            run_dir=str(run_dir),
            grpc_port=global_config.grpc_port if hasattr(global_config, "grpc_port") else 9877,
            original_argv=sys.argv,
            config_overrides=cli_config_overrides,
        )
        register(info)
        try:
            coordinator.loop()
        finally:
            unregister(instance_name)


@main.command()
def status(
    name: str = typer.Argument("", help="Instance name (optional)"),
) -> None:
    """Show running DimOS instance(s)."""
    if name:
        info = get(name)
        if not info:
            typer.echo(f"No running instance named '{name}'")
            return
        _print_instance(info)
        return

    running = list_running()
    if not running:
        typer.echo("No running DimOS instances")
        return

    for info in running:
        _print_instance(info)
        if len(running) > 1:
            typer.echo("")


def _print_instance(info: InstanceInfo) -> None:
    try:
        started = datetime.fromisoformat(info.started_at)
        age = datetime.now(timezone.utc) - started
        hours, remainder = divmod(int(age.total_seconds()), 3600)
        minutes, seconds = divmod(remainder, 60)
        uptime = f"{hours}h {minutes}m" if hours > 0 else f"{minutes}m {seconds}s"
    except Exception:
        uptime = "unknown"

    typer.echo(f"  Name:      {info.name}")
    typer.echo(f"  PID:       {info.pid}")
    typer.echo(f"  Blueprint: {info.blueprint}")
    typer.echo(f"  Uptime:    {uptime}")
    typer.echo(f"  Run dir:   {info.run_dir}")


@main.command()
def stop(
    name: str = typer.Argument("", help="Instance name (optional)"),
    force: bool = typer.Option(False, "--force", "-f", help="Force kill (SIGKILL)"),
) -> None:
    """Stop a running DimOS instance."""
    if name:
        info = get(name)
        if not info:
            typer.echo(f"No running instance named '{name}'", err=True)
            raise typer.Exit(1)
    else:
        running = list_running()
        if len(running) == 0:
            typer.echo("No running DimOS instances", err=True)
            raise typer.Exit(1)
        if len(running) > 1:
            typer.echo("Multiple instances running. Specify a name:", err=True)
            for r in running:
                typer.echo(f"  {r.name} (PID {r.pid}, blueprint: {r.blueprint})")
            raise typer.Exit(1)
        info = running[0]

    sig_name = "SIGKILL" if force else "SIGTERM"
    typer.echo(f"Stopping {info.name} (PID {info.pid}) with {sig_name}...")
    msg, _ok = registry_stop(info.name, force=force)
    typer.echo(f"  {msg}")


@main.command("log")
def log_cmd(
    name: str = typer.Argument("", help="Instance name (optional)"),
    follow: bool = typer.Option(False, "--follow", "-f", help="Follow log output"),
    lines: int = typer.Option(50, "--lines", "-n", help="Number of lines to show"),
    all_lines: bool = typer.Option(False, "--all", "-a", help="Show full log"),
    json_output: bool = typer.Option(False, "--json", help="Raw JSONL output"),
    run_datetime: str = typer.Option("", "--run", "-r", help="Specific run datetime"),
) -> None:
    """View logs from a DimOS run."""
    from dimos.core.log_viewer import follow_log, format_line, read_log, resolve_log_path

    path = resolve_log_path(name=name, run_datetime=run_datetime)
    if not path:
        typer.echo("No log files found", err=True)
        raise typer.Exit(1)

    if follow:
        import signal

        _stop = False

        def _on_sigint(_sig: int, _frame: object) -> None:
            nonlocal _stop
            _stop = True

        prev = signal.signal(signal.SIGINT, _on_sigint)
        try:
            for line in follow_log(path, stop=lambda: _stop):
                typer.echo(format_line(line, json_output=json_output))
        finally:
            signal.signal(signal.SIGINT, prev)
    else:
        count = None if all_lines else lines
        for line in read_log(path, count):
            typer.echo(format_line(line, json_output=json_output))


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
def restart(
    name: str = typer.Argument("", help="Instance name (optional)"),
    force: bool = typer.Option(False, "--force", "-f", help="Force kill before restarting"),
) -> None:
    """Restart a running DimOS instance with the same arguments."""
    info = _resolve_name(name or None)

    if not info.original_argv:
        typer.echo("Cannot restart: instance missing original command", err=True)
        raise typer.Exit(1)

    argv = info.original_argv
    old_pid = info.pid
    instance_name = info.name

    typer.echo(f"Restarting {instance_name} ({info.blueprint})...")
    msg, _ok = registry_stop(instance_name, force=force)
    typer.echo(f"  {msg}")

    for _ in range(20):
        if not is_pid_alive(old_pid):
            break
        time.sleep(0.1)

    typer.echo(f"  Running: {' '.join(argv)}")
    try:
        os.execvp(argv[0], argv)
    except OSError as exc:
        typer.echo(f"Error: failed to restart — {exc}", err=True)
        raise typer.Exit(1)


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


@main.command()
def dio(
    debug: bool = typer.Option(False, "--debug", help="Show debug panel with key event log"),
) -> None:
    """Launch the DimOS Unified TUI."""
    from dimos.utils.cli.dio.app import main as dio_main

    if debug:
        sys.argv.append("--debug")
    dio_main()


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
