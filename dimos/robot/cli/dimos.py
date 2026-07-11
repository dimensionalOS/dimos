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

from collections.abc import Iterable
from datetime import datetime, timezone
import inspect
import json
import os
from pathlib import Path
import sys
import time
import types
from typing import TYPE_CHECKING, Any, Literal, Union, cast, get_args, get_origin

from dotenv import load_dotenv
from pydantic import BaseModel
from pydantic.fields import FieldInfo
from pydantic_core import PydanticUndefined
import requests
import typer

from dimos.agents.mcp.mcp_adapter import McpAdapter, McpError
from dimos.constants import CONFIG_DIR, LOG_DIR
from dimos.core.daemon import daemonize, install_signal_handlers
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.run_registry import get_most_recent, is_pid_alive, stop_entry
from dimos.mapping.utils.cli.map import main as _map_main
from dimos.mapping.utils.cli.pose_fill import main as _map_pose_fill_main
from dimos.mapping.utils.cli.rename import main as _map_rename_main
from dimos.mapping.utils.cli.replay import main as _map_replay_main
from dimos.mapping.utils.cli.replay_marker import main as _map_replay_marker_main
from dimos.robot.unitree.go2.cli.go2tool import app as go2tool_app
from dimos.utils.logging_config import setup_logger
from dimos.visualization.rerun.constants import RerunOpenOption

if TYPE_CHECKING:
    from dimos.core.coordination.blueprints import Blueprint, BlueprintAtom

logger = setup_logger()

main = typer.Typer(
    help="Dimensional CLI",
    no_args_is_help=True,
)

load_dotenv()

SIMULATORS = ("mujoco", "dimsim")


def _normalize_simulation_argv(argv: list[str]) -> list[str]:
    """Keep `--simulation` backwards compatible.

    Without an argument it should be `mujoco`, but can be overridden.
    """
    out: list[str] = []
    for arg, nxt in zip(argv, [*argv[1:], None], strict=False):
        out.append(arg)
        if arg == "--simulation" and nxt not in SIMULATORS:
            out.append(SIMULATORS[0])
    return out


def cli_main() -> None:
    sys.argv = _normalize_simulation_argv(sys.argv)
    main()


def create_dynamic_callback():  # type: ignore[no-untyped-def]
    fields = GlobalConfig.model_fields

    # Build the function signature dynamically
    params = [
        inspect.Parameter("ctx", inspect.Parameter.POSITIONAL_OR_KEYWORD, annotation=typer.Context),
    ]

    # Create parameters for each field in GlobalConfig
    for field_name, field_info in fields.items():
        field_type = field_info.annotation

        # Container generics (e.g. `tuple[...]` fields) have no single-flag CLI
        # representation; they're configured via env/JSON. Skip like arg_help does.
        if isinstance(field_type, types.GenericAlias):
            continue

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
        overrides = {k: v for k, v in kwargs.items() if v is not None}
        ctx.obj = overrides
        # Apply overrides (e.g. --transport, --viewer) to the process-global config
        # up front so every subcommand honors flags given before the subcommand name.
        global_config.update(**overrides)

    callback.__signature__ = inspect.Signature(params)  # type: ignore[attr-defined]

    return callback


main.callback()(create_dynamic_callback())  # type: ignore[no-untyped-call]
main.add_typer(go2tool_app, name="go2tool")


def arg_help(
    config: type[BaseModel],
    blueprint: Blueprint,
    indent: str = "    ",
    module: str = "",
    _atom: BlueprintAtom | None = None,
    _defaults: BaseModel | dict[str, Any] | None = None,
) -> str:
    output = ""
    for k, info in config.model_fields.items():
        if k == "g":
            continue
        t: object = info.annotation
        if isinstance(t, types.GenericAlias):
            # Can't be specified on CLI
            continue

        fallback = _field_default(info)
        field_defaults = _get_default_value(_defaults, k, fallback)
        t = _unwrap_base_model_annotation(t, field_defaults)

        if inspect.isclass(t) and issubclass(t, BaseModel):
            output += f"{indent}{module}{k}:\n"
            if _atom is None:
                # Root BlueprintConfig fields are blueprint atoms, except schema
                # branches such as transports.* that have no backing atom.
                bp = next((bp for bp in blueprint.blueprints if bp.module.name == k), None)
                defaults = bp.kwargs if bp is not None else field_defaults
            else:
                # Nested BaseModel fields belong to the current atom and must not
                # be atom-looked-up.
                bp = _atom
                defaults = field_defaults
            output += arg_help(
                t,
                blueprint,
                indent=indent + "  ",
                module=module + k + ".",
                _atom=bp,
                _defaults=defaults,
            )
        else:
            # Use __name__ to avoid "<class 'int'>" style output on basic types.
            display_type = t.__name__ if isinstance(t, type) else t
            has_default = _has_default_value(_defaults, k)
            required = "[Required] " if info.is_required() and not has_default else ""
            d = field_defaults
            default = f" (default: {d})" if d is not PydanticUndefined else ""
            output += f"{indent}* {required}{module}{k}: {display_type}{default}\n"
    return output


def _field_default(info: FieldInfo) -> Any:
    if info.default is not PydanticUndefined:
        return info.default
    if info.default_factory is not None:
        return info.get_default(call_default_factory=True)
    return PydanticUndefined


def _unwrap_base_model_annotation(annotation: object, defaults: object) -> object:
    # TODO(PY314): if isinstance(annotation, Union):
    if get_origin(annotation) not in {Union, types.UnionType}:
        return annotation

    candidates = tuple(
        u for u in get_args(annotation) if inspect.isclass(u) and issubclass(u, BaseModel)
    )
    if not candidates:
        return annotation
    return _select_base_model_candidate(candidates, defaults)


def _select_base_model_candidate(
    candidates: tuple[type[BaseModel], ...], defaults: object
) -> type[BaseModel]:
    backend = _backend_default(defaults)
    if backend is not PydanticUndefined:
        for candidate in candidates:
            backend_info = candidate.model_fields.get("backend")
            if backend_info is not None and _field_default(backend_info) == backend:
                return candidate
    return candidates[0]


def _backend_default(defaults: object) -> object:
    if isinstance(defaults, BaseModel):
        return getattr(defaults, "backend", PydanticUndefined)
    if isinstance(defaults, dict):
        return defaults.get("backend", PydanticUndefined)
    return PydanticUndefined


def _has_default_value(defaults: BaseModel | dict[str, Any] | None, key: str) -> bool:
    if isinstance(defaults, BaseModel):
        return key in defaults.model_fields_set
    if isinstance(defaults, dict):
        return key in defaults
    return False


def _get_default_value(defaults: object, key: str, fallback: Any) -> Any:
    if isinstance(defaults, BaseModel):
        if key in defaults.model_fields_set:
            return getattr(defaults, key)
    if isinstance(defaults, dict):
        return defaults.get(key, fallback)
    return fallback


def load_config_args(config: type[BaseModel], args: Iterable[str], path: Path) -> dict[str, Any]:
    try:
        kwargs = json.loads(path.read_text())
    except (OSError, json.JSONDecodeError):
        kwargs = {}

    for k, v in os.environ.items():
        parts = k.lower().split("__")
        if parts[0] not in config.model_fields:
            continue
        d = kwargs
        for p in parts[:-1]:
            d = d.setdefault(p, {})
        d[parts[-1]] = v

    for arg in args:
        k, _, v = arg.partition("=")
        parts = k.split(".")
        d = kwargs
        for p in parts[:-1]:
            d = d.setdefault(p, {})
        d[parts[-1]] = v

    # We don't need this config, but this atleast validates the user input first.
    # This will help catch misspellings and similar mistakes.
    config(**kwargs)

    return kwargs  # type: ignore[no-any-return]


@main.command()
def run(
    ctx: typer.Context,
    robot_types: list[str] = typer.Argument(..., help="Blueprints or modules to run"),
    daemon: bool = typer.Option(False, "--daemon", "-d", help="Run in background"),
    disable: list[str] = typer.Option([], "--disable", help="Module names to disable"),
    blueprint_args: list[str] = typer.Option((), "--option", "-o"),
    config_path: Path = typer.Option(
        CONFIG_DIR / "dimos", "--config", "-c", help="Path to config file"
    ),
    show_help: bool = typer.Option(False, "--help"),
) -> None:
    """Start a robot blueprint"""
    logger.info("Starting DimOS")

    from dimos.core.coordination.blueprints import autoconnect
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.core.coordination.process_lifecycle import (
        DIMOS_RUN_ID_ENV,
        spawn_watchdog,
    )
    from dimos.core.run_registry import (
        RunEntry,
        cleanup_stale,
        generate_run_id,
    )
    from dimos.robot.get_all_blueprints import get_by_name_or_exit, get_module_by_name_or_exit
    from dimos.utils.logging_config import set_run_log_dir, setup_exception_handler

    setup_exception_handler()

    cli_config_overrides: dict[str, Any] = ctx.obj

    # Clean stale registry entries
    stale = cleanup_stale()
    if stale:
        logger.info(f"Cleaned {stale} stale run entries")

    blueprint_name = "-".join(robot_types)
    run_id = generate_run_id(blueprint_name)
    log_dir = LOG_DIR / run_id

    # Tag every descendant with the run id so the watchdog and stale-run
    # cleanup can identify them via os.environ after main dies.
    os.environ[DIMOS_RUN_ID_ENV] = run_id

    # Route structured logs (main.jsonl) to the per-run directory.
    # Workers inherit DIMOS_RUN_LOG_DIR env var via forkserver.
    set_run_log_dir(log_dir)

    blueprint = autoconnect(*map(get_by_name_or_exit, robot_types))

    if disable:
        disabled_classes = tuple(
            get_module_by_name_or_exit(name).blueprints[0].module for name in disable
        )
        blueprint = blueprint.disabled_modules(*disabled_classes)

    if show_help:
        print("Blueprint arguments:")
        print("  Override with --option/-o module.field=value.")
        print("  Nested config paths use dotted names, e.g. module.nested.field=value.")
        print(arg_help(blueprint.config(), blueprint))
        return

    blueprint_config = blueprint.config()
    kwargs = load_config_args(blueprint_config, blueprint_args, config_path)
    if cli_config_overrides:
        kwargs["g"] = cli_config_overrides

    coordinator = ModuleCoordinator.build(blueprint, kwargs)

    if daemon:
        # Health check before daemonizing — catch early crashes
        if not coordinator.health_check():
            typer.echo("Error: health check failed — a worker process died.", err=True)
            coordinator.stop()
            raise typer.Exit(1)

        n_modules = coordinator.n_modules
        typer.echo(f"✓ All modules started ({n_modules} modules)")
        typer.echo("✓ Health check passed")
        typer.echo("✓ DimOS running in background\n")
        typer.echo(f"  Run ID:    {run_id}")
        typer.echo(f"  Log:       {log_dir}")
        typer.echo("  Stop:      dimos stop")
        typer.echo("  Status:    dimos status")

        coordinator.suppress_console()

        daemonize(log_dir)

        coordinator.start_rpc_service()  # After daemonize().
        entry = RunEntry(
            run_id=run_id,
            pid=os.getpid(),
            blueprint=blueprint_name,
            started_at=datetime.now(timezone.utc).isoformat(),
            log_dir=str(log_dir),
            cli_args=list(robot_types),
            config_overrides=cli_config_overrides,
            original_argv=sys.argv,
        )
        entry.save()
        spawn_watchdog(run_id, log_dir=log_dir)
        install_signal_handlers(entry, coordinator)
        coordinator.loop()
    else:
        coordinator.start_rpc_service()
        entry = RunEntry(
            run_id=run_id,
            pid=os.getpid(),
            blueprint=blueprint_name,
            started_at=datetime.now(timezone.utc).isoformat(),
            log_dir=str(log_dir),
            cli_args=list(robot_types),
            config_overrides=cli_config_overrides,
            original_argv=sys.argv,
        )
        entry.save()
        spawn_watchdog(run_id, log_dir=log_dir)
        # Foreground: only SIGTERM goes through the handler. SIGINT stays at
        # default so Ctrl+C raises KeyboardInterrupt and the try/finally below
        # runs with a visible traceback.
        install_signal_handlers(entry, coordinator, sigint=False)
        try:
            coordinator.loop()
        finally:
            entry.remove()


@main.command()
def status() -> None:
    """Show the running DimOS instance."""
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

    entry = get_most_recent(alive_only=True)
    if not entry:
        typer.echo("No running DimOS instance", err=True)
        raise typer.Exit(1)

    sig_name = "SIGKILL" if force else "SIGTERM"
    typer.echo(f"Stopping {entry.run_id} (PID {entry.pid}) with {sig_name}...")
    msg, _ok = stop_entry(entry, force=force)
    typer.echo(f"  {msg}")


@main.command("log")
def log_cmd(
    follow: bool = typer.Option(False, "--follow", "-f", help="Follow log output"),
    lines: int = typer.Option(50, "--lines", "-n", help="Number of lines to show"),
    all_lines: bool = typer.Option(False, "--all", "-a", help="Show full log"),
    json_output: bool = typer.Option(False, "--json", help="Raw JSONL output"),
    run_id: str = typer.Option("", "--run", "-r", help="Specific run ID"),
) -> None:
    """View logs from a DimOS run."""
    from dimos.core.log_viewer import follow_log, format_line, read_log, resolve_log_path

    path = resolve_log_path(run_id)
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
    force: bool = typer.Option(False, "--force", "-f", help="Force kill before restarting"),
) -> None:
    """Restart the running DimOS instance with the same arguments."""
    entry = get_most_recent(alive_only=True)
    if not entry:
        typer.echo("No running DimOS instance to restart", err=True)
        raise typer.Exit(1)

    if not entry.original_argv:
        typer.echo("Cannot restart: run entry missing original command", err=True)
        raise typer.Exit(1)

    # Save argv and pid before stopping (stop removes the entry)
    argv = entry.original_argv
    old_pid = entry.pid

    typer.echo(f"Restarting {entry.run_id} ({entry.blueprint})...")
    msg, _ok = stop_entry(entry, force=force)
    typer.echo(f"  {msg}")

    # Wait for the old process to fully exit so ports are released.
    for _ in range(20):  # up to 2s
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
def show_config() -> None:
    """Show current config settings and their values."""
    for field_name, value in global_config.model_dump().items():
        typer.echo(f"{field_name}: {value}")


@main.command(name="list")
def list_blueprints() -> None:
    """List all available blueprints."""
    from dimos.robot.all_blueprints import all_blueprints
    from dimos.robot.external_blueprints import (
        ExternalBlueprintError,
        list_external_blueprint_names,
    )

    blueprints = [name for name in all_blueprints.keys() if not name.startswith("demo-")]
    typer.echo("Built-in blueprints:")
    for blueprint_name in sorted(blueprints):
        typer.echo(f"  {blueprint_name}")

    try:
        external_blueprints = list_external_blueprint_names()
    except ExternalBlueprintError as exc:
        typer.echo(typer.style(str(exc), fg=typer.colors.RED), err=True)
        raise typer.Exit(1) from exc

    if external_blueprints:
        typer.echo("")
        typer.echo("External blueprints:")
        for blueprint_name in external_blueprints:
            typer.echo(f"  {blueprint_name}")


@main.command(context_settings={"allow_extra_args": True, "ignore_unknown_options": True})
def spy(ctx: typer.Context) -> None:
    """Universal transport spy: topics, rates, sizes across all pubsub transports."""
    # A root-level `--transport` (before the subcommand) sets the stack's pubsub
    # backend — which single transport dimos processes participate on. The spy is an
    # observer: it watches every transport and takes its own repeatable `--transport`
    # filter *after* the subcommand. The two look alike but mean different things, so
    # reject the root placement rather than silently ignoring the requested filter.
    if (ctx.obj or {}).get("transport") is not None:
        typer.echo(
            "Error: `--transport` before `spy` sets the stack backend, which the spy "
            "ignores. Put the filter after the subcommand: `dimos spy --transport <name>`.",
            err=True,
        )
        raise typer.Exit(2)
    from dimos.utils.cli.spy.run_spy import main as spy_main

    sys.argv = ["spy", *ctx.args]
    spy_main()


@main.command(context_settings={"allow_extra_args": True, "ignore_unknown_options": True})
def lcmspy(ctx: typer.Context) -> None:
    """Alias for `dimos spy --transport lcm`."""
    from dimos.utils.cli.spy.run_spy import lcm_only_argv, main as spy_main

    sys.argv = lcm_only_argv(list(ctx.args))
    spy_main()


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


map_app = typer.Typer(help="Voxel-map tools over recorded sqlite datasets")
main.add_typer(map_app, name="map")
map_app.command("global")(_map_main)


dataprep_app = typer.Typer(help="Build and inspect learning datasets from recordings")
main.add_typer(dataprep_app, name="dataprep")


@dataprep_app.command("build")
def dataprep_build(
    source: Path | None = typer.Option(None, "--source", "-s", help="Recording .db to read"),
    output: Path | None = typer.Option(None, "--output", help="Dataset output directory"),
    output_format: str = typer.Option(None, "--format", "-f", help="Output format: lerobot | hdf5"),
    config_path: Path | None = typer.Option(
        None, "--config", "-c", help="JSON DataPrepConfig (needed for obs/action stream maps)"
    ),
) -> None:
    """Build a dataset from a recording (lerobot/hdf5 + dimos_meta.json)."""
    from dimos.learning.dataprep.cli import build

    build(config_path, source, output, cast("Literal['lerobot', 'hdf5'] | None", output_format))


@dataprep_app.command("inspect")
def dataprep_inspect(
    dataset: Path | None = typer.Argument(
        None, help="Built dataset: a .hdf5 file or a lerobot directory"
    ),
    output_format: str = typer.Option(
        None, "--format", "-f", help="lerobot | hdf5 (auto-detected from the path if omitted)"
    ),
) -> None:
    """Summarize a built dataset: features, shapes, episode/frame counts, uniformity."""
    from dimos.learning.dataprep.cli import inspect

    inspect(dataset, cast("Literal['lerobot', 'hdf5'] | None", output_format))


map_app.command("rename")(_map_rename_main)
map_app.command("pose-fill")(_map_pose_fill_main)
map_app.command("replay")(_map_replay_main)
map_app.command("replay-marker")(_map_replay_marker_main)

from dimos.memory2.cli.app import mem_app

main.add_typer(mem_app, name="mem")


@main.command()
def cameracalibrate(
    source: str = typer.Option(..., "--source", help="Frame source: webcam, folder, or topic"),
    device_index: int = typer.Option(0, "--device-index", help="Webcam device index"),
    images: Path | None = typer.Option(
        None, "--images", help="Directory of calibration images for --source folder"
    ),
    topic: str | None = typer.Option(
        None,
        "--topic",
        help=(
            "Pubsub URI for --source topic (proto:channel), "
            "e.g. 'jpeg_lcm:/color_image' or 'pshm:color_image'."
        ),
    ),
    topic_timeout_sec: float = typer.Option(
        60.0,
        "--topic-timeout-sec",
        help="Abort --source topic if no frames arrive within this many seconds.",
    ),
    cols: int = typer.Option(..., "--cols", help="Inner chessboard corner columns"),
    rows: int = typer.Option(..., "--rows", help="Inner chessboard corner rows"),
    square_size_m: float = typer.Option(
        ..., "--square-size-m", help="Chessboard square size in meters"
    ),
    out: Path | None = typer.Option(None, "--out", help="Optional ROS CameraInfo YAML output path"),
    preview_out: Path | None = typer.Argument(
        None, help="Optional preview PNG output path. Requires --out."
    ),
    camera_name: str = typer.Option("webcam", "--camera-name", help="Camera name in YAML"),
    target_count: int = typer.Option(20, "--target-count", help="Accepted webcam frame count"),
    no_display: bool = typer.Option(False, "--no-display", help="Disable OpenCV preview windows"),
    distortion_model: str = typer.Option(
        "plumb_bob",
        "--distortion-model",
        help=(
            "Lens model: 'plumb_bob' (5 coeffs, near-pinhole) or 'fisheye' "
            "(4 coeffs, wide-angle / fisheye; written as ROS 'equidistant')."
        ),
    ),
) -> None:
    """Calibrate camera intrinsics and write ROS CameraInfo YAML."""
    from dimos.utils.cli.cameracalibrate.cameracalibrate import run_calibration

    if preview_out is not None and out is None:
        raise typer.BadParameter("preview output requires --out")

    try:
        result = run_calibration(
            source=source,
            device_index=device_index,
            images=images,
            topic=topic,
            topic_timeout_sec=topic_timeout_sec,
            cols=cols,
            rows=rows,
            square_size_m=square_size_m,
            out=out,
            preview_out=preview_out,
            camera_name=camera_name,
            target_count=target_count,
            no_display=no_display,
            distortion_model=distortion_model,
        )
    except (ValueError, RuntimeError) as exc:
        raise typer.BadParameter(str(exc)) from exc

    typer.echo(f"RMS: {float(result['rms']):.6f} px ({int(result['n_used'])} frame(s) used)")
    typer.echo(
        f"Detected pattern: {tuple(result.get('pattern_size', (cols, rows)))} "
        f"({result.get('pattern_label', 'requested inner corners')})"
    )
    if out is not None:
        typer.echo(f"Wrote camera info YAML to {out}")
    if preview_out is not None:
        typer.echo(f"Wrote preview overlay PNG to {preview_out}")


@main.command()
def apriltag(
    out: Path = typer.Option(Path("apriltags.pdf"), "--out", "-o", help="Output PDF path"),
    ids: str = typer.Option("0-11", "--ids", help="ID spec, e.g. '0-49' or '0,1,5,10-20'"),
    size_mm: float = typer.Option(
        50.0, "--size-mm", "-s", help="Tag black-border edge size in mm (typical: 50 or 100)"
    ),
    page_size: str = typer.Option(
        "a4", "--page-size", "-p", help="Page size: a0..a8 (ISO A series) or letter"
    ),
    pack: bool = typer.Option(
        True, "--pack/--no-pack", help="Pack as many tags per page as fit (vs one per page)"
    ),
    family: str = typer.Option(
        "tag36h11",
        "--family",
        help=(
            "Tag family: AprilTag (tag36h11, tag25h9, tag16h5) or "
            "ArUco (aruco_original, aruco_mip_36h12, aruco_{4x4,5x5,6x6,7x7}_{50,100,250,1000})"
        ),
    ),
) -> None:
    """Generate a printable AprilTag/ArUco PDF with calibration ruler."""
    from dimos.utils.cli.apriltag import generate_pdf, parse_id_spec

    id_list = parse_id_spec(ids)
    path = generate_pdf(
        id_list, out, family=family, size_mm=size_mm, page_size=page_size, pack=pack
    )
    typer.echo(f"Wrote {len(id_list)} tag(s) to {path}")


@main.command(name="rerun-bridge")
def rerun_bridge_cmd(
    memory_limit: str = typer.Option(
        "25%", help="Memory limit for Rerun viewer (e.g., '4GB', '16GB', '25%')"
    ),
    rerun_open: str = typer.Option("native", help="How to open Rerun: native, web, both, none"),
    rerun_web: bool = typer.Option(
        True, "--rerun-web/--no-rerun-web", help="Enable/Disable Rerun web server"
    ),
) -> None:
    """Launch the Rerun visualization bridge."""
    from dimos.visualization.rerun.bridge import run_bridge

    run_bridge(
        memory_limit=memory_limit,
        rerun_open=cast("RerunOpenOption", rerun_open),
        rerun_web=rerun_web,
    )


if __name__ == "__main__":
    cli_main()
