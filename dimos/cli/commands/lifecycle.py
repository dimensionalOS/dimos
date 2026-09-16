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

"""Robot process lifecycle commands: run, status, stop, restart, and log."""

from __future__ import annotations

from collections.abc import Mapping
import contextlib
from dataclasses import dataclass
from datetime import datetime, timezone
import os
from pathlib import Path
import signal
import sys
import time
import traceback
from typing import TYPE_CHECKING, Any

from pydantic import ValidationError
import typer

from dimos.constants import CONFIG_DIR, LOG_DIR
from dimos.core.daemon import (
    fork_daemon,
    install_signal_handlers,
    read_daemon_status,
    redirect_stdio_to_devnull,
    write_daemon_status,
)
from dimos.core.global_config import SECRET_CONFIG_FIELDS, GlobalConfig, global_config
from dimos.core.run_registry import get_most_recent, is_pid_alive, stop_entry
from dimos.deps.catalog import CatalogError, catalog_names, default_catalog
from dimos.deps.launch import (
    OFFLINE_ENV,
    Decision,
    LaunchError,
    exec_into,
    hold_current_lease,
    select_environment,
)
from dimos.deps.planning import RunPlan, plan_run
from dimos.deps.selectors import collect_selector_inputs
from dimos.utils.cache import cache_usage_guard
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.coordination.blueprints import Blueprint

logger = setup_logger()

DEFAULT_CONFIG_PATH = CONFIG_DIR / "config"


@dataclass(frozen=True)
class RunRequest:
    """What a run-like command asked for, resolved before any blueprint is imported."""

    blueprint_names: tuple[str, ...]
    config_tokens: tuple[str, ...]
    global_overrides: dict[str, Any]
    """Root flags plus the command's own global-config flags."""
    global_values: dict[str, Any]
    """Every GlobalConfig value after config file, environment and flags."""
    config_sections: Mapping[str, Any]
    """Raw contents of the config file; the planner reads its module sections for selections."""

    @property
    def planning_values(self) -> dict[str, Any]:
        """Global values plus the derived properties the catalog selects on."""
        return GlobalConfig.planning_values(self.global_values)


def resolve_run_request(
    ctx: typer.Context,
    robot_types: list[str],
    config_path: Path,
    run_overrides: Mapping[str, Any] | None = None,
) -> RunRequest:
    """Split blueprint names from config tokens and preparse the global config.

    Shared by ``run``, ``deps``, ``doctor`` and ``prepare`` so every command
    resolves the same names under the same configuration precedence.
    """
    from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
    from dimos.core.coordination.blueprint_config.parser import (
        BlueprintConfigParser,
        split_run_arguments,
    )
    from dimos.core.coordination.blueprint_config.sources import read_config_file

    try:
        blueprint_names, config_tokens = split_run_arguments(robot_types)
        config_sections = read_config_file(config_path)
    except BlueprintConfigError as error:
        typer.echo(f"Error: {error}", err=True)
        raise typer.Exit(2) from error

    global_overrides: dict[str, Any] = dict(ctx.obj or {})
    if run_overrides:
        global_overrides.update(run_overrides)
    try:
        global_values = BlueprintConfigParser.preparse_global_config(
            config_tokens,
            config_path=config_path,
            environ=os.environ,
            global_overrides=global_overrides,
        )
    except BlueprintConfigError as error:
        typer.echo(f"Error: {error}", err=True)
        raise typer.Exit(2) from error
    return RunRequest(
        blueprint_names, config_tokens, global_overrides, global_values, config_sections
    )


def plan_request(request: RunRequest, profile: str | None) -> RunPlan:
    """Plan the request from the catalog, with the selections it can read without a schema."""
    inputs = collect_selector_inputs(
        request.config_tokens,
        request.config_sections,
        os.environ,
        default_catalog().selector_fields(),
    )
    return plan_run(request.blueprint_names, request.planning_values, profile, inputs)


def apply_global_values(values: Mapping[str, Any]) -> None:
    """Make the preparsed global config visible to blueprint modules."""
    try:
        global_config.update(**values)
    except ValidationError as error:
        typer.echo(f"Error: {error.errors()[0]['msg']}", err=True)
        raise typer.Exit(2) from error


def public_config(values: Mapping[str, Any]) -> dict[str, Any]:
    """Global values without secrets, safe to hand to another process or a file."""
    return {key: value for key, value in values.items() if key not in SECRET_CONFIG_FIELDS}


def _echo_error(message: str) -> None:
    typer.echo(message, err=True)


def _reject_legacy_config() -> None:
    """~/.config/dimos used to BE the config file; it is now a directory."""
    if CONFIG_DIR.is_file():
        typer.echo(
            f"config found at old path {CONFIG_DIR}, which is now a directory; move it:\n"
            f"  mv {CONFIG_DIR} {CONFIG_DIR}.tmp && mkdir {CONFIG_DIR}"
            f" && mv {CONFIG_DIR}.tmp {CONFIG_DIR}/config",
            err=True,
        )
        raise typer.Exit(2)


def _with_relay_bridge(blueprint: Blueprint) -> Blueprint:
    """Append one relay bridge to an enabled CLI run after blueprint resolution."""
    if not (global_config.local_relay or global_config.relay_url):
        return blueprint

    try:
        from dimos.web.relay_bridge.relay_bridge_module import with_relay_bridge
    except ImportError as e:
        raise RuntimeError(
            "--local-relay/--relay-url need the web extra: `uv sync --extra web --inexact`"
        ) from e

    return with_relay_bridge(blueprint)


def run(
    ctx: typer.Context,
    robot_types: list[str] = typer.Argument(..., help="Blueprints or modules to run"),
    daemon: bool = typer.Option(False, "--daemon", "-d", help="Run in background"),
    disable: list[str] = typer.Option([], "--disable", help="Module names to disable"),
    config_path: Path = typer.Option(
        DEFAULT_CONFIG_PATH, "--config", "-c", help="Path to config file"
    ),
    local_relay: bool | None = typer.Option(
        None,
        "--local-relay/--no-local-relay",
        help="Spawn a local cockpit relay and bridge this robot to it",
    ),
    relay_url: str | None = typer.Option(
        None,
        "--relay-url",
        help="Bridge this robot to a relay started elsewhere (its HTTP URL, e.g. "
        "http://localhost:7780)",
    ),
    relay_ca: str | None = typer.Option(
        None,
        "--relay-ca",
        help="PEM CA bundle that signed the relay's certificate (mkcert, a private CA)",
    ),
    environment: str = typer.Option(
        "auto",
        "--environment",
        help="Where to run: auto, current, managed, or the path of a virtualenv with dimos",
    ),
    profile: str | None = typer.Option(
        None, "--profile", help="Hardware profile of the managed environment (default: detect)"
    ),
    offline: bool = typer.Option(
        False, "--offline", help="Require a prepared environment; never download anything"
    ),
    show_help: bool = typer.Option(False, "--help"),
) -> None:
    """Start a robot blueprint"""

    # Log this at the start so that people get immediate feedback that the program has started.
    logger.info("Starting DimOS")

    if config_path == DEFAULT_CONFIG_PATH:
        _reject_legacy_config()
    from dimos.utils.logging_config import setup_exception_handler

    setup_exception_handler()

    # These flags are accepted on `run` itself, not just as global options.
    run_overrides = {
        name: value
        for name, value in (
            ("local_relay", local_relay),
            ("relay_url", relay_url),
            ("relay_ca", relay_ca),
        )
        if value is not None
    }
    request = resolve_run_request(ctx, robot_types, config_path, run_overrides)
    # Some blueprint modules select their composition at import time, so all
    # global sources must be visible before resolving the requested names.
    apply_global_values(request.global_values)

    if offline:
        os.environ.update(OFFLINE_ENV)
    # The lease on the managed environment the run executes from lives as long
    # as this process; a daemon keeps it through the fork, an exec hands it over.
    with contextlib.ExitStack() as resources:
        if not show_help:
            with cache_usage_guard():
                decision = _select_runtime(request, environment, profile, offline)
            if decision is not None and decision.dimos_executable is not None:
                assert decision.env_dir is not None
                try:
                    exec_into(
                        decision.dimos_executable,
                        decision.env_dir,
                        sys.argv,
                        offline=offline,
                        lease=decision.lease,
                    )
                except LaunchError as error:
                    typer.echo(f"Error: {error}", err=True)
                    raise typer.Exit(error.exit_code) from error
            lease = hold_current_lease(echo=_echo_error)
            if lease is not None:
                resources.enter_context(lease)

        with cache_usage_guard():
            _start(
                ctx,
                request,
                daemon=daemon,
                disable=disable,
                config_path=config_path,
                show_help=show_help,
            )


def _select_runtime(
    request: RunRequest, environment: str, profile: str | None, offline: bool
) -> Decision | None:
    """Plan the run and pick its interpreter; ``None`` continues in this one."""
    builtin = [name for name in request.blueprint_names if "." not in name]
    if any(name not in catalog_names() for name in builtin):
        # Unknown names are reported with suggestions by the resolver.
        return None
    try:
        planned = plan_request(request, profile)
    except (CatalogError, ValueError) as error:
        typer.echo(f"Error: {error}", err=True)
        raise typer.Exit(2) from error
    if planned.unsupported is not None and environment == "auto":
        logger.warning(
            "No tested hardware profile for this host; running in the current environment",
            reason=str(planned.unsupported),
        )
        return None
    try:
        return select_environment(
            planned,
            environment=environment,
            offline=offline,
            blueprints=planned.builtin_names,
            global_config=public_config(request.global_values),
            echo=_echo_error,
        )
    except LaunchError as error:
        typer.echo(f"Error: {error}", err=True)
        raise typer.Exit(error.exit_code) from error


def _start(
    ctx: typer.Context,
    request: RunRequest,
    *,
    daemon: bool,
    disable: list[str],
    config_path: Path,
    show_help: bool,
) -> None:
    """Build and run the blueprint in this interpreter."""
    from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
    from dimos.core.coordination.blueprint_config.parser import BlueprintConfigParser
    from dimos.core.coordination.blueprints import autoconnect
    from dimos.core.coordination.module_coordinator import ModuleCoordinator, stream_name_types
    from dimos.core.coordination.process_lifecycle import (
        DIMOS_RUN_ID_ENV,
        spawn_watchdog,
    )
    from dimos.core.run_registry import (
        RunEntry,
        cleanup_stale,
        generate_run_id,
    )
    from dimos.memory.tap import check_topics, recording
    from dimos.robot.get_all_blueprints import get_by_name_or_exit, get_module_by_name_or_exit
    from dimos.utils.logging_config import set_run_log_dir

    blueprint_names = request.blueprint_names
    config_tokens = request.config_tokens
    global_option_overrides = request.global_overrides

    blueprint = autoconnect(*map(get_by_name_or_exit, blueprint_names))

    if disable:
        disabled_classes = tuple(
            get_module_by_name_or_exit(name).blueprints[0].module for name in disable
        )
        blueprint = blueprint.disabled_modules(*disabled_classes)

    blueprint = _with_relay_bridge(blueprint)
    if global_config.record:
        try:
            check_topics(global_config.record_topics, {n for n, _ in stream_name_types(blueprint)})
        except ValueError as error:
            typer.echo(f"Error: {error}", err=True)
            raise typer.Exit(2) from error
    parser = BlueprintConfigParser(blueprint)

    if show_help:
        reserved_options = {
            option
            for parameter in ctx.command.params
            for option in (
                *getattr(parameter, "opts", ()),
                *getattr(parameter, "secondary_opts", ()),
            )
            if option.startswith("--")
        }
        typer.echo(ctx.get_help())
        typer.echo()
        typer.echo(parser.format_help(reserved_options))
        return

    try:
        parsed_config = parser.parse(
            config_tokens,
            config_path=config_path,
            environ=os.environ,
            global_overrides=global_option_overrides,
        )
    except BlueprintConfigError as error:
        typer.echo(f"Error: {error}", err=True)
        raise typer.Exit(2) from error

    # Clean stale registry entries only after the full command has validated.
    stale = cleanup_stale()
    if stale:
        logger.info(f"Cleaned {stale} stale run entries")

    blueprint_name = "-".join(blueprint_names)
    run_id = generate_run_id(blueprint_name)
    log_dir = LOG_DIR / run_id

    # Tag every descendant with the run id so the watchdog and stale-run
    # cleanup can identify them via os.environ after main dies.
    os.environ[DIMOS_RUN_ID_ENV] = run_id

    # Route structured logs (main.jsonl) to the per-run directory.
    # Workers inherit DIMOS_RUN_LOG_DIR env var via forkserver.
    set_run_log_dir(log_dir)

    if daemon:
        # Fork before building: zenoh's process-global runtime does not survive
        # fork, so the daemon must open every session itself (issue #3395).
        daemon_pgid, status_fd = fork_daemon(log_dir)

        if daemon_pgid:
            # Launcher: wait for the daemon to report build/health outcome. Its
            # build output streams to this terminal via the inherited stdio.
            try:
                status = read_daemon_status(status_fd)
            except KeyboardInterrupt:
                try:
                    os.killpg(daemon_pgid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                typer.echo("Interrupted; daemon startup aborted.", err=True)
                raise typer.Exit(130) from None
            if not status or not status.get("ok"):
                message = (status or {}).get("error") or "daemon exited during startup"
                typer.echo(f"Error: {message}", err=True)
                raise typer.Exit(1)

            typer.echo(f"✓ All modules started ({status['n_modules']} modules)")
            typer.echo("✓ Health check passed")
            typer.echo("✓ DimOS running in background\n")
            typer.echo(f"  Run ID:    {run_id}")
            typer.echo(f"  Log:       {log_dir}")
            typer.echo("  Stop:      dimos stop")
            typer.echo("  Status:    dimos status")
            return

        # Daemon grandchild — stdio still attached so build output streams.
        coordinator = None
        try:
            coordinator = ModuleCoordinator.build(blueprint, parsed_config)
            if not coordinator.health_check():
                write_daemon_status(
                    status_fd,
                    {"ok": False, "error": "health check failed — a worker process died."},
                )
                coordinator.stop()
                os._exit(1)
            # Workers dup2 /dev/null over the terminal fds they inherited.
            coordinator.suppress_console()
            # Idempotent with loop(); serving now means the success status below
            # guarantees Coordinator RPC is actually reachable.
            coordinator.start_rpc_service()
            entry = RunEntry(
                run_id=run_id,
                pid=os.getpid(),
                blueprint=blueprint_name,
                started_at=datetime.now(timezone.utc).isoformat(),
                log_dir=str(log_dir),
                cli_args=list(blueprint_names),
                config_overrides=global_option_overrides,
                original_argv=sys.argv,
                environment=sys.prefix,
            )
            entry.save()
            spawn_watchdog(run_id, log_dir=log_dir)
            install_signal_handlers(entry, coordinator)
            redirect_stdio_to_devnull()
        except Exception as exc:
            traceback.print_exc()
            write_daemon_status(status_fd, {"ok": False, "error": f"{type(exc).__name__}: {exc}"})
            if coordinator is not None:
                try:
                    coordinator.stop()
                except Exception:
                    logger.error("Error stopping coordinator", exc_info=True)
            sys.stderr.flush()
            # os._exit: never unwind the launcher's typer/atexit state in a
            # forked image.
            os._exit(1)
        write_daemon_status(status_fd, {"ok": True, "n_modules": coordinator.n_modules})
        os.close(status_fd)
        # The launcher's exit released the pre-fork cache-usage marker (shared
        # flock); hold a fresh one for the daemon's lifetime.
        try:
            with cache_usage_guard(), recording(coordinator.transports):
                coordinator.loop()
        except Exception:
            coordinator.stop()
            raise
    else:
        coordinator = ModuleCoordinator.build(blueprint, parsed_config)
        entry = RunEntry(
            run_id=run_id,
            pid=os.getpid(),
            blueprint=blueprint_name,
            started_at=datetime.now(timezone.utc).isoformat(),
            log_dir=str(log_dir),
            cli_args=list(blueprint_names),
            config_overrides=global_option_overrides,
            original_argv=sys.argv,
            environment=sys.prefix,
        )
        entry.save()
        spawn_watchdog(run_id, log_dir=log_dir)
        # Foreground: only SIGTERM goes through the handler. SIGINT stays at
        # default so Ctrl+C raises KeyboardInterrupt and the try/finally below
        # runs with a visible traceback.
        install_signal_handlers(entry, coordinator, sigint=False)
        try:
            with recording(coordinator.transports):
                coordinator.loop()
        except Exception:
            coordinator.stop()
            raise
        finally:
            entry.remove()


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
    if entry.environment:
        typer.echo(f"  Env:       {entry.environment}")


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
