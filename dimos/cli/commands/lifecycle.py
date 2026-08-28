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

from datetime import datetime, timezone
import os
from pathlib import Path
import signal
import sys
import time
import traceback
from typing import TYPE_CHECKING, Any

import typer

from dimos.constants import CONFIG_DIR, LOG_DIR
from dimos.core.daemon import (
    fork_daemon,
    install_signal_handlers,
    read_daemon_status,
    redirect_stdio_to_devnull,
    write_daemon_status,
)
from dimos.core.global_config import global_config
from dimos.core.run_registry import get_most_recent, is_pid_alive, stop_entry
from dimos.utils.cache import cache_usage_guard, cache_usage_locked
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.coordination.blueprints import Blueprint

logger = setup_logger()

DEFAULT_CONFIG_PATH = CONFIG_DIR / "config"


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


@cache_usage_locked
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
        None, "--relay-url", help="Bridge this robot to a running relay (wtUrl)"
    ),
    show_help: bool = typer.Option(False, "--help"),
) -> None:
    """Start a robot blueprint"""

    # Log this at the start so that people get immediate feedback that the program has started.
    logger.info("Starting DimOS")

    if config_path == DEFAULT_CONFIG_PATH:
        _reject_legacy_config()
    from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
    from dimos.core.coordination.blueprint_config.parser import (
        BlueprintConfigParser,
        split_run_arguments,
    )
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
    from dimos.utils.logging_config import set_run_log_dir, setup_exception_handler

    setup_exception_handler()

    try:
        blueprint_names, config_tokens = split_run_arguments(robot_types)
    except BlueprintConfigError as error:
        typer.echo(f"Error: {error}", err=True)
        raise typer.Exit(2) from error

    global_option_overrides: dict[str, Any] = dict(ctx.obj or {})

    # These flags are accepted on `run` itself, not just as global options.
    run_overrides = {
        name: value
        for name, value in (("local_relay", local_relay), ("relay_url", relay_url))
        if value is not None
    }
    if run_overrides:
        global_option_overrides.update(run_overrides)

    try:
        preparsed_global_config = BlueprintConfigParser.preparse_global_config(
            config_tokens,
            config_path=config_path,
            environ=os.environ,
            global_overrides=global_option_overrides,
        )
    except BlueprintConfigError as error:
        typer.echo(f"Error: {error}", err=True)
        raise typer.Exit(2) from error
    # Some blueprint modules select their composition at import time, so all
    # global sources must be visible before resolving the requested names.
    global_config.update(**preparsed_global_config)
    # Workers re-import blueprint modules before they receive the global config;
    # what shapes a module class at import (the replay blueprint's ports) goes via env.
    os.environ["REPLAY_DB"] = global_config.replay_db

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
