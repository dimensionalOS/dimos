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
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import traceback
from typing import TYPE_CHECKING, Any

import psutil
from pydantic import ValidationError
import typer

from dimos.constants import CONFIG_DIR, LOG_DIR
from dimos.core.coordination.coordinator_rpc import CoordinatorAlreadyRunningError
from dimos.core.daemon import (
    DAEMON_STATUS_FD_ENV,
    fork_daemon,
    python_command,
    read_daemon_status,
    redirect_stdio_to_devnull,
    run_coordinator,
    write_daemon_status,
)
from dimos.core.global_config import global_config
from dimos.core.run_registry import (
    get_most_recent,
    is_pid_alive,
    stop_entry,
    without_secret_options,
)
from dimos.utils.cache import cache_usage_guard, cache_usage_locked
from dimos.utils.logging_config import setup_logger
from dimos.web.relay_bridge.topic_descriptors import with_topic_metadata

if TYPE_CHECKING:
    from collections.abc import Callable, Mapping

    from dimos.core.coordination.blueprint_config.parsed import ParsedBlueprintConfig
    from dimos.core.coordination.blueprints import Blueprint
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.core.run_registry import RunEntry

logger = setup_logger()

DEFAULT_CONFIG_PATH = CONFIG_DIR / "config"
_RESTART_ENV = "_DIMOS_RESTART_PREVIOUS_RUN"


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


def _run_registered(
    blueprint: Blueprint,
    config: ParsedBlueprintConfig,
    entry: RunEntry,
    *,
    sigint: bool = True,
    on_ready: Callable[[ModuleCoordinator], None] | None = None,
) -> None:
    """run_coordinator with ``entry`` in the registry from readiness until exit."""

    def ready(coordinator: ModuleCoordinator) -> None:
        entry.mcp_url = _mcp_url(blueprint, coordinator)
        entry.save()
        if on_ready is not None:
            on_ready(coordinator)

    try:
        run_coordinator(
            blueprint,
            config,
            run_id=entry.run_id,
            log_dir=entry.log_dir,
            sigint=sigint,
            on_ready=ready,
        )
    finally:
        entry.remove()


def _with_relay_bridge(blueprint: Blueprint) -> Blueprint:
    """Append one relay bridge to an enabled CLI run after blueprint resolution."""
    if global_config.topic_metadata:
        blueprint = with_topic_metadata(blueprint)
    if not (global_config.local_relay or global_config.relay_url):
        return blueprint

    try:
        from dimos.web.relay_bridge.relay_bridge_module import with_relay_bridge
    except ImportError as e:
        raise RuntimeError(
            "--local-relay/--relay-url need the web extra: `uv sync --extra web --inexact`"
        ) from e

    return with_relay_bridge(blueprint)


def _run_hosted(
    blueprint: Blueprint,
    entry: RunEntry,
    overrides: Mapping[str, Any],
    global_overrides: Mapping[str, Any],
) -> None:
    """Place the Blueprint on the live Hosts and stream their logs until Ctrl-C or SIGTERM."""
    import threading

    from dimos.hosted.client import root_rpc
    from dimos.hosted.deploy import deployed, log_printer, wait_for_hosts
    from dimos.hosted.machine import local_host
    from dimos.protocol.service.zenohservice import split_csv

    local = local_host()
    connect = split_csv(global_config.zenoh_connect) or ([local.client_endpoint] if local else [])
    if not connect:
        typer.echo(
            "Error: no local Host is running: `dimos host start`, or --zenoh-connect", err=True
        )
        raise typer.Exit(2)
    named = {p.host for p in blueprint.hosted_placements if isinstance(p.host, str)}
    if local is not None:
        named.add(local.host_id)
    tag_sets = [p.tags for p in blueprint.hosted_placements if p.tags]
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    try:
        with root_rpc(connect) as rpc:
            descriptors = wait_for_hosts(rpc, named, tag_sets=tag_sets)
            with deployed(
                blueprint,
                rpc,
                run_id=entry.run_id,
                local_host_id=local.host_id if local else None,
                descriptors=descriptors,
                overrides=overrides,
                global_overrides=global_overrides,
                on_log=log_printer(),
            ) as placement:
                for module, host in sorted(placement.items()):
                    typer.echo(f"{host:>16}  {module}")
                typer.echo("Running; Ctrl-C stops every fragment")
                entry.save()
                try:
                    stop.wait()
                except KeyboardInterrupt:
                    pass
    except (RuntimeError, TimeoutError, ValueError) as exc:
        typer.echo(f"Error: {exc}", err=True)
        raise typer.Exit(1) from exc
    finally:
        entry.remove()


def _mcp_url(blueprint: Blueprint, coordinator: ModuleCoordinator) -> str | None:
    for atom in blueprint.active_blueprints:
        if "mcp_url" in atom.module.rpcs:
            return str(coordinator.get_instance(atom.name).mcp_url())
    return None


@cache_usage_locked
def run(
    ctx: typer.Context,
    robot_types: list[str] = typer.Argument(..., help="Blueprints or modules to run"),
    daemon: bool = typer.Option(False, "--daemon", "-d", help="Run in background"),
    disable: list[str] = typer.Option([], "--disable", help="Module names to disable"),
    choice: list[str] = typer.Option(
        [], "--choice", help="Select a blueprint alternative as NAME=OPTION; may be repeated"
    ),
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
    show_help: bool = typer.Option(False, "--help"),
) -> None:
    """Start a robot blueprint"""

    working_directory = os.getcwd()
    directory_stat = os.stat(".")
    command, source = python_command()
    # Log this at the start so that people get immediate feedback that the program has started.
    logger.info("Starting DimOS")

    if config_path == DEFAULT_CONFIG_PATH:
        _reject_legacy_config()
    from dimos.core.coordination.blueprint_config.errors import BlueprintConfigError
    from dimos.core.coordination.blueprint_config.parser import (
        BlueprintConfigParser,
        split_run_arguments,
    )
    from dimos.core.coordination.blueprints import ChoiceResolutionError, autoconnect
    from dimos.core.coordination.module_coordinator import (
        check_module_dependencies,
        stream_name_types,
    )
    from dimos.core.coordination.process_lifecycle import DIMOS_RUN_ID_ENV
    from dimos.core.run_registry import RunEntry, cleanup_stale, generate_run_id
    from dimos.memory.tap import check_topics
    from dimos.robot.get_all_blueprints import get_by_name_or_exit, get_module_by_name_or_exit
    from dimos.utils.logging_config import set_run_log_dir, setup_exception_handler

    setup_exception_handler()

    try:
        blueprint_names, config_tokens = split_run_arguments(robot_types)
        choices: dict[str, str] = {}
        for selection in choice:
            name, separator, option = selection.partition("=")
            if not separator or not name or not option:
                raise BlueprintConfigError(f"Invalid --choice {selection!r}: expected NAME=OPTION")
            if name in choices:
                raise BlueprintConfigError(f"Choice {name!r} was specified more than once")
            choices[name] = option
    except BlueprintConfigError as error:
        typer.echo(f"Error: {error}", err=True)
        raise typer.Exit(2) from error

    global_option_overrides: dict[str, Any] = dict(ctx.obj or {})

    # These flags are accepted on `run` itself, not just as global options.
    run_overrides = {
        name: value
        for name, value in (
            ("local_relay", local_relay),
            ("relay_url", relay_url),
        )
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
    try:
        global_config.update(**preparsed_global_config)
    except ValidationError as error:
        typer.echo(f"Error: {error.errors()[0]['msg']}", err=True)
        raise typer.Exit(2) from error

    try:
        blueprint = autoconnect(*map(get_by_name_or_exit, blueprint_names)).solidify(
            choices=choices
        )
    except ChoiceResolutionError as error:
        typer.echo(f"Error: {error}", err=True)
        raise typer.Exit(2) from error

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

    # Hosted fragments check on their executing host, where dependencies live.
    if not blueprint.hosted_placements:
        try:
            check_module_dependencies(blueprint, parsed_config)
        except ImportError as error:
            typer.echo(f"Error: {error}", err=True)
            raise typer.Exit(1) from error

    # Clean stale registry entries only after the full command has validated.
    stale = cleanup_stale()
    if stale:
        logger.info(f"Cleaned {stale} stale run entries")

    blueprint_name = "-".join(blueprint_names)
    # A daemon executes its wrapper again in the background. Only the final
    # invocation can confirm which project the wrapper actually entered.
    if not daemon or DAEMON_STATUS_FD_ENV in os.environ:
        _stop_previous_run(blueprint_name, directory_stat)
    # A daemon re-execs this command. Preserve the launcher's run/log
    # identity rather than generating a second one in the fresh interpreter.
    inherited_run_id = (
        os.environ.get(DIMOS_RUN_ID_ENV) if daemon and DAEMON_STATUS_FD_ENV in os.environ else None
    )
    run_id = inherited_run_id or generate_run_id(blueprint_name)
    log_dir = LOG_DIR / run_id

    # Tag every descendant with the run id so the watchdog and stale-run
    # cleanup can identify them via os.environ after main dies.
    os.environ[DIMOS_RUN_ID_ENV] = run_id

    # Route structured logs (main.jsonl) to the per-run directory.
    # Workers inherit DIMOS_RUN_LOG_DIR env var via forkserver.
    set_run_log_dir(log_dir)

    entry = RunEntry(
        run_id=run_id,
        pid=os.getpid(),
        blueprint=blueprint_name,
        started_at=datetime.now(timezone.utc).isoformat(),
        log_dir=str(log_dir),
        cli_args=list(blueprint_names),
        config_overrides=global_option_overrides,
        original_argv=[source, *sys.argv[1:]],
        working_directory=working_directory,
        working_directory_id=[directory_stat.st_dev, directory_stat.st_ino],
        python_argv=command if source not in ("", "-", "-c") else [],
    )
    if blueprint.hosted_placements:
        if daemon:
            typer.echo("Error: a hosted blueprint runs in the foreground", err=True)
            raise typer.Exit(2)
        _run_hosted(
            blueprint,
            entry,
            parsed_config.file_and_cli,
            global_option_overrides,
        )
        return

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

        # Daemon grandchild: stdio still attached so build output streams.
        started = False

        def ready(coordinator: ModuleCoordinator) -> None:
            nonlocal started
            # Workers dup2 /dev/null over the terminal fds they inherited.
            coordinator.suppress_console()
            write_daemon_status(status_fd, {"ok": True, "n_modules": coordinator.n_modules})
            os.close(status_fd)
            redirect_stdio_to_devnull()
            started = True

        entry.pid = os.getpid()
        try:
            # The launcher's exit released the pre-fork cache-usage marker (shared
            # flock); hold a fresh one for the daemon's lifetime.
            with cache_usage_guard():
                _run_registered(blueprint, parsed_config, entry, on_ready=ready)
        except Exception as exc:
            if started:
                raise
            if isinstance(exc, CoordinatorAlreadyRunningError):
                startup_error = str(exc)
            else:
                traceback.print_exc()
                startup_error = f"{type(exc).__name__}: {exc}"
            write_daemon_status(status_fd, {"ok": False, "error": startup_error})
            sys.stderr.flush()
            # os._exit: never unwind the launcher's typer/atexit state in a
            # forked image.
            os._exit(1)
    else:
        try:
            _run_registered(blueprint, parsed_config, entry)
        except CoordinatorAlreadyRunningError as exc:
            typer.echo(f"Error: {exc}", err=True)
            raise typer.Exit(1) from exc


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
    lines: int = typer.Option(50, "--lines", "-n", min=0, help="Number of lines to show"),
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

    count = None if all_lines else lines
    if follow:
        _stop = False

        def _on_sigint(_sig: int, _frame: object) -> None:
            nonlocal _stop
            _stop = True

        prev = signal.signal(signal.SIGINT, _on_sigint)
        try:
            for line in follow_log(path, stop=lambda: _stop, count=count):
                typer.echo(format_line(line, json_output=json_output))
        finally:
            signal.signal(signal.SIGINT, prev)
    else:
        for line in read_log(path, count):
            typer.echo(format_line(line, json_output=json_output))


def _check_module_source(argv: list[str], source: Path) -> None:
    # Only inspect Python options. A script or an option value can itself contain "-m".
    args = iter(enumerate(argv[1:], 1))
    for position, arg in args:
        if arg == "--check-hash-based-pycs":
            next(args, None)
            continue
        if not arg.startswith("-") or arg in ("-", "--"):
            return
        for offset, flag in enumerate(arg[1:], 1):
            if flag not in "WXcm":
                continue
            value = arg[offset + 1 :] or next(args, (0, ""))[1]
            if flag in "WX":
                break
            if flag == "c":
                return
            # Ask the saved interpreter with its own flags, cwd and import path.
            # Resolving a package with -m selects its __main__ submodule.
            options = argv[:position]
            if offset > 1:
                options.append(arg[:offset])
            probe = (
                "from importlib.util import find_spec\n"
                "import os, sys\n"
                "name, source = sys.argv[1:]\n"
                "spec = find_spec(name)\n"
                "if spec and spec.submodule_search_locations is not None:\n"
                "    spec = find_spec(name + '.__main__')\n"
                "if spec and spec.origin and os.path.samefile(spec.origin, source):\n"
                "    os.write(1, b'\\0')"
            )
            result = subprocess.run(
                [*options, "-c", probe, value, str(source)],
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
            )
            if result.returncode or not result.stdout.endswith(b"\0"):
                raise OSError(
                    f"the original Python module {value!r} cannot be resolved to {source}"
                )
            return


def _stop_previous_run(blueprint: str, directory: os.stat_result) -> None:
    pending = os.environ.pop(_RESTART_ENV, None)
    if pending is None:
        return
    run_id, pid, device, inode, force = json.loads(pending)
    if (directory.st_dev, directory.st_ino) != (device, inode):
        typer.echo("Cannot restart: the launcher entered a different working directory", err=True)
        raise typer.Exit(1)
    entry = get_most_recent(alive_only=True)
    if entry is None or (entry.run_id, entry.pid, entry.blueprint) != (run_id, pid, blueprint):
        typer.echo("Cannot restart: the running instance or blueprint has changed", err=True)
        raise typer.Exit(1)
    msg, ok = stop_entry(entry, force=force)
    typer.echo(f"  {msg}")

    if not ok:
        raise typer.Exit(1)

    # Wait for the old process to fully exit so ports are released.
    for _ in range(20):  # up to 2s
        if not is_pid_alive(pid):
            break
        time.sleep(0.1)


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

    source = entry.original_argv[0]
    if source in ("", "-", "-c"):
        typer.echo("Cannot restart a run launched from standard input or inline code", err=True)
        raise typer.Exit(1)

    old_pid = entry.pid
    try:
        # A released registry contains only sys.argv. Read its missing context
        # while the process is still alive, then apply the usual argument filter.
        directory = entry.working_directory or psutil.Process(old_pid).cwd()
        os.chdir(directory)
        current = os.stat(".")
        if entry.working_directory_id is not None and entry.working_directory_id != [
            current.st_dev,
            current.st_ino,
        ]:
            raise OSError("the original launch directory was replaced")
        source_path = Path(source)
        candidates = (
            [source_path / "__main__.py", source_path / "__main__.pyc"]
            if source_path.is_dir()
            else [source_path]
        )
        if not any(path.is_file() and os.access(path, os.R_OK) for path in candidates):
            raise OSError(f"original Python source is unavailable: {source}")
        argv = (
            without_secret_options(psutil.Process(old_pid).cmdline())
            if entry.python_argv is None
            else entry.python_argv
        )
        if not argv or shutil.which(argv[0]) is None:
            raise OSError("original command is unavailable")
        _check_module_source(argv, source_path)
    except (OSError, psutil.Error) as exc:
        typer.echo(f"Cannot restart: {exc}", err=True)
        raise typer.Exit(1) from exc

    typer.echo(f"Restarting {entry.run_id} ({entry.blueprint})...")
    typer.echo(f"  Running: {' '.join(argv)}")
    try:
        os.environ[_RESTART_ENV] = json.dumps(
            [entry.run_id, old_pid, current.st_dev, current.st_ino, force]
        )
        os.execvp(argv[0], argv)
    except OSError as exc:
        typer.echo(f"Error: failed to restart — {exc}", err=True)
        raise typer.Exit(1)
    finally:
        os.environ.pop(_RESTART_ENV, None)
