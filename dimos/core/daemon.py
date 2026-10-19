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

"""Daemonization and health-check support for DimOS processes."""

from __future__ import annotations

from collections.abc import Callable, Mapping
import json
import os
import signal
import sys
from typing import TYPE_CHECKING, Any

from dimos.core.coordination.process_lifecycle import DIMOS_RUN_ID_ENV, kill_run_processes
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from pathlib import Path

    from dimos.core.coordination.blueprint_config.parsed import ParsedBlueprintConfig
    from dimos.core.coordination.blueprints import Blueprint
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.core.module import ModuleBase

logger = setup_logger()


def health_check(coordinator: ModuleCoordinator) -> bool:
    """Verify all coordinator workers are alive after build.

    .. deprecated:: 0.1.0
        Use ``coordinator.health_check()`` directly.
    """
    return coordinator.health_check()


def fork_daemon(log_dir: Path) -> tuple[int, int]:
    """Double-fork into a daemon, keeping a status pipe to the launcher.

    Returns ``(daemon_pgid, read_fd)`` in the launcher and ``(0, write_fd)``
    in the daemon grandchild — test the first element like ``os.fork()``.
    The intermediate child calls ``setsid`` before forking again, so
    ``daemon_pgid`` covers the grandchild and everything it spawns.

    Building must happen in the grandchild: zenoh's process-global runtime
    does not survive fork, so the daemon must not inherit any open sessions.
    The grandchild keeps the launcher's stdio so startup output still reaches
    the terminal; it must call ``redirect_stdio_to_devnull`` and
    ``write_daemon_status`` once startup succeeds or fails.
    """
    log_dir.mkdir(parents=True, exist_ok=True)
    # Anything buffered would otherwise be flushed by both processes.
    sys.stdout.flush()
    sys.stderr.flush()
    read_fd, write_fd = os.pipe()

    pid = os.fork()
    if pid > 0:
        os.close(write_fd)
        os.waitpid(pid, 0)  # The intermediate exits immediately; don't leave a zombie.
        return pid, read_fd

    os.close(read_fd)
    os.setsid()

    # Second fork — can never reacquire a controlling terminal
    if os.fork() > 0:
        os._exit(0)
    return 0, write_fd


def redirect_stdio_to_devnull() -> None:
    """Point stdin/stdout/stderr at /dev/null — logging goes to ``main.jsonl``."""
    sys.stdout.flush()
    sys.stderr.flush()

    devnull = open(os.devnull)
    os.dup2(devnull.fileno(), sys.stdin.fileno())
    os.dup2(devnull.fileno(), sys.stdout.fileno())
    os.dup2(devnull.fileno(), sys.stderr.fileno())
    devnull.close()


def write_daemon_status(fd: int, status: dict[str, Any]) -> None:
    """Send one JSON status line to the launcher; tolerate a vanished reader."""
    try:
        os.write(fd, (json.dumps(status) + "\n").encode())
    except BrokenPipeError:
        pass


def read_daemon_status(fd: int) -> dict[str, Any] | None:
    """Read the daemon's status line; ``None`` if it died before reporting."""
    with os.fdopen(fd) as pipe:
        line = pipe.readline()
    try:
        return json.loads(line)  # type: ignore[no-any-return]
    except json.JSONDecodeError:
        return None


def install_signal_handlers(
    run_id: str,
    coordinator: ModuleCoordinator,
    *,
    sigint: bool = True,
) -> None:
    """Install SIGTERM/SIGINT handlers that stop the coordinator, sweep the run and exit.

    When `sigint` is False, only SIGTERM is wired. This is useful for foreground
    mode where the default SIGINT behavior (`KeyboardInterrupt`) should be
    preserved so Ctrl+C produces a visible traceback.
    """

    def _shutdown(signum: int, frame: object) -> None:
        logger.info("Received signal, shutting down", signal=signum)
        try:
            coordinator.stop()
        except Exception:
            logger.error("Error during coordinator stop", exc_info=True)
        try:
            kill_run_processes(run_id)
        except Exception:
            logger.error("Error during run-process sweep", exc_info=True)
        sys.exit(0)

    signal.signal(signal.SIGTERM, _shutdown)
    if sigint:
        signal.signal(signal.SIGINT, _shutdown)


def run_coordinator(
    blueprint: Blueprint,
    config: ParsedBlueprintConfig,
    *,
    run_id: str,
    log_dir: str | os.PathLike[str],
    rpc_name: str | None = None,
    remote_module_refs: Mapping[tuple[str, str], tuple[type[ModuleBase] | frozenset[str], str]]
    | None = None,
    sigint: bool = True,
    on_ready: Callable[[ModuleCoordinator], None] | None = None,
) -> None:
    """Build, serve RPC, call ``on_ready`` and loop until a signal; the caller owns any registry entry."""
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.core.coordination.process_lifecycle import spawn_watchdog
    from dimos.memory.tap import recording

    os.environ[DIMOS_RUN_ID_ENV] = run_id
    coordinator = ModuleCoordinator.build(blueprint, config, remote_module_refs=remote_module_refs)
    try:
        if not coordinator.health_check():
            raise RuntimeError("health check failed, a worker process died")
        coordinator.start_rpc_service(name=rpc_name)
    except BaseException:
        coordinator.stop()
        raise
    spawn_watchdog(run_id, log_dir=log_dir)
    install_signal_handlers(run_id, coordinator, sigint=sigint)
    try:
        if on_ready is not None:
            on_ready(coordinator)
        with recording(coordinator.transports):
            coordinator.loop()
    except Exception:
        coordinator.stop()
        raise
