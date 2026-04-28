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

"""Daemonization support for DimOS processes.

Architecture: subprocess + double-fork.

Caller (DIO or CLI)
  |- Compute instance_name, create run_dir
  |- Write launch_params.json to run_dir
  |- subprocess.Popen([python, -m, dimos.core.daemon, run_dir],
  |      start_new_session=True, stdin=DEVNULL, stdout=DEVNULL, stderr=DEVNULL)
  |- Return LaunchResult(instance_name, run_dir) immediately
  |
  +- SUBPROCESS (__main__):
      |- Read launch_params.json
      |- Double-fork to become daemon
      |- Redirect stdin -> /dev/null, stdout/stderr -> stdout.log
      |- Start OutputTee -> stdout.log + stdout.plain.log
      |- Apply config_overrides to global_config
      |- Import and build blueprint
      |- health_check()
      |- register(InstanceInfo) -> writes current.json
      |- install_signal_handlers()
      +- coordinator.loop() <- blocks forever

DIO reads stdout.log directly from run_dir. CLI uses attached_tail() which
also reads stdout.log. No pipe between caller and daemon.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time
from typing import TYPE_CHECKING

from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.core.instance_registry import InstanceInfo
    from dimos.core.output_tee import OutputTee

logger = setup_logger()


@dataclass
class LaunchResult:
    """Returned by launch_blueprint() with info about the launched instance."""

    instance_name: str
    run_dir: Path


def launch_blueprint(
    robot_types: list[str],
    config_overrides: dict[str, object] | None = None,
    instance_name: str | None = None,
    force_replace: bool = False,
    disable: list[str] | None = None,
) -> LaunchResult:
    """Launch a blueprint as a daemon process.

    Creates run_dir, writes launch_params.json, spawns a fresh Python
    process via subprocess that double-forks into a daemon.  Returns
    immediately.  Works identically from CLI and TUI.

    Parameters
    ----------
    robot_types:
        Blueprint names to autoconnect.
    config_overrides:
        GlobalConfig overrides as a dict (passed directly, no CLI parsing).
    instance_name:
        Global instance name (default: "-".join(robot_types)).
    force_replace:
        If True, stop any existing instance with the same name.
    disable:
        Module names to disable.

    Returns
    -------
    LaunchResult with instance_name and run_dir.
    """
    from dimos.core.global_config import global_config
    from dimos.core.instance_registry import (
        get,
        is_pid_alive,
        make_run_dir,
        stop as registry_stop,
    )

    config_overrides = config_overrides or {}
    disable = disable or []

    # Compute instance name
    blueprint_name = "-".join(robot_types)
    if not instance_name:
        instance_name = blueprint_name

    # Handle existing instance
    existing = get(instance_name)
    if existing is not None:
        if force_replace:
            msg, _ok = registry_stop(instance_name)
            for _ in range(20):
                if not is_pid_alive(existing.pid):
                    break
                time.sleep(0.1)
        else:
            raise RuntimeError(
                f"Instance '{instance_name}' already running (PID {existing.pid}). "
                f"Use force_replace=True to auto-stop."
            )

    # Create run directory
    run_dir = make_run_dir(instance_name)

    # Dump config snapshot without mutating the host's global_config.
    # The daemon subprocess applies overrides itself from launch_params.json.
    snapshot = global_config.model_dump(mode="json")
    snapshot.update(config_overrides)
    (run_dir / "config.json").write_text(json.dumps(snapshot, indent=2))

    # Write launch parameters for the subprocess to read
    launch_params = {
        "robot_types": robot_types,
        "config_overrides": config_overrides,
        "instance_name": instance_name,
        "blueprint_name": blueprint_name,
        "disable": disable,
    }
    (run_dir / "launch_params.json").write_text(json.dumps(launch_params))

    # Spawn a completely fresh Python process.
    # start_new_session=True detaches it from the caller's terminal/process group.
    # All stdio goes to DEVNULL — the daemon writes its own stdout.log.
    logger.info(
        "launch_blueprint: spawning daemon",
        instance_name=instance_name,
        run_dir=str(run_dir),
    )
    subprocess.Popen(
        [sys.executable, "-m", "dimos.core.daemon", str(run_dir)],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )

    return LaunchResult(instance_name=instance_name, run_dir=run_dir)


def _daemon_main(run_dir: Path) -> None:
    """Read launch_params.json, double-fork, build blueprint, loop forever."""
    params = json.loads((run_dir / "launch_params.json").read_text())
    robot_types: list[str] = params["robot_types"]
    config_overrides: dict[str, object] = params["config_overrides"]
    instance_name: str = params["instance_name"]
    blueprint_name: str = params["blueprint_name"]
    disable: list[str] = params["disable"]

    # Double-fork to become a proper daemon
    pid = os.fork()
    if pid > 0:
        os._exit(0)

    os.setsid()

    pid = os.fork()
    if pid > 0:
        os._exit(0)

    # Daemon grandchild
    # Redirect stdin to /dev/null
    sys.stdout.flush()
    sys.stderr.flush()
    devnull_fd = os.open(os.devnull, os.O_RDWR)
    os.dup2(devnull_fd, 0)  # stdin

    # Redirect stdout/stderr to stdout.log immediately (crash safety).
    # OutputTee will take over these fds later with its pipe.
    run_dir.mkdir(parents=True, exist_ok=True)
    log_fd = os.open(str(run_dir / "stdout.log"), os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o644)
    os.dup2(log_fd, 1)  # stdout
    os.dup2(log_fd, 2)  # stderr
    os.close(devnull_fd)
    os.close(log_fd)

    # Rebuild Python's sys.stdout/stderr on the new fds
    sys.stdout = os.fdopen(1, "w", buffering=1)
    sys.stderr = os.fdopen(2, "w", buffering=1)

    try:
        from dimos.core.global_config import global_config

        global_config.update(**config_overrides)

        from dimos.utils.logging_config import set_run_log_dir, setup_exception_handler

        setup_exception_handler()
        set_run_log_dir(run_dir)

        # Start OutputTee — takes over fd 1+2 with its pipe, reader thread
        # fans out to stdout.log (reopened) + stdout.plain.log
        from dimos.core.output_tee import OutputTee

        tee = OutputTee(run_dir)
        tee.start()

        # Import and build blueprint (heavy imports happen here)
        from dimos.core.blueprints import autoconnect
        from dimos.robot.get_all_blueprints import get_by_name, get_module_by_name

        blueprint = autoconnect(*map(get_by_name, robot_types))

        if disable:
            disabled_classes = tuple(get_module_by_name(d).blueprints[0].module for d in disable)
            blueprint = blueprint.disabled_modules(*disabled_classes)

        coordinator = blueprint.build(cli_config_overrides=config_overrides)

        # Health check
        if not coordinator.health_check():
            sys.stderr.write("Error: health check failed — a worker process died.\n")
            coordinator.stop()
            tee.close()
            os._exit(1)

        n_workers = coordinator.n_workers
        n_modules = coordinator.n_modules
        print(f"All modules started ({n_modules} modules, {n_workers} workers)")
        print("Health check passed")
        print("DimOS running in background\n")
        print(f"  Instance: {instance_name}")
        print(f"  Run dir:  {run_dir}")
        print("  Stop:     dimos stop")
        print("  Status:   dimos status")

        coordinator.suppress_console()

        # Register with current.json (signals build completion)
        from dimos.core.instance_registry import InstanceInfo, register

        info = InstanceInfo(
            name=instance_name,
            pid=os.getpid(),
            blueprint=blueprint_name,
            started_at=datetime.now(timezone.utc).isoformat(),
            run_dir=str(run_dir),
            grpc_port=global_config.grpc_port if hasattr(global_config, "grpc_port") else 9877,
            original_argv=sys.argv,
            config_overrides=config_overrides,
        )
        register(info)
        install_signal_handlers(info, coordinator, tee)

        # Block forever
        coordinator.loop()

    except Exception:
        import traceback

        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(1)


def health_check(coordinator: ModuleCoordinator) -> bool:
    """Verify all coordinator workers are alive after build."""
    return coordinator.health_check()


def attached_tail(stdout_log: Path, instance_name: str) -> int:
    """Tail stdout.log until the daemon dies. Forward SIGINT as SIGTERM.

    Returns exit code: 0 on success, 1 on error.
    """
    from dimos.core.instance_registry import get, is_pid_alive

    got_signal = False

    # Wait for log file to appear
    for _ in range(150):  # ~30s
        if stdout_log.exists():
            break
        time.sleep(0.2)
    else:
        sys.stderr.write(f"Error: {stdout_log} never appeared\n")
        return 1

    daemon_pid: int | None = None

    def _check_pid() -> int | None:
        info = get(instance_name)
        if info is not None:
            return info.pid
        return None

    def _forward_sigint(signum: int, frame: object) -> None:
        nonlocal got_signal
        got_signal = True
        pid = daemon_pid
        if pid is not None:
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

    signal.signal(signal.SIGINT, _forward_sigint)

    try:
        with open(stdout_log) as f:
            while True:
                line = f.readline()
                if line:
                    sys.stdout.write(line)
                    sys.stdout.flush()
                else:
                    if daemon_pid is None:
                        daemon_pid = _check_pid()

                    if daemon_pid is not None and not is_pid_alive(daemon_pid):
                        for remaining in f:
                            sys.stdout.write(remaining)
                        sys.stdout.flush()
                        break

                    if got_signal:
                        for _ in range(50):
                            if daemon_pid is not None and not is_pid_alive(daemon_pid):
                                break
                            time.sleep(0.1)
                        for remaining in f:
                            sys.stdout.write(remaining)
                        sys.stdout.flush()
                        break

                    try:
                        select.select([], [], [], 0.1)
                    except (OSError, ValueError):
                        time.sleep(0.1)
    except KeyboardInterrupt:
        if daemon_pid is not None:
            try:
                os.kill(daemon_pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

    return 0


def install_signal_handlers(
    info: InstanceInfo,
    coordinator: ModuleCoordinator,
    tee: OutputTee | None = None,
    *,
    sigint: bool = True,
) -> None:
    """Install SIGTERM/SIGINT handlers that stop the coordinator and clean the registry.

    When ``sigint`` is False, only SIGTERM is wired. This is useful for foreground
    mode where the default SIGINT behavior (``KeyboardInterrupt``) should be
    preserved so Ctrl+C produces a visible traceback.
    """
    from dimos.core import instance_registry

    def _shutdown(signum: int, frame: object) -> None:
        logger.info("Received signal, shutting down", signal=signum)
        try:
            coordinator.stop()
        except Exception:
            logger.error("Error during coordinator stop", exc_info=True)
        sys.stdout.flush()
        sys.stderr.flush()
        instance_registry.unregister(info.name)
        if tee is not None:
            tee.close()
        sys.exit(0)

    signal.signal(signal.SIGTERM, _shutdown)
    if sigint:
        signal.signal(signal.SIGINT, _shutdown)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.stderr.write("Usage: python -m dimos.core.daemon <run_dir>\n")
        sys.exit(1)
    _daemon_main(Path(sys.argv[1]))
