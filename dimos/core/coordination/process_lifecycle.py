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

"""Process-lifecycle utilities: tag descendants by run id, sweep strays.

Every descendant of a `dimos run` invocation inherits `DIMOS_RUN_ID` in
its environment. kill_run_processes scans the system via psutil and
terminates any process whose environment matches a given run id.

spawn_watchdog launches a small sidecar process that watches the main
PID; when main dies for any reason (including SIGKILL) the sidecar invokes
the sweep so workers and grandchildren cannot survive as orphans.
"""

from __future__ import annotations

from collections.abc import Iterable
import os
import subprocess
import sys
import time

import psutil

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DIMOS_RUN_ID_ENV = "DIMOS_RUN_ID"


def _iter_matching_processes(run_id: str, exclude_pids: frozenset[int]) -> list[psutil.Process]:
    """Return live processes whose environment contains DIMOS_RUN_ID=run_id."""
    matches: list[psutil.Process] = []
    for proc in psutil.process_iter(attrs=["pid"]):
        pid = proc.info["pid"]
        if pid in exclude_pids:
            continue
        try:
            env = proc.environ()
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            continue
        except OSError:
            # /proc entry may transiently fail on slow systems.
            continue
        if env.get(DIMOS_RUN_ID_ENV) == run_id:
            matches.append(proc)
    return matches


def kill_run_processes(
    run_id: str,
    *,
    exclude_pids: Iterable[int] = (),
    term_timeout: float = 2.0,
    kill_timeout: float = 1.0,
) -> int:
    """Terminate every process tagged with `DIMOS_RUN_ID == run_id`."""
    excluded = frozenset({os.getpid(), *exclude_pids})
    targets = _iter_matching_processes(run_id, excluded)
    if not targets:
        return 0

    for proc in targets:
        try:
            proc.terminate()
        except psutil.NoSuchProcess:
            pass

    _, alive = psutil.wait_procs(targets, timeout=term_timeout)
    if alive:
        for proc in alive:
            try:
                proc.kill()
            except psutil.NoSuchProcess:
                pass
        psutil.wait_procs(alive, timeout=kill_timeout)

    return len(targets)


def spawn_watchdog(
    run_id: str, log_dir: str | os.PathLike[str] | None = None
) -> subprocess.Popen[bytes]:
    """Launch a sidecar that kills run processes if main dies abruptly."""
    env = {k: v for k, v in os.environ.items() if k != DIMOS_RUN_ID_ENV}
    if log_dir is not None:
        env["DIMOS_RUN_LOG_DIR"] = str(log_dir)

    main_pid = os.getpid()
    cmd = [
        sys.executable,
        "-m",
        "dimos.core.coordination.watchdog_main",
        str(main_pid),
        run_id,
    ]
    proc = subprocess.Popen(
        cmd,
        env=env,
        start_new_session=True,
        close_fds=True,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return proc


def wait_for_pid_exit(pid: int, poll_interval: float = 0.5) -> None:
    while True:
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return
        except PermissionError:
            # Process exists under a different UID — still alive from our POV.
            pass
        time.sleep(poll_interval)
