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

"""The run a ``dimos arm calibrate`` starts: launching it, telling it from others, stopping it.

The walk-through starts ``dimos run <robot>-calibrate --daemon`` with a token
of its own in the launcher's environment (``LAUNCH_TOKEN_ENV``). On Linux the
daemon is forked from that launcher without an exec, so its
``/proc/<pid>/environ``, and every worker's, carries the token. ``DIMOS_RUN_ID``
does not show there for the daemon itself: ``dimos run`` sets it in-process
after the exec. A process belongs to the walk-through's run only when it
carries the token and started after the launch; nothing else is ever
signalled.
"""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass
import fcntl
import os
from pathlib import Path
import re
import secrets
import signal
import subprocess
import tempfile
import time

import psutil

LAUNCH_TOKEN_ENV = "DIMOS_ARM_CALIBRATE_LAUNCH"
LAUNCH_TIMEOUT_S = 120.0
LAUNCH_POLL_S = 0.1
# How long the launcher gets to exit, after it says its run id or is interrupted.
LAUNCHER_EXIT_S = 5.0
# How long leftover processes get between SIGTERM and SIGKILL.
SWEEP_TERM_S = 5.0
# create_time() and time.time() come from different clocks; this much apart is "the same".
CLOCK_SLACK_S = 1.0
RUN_ID_PATTERN = re.compile(r"Run ID:\s+(\S+)")
OUTPUT_TAIL_LINES = 20


@dataclass(frozen=True)
class OurRun:
    """The run this walk-through started.

    Attributes:
        run_id: What ``dimos run`` said ("Run ID: ...").
        pid: The daemon's pid, from that run's registry entry.
        token: The ``LAUNCH_TOKEN_ENV`` value its processes carry.
        launched_at: When the launcher started, Unix seconds.
        output: The launcher's stdout and stderr.
    """

    run_id: str
    pid: int
    token: str
    launched_at: float
    output: Path


class LaunchError(RuntimeError):
    """``dimos run`` did not start our run; what was left of it has been stopped."""


class LaunchInterruptedError(LaunchError):
    """The launch was interrupted (Ctrl-C, SIGHUP, SIGTERM); what was left of it has been stopped."""


class CalibrateLockedError(RuntimeError):
    """Another walk-through of the same robot holds its lock."""


def new_token(blueprint: str) -> str:
    return f"{blueprint}:{secrets.token_hex(8)}"


def process_is_ours(pid: int, token: str, launched_at: float) -> bool:
    """Whether ``pid`` carries ``token`` and started after ``launched_at``; False when unreadable."""
    try:
        process = psutil.Process(pid)
        if process.create_time() < launched_at - CLOCK_SLACK_S:
            return False
        return bool(process.environ().get(LAUNCH_TOKEN_ENV) == token)
    except (psutil.Error, OSError):
        return False


def run_alive(run: OurRun) -> bool:
    """Whether our run's daemon still runs (and its pid was not reused)."""
    return process_is_ours(run.pid, run.token, run.launched_at)


def our_processes(token: str, launched_at: float) -> list[psutil.Process]:
    """Every live process of this machine that carries ``token`` and started after the launch."""
    found: list[psutil.Process] = []
    for process in psutil.process_iter(["pid"]):
        if process.pid == os.getpid():
            continue
        if process_is_ours(process.pid, token, launched_at):
            found.append(process)
    return found


def stop_leftovers(token: str, launched_at: float) -> str:
    """Stop what a failed launch left running, and only that; what happened, in words.

    A run that reached its registry is stopped through it (SIGTERM, so its
    modules stop and the arm parks); whatever else carries the token gets
    SIGTERM, then SIGKILL after ``SWEEP_TERM_S``.
    """
    from dimos.core.run_registry import list_runs, stop_entry

    leftovers = our_processes(token, launched_at)
    if not leftovers:
        return "nothing of it is left running"
    pids = {process.pid for process in leftovers}
    said: list[str] = []
    for entry in list_runs(alive_only=True):
        if entry.pid in pids:
            message, _ok = stop_entry(entry)
            said.append(f"stopped {entry.run_id} ({message})")
    rest = our_processes(token, launched_at)
    for process in rest:
        try:
            process.terminate()
        except psutil.Error:
            pass
    _gone, alive = psutil.wait_procs(rest, timeout=SWEEP_TERM_S)
    for process in alive:
        try:
            process.kill()
        except psutil.Error:
            pass
    if rest:
        said.append(f"stopped {len(rest)} process(es) it had started")
    return "; ".join(said)


def _read(path: Path) -> str:
    try:
        return path.read_text(errors="replace")
    except OSError:
        return ""


def _tail(text: str) -> str:
    return "\n".join(text.strip().splitlines()[-OUTPUT_TAIL_LINES:])


def _end_launcher(launcher: subprocess.Popen[bytes]) -> None:
    """Interrupt the launcher (it then stops its own daemon) and reap it."""
    if launcher.poll() is None:
        try:
            launcher.send_signal(signal.SIGINT)
            launcher.wait(LAUNCHER_EXIT_S)
        except subprocess.TimeoutExpired:
            launcher.kill()
            launcher.wait(LAUNCHER_EXIT_S)
        except ProcessLookupError:
            pass


def _abandon(launcher: subprocess.Popen[bytes], token: str, launched_at: float, why: str) -> str:
    """Interrupt the launch and stop what it left; ``why`` and what happened, in words."""
    try:
        _end_launcher(launcher)
        cleaned = stop_leftovers(token, launched_at)
    except KeyboardInterrupt:
        cleaned = "interrupted again while stopping it: check `dimos status`"
    return f"{why}; {cleaned}"


def launch_run(
    argv: list[str],
    *,
    token: str,
    timeout_s: float = LAUNCH_TIMEOUT_S,
    output_dir: Path | None = None,
) -> OurRun:
    """Start ``argv`` (a ``dimos run ... --daemon``) and return the run it reports.

    The launcher writes to a file, not a pipe: the daemon's forkserver and
    resource tracker inherit its stdout, and a pipe would stay open until the
    run ends. The file is read while the launcher lives, for its "Run ID:"
    line. On a timeout, an interrupt or a launcher that ends without that line,
    whatever the launch left running is stopped (only processes carrying
    ``token``) and ``LaunchError`` says so; a run is never adopted otherwise.
    The file goes to ``output_dir``, the system's temp directory by default.
    """
    from dimos.core.run_registry import list_runs

    launched_at = time.time()
    descriptor, name = tempfile.mkstemp(
        prefix="dimos-arm-calibrate-", suffix=".log", dir=output_dir
    )
    output = Path(name)
    with os.fdopen(descriptor, "wb") as sink:
        launcher = subprocess.Popen(
            argv,
            stdin=subprocess.DEVNULL,
            stdout=sink,
            stderr=subprocess.STDOUT,
            env={**os.environ, LAUNCH_TOKEN_ENV: token},
            start_new_session=True,
        )
    deadline = time.monotonic() + timeout_s
    try:
        while True:
            exited = launcher.poll() is not None
            found = RUN_ID_PATTERN.search(_read(output))
            if found:
                run_id = found.group(1)
                try:
                    launcher.wait(LAUNCHER_EXIT_S)
                except subprocess.TimeoutExpired:
                    pass
                break
            if exited:
                cleaned = stop_leftovers(token, launched_at)
                raise LaunchError(
                    f"dimos run exited with {launcher.returncode} without saying which run it "
                    f"started; {cleaned}. Its output ({output}):\n{_tail(_read(output))}"
                )
            if time.monotonic() >= deadline:
                said = _abandon(
                    launcher, token, launched_at, f"dimos run did not start within {timeout_s:g} s"
                )
                raise LaunchError(f"{said}. Its output: {output}")
            time.sleep(LAUNCH_POLL_S)
    except KeyboardInterrupt:
        said = _abandon(launcher, token, launched_at, "interrupted while the run was starting")
        raise LaunchInterruptedError(f"{said}. Its output: {output}") from None
    entry = next((e for e in list_runs(alive_only=True) if e.run_id == run_id), None)
    if entry is None or not process_is_ours(entry.pid, token, launched_at):
        cleaned = stop_leftovers(token, launched_at)
        whose = "no live run has that id" if entry is None else f"pid {entry.pid} is not ours"
        raise LaunchError(
            f"dimos run said it started {run_id}, but {whose}; not adopting it; {cleaned}. "
            f"Its output: {output}"
        )
    return OurRun(run_id, entry.pid, token, launched_at, output)


def stop_run(run: OurRun) -> str:
    """Stop our run and nothing else; what happened, in words.

    The registry entry's pid is signalled only while it is still our daemon
    (it carries our token and started after the launch). A stale entry, or a
    pid reused by another process, is only reported.
    """
    from dimos.core.run_registry import list_runs, stop_entry

    for entry in list_runs(alive_only=True):
        if entry.run_id != run.run_id:
            continue
        if not process_is_ours(entry.pid, run.token, run.launched_at):
            return (
                f"{run.run_id} is not running any more: its registry entry names pid "
                f"{entry.pid}, which is not the process this walk-through started; not signalled"
            )
        message, _ok = stop_entry(entry)
        return f"stopped {run.run_id} ({message})"
    return f"{run.run_id} is not running any more"


def coordinator_answers(timeout_s: float = 1.0) -> bool:
    """Whether any dimos coordinator answers on the configured bus (any user's, even one starting)."""
    from dimos.core.coordination.coordinator_rpc import CoordinatorRPC

    try:
        client = CoordinatorRPC.connect(timeout=timeout_s)
    except TimeoutError:
        return False
    client.stop()
    return True


@contextmanager
def calibrate_lock(path: Path) -> Iterator[None]:
    """Hold ``path`` (an flock) for the block; ``CalibrateLockedError`` when someone else holds it."""
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor = os.open(path, os.O_RDWR | os.O_CREAT, 0o644)
    try:
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            holder = os.read(descriptor, 64).decode(errors="replace").strip() or "?"
            raise CalibrateLockedError(
                f"another `dimos arm calibrate` of this robot is running (pid {holder}; "
                f"lock {path})"
            ) from None
        os.ftruncate(descriptor, 0)
        os.write(descriptor, f"{os.getpid()}\n".encode())
        yield
    finally:
        os.close(descriptor)
