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

from __future__ import annotations

import os
import subprocess
import sys
import time
import uuid

import psutil
import pytest
from pytest_mock import MockerFixture

from dimos.core.coordination.process_lifecycle import (
    DIMOS_RUN_ID_ENV,
    kill_run_processes,
    spawn_watchdog,
    wait_for_pid_exit,
)


def _wait_gone(proc: subprocess.Popen, timeout: float = 3.0) -> bool:
    try:
        proc.wait(timeout=timeout)
        return True
    except subprocess.TimeoutExpired:
        return False


@pytest.fixture
def run_id() -> str:
    return f"test-{uuid.uuid4().hex[:12]}"


@pytest.fixture
def spawn_tagged():
    """Factory that spawns subprocesses tagged with DIMOS_RUN_ID; auto-cleans."""
    procs: list[subprocess.Popen[bytes]] = []

    def _spawn(rid: str, code: str = "import time; time.sleep(60)") -> subprocess.Popen[bytes]:
        env = {**os.environ, DIMOS_RUN_ID_ENV: rid}
        proc = subprocess.Popen(
            [sys.executable, "-c", code],
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        procs.append(proc)
        return proc

    yield _spawn

    for proc in procs:
        if proc.poll() is None:
            proc.kill()
            try:
                proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                pass


def test_kill_run_processes_kills_only_matching_run_id(spawn_tagged, run_id):
    other_id = f"test-{uuid.uuid4().hex[:12]}"
    a1 = spawn_tagged(run_id)
    a2 = spawn_tagged(run_id)
    b = spawn_tagged(other_id)

    killed = kill_run_processes(run_id)
    assert killed == 2
    assert _wait_gone(a1)
    assert _wait_gone(a2)
    # The other child must still be alive.
    assert b.poll() is None


def test_kill_run_processes_no_match_returns_zero(run_id: str) -> None:
    assert kill_run_processes(run_id) == 0


def test_kill_run_processes_excludes_current_process(
    monkeypatch: pytest.MonkeyPatch, run_id: str
) -> None:
    # Tag ourselves with a run id; kill_run_processes must skip self.
    monkeypatch.setenv(DIMOS_RUN_ID_ENV, run_id)
    assert kill_run_processes(run_id) == 0


def test_kill_run_processes_respects_explicit_exclude_pids(spawn_tagged, run_id):
    keeper = spawn_tagged(run_id)
    target = spawn_tagged(run_id)

    killed = kill_run_processes(run_id, exclude_pids=[keeper.pid])
    assert killed == 1
    assert _wait_gone(target)
    assert keeper.poll() is None


def test_kill_run_processes_escalates_to_sigkill_on_sigterm_ignorer(spawn_tagged, run_id):
    # A subprocess that ignores SIGTERM. We kill it via kill_run_processes
    # with a short term_timeout so it escalates to SIGKILL.
    code = "import signal, time; signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(60)"
    proc = spawn_tagged(run_id, code=code)
    # Give the child a moment to install the SIG_IGN handler.
    time.sleep(0.3)
    killed = kill_run_processes(run_id, term_timeout=0.3, kill_timeout=2.0)
    assert killed == 1
    assert _wait_gone(proc, timeout=3.0)


def test_kill_run_processes_tolerates_psutil_errors(mocker: MockerFixture, run_id: str) -> None:
    # All psutil access errors should be swallowed, not raised.
    bad = mocker.MagicMock(spec=psutil.Process)
    bad.info = {"pid": 99999999}
    bad.environ.side_effect = psutil.AccessDenied(pid=99999999)
    mocker.patch("psutil.process_iter", return_value=[bad])

    assert kill_run_processes(run_id) == 0


def test_wait_for_pid_exit_returns_when_pid_gone():
    # Spawn a short-lived child, wait for it.
    proc = subprocess.Popen(
        [sys.executable, "-c", "pass"],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    proc.wait(timeout=2)
    # PID is now gone — wait_for_pid_exit should return immediately.
    start = time.monotonic()
    wait_for_pid_exit(proc.pid, poll_interval=0.1)
    assert time.monotonic() - start < 0.5


@pytest.mark.slow
def test_spawn_watchdog_sweeps_on_parent_death(spawn_tagged, run_id):
    # The sleeper is tagged with run_id and should be swept by the watchdog
    # once the helper (acting as "main") dies.
    sleeper = spawn_tagged(run_id)
    # The helper becomes "main": it spawns the watchdog pointing at its own
    # pid, then sleeps. Killing it should trigger the watchdog sweep.
    helper_code = (
        "import subprocess, sys, time, os; "
        f"subprocess.Popen([sys.executable, '-m', 'dimos.core.coordination.watchdog_main', str(os.getpid()), {run_id!r}], "
        "start_new_session=True, close_fds=True, "
        "stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL); "
        "time.sleep(30)"
    )
    helper = spawn_tagged(run_id, code=helper_code)

    # Wait a moment for the watchdog to start polling.
    time.sleep(1.0)
    assert sleeper.poll() is None
    helper.kill()
    helper.wait(timeout=2)

    # Watchdog should detect helper death and sweep within a few seconds.
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        if sleeper.poll() is not None:
            break
        time.sleep(0.1)
    assert sleeper.poll() is not None, "watchdog did not sweep the sleeper"


def test_spawn_watchdog_strips_run_id_env(mocker: MockerFixture, run_id: str) -> None:
    mocker.patch.dict(os.environ, {DIMOS_RUN_ID_ENV: run_id})
    popen_mock = mocker.patch("dimos.core.coordination.process_lifecycle.subprocess.Popen")
    popen_mock.return_value = mocker.MagicMock(pid=12345)
    spawn_watchdog(run_id)

    kwargs = popen_mock.call_args.kwargs
    assert DIMOS_RUN_ID_ENV not in kwargs["env"]
    assert kwargs["start_new_session"] is True
