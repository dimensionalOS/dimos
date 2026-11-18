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

"""The walk-through's run: fake launcher scripts and a temp run registry, never `dimos run`."""

from __future__ import annotations

from collections.abc import Iterator
import json
import os
from pathlib import Path
import subprocess
import sys
import textwrap
import time

import psutil
import pytest

from dimos.cli.commands import arm_run
from dimos.cli.commands.arm_run import (
    LAUNCH_TOKEN_ENV,
    CalibrateLockedError,
    LaunchError,
    LaunchInterruptedError,
    OurRun,
    calibrate_lock,
    launch_run,
    process_is_ours,
    stop_run,
)
from dimos.core import run_registry

RUN_ID = "20261118-101010-abcd-dual-openyam-calibrate"

# A stand-in for `dimos run --daemon`: forks a "daemon" that holds the inherited
# stdout open for 30 s (as the forkserver and resource tracker do), puts it in
# the registry, then says (or not) which run it started and exits.
FAKE_LAUNCHER = textwrap.dedent(
    """
    import json, os, sys, time
    from pathlib import Path

    registry, run_id, say, mode = sys.argv[1:5]
    if mode == "hang":
        time.sleep(30)
    child = os.fork()
    if child == 0:
        time.sleep(30)
        os._exit(0)
    entry = {"run_id": run_id, "pid": child, "blueprint": "dual-openyam-calibrate",
             "started_at": "2026-11-18T10:10:10+00:00", "log_dir": "/nonexistent"}
    Path(registry).mkdir(parents=True, exist_ok=True)
    Path(registry, run_id + ".json").write_text(json.dumps(entry))
    if say == "yes":
        print("  Run ID:    " + run_id, flush=True)
    else:
        print("Error: something went wrong", flush=True)
    sys.exit(0)
    """
)


@pytest.fixture
def registry(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    path = tmp_path / "runs"
    monkeypatch.setattr(run_registry, "REGISTRY_DIR", path)
    return path


@pytest.fixture
def launcher(tmp_path: Path) -> Path:
    script = tmp_path / "fake_launcher.py"
    script.write_text(FAKE_LAUNCHER)
    return script


@pytest.fixture
def spawned() -> Iterator[list[subprocess.Popen[bytes]]]:
    """Processes a test starts; killed at the end whatever happened."""
    processes: list[subprocess.Popen[bytes]] = []
    yield processes
    for process in processes:
        if process.poll() is None:
            process.kill()
            process.wait(5)


def _token_processes(token: str) -> list[psutil.Process]:
    return arm_run.our_processes(token, 0.0)


def _foreign_sleeper(spawned: list[subprocess.Popen[bytes]]) -> subprocess.Popen[bytes]:
    """A live process that is not ours: it carries no launch token."""
    env = {k: v for k, v in os.environ.items() if k != LAUNCH_TOKEN_ENV}
    process = subprocess.Popen(["sleep", "30"], env=env)
    spawned.append(process)
    return process


def _register(registry: Path, run_id: str, pid: int) -> None:
    registry.mkdir(parents=True, exist_ok=True)
    entry = {
        "run_id": run_id,
        "pid": pid,
        "blueprint": "xarm",
        "started_at": "2026-11-18T09:00:00+00:00",
        "log_dir": "/nonexistent",
    }
    (registry / f"{run_id}.json").write_text(json.dumps(entry))


def test_launch_returns_at_once_though_the_daemon_holds_the_launchers_stdout(
    registry: Path, launcher: Path
) -> None:
    token = "dual-openyam-calibrate:test-fast"
    started = time.monotonic()

    run = launch_run(
        [sys.executable, str(launcher), str(registry), RUN_ID, "yes", "fork"], token=token
    )

    elapsed = time.monotonic() - started
    try:
        assert elapsed < 3.0, f"launch_run took {elapsed:.1f} s"
        assert run.run_id == RUN_ID and run.token == token
        assert process_is_ours(run.pid, token, run.launched_at)
        assert "Run ID:" in run.output.read_text()
    finally:
        said = stop_run(run)
    assert said.startswith(f"stopped {RUN_ID}")
    assert not psutil.pid_exists(run.pid) or psutil.Process(run.pid).status() == "zombie"


def test_a_launcher_that_never_says_its_run_id_is_never_adopted(
    registry: Path, launcher: Path, spawned: list[subprocess.Popen[bytes]]
) -> None:
    token = "dual-openyam-calibrate:test-silent"
    foreign = _foreign_sleeper(spawned)
    _register(registry, "20261118-090000-ffff-xarm", foreign.pid)

    with pytest.raises(LaunchError, match="without saying which run it started") as raised:
        launch_run(
            [sys.executable, str(launcher), str(registry), RUN_ID, "no", "fork"], token=token
        )

    assert "something went wrong" in str(raised.value)
    # Its own leftover (carrying the token) was stopped; the foreign run was not touched.
    assert _token_processes(token) == []
    assert foreign.poll() is None
    assert (registry / "20261118-090000-ffff-xarm.json").exists()


def test_a_launch_that_hangs_times_out_and_leaves_nothing_behind(
    registry: Path, launcher: Path
) -> None:
    token = "dual-openyam-calibrate:test-hang"
    started = time.monotonic()

    with pytest.raises(LaunchError, match="did not start within 0.5 s"):
        launch_run(
            [sys.executable, str(launcher), str(registry), RUN_ID, "yes", "hang"],
            token=token,
            timeout_s=0.5,
        )

    assert time.monotonic() - started < 10.0
    assert _token_processes(token) == []


def test_an_interrupted_launch_stops_what_it_started(
    registry: Path, launcher: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    token = "dual-openyam-calibrate:test-interrupt"
    reads = {"n": 0}
    real_read = arm_run._read

    def interrupted(path: Path) -> str:
        reads["n"] += 1
        if reads["n"] == 3:
            raise KeyboardInterrupt
        return real_read(path)

    monkeypatch.setattr(arm_run, "_read", interrupted)

    with pytest.raises(LaunchInterruptedError, match="interrupted while the run was starting"):
        launch_run(
            [sys.executable, str(launcher), str(registry), RUN_ID, "yes", "hang"], token=token
        )

    assert _token_processes(token) == []


def test_stop_run_signals_only_a_pid_that_is_still_our_daemon(
    registry: Path, spawned: list[subprocess.Popen[bytes]], tmp_path: Path
) -> None:
    foreign = _foreign_sleeper(spawned)
    _register(registry, RUN_ID, foreign.pid)
    log = tmp_path / "launch.log"

    # The entry's pid carries no token of ours: reported, never signalled.
    said = stop_run(OurRun(RUN_ID, foreign.pid, "dual-openyam-calibrate:x", 0.0, log))
    assert "not signalled" in said and foreign.poll() is None

    # A pid that started before our launch is stale, whatever its environment.
    later = time.time() + 3600.0
    said = stop_run(OurRun(RUN_ID, foreign.pid, "dual-openyam-calibrate:x", later, log))
    assert "not signalled" in said and foreign.poll() is None

    # An entry whose process is gone is only cleaned up.
    foreign.kill()
    foreign.wait(5)
    said = stop_run(OurRun(RUN_ID, foreign.pid, "dual-openyam-calibrate:x", 0.0, log))
    assert said == f"{RUN_ID} is not running any more"
    assert not (registry / f"{RUN_ID}.json").exists()


def test_a_process_is_ours_only_with_our_token_and_after_our_launch(
    spawned: list[subprocess.Popen[bytes]],
) -> None:
    before = time.time()
    ours = subprocess.Popen(["sleep", "30"], env={**os.environ, LAUNCH_TOKEN_ENV: "bp:ours"})
    spawned.append(ours)

    assert process_is_ours(ours.pid, "bp:ours", before)
    assert not process_is_ours(ours.pid, "bp:theirs", before)
    assert not process_is_ours(ours.pid, "bp:ours", before + 3600.0)
    assert not process_is_ours(2**22 + 12345, "bp:ours", before)


def test_the_calibrate_lock_admits_one_holder_at_a_time(tmp_path: Path) -> None:
    path = tmp_path / "arms" / "dual-openyam.calibrate.lock"
    with calibrate_lock(path):
        with pytest.raises(CalibrateLockedError, match=f"pid {os.getpid()}"):
            with calibrate_lock(path):
                pass
    with calibrate_lock(path):
        pass


def test_the_bus_probe_says_whether_a_coordinator_answers(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from dimos.core.coordination.coordinator_rpc import CoordinatorRPC

    stopped: list[bool] = []

    class Client:
        def stop(self) -> None:
            stopped.append(True)

    def answers(*, timeout: float, name: str | None = None) -> Client:
        return Client()

    def silent(*, timeout: float, name: str | None = None) -> Client:
        raise TimeoutError("Coordinator did not respond")

    monkeypatch.setattr(CoordinatorRPC, "connect", answers)
    assert arm_run.coordinator_answers() is True and stopped == [True]
    monkeypatch.setattr(CoordinatorRPC, "connect", silent)
    assert arm_run.coordinator_answers() is False
