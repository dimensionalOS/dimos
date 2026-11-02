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

import os
from pathlib import Path
import subprocess
from types import SimpleNamespace
from typing import Any

import pytest

from dimos.hosted import machine
from dimos.hosted.machine import HostRecord

_real_locked = machine._locked


@pytest.fixture(autouse=True)
def sandbox(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    monkeypatch.setattr(machine, "HOST_FILE", tmp_path / "run" / "host.json")
    monkeypatch.setattr(machine, "LOCK_FILE", tmp_path / "run" / "host.lock")
    monkeypatch.setattr(machine, "LOG_FILE", tmp_path / "run" / "host.log")
    monkeypatch.setattr(machine, "unit_path", lambda: tmp_path / "unit" / machine.UNIT_NAME)
    # A record counts while the lock is held; the daemon under test is this process.
    monkeypatch.setattr(machine, "_locked", lambda: (tmp_path / "run" / "host.json").exists())
    return tmp_path


def _ok(args: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.CompletedProcess(args, 0, "active\n", "")


def _record(pid: int) -> HostRecord:
    return HostRecord("id", "name", pid, ("tcp/0.0.0.0:7447",), "tcp/127.0.0.1:7447")


def test_host_record_counts_only_while_the_lock_is_held(monkeypatch: pytest.MonkeyPatch) -> None:
    assert machine.local_host() is None
    machine.write_host_file(_record(os.getpid()))
    assert machine.local_host() == _record(os.getpid())
    monkeypatch.setattr(machine, "_locked", lambda: False)
    assert machine.local_host() is None


def test_host_lock_is_exclusive(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(machine, "_locked", _real_locked)
    assert not machine._locked()
    with machine.host_lock():
        assert machine._locked()
        with pytest.raises(RuntimeError, match="already running"):
            with machine.host_lock():
                pass
    assert not machine._locked()


def test_unit_runs_host_run_from_the_checkout() -> None:
    unit = machine.render_unit("/venv/bin/dimos", Path("/repo"))
    assert "ExecStart=/venv/bin/dimos host start --foreground" in unit
    assert "WorkingDirectory=/repo" in unit
    assert "Environment=PATH=/venv/bin:" in unit
    assert "KillSignal=SIGINT" in unit and "Restart=on-failure" in unit


def test_install_writes_reloads_and_enables() -> None:
    calls: list[list[str]] = []
    path = machine.install(run=lambda a: (calls.append(a), _ok(a))[1])
    assert path.exists() and machine.installed()
    assert calls == [
        ["daemon-reload"],
        ["enable", machine.UNIT_NAME],
        ["is-active", machine.UNIT_NAME],
        ["restart", machine.UNIT_NAME],
    ]
    assert machine.unit_current()
    machine.unit_path().write_text("[Service]\nExecStart=/old/dimos host run\n")
    assert not machine.unit_current()


def test_installed_unit_drives_systemctl() -> None:
    machine.unit_path().parent.mkdir(parents=True)
    machine.unit_path().write_text("x")
    calls: list[list[str]] = []

    def run(args: list[str]) -> subprocess.CompletedProcess[str]:
        calls.append(args)
        return _ok(args)

    def spawn(*_: Any, **__: Any) -> None:
        raise AssertionError("must not spawn with a unit installed")

    machine.start(run=run, spawn=spawn)
    machine.stop(run=run)
    assert machine.status(run=run).startswith("active")
    assert [c[0] for c in calls] == ["start", "is-active", "stop", "is-active"]


def test_without_unit_start_spawns_once_and_stop_signals(monkeypatch: pytest.MonkeyPatch) -> None:
    spawned: list[list[str]] = []
    alive = {12345}

    def spawn(cmd: list[str], **_: Any) -> SimpleNamespace:
        spawned.append(cmd)
        machine.write_host_file(_record(12345))
        return SimpleNamespace(pid=12345)

    def kill(pid: int, sig: int) -> None:
        if pid not in alive:
            raise ProcessLookupError
        if sig:
            alive.discard(pid)

    monkeypatch.setattr(machine.os, "kill", kill)
    assert machine.start(spawn=spawn).startswith("started (pid 12345)")
    assert machine.start(spawn=spawn) == "already running (pid 12345)"
    assert spawned == [[machine.dimos_executable(), "host", "start", "--foreground"]]
    assert machine.status() == "running (pid 12345)"
    assert machine.stop() == "stopped (pid 12345)"
    assert machine.status() == "not running"


def test_unit_start_refuses_while_a_detached_host_runs(monkeypatch: pytest.MonkeyPatch) -> None:
    machine.unit_path().parent.mkdir(parents=True)
    machine.unit_path().write_text("x")
    machine.write_host_file(_record(4242))

    def run(args: list[str]) -> subprocess.CompletedProcess[str]:
        raise AssertionError("must not start the unit")

    assert machine.start(run=run) == "already running (pid 4242)"


def test_restart_uses_the_unit_when_installed() -> None:
    machine.unit_path().parent.mkdir(parents=True)
    machine.unit_path().write_text("x")
    calls: list[list[str]] = []
    machine.restart(run=lambda a: (calls.append(a), _ok(a))[1])
    assert calls == [["restart", machine.UNIT_NAME]]


def _git(cwd: Path, *args: str) -> str:
    return subprocess.run(
        ["git", "-C", str(cwd), *args], check=True, capture_output=True, text=True
    ).stdout.strip()


@pytest.fixture
def clone(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> tuple[Path, Path, list[bool]]:
    """An origin, a working clone of it standing in for this checkout, and the restarts asked for."""
    origin, work = tmp_path / "origin", tmp_path / "work"
    _git(tmp_path, "init", "-q", "-b", "main", str(origin))
    for args in (("config", "user.email", "t@t"), ("config", "user.name", "t")):
        _git(origin, *args)
    _git(origin, "commit", "-q", "--allow-empty", "-m", "one")
    _git(tmp_path, "clone", "-q", str(origin), str(work))
    restarts: list[bool] = []
    monkeypatch.setattr(machine, "DIMOS_PROJECT_ROOT", work)
    monkeypatch.setattr(machine, "_restart_detached", lambda: restarts.append(True))
    monkeypatch.setattr(machine.threading, "Timer", lambda _delay, fn: SimpleNamespace(start=fn))
    return origin, work, restarts


def test_sync_checkout_fast_forwards_to_the_commit_and_restarts(
    clone: tuple[Path, Path, list[bool]],
) -> None:
    origin, work, restarts = clone
    _git(origin, "commit", "-q", "--allow-empty", "-m", "two")
    commit = _git(origin, "rev-parse", "HEAD")

    machine.sync_checkout("main", commit)

    assert _git(work, "rev-parse", "HEAD") == commit
    assert _git(work, "rev-parse", "--abbrev-ref", "HEAD") == "main"
    assert restarts == [True]


def test_sync_checkout_refuses_what_is_not_a_fast_forward(
    clone: tuple[Path, Path, list[bool]],
) -> None:
    origin, work, restarts = clone
    for args in (("config", "user.email", "t@t"), ("config", "user.name", "t")):
        _git(work, *args)
    _git(work, "commit", "-q", "--allow-empty", "-m", "local only")
    _git(origin, "commit", "-q", "--allow-empty", "-m", "two")

    with pytest.raises(RuntimeError, match="git merge --ff-only"):
        machine.sync_checkout("main", _git(origin, "rev-parse", "HEAD"))
    assert restarts == []
