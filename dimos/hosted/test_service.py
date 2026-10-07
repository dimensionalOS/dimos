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

from pathlib import Path
import subprocess
from types import SimpleNamespace
from typing import Any

import pytest

from dimos.hosted import service


@pytest.fixture(autouse=True)
def sandbox(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    monkeypatch.setenv("XDG_RUNTIME_DIR", str(tmp_path / "run"))
    monkeypatch.setattr(service, "unit_path", lambda: tmp_path / "unit" / service.UNIT_NAME)
    return tmp_path


def _ok(args: list[str]) -> subprocess.CompletedProcess[str]:
    return subprocess.CompletedProcess(args, 0, "active\n", "")


def test_unit_runs_host_run_from_the_checkout() -> None:
    unit = service.render_unit("/venv/bin/dimos", Path("/repo"))
    assert "ExecStart=/venv/bin/dimos host start --foreground" in unit
    assert "WorkingDirectory=/repo" in unit
    assert "Environment=PATH=/venv/bin:" in unit
    assert "KillSignal=SIGINT" in unit and "Restart=on-failure" in unit


def test_install_writes_reloads_and_enables() -> None:
    calls: list[list[str]] = []
    path = service.install(run=lambda a: (calls.append(a), _ok(a))[1])
    assert path.exists() and service.installed()
    assert calls == [
        ["daemon-reload"],
        ["enable", service.UNIT_NAME],
        ["is-active", service.UNIT_NAME],
        ["restart", service.UNIT_NAME],
    ]
    assert service.unit_current()
    service.unit_path().write_text("[Service]\nExecStart=/old/dimos host run\n")
    assert not service.unit_current()


def test_installed_unit_drives_systemctl() -> None:
    service.unit_path().parent.mkdir(parents=True)
    service.unit_path().write_text("x")
    calls: list[list[str]] = []

    def run(args: list[str]) -> subprocess.CompletedProcess[str]:
        calls.append(args)
        return _ok(args)

    def spawn(*_: Any, **__: Any) -> None:
        raise AssertionError("must not spawn with a unit installed")

    service.start(run=run, spawn=spawn)
    service.stop(run=run)
    assert service.status(run=run).startswith("active")
    assert [c[0] for c in calls] == ["start", "stop", "is-active"]


def test_without_unit_start_spawns_once_and_stop_signals(monkeypatch: pytest.MonkeyPatch) -> None:
    spawned: list[list[str]] = []
    alive = {12345}

    def spawn(cmd: list[str], **_: Any) -> SimpleNamespace:
        spawned.append(cmd)
        return SimpleNamespace(pid=12345)

    def kill(pid: int, sig: int) -> None:
        if pid not in alive:
            raise ProcessLookupError
        if sig:
            alive.discard(pid)

    monkeypatch.setattr(service.os, "kill", kill)
    assert service.start(spawn=spawn).startswith("started (pid 12345)")
    assert service.start(spawn=spawn) == "already running (pid 12345)"
    assert spawned == [[service.dimos_executable(), "host", "start", "--foreground"]]
    assert service.status() == "running (pid 12345)"
    assert service.stop() == "stopped (pid 12345)"
    assert service.status() == "not running"


def test_unit_start_refuses_while_a_detached_host_runs(monkeypatch: pytest.MonkeyPatch) -> None:
    service.unit_path().parent.mkdir(parents=True)
    service.unit_path().write_text("x")
    service.pid_file().write_text("4242")
    monkeypatch.setattr(service, "_alive", lambda pid: pid == 4242)

    def run(args: list[str]) -> subprocess.CompletedProcess[str]:
        raise AssertionError("must not start the unit")

    assert "dimos host stop" in service.start(run=run)
