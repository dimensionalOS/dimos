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
import sys

import pytest

import dimos.core.coordination.external_python_worker as runtime_module
from dimos.core.coordination.external_python_worker import ExternalPythonRuntime


class Declaration:
    implementation = "external_package:Implementation"


def make_runtime(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> ExternalPythonRuntime:
    source = tmp_path / "declaration.py"
    source.touch()
    monkeypatch.setattr(runtime_module.inspect, "getfile", lambda _: str(source))
    project = source.parent / "python"
    project.mkdir()
    (project / "pyproject.toml").write_text("[project]\nname = 'external-package'\n")
    return ExternalPythonRuntime(Declaration, {"answer": 42})


def test_missing_runtime_project_is_reported(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = tmp_path / "declaration.py"
    source.touch()
    monkeypatch.setattr(runtime_module.inspect, "getfile", lambda _: str(source))

    with pytest.raises(FileNotFoundError, match="sibling 'python/' directory"):
        ExternalPythonRuntime(Declaration, {})


def test_missing_runtime_manifest_is_reported(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = tmp_path / "declaration.py"
    source.touch()
    (tmp_path / "python").mkdir()
    monkeypatch.setattr(runtime_module.inspect, "getfile", lambda _: str(source))

    with pytest.raises(FileNotFoundError, match="pyproject.toml"):
        ExternalPythonRuntime(Declaration, {})


def test_uv_commands_use_uv_lock_and_pixi_when_present(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    runtime = make_runtime(tmp_path, monkeypatch)
    assert runtime.prepare_command() == ["uv", "sync"]
    assert runtime.launch_command("decl:Declaration", "impl:Implementation", 17)[2:4] == [
        "python",
        "-m",
    ]
    (runtime.project / "uv.lock").touch()
    assert runtime.prepare_command() == ["uv", "sync", "--locked"]
    assert runtime.launch_command("decl:Declaration", "impl:Implementation", 17)[:5] == [
        "uv",
        "run",
        "--locked",
        "python",
        "-m",
    ]

    (runtime.project / "pixi.toml").write_text(
        """[workspace]
name = 'external-package'
channels = ['conda-forge']
platforms = ['linux-64']

[dependencies]
uv = '*'
"""
    )
    assert runtime.prepare_command() == [
        "pixi",
        "run",
        "--executable",
        "uv",
        "sync",
        "--locked",
    ]
    assert runtime.launch_command("decl:Declaration", "impl:Implementation", 17)[:5] == [
        "pixi",
        "run",
        "--executable",
        "uv",
        "run",
    ]


def test_command_failure_keeps_bounded_diagnostics(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    runtime = make_runtime(tmp_path, monkeypatch)
    monkeypatch.setattr(
        runtime_module.subprocess,
        "run",
        lambda *args, **kwargs: subprocess.CompletedProcess(
            args[0], 9, stdout="out\n" * 100_000, stderr="err\n" * 100_000
        ),
    )

    with pytest.raises(RuntimeError) as error:
        runtime._run(["uv", "sync"])
    message = str(error.value)
    assert "exit 9" in message
    assert len(message) < runtime.output_limit + 100


class FakeProcess:
    pid = 1234

    def __init__(self, running: bool = True) -> None:
        self.running = running
        self.wait_calls: list[float | None] = []

    def poll(self) -> int | None:
        return None if self.running else 0

    def wait(self, timeout: float | None = None) -> int:
        self.wait_calls.append(timeout)
        self.running = False
        return 0


def test_stop_terminates_process_group_and_is_idempotent(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    runtime = make_runtime(tmp_path, monkeypatch)
    process = FakeProcess()
    runtime._process = process  # type: ignore[assignment]
    killed: list[tuple[int, int]] = []
    monkeypatch.setattr(runtime_module.os, "killpg", lambda pid, sig: killed.append((pid, sig)))

    runtime.stop()
    runtime.stop()

    assert killed == [(1234, runtime_module.signal.SIGTERM)]
    assert process.wait_calls == [runtime.shutdown_timeout]
    assert runtime.pid is None


def test_stop_escalates_when_graceful_shutdown_times_out(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    runtime = make_runtime(tmp_path, monkeypatch)

    class HungProcess(FakeProcess):
        def wait(self, timeout: float | None = None) -> int:
            self.wait_calls.append(timeout)
            if timeout == runtime.shutdown_timeout:
                raise subprocess.TimeoutExpired("external", timeout or 0)
            self.running = False
            return 0

    process = HungProcess()
    runtime._process = process  # type: ignore[assignment]
    killed: list[tuple[int, int]] = []
    monkeypatch.setattr(runtime_module.os, "killpg", lambda pid, sig: killed.append((pid, sig)))

    runtime.stop()

    assert killed == [
        (1234, runtime_module.signal.SIGTERM),
        (1234, runtime_module.signal.SIGKILL),
    ]
    assert process.wait_calls == [runtime.shutdown_timeout, 1]
    assert runtime.pid is None


def test_stop_reaps_real_child_process(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    runtime = make_runtime(tmp_path, monkeypatch)
    process = subprocess.Popen(
        [sys.executable, "-c", "import time; time.sleep(30)"],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
    )
    runtime._process = process  # type: ignore[assignment]

    runtime.stop()

    assert process.poll() is not None
    assert runtime.pid is None
