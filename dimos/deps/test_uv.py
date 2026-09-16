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

from packaging.version import Version
import pytest

from dimos.deps import uv as uv_module
from dimos.deps.uv import UvNotFoundError, UvRunner, find_uv, require_uv, uv_environment, uv_version


def test_find_uv_prefers_path(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(uv_module.shutil, "which", lambda name: "/usr/bin/uv")
    assert find_uv() == ["/usr/bin/uv"]


def test_find_uv_falls_back_to_the_module(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(uv_module.shutil, "which", lambda name: None)
    monkeypatch.setattr(uv_module.importlib.util, "find_spec", lambda name: object())
    assert find_uv() == [sys.executable, "-m", "uv"]
    monkeypatch.setattr(uv_module.importlib.util, "find_spec", lambda name: None)
    with pytest.raises(UvNotFoundError, match="astral.sh/uv/install.sh"):
        find_uv()


def test_uv_version_and_minimum(monkeypatch: pytest.MonkeyPatch) -> None:
    def fake_run(args: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        return subprocess.CompletedProcess(args, 0, stdout="uv 0.11.15 (abc 2026-09-01)\n")

    monkeypatch.setattr(uv_module.subprocess, "run", fake_run)
    assert uv_version(["uv"]) == Version("0.11.15")
    monkeypatch.setattr(uv_module.shutil, "which", lambda name: "/usr/bin/uv")
    assert require_uv() == ["/usr/bin/uv"]

    def old_run(args: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        return subprocess.CompletedProcess(args, 0, stdout="uv 0.8.0\n")

    monkeypatch.setattr(uv_module.subprocess, "run", old_run)
    with pytest.raises(UvNotFoundError, match="need >= 0.9.25"):
        require_uv()


def test_uv_runner_runs_the_command(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    calls: list[tuple[list[str], dict[str, str] | None, Path | None]] = []

    def fake_run(args: list[str], **kwargs: object) -> subprocess.CompletedProcess[str]:
        env = kwargs.get("env")
        cwd = kwargs.get("cwd")
        calls.append(
            (
                args,
                dict(env) if isinstance(env, dict) else None,
                cwd if isinstance(cwd, Path) else None,
            )
        )
        return subprocess.CompletedProcess(args, 3)

    monkeypatch.setattr(uv_module.subprocess, "run", fake_run)
    runner = UvRunner(["/usr/bin/uv"])
    assert runner.run(["sync", "--frozen"], env={"A": "1"}, cwd=tmp_path) == 3
    assert calls == [(["/usr/bin/uv", "sync", "--frozen"], {"A": "1"}, tmp_path)]


def test_uv_environment(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    monkeypatch.setenv("VIRTUAL_ENV", "/somewhere")
    monkeypatch.setenv("UV_PYTHON", "3.10")
    monkeypatch.setenv("UV_NO_SYNC", "1")
    monkeypatch.setenv("HOME", "/home/x")
    env = uv_environment(tmp_path / "venv", offline=True, find_links=str(tmp_path))
    assert env["UV_PROJECT_ENVIRONMENT"] == str((tmp_path / "venv").resolve())
    assert env["UV_OFFLINE"] == "1" and env["UV_FIND_LINKS"] == str(tmp_path.resolve())
    assert env["HOME"] == "/home/x"
    assert "VIRTUAL_ENV" not in env and "UV_PYTHON" not in env and "UV_NO_SYNC" not in env
    plain = uv_environment(tmp_path / "venv")
    assert "UV_OFFLINE" not in plain and "UV_FIND_LINKS" not in plain
