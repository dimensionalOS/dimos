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

import pytest

from dimos.constants import CACHE_DIR
from dimos.core.core import rpc
from dimos.core.python_native_module import PythonNativeModule, PythonNativeModuleConfig


class Contract(PythonNativeModule):
    implementation = "runtime:Runtime"
    config: PythonNativeModuleConfig

    @rpc
    def value(self) -> int:
        raise NotImplementedError


def test_sibling_project_is_required(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    source = tmp_path / "contract.py"
    source.touch()
    monkeypatch.setattr("dimos.core.python_native_module.inspect.getfile", lambda _: str(source))
    module = Contract()
    try:
        with pytest.raises(FileNotFoundError, match="sibling 'python/'"):
            _ = module.runtime_project
    finally:
        module.stop()


def test_uv_lock_enables_locked_commands(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    source = tmp_path / "contract.py"
    source.touch()
    project = tmp_path / "python"
    project.mkdir()
    (project / "pyproject.toml").touch()
    (project / "uv.lock").touch()
    monkeypatch.setattr("dimos.core.python_native_module.inspect.getfile", lambda _: str(source))
    module = Contract()
    try:
        prepare = module._prepare_command()
        launch = module._launch_command(7)

        assert prepare[:2] == ["uv", "sync"]
        assert prepare[2:6] == ["--project", str(project), "--locked", "--python"]
        assert launch[:2] == ["uv", "run"]
        assert ["--project", str(project), "--locked", "--python", "3.12"] == launch[2:7]
        assert "--with-editable" in launch or "--with" in launch
    finally:
        module.stop()


def test_pixi_supplies_uv_when_manifest_exists(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = tmp_path / "contract.py"
    source.touch()
    project = tmp_path / "python"
    project.mkdir()
    (project / "pyproject.toml").touch()
    (project / "uv.lock").touch()
    (project / "pixi.toml").touch()
    monkeypatch.setattr("dimos.core.python_native_module.inspect.getfile", lambda _: str(source))
    module = Contract()
    try:
        assert module._prepare_command()[:5] == [
            "pixi",
            "run",
            "--executable",
            "uv",
            "sync",
        ]
    finally:
        module.stop()


def test_runtime_environment_uses_sibling_virtualenv(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    source = tmp_path / "contract.py"
    source.touch()
    project = tmp_path / "python"
    project.mkdir()
    (project / "pyproject.toml").touch()
    (project / "uv.lock").write_text("locked")
    monkeypatch.setattr("dimos.core.python_native_module.inspect.getfile", lambda _: str(source))
    monkeypatch.setenv("VIRTUAL_ENV", "/parent/.venv")
    module = Contract(extra_env={"EXAMPLE_SETTING": "configured"})
    try:
        env = module._runtime_env()

        assert "VIRTUAL_ENV" not in env
        assert env["EXAMPLE_SETTING"] == "configured"
        assert Path(env["UV_PROJECT_ENVIRONMENT"]).is_relative_to(CACHE_DIR / "python-native")
    finally:
        module.stop()
