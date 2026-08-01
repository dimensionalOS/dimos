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
import sys

import pytest

from dimos.core.runtime_environment import (
    CurrentProcessRuntimeEnvironment,
    MissingPreparedPythonProjectError,
    MissingPythonProjectFileError,
    NativeRuntimeEnvironment,
    NixNativeRuntimeEnvironment,
    PythonProjectRuntimeEnvironment,
    PythonVenvRuntimeEnvironment,
    RuntimeEnvironmentRegistry,
)


def test_register_and_resolve_environment() -> None:
    env = PythonVenvRuntimeEnvironment(name="tools", python_executable=Path("python"))
    registry = RuntimeEnvironmentRegistry.with_current_process().register(env)

    assert registry.resolve("tools") is env


def test_registry_resolves_python_project_runtime_environment(tmp_path: Path) -> None:
    env = PythonProjectRuntimeEnvironment(name="worker", project=tmp_path)
    registry = RuntimeEnvironmentRegistry.with_current_process().register(env)

    assert registry.resolve("worker") is env


def test_python_project_runtime_is_convention_only_and_normalizes_project_path(
    tmp_path: Path,
) -> None:
    project = tmp_path / "project"
    project.mkdir()
    env = PythonProjectRuntimeEnvironment(name="worker", project=project / ".." / "project")

    assert env.name == "worker"
    assert env.project == project.resolve()
    assert set(env.__dataclass_fields__) == {"name", "project"}


def test_python_project_missing_pyproject_error_is_actionable(tmp_path: Path) -> None:
    env = PythonProjectRuntimeEnvironment(name="worker", project=tmp_path)
    missing_file = tmp_path / "pyproject.toml"

    with pytest.raises(MissingPythonProjectFileError) as exc_info:
        env.resolve_python_project()

    message = str(exc_info.value)
    assert "worker" in message
    assert str(tmp_path) in message
    assert str(missing_file) in message
    assert "pyproject.toml" in message


def test_python_project_uv_only_launch_material(tmp_path: Path) -> None:
    (tmp_path / "pyproject.toml").write_text("[project]\nname = 'worker'\n")
    prepared_python = tmp_path / ".venv" / "bin" / "python"
    prepared_python.parent.mkdir(parents=True)
    prepared_python.touch()
    env = PythonProjectRuntimeEnvironment(name="worker", project=tmp_path)

    material = env.resolve_python_project()

    assert env.convention() == "uv"
    assert material.argv_prefix == ["uv", "run", "--no-sync", "python"]
    assert material.cwd == tmp_path.resolve()
    assert material.env == {}
    assert material.prepared_python == prepared_python


def test_python_project_pixi_backed_launch_material(tmp_path: Path) -> None:
    (tmp_path / "pyproject.toml").write_text("[project]\nname = 'worker'\n")
    (tmp_path / "pixi.toml").write_text("[workspace]\n")
    prepared_python = tmp_path / ".venv" / "bin" / "python"
    prepared_python.parent.mkdir(parents=True)
    prepared_python.touch()
    env = PythonProjectRuntimeEnvironment(name="worker", project=tmp_path)

    material = env.resolve_python_project()

    assert env.convention() == "pixi-backed-uv"
    assert material.argv_prefix == ["pixi", "run", "uv", "run", "--no-sync", "python"]
    assert material.cwd == tmp_path.resolve()
    assert material.env == {}
    assert material.prepared_python == prepared_python


def test_python_project_requires_prepared_venv_python(tmp_path: Path) -> None:
    (tmp_path / "pyproject.toml").write_text("[project]\nname = 'worker'\n")
    env = PythonProjectRuntimeEnvironment(name="worker", project=tmp_path)
    missing_executable = tmp_path / ".venv" / "bin" / "python"

    with pytest.raises(MissingPreparedPythonProjectError) as exc_info:
        env.resolve_python_project()

    message = str(exc_info.value)
    assert "worker" in message
    assert str(tmp_path) in message
    assert str(missing_executable) in message
    assert "uv" in message
    assert "dimos runtime prepare <blueprint> --runtime worker" in message


def test_current_process_python_material() -> None:
    material = CurrentProcessRuntimeEnvironment().resolve_python()

    assert material.python_executable == Path(sys.executable)
    assert material.env


def test_python_venv_material() -> None:
    env = PythonVenvRuntimeEnvironment(
        name="venv", python_executable=Path("/tmp/venv/bin/python"), env={"A": "B"}
    )

    material = env.resolve_python()

    assert material.python_executable == Path("/tmp/venv/bin/python")
    assert material.env == {"A": "B"}


def test_nix_native_material() -> None:
    env = NixNativeRuntimeEnvironment(
        name="native",
        executable="/nix/store/bin/tool",
        build_command="nix build",
        cwd=Path("/tmp"),
        env={"X": "Y"},
    )

    material = env.resolve_native()

    assert material.executable == "/nix/store/bin/tool"
    assert material.build_command == "nix build"
    assert material.cwd == Path("/tmp")
    assert material.env == {"X": "Y"}


def test_missing_name_error_lists_known_names() -> None:
    registry = RuntimeEnvironmentRegistry.with_current_process()

    with pytest.raises(KeyError, match="Unknown runtime environment 'missing'.*current"):
        registry.resolve("missing")


def test_unsupported_python_capability_error() -> None:
    env = NativeRuntimeEnvironment(name="native", executable="tool")

    with pytest.raises(RuntimeError, match="does not provide Python launch material"):
        env.resolve_python()


def test_unsupported_native_capability_error() -> None:
    env = PythonVenvRuntimeEnvironment(name="venv", python_executable=Path("python"))

    with pytest.raises(RuntimeError, match="does not provide native launch material"):
        env.resolve_native()
