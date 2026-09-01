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
from pytest_mock import MockerFixture

from dimos.core import python_native_environment
from dimos.core.python_native_environment import PythonNativeProject


def _project(tmp_path: Path, *, lock: str = "locked") -> PythonNativeProject:
    project = tmp_path / "python"
    project.mkdir(parents=True)
    (project / "pyproject.toml").touch()
    (project / "uv.lock").write_text(lock)
    return PythonNativeProject(project)


def test_source_checkout_is_overlaid_editably(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    package = tmp_path / "dimos"
    package.mkdir()
    (package / "__init__.py").touch()
    (tmp_path / "pyproject.toml").touch()
    (tmp_path / ".git").touch()
    monkeypatch.setattr(python_native_environment.dimos, "__file__", str(package / "__init__.py"))

    project = _project(tmp_path / "runtime")

    assert project.run_command("python")[-3:] == ["--with-editable", str(tmp_path), "python"]


def test_installed_release_is_overlaid_at_exact_version(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    package = tmp_path / "site-packages" / "dimos"
    package.mkdir(parents=True)
    (package / "__init__.py").touch()
    monkeypatch.setattr(python_native_environment.dimos, "__file__", str(package / "__init__.py"))
    monkeypatch.setattr(python_native_environment, "version", lambda _: "1.2.3")

    project = _project(tmp_path / "runtime")

    assert project.run_command("python")[-3:] == ["--with", "dimos==1.2.3", "python"]


def test_locked_project_is_required(tmp_path: Path) -> None:
    project = tmp_path / "python"
    project.mkdir()
    (project / "pyproject.toml").touch()

    with pytest.raises(FileNotFoundError, match="runtime lock is missing"):
        PythonNativeProject(project)


def test_python_native_project_resolves_sibling_project(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = tmp_path / "contract.py"
    source.touch()
    project = tmp_path / "python"
    project.mkdir()
    (project / "pyproject.toml").touch()
    (project / "uv.lock").touch()
    monkeypatch.setattr(python_native_environment.inspect, "getfile", lambda _: str(source))

    assert PythonNativeProject.sibling(type("Contract", (), {})).path == project


def test_python_native_project_requires_sibling_directory(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = tmp_path / "contract.py"
    source.touch()
    monkeypatch.setattr(python_native_environment.inspect, "getfile", lambda _: str(source))

    with pytest.raises(FileNotFoundError, match="sibling 'python/'"):
        PythonNativeProject.sibling(type("Contract", (), {}))


def test_project_owns_environment_and_command_execution(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, mocker: MockerFixture
) -> None:
    project = _project(tmp_path)
    monkeypatch.setenv("VIRTUAL_ENV", "/parent/.venv")
    run = mocker.patch.object(python_native_environment.subprocess, "run")

    project.run(project.sync_command(), extra_env={"EXAMPLE_SETTING": "configured"})

    run.assert_called_once()
    kwargs = run.call_args.kwargs
    assert kwargs["cwd"] == project.path
    assert "VIRTUAL_ENV" not in kwargs["env"]
    assert kwargs["env"]["EXAMPLE_SETTING"] == "configured"
    assert kwargs["env"]["UV_PROJECT_ENVIRONMENT"] == str(project.environment_path)


def test_callable_command_uses_generic_runner(tmp_path: Path) -> None:
    project = _project(tmp_path)

    command = project.callable_command("package.module:convert", ["input.mcap", "dataset"])

    assert command[-6:] == [
        "python",
        "-m",
        "dimos.core.python_native_call",
        "package.module:convert",
        "input.mcap",
        "dataset",
    ]
