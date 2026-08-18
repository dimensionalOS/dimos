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

from dimos.core import python_native_environment


def test_source_checkout_is_overlaid_editably(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    package = tmp_path / "dimos"
    package.mkdir()
    (package / "__init__.py").touch()
    (tmp_path / "pyproject.toml").touch()
    (tmp_path / ".git").touch()
    monkeypatch.setattr(python_native_environment.dimos, "__file__", str(package / "__init__.py"))

    assert python_native_environment.dimos_overlay_args() == ["--with-editable", str(tmp_path)]


def test_installed_release_is_overlaid_at_exact_version(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    package = tmp_path / "site-packages" / "dimos"
    package.mkdir(parents=True)
    (package / "__init__.py").touch()
    monkeypatch.setattr(python_native_environment.dimos, "__file__", str(package / "__init__.py"))
    monkeypatch.setattr(python_native_environment, "version", lambda _: "1.2.3")

    assert python_native_environment.dimos_overlay_args() == ["--with", "dimos==1.2.3"]


def test_locked_project_is_required(tmp_path: Path) -> None:
    project = tmp_path / "python"
    project.mkdir()
    (project / "pyproject.toml").touch()

    with pytest.raises(FileNotFoundError, match="runtime lock is missing"):
        python_native_environment.require_locked_project(project)


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

    assert python_native_environment.python_native_project(type("Contract", (), {})) == project


def test_python_native_project_requires_sibling_directory(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    source = tmp_path / "contract.py"
    source.touch()
    monkeypatch.setattr(python_native_environment.inspect, "getfile", lambda _: str(source))

    with pytest.raises(FileNotFoundError, match="sibling 'python/'"):
        python_native_environment.python_native_project(type("Contract", (), {}))
