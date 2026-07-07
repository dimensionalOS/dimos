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
from pathlib import Path

import pytest

from dimos.core.runtime_environment import (
    PythonProjectRuntimeEnvironment,
    RuntimeEnvironmentError,
    RuntimeEnvironmentRegistry,
)


def test_register_and_merge_reject_duplicate_names() -> None:
    registry = RuntimeEnvironmentRegistry()
    runtime = PythonProjectRuntimeEnvironment(name="project", project="/tmp/project")

    registry = registry.register(runtime)

    conflicting_current = RuntimeEnvironmentRegistry(
        {"project": PythonProjectRuntimeEnvironment(name="project", project="/tmp/other")}
    )
    with pytest.raises(RuntimeEnvironmentError, match="registered more than once"):
        registry.merge(conflicting_current)

    assert registry.merge(RuntimeEnvironmentRegistry({"project": runtime})) == registry


def test_duplicate_python_project_runtime_environment_project_path_rejected(tmp_path: Path) -> None:
    first = PythonProjectRuntimeEnvironment(name="first", project=tmp_path)
    second = PythonProjectRuntimeEnvironment(name="second", project=tmp_path / ".")

    with pytest.raises(RuntimeEnvironmentError, match="duplicate Runtime Project path"):
        RuntimeEnvironmentRegistry().register(first, second)


def test_unknown_runtime_error_lists_known_runtime_names() -> None:
    registry = RuntimeEnvironmentRegistry()

    with pytest.raises(RuntimeEnvironmentError, match="Known runtimes: <none>"):
        registry.resolve("missing")


def test_python_project_requires_pyproject_and_uv_lock(tmp_path: Path) -> None:
    runtime = PythonProjectRuntimeEnvironment(name="project", project=tmp_path)

    with pytest.raises(RuntimeEnvironmentError, match="pyproject.toml"):
        runtime.validate_project_files()

    runtime.pyproject_path.write_text("[project]\nname = 'example'\n", encoding="utf-8")

    with pytest.raises(RuntimeEnvironmentError, match="uv.lock"):
        runtime.validate_project_files()


def test_pixi_toml_requires_pixi_lock(tmp_path: Path) -> None:
    runtime = PythonProjectRuntimeEnvironment(name="project", project=tmp_path)
    runtime.pyproject_path.write_text("[project]\nname = 'example'\n", encoding="utf-8")
    runtime.uv_lock_path.write_text("version = 1\n", encoding="utf-8")
    runtime.pixi_toml_path.write_text("[workspace]\n", encoding="utf-8")

    with pytest.raises(RuntimeEnvironmentError, match="pixi.lock"):
        runtime.validate_project_files()


def test_resolve_python_project_requires_prepared_python_after_validation(tmp_path: Path) -> None:
    runtime = PythonProjectRuntimeEnvironment(name="project", project=tmp_path)
    runtime.pyproject_path.write_text("[project]\nname = 'example'\n", encoding="utf-8")
    runtime.uv_lock_path.write_text("version = 1\n", encoding="utf-8")

    with pytest.raises(RuntimeEnvironmentError, match=r"\.venv/bin/python"):
        runtime.resolve_python_project()

    runtime.prepared_python.parent.mkdir(parents=True)
    runtime.prepared_python.write_text("#!/usr/bin/env python\n", encoding="utf-8")

    material = runtime.resolve_python_project()
    assert material.argv_prefix == (str(runtime.prepared_python),)
    assert material.has_pixi is False
    assert material.prepared_python == runtime.prepared_python
    assert str(Path(__file__).resolve().parents[2]) in material.env["PYTHONPATH"].split(os.pathsep)


def test_resolve_python_project_preserves_runtime_pythonpath(tmp_path: Path) -> None:
    runtime = PythonProjectRuntimeEnvironment(
        name="project",
        project=tmp_path,
        env={"PYTHONPATH": "/runtime/src"},
    )
    runtime.pyproject_path.write_text("[project]\nname = 'example'\n", encoding="utf-8")
    runtime.uv_lock_path.write_text("version = 1\n", encoding="utf-8")
    runtime.prepared_python.parent.mkdir(parents=True)
    runtime.prepared_python.write_text("#!/usr/bin/env python\n", encoding="utf-8")

    entries = runtime.resolve_python_project().env["PYTHONPATH"].split(os.pathsep)

    assert entries[0] == str(Path(__file__).resolve().parents[2])
    assert "/runtime/src" in entries
