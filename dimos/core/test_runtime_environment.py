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

from pathlib import Path
import sys

import pytest

from dimos.core.runtime_environment import (
    CurrentProcessRuntimeEnvironment,
    MissingPreparedPythonProjectError,
    MissingPythonProjectFileError,
    MissingPythonProjectLockfileError,
    PythonProjectRuntimeEnvironment,
    PythonVenvRuntimeEnvironment,
    RuntimeEnvironmentRegistrationError,
    RuntimeEnvironmentRegistry,
    UnknownRuntimeEnvironmentError,
)


def test_registry_with_current_process_resolves_sys_executable() -> None:
    registry = RuntimeEnvironmentRegistry.with_current_process()

    runtime = registry.resolve("current")

    assert isinstance(runtime, CurrentProcessRuntimeEnvironment)
    assert runtime.resolve_python().python_executable == Path(sys.executable)


def test_register_and_merge_reject_duplicate_names() -> None:
    registry = RuntimeEnvironmentRegistry.with_current_process()

    with pytest.raises(RuntimeEnvironmentRegistrationError, match="already registered"):
        registry.register(CurrentProcessRuntimeEnvironment())

    conflicting_current = RuntimeEnvironmentRegistry(
        {"current": PythonVenvRuntimeEnvironment(name="current", python_executable=sys.executable)}
    )
    with pytest.raises(RuntimeEnvironmentRegistrationError, match="registered more than once"):
        registry.merge(conflicting_current)

    assert registry.merge(RuntimeEnvironmentRegistry.with_current_process()) == registry


def test_duplicate_python_project_runtime_environment_project_path_rejected(tmp_path: Path) -> None:
    first = PythonProjectRuntimeEnvironment(name="first", project=tmp_path)
    second = PythonProjectRuntimeEnvironment(name="second", project=tmp_path / ".")

    with pytest.raises(RuntimeEnvironmentRegistrationError, match="duplicate Runtime Project path"):
        RuntimeEnvironmentRegistry().register(first, second)


def test_unknown_runtime_error_lists_known_runtime_names() -> None:
    registry = RuntimeEnvironmentRegistry.with_current_process()

    with pytest.raises(UnknownRuntimeEnvironmentError, match="Known runtimes: current"):
        registry.resolve("missing")


def test_python_project_requires_pyproject_and_uv_lock(tmp_path: Path) -> None:
    runtime = PythonProjectRuntimeEnvironment(name="project", project=tmp_path)

    with pytest.raises(MissingPythonProjectFileError, match="pyproject.toml"):
        runtime.validate_project_files()

    runtime.pyproject_path.write_text("[project]\nname = 'example'\n", encoding="utf-8")

    with pytest.raises(MissingPythonProjectLockfileError, match="uv.lock"):
        runtime.validate_project_files()


def test_pixi_toml_requires_pixi_lock(tmp_path: Path) -> None:
    runtime = PythonProjectRuntimeEnvironment(name="project", project=tmp_path)
    runtime.pyproject_path.write_text("[project]\nname = 'example'\n", encoding="utf-8")
    runtime.uv_lock_path.write_text("version = 1\n", encoding="utf-8")
    runtime.pixi_toml_path.write_text("[workspace]\n", encoding="utf-8")

    with pytest.raises(MissingPythonProjectLockfileError, match="pixi.lock"):
        runtime.validate_project_files()


def test_resolve_python_project_requires_prepared_python_after_validation(tmp_path: Path) -> None:
    runtime = PythonProjectRuntimeEnvironment(name="project", project=tmp_path)
    runtime.pyproject_path.write_text("[project]\nname = 'example'\n", encoding="utf-8")
    runtime.uv_lock_path.write_text("version = 1\n", encoding="utf-8")

    with pytest.raises(MissingPreparedPythonProjectError, match=r"\.venv/bin/python"):
        runtime.resolve_python_project()

    runtime.prepared_python.parent.mkdir(parents=True)
    runtime.prepared_python.write_text("#!/usr/bin/env python\n", encoding="utf-8")

    material = runtime.resolve_python_project()
    assert material.argv_prefix == ("uv", "run", "--no-sync", "python")
    assert material.prepared_python == runtime.prepared_python
