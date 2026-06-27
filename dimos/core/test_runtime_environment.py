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
    NativeRuntimeEnvironment,
    NixNativeRuntimeEnvironment,
    PythonVenvRuntimeEnvironment,
    RuntimeEnvironmentRegistry,
)


def test_register_and_resolve_environment() -> None:
    env = PythonVenvRuntimeEnvironment(name="tools", python_executable=Path("python"))
    registry = RuntimeEnvironmentRegistry.with_current_process().register(env)

    assert registry.resolve("tools") is env


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
