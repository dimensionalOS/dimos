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

from dataclasses import dataclass, field
import os
from pathlib import Path
import sys


@dataclass(frozen=True)
class PythonLaunchMaterial:
    python_executable: Path
    env: dict[str, str] = field(default_factory=dict)


@dataclass(frozen=True)
class PythonProjectLaunchMaterial:
    argv_prefix: list[str]
    cwd: Path
    env: dict[str, str] = field(default_factory=dict)
    runtime_name: str = ""
    project: Path = Path()
    convention: str = "uv"
    prepared_python: Path = Path()


@dataclass(frozen=True)
class NativeLaunchMaterial:
    executable: str
    build_command: str | None = None
    cwd: Path | None = None
    env: dict[str, str] = field(default_factory=dict)


class RuntimeEnvironment:
    name: str

    def resolve_python(self) -> PythonLaunchMaterial:
        raise RuntimeError(
            f"Runtime environment '{self.name}' does not provide Python launch material"
        )

    def resolve_python_project(self) -> PythonProjectLaunchMaterial:
        raise RuntimeError(
            f"Runtime environment '{self.name}' does not provide Python project launch material"
        )

    def resolve_native(self) -> NativeLaunchMaterial:
        raise RuntimeError(
            f"Runtime environment '{self.name}' does not provide native launch material"
        )


@dataclass(frozen=True)
class CurrentProcessRuntimeEnvironment(RuntimeEnvironment):
    name: str = "current"

    def resolve_python(self) -> PythonLaunchMaterial:
        return PythonLaunchMaterial(python_executable=Path(sys.executable), env=dict(os.environ))


@dataclass(frozen=True)
class PythonVenvRuntimeEnvironment(RuntimeEnvironment):
    name: str
    python_executable: Path
    env: dict[str, str] = field(default_factory=dict)

    def resolve_python(self) -> PythonLaunchMaterial:
        return PythonLaunchMaterial(python_executable=self.python_executable, env=dict(self.env))


class PythonProjectRuntimeEnvironmentError(RuntimeError):
    """Base error for invalid Python project runtime environments."""


class MissingPythonProjectFileError(PythonProjectRuntimeEnvironmentError):
    def __init__(self, *, runtime_name: str, project: Path, missing_file: Path) -> None:
        self.runtime_name = runtime_name
        self.project = project
        self.missing_file = missing_file
        super().__init__(
            "Python project runtime "
            f"'{runtime_name}' at '{project}' is missing required project file "
            f"'{missing_file}'. First-slice Python project runtimes require pyproject.toml."
        )


class MissingPreparedPythonProjectError(PythonProjectRuntimeEnvironmentError):
    def __init__(
        self,
        *,
        runtime_name: str,
        project: Path,
        missing_executable: Path,
        convention: str,
    ) -> None:
        self.runtime_name = runtime_name
        self.project = project
        self.missing_executable = missing_executable
        self.convention = convention
        prepare_command = f"dimos runtime prepare <blueprint> --runtime {runtime_name}"
        super().__init__(
            "Python project runtime "
            f"'{runtime_name}' at '{project}' is not prepared; missing executable "
            f"'{missing_executable}'. Detected convention: {convention}. "
            f"Prepare it with: {prepare_command}"
        )


@dataclass(frozen=True)
class PythonProjectRuntimeEnvironment(RuntimeEnvironment):
    name: str
    project: Path

    def __post_init__(self) -> None:
        object.__setattr__(self, "project", Path(self.project).expanduser().resolve())

    @property
    def pyproject_toml(self) -> Path:
        return self.project / "pyproject.toml"

    @property
    def pixi_toml(self) -> Path:
        return self.project / "pixi.toml"

    @property
    def prepared_python(self) -> Path:
        return self.project / ".venv" / "bin" / "python"

    def convention(self) -> str:
        self._validate_project_file()
        if self.pixi_toml.exists():
            return "pixi-backed-uv"
        return "uv"

    def resolve_python_project(self) -> PythonProjectLaunchMaterial:
        convention = self.convention()
        if not self.prepared_python.exists():
            raise MissingPreparedPythonProjectError(
                runtime_name=self.name,
                project=self.project,
                missing_executable=self.prepared_python,
                convention=convention,
            )
        argv_prefix = ["uv", "run", "--no-sync", "python"]
        if convention == "pixi-backed-uv":
            argv_prefix = ["pixi", "run", *argv_prefix]
        return PythonProjectLaunchMaterial(
            argv_prefix=argv_prefix,
            cwd=self.project,
            env={},
            runtime_name=self.name,
            project=self.project,
            convention=convention,
            prepared_python=self.prepared_python,
        )

    def _validate_project_file(self) -> None:
        if not self.pyproject_toml.exists():
            raise MissingPythonProjectFileError(
                runtime_name=self.name,
                project=self.project,
                missing_file=self.pyproject_toml,
            )


@dataclass(frozen=True)
class NativeRuntimeEnvironment(RuntimeEnvironment):
    name: str
    executable: str
    build_command: str | None = None
    cwd: Path | None = None
    env: dict[str, str] = field(default_factory=dict)

    def resolve_native(self) -> NativeLaunchMaterial:
        return NativeLaunchMaterial(
            executable=self.executable,
            build_command=self.build_command,
            cwd=self.cwd,
            env=dict(self.env),
        )


@dataclass(frozen=True)
class NixNativeRuntimeEnvironment(NativeRuntimeEnvironment):
    """Named native runtime backed by externally-produced Nix launch material.

    This is intentionally a thin typed model over native launch material. DimOS
    does not evaluate Nix expressions here; callers register the resolved
    executable/build/cwd/env material.
    """


@dataclass(frozen=True)
class RuntimeEnvironmentRegistry:
    environments: dict[str, RuntimeEnvironment] = field(default_factory=dict)

    @classmethod
    def with_current_process(cls) -> RuntimeEnvironmentRegistry:
        current = CurrentProcessRuntimeEnvironment()
        return cls(environments={current.name: current})

    def register(self, environment: RuntimeEnvironment) -> RuntimeEnvironmentRegistry:
        return RuntimeEnvironmentRegistry(
            environments={**self.environments, environment.name: environment}
        )

    def merge(self, other: RuntimeEnvironmentRegistry) -> RuntimeEnvironmentRegistry:
        return RuntimeEnvironmentRegistry(environments={**self.environments, **other.environments})

    def resolve(self, name: str) -> RuntimeEnvironment:
        try:
            return self.environments[name]
        except KeyError as exc:
            known = ", ".join(sorted(self.environments)) or "<none>"
            raise KeyError(
                f"Unknown runtime environment '{name}'. Known environments: {known}"
            ) from exc
