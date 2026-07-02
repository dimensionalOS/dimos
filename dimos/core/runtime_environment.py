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

from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field
import os
from pathlib import Path
import sys


@dataclass(frozen=True)
class PythonLaunchMaterial:
    python_executable: Path
    env: Mapping[str, str] = field(default_factory=dict)


@dataclass(frozen=True)
class PythonProjectLaunchMaterial:
    argv_prefix: tuple[str, ...]
    cwd: Path
    env: Mapping[str, str] = field(default_factory=dict)
    runtime_name: str = ""
    project: Path = Path()
    convention: str = "uv"
    prepared_python: Path = Path()


@dataclass(frozen=True)
class RuntimePlacement:
    runtime: str
    implementation: str


class RuntimeEnvironmentError(RuntimeError):
    pass


class RuntimeEnvironmentRegistrationError(RuntimeEnvironmentError):
    pass


class UnknownRuntimeEnvironmentError(RuntimeEnvironmentError):
    def __init__(self, name: str, known: Iterable[str]) -> None:
        known_names = ", ".join(sorted(known)) or "<none>"
        super().__init__(f"Unknown runtime environment {name!r}. Known runtimes: {known_names}")
        self.name = name


class PythonProjectRuntimeEnvironmentError(RuntimeEnvironmentError):
    pass


class MissingPythonProjectFileError(PythonProjectRuntimeEnvironmentError):
    pass


class MissingPythonProjectLockfileError(PythonProjectRuntimeEnvironmentError):
    pass


class MissingPreparedPythonProjectError(PythonProjectRuntimeEnvironmentError):
    def __init__(self, runtime_name: str, project: Path, prepared_python: Path) -> None:
        super().__init__(
            f"Runtime project {runtime_name!r} at {project} is not prepared: "
            f"missing {prepared_python}. Deployment reconciliation should create or update "
            "runtime environment state without changing project files."
        )
        self.runtime_name = runtime_name
        self.project = project
        self.prepared_python = prepared_python


class RuntimeEnvironment:
    name: str

    def resolve_python(self) -> PythonLaunchMaterial:
        raise RuntimeEnvironmentError(f"Runtime environment {self.name!r} cannot launch Python")

    def resolve_python_project(self) -> PythonProjectLaunchMaterial:
        raise RuntimeEnvironmentError(
            f"Runtime environment {self.name!r} is not a Python Runtime Project"
        )

    @property
    def project_path(self) -> Path | None:
        return None


@dataclass(frozen=True)
class CurrentProcessRuntimeEnvironment(RuntimeEnvironment):
    name: str = "current"
    env: Mapping[str, str] = field(default_factory=lambda: dict(os.environ))

    def resolve_python(self) -> PythonLaunchMaterial:
        return PythonLaunchMaterial(python_executable=Path(sys.executable), env=dict(self.env))


@dataclass(frozen=True)
class PythonVenvRuntimeEnvironment(RuntimeEnvironment):
    name: str
    python_executable: Path | str
    env: Mapping[str, str] = field(default_factory=dict)

    def resolve_python(self) -> PythonLaunchMaterial:
        return PythonLaunchMaterial(
            python_executable=Path(self.python_executable).expanduser().resolve(),
            env=dict(self.env),
        )


@dataclass(frozen=True)
class PythonProjectRuntimeEnvironment(RuntimeEnvironment):
    name: str
    project: Path | str
    env: Mapping[str, str] = field(default_factory=dict)

    @property
    def project_path(self) -> Path:
        return Path(self.project).expanduser().resolve()

    @property
    def pyproject_path(self) -> Path:
        return self.project_path / "pyproject.toml"

    @property
    def uv_lock_path(self) -> Path:
        return self.project_path / "uv.lock"

    @property
    def pixi_toml_path(self) -> Path:
        return self.project_path / "pixi.toml"

    @property
    def pixi_lock_path(self) -> Path:
        return self.project_path / "pixi.lock"

    @property
    def prepared_python(self) -> Path:
        return self.project_path / ".venv" / "bin" / "python"

    @property
    def convention(self) -> str:
        return "pixi-backed-uv" if self.pixi_toml_path.exists() else "uv"

    def validate_project_files(self) -> None:
        if not self.pyproject_path.exists():
            raise MissingPythonProjectFileError(
                f"Runtime project {self.name!r} is missing {self.pyproject_path}"
            )
        if not self.uv_lock_path.exists():
            raise MissingPythonProjectLockfileError(
                f"Runtime project {self.name!r} is missing committed lockfile {self.uv_lock_path}. "
                "Run a manual package-manager lock/update command or future DimOS build/update "
                "command; deployment reconciliation will not rewrite lockfiles."
            )
        if self.pixi_toml_path.exists() and not self.pixi_lock_path.exists():
            raise MissingPythonProjectLockfileError(
                f"Runtime project {self.name!r} is missing committed lockfile {self.pixi_lock_path}. "
                "Run `pixi lock` or a future DimOS build/update command; deployment "
                "reconciliation will not rewrite lockfiles."
            )

    def resolve_python_project(self) -> PythonProjectLaunchMaterial:
        self.validate_project_files()
        if not self.prepared_python.exists():
            raise MissingPreparedPythonProjectError(
                self.name, self.project_path, self.prepared_python
            )
        argv_prefix = (
            ("pixi", "run", "uv", "run", "--no-sync", "python")
            if self.convention == "pixi-backed-uv"
            else ("uv", "run", "--no-sync", "python")
        )
        return PythonProjectLaunchMaterial(
            argv_prefix=argv_prefix,
            cwd=self.project_path,
            env=dict(self.env),
            runtime_name=self.name,
            project=self.project_path,
            convention=self.convention,
            prepared_python=self.prepared_python,
        )


@dataclass(frozen=True)
class RuntimeEnvironmentRegistry:
    environments: Mapping[str, RuntimeEnvironment] = field(default_factory=dict)

    @classmethod
    def with_current_process(cls) -> RuntimeEnvironmentRegistry:
        return cls({"current": CurrentProcessRuntimeEnvironment()})

    def register(self, *environments: RuntimeEnvironment) -> RuntimeEnvironmentRegistry:
        merged = dict(self.environments)
        for environment in environments:
            if environment.name in merged:
                raise RuntimeEnvironmentRegistrationError(
                    f"Runtime environment {environment.name!r} is already registered"
                )
            merged[environment.name] = environment
        _validate_unique_project_paths(merged)
        return RuntimeEnvironmentRegistry(merged)

    def merge(self, other: RuntimeEnvironmentRegistry) -> RuntimeEnvironmentRegistry:
        merged = dict(self.environments)
        for name, environment in other.environments.items():
            if name in merged:
                if isinstance(merged[name], CurrentProcessRuntimeEnvironment) and isinstance(
                    environment, CurrentProcessRuntimeEnvironment
                ):
                    continue
                if merged[name] != environment:
                    raise RuntimeEnvironmentRegistrationError(
                        f"Runtime environment {name!r} is registered more than once"
                    )
                continue
            merged[name] = environment
        _validate_unique_project_paths(merged)
        return RuntimeEnvironmentRegistry(merged)

    def resolve(self, name: str) -> RuntimeEnvironment:
        try:
            return self.environments[name]
        except KeyError as e:
            raise UnknownRuntimeEnvironmentError(name, self.environments) from e


def _validate_unique_project_paths(environments: Mapping[str, RuntimeEnvironment]) -> None:
    paths: dict[Path, str] = {}
    for name, environment in environments.items():
        project_path = environment.project_path
        if project_path is None:
            continue
        if project_path in paths:
            other_name = paths[project_path]
            raise RuntimeEnvironmentRegistrationError(
                f"Runtime environments {other_name!r} and {name!r} use duplicate "
                f"Runtime Project path {project_path}"
            )
        paths[project_path] = name
