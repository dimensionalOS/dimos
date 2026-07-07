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

from collections.abc import Mapping
from dataclasses import dataclass, field
import os
from pathlib import Path


@dataclass(frozen=True)
class PythonProjectLaunchMaterial:
    argv_prefix: tuple[str, ...]
    cwd: Path
    env: Mapping[str, str] = field(default_factory=dict)
    runtime_name: str = ""
    project: Path = Path()
    has_pixi: bool = False
    prepared_python: Path = Path()


@dataclass(frozen=True)
class RuntimePlacement:
    runtime: str
    implementation: str


class RuntimeEnvironmentError(RuntimeError):
    pass


class RuntimeEnvironment:
    name: str

    def resolve_python_project(self) -> PythonProjectLaunchMaterial:
        raise RuntimeEnvironmentError(
            f"Runtime environment {self.name!r} is not a Python Runtime Project"
        )

    @property
    def project_path(self) -> Path | None:
        return None


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
    def has_pixi(self) -> bool:
        return self.pixi_toml_path.exists()

    def validate_project_files(self) -> None:
        if not self.pyproject_path.exists():
            raise RuntimeEnvironmentError(
                f"Runtime project {self.name!r} is missing {self.pyproject_path}"
            )
        if not self.uv_lock_path.exists():
            raise RuntimeEnvironmentError(
                f"Runtime project {self.name!r} is missing committed lockfile {self.uv_lock_path}. "
                "Run a manual package-manager lock/update command or future DimOS build/update "
                "command; deployment reconciliation will not rewrite lockfiles."
            )
        if self.pixi_toml_path.exists() and not self.pixi_lock_path.exists():
            raise RuntimeEnvironmentError(
                f"Runtime project {self.name!r} is missing committed lockfile {self.pixi_lock_path}. "
                "Run `pixi lock` or a future DimOS build/update command; deployment "
                "reconciliation will not rewrite lockfiles."
            )

    def resolve_python_project(self) -> PythonProjectLaunchMaterial:
        self.validate_project_files()
        if not self.prepared_python.exists():
            raise RuntimeEnvironmentError(
                f"Runtime project {self.name!r} at {self.project_path} is not prepared: "
                f"missing {self.prepared_python}. Deployment reconciliation should create or update "
                "runtime environment state without changing project files."
            )
        env = dict(self.env)
        env["PYTHONPATH"] = _merge_pythonpath(str(_dimos_import_path()), env.get("PYTHONPATH", ""))
        return PythonProjectLaunchMaterial(
            argv_prefix=(str(self.prepared_python),),
            cwd=self.project_path,
            env=env,
            runtime_name=self.name,
            project=self.project_path,
            has_pixi=self.has_pixi,
            prepared_python=self.prepared_python,
        )


@dataclass(frozen=True)
class RuntimeEnvironmentRegistry:
    environments: Mapping[str, RuntimeEnvironment] = field(default_factory=dict)

    def register(self, *environments: RuntimeEnvironment) -> RuntimeEnvironmentRegistry:
        merged = dict(self.environments)
        for environment in environments:
            if environment.name in merged:
                raise RuntimeEnvironmentError(
                    f"Runtime environment {environment.name!r} is already registered"
                )
            merged[environment.name] = environment
        _validate_unique_project_paths(merged)
        return RuntimeEnvironmentRegistry(merged)

    def merge(self, other: RuntimeEnvironmentRegistry) -> RuntimeEnvironmentRegistry:
        merged = dict(self.environments)
        for name, environment in other.environments.items():
            if name in merged:
                if merged[name] != environment:
                    raise RuntimeEnvironmentError(
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
            known_names = ", ".join(sorted(self.environments)) or "<none>"
            raise RuntimeEnvironmentError(
                f"Unknown runtime environment {name!r}. Known runtimes: {known_names}"
            ) from e


def _validate_unique_project_paths(environments: Mapping[str, RuntimeEnvironment]) -> None:
    paths: dict[Path, str] = {}
    for name, environment in environments.items():
        project_path = environment.project_path
        if project_path is None:
            continue
        if project_path in paths:
            other_name = paths[project_path]
            raise RuntimeEnvironmentError(
                f"Runtime environments {other_name!r} and {name!r} use duplicate "
                f"Runtime Project path {project_path}"
            )
        paths[project_path] = name


def _dimos_import_path() -> Path:
    return Path(__file__).resolve().parents[2]


def _merge_pythonpath(*values: str) -> str:
    paths: list[str] = []
    for value in values:
        paths.extend(path for path in value.split(os.pathsep) if path)
    return os.pathsep.join(dict.fromkeys(paths))
