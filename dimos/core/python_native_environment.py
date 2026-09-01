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

"""Resolve locked Python-native projects without writing into their package."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import hashlib
from importlib.metadata import version
import inspect
import os
from pathlib import Path
import re
import subprocess
from typing import Any

import dimos
from dimos.constants import CACHE_DIR

PYTHON_NATIVE_VERSION = "3.12"


def _dimos_overlay_args() -> list[str]:
    """Overlay the running checkout or exact installed DimOS release."""
    package_root = Path(dimos.__file__).resolve().parent
    source_root = package_root.parent
    if (source_root / "pyproject.toml").is_file() and (source_root / ".git").exists():
        return ["--with-editable", str(source_root)]
    return ["--with", f"dimos=={version('dimos')}"]


@dataclass(frozen=True)
class PythonNativeProject:
    """One locked Python project and every command needed to run it."""

    path: Path

    def __post_init__(self) -> None:
        path = self.path.resolve()
        if not path.is_dir():
            raise FileNotFoundError(f"Python-native runtime project is missing: {path}")
        if not (path / "pyproject.toml").is_file():
            raise FileNotFoundError(
                f"Python-native runtime manifest is missing: {path / 'pyproject.toml'}"
            )
        if not (path / "uv.lock").is_file():
            raise FileNotFoundError(f"Python-native runtime lock is missing: {path / 'uv.lock'}")
        object.__setattr__(self, "path", path)

    @classmethod
    def sibling(cls, module_class: type[Any]) -> PythonNativeProject:
        """Resolve the locked ``python/`` project beside a module contract."""
        source = Path(inspect.getfile(module_class)).resolve()
        path = source.parent / "python"
        try:
            return cls(path)
        except FileNotFoundError as error:
            if not path.is_dir():
                raise FileNotFoundError(
                    f"Python-native runtime project is missing: {path}; "
                    "create a sibling 'python/' directory"
                ) from error
            raise

    @property
    def environment_path(self) -> Path:
        digest = hashlib.sha256((self.path / "uv.lock").read_bytes()).hexdigest()[:12]
        name = re.sub(r"[^a-zA-Z0-9_.-]+", "-", self.path.parent.name)
        return CACHE_DIR / "python-native" / f"{name}-{digest}"

    def environment(self, extra: Mapping[str, str] | None = None) -> dict[str, str]:
        env = dict(os.environ)
        env.pop("VIRTUAL_ENV", None)
        env.update(extra or {})
        env["UV_PROJECT_ENVIRONMENT"] = str(self.environment_path)
        return env

    def _uv(self, *args: str) -> list[str]:
        command = ["uv", *args]
        if (self.path / "pixi.toml").is_file():
            return ["pixi", "run", "--executable", *command]
        return command

    def sync_command(self) -> list[str]:
        return self._uv(
            "sync",
            "--project",
            str(self.path),
            "--locked",
            "--python",
            PYTHON_NATIVE_VERSION,
        )

    def run_command(self, *command: str) -> list[str]:
        return self._uv(
            "run",
            "--project",
            str(self.path),
            "--locked",
            "--python",
            PYTHON_NATIVE_VERSION,
            *_dimos_overlay_args(),
            *command,
        )

    def callable_command(self, declaration: str, args: Sequence[str] = ()) -> list[str]:
        """Build a one-shot command for ``module:function`` inside this project."""
        return self.run_command(
            "python",
            "-m",
            "dimos.core.python_native_call",
            declaration,
            *args,
        )

    def run(
        self,
        command: Sequence[str],
        *,
        extra_env: Mapping[str, str] | None = None,
    ) -> subprocess.CompletedProcess[str]:
        """Run a prepared project command and capture its diagnostics."""
        return subprocess.run(
            command,
            cwd=self.path,
            env=self.environment(extra_env),
            capture_output=True,
            text=True,
        )
