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

"""Locate uv and run it with an environment that cannot leak the caller's venv."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import sys

from packaging.version import InvalidVersion, Version

MIN_UV = Version("0.9.25")
INSTALL_HINT = "Install uv: curl -LsSf https://astral.sh/uv/install.sh | sh  (or: pip install uv)"
# Settings that would redirect uv away from the managed environment.
LEAKING_VARIABLES = (
    "VIRTUAL_ENV",
    "CONDA_PREFIX",
    "UV_PYTHON",
    "UV_PROJECT",
    "UV_PROJECT_ENVIRONMENT",
    "UV_FROZEN",
    "UV_LOCKED",
    "UV_NO_SYNC",
    "UV_FIND_LINKS",
    "UV_OFFLINE",
)


class UvNotFoundError(RuntimeError):
    pass


def find_uv() -> list[str]:
    """The uv command: the binary on PATH, else the ``uv`` package of this interpreter."""
    found = shutil.which("uv")
    if found:
        return [found]
    if importlib.util.find_spec("uv") is not None:
        return [sys.executable, "-m", "uv"]
    raise UvNotFoundError(f"uv >= {MIN_UV} is required to prepare environments. {INSTALL_HINT}")


def uv_version(command: Sequence[str]) -> Version:
    output = subprocess.run(
        [*command, "--version"], capture_output=True, text=True, check=False
    ).stdout
    try:
        return Version(output.split()[1])
    except (IndexError, InvalidVersion) as error:
        raise UvNotFoundError(f"could not read the uv version from {output!r}") from error


def require_uv() -> list[str]:
    command = find_uv()
    version = uv_version(command)
    if version < MIN_UV:
        raise UvNotFoundError(f"uv {version} found; need >= {MIN_UV}. {INSTALL_HINT}")
    return command


class UvRunner:
    """Runs uv with inherited stdio so its progress reaches the terminal."""

    def __init__(self, command: Sequence[str] | None = None) -> None:
        self._command = list(command) if command is not None else None

    @property
    def command(self) -> list[str]:
        if self._command is None:
            self._command = require_uv()
        return self._command

    def run(self, args: Sequence[str], *, env: Mapping[str, str], cwd: Path | None = None) -> int:
        completed = subprocess.run([*self.command, *args], env=dict(env), cwd=cwd, check=False)
        return completed.returncode


def uv_environment(
    project_environment: Path, *, offline: bool = False, find_links: str | None = None
) -> dict[str, str]:
    """Environment for a uv subprocess targeting ``project_environment``."""
    env = {key: value for key, value in os.environ.items() if key not in LEAKING_VARIABLES}
    env["UV_PROJECT_ENVIRONMENT"] = str(project_environment.resolve())
    if offline:
        env["UV_OFFLINE"] = "1"
    if find_links:
        env["UV_FIND_LINKS"] = str(Path(find_links).resolve())
    return env
