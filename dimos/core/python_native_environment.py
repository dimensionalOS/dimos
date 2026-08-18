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

from collections.abc import Mapping
import hashlib
from importlib.metadata import version
import inspect
from pathlib import Path
import re
from typing import Any

import dimos
from dimos.constants import CACHE_DIR

PYTHON_NATIVE_VERSION = "3.12"


def require_locked_project(project: Path) -> Path:
    """Validate and return a Python-native project shipped with DimOS."""
    project = project.resolve()
    manifest = project / "pyproject.toml"
    lock = project / "uv.lock"
    if not project.is_dir():
        raise FileNotFoundError(f"Python-native runtime project is missing: {project}")
    if not manifest.is_file():
        raise FileNotFoundError(f"Python-native runtime manifest is missing: {manifest}")
    if not lock.is_file():
        raise FileNotFoundError(f"Python-native runtime lock is missing: {lock}")
    return project


def python_native_project(module_class: type[Any]) -> Path:
    """Return the locked sibling project for a Python-native module contract."""
    source = Path(inspect.getfile(module_class)).resolve()
    project = source.parent / "python"
    try:
        return require_locked_project(project)
    except FileNotFoundError as error:
        if not project.is_dir():
            raise FileNotFoundError(
                f"Python-native runtime project is missing: {project}; "
                "create a sibling 'python/' directory"
            ) from error
        raise


def dimos_overlay_args() -> list[str]:
    """Overlay the running checkout or exact installed DimOS release."""
    package_root = Path(dimos.__file__).resolve().parent
    source_root = package_root.parent
    if (source_root / "pyproject.toml").is_file() and (source_root / ".git").exists():
        return ["--with-editable", str(source_root)]
    return ["--with", f"dimos=={version('dimos')}"]


def project_environment(project: Path) -> Path:
    """Return a writable, lock-versioned virtualenv location for ``project``."""
    project = require_locked_project(project)
    digest = hashlib.sha256((project / "uv.lock").read_bytes()).hexdigest()[:12]
    name = re.sub(r"[^a-zA-Z0-9_.-]+", "-", project.parent.name)
    return CACHE_DIR / "python-native" / f"{name}-{digest}"


def project_environment_vars(
    project: Path, extra: Mapping[str, str] | None = None
) -> dict[str, str]:
    """Environment additions used by uv for a packaged project."""
    result = dict(extra or {})
    result["UV_PROJECT_ENVIRONMENT"] = str(project_environment(project))
    return result


def uv_sync_command(project: Path) -> list[str]:
    """Build the command that installs a locked project's third-party stack."""
    project = require_locked_project(project)
    return [
        "uv",
        "sync",
        "--project",
        str(project),
        "--locked",
        "--python",
        PYTHON_NATIVE_VERSION,
    ]


def uv_run_command(project: Path, *command: str) -> list[str]:
    """Run inside a locked project with the current DimOS overlaid."""
    project = require_locked_project(project)
    return [
        "uv",
        "run",
        "--project",
        str(project),
        "--locked",
        "--python",
        PYTHON_NATIVE_VERSION,
        *dimos_overlay_args(),
        *command,
    ]
