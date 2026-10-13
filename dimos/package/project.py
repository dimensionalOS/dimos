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

"""Delegate project edits, resolution and installation to uv."""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
import os
from pathlib import Path
import subprocess

from dimos.package.sources import read_manifest


def consumer_project(path: Path | None, *, create: bool = False) -> Path:
    if path is not None:
        project = path.expanduser().resolve()
    else:
        project = Path.cwd()
        for parent in (project, *project.parents):
            if (parent / "pyproject.toml").is_file():
                project = parent
                break
        if (project / "pyproject.toml").is_file() and read_manifest(project).get("project", {}).get(
            "name"
        ) == "dimos":
            raise ValueError(
                "Select a consumer project with --project; do not edit the dimOS checkout"
            )
    manifest = project / "pyproject.toml"
    if not manifest.exists():
        if not create:
            raise ValueError(f"No consumer pyproject.toml in {project}; use package add first")
        project.mkdir(parents=True, exist_ok=True)
        manifest.write_text(
            '[project]\nname = "dimos-consumer"\nversion = "0.1.0"\nrequires-python = ">=3.10,<3.13"\ndependencies = []\n\n[tool.uv]\npackage = false\n'
        )
    return project


@contextmanager
def project_edit(project: Path) -> Iterator[None]:
    """Restore authored metadata if uv cannot complete resolution or installation."""
    paths = [project / name for name in ("pyproject.toml", "uv.lock")]
    previous = {path: path.read_bytes() if path.exists() else None for path in paths}
    try:
        yield
    except (subprocess.CalledProcessError, OSError):
        for path, content in previous.items():
            if content is None:
                path.unlink(missing_ok=True)
            else:
                path.write_bytes(content)
        raise


def uv(project: Path, *args: str) -> None:
    env = {
        key: value
        for key, value in os.environ.items()
        if key not in {"VIRTUAL_ENV", "UV_PROJECT_ENVIRONMENT", "PYTHONPATH", "PYTHONHOME"}
    }
    subprocess.run(["uv", "--project", str(project), *args], cwd=project, env=env, check=True)


def add(
    project: Path,
    source: str,
    *,
    editable: bool = False,
    tag: str | None = None,
    branch: str | None = None,
    rev: str | None = None,
    extras: tuple[str, ...] = (),
) -> None:
    args = ["add", "--no-workspace", source]
    if editable:
        args.append("--editable")
    for option, value in (("--tag", tag), ("--branch", branch), ("--rev", rev)):
        if value:
            args.extend((option, value))
    for extra in extras:
        args.extend(("--extra", extra))
    with project_edit(project):
        uv(project, *args)
