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

"""Serve the documentation site locally."""

from __future__ import annotations

import os
from pathlib import Path
import shutil
import subprocess
import sys

import typer


def _repo_root() -> Path | None:
    """The checkout holding mkdocs.yml, or None when running from a wheel."""
    for candidate in Path(__file__).resolve().parents:
        if (candidate / "mkdocs.yml").is_file():
            return candidate
    return None


def docs(
    port: int = typer.Option(8000, "--port", "-p", help="Port to serve on."),
    host: str = typer.Option("127.0.0.1", "--host", help="Address to bind."),
    build: bool = typer.Option(False, "--build", help="Build into site/ instead of serving."),
) -> None:
    """Serve the docs at http://127.0.0.1:8000, live-reloading as you edit."""
    root = _repo_root()
    if root is None:
        typer.echo(
            "No mkdocs.yml found. The docs are built from a source checkout;\n"
            "read them online at https://dimensionalos.github.io/mkdocs/",
            err=True,
        )
        raise typer.Exit(1)

    # An installed mkdocs is preferred over `uv run`, which syncs the project
    # and its default groups before running anything. The site needs none of
    # that, and the sync is slower than the build it precedes.
    venv_mkdocs = root / ".venv" / "bin" / "mkdocs"
    if venv_mkdocs.is_file():
        command = [str(venv_mkdocs)]
    elif shutil.which("mkdocs"):
        command = ["mkdocs"]
    elif shutil.which("uv"):
        command = ["uv", "run", "mkdocs"]
    else:
        typer.echo("mkdocs is not installed. Install it with: uv sync --group docs", err=True)
        raise typer.Exit(1)

    command += ["build"] if build else ["serve", "-a", f"{host}:{port}"]

    # mkdocs-material prints a banner on every run about mkdocs 2.0 breaking
    # plugins and theme overrides. It is a notice about an upstream dispute,
    # not a diagnostic for this build, and our versions are pinned in uv.lock.
    # Unset NO_MKDOCS_2_WARNING to read it again.
    env = {**os.environ, "NO_MKDOCS_2_WARNING": "true"}
    sys.exit(subprocess.call(command, cwd=root, env=env))
