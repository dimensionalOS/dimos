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

"""Build external message packages without rebuilding the DimOS runtime."""

from importlib import import_module
from pathlib import Path
from subprocess import CalledProcessError

import typer


def build(
    project: Path = typer.Option(Path("."), "--project", help="Message project directory."),
    language: list[str] = typer.Option(
        [], "--language", help="Build only this language; repeatable."
    ),
    install: bool = typer.Option(
        False, "--install", help="Also install the wheel into the active virtualenv."
    ),
    offline: bool = typer.Option(False, "--offline", help="Require cached Cargo dependencies."),
) -> None:
    """Discover, generate and build this project's custom message packages."""
    try:
        core = import_module("dimos_message_build.build")
        config = core.Project.load(project)
        artifacts = core.build_project(
            config, tuple(language) or None, install=install, offline=offline
        )
        for kind, path in artifacts.items():
            typer.echo(f"{kind}: {path}")
    except (ValueError, FileNotFoundError, CalledProcessError) as error:
        typer.echo(f"build: {error}", err=True)
        raise typer.Exit(1) from error
