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

"""`dimos package` — set up the project in the current directory to use dimOS.

The only addition today is message packaging: an existing Python project gains
`interfaces/` for its `.msg` definitions and the build configuration that
generates, packages and registers them on `pip install`.
"""

from __future__ import annotations

from pathlib import Path

import typer

package_app = typer.Typer(
    help="Set up this Python project to use dimOS features.",
    no_args_is_help=True,
)


@package_app.command("add")
def add(
    what: str = typer.Argument(..., help="What to add. Only 'messages' is supported today."),
    directory: Path = typer.Option(
        Path("."),
        "--directory",
        "-C",
        help="Project directory holding pyproject.toml.",
        resolve_path=True,
    ),
) -> None:
    """Add message packaging to an existing Python project.

    Creates `interfaces/` and the build configuration that regenerates the
    message Python packages on every `pip install .` or `pip install -e .`.
    """
    if what != "messages":
        typer.echo(f"Only 'messages' can be added today, not {what!r}.", err=True)
        raise typer.Exit(1)

    # Imported here so `dimos --help` and every other command stay fast.
    from dimos.message_codegen.external import AddError, apply, plan

    try:
        addition = plan(directory)
    except AddError as error:
        typer.echo(f"Cannot add messages to {directory}:\n  {error}", err=True)
        raise typer.Exit(1) from error

    if not addition.changed:
        typer.echo(f"{directory} is already set up for messages ({addition.notes[0]}).")
        typer.echo(f"Definitions belong in {directory / 'interfaces'}/<schema>/msg/.")
        return

    apply(addition)
    typer.echo(f"Added message packaging to {directory}:")
    for change in addition.changes:
        typer.echo(f"  - {change}")
    for note in addition.notes:
        typer.echo(f"  note: {note}")
    typer.echo("")
    typer.echo("Next:")
    typer.echo(f"  1. write {addition.project / 'interfaces'}/<schema>/msg/<Name>.msg")
    typer.echo("  2. pip install .        (or: pip install -e .)")
    typer.echo(f"  3. from {addition.module}.<schema>.msg import <Name>")
