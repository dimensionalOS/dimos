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

"""CLI commands for inspecting and cleaning DimOS caches."""

import os

import typer

from dimos.constants import CACHE_DIR
from dimos.core.run_registry import get_most_recent
from dimos.utils.cache import CacheInUseError, cache_cleanup_guard, clean_caches

app = typer.Typer(help="Manage DimOS caches", no_args_is_help=True)


@app.command("clean")
def clean(
    force: bool = typer.Option(
        False,
        "--force",
        "-f",
        help="Delete cached robot Git checkouts containing local work",
    ),
    yes: bool = typer.Option(
        False,
        "--yes",
        "-y",
        help="Skip the confirmation prompt",
    ),
) -> None:
    """Remove regenerable files cached by DimOS."""
    _refuse_active_run()

    if not os.path.lexists(CACHE_DIR):
        typer.echo(f"No DimOS cache found at {CACHE_DIR}.")
        return

    typer.echo("DimOS cache cleanup")
    typer.echo("")
    typer.echo("Cache root:")
    typer.echo(f"  {CACHE_DIR}")
    typer.echo("")
    typer.echo("Current entries:")
    try:
        entries = sorted(CACHE_DIR.iterdir(), key=lambda path: path.name)
    except OSError as error:
        typer.echo(f"  Could not list cache entries: {error}", err=True)
        entries = []
    if entries:
        for entry in entries:
            typer.echo(f"  - {entry}")
    else:
        typer.echo(f"  - {CACHE_DIR} (empty cache root)")

    if force:
        typer.echo("")
        typer.echo("Force mode:")
        typer.echo("  - Removes robot Git checkouts with local changes or local-only commits")

    typer.echo("")
    if not yes and not typer.confirm("Continue?", default=False):
        typer.echo("Cache cleanup cancelled.")
        return

    try:
        with cache_cleanup_guard():
            _refuse_active_run()
            result = clean_caches(force=force)
    except CacheInUseError:
        typer.echo(
            "Error: DimOS is starting or running. Stop it before cleaning caches.",
            err=True,
        )
        raise typer.Exit(1)
    for path in result.cleaned:
        typer.echo(f"Cleaned cache: {path}")
    for issue in result.skipped:
        typer.echo(f"Skipped cache: {issue.path} ({issue.reason})", err=True)
    for issue in result.failed:
        typer.echo(f"Failed to remove cache: {issue.path} ({issue.reason})", err=True)

    if not result.complete:
        raise typer.Exit(1)


def _refuse_active_run() -> None:
    active_run = get_most_recent(alive_only=True)
    if active_run is not None:
        typer.echo(
            f"Error: DimOS run {active_run.run_id} is active. Stop it before cleaning caches.",
            err=True,
        )
        raise typer.Exit(1)
