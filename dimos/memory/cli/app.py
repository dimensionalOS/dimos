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

"""``dimos mem`` — memory store commands."""

from __future__ import annotations

import json
from pathlib import Path

import typer

from dimos.memory.cli.summary import main as _summary_main
from dimos.memory.convert_recording import convert as convert_recording
from dimos.memory.recording_migration import inspect_recording, migrate

mem_app = typer.Typer(help="memory store commands", no_args_is_help=True)
mem_app.command("summary")(_summary_main)


@mem_app.command()
def rerun(
    path: str = typer.Argument(..., help="Store: bare name (cwd, data/, LFS), .db or .mcap path"),
    out: str = typer.Option(None, "--out", help="Output .rrd (default: alongside the source)"),
    seconds: float = typer.Option(None, "--seconds", help="Only the first N seconds"),
    no_gui: bool = typer.Option(False, "--no-gui", help="Write the .rrd but don't open the viewer"),
    root: str = typer.Option(
        None, "--root", help="Nest every stream under this entity path (<root>/<name>)"
    ),
) -> None:
    """Render a memory store into rerun (writes a .rrd, then opens the viewer)."""
    from dimos.memory.cli.dataset import open_dataset
    from dimos.memory.cli.render import render_store

    render_store(open_dataset(path), out=out, seconds=seconds, no_gui=no_gui, root=root)


@mem_app.command("convert")
def convert(
    source: Path = typer.Argument(..., help="Local recording file or explicit batch directory"),
    destination: Path = typer.Argument(..., help="New output file or new batch directory"),
    output_format: str = typer.Option(
        "mcap", "--format", help="Directory output format: mcap or db"
    ),
    dry_run: bool = typer.Option(
        False, "--dry-run", help="Read-only type/schema/count preflight; no outputs or downloads"
    ),
) -> None:
    """Migrate legacy recordings to generated CDR; originals are never overwritten."""
    try:
        if source.is_dir():
            result = migrate(source, destination, output_format=output_format, dry_run=dry_run)
        elif dry_run:
            if (
                destination.exists()
                or destination.is_symlink()
                or Path(str(destination) + ".conversion.jsonl").exists()
                or Path(str(destination) + ".conversion.jsonl").is_symlink()
            ):
                raise FileExistsError("Refusing an existing output or report")
            result = inspect_recording(source, destination.suffix.lstrip("."))
        else:
            result = convert_recording(source, destination)
    except Exception as exc:
        typer.echo(f"conversion failed: {exc}", err=True)
        raise typer.Exit(2) from exc
    typer.echo(json.dumps(result, indent=2))
    if result.get("blocked", 0):
        raise typer.Exit(2)
