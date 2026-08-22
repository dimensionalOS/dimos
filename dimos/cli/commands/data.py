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

"""`dimos data` commands; the implementation lives in dimos.cloud.data."""

from __future__ import annotations

from pathlib import Path

import typer

data_app = typer.Typer(help="Dimensional cloud data", no_args_is_help=True)


def _parse_since(s: str) -> float:
    units = {"m": 60, "h": 3600, "d": 86400}
    if s[-1] in units:
        return float(s[:-1]) * units[s[-1]]
    return float(s)


@data_app.command()
def upload(
    path: Path | None = typer.Argument(None, help="Recording or file; default: newest recording"),
    robot: str | None = typer.Option(None, "--robot", help="Robot id for attribution"),
    kind: str | None = typer.Option(None, "--kind"),
    since: str | None = typer.Option(
        None, "--since", help="Upload all recordings newer than e.g. 1h"
    ),
    chunk: int | None = typer.Option(None, "--chunk", help="Upload part size in MB"),
) -> None:
    """Upload to Dimensional cloud. Resumable: re-run after any failure and only
    missing parts transfer."""
    from dimos.cloud.data import CloudData, recordings

    cloud = CloudData()
    if since:
        targets = recordings(_parse_since(since))
        if not targets:
            typer.echo(f"no recordings in the last {since}")
            raise typer.Exit(0)
    else:
        targets = [path] if path else recordings()[-1:]
        if not targets:
            typer.echo("no recordings found — pass a path", err=True)
            raise typer.Exit(1)

    failed = False
    for t in targets:
        try:
            r = cloud.upload(t, robot_id=robot, kind=kind, chunk_mb=chunk)
            note = "already uploaded" if r["skipped"] else r["state"]
            typer.echo(f"{t.name}: {note} ({r['upload_id'][:12]})")
            if r["quota"]["state"] != "ok":
                typer.echo(r["quota"]["message"], err=True)
        except (RuntimeError, OSError) as e:
            typer.echo(f"{t.name}: {e}", err=True)
            failed = True
    if failed:
        raise typer.Exit(1)


@data_app.command("ls")
def data_ls() -> None:
    from rich.console import Console
    from rich.table import Table

    from dimos.cloud.data import CloudData

    try:
        rows = CloudData().ls()
    except RuntimeError as e:
        typer.echo(str(e), err=True)
        raise typer.Exit(1) from e
    table = Table()
    for col in ("id", "file", "kind", "robot", "size", "state"):
        table.add_column(col)
    for u in rows:
        table.add_row(
            u["id"][:12],
            u["filename"],
            u.get("kind", ""),
            u.get("robot_id") or "—",
            str(u["size"]),
            u["state"],
        )
    Console().print(table)


@data_app.command()
def pull(upload_id: str, dest: Path | None = typer.Option(None, "--dest")) -> None:
    from dimos.cloud.data import CloudData

    try:
        typer.echo(f"pulled to {CloudData().pull(upload_id, dest)}")
    except RuntimeError as e:
        typer.echo(str(e), err=True)
        raise typer.Exit(1) from e


@data_app.command("status")
def data_status(upload_id: str) -> None:
    from dimos.cloud.data import CloudData

    try:
        s = CloudData().status(upload_id)
    except RuntimeError as e:
        typer.echo(str(e), err=True)
        raise typer.Exit(1) from e
    typer.echo(s["state"] + (f" — parts on server: {len(s['parts'])}" if s["parts"] else ""))


@data_app.command()
def quota() -> None:
    from dimos.cloud.data import CloudData

    try:
        q = CloudData().quota()
    except RuntimeError as e:
        typer.echo(str(e), err=True)
        raise typer.Exit(1) from e
    typer.echo(
        f"{q['pct']}% used ({q['state']}) — {q['used_total']} bytes of {q['limits']['total_gb']} GB"
    )
