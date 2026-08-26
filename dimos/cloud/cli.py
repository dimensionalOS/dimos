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

"""CLI-facing operations for `dimos data`; commands delegate here one-line each."""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path

import typer

from dimos.cloud.data import CloudData, recordings


def _fail(e: Exception) -> None:
    typer.echo(str(e), err=True)
    raise typer.Exit(1) from e


def upload(
    path: Path | None, robot: str | None, kind: str | None, since_s: float | None, chunk: int | None
) -> None:
    try:
        cloud = CloudData()
    except RuntimeError as e:
        _fail(e)
    targets = recordings(since_s) if since_s else [path] if path else recordings()[-1:]
    if not targets:
        typer.echo("nothing to upload — pass a path", err=True)
        raise typer.Exit(1)
    failed = False
    for t in targets:
        try:
            r = cloud.upload(t, robot_id=robot, kind=kind, chunk_mb=chunk, progress=_bar(t.name))
            typer.echo(
                f"{t.name}: {'already uploaded' if r['skipped'] else r['state']} "
                f"({r['upload_id'][:12]})"
            )
            if r["quota"].get("state") not in (None, "ok"):
                typer.echo(r["quota"]["message"], err=True)
        except (RuntimeError, OSError) as e:
            typer.echo(f"{t.name}: {e}", err=True)
            failed = True
    if failed:
        raise typer.Exit(1)


def _bar(name: str) -> Callable[[str, int, int], None]:
    from rich.progress import Progress

    bar = Progress(transient=True)
    bar.start()
    task = bar.add_task(name, total=None)

    def tick(phase: str, done: int, total: int) -> None:
        bar.update(task, description=f"{phase} {name}", completed=done, total=total or None)
        if phase == "upload" and done >= total:
            bar.stop()

    return tick


def ls() -> None:
    from rich.console import Console
    from rich.filesize import decimal
    from rich.table import Table

    try:
        rows = CloudData().ls()
    except RuntimeError as e:
        _fail(e)
    table = Table("id", "file", "kind", "robot", "size", "state")
    for u in rows:
        table.add_row(
            u["id"][:12],
            u["filename"],
            u.get("kind", ""),
            u.get("robot_id") or "—",
            decimal(u["size"]),
            u["state"],
        )
    Console().print(table)


def pull(upload_id: str | None, dest: Path | None) -> None:
    try:
        typer.echo(f"pulled to {CloudData().pull(upload_id, dest)}")
    except RuntimeError as e:
        _fail(e)


def status(upload_id: str) -> None:
    try:
        s = CloudData().status(upload_id)
    except RuntimeError as e:
        _fail(e)
    typer.echo(s["state"] + (f" — parts on server: {len(s['parts'])}" if s["parts"] else ""))


def quota() -> None:
    try:
        q = CloudData().quota()
    except RuntimeError as e:
        _fail(e)
    typer.echo(
        f"{q['pct']}% used ({q['state']}) — {q['used_total']} bytes of {q['limits']['total_gb']} GB"
    )
