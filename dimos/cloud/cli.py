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

from collections.abc import Callable, Iterator
import contextlib
import functools
from pathlib import Path
from typing import Any

import typer

from dimos.cloud.data import CloudData, recordings


def handle_fail(fn: Callable[..., None]) -> Callable[..., None]:
    @functools.wraps(fn)
    def wrapper(*a: Any, **kw: Any) -> None:
        try:
            fn(*a, **kw)
        except (RuntimeError, OSError) as e:
            typer.echo(str(e), err=True)
            raise typer.Exit(1) from e

    return wrapper


@contextlib.contextmanager
def _bar(name: str) -> Iterator[Callable[[str, int, int], None]]:
    from rich.progress import Progress

    with Progress(transient=True) as bar:
        task = bar.add_task(name, total=None)

        def tick(phase: str, done: int, total: int) -> None:
            bar.update(task, description=f"{phase} {name}", completed=done, total=total or None)

        yield tick


@handle_fail
def upload(
    path: Path | None, robot: str | None, kind: str | None, since_s: float | None, chunk: int | None
) -> None:
    explicit = path is not None
    path = None if str(path) == "latest" else path
    cloud = CloudData()
    targets = recordings(since_s) if since_s else [path] if path else recordings()[-1:]
    if not targets:
        raise RuntimeError("nothing to upload — pass a path")
    failed = False
    for t in targets:
        try:
            with _bar(t.name) as tick:
                r = cloud.upload(
                    t,
                    robot_id=robot,
                    kind=kind,
                    chunk_mb=chunk,
                    progress=tick,
                    skip_recent=not explicit,
                )
            note = "already uploaded" if r["skipped"] else r["state"]
            typer.echo(f"{t.name}: {note} ({r['upload_id'][:12]})")
            if r["quota"].get("state") not in (None, "ok"):
                typer.echo(r["quota"]["message"], err=True)
        except (RuntimeError, OSError) as e:
            typer.echo(f"{t.name}: {e}", err=True)
            failed = True
    if failed:
        raise typer.Exit(1)


@handle_fail
def ls() -> None:
    from datetime import datetime, timezone
    import sys

    if sys.stdout.isatty() and sys.stdin.isatty():  # Textual needs a real TTY
        from dimos.cloud.tui import DataBrowser

        DataBrowser().run()
        return

    from rich import box
    from rich.console import Console
    from rich.filesize import decimal
    from rich.table import Table

    tz_now = datetime.now().astimezone().tzname()

    def local(ts: str) -> str:
        try:
            d = datetime.fromisoformat(ts)
        except ValueError:
            return ts[:16].replace("T", " ")
        if d.tzinfo is None:
            d = d.replace(tzinfo=timezone.utc)
        d = d.astimezone()
        # A row across a DST boundary carries its own label (PST vs the PDT header).
        suffix = "" if d.tzname() == tz_now else f" {d.tzname()}"
        return d.strftime("%Y-%m-%d %H:%M") + suffix

    rows = CloudData().ls()
    org = any(u.get("uploader_email") for u in rows)
    table = Table(box=box.SIMPLE_HEAVY, header_style="bold")
    table.add_column("id", style="cyan", no_wrap=True)
    table.add_column("file", style="bold")
    table.add_column(
        f"uploaded ({datetime.now().astimezone().tzname()})", style="dim", no_wrap=True
    )
    table.add_column("kind")
    if org:
        table.add_column("uploader", style="dim")
    table.add_column("blueprint", style="magenta")
    table.add_column("robot", style="magenta")
    table.add_column("topics", style="dim", max_width=48)
    table.add_column("size", justify="right")
    table.add_column("state")
    for u in rows:
        mani = u.get("manifest") or {}
        state = u["state"]
        table.add_row(
            u["id"][:12],
            u["filename"],
            local(str(u.get("created_at") or "")) or "—",
            u.get("kind", ""),
            *([u.get("uploader_email") or "—"] if org else []),
            mani.get("blueprint") or "—",
            u.get("robot_id") or "—",
            ", ".join(s.get("name", "?") for s in (mani.get("streams") or [])) or "—",
            decimal(u["size"]),
            f"[green]{state}[/]" if state == "complete" else f"[yellow]{state}[/]",
        )
    Console().print(table)


@handle_fail
def pull(upload_id: str | None, dest: Path | None) -> None:
    upload_id = None if upload_id == "latest" else upload_id
    typer.echo(f"pulled to {CloudData().pull(upload_id, dest)}")


@handle_fail
def status(upload_id: str) -> None:
    s = CloudData().status(upload_id)
    typer.echo(s["state"] + (f" — parts on server: {len(s['parts'])}" if s["parts"] else ""))


@handle_fail
def quota() -> None:
    from rich.filesize import decimal

    q = CloudData().quota()
    lim = q["limits"]
    typer.echo(
        f"{q['pct']}% used ({q['state']}) — total {decimal(q['used_total'])}"
        f" of {lim['total_gb']} GB, today {decimal(q['used_today'])}"
        f" of {lim['daily_gb']} GB"
    )
