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

"""``dimos pure`` CLI: a thin shell over the T15 debug reader (spec §C10)."""

from __future__ import annotations

from typing import Optional

import typer

pure_app = typer.Typer(help="Pure-module tools", no_args_is_help=True)


@pure_app.command("debug")
def debug(
    run_dir: str = typer.Argument(..., help="run dir (containing debug.db) or a debug.db path"),
    module: Optional[str] = typer.Option(  # noqa: UP045 — typer needs Optional at runtime
        None, "-m", "--module", help="one module path (dot-separated T13 member path)"
    ),
    drops: bool = typer.Option(False, "--drops", help="list drops by reason for -m <module>"),
    tick: Optional[int] = typer.Option(  # noqa: UP045
        None, "--tick", help="dump one decision (+ row if captured) for -m <module>"
    ),
) -> None:
    """Aggregate summary, per-module drops, or a single tick — a thin reader shell."""
    from dimos.pure import debugrec

    run = debugrec.load(run_dir)
    try:
        if module is None:
            typer.echo(run.summary())
            return
        md = run.module(module.replace("/", "."))
        if tick is not None:
            hit = next((t for t in md.ticks() if t.seq == tick), None)
            if hit is None:
                raise typer.BadParameter(f"no tick seq={tick} for module {module!r}")
            typer.echo(str(hit))
            for record in md.in_rows():
                if record.seq == tick:
                    typer.echo(f"  in : {record.row!r}")
            for record in md.out_rows():
                if record.seq == tick:
                    typer.echo(f"  out: {record.row!r}")
            return
        if drops:
            counts: dict[str, int] = {}
            for t in md.drops():
                reason = t.drop_reason or "unknown"
                counts[reason] = counts.get(reason, 0) + 1
            for reason, count in sorted(counts.items(), key=lambda kv: -kv[1]):
                typer.echo(f"{reason}: {count}")
            return
        typer.echo(md.summary())
    finally:
        run.close()
