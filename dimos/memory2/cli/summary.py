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

"""Print ``Store.summary()`` for a memory2 sqlite recording.

Usage:
    uv run dimos mem summary mid360
"""

from __future__ import annotations

from datetime import datetime, timezone
import json
from math import log
import sqlite3
from typing import TYPE_CHECKING, Any

import typer

from dimos.utils.colors import HEAT_GRADIENT_ANSI256
from dimos.utils.data import resolve_named_path
from dimos.utils.human import human_bytes

if TYPE_CHECKING:
    from pathlib import Path

    from dimos.memory2.stream import Stream

# Heavy dimos imports (memory2 store → codecs, msgs) and rich are deferred into
# the function bodies so that `dimos --help` — which imports this module just to
# register the `mem summary` command — stays fast. See test_cli_startup.py.


def stream_payload_types(db_path: Path) -> dict[str, type]:
    """Read each stream's registered payload type from the _streams table."""
    from dimos.memory2.codecs.base import resolve_payload_type

    conn = sqlite3.connect(str(db_path))
    try:
        rows = conn.execute("SELECT name, config FROM _streams").fetchall()
    finally:
        conn.close()
    return {name: resolve_payload_type(json.loads(cfg)["payload_module"]) for name, cfg in rows}


def _shade(value: float, lo: float, hi: float) -> str:
    """Rich style for ``value`` relative to [lo, hi], log-scaled (columns span decades)."""
    if value <= 0:
        return "dim"
    t = 0.5 if hi <= lo else (log(value) - log(lo)) / (log(hi) - log(lo))
    return f"color({HEAT_GRADIENT_ANSI256[round(t * (len(HEAT_GRADIENT_ANSI256) - 1))]})"


def _heat(text: str, value: float, column: list[float]) -> str:
    """Wrap ``text`` in rich markup colored by ``value``'s rank within ``column``."""
    positive = [v for v in column if v > 0]
    lo, hi = (min(positive), max(positive)) if positive else (0.0, 0.0)
    return f"[{_shade(value, lo, hi)}]{text}[/]"


def main(
    dataset: str = typer.Argument(..., help="Dataset .db: bare name (cwd or data/) or path"),
) -> None:
    """Print per-stream counts, time ranges, and payload sizes for a recorded SQLite dataset."""
    from rich.console import Console
    from rich.progress import Progress
    from rich.table import Table

    from dimos.memory2.store.sqlite import SqliteStore

    db_path = resolve_named_path(dataset, ".db")
    payload_types = stream_payload_types(db_path)

    rows: list[tuple[str, int, float | None, float | None, int]] = []
    store = SqliteStore(path=str(db_path))
    with store, Progress(transient=True) as prog:
        task = prog.add_task("scanning", total=len(payload_types))
        for name, ptype in payload_types.items():
            prog.update(task, description=name)
            stream: Stream[Any] = store.stream(name, ptype)
            n = stream.count()
            t0, t1 = stream.get_time_range() if n else (None, None)
            rows.append((name, n, t0, t1, stream.size_bytes() or 0))
            prog.advance(task)
    rows.sort(key=lambda r: r[4], reverse=True)

    table = Table(title=db_path.name)
    table.add_column("Stream", style="cyan")
    table.add_column("Items", justify="right")
    table.add_column("Hz", justify="right")
    table.add_column("Start (UTC)")
    table.add_column("Duration", justify="right")
    table.add_column("Size", justify="right")

    def hz(n: int, t0: float | None, t1: float | None) -> float:
        return (n - 1) / (t1 - t0) if t0 is not None and t1 is not None and t1 > t0 else 0.0

    items_col = [float(r[1]) for r in rows]
    hz_col = [hz(r[1], r[2], r[3]) for r in rows]
    size_col = [float(r[4]) for r in rows]

    for name, n, t0, t1, size in rows:
        dur = t1 - t0 if t0 is not None and t1 is not None else None
        rate = hz(n, t0, t1)
        table.add_row(
            name,
            _heat(f"{n:,}", n, items_col),
            _heat(f"{rate:.1f}", rate, hz_col) if rate > 0 else "—",
            datetime.fromtimestamp(t0, tz=timezone.utc).strftime("%Y-%m-%d %H:%M:%S")
            if t0 is not None
            else "—",
            f"{dur:.1f}s" if dur is not None else "—",
            _heat(human_bytes(size), size, size_col),
        )
    table.add_section()
    table.add_row(
        "total", f"{sum(r[1] for r in rows):,}", "", "", "", human_bytes(sum(r[4] for r in rows))
    )

    console = Console()
    if not console.is_terminal:  # piped: don't squeeze the table into the 80-col default
        console = Console(width=250)
    console.print(table)


if __name__ == "__main__":
    typer.run(main)
