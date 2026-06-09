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

"""Copy a memory2 sqlite recording into a new file, renaming streams.

Iterates each source stream and re-appends every observation under its new
name in a fresh destination db. Slower than in-place ``ALTER TABLE`` but
forces a full re-read of every row, so any pre-existing corruption surfaces
immediately. Streams not mentioned in ``--rename`` are copied verbatim.
"""

from __future__ import annotations

from collections.abc import Callable
import json
from pathlib import Path
import sqlite3
import time
from typing import Any

import typer

from dimos.memory2.codecs.base import _resolve_payload_type
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.stream import Stream
from dimos.memory2.type.observation import Observation
from dimos.utils.data import resolve_named_path


def progress(total: int, label: str = "") -> Callable[[Observation[Any]], None]:
    """Matches dimos/mapping/utils/cli/map.py:progress — kept inline to avoid pulling rerun."""
    seen = 0
    wall_start: float | None = None
    last_wall: float | None = None
    first_ts: float | None = None

    def _progress(obs: Observation[Any]) -> None:
        nonlocal seen, wall_start, last_wall, first_ts
        now = time.monotonic()
        if wall_start is None:
            wall_start = now
            first_ts = obs.ts
        assert first_ts is not None
        frame_ms = (now - last_wall) * 1000 if last_wall is not None else 0.0
        last_wall = now
        seen += 1
        pct = 100 * seen // total if total else 100
        wall = now - wall_start
        data = obs.ts - first_ts
        speed = data / wall if wall > 0 else 0.0
        end = "\n" if seen >= total else ""
        prefix = f"{label} " if label else ""
        print(
            f"\r{prefix}{pct:>3}% [{seen}/{total}] {data:.1f}s ({speed:.1f} x rt) {frame_ms:.0f}ms/frame",
            end=end,
            flush=True,
        )

    return _progress


def _stream_payload_types(db_path: Path) -> dict[str, type]:
    """Read each stream's registered payload type from the _streams table."""
    conn = sqlite3.connect(str(db_path))
    try:
        rows = conn.execute("SELECT name, config FROM _streams").fetchall()
    finally:
        conn.close()
    return {name: _resolve_payload_type(json.loads(cfg)["payload_module"]) for name, cfg in rows}


def _parse_renames(pairs: list[str]) -> dict[str, str]:
    """Parse ``OLD=NEW`` pairs into a dict; reject malformed or duplicate OLDs."""
    out: dict[str, str] = {}
    for raw in pairs:
        if "=" not in raw:
            raise typer.BadParameter(f"--rename must be OLD=NEW, got {raw!r}")
        old, new = raw.split("=", 1)
        old, new = old.strip(), new.strip()
        if not old or not new:
            raise typer.BadParameter(f"--rename has empty side in {raw!r}")
        if old in out:
            raise typer.BadParameter(f"--rename {old!r} specified more than once")
        out[old] = new
    return out


def main(
    dataset: str = typer.Argument(..., help="Source .db: bare name (cwd or data/) or path"),
    out: Path = typer.Option(..., "--out", help="Output renamed .db path (must not exist)"),
    rename: list[str] = typer.Option(
        [],
        "--rename",
        help="OLD=NEW rename pair (can be passed multiple times); unmapped streams pass through",
    ),
    drop: list[str] = typer.Option(
        [], "--drop", help="Stream name to omit from output (can be passed multiple times)"
    ),
    seek: float = typer.Option(0.0, "--seek", help="Skip the first N seconds of the recording"),
    duration: float | None = typer.Option(
        None, "--duration", help="Use only N seconds from --seek (default: to the end)"
    ),
) -> None:
    """Copy a recording into a new .db, renaming and/or dropping streams."""
    src_path = resolve_named_path(dataset, ".db")
    if out.exists():
        raise typer.BadParameter(f"Output already exists: {out}")

    print(f"analizing dataset {src_path}")
    rename_map = _parse_renames(rename)
    payload_types = _stream_payload_types(src_path)

    drop_set = set(drop)
    missing = sorted((set(rename_map) | drop_set) - set(payload_types))
    if missing:
        raise typer.BadParameter(
            f"--rename / --drop refers to streams not in source: {missing}. "
            f"Available: {sorted(payload_types)}"
        )
    overlap = drop_set & set(rename_map)
    if overlap:
        raise typer.BadParameter(f"stream(s) in both --rename and --drop: {sorted(overlap)}")

    kept = {name: rename_map.get(name, name) for name in payload_types if name not in drop_set}
    seen: dict[str, str] = {}
    for src_name, dst_name in kept.items():
        if dst_name in seen:
            raise typer.BadParameter(
                f"name collision: {seen[dst_name]!r} and {src_name!r} both map to {dst_name!r}"
            )
        seen[dst_name] = src_name

    print("rename plan:")
    for src_name in payload_types:
        if src_name in drop_set:
            print(f"  {src_name:>16s} ✗ (dropped)")
        else:
            dst_name = kept[src_name]
            arrow = "→" if src_name != dst_name else " "
            print(f"  {src_name:>16s} {arrow} {dst_name}")
    print()

    src = SqliteStore(path=str(src_path))
    with src:
        dst = SqliteStore(path=str(out))
        with dst:
            for src_name, dst_name in kept.items():
                ptype = payload_types[src_name]
                src_s: Stream[Any] = (
                    src.stream(src_name, ptype).from_time(seek or None).to_time(duration)
                )
                dst_s: Stream[Any] = dst.stream(dst_name, ptype)
                total = src_s.count()
                cb = progress(total, f"{dst_name:>16s}")
                for obs in src_s:
                    dst_s.append(obs.data, ts=obs.ts, pose=obs.pose, tags=obs.tags or None)
                    cb(obs)

    print(f"\nwrote {out}")


if __name__ == "__main__":
    typer.run(main)
