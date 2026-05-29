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

"""Trim the static lead-in from a memory2 sqlite recording.

Uses one stream's poses to find the first moment the robot displaces from
its starting pose by at least ``--tolerance``, then writes a trimmed copy of
every stream in the source, starting ``--lead-in`` seconds before that
motion. With ``--main-stream`` omitted, scans every stream that has poses
and prompts you to pick.

Usage:
    uv run python -m dimos.mapping.loop_closure.utils.autotrim mid360 \\
        --out mid360_trimmed.db --tolerance 0.20
"""

from __future__ import annotations

from collections.abc import Callable
import json
import math
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
    """Matches dimos/utils/cli/map.py:progress — kept inline to avoid pulling rerun."""
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


def _find_motion_edge_ts(
    stream: Stream[Any], tolerance: float, *, reverse: bool = False
) -> tuple[float, tuple[float, float, float]]:
    """Walks a stream in ts order (or descending if reverse=True) and returns the
    first ts whose pose is ≥ tolerance from the first pose seen in that walk.

    Forward: motion *start* — anchor = initial pose, first ts that moves away.
    Reverse: motion *stop*  — anchor = final  pose, last  ts that's still away.
    """
    walk = stream.order_by("ts", desc=True) if reverse else stream
    anchor: tuple[float, float, float] | None = None
    for obs in walk:
        if obs.pose is None:
            continue
        x, y, z = obs.pose[0], obs.pose[1], obs.pose[2]
        if anchor is None:
            anchor = (x, y, z)
            continue
        if math.dist((x, y, z), anchor) >= tolerance:
            return obs.ts, anchor
    raise RuntimeError(
        f"No pose ever displaced ≥ {tolerance:.3f} m from {'final' if reverse else 'initial'} pose"
        + (" (no poses in stream)" if anchor is None else "")
    )


def _stream_payload_types(db_path: Path) -> dict[str, type]:
    """Read each stream's registered payload type from the _streams table."""
    conn = sqlite3.connect(str(db_path))
    try:
        rows = conn.execute("SELECT name, config FROM _streams").fetchall()
    finally:
        conn.close()
    return {name: _resolve_payload_type(json.loads(cfg)["payload_module"]) for name, cfg in rows}


def _pick_main_stream_interactive(
    src: SqliteStore, payload_types: dict[str, type], tolerance: float
) -> str:
    """Scan every stream's poses, print first/last motion times, prompt for a pick."""
    print(f"scanning streams for motion (≥{tolerance:.2f} m)...")
    candidates: list[str] = []
    for name, ptype in payload_types.items():
        s = src.stream(name, ptype)
        try:
            start_ts, _ = _find_motion_edge_ts(s, tolerance)
            stop_ts, _ = _find_motion_edge_ts(s, tolerance, reverse=True)
        except RuntimeError:
            print(f"  {name:>12s}: no motion detected")
            continue
        t0, tN = s.first().ts, s.last().ts
        candidates.append(name)
        print(
            f"  {name:>12s}: motion +{start_ts - t0:6.2f} s → +{stop_ts - t0:6.2f} s"
            f"  (tail idle {tN - stop_ts:5.2f} s)"
        )

    if not candidates:
        raise typer.BadParameter(
            f"No stream has poses with ≥{tolerance:.2f} m motion. Lower --tolerance."
        )

    print("\npick main stream (number or name):")
    for i, name in enumerate(candidates, 1):
        print(f"  {i}) {name}")
    by_idx = {str(i): name for i, name in enumerate(candidates, 1)}
    by_name = set(candidates)
    while True:
        choice = typer.prompt("choice", default="1").strip()
        if choice in by_idx:
            return by_idx[choice]
        if choice in by_name:
            return choice
        print(f"invalid choice {choice!r}; try a number 1..{len(candidates)} or a stream name")


def main(
    dataset: str = typer.Argument(..., help="Source .db: bare name (cwd or data/) or path"),
    out: Path = typer.Option(..., "--out", help="Output trimmed .db path"),
    main_stream: str | None = typer.Option(
        None,
        "--main-stream",
        help="Stream whose poses define motion; omit to pick interactively",
    ),
    tolerance: float = typer.Option(
        0.20, "--tolerance", help="Distance (m) the main stream must travel before 'motion' starts"
    ),
    lead_in: float = typer.Option(
        1.0, "--lead-in", help="Seconds to keep before the first-motion timestamp"
    ),
) -> None:
    src_path = resolve_named_path(dataset, ".db")
    if out.exists():
        raise typer.BadParameter(f"Output already exists: {out}")

    print(f"analizing dataset {src_path}")
    payload_types = _stream_payload_types(src_path)
    if main_stream is not None and main_stream not in payload_types:
        raise typer.BadParameter(
            f"Main stream {main_stream!r} not in source db. Available: {sorted(payload_types)}"
        )

    src = SqliteStore(path=str(src_path))
    with src:
        if main_stream is None:
            main_stream = _pick_main_stream_interactive(src, payload_types, tolerance)

        motion_ts, origin = _find_motion_edge_ts(
            src.stream(main_stream, payload_types[main_stream]), tolerance
        )
        cutoff = motion_ts - lead_in
        ox, oy, oz = origin
        print(f"\nmain stream {main_stream!r}: origin=({ox:.3f},{oy:.3f},{oz:.3f})")
        print(f"  motion start (≥{tolerance:.2f} m): ts={motion_ts:.3f}")
        print(f"  cutoff (lead-in {lead_in:.1f} s):    ts={cutoff:.3f}\n")

        dst = SqliteStore(path=str(out))
        with dst:
            for name, ptype in payload_types.items():
                src_s = src.stream(name, ptype)
                dst_s = dst.stream(name, ptype)
                total_src = src_s.count()
                filtered = src_s.after(cutoff)
                total_kept = filtered.count()
                dropped = total_src - total_kept
                cb = progress(total_kept, f"{name:>12s}")
                for obs in src_s.after(cutoff):
                    dst_s.append(obs.data, ts=obs.ts, pose=obs.pose, tags=obs.tags or None)
                    cb(obs)
                if total_kept == 0:
                    print(f"{name:>12s} kept 0/{total_src} (dropped {dropped})")
                else:
                    print(f"             ↳ dropped {dropped}/{total_src}")

    print(f"\nwrote {out}")


if __name__ == "__main__":
    typer.run(main)
