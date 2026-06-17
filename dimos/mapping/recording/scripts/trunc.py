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

"""Truncate a recording to start at its first fastlio_odometry message.

Everything timestamped before the first `fastlio_odometry` sample is dropped:
  - .db: deleted from every stream (the data, `_blob` and `_rtree` tables),
  - .rrd: rebuilt from the truncated db, so it starts exactly at t0 (rebuilding
    avoids the timeline-boundary float imprecision of `rerun rrd split`, and the
    aggregated maps/paths only reflect the post-t0 data).

The db edit is in place; deleting rows doesn't shrink the file (SQLite reuses
free pages) but the dropped prefix is tiny.

    uv run python dimos/mapping/recording/scripts/trunc.py REC/mem2.db [REC/main.rrd]
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sqlite3

from dimos.mapping.recording.utils import stream_names

REALSENSE_INFO_STREAM = "realsense_camera_info"


def first_fastlio_ts(db_path: str) -> float | None:
    """Timestamp of the first `fastlio_odometry` message, or None if absent."""
    conn = sqlite3.connect(db_path)
    try:
        row = conn.execute(f'SELECT MIN(ts) FROM "{stream_names.FASTLIO_ODOM}"').fetchone()
    except sqlite3.OperationalError:
        return None
    finally:
        conn.close()
    return float(row[0]) if row and row[0] is not None else None


def truncate_db(db_path: str, t0: float) -> dict[str, int]:
    """Delete every row with ts < t0 from each stream (data + blob + rtree).
    Returns {stream: rows_removed}. Transactional."""
    conn = sqlite3.connect(db_path)
    try:
        streams = [row[0] for row in conn.execute("SELECT name FROM _streams")]
        tables = {
            row[0] for row in conn.execute("SELECT name FROM sqlite_master WHERE type='table'")
        }
        removed: dict[str, int] = {}
        conn.execute("BEGIN")
        for name in streams:
            if name not in tables:
                continue
            count = conn.execute(f'SELECT COUNT(*) FROM "{name}" WHERE ts < ?', (t0,)).fetchone()[0]
            if not count:
                continue
            ids_before = f'SELECT id FROM "{name}" WHERE ts < ?'
            if f"{name}_blob" in tables:
                conn.execute(f'DELETE FROM "{name}_blob" WHERE id IN ({ids_before})', (t0,))
            if f"{name}_rtree" in tables:
                conn.execute(f'DELETE FROM "{name}_rtree" WHERE id IN ({ids_before})', (t0,))
            conn.execute(f'DELETE FROM "{name}" WHERE ts < ?', (t0,))
            removed[name] = count
        conn.execute("COMMIT")
        return removed
    except Exception:
        conn.execute("ROLLBACK")
        raise
    finally:
        conn.close()


def rebuild_rrd(db_path: str, rrd_path: Path) -> None:
    """Rebuild the .rrd from the (truncated) db, picking camera params by rig."""
    # imported lazily: pulls in rerun + the rig camera configs only when needed.
    from dimos.mapping.recording.go2_mid360.post_process import load_camera as load_go2_camera
    from dimos.mapping.recording.mid360_realsense.post_process import (
        load_camera as load_realsense_camera,
    )
    from dimos.mapping.recording.utils.build_rrd import build_rrd

    conn = sqlite3.connect(db_path)
    streams = {row[0] for row in conn.execute("SELECT name FROM _streams")}
    conn.close()
    load_camera = load_realsense_camera if REALSENSE_INFO_STREAM in streams else load_go2_camera
    intrinsics, _distortion, optical_in_base, resolution = load_camera(Path(db_path))
    build_rrd(db_path, str(rrd_path), intrinsics, optical_in_base, resolution)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("db", help="recording mem2.db")
    parser.add_argument("rrd", nargs="?", help="output .rrd (default: main.rrd next to the db)")
    args = parser.parse_args()

    db_path = Path(args.db)
    if not db_path.exists():
        raise SystemExit(f"no such db: {db_path}")
    rrd_path = Path(args.rrd) if args.rrd else db_path.parent / "main.rrd"

    t0 = first_fastlio_ts(str(db_path))
    if t0 is None:
        raise SystemExit(
            f"no '{stream_names.FASTLIO_ODOM}' messages in {db_path} — nothing to truncate"
        )
    print(f">> truncating before t0={t0:.6f} (first {stream_names.FASTLIO_ODOM})")

    removed = truncate_db(str(db_path), t0)
    print(f"   db: removed {sum(removed.values())} rows from {len(removed)} streams")
    for name, count in sorted(removed.items()):
        print(f"      {name}: {count}")

    print(f"   rrd: rebuilding from truncated db -> {rrd_path}")
    rebuild_rrd(str(db_path), rrd_path)
    print("done")


if __name__ == "__main__":
    main()
