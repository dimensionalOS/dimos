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

"""Make short.db + short.rrd: the first N seconds of a recording.

Given a recording dir, copies only the rows within [first_timestamp,
first_timestamp + seconds] into a fresh `short.db` (schema-faithful: data,
`_blob` and `_rtree` tables, so poses/tags survive), then rebuilds `short.rrd`
from it. Copies only the in-window rows (never the whole db), so it stays small
and works even when the source is huge / the disk is nearly full.

    uv run python dimos/mapping/recording/utils/short.py REC_DIR [--seconds 30]
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sqlite3

from dimos.mapping.recording.utils import stream_names
from dimos.mapping.recording.utils.trunc import first_fastlio_ts, rebuild_rrd

DB_NAME = "mem2_orig.db"
SHORT_DB = "short.db"
SHORT_RRD = "short.rrd"
DEFAULT_SECONDS = 30.0
# rtree shadow tables are created automatically by the virtual table, not by us.
_RTREE_SHADOWS = ("_rtree_node", "_rtree_parent", "_rtree_rowid")


def make_short_db(src_db: str, short_db: str, seconds: float) -> tuple[float, float, int]:
    """Build `short_db` from the first `seconds` *after the first fastlio_odometry*
    of `src_db` (truncates the pre-fastlio prefix). Returns (t_start, cutoff, rows)."""
    for suffix in ("", "-wal", "-shm"):
        stale = Path(short_db + suffix)
        if stale.exists():
            stale.unlink()

    t_start = first_fastlio_ts(src_db)
    if t_start is None:
        raise SystemExit(f"no '{stream_names.FASTLIO_ODOM}' in source db")
    cutoff = t_start + seconds

    conn = sqlite3.connect(short_db)
    try:
        conn.execute("ATTACH DATABASE ? AS src", (src_db,))
        # Recreate every schema object verbatim (skip rtree shadow tables — the
        # `CREATE VIRTUAL TABLE ... rtree` recreates those itself).
        for _type, name, sql in conn.execute(
            "SELECT type, name, sql FROM src.sqlite_master "
            "WHERE sql IS NOT NULL AND name NOT LIKE 'sqlite_%'"
        ).fetchall():
            if name.endswith(_RTREE_SHADOWS):
                continue
            conn.execute(sql)

        tables = {
            row[0] for row in conn.execute("SELECT name FROM src.sqlite_master WHERE type='table'")
        }
        conn.execute("INSERT INTO _streams SELECT * FROM src._streams")
        rows_copied = 0
        window = (t_start, cutoff)
        for (name,) in conn.execute("SELECT name FROM src._streams").fetchall():
            in_window_ids = f'SELECT id FROM src."{name}" WHERE ts >= ? AND ts <= ?'
            cursor = conn.execute(
                f'INSERT INTO "{name}" SELECT * FROM src."{name}" WHERE ts >= ? AND ts <= ?', window
            )
            rows_copied += cursor.rowcount
            if f"{name}_blob" in tables:
                conn.execute(
                    f'INSERT INTO "{name}_blob" SELECT * FROM src."{name}_blob" '
                    f"WHERE id IN ({in_window_ids})",
                    window,
                )
            if f"{name}_rtree" in tables:
                conn.execute(
                    f'INSERT INTO "{name}_rtree" SELECT * FROM src."{name}_rtree" '
                    f"WHERE id IN ({in_window_ids})",
                    window,
                )
        conn.commit()
        return t_start, cutoff, rows_copied
    finally:
        conn.close()


def rename_go2_streams(db_path: str) -> list[tuple[str, str]]:
    """Rename legacy go2_odom/go2_lidar -> odom/lidar in `db_path` (data, _blob and
    _rtree tables + the _streams row), so reframe_go2 finds them. No-op if absent or
    already renamed. ALTER TABLE is metadata-only; the rtree renames its shadows."""
    renames = [("go2_odom", "odom"), ("go2_lidar", "lidar")]
    conn = sqlite3.connect(db_path)
    try:
        streams = {row[0] for row in conn.execute("SELECT name FROM _streams")}
        tables = {
            row[0] for row in conn.execute("SELECT name FROM sqlite_master WHERE type='table'")
        }
        done: list[tuple[str, str]] = []
        conn.execute("BEGIN")
        for old, new in renames:
            if old not in streams or new in streams:
                continue
            conn.execute(f'ALTER TABLE "{old}" RENAME TO "{new}"')
            if f"{old}_blob" in tables:
                conn.execute(f'ALTER TABLE "{old}_blob" RENAME TO "{new}_blob"')
            if f"{old}_rtree" in tables:
                conn.execute(f'ALTER TABLE "{old}_rtree" RENAME TO "{new}_rtree"')
            conn.execute("UPDATE _streams SET name=? WHERE name=?", (new, old))
            done.append((old, new))
        conn.execute("COMMIT")
        return done
    except Exception:
        conn.execute("ROLLBACK")
        raise
    finally:
        conn.close()


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("recording", help="recording dir (or its mem2.db)")
    parser.add_argument(
        "--seconds", type=float, default=DEFAULT_SECONDS, help="window length (default: 30)"
    )
    args = parser.parse_args()

    target = Path(args.recording)
    src_db = target if target.name == DB_NAME else target / DB_NAME
    if not src_db.exists():
        raise SystemExit(f"no {DB_NAME} at {target}")
    recording_dir = src_db.parent
    short_db = recording_dir / SHORT_DB
    short_rrd = recording_dir / SHORT_RRD

    print(f">> first {args.seconds:g}s after first {stream_names.FASTLIO_ODOM} of {src_db}")
    *_, rows = make_short_db(str(src_db), str(short_db), args.seconds)
    renamed = rename_go2_streams(str(short_db))
    if renamed:
        print(f"   renamed {', '.join(f'{old}->{new}' for old, new in renamed)}")
    span = (
        sqlite3.connect(str(short_db))
        .execute(f'SELECT MAX(ts) - MIN(ts) FROM "{stream_names.FASTLIO_ODOM}"')
        .fetchone()[0]
    )
    print(f"   db: {short_db.name} ({rows} rows, ~{span:.1f}s of fastlio span)")
    print(f"   rrd: building -> {short_rrd.name}")
    rebuild_rrd(str(short_db), short_rrd)
    print("done")


if __name__ == "__main__":
    main()
