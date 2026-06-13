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

"""Run FAST-LIO over a .pcap and write its outputs into a .db.

Given a Livox Mid-360 pcap capture, this streams the pcap through the FastLio2
native module and writes two streams into a memory2 SQLite database:

* ``fastlio_odometry`` -- the IESKF pose at the native odom rate (~30 Hz).
* ``fastlio_lidar`` -- the registered (deskewed, odom-frame) point cloud at the
  native pointcloud rate (~10 Hz).

The ``--db`` is optional. With no existing db the tool builds one **from
scratch** (omit ``--db`` and it defaults to ``<pcap>.db`` next to the pcap).
With an existing db the two streams are appended and time-aligned onto the db's
clock, so FAST-LIO output can be compared against whatever it already holds.

This mirrors the Point-LIO ``pcap_to_db.py`` tool, with one deliberate
difference: FAST-LIO is *not* bit-deterministic (OpenMP reduction order), so the
replay runs ``deterministic_clock=False`` -- the feeder paces packets at
wall-clock realtime, exactly as the live SDK delivers them, and publish
timestamps come from the pcap's capture clock. A 20-minute recording therefore
takes ~20 minutes of wall time to replay.

If either stream already exists in the db the run aborts, unless ``--force`` is
given, in which case the existing ``fastlio_odometry`` and ``fastlio_lidar``
streams are dropped before the new ones are written.

Timing conversion
-----------------
With ``deterministic_clock=False`` FAST-LIO publishes with the pcap packet
clock, which for a real recording is the original capture's *unix wall time* --
the same clock the db's other streams already use. So the common case needs no
shift. The offset is auto-derived from the two clocks:

* db + replay on the same clock family (both wall, or both sensor): offset 0.
* cross-clock (e.g. a deterministic sensor-clock replay into a wall-clock db):
  start-align by shifting the replay's first ts onto the db's earliest ts.
* db has no existing timestamped rows: offset 0.

Pass ``--time-offset`` to override the auto choice.

Usage (from the dimos5 venv)::

    source .venv/bin/activate

    # Build a fresh db from scratch (no existing db needed); defaults to
    # <pcap>.db next to the pcap.
    python -m dimos.hardware.sensors.lidar.fastlio2.tools.pcap_to_db \
        --pcap /path/to/capture.pcap

    # Or append into an existing recording db for comparison.
    python -m dimos.hardware.sensors.lidar.fastlio2.tools.pcap_to_db \
        --pcap /path/to/capture.pcap --db /path/to/memory.db
"""

from __future__ import annotations

import argparse
from collections.abc import AsyncIterator
import math
from pathlib import Path
import sqlite3
import sys
import time

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

# Below this an absolute timestamp is sensor-boot seconds, not unix wall time.
_SENSOR_CLOCK_MAX = 1e8
# Strictly-increasing tie-breaker so two samples never collide on ts.
_EPS = 1e-9
# Poll the db on this cadence while the replay drains the pcap.
_POLL_SEC = 1.0
# Stop after the odom stream has been stagnant this long (pcap fully drained).
_STAGNANT_SEC = 6.0


class _RecConfig(ModuleConfig):
    """Configures the recorder with the target db and timing conversion."""

    db_path: str = ""
    # Earliest existing ts in the db, or -1.0 if the db has no timestamped rows.
    ref_start_ts: float = -1.0
    # Explicit offset override; NaN means auto-derive from ref_start_ts.
    time_offset: float = float("nan")


class _Rec(Module):
    """Append FAST-LIO odometry + lidar into an existing SQLite db with ts conversion."""

    config: _RecConfig
    fastlio_odometry: In[Odometry]
    fastlio_lidar: In[PointCloud2]
    _offset: float | None = None
    _last_odom_ts: float = 0.0
    _last_lidar_ts: float = 0.0
    _last_pose: object = None
    _odom_count: int = 0
    _lidar_count: int = 0

    async def main(self) -> AsyncIterator[None]:
        from dimos.memory2.store.sqlite import SqliteStore

        self._store = SqliteStore(path=self.config.db_path)
        self._os = self._store.stream("fastlio_odometry", Odometry)
        self._ls = self._store.stream("fastlio_lidar", PointCloud2)
        yield
        self._store.stop()

    def _resolve_offset(self, first_ts: float) -> float:
        override = self.config.time_offset
        if not math.isnan(override):
            return override
        ref = self.config.ref_start_ts
        if ref < 0.0:
            return 0.0
        # Same clock family (both wall, or both sensor) -> already aligned.
        # Cross-clock -> start-align the replay's first ts onto the db's first.
        if (first_ts > _SENSOR_CLOCK_MAX) == (ref > _SENSOR_CLOCK_MAX):
            return 0.0
        return ref - first_ts

    def _aligned_ts(self, raw_ts: float, last_ts: float) -> float:
        """Convert a replay ts onto the db clock, kept strictly above last_ts."""
        if self._offset is None:
            self._offset = self._resolve_offset(raw_ts)
        return max(raw_ts + self._offset, last_ts + _EPS)

    async def handle_fastlio_odometry(self, v: Odometry) -> None:
        raw_ts = getattr(v, "ts", None) or time.time()
        ts = self._aligned_ts(raw_ts, self._last_odom_ts)
        self._last_odom_ts = ts
        pose = getattr(v, "pose", None)
        self._last_pose = getattr(pose, "pose", None) if pose is not None else None
        self._os.append(v, ts=ts, pose=self._last_pose)
        self._odom_count += 1

    async def handle_fastlio_lidar(self, v: PointCloud2) -> None:
        raw_ts = getattr(v, "ts", None) or time.time()
        ts = self._aligned_ts(raw_ts, self._last_lidar_ts)
        self._last_lidar_ts = ts
        self._ls.append(v, ts=ts, pose=self._last_pose)
        self._lidar_count += 1


def _db_ref_start_ts(db_path: Path) -> float:
    """Min ts across the db's existing streams, or -1.0 if none/absent."""
    if not db_path.exists():
        return -1.0
    con = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True, timeout=2.0)
    try:
        tables = [
            row[0]
            for row in con.execute("SELECT name FROM sqlite_master WHERE type='table'").fetchall()
        ]
        best: float | None = None
        for table in tables:
            if table.startswith("_") or table.startswith("sqlite_"):
                continue
            try:
                # vec0/rtree virtual tables (sqlite-vec etc.) raise "no such
                # module" here when the extension isn't loaded -- skip them.
                cols = [c[1] for c in con.execute(f"PRAGMA table_info('{table}')").fetchall()]
                if "ts" not in cols:
                    continue
                row = con.execute(f"SELECT MIN(ts) FROM '{table}'").fetchone()
            except sqlite3.OperationalError:
                continue
            if row and row[0] is not None:
                best = row[0] if best is None else min(best, row[0])
        return best if best is not None else -1.0
    finally:
        con.close()


def _table_stats(db_path: Path, table: str) -> tuple[int, float, float]:
    """(count, min_ts, max_ts) for a stream table; zeros if absent."""
    if not db_path.exists():
        return 0, 0.0, 0.0
    con = sqlite3.connect(f"file:{db_path}?mode=ro", uri=True, timeout=2.0)
    try:
        try:
            row = con.execute(f"SELECT COUNT(*), MIN(ts), MAX(ts) FROM '{table}'").fetchone()
        except sqlite3.OperationalError:
            return 0, 0.0, 0.0
        cnt = row[0] or 0
        return cnt, row[1] or 0.0, row[2] or 0.0
    finally:
        con.close()


def _run(args: argparse.Namespace) -> int:
    pcap_path = Path(args.pcap).expanduser().resolve()
    if not pcap_path.exists():
        print(f"[pcap_to_db] missing pcap: {pcap_path}", file=sys.stderr)
        return 2
    if args.max_sensor_sec < 0:
        print("[pcap_to_db] --max-sensor-sec must be >= 0", file=sys.stderr)
        return 2
    # --db is optional: with no existing db, build one from scratch. When
    # omitted the output defaults to <pcap>.db next to the pcap, so a fresh
    # db can be generated with just --pcap.
    db_path = Path(args.db).expanduser().resolve() if args.db else pcap_path.with_suffix(".db")
    db_existed = db_path.exists()
    db_path.parent.mkdir(parents=True, exist_ok=True)

    from dimos.core.coordination.blueprints import autoconnect
    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
    from dimos.memory2.store.sqlite import SqliteStore

    fastlio_streams = ("fastlio_odometry", "fastlio_lidar")
    store = SqliteStore(path=str(db_path))
    try:
        existing = sorted(set(store.list_streams()) & set(fastlio_streams))
        if existing and not args.force:
            print(
                f"[pcap_to_db] {db_path.name} already has {existing}; pass --force to overwrite",
                file=sys.stderr,
            )
            return 2
        for name in existing:
            store.delete_stream(name)
        if existing:
            print(f"[pcap_to_db] --force: dropped existing {existing}", flush=True)
    finally:
        store.stop()

    ref_start_ts = _db_ref_start_ts(db_path)
    time_offset = float("nan") if args.time_offset is None else args.time_offset
    if not math.isnan(time_offset):
        offset_desc = f"explicit {time_offset:+.3f}s"
    elif ref_start_ts < 0.0:
        offset_desc = "auto: db empty -> 0"
    elif ref_start_ts < _SENSOR_CLOCK_MAX:
        offset_desc = f"auto: db sensor-clock (R0={ref_start_ts:.2f})"
    else:
        offset_desc = f"auto: db wall-clock (R0={ref_start_ts:.2f})"
    print(
        f"[pcap_to_db] pcap={pcap_path.name} db={db_path.name} "
        f"({'append' if db_existed else 'new'}) "
        f"odom_freq={args.odom_freq}Hz offset={offset_desc}",
        flush=True,
    )

    fastlio_kwargs: dict[str, object] = dict(
        frame_id="world",
        map_freq=-1,
        odom_freq=args.odom_freq,
        replay_pcap=pcap_path,
        deterministic_clock=False,
        debug=False,
    )
    # Omit config to fall back to the module default (config/mid360.yaml).
    if args.config:
        fastlio_kwargs["config"] = Path(args.config)
    fastlio = FastLio2.blueprint(**fastlio_kwargs).remappings(
        [
            (FastLio2, "odometry", "fastlio_odometry"),
            (FastLio2, "lidar", "fastlio_lidar"),
        ]
    )
    blueprint = autoconnect(
        fastlio,
        _Rec.blueprint(
            db_path=str(db_path),
            ref_start_ts=ref_start_ts,
            time_offset=time_offset,
        ),
    ).global_config(n_workers=4, robot_model="mid360_fastlio2_pcap_to_db")
    coord = ModuleCoordinator.build(blueprint)

    t0 = time.time()
    last_max = 0.0
    first_max: float | None = None
    stagnant_since: float | None = None
    try:
        while True:
            time.sleep(_POLL_SEC)
            cnt, min_ts, max_ts = _table_stats(db_path, "fastlio_odometry")
            if cnt == 0:
                continue
            if first_max is None:
                first_max = min_ts
            if args.max_sensor_sec > 0 and (max_ts - first_max) >= args.max_sensor_sec:
                print(
                    f"[pcap_to_db] reached --max-sensor-sec={args.max_sensor_sec:.1f}s",
                    flush=True,
                )
                break
            if max_ts == last_max:
                if stagnant_since is None:
                    stagnant_since = time.time()
                elif time.time() - stagnant_since > _STAGNANT_SEC:
                    break
            else:
                last_max = max_ts
                stagnant_since = None
    finally:
        coord.stop()

    o_cnt, o_min, o_max = _table_stats(db_path, "fastlio_odometry")
    l_cnt = _table_stats(db_path, "fastlio_lidar")[0]
    span = o_max - o_min
    print(
        f"[pcap_to_db] done odom={o_cnt} lidar={l_cnt} "
        f"ts=[{o_min:.3f}, {o_max:.3f}] span={span:.1f}s "
        f"wall={time.time() - t0:.1f}s",
        flush=True,
    )
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pcap", required=True, help="Livox Mid-360 pcap capture")
    parser.add_argument(
        "--db",
        default=None,
        help="target memory2 SQLite db. If it exists, fastlio streams are appended/aligned "
        "onto its clock; if it doesn't, a fresh db is built from scratch. "
        "Omit to default to <pcap>.db next to the pcap.",
    )
    parser.add_argument(
        "--odom-freq",
        type=float,
        default=30.0,
        help="FAST-LIO odometry publish rate in Hz (default 30)",
    )
    parser.add_argument(
        "--config",
        type=str,
        default=None,
        help="FAST-LIO yaml (relative to config/ or absolute); omit for the module default",
    )
    parser.add_argument(
        "--max-sensor-sec",
        type=float,
        default=0.0,
        help="stop after this many seconds of sensor time (0 = whole pcap)",
    )
    parser.add_argument(
        "--time-offset",
        type=float,
        default=None,
        help="seconds added to every output ts; omit to auto-derive from the db clock",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="overwrite existing fastlio_odometry/fastlio_lidar streams in the db",
    )
    return _run(parser.parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
