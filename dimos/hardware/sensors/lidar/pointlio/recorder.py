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

"""Record Point-LIO odometry + lidar into a memory2 SQLite db.

Subscribes to a PointLio's ``odometry`` / ``lidar`` outputs (auto-connected by
matching stream name + type — no remappings needed) and appends them to a
memory2 store. Timestamps are converted onto the db's existing clock so a run
can be appended to an existing db (e.g. a fastlio replay) and compared on one
timeline. Owns the db lifecycle: refuses to clobber existing streams unless
``force``, and derives the alignment reference from whatever the db already holds.

Each lidar frame is stamped with the latest odometry pose, so ``pointlio_lidar``
carries the trajectory and ``dimos map global`` can register it directly (it
transforms the body-frame cloud by that pose) — no ``dimos map pose-fill`` pass.
"""

from __future__ import annotations

from collections.abc import AsyncIterator
import math
from pathlib import Path
import sqlite3
import time

from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

# Below this the db's existing timestamps are sensor-boot seconds, not unix time.
_SENSOR_CLOCK_MAX = 1e8
# Strictly-increasing tie-breaker so two samples never collide on ts.
_EPS = 1e-9
# Max sensor-ts gap to attach the latest odometry pose to a lidar frame, so
# pointlio_lidar carries the trajectory and `dimos map global` can register it
# (it transforms by the per-frame pose). Matches pose_fill's nearest-match window.
_POSE_MATCH_TOL = 0.1


def _existing_min_ts(db_path: Path) -> float:
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
                cols = [col[1] for col in con.execute(f"PRAGMA table_info('{table}')").fetchall()]
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


class PointlioRecorderConfig(ModuleConfig):
    """Configures the recorder with the target db and timing conversion."""

    db_path: str = ""
    # db stream/table names the Point-LIO outputs are recorded under.
    odom_stream_name: str = "pointlio_odometry"
    lidar_stream_name: str = "pointlio_lidar"
    # Explicit offset override; NaN means auto-derive from the db's earliest ts.
    time_offset: float = float("nan")
    # Drop pre-existing odom/lidar streams instead of refusing to overwrite.
    force: bool = False


class PointlioRecorder(Module):
    config: PointlioRecorderConfig

    lidar: In[PointCloud2]
    odometry: In[Odometry]
    _offset: float | None = None
    _ref_start_ts: float = -1.0
    _last_odom_ts: float = 0.0
    _last_lidar_ts: float = 0.0
    _odom_count: int = 0
    _lidar_count: int = 0
    # Latest odometry pose + its raw sensor ts, stamped onto each lidar frame so
    # pointlio_lidar carries the trajectory (no separate pose-fill pass).
    _last_odom_pose: Pose | None = None
    _last_odom_raw_ts: float = 0.0

    async def main(self) -> AsyncIterator[None]:
        # Deferred: the store is opened in the worker process that runs main(),
        # not at module-scan/import time on the host.
        from dimos.memory2.store.sqlite import SqliteStore

        cfg = self.config
        self._store = SqliteStore(path=cfg.db_path)
        names = (cfg.odom_stream_name, cfg.lidar_stream_name)
        existing = sorted(set(self._store.list_streams()) & set(names))
        if existing and not cfg.force:
            raise RuntimeError(
                f"PointlioRecorder: {Path(cfg.db_path).name} already has {existing}; "
                "set force=True to overwrite"
            )
        for name in existing:
            self._store.delete_stream(name)
        # Reference is the db's earliest ts *after* dropping our own old streams,
        # so only the data we're aligning against (e.g. a fastlio replay) counts.
        self._ref_start_ts = _existing_min_ts(Path(cfg.db_path))
        self._os = self._store.stream(cfg.odom_stream_name, Odometry)
        self._ls = self._store.stream(cfg.lidar_stream_name, PointCloud2)
        yield
        self._store.stop()

    def _resolve_offset(self, first_ts: float) -> float:
        override = self.config.time_offset
        if not math.isnan(override):
            return override
        ref = self._ref_start_ts
        if ref < 0.0 or ref < _SENSOR_CLOCK_MAX:
            # Empty db, or db already on the sensor clock -> exact alignment.
            return 0.0
        # db on wall-clock time -> start-align Point-LIO onto the db's earliest ts.
        return ref - first_ts

    def _aligned_ts(self, raw_ts: float, last_ts: float) -> float:
        """Convert a sensor ts onto the db clock, kept strictly above last_ts."""
        if self._offset is None:
            self._offset = self._resolve_offset(raw_ts)
        return max(raw_ts + self._offset, last_ts + _EPS)

    async def handle_odometry(self, msg: Odometry) -> None:
        # `is not None`, not `or`: a real sensor ts of 0.0 must not fall back to
        # wall time (would misclassify the stream's clock in _resolve_offset).
        raw_ts_raw = getattr(msg, "ts", None)
        raw_ts = raw_ts_raw if raw_ts_raw is not None else time.time()
        ts = self._aligned_ts(raw_ts, self._last_odom_ts)
        self._last_odom_ts = ts
        pose = getattr(msg, "pose", None)
        pose_inner = getattr(pose, "pose", None) if pose is not None else None
        self._os.append(msg, ts=ts, pose=pose_inner)
        self._last_odom_pose = pose_inner
        self._last_odom_raw_ts = raw_ts
        self._odom_count += 1

    async def handle_lidar(self, msg: PointCloud2) -> None:
        raw_ts_raw = getattr(msg, "ts", None)
        raw_ts = raw_ts_raw if raw_ts_raw is not None else time.time()
        ts = self._aligned_ts(raw_ts, self._last_lidar_ts)
        self._last_lidar_ts = ts
        # Stamp the latest odometry pose (within tolerance) onto the frame so
        # pointlio_lidar carries the trajectory; map global transforms the
        # body-frame cloud by it. Both Point-LIO outputs share a publish ts, so
        # the nearest odometry is at most ~one odom period stale. Frames with no
        # match (e.g. before the first odometry) get None and are map-skipped.
        pose = (
            self._last_odom_pose
            if self._last_odom_pose is not None
            and abs(raw_ts - self._last_odom_raw_ts) <= _POSE_MATCH_TOL
            else None
        )
        self._ls.append(msg, ts=ts, pose=pose)
        self._lidar_count += 1
