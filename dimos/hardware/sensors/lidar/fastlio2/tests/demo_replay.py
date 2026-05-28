#!/usr/bin/env python3
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

"""Replay a recorded Mid-360 pcap through FastLio2 offline.

Runs `fastlio2_native --replay_pcap <path>` (no SDK, no network) and saves
its `lidar` and `odometry` outputs to `replay.db` next to the pcap.

Pair with `demo_record.py` (which produces the pcap + ground-truth db) and
`demo_compare.py` (which checks the two dbs match).
"""

from __future__ import annotations

import argparse
from collections.abc import AsyncIterator
from pathlib import Path
import time
from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.stream import Stream
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

_VOXEL_SIZE = 0.05
# memory2 SqliteStore enforces UNIQUE on ts. In replay the C++ binary may
# publish two consecutive messages with the same virtual-clock ts (because
# the wall-clock-driven publish loop fires faster than the packet feeder
# advances the clock). Bump duplicate ts by this much to keep all rows.
_TS_EPSILON = 1e-9


class FastLio2ReplayRecorderConfig(ModuleConfig):
    db_path: str = "replay.db"


class FastLio2ReplayRecorder(Module):
    config: FastLio2ReplayRecorderConfig
    lidar: In[PointCloud2]
    odometry: In[Odometry]

    _store: SqliteStore
    _lidar_stream: Stream[Any, Any]
    _odom_stream: Stream[Any, Any]
    _last_lidar_ts: float = 0.0
    _last_odom_ts: float = 0.0

    async def main(self) -> AsyncIterator[None]:
        self._store = SqliteStore(path=self.config.db_path)
        self._lidar_stream = self._store.stream("lidar", PointCloud2)
        self._odom_stream = self._store.stream("odometry", Odometry)
        yield
        self._store.stop()

    async def handle_lidar(self, value: PointCloud2) -> None:
        ts = max(
            getattr(value, "ts", None) or time.time(),
            self._last_lidar_ts + _TS_EPSILON,
        )
        self._last_lidar_ts = ts
        self._lidar_stream.append(value, ts=ts)

    async def handle_odometry(self, value: Odometry) -> None:
        ts = max(
            getattr(value, "ts", None) or time.time(),
            self._last_odom_ts + _TS_EPSILON,
        )
        self._last_odom_ts = ts
        self._odom_stream.append(value, ts=ts)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "pcap",
        type=Path,
        help="Recorded mid360.pcap from demo_record",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help="Output replay.db path. Default: <pcap dir>/replay.db",
    )
    parser.add_argument(
        "--max-wait",
        type=float,
        default=120.0,
        help=(
            "Hard cap on replay wall time, in seconds. Replay is real-time-"
            "paced, so a 20s pcap takes ~20s. Default: %(default)s"
        ),
    )
    args = parser.parse_args()

    pcap = args.pcap.resolve()
    if not pcap.exists():
        print(f"[demo_replay] pcap not found: {pcap}")
        return 1

    # If the paired live run wrote a first-packet sidecar, use it to skip
    # the SDK-warmup prefix in the pcap so replay sees the same first
    # packet the live algorithm saw.
    marker = pcap.with_suffix(pcap.suffix + ".first.ns")
    skip_until_ns: int | None = None
    if marker.exists():
        skip_until_ns = int(marker.read_text().strip())
        print(f"[demo_replay] sidecar: {marker}  skip_until_ns={skip_until_ns}")

    db_path = args.out.resolve() if args.out else pcap.parent / "replay.db"
    if db_path.exists():
        db_path.unlink()
    db_path.parent.mkdir(parents=True, exist_ok=True)

    print(f"[demo_replay] pcap: {pcap}")
    print(f"[demo_replay] db:   {db_path}")

    blueprint = autoconnect(
        FastLio2.blueprint(
            voxel_size=_VOXEL_SIZE,
            map_voxel_size=_VOXEL_SIZE,
            map_freq=-1,
            replay_pcap=pcap,
            replay_skip_until_ns=skip_until_ns,
            single_threaded=True,
            deterministic_clock=True,
        ),
        FastLio2ReplayRecorder.blueprint(db_path=str(db_path)),
    ).global_config(n_workers=2, robot_model="mid360_fastlio2_replay")

    coordinator = ModuleCoordinator.build(blueprint)
    try:
        time.sleep(args.max_wait)
    finally:
        coordinator.stop()

    db_size = db_path.stat().st_size if db_path.exists() else 0
    print(f"[demo_replay] wrote {db_path} ({db_size / 1e6:.2f} MB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
