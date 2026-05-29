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

"""Record raw Mid-360 pcap + FastLio2 outputs into a memory2 SqliteStore.

Spins up the canonical fastlio blueprint with `record_pcap=True`, attaches a
recorder module that appends every `lidar` and `odometry` message to a
memory2 store, runs for --duration seconds, and tears down.

Output goes under `dimos/hardware/sensors/lidar/fastlio2/recordings/<ts>/`:
    mid360.pcap   raw UDP traffic during the run
    fastlio.db    memory2 SqliteStore with `lidar` + `odometry` streams
"""

from __future__ import annotations

import argparse
from collections.abc import AsyncIterator
from datetime import datetime
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
_DEFAULT_LIDAR_IP = "192.168.1.157"
_RECORDINGS_DIR = Path(__file__).resolve().parent.parent / "recordings"


class FastLio2RecorderConfig(ModuleConfig):
    db_path: str = "fastlio.db"


class FastLio2Recorder(Module):
    config: FastLio2RecorderConfig
    lidar: In[PointCloud2]
    odometry: In[Odometry]

    _store: SqliteStore
    _lidar_stream: Stream[Any, Any]
    _odom_stream: Stream[Any, Any]
    _lidar_count: int = 0
    _odom_count: int = 0

    async def main(self) -> AsyncIterator[None]:
        self._store = SqliteStore(path=self.config.db_path)
        self._lidar_stream = self._store.stream("lidar", PointCloud2)
        self._odom_stream = self._store.stream("odometry", Odometry)
        yield
        self._store.stop()

    async def handle_lidar(self, value: PointCloud2) -> None:
        self._lidar_stream.append(value, ts=getattr(value, "ts", None) or time.time())
        self._lidar_count += 1

    async def handle_odometry(self, value: Odometry) -> None:
        self._odom_stream.append(value, ts=getattr(value, "ts", None) or time.time())
        self._odom_count += 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--duration",
        "-d",
        type=float,
        default=1.0,
        help="Recording wall time, in seconds (default: %(default)s)",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=None,
        help=(f"Recording directory. Default: {_RECORDINGS_DIR}/<YYYYMMDD_HHMMSS>/"),
    )
    parser.add_argument(
        "--lidar-ip",
        default=_DEFAULT_LIDAR_IP,
        help=f"Mid-360 IP (default: {_DEFAULT_LIDAR_IP})",
    )
    args = parser.parse_args()

    if args.out_dir is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        rec_dir = _RECORDINGS_DIR / stamp
    else:
        rec_dir = args.out_dir
    rec_dir.mkdir(parents=True, exist_ok=True)
    pcap_path = rec_dir / "mid360.pcap"
    db_path = rec_dir / "fastlio.db"

    print(f"[demo_record] recording to {rec_dir} for {args.duration:.1f}s", flush=True)

    blueprint = autoconnect(
        FastLio2.blueprint(
            voxel_size=_VOXEL_SIZE,
            map_voxel_size=_VOXEL_SIZE,
            map_freq=-1,
            lidar_ip=args.lidar_ip,
            record_pcap=True,
            record_pcap_path=pcap_path,
            single_threaded=True,
            deterministic_clock=True,
        ),
        FastLio2Recorder.blueprint(db_path=str(db_path)),
    ).global_config(n_workers=2, robot_model="mid360_fastlio2_record")

    coordinator = ModuleCoordinator.build(blueprint)
    try:
        time.sleep(args.duration)
    finally:
        coordinator.stop()

    pcap_size = pcap_path.stat().st_size if pcap_path.exists() else 0
    db_size = db_path.stat().st_size if db_path.exists() else 0
    print(f"[demo_record] pcap: {pcap_path}  ({pcap_size / 1e6:.2f} MB)", flush=True)
    print(f"[demo_record] db:   {db_path}  ({db_size / 1e6:.2f} MB)", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
