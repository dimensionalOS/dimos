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

"""Internal native build/replay test. Run with the installed application's Python."""

import math
from pathlib import Path
import shutil
import sys

from dimos.core.native_sources import native_source_root
from dimos.hardware.sensors.lidar.pointlio.scripts.pcap_to_db import main as replay
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.data import get_data


def main() -> None:
    for tool in ("cargo", "nix"):
        if shutil.which(tool) is None:
            raise RuntimeError(f"Missing {tool}; native acceptance cannot be skipped")
    root = native_source_root()
    if "site-packages" in str(root) or not (root / ".git").is_dir():
        raise RuntimeError(f"Expected an extracted native source snapshot, got {root}")
    results = Path(sys.argv[1])
    pcap = get_data("mid360_shake_stairs/mid360_shake_stairs.pcap")
    database = results / "pointlio.db"
    code = replay(
        ["--pcap", str(pcap), "--db", str(database), "--max-sensor-sec", "20", "--no-rrd"]
    )
    if code:
        raise RuntimeError(f"Point-LIO replay failed: {code}")
    store = SqliteStore(path=str(database), must_exist=True)
    try:
        odometry = list(store.stream("pointlio_odometry", Odometry).order_by("ts"))
        lidar = list(store.stream("pointlio_lidar", PointCloud2).order_by("ts"))
        for rows in (odometry, lidar):
            if len(rows) < 2 or rows[-1].ts <= rows[0].ts:
                raise RuntimeError("Native replay did not produce advancing lidar and odometry")
        for row in odometry:
            pose = row.data.pose.pose
            values = (
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            )
            if not all(math.isfinite(value) for value in values):
                raise RuntimeError("Native replay produced a non-finite pose")
    finally:
        store.stop()
    print(f"Native replay passed: {len(odometry)} poses, {len(lidar)} lidar frames")


if __name__ == "__main__":
    main()
