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

"""End-to-end Rust-vs-Python mapper benchmark on the go2_short replay.

Launches a blueprint (`unitree-go2` = Python mappers, `unitree-go2-rust-mapping`
= Rust mappers), and while the replay plays:

  - subscribes (Zenoh) to dimos/lidar, dimos/global_map, dimos/global_costmap,
    recording (stamp, wall-arrival) pairs;
  - samples the whole process tree's CPU%/RSS once a second.

Because both stacks propagate the lidar stamp (global_map.ts = last lidar ts,
costmap.ts = map ts), stamp matching yields per-hop latencies that include
decode + compute + encode + transport:

    lidar->map   voxel accumulation (5 frames) + emit + publish
    map->cost    costmap compute + publish

Usage:
    uv run python scripts/benchmarks/e2e_bench.py unitree-go2 out_python.json
    uv run python scripts/benchmarks/e2e_bench.py unitree-go2-rust-mapping out_rust.json
"""

import json
import signal
import statistics
import subprocess
import sys
import time

from dimos.core.transport import ZenohTransport
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

IDLE_STOP_S = 8.0  # stop once lidar has been silent this long (replay drained)
MAX_RUN_S = 180.0


def proc_tree(root_pid: int) -> list[tuple[int, float, float]]:
    """(pid, pcpu, rss_mb) for root and all descendants."""
    out = subprocess.run(
        ["ps", "-axo", "pid=,ppid=,pcpu=,rss="], capture_output=True, text=True
    ).stdout
    children: dict[int, list[int]] = {}
    info: dict[int, tuple[float, float]] = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) != 4:
            continue
        pid, ppid = int(parts[0]), int(parts[1])
        children.setdefault(ppid, []).append(pid)
        info[pid] = (float(parts[2]), float(parts[3]) / 1024.0)
    result, stack = [], [root_pid]
    while stack:
        pid = stack.pop()
        if pid in info:
            result.append((pid, info[pid][0], info[pid][1]))
        stack.extend(children.get(pid, []))
    return result


def pct(vals, p):
    s = sorted(vals)
    return s[min(len(s) - 1, round(p / 100 * (len(s) - 1)))] if s else float("nan")


def main() -> None:
    blueprint, out_path = sys.argv[1], sys.argv[2]

    arrivals: dict[str, list[tuple[float, float]]] = {"lidar": [], "map": [], "cost": []}

    def rec(name):
        def cb(msg):
            arrivals[name].append((round(float(msg.ts), 6), time.time()))

        return cb

    ZenohTransport("dimos/lidar", PointCloud2).subscribe(rec("lidar"))
    ZenohTransport("dimos/global_map", PointCloud2).subscribe(rec("map"))
    ZenohTransport("dimos/global_costmap", OccupancyGrid).subscribe(rec("cost"))

    proc = subprocess.Popen(
        ["uv", "run", "dimos", "--replay", "run", blueprint],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    print(f"launched {blueprint} (pid {proc.pid}); waiting for data...")

    cpu_samples: list[float] = []
    rss_samples: list[float] = []
    t0 = time.time()
    last_lidar_n = 0
    last_lidar_t = time.time()
    try:
        while time.time() - t0 < MAX_RUN_S:
            time.sleep(1.0)
            tree = proc_tree(proc.pid)
            if tree:
                cpu_samples.append(sum(c for _, c, _ in tree))
                rss_samples.append(sum(r for _, _, r in tree))
            n = len(arrivals["lidar"])
            if n > last_lidar_n:
                last_lidar_n, last_lidar_t = n, time.time()
            elif n > 0 and time.time() - last_lidar_t > IDLE_STOP_S:
                print("replay drained; stopping")
                break
    finally:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=20)
        except subprocess.TimeoutExpired:
            proc.kill()

    # Stamp-matched per-hop latencies (first arrival per stamp).
    def first_by_stamp(name):
        d: dict[float, float] = {}
        for ts, wall in arrivals[name]:
            d.setdefault(ts, wall)
        return d

    lid, gmap, cost = first_by_stamp("lidar"), first_by_stamp("map"), first_by_stamp("cost")
    lid_to_map = [gmap[ts] - lid[ts] for ts in gmap if ts in lid]
    map_to_cost = [cost[ts] - gmap[ts] for ts in cost if ts in gmap]

    def stats(vals):
        vals = [v * 1000 for v in vals]
        if not vals:
            return {}
        return {
            "n": len(vals),
            "mean_ms": statistics.mean(vals),
            "p50_ms": pct(vals, 50),
            "p95_ms": pct(vals, 95),
            "max_ms": max(vals),
        }

    result = {
        "blueprint": blueprint,
        "counts": {k: len(v) for k, v in arrivals.items()},
        "lidar_to_map": stats(lid_to_map),
        "map_to_cost": stats(map_to_cost),
        "cpu_pct": {
            "mean": statistics.mean(cpu_samples) if cpu_samples else None,
            "p95": pct(cpu_samples, 95),
        },
        "rss_mb": {
            "mean": statistics.mean(rss_samples) if rss_samples else None,
            "max": max(rss_samples) if rss_samples else None,
        },
    }
    with open(out_path, "w") as f:
        json.dump(result, f, indent=2)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
