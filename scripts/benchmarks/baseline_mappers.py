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

"""Time the Python VoxelGridMapper + CostMapper algorithms on the exact frames
`--replay run unitree-go2` feeds them (data/go2_short.db).

Mirrors the unitree-go2 blueprint configuration:
  VoxelGridMapper.blueprint(emit_every=5)  -> VoxelGrid(voxel_size=0.05, carve_columns=True)
  CostMapper.blueprint()                   -> height_cost_occupancy(HeightCostConfig())

Stages timed per frame:
  add_frame  - voxelize + column-carve + insert (every frame)
  emit       - rebuild global PointCloud2 from the voxel hash map (every 5th frame)
  costmap    - height_cost_occupancy on the emitted global map (every 5th frame)

``--device`` selects the Open3D device VoxelGrid runs on (the only GPU-capable
stage; height_cost_occupancy is numba + scipy on CPU regardless). A CUDA request
falls back to CPU when no CUDA device is present, so the *resolved* device is
what gets recorded next to the results.

Usage:
    uv run python scripts/benchmarks/baseline_mappers.py --device CUDA:0 --out py_cuda.csv
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
import json
from pathlib import Path
import statistics
import time

from bench_util import growth, machine_info, pct, resolve_open3d_device

from dimos.mapping.pointclouds.occupancy import HeightCostConfig, height_cost_occupancy
from dimos.mapping.voxels import VoxelGrid

EMIT_EVERY = 5


def summarize(name: str, values: list[float]) -> None:
    if not values:
        print(f"  {name:10s}  (no samples)")
        return
    print(
        f"  {name:10s}  n={len(values):4d}  mean={statistics.mean(values):8.2f}  "
        f"p50={pct(values, 50):8.2f}  p95={pct(values, 95):8.2f}  max={max(values):8.2f}  (ms)"
    )


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument(
        "--device",
        default="CUDA:0",
        help="Open3D device for VoxelGrid (default: CUDA:0, falling back to CPU:0 when "
        "no CUDA device is present). Pass CPU:0 to force the CPU path.",
    )
    ap.add_argument("--db", default="data/go2_short.db", help="memory2 SQLite replay dataset")
    ap.add_argument("--out", default="baseline_python.csv", help="per-frame CSV to write")
    return ap.parse_args()


def main() -> None:
    args = parse_args()
    # Imported here so --help works without the (slow) dimos memory2 import chain.
    from dimos.memory2.store.sqlite import SqliteStore

    resolved_device, cuda_available = resolve_open3d_device(args.device)
    if args.device.startswith("CUDA") and not cuda_available:
        print(
            f"WARNING: {args.device} requested but no CUDA device is available; "
            "running on CPU:0. These are NOT GPU numbers."
        )

    store = SqliteStore(path=args.db, must_exist=True)
    store.start()
    lidar = store.replay().stream("lidar")

    n_frames = lidar.count()
    print(f"dataset: {args.db}  lidar frames: {n_frames}  device: {resolved_device}")

    grid = VoxelGrid(voxel_size=0.05, carve_columns=True, device=args.device)
    cost_cfg = asdict(HeightCostConfig())

    rows = []
    add_ms: list[float] = []
    emit_ms: list[float] = []
    cost_ms: list[float] = []
    first_ts = last_ts = None
    n_points_total = 0
    t_start = time.perf_counter()

    for i, (ts, frame) in enumerate(lidar.iterate_ts(), 1):
        if first_ts is None:
            first_ts = ts
        last_ts = ts

        pts, _ = frame.as_numpy()
        n_pts = len(pts)
        n_points_total += n_pts

        t0 = time.perf_counter()
        grid.add_frame(frame)
        t_add = (time.perf_counter() - t0) * 1000
        add_ms.append(t_add)

        t_emit = t_cost = float("nan")
        n_map_pts = grid_wh = None
        if i % EMIT_EVERY == 0:
            t0 = time.perf_counter()
            gm = grid.get_global_pointcloud2()
            t_emit = (time.perf_counter() - t0) * 1000
            emit_ms.append(t_emit)

            map_pts, _ = gm.as_numpy()
            n_map_pts = len(map_pts)

            t0 = time.perf_counter()
            og = height_cost_occupancy(gm, **cost_cfg)
            t_cost = (time.perf_counter() - t0) * 1000
            cost_ms.append(t_cost)
            grid_wh = f"{og.grid.shape[1]}x{og.grid.shape[0]}"

        rows.append(
            {
                "frame": i,
                "ts": ts,
                "n_points": n_pts,
                "voxels": grid.size(),
                "add_ms": round(t_add, 3),
                "emit_ms": round(t_emit, 3) if t_emit == t_emit else "",
                "cost_ms": round(t_cost, 3) if t_cost == t_cost else "",
                "map_points": n_map_pts if n_map_pts is not None else "",
                "grid_wh": grid_wh or "",
            }
        )
        if i % 100 == 0:
            print(f"  ...frame {i}/{n_frames}  voxels={grid.size()}")

    wall = time.perf_counter() - t_start
    rec_dur = (last_ts - first_ts) if (first_ts is not None and last_ts is not None) else 0.0
    rate = (len(add_ms) - 1) / rec_dur if rec_dur > 0 else float("nan")

    out_csv = Path(args.out)
    with open(out_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    meta = {
        "impl": "python",
        "requested_device": args.device,
        "resolved_device": resolved_device,
        "cuda_available": cuda_available,
        "db": args.db,
        "n_frames": len(add_ms),
        "emit_every": EMIT_EVERY,
        "wall_s": round(wall, 3),
        "final_voxels": grid.size(),
        "machine": machine_info(),
    }
    meta_path = out_csv.with_suffix(".meta.json")
    meta_path.write_text(json.dumps(meta, indent=2) + "\n")

    print(f"\nwrote {out_csv} and {meta_path}")
    print(
        f"\nrecording: {rec_dur:.1f}s of lidar at {rate:.2f} Hz "
        f"(budget {1000 / rate:.0f} ms/frame, {EMIT_EVERY * 1000 / rate:.0f} ms/emit-cycle)"
    )
    print(f"avg points/frame: {n_points_total // max(1, len(add_ms))}")
    print(f"final voxel count: {grid.size()}")
    print(
        f"processed {len(add_ms)} frames in {wall:.1f}s wall "
        f"({len(add_ms) / wall:.1f} frames/s offline)\n"
    )

    print("per-stage latency (ms):")
    summarize("add_frame", add_ms)
    summarize("emit", emit_ms)
    # First costmap call includes numba JIT compile; report separately.
    summarize("costmap", cost_ms[1:])
    if cost_ms:
        print(f"  {'costmap[0]':10s}  first call (numba JIT): {cost_ms[0]:.0f} ms")

    print("\ngrowth (first-quarter mean -> last-quarter mean, ms):")
    for name, values in (("add_frame", add_ms), ("emit", emit_ms), ("costmap", cost_ms[1:])):
        a, b = growth(values)
        if a == a:
            print(f"  {name:10s}  {a:8.2f} -> {b:8.2f}   ({b / a:.1f}x)")

    grid.dispose()
    store.stop()


if __name__ == "__main__":
    main()
