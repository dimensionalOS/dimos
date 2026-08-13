# REMOVE BEFORE MERGE: benchmarking scaffold for this branch only.

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

"""Benchmark the voxel ray-tracing Mapper on a recorded lidar+odometry .db.

Replays frames through the same Rust Mapper the native module runs and reports
per-frame update and emission timings at deployment cadences, plus map growth.

Usage:
    python -m dimos.mapping.ray_tracing.utils.bench_mapper nav_eval_sf_conference \
        --lidar-stream pointlio_lidar --odom-stream pointlio_odometry \
        --voxel-size 0.09 --fine-divisor 3

Set RAYON_NUM_THREADS before launch to test thread caps, e.g.
RAYON_NUM_THREADS=4.
"""

from __future__ import annotations

import os
import resource
import time

import numpy as np
import typer

from dimos.mapping.ray_tracing.utils.raytrace_rrd import _attach_pose_from_odom
from dimos.mapping.ray_tracing.voxel_map import VoxelRayMapper
from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.transform import FnTransformer
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.data import resolve_named_path


def _rss_mb() -> float:
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0


def _report(name: str, times_s: list[float], every: int) -> None:
    if not times_s:
        print(f"{name:12s} (never ran)")
        return
    ms = np.asarray(times_s) * 1e3
    amortized = float(np.mean(ms)) / every
    print(
        f"{name:12s} median {np.median(ms):7.2f} ms  p95 {np.percentile(ms, 95):7.2f} ms"
        f"  max {ms.max():7.2f} ms  amortized {amortized:6.2f} ms/frame (every {every})"
    )


def main(
    dataset: str = typer.Argument(..., help="Dataset .db: bare name (cwd or data/) or path"),
    lidar_stream: str = typer.Option("pointlio_lidar", "--lidar-stream"),
    odom_stream: str = typer.Option("pointlio_odometry", "--odom-stream"),
    align_tol: float = typer.Option(0.05, "--align-tol", help="Lidar/odom alignment tolerance (s)"),
    voxel_size: float = typer.Option(0.09, "--voxel-size", help="Voxel edge length (m)"),
    fine_divisor: int = typer.Option(0, "--fine-divisor", help="Fine cells per voxel edge, 0 off"),
    max_range: float = typer.Option(30.0, "--max-range", help="Max ray cast distance (m)"),
    emit_every: int = typer.Option(1, "--emit-every", help="Local map cadence, 0 disables"),
    fine_emit_every: int = typer.Option(
        1, "--fine-emit-every", help="Fine map cadence, 0 disables. Ignored without a divisor"
    ),
    global_emit_every: int = typer.Option(
        50, "--global-emit-every", help="Global map cadence, 0 disables"
    ),
    max_frames: int = typer.Option(0, "--max-frames", help="Stop after N frames, 0 for all"),
    lidar_hz: float = typer.Option(10.0, "--lidar-hz", help="Frame rate the budget line assumes"),
) -> None:
    mapper = VoxelRayMapper(
        voxel_size=voxel_size,
        max_range=max_range,
        fine_divisor=fine_divisor,
    )
    fine_on = fine_divisor > 0 and fine_emit_every > 0

    add_t: list[float] = []
    local_t: list[float] = []
    fine_t: list[float] = []
    global_t: list[float] = []
    points_per_frame: list[int] = []
    local_pts = fine_pts = global_pts = 0

    db_path = resolve_named_path(dataset, ".db")
    if not db_path.is_file():
        raise typer.BadParameter(f"dataset not found: {db_path}")
    store = SqliteStore(path=str(db_path))
    count = 0
    wall_start = time.perf_counter()
    with store:
        lidar = store.stream(lidar_stream, PointCloud2).order_by("ts")
        odom = store.stream(odom_stream, Odometry).order_by("ts")
        stream = lidar.align(odom, tolerance=align_tol).transform(
            FnTransformer(_attach_pose_from_odom)
        )
        for obs in stream:
            if obs.pose_tuple is None:
                continue
            x, y, z, qx, qy, qz, qw = obs.pose_tuple
            pts = obs.data.points_f32()
            points_per_frame.append(len(pts))

            t0 = time.perf_counter()
            mapper.add_frame(pts, (x, y, z), (qx, qy, qz, qw))
            add_t.append(time.perf_counter() - t0)
            count += 1

            local_due = emit_every > 0 and count % emit_every == 0
            fine_due = fine_on and count % fine_emit_every == 0
            if local_due or fine_due:
                cx, cy, radius, z_lo, z_hi = mapper.take_local_bounds()
            if local_due:
                t0 = time.perf_counter()
                local = mapper.local_map((cx, cy, 0.0), radius, z_lo, z_hi)
                local_t.append(time.perf_counter() - t0)
                local_pts = len(local)
            if fine_due:
                t0 = time.perf_counter()
                fine = mapper.local_map_fine((cx, cy, 0.0), radius, z_lo, z_hi)
                fine_t.append(time.perf_counter() - t0)
                fine_pts = len(fine)
            if global_emit_every > 0 and count % global_emit_every == 0:
                t0 = time.perf_counter()
                global_map = mapper.global_map()
                global_t.append(time.perf_counter() - t0)
                global_pts = len(global_map)

            if max_frames and count >= max_frames:
                break
    wall = time.perf_counter() - wall_start

    budget_ms = 1e3 / lidar_hz
    add_ms = np.asarray(add_t) * 1e3
    amortized = float(np.mean(add_ms))
    for times, every in [
        (local_t, emit_every or 1),
        (fine_t, fine_emit_every or 1),
        (global_t, global_emit_every or 1),
    ]:
        if times:
            amortized += float(np.mean(np.asarray(times) * 1e3)) / every

    print(
        f"dataset {db_path.name}, {count} frames, median {np.median(points_per_frame):.0f} pts/frame"
    )
    print(
        f"config voxel={voxel_size} divisor={fine_divisor} "
        f"cadences local/fine/global = {emit_every}/{fine_emit_every if fine_on else 0}/{global_emit_every}, "
        f"RAYON_NUM_THREADS={os.environ.get('RAYON_NUM_THREADS', 'unset')}"
    )
    _report("add_frame", add_t, 1)
    _report("local emit", local_t, emit_every or 1)
    _report("fine emit", fine_t, fine_emit_every or 1)
    _report("global emit", global_t, global_emit_every or 1)
    print(
        f"clouds: local {local_pts} pts ({local_pts * 16 / 1e3:.0f} KB), "
        f"fine {fine_pts} pts ({fine_pts * 16 / 1e3:.0f} KB), "
        f"global {global_pts} pts ({global_pts * 16 / 1e3:.0f} KB)"
    )
    print(
        f"amortized {amortized:.2f} ms/frame vs {budget_ms:.0f} ms budget at {lidar_hz:.0f} Hz "
        f"({100 * amortized / budget_ms:.0f}% of one core-equivalent)"
    )
    print(f"healthy voxels {mapper.voxel_count()}, process peak RSS {_rss_mb():.0f} MB")
    print(f"wall {wall:.1f}s ({count / wall:.1f} frames/s replay, includes .db decode)")


if __name__ == "__main__":
    typer.run(main)
