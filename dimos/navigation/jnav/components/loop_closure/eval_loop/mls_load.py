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

"""What the MLS planner pays downstream of each map design, over a recording.

whole: Ivan's PGOVoxelMapper publishes its whole corrected map (every 5th frame, and on each rebuild),
       which the planner can only take through update_global_map, a full rebuild. Every
       `--whole-every`th publish and every rebuild is timed, as published (5 cm) and downsampled to the
       planner's voxel size; on each rebuild the keyframes' map positions are kept, to measure how far a
       fixed goal ends up from the place it was set on.
region: the ray tracer's local cylinder after each frame goes to update_region, as main's nav_3d
        blueprints wire it. With --closures-from <gsc pgo-eval json>, gsc's loop closures are also
        charged a full update_global_map of the ray tracer's map, at most once per Ivan's rebuild
        cooldown (10 s): an upper bound for re-planning on a corrected map without patching in place.
        With --patch-from <pgo-eval json from --loop-snapshots> (-> <name>.mls_patch.json), each
        correction (same cooldown) is instead applied in place: only the PATCH_CELL_M cells holding a
        keyframe that moved more than a voxel, plus their neighbors, go to update_region. --patch-from also takes
        a mls_whole json, to patch Ivan's rebuilds the same way (with --label patch_ivan).

usage: mls_load.py <name> whole|region [--whole-every N] [--closures-from JSON]
       (reads $EVAL_LOOP_OUT/<name>.src.db)
       -> $EVAL_LOOP_OUT/<name>.mls_<mode>.json
"""

import argparse
import json
from pathlib import Path
import resource
import time

from dimos_mls_planner import MLSPlanner
import numpy as np
from paths import OUT, import_ivan

from dimos.memory.store.sqlite import SqliteStore
from dimos.robot.unitree.go2.nav_3d_config import mls_planner_config, ray_tracing_config

# Ivan's PGOMap rebuilds at most this often (PGOVoxelMapperConfig.rebuild_cooldown_s)
REBUILD_COOLDOWN_S = 10.0


def planner() -> MLSPlanner:
    cfg = mls_planner_config
    return MLSPlanner(
        voxel_size=cfg.voxel_size,
        robot_height=cfg.robot_height,
        max_overhead_m=cfg.max_overhead_m,
        surface_closing_radius=cfg.surface_closing_radius,
        node_spacing_m=cfg.node_spacing_m,
        wall_clearance_m=cfg.wall_clearance_m,
        wall_buffer_m=cfg.wall_buffer_m,
        wall_buffer_weight=cfg.wall_buffer_weight,
        step_threshold_m=cfg.step_threshold_m,
        step_penalty_weight=cfg.step_penalty_weight,
        worker_threads=cfg.worker_threads,
    )


def downsample(points: np.ndarray, voxel: float) -> np.ndarray:
    """One point (the voxel center) per occupied voxel."""
    keys = np.unique(np.floor(points / voxel).astype(np.int64), axis=0)
    return ((keys + 0.5) * voxel).astype(np.float32)


# A correction is patched in square cells of this size (the ray tracer's region_m), each with its neighbors
PATCH_CELL_M = 4.0


def moved_cells(before: np.ndarray, after: np.ndarray, voxel: float) -> list[tuple[int, int]]:
    """Cells holding a keyframe that moved more than a voxel between two snapshots, plus their neighbors."""
    moved = after[np.linalg.norm(after - before, axis=1) > voxel]
    cells = {tuple(c) for c in np.floor(moved[:, :2] / PATCH_CELL_M).astype(int)}
    return sorted({(x + dx, y + dy) for x, y in cells for dx in (-1, 0, 1) for dy in (-1, 0, 1)})


def load_snapshots(path: str) -> list:
    """[ts, keyframe xyz] per correction, from a pgo-eval --loop-snapshots json or a mls_whole json (Ivan's rebuilds)."""
    result = json.loads(Path(path).read_text())
    if "loop_snapshots" in result:
        return result["loop_snapshots"]
    return [[r["ts"], r["keyframe_xyz"]] for r in result["rebuilds"]]


def peak_rss_mb() -> float:
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 2**20  # macOS: bytes


def whole(store, every: int) -> dict:
    module, _, pgo_map = import_ivan()
    cfg = module.PGOVoxelMapperConfig(emit_every=5)  # what unitree-go2-pgo runs
    gmap = pgo_map.PGOMap(
        voxel_size=cfg.voxel_size,
        frame_id=cfg.frame_id,
        rebuild_cooldown_s=cfg.rebuild_cooldown_s,
        pgo=cfg.pgo,
    )
    mls = planner()
    publishes, timed, rebuilds = 0, [], []
    for i, obs in enumerate(store.streams.lidar.order_by("ts")):
        rebuilt = gmap.add(obs.data, obs.pose_stamped)
        if rebuilt:
            positions, _ = gmap.keyframe_poses()
            rebuilds.append({"ts": obs.ts, "keyframe_xyz": np.round(positions, 3).tolist()})
        if not (rebuilt or (i + 1) % cfg.emit_every == 0):
            continue
        publishes += 1
        if publishes % every and not rebuilt:
            continue
        points = np.asarray(gmap.global_map().points_f32(), np.float32)
        started = time.perf_counter()
        mls.update_global_map(points)
        full_ms = (time.perf_counter() - started) * 1e3
        coarse = downsample(points, mls_planner_config.voxel_size)
        started = time.perf_counter()
        mls.update_global_map(coarse)
        coarse_ms = (time.perf_counter() - started) * 1e3
        timed.append([obs.ts, full_ms, len(points), int(rebuilt), coarse_ms, len(coarse)])
        if len(timed) % 20 == 0:
            print(
                f"frame {i} publish {publishes} points {len(points)} mls {timed[-1][1]:.0f} ms",
                flush=True,
            )
    gmap.dispose()
    return {
        "publishes": publishes,
        "timed": timed,
        "rebuilds": rebuilds,
        "peak_rss_mb": peak_rss_mb(),
    }


def region(store, closures_from: str | None, patch_from: str | None) -> dict:
    from dimos.mapping.ray_tracing.voxel_map import VoxelRayMapper

    rt = ray_tracing_config
    mapper = VoxelRayMapper(voxel_size=rt.voxel_size, max_range=rt.max_range, emit_every=1)
    mls = planner()
    timed, closure_rebuilds = [], []
    closure_times: list[float] = []
    if closures_from:
        result = json.loads(Path(closures_from).read_text())
        graph_times = [row[0] for row in result["graph"]]
        closure_times = sorted(
            graph_times[edge["source_id"]]
            for edge in result["committed_loop_edges"]
            if edge["gnc_weight"] > 0
        )
    snapshots = load_snapshots(patch_from) if patch_from else []
    patches: list = []
    next_snapshot, applied, last_patch = 0, None, -np.inf
    pending, last_rebuild = 0, -np.inf
    for i, obs in enumerate(store.streams.lidar.order_by("ts")):
        while next_snapshot < len(snapshots) and snapshots[next_snapshot][0] <= obs.ts:
            next_snapshot += 1
        if next_snapshot and obs.ts - last_patch >= REBUILD_COOLDOWN_S:
            latest = np.asarray(snapshots[next_snapshot - 1][1], float)
            if (
                applied is None
                or len(latest) > len(applied)
                or not np.array_equal(latest[: len(applied)], applied)
            ):
                cells = (
                    []
                    if applied is None
                    else moved_cells(applied, latest[: len(applied)], mapper.voxel_size)
                )
                started = time.perf_counter()
                points_total = 0
                for x, y in cells:
                    center = ((x + 0.5) * PATCH_CELL_M, (y + 0.5) * PATCH_CELL_M)
                    near = latest[np.linalg.norm(latest[:, :2] - center, axis=1) < PATCH_CELL_M * 2]
                    sensor_z = float(near[:, 2].mean()) if len(near) else float(obs.pose.position.z)
                    z_min, z_max = sensor_z - 3.0, sensor_z + mls_planner_config.max_overhead_m
                    radius = PATCH_CELL_M / np.sqrt(2)
                    cell_points = mapper.local_map((*center, sensor_z), radius, z_min, z_max)
                    points_total += len(cell_points)
                    mls.update_region(cell_points, center, radius, z_min, z_max, sensor_z)
                if applied is not None:
                    patches.append(
                        [obs.ts, (time.perf_counter() - started) * 1e3, points_total, len(cells)]
                    )
                applied, last_patch = latest, obs.ts
        while pending < len(closure_times) and closure_times[pending] <= obs.ts:
            pending += 1
        if (
            pending
            and obs.ts - last_rebuild >= REBUILD_COOLDOWN_S
            and len(closure_rebuilds) < pending
        ):
            points = mapper.global_map()
            started = time.perf_counter()
            mls.update_global_map(points)
            closure_rebuilds.append(
                [obs.ts, (time.perf_counter() - started) * 1e3, len(points), pending]
            )
            last_rebuild = obs.ts
        p = obs.pose.position
        origin = (p.x, p.y, p.z)
        mapper.add_frame_world(np.asarray(obs.data.points_f32(), np.float32), origin)
        cx, cy, radius, z_min, z_max = mapper.take_local_bounds()
        local = mapper.local_map(origin, radius, z_min, z_max)
        started = time.perf_counter()
        mls.update_region(local, (cx, cy), radius, z_min, z_max, p.z)
        timed.append([obs.ts, (time.perf_counter() - started) * 1e3, len(local), 0])
        if i % 2000 == 0:
            print(f"frame {i} region points {len(local)} mls {timed[-1][1]:.1f} ms", flush=True)
    return {
        "publishes": len(timed),
        "timed": timed,
        "rebuilds": [],
        "closure_rebuilds": closure_rebuilds,
        "closures": len(closure_times),
        "patches": patches,
        "patch_cell_m": PATCH_CELL_M,
        "peak_rss_mb": peak_rss_mb(),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("name")
    parser.add_argument("mode", choices=("whole", "region"))
    parser.add_argument("--whole-every", type=int, default=10)
    parser.add_argument("--closures-from")
    parser.add_argument("--patch-from")
    parser.add_argument("--label", help="output suffix, default mode (or patch with --patch-from)")
    args = parser.parse_args()
    with SqliteStore(path=str(OUT / f"{args.name}.src.db")) as store:
        result = (
            whole(store, args.whole_every)
            if args.mode == "whole"
            else region(store, args.closures_from, args.patch_from)
        )
    result["whole_every"] = args.whole_every if args.mode == "whole" else 1
    label = args.label or ("patch" if args.patch_from else args.mode)
    (OUT / f"{args.name}.mls_{label}.json").write_text(json.dumps(result))
    print(f"{args.mode}: {result['publishes']} publishes, {len(result['timed'])} timed", flush=True)


if __name__ == "__main__":
    main()
