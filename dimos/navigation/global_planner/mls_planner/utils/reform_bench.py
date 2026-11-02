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

"""What the MLS planner pays to follow loop closures, replayed over a recording.

replay: the ray tracer's local cylinder after each frame goes to update_region, as main's nav_3d
        blueprints wire it, into two planners:
        - patch: the cylinder as the ray tracer has it, and after each closure (10 s cooldown) the
          4 m cells holding a keyframe that moved more than a voxel, plus neighbors, re-applied
          through update_region (the eval_loop "gsc patched in place" method);
        - reform: the cylinder moved into the corrected frame, and every closure applied in place
          with apply_loop_closure (fast-reform warp of voxels, nodes, cached path and held goals).
        Goals held at keyframes are compared against where the optimizer puts those keyframes, and
        with --checks a few big closures are checked against a planner rebuilt from scratch on the
        same corrected voxels.
ivan:   Ivan's PGOVoxelMapper publishes its whole corrected map, which the planner can only take
        through update_global_map, a full rebuild: every rebuild and every `--whole-every`th
        publish is timed.

Closures come from `pgo-eval --event-dump`: per closure, the scan time and every keyframe's
pose before and after it (and its raw odometry pose), from which the loop_closure_event and
pose_graph records the live modules exchange are rebuilt.

usage: reform_bench.py replay --db <src.db> --events <events.bin> --out <json> [--checks N]
       reform_bench.py ivan --db <src.db> --out <json> [--whole-every N]
       reform_bench.py report --replay <json> --ivan <json> --out <json>
"""

from __future__ import annotations

import argparse
import itertools
import json
from pathlib import Path
import resource
import time

from dimos_mls_planner import MLSPlanner
import numpy as np
from scipy.spatial.transform import Rotation

from dimos.memory.store.sqlite import SqliteStore
from dimos.navigation.global_planner.mls_planner.mls_reform_planner_native import (
    MLSReformPlannerNativeConfig,
)
from dimos.robot.unitree.go2.nav_3d_config import mls_planner_config

MODULE_DEFAULTS = MLSReformPlannerNativeConfig()

# Ivan's PGOMap rebuilds at most this often (PGOVoxelMapperConfig.rebuild_cooldown_s), and
# the patch method waited as long between corrections.
REBUILD_COOLDOWN_S = 10.0
PATCH_CELL_M = 4.0
# --checks picks warps that move at least this many voxels, this many closures apart
CHECK_MIN_VOXELS = 20_000
CHECK_SPACING = 20
DUMP_RECORD = np.dtype(
    [
        ("id", "<u4"),
        ("ts", "<f8"),
        ("pre", "<f4", 7),
        ("post", "<f4", 7),
        ("local", "<f4", 7),
    ]
)
EVENT_RECORD = np.dtype(
    [
        ("xyz", "<f4", 3),
        ("id", "<u4"),
        ("ts", "<f8"),
        ("q", "<f4", 4),
        ("dt", "<f4", 3),
        ("dq", "<f4", 4),
    ]
)
POSE_RECORD = np.dtype([("xyz", "<f4", 3), ("id", "<u4"), ("ts", "<f8"), ("q", "<f4", 4)])
assert EVENT_RECORD.itemsize == 68 and POSE_RECORD.itemsize == 40 and DUMP_RECORD.itemsize == 96


def planner(voxel_times: bool = False) -> MLSPlanner:
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
        voxel_times=voxel_times,
    )


def read_events(path: Path) -> list[tuple[float, np.ndarray]]:
    """[(scan ts, DUMP_RECORD per keyframe)] per closure."""
    raw = path.read_bytes()
    out, at = [], 0
    while at < len(raw):
        ts = float(np.frombuffer(raw, "<f8", 1, at)[0])
        n = int(np.frombuffer(raw, "<u4", 1, at + 8)[0])
        at += 12
        out.append((ts, np.frombuffer(raw, DUMP_RECORD, n, at)))
        at += n * DUMP_RECORD.itemsize
    return out


def pose_records(keyframes: np.ndarray, which: str) -> bytes:
    out = np.zeros(len(keyframes), POSE_RECORD)
    out["xyz"] = keyframes[which][:, :3]
    out["q"] = keyframes[which][:, 3:]
    out["id"], out["ts"] = keyframes["id"], keyframes["ts"]
    return out.tobytes()


def event_records(keyframes: np.ndarray) -> bytes:
    """loop_closure_event: each keyframe the closure moved, pre pose and delta = post * pre^-1."""
    pre, post = keyframes["pre"].astype(float), keyframes["post"].astype(float)
    moved = np.any(pre != post, axis=1)
    pre, post, kf = pre[moved], post[moved], keyframes[moved]
    r_pre, r_post = Rotation.from_quat(pre[:, 3:]), Rotation.from_quat(post[:, 3:])
    r_delta = r_post * r_pre.inv()
    out = np.zeros(len(kf), EVENT_RECORD)
    out["xyz"], out["q"] = pre[:, :3], pre[:, 3:]
    out["id"], out["ts"] = kf["id"], kf["ts"]
    out["dt"] = post[:, :3] - r_delta.apply(pre[:, :3])
    out["dq"] = r_delta.as_quat()
    return out.tobytes()


def correction(keyframes: np.ndarray) -> tuple[Rotation, np.ndarray]:
    """map <- odom at the newest keyframe: post * local^-1."""
    newest = keyframes[-1]
    r_post = Rotation.from_quat(newest["post"][3:].astype(float))
    r_local = Rotation.from_quat(newest["local"][3:].astype(float))
    rotation = r_post * r_local.inv()
    return rotation, newest["post"][:3].astype(float) - rotation.apply(newest["local"][:3])


def moved_cells(before: np.ndarray, after: np.ndarray, voxel: float) -> list[tuple[int, int]]:
    """4 m cells holding a keyframe that moved more than a voxel, plus their neighbors."""
    moved = after[np.linalg.norm(after - before, axis=1) > voxel]
    cells = {tuple(c) for c in np.floor(moved[:, :2] / PATCH_CELL_M).astype(int)}
    return sorted({(x + dx, y + dy) for x, y in cells for dx in (-1, 0, 1) for dy in (-1, 0, 1)})


def cpu_s() -> float:
    usage = resource.getrusage(resource.RUSAGE_SELF)
    return usage.ru_utime + usage.ru_stime


def rss_mb() -> float:
    for line in Path("/proc/self/status").read_text().splitlines():
        if line.startswith("VmRSS:"):
            return float(line.split()[1]) / 1024
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 2**20  # macOS: bytes


def timed(fn, *args, **kwargs):
    wall, cpu = time.perf_counter(), cpu_s()
    result = fn(*args, **kwargs)
    return result, (time.perf_counter() - wall) * 1e3, (cpu_s() - cpu) * 1e3


def path_length(path: np.ndarray) -> float:
    return float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())


def hausdorff(a: np.ndarray, b: np.ndarray, step: float = 0.1) -> float:
    def densify(p):
        out = [p[0]]
        for u, v in itertools.pairwise(p):
            n = max(1, int(np.linalg.norm(v - u) / step))
            out.extend(u + (v - u) * k / n for k in range(1, n + 1))
        return np.asarray(out)

    da, db = densify(a), densify(b)
    d = np.linalg.norm(da[:, None, :2] - db[None, :, :2], axis=2)
    return float(max(d.min(axis=1).max(), d.min(axis=0).max()))


def check(reform: MLSPlanner, keyframes: np.ndarray, rng: np.random.Generator) -> dict:
    """Plan between floor cells along the drive on the warped planner and on one rebuilt from its
    voxels, and compare the paths."""
    from scipy.spatial import cKDTree

    rebuilt = planner()
    _, rebuild_ms, rebuild_cpu_ms = timed(rebuilt.update_global_map, reform.voxel_map())
    surface = reform.surface_map()
    surface_a = {tuple(p) for p in np.round(surface / 0.01).astype(int)}
    surface_b = {tuple(p) for p in np.round(rebuilt.surface_map() / 0.01).astype(int)}
    # floor the drive passed over: surface cells within a meter of a keyframe, below it
    path = keyframes["post"][:, :3].astype(float)
    distance, nearest = cKDTree(path[:, :2]).query(surface[:, :2].astype(float))
    floor = surface[(distance < 1.0) & (surface[:, 2] < path[nearest, 2])]
    pairs = []
    for _ in range(16):
        i, j = rng.choice(len(floor), 2, replace=False)
        start, goal = tuple(map(float, floor[i])), tuple(map(float, floor[j]))
        a, b = reform.plan(start, goal), rebuilt.plan(start, goal)
        row = {"start": start, "goal": goal, "warped": a is not None, "rebuilt": b is not None}
        if a is not None and b is not None:
            row["len_warped"], row["len_rebuilt"] = path_length(a), path_length(b)
            row["hausdorff_m"] = hausdorff(a, b)
        pairs.append(row)
    del rebuilt
    return {
        "rebuild_ms": rebuild_ms,
        "rebuild_cpu_ms": rebuild_cpu_ms,
        "surface_cells": len(surface_a),
        "surface_symmetric_difference": len(surface_a ^ surface_b),
        "pairs": pairs,
    }


def patch_in_place(patch, mapper, applied: np.ndarray, latest: np.ndarray, obs) -> dict:
    cells = moved_cells(applied, latest[: len(applied)], mapper.voxel_size)
    wall, cpu, points_total = time.perf_counter(), cpu_s(), 0
    for x, y in cells:
        center = ((x + 0.5) * PATCH_CELL_M, (y + 0.5) * PATCH_CELL_M)
        near = latest[np.linalg.norm(latest[:, :2] - center, axis=1) < PATCH_CELL_M * 2]
        sensor_z = float(near[:, 2].mean()) if len(near) else float(obs.pose.position.z)
        z_min, z_max = sensor_z - 3.0, sensor_z + mls_planner_config.max_overhead_m
        radius = PATCH_CELL_M / np.sqrt(2)
        cell_points = mapper.local_map((*center, sensor_z), radius, z_min, z_max)
        points_total += len(cell_points)
        patch.update_region(cell_points, center, radius, z_min, z_max, sensor_z)
    return {
        "ts": obs.ts,
        "wall_ms": (time.perf_counter() - wall) * 1e3,
        "cpu_ms": (cpu_s() - cpu) * 1e3,
        "cells": len(cells),
        "points": points_total,
    }


def replay(db: Path, events_path: Path, checks_wanted: int, max_frames: int) -> dict:
    from dimos.mapping.ray_tracing.voxel_map import VoxelRayMapper
    from dimos.robot.unitree.go2.nav_3d_config import ray_tracing_config as rt

    events = read_events(events_path)
    print(f"{len(events)} closures", flush=True)
    mapper = VoxelRayMapper(voxel_size=rt.voxel_size, max_range=rt.max_range, emit_every=1)
    patch, reform = planner(), planner(voxel_times=True)
    rotation, translation = Rotation.identity(), np.zeros(3)
    rows = {"patch_region": [], "reform_region": [], "patch": [], "reform": []}
    goals: dict[int, tuple] = {}  # keyframe id -> held goal
    goal_errors, checks, rss = [], [], []
    applied_xyz, latest_xyz, last_patch = None, None, -np.inf
    next_event = 0
    rng = np.random.default_rng(0)
    with SqliteStore(path=str(db)) as store:
        for i, obs in enumerate(store.streams.lidar.order_by("ts")):
            if i == max_frames:
                break
            while next_event < len(events) and events[next_event][0] <= obs.ts:
                ts, keyframes = events[next_event]
                reform.add_keyframes(pose_records(keyframes, "pre"))
                held = [goals[k] for k in sorted(goals)]
                result, wall_ms, cpu_ms = timed(
                    reform.apply_loop_closure,
                    event_records(keyframes),
                    held,
                    blend_time_sigma_s=MODULE_DEFAULTS.reform_blend_time_sigma_s,
                    min_shift_m=MODULE_DEFAULTS.reform_min_shift_m,
                    rebuild_fraction=MODULE_DEFAULTS.reform_rebuild_fraction,
                )
                if result is not None:
                    held = result.pop("held")
                    goals = dict(zip(sorted(goals), held, strict=True))
                    rows["reform"].append(
                        {"ts": ts, "wall_ms": wall_ms, "cpu_ms": cpu_ms, **result}
                    )
                post = {int(k["id"]): k["post"][:3] for k in keyframes}
                goal_errors.append(
                    [ts]
                    + [float(np.linalg.norm(np.subtract(goals[k], post[k]))) for k in sorted(goals)]
                )
                rotation, translation = correction(keyframes)
                for k in keyframes[::500]:
                    goals.setdefault(int(k["id"]), tuple(map(float, k["post"][:3])))
                # check the first warps that move a lot, spaced out
                big = result is not None and result["voxels_moved"] >= CHECK_MIN_VOXELS
                spaced = not checks or next_event - checks[-1]["closure"] >= CHECK_SPACING
                if big and spaced and len(checks) < checks_wanted:
                    checks.append(
                        {"closure": next_event, "ts": ts, **check(reform, keyframes, rng)}
                    )
                    print(f"check {next_event}: {json.dumps(checks[-1])[:300]}", flush=True)
                latest_xyz = keyframes["post"][:, :3].astype(float)
                next_event += 1

            # patch: once per cooldown, the 4 m cells whose keyframes moved since the last patch,
            # re-applied from the ray tracer (mls_load.py region --patch-from)
            changed = latest_xyz is not None and (
                applied_xyz is None
                or len(latest_xyz) > len(applied_xyz)
                or not np.array_equal(latest_xyz[: len(applied_xyz)], applied_xyz)
            )
            if changed and obs.ts - last_patch >= REBUILD_COOLDOWN_S:
                if applied_xyz is not None:
                    rows["patch"].append(
                        patch_in_place(patch, mapper, applied_xyz, latest_xyz, obs)
                    )
                applied_xyz, last_patch = latest_xyz, obs.ts

            p = obs.pose.position
            origin = (p.x, p.y, p.z)
            mapper.add_frame_world(np.asarray(obs.data.points_f32(), np.float32), origin)
            cx, cy, radius, z_min, z_max = mapper.take_local_bounds()
            local = mapper.local_map(origin, radius, z_min, z_max)
            _, wall_ms, cpu_ms = timed(
                patch.update_region, local, (cx, cy), radius, z_min, z_max, p.z, obs.ts
            )
            rows["patch_region"].append([obs.ts, wall_ms, cpu_ms, len(local)])
            # the same cylinder in the corrected frame
            moved = rotation.apply(local).astype(np.float32) + translation.astype(np.float32)
            center = rotation.apply([cx, cy, p.z]) + translation
            dz = center[2] - p.z
            _, wall_ms, cpu_ms = timed(
                reform.update_region,
                moved,
                (float(center[0]), float(center[1])),
                radius,
                z_min + dz,
                z_max + dz,
                float(center[2]),
                obs.ts,
            )
            rows["reform_region"].append([obs.ts, wall_ms, cpu_ms, len(local)])
            if i % 500 == 0:
                rss.append([obs.ts, rss_mb(), patch.voxel_count(), reform.voxel_count()])
            if i % 2000 == 0:
                print(
                    f"frame {i} closures {next_event} voxels {reform.voxel_count()} "
                    f"rss {rss[-1][1]:.0f} MB",
                    flush=True,
                )
    return {
        "closures": len(events),
        "rows": rows,
        "goal_errors": goal_errors,
        "goal_ids": sorted(goals),
        "checks": checks,
        "rss": rss,
        "peak_rss_mb": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024,
    }


def ivan(db: Path, every: int) -> dict:
    from dimos.navigation.go2.loop_closure import module, pgo_map

    cfg = module.PGOVoxelMapperConfig(emit_every=5)  # what unitree-go2-pgo runs
    gmap = pgo_map.PGOMap(
        voxel_size=cfg.voxel_size,
        frame_id=cfg.frame_id,
        rebuild_cooldown_s=cfg.rebuild_cooldown_s,
        pgo=cfg.pgo,
    )
    mls = planner()
    publishes, rows, rss = 0, [], []
    with SqliteStore(path=str(db)) as store:
        for i, obs in enumerate(store.streams.lidar.order_by("ts")):
            rebuilt = gmap.add(obs.data, obs.pose_stamped)
            if i % 500 == 0:
                rss.append([obs.ts, rss_mb()])
            if not (rebuilt or (i + 1) % cfg.emit_every == 0):
                continue
            publishes += 1
            if publishes % every and not rebuilt:
                continue
            points = np.asarray(gmap.global_map().points_f32(), np.float32)
            _, wall_ms, cpu_ms = timed(mls.update_global_map, points)
            rows.append([obs.ts, wall_ms, cpu_ms, len(points), int(rebuilt)])
            if len(rows) % 20 == 0:
                print(
                    f"frame {i} publish {publishes} points {len(points)} mls {wall_ms:.0f} ms",
                    flush=True,
                )
    gmap.dispose()
    return {"publishes": publishes, "whole_every": every, "rows": rows, "rss": rss}


def stats(values) -> dict:
    values = np.asarray(values, float)
    if not len(values):
        return {"n": 0}
    return {
        "n": len(values),
        "median": round(float(np.median(values)), 1),
        "p90": round(float(np.percentile(values, 90)), 1),
        "max": round(float(values.max()), 1),
        "mean": round(float(values.mean()), 1),
    }


def per_hour(ms_total: float, duration_s: float) -> float:
    return round(ms_total / 1e3 * 3600 / duration_s, 1)


def report(replay_path: Path, ivan_path: Path | None) -> dict:
    """The per-approach table: MLS cost per hour of driving, latency per update, memory."""
    run = json.loads(replay_path.read_text())
    rows = run["rows"]
    region = np.asarray(rows["patch_region"], float)
    region_reform = np.asarray(rows["reform_region"], float)
    duration_s = float(region[-1, 0] - region[0, 0])
    out: dict = {"duration_s": round(duration_s), "closures": run["closures"]}

    def approach(updates: list[dict] | np.ndarray, wall: str, cpu: str) -> dict:
        wall_ms = np.asarray([u[wall] for u in updates], float)
        cpu_ms = np.asarray([u[cpu] for u in updates], float)
        return {
            "updates": len(updates),
            "cpu_s_per_hour": per_hour(cpu_ms.sum(), duration_s),
            "wall_s_per_hour": per_hour(wall_ms.sum(), duration_s),
            "wall_ms": stats(wall_ms),
            "cpu_ms": stats(cpu_ms),
        }

    out["region_updates"] = {
        "plain": approach([{"w": r[1], "c": r[2]} for r in region], "w", "c"),
        "with_voxel_times": approach([{"w": r[1], "c": r[2]} for r in region_reform], "w", "c"),
    }
    out["gsc_patch_in_place"] = approach(rows["patch"], "wall_ms", "cpu_ms")
    reform = rows["reform"]
    out["gsc_reform_warp"] = approach(reform, "wall_ms", "cpu_ms")
    out["gsc_reform_warp"]["stage_ms_median"] = (
        {
            k: round(float(np.median([r[k] for r in reform])), 1)
            for k in ("build_ms", "warp_ms", "voxel_ms", "surface_ms", "graph_ms")
        }
        if reform
        else {}
    )
    out["gsc_reform_warp"]["voxels_moved"] = stats([r["voxels_moved"] for r in reform])
    out["gsc_reform_warp"]["keyframes_moved"] = stats([r["keyframes_moved"] for r in reform])
    errors = [e for row in run["goal_errors"] for e in row[1:]]
    out["held_goal_error_m"] = stats(errors)
    out["rss_mb"] = {
        "peak": round(run["peak_rss_mb"]),
        "last": round(run["rss"][-1][1]),
        "voxels_at_end": run["rss"][-1][3],
    }
    checks = []
    for c in run["checks"]:
        both = [p for p in c["pairs"] if p["warped"] and p["rebuilt"]]
        checks.append(
            {
                "closure": c["closure"],
                "surface_cells": c["surface_cells"],
                "surface_cells_differing": c["surface_symmetric_difference"],
                "pairs": len(c["pairs"]),
                "existence_agrees": sum(p["warped"] == p["rebuilt"] for p in c["pairs"]),
                "both_planned": len(both),
                "length_ratio_max": round(
                    max((p["len_warped"] / max(p["len_rebuilt"], 1e-6) for p in both), default=1), 3
                ),
                "hausdorff_m": stats([p["hausdorff_m"] for p in both]),
                "rebuild_ms": round(c["rebuild_ms"]),
            }
        )
    out["checks"] = checks
    if ivan_path is not None:
        ivan_run = json.loads(ivan_path.read_text())
        timed_rows = np.asarray(ivan_run["rows"], float)
        rebuilds = timed_rows[timed_rows[:, 4] == 1]
        publish_rate = ivan_run["publishes"] / duration_s
        out["ivan_every_publish"] = {
            "publishes": ivan_run["publishes"],
            "timed": len(timed_rows),
            "cpu_s_per_hour_demanded": round(
                float(timed_rows[:, 2].mean()) / 1e3 * publish_rate * 3600
            ),
            "wall_ms": stats(timed_rows[:, 1]),
            "cpu_ms": stats(timed_rows[:, 2]),
            "map_points": stats(timed_rows[:, 3]),
        }
        out["ivan_rebuilds_only"] = {
            "updates": len(rebuilds),
            "cpu_s_per_hour": per_hour(rebuilds[:, 2].sum(), duration_s),
            "wall_s_per_hour": per_hour(rebuilds[:, 1].sum(), duration_s),
            "wall_ms": stats(rebuilds[:, 1]),
            "cpu_ms": stats(rebuilds[:, 2]),
        }
        out["ivan_rss_mb_peak"] = round(max(r[1] for r in ivan_run["rss"]))
    return out


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=("replay", "ivan", "report"))
    parser.add_argument("--replay", type=Path, help="report: a replay json")
    parser.add_argument("--ivan", type=Path, help="report: an ivan json")
    parser.add_argument("--db", type=Path)
    parser.add_argument("--events", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--checks", type=int, default=0, help="closures to check against a rebuild")
    parser.add_argument("--whole-every", type=int, default=40)
    parser.add_argument("--max-frames", type=int, default=-1, help="stop early, for a smoke run")
    args = parser.parse_args()
    if args.mode == "report":
        summary = report(args.replay, args.ivan)
        args.out.write_text(json.dumps(summary, indent=1))
        print(json.dumps(summary, indent=1))
        return
    started = time.perf_counter()
    if args.mode == "replay":
        result = replay(args.db, args.events, args.checks, args.max_frames)
    else:
        result = ivan(args.db, args.whole_every)
    result["runtime_s"] = time.perf_counter() - started
    args.out.write_text(json.dumps(result))
    print(f"{args.mode} done in {result['runtime_s']:.0f} s -> {args.out}", flush=True)


if __name__ == "__main__":
    main()
