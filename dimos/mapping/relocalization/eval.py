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

"""Ground-truth accuracy + latency benchmark for live relocalization.

`RelocalizationModule` only ever sees `fitness` (ICP's own self-reported
score) — as of now, that score is not verified against real pose error. This
script replays a recording's own `lidar` stream through the same
accumulation `VoxelGridMapper` uses (`VoxelGrid`, column-carved), calls the
production `relocalize()`/`track()` at the same cadence the live module
does, and compares the output against the recording's own PGO-corrected
trajectory.

The ground-truth trick: build the premap from a recording via
`dimos map global {name} --export`, and that premap sits in the PGO
`world_corrected` frame. `PoseGraph.correction_at(ts)` (from
`dimos/mapping/loop_closure/pgo.py`) returns exactly the
`world_corrected <- world_raw` transform at a given timestamp — which is
the same quantity `relocalize()` is trying to estimate live (aligning a
live raw-odometry submap onto the premap). So replaying a recording
against a premap built from *itself* gives a reference ground truth.

Usage:
    # Baseline: every attempt uses the full FPFH+RANSAC+ICP search (default).
    uv run python -m dimos.mapping.relocalization.eval go2_hongkong_office \
        --map-file go2_hongkong_office_twopass_map

    # Quick smoke run before committing to a full pass (each relocalize()
    # call can take several seconds — see relocalization.md's sample log).
    uv run python -m dimos.mapping.relocalization.eval go2_hongkong_office \
        --map-file go2_hongkong_office_twopass_map --max-attempts 5

    # Tracking mode: exercises the live SEARCHING/TRACKING gate
    # (RelocalizationModule.should_force_full_search / yaw_fan_for_travel),
    # reporting search vs. tracking attempts separately.
    uv run python -m dimos.mapping.relocalization.eval go2_hongkong_office \
        --map-file go2_hongkong_office_twopass_map --mode tracking

See `compare.sh` to run both modes and produce a comparison table + plots
in one shot, and `README.md` in this directory for the full design writeup.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any

import numpy as np
import typer

from dimos.mapping.loop_closure.pgo import PGO
from dimos.mapping.relocalization.module import (
    MAP_SUFFIX,
    MIN_LOCAL_POINTS,
    RELOC_INTERVAL,
    should_force_full_search,
    yaw_fan_for_travel,
)
from dimos.mapping.relocalization.relocalize import relocalize, track
from dimos.mapping.voxels import VoxelGrid
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.data import get_data, resolve_named_path


@dataclass
class Attempt:
    ts: float
    n_pts: int
    fitness: float
    wall_s: float
    cpu_s: float
    trans_err_m: float
    yaw_err_deg: float
    accepted: bool
    mode: str = "search"  # "search" (full relocalize()) or "tracking" (seeded track())


def _pose_error(T_est: np.ndarray, T_gt: object) -> tuple[float, float]:
    """Translation (m) and yaw (deg) error between relocalize()'s estimate
    and the ground-truth world_raw -> world_corrected transform at this ts."""
    t_est = T_est[:3, 3]
    t_gt = T_gt.translation.to_numpy()  # type: ignore[attr-defined]
    trans_err = float(np.linalg.norm(t_est - t_gt))

    R_est = T_est[:3, :3]
    R_gt = T_gt.rotation.to_rotation_matrix()  # type: ignore[attr-defined]
    R_diff = R_est.T @ R_gt
    yaw_err = float(np.degrees(np.arccos(np.clip((np.trace(R_diff) - 1) / 2, -1.0, 1.0))))
    return trans_err, yaw_err


def _odom_at(odom_stream: Any, ts: float) -> Any | None:
    """Nearest odom PoseStamped at `ts`, or None if none nearby."""
    try:
        obs = odom_stream.at(ts, tolerance=2.0).first()
    except Exception:
        return None
    return obs.data


def run_eval(
    recording: str,
    map_file: str,
    *,
    sample_interval: float = RELOC_INTERVAL,
    fitness_threshold: float = 0.45,
    tracking_fitness_threshold: float = 0.5,
    voxel_size: float = 0.05,
    max_attempts: int | None = None,
    mode: str = "global",
) -> list[Attempt]:
    """Replay `recording` against `map_file`, returning one `Attempt` per
    relocalize()/track() call. `map_file` should be a premap built from
    `recording` itself (`dimos map global {recording} --export`) so the
    ground-truth trick above holds.

    `mode="global"` (default, the original baseline behavior of this script)
    benchmarks the untouched full search on every attempt. `mode="tracking"`
    exercises the exact SEARCHING/TRACKING gate the live `RelocalizationModule`
    runs, via the same `should_force_full_search`/`yaw_fan_for_travel`
    functions it imports its logic from -- see those functions' docstrings
    in `module.py` for the fan-sizing design history -- so a single run shows
    both the bootstrap full-search cost and steady-state tracking cost.
    """
    if mode not in ("global", "tracking"):
        raise ValueError(f"mode must be 'global' or 'tracking', got {mode!r}")

    db_path = get_data(f"{recording}.db")
    premap_path = resolve_named_path(map_file, MAP_SUFFIX)
    premap = PointCloud2.lcm_decode(premap_path.read_bytes())

    attempts: list[Attempt] = []
    with SqliteStore(path=str(db_path)) as store:
        lidar = store.streams.lidar
        odom_stream = store.streams.odom

        print("running PGO for ground truth...")
        pgo_wall0 = time.monotonic()
        graph = lidar.transform(PGO()).last().data
        print(
            f"PGO ground truth ready in {time.monotonic() - pgo_wall0:.1f}s "
            f"({len(graph.keyframes)} keyframes, {len(graph.loops)} loop closures)"
        )
        if not graph.keyframes:
            raise SystemExit(
                f"PGO produced no keyframes for {recording!r} — recording too short/static?"
            )

        # Tracking-mode state -- mirrors RelocalizationModule's instance
        # state, kept as local variables since this is a plain loop, not an
        # rx pipeline. Unused when mode="global".
        last_good: tuple[np.ndarray, float, float] | None = None
        miss_count = 0
        last_full_search_ts = 0.0
        last_full_search_pos: np.ndarray | None = None

        grid = VoxelGrid(voxel_size=voxel_size, carve_columns=True, show_startup_log=False)
        try:
            last_attempt_ts = float("-inf")
            for obs in lidar:
                grid.add_frame(obs.data)
                if obs.ts - last_attempt_ts < sample_interval:
                    continue

                local = grid.get_global_pointcloud2()
                if len(local) < MIN_LOCAL_POINTS:
                    continue
                last_attempt_ts = obs.ts

                latest_odom = _odom_at(odom_stream, obs.ts)
                latest_odom_xy = (
                    np.array([latest_odom.position.x, latest_odom.position.y])
                    if latest_odom is not None
                    else None
                )
                use_tracking = (
                    mode == "tracking"
                    and last_good is not None
                    and not should_force_full_search(
                        obs.ts,
                        last_good,
                        miss_count,
                        last_full_search_ts,
                        last_full_search_pos,
                        latest_odom_xy,
                    )
                )

                cpu0, wall0 = time.process_time(), time.monotonic()
                if use_tracking:
                    assert last_good is not None
                    seed_T, _, _ = last_good
                    # Fan seed/width driven by distance/time since the last FULL
                    # SEARCH, not since last_good -- see module.py's
                    # yaw_fan_for_travel docstring for why (design history of
                    # two prior regressions this avoids).
                    distance_since_full_search = 0.0
                    if latest_odom_xy is not None and last_full_search_pos is not None:
                        distance_since_full_search = float(
                            np.linalg.norm(latest_odom_xy - last_full_search_pos)
                        )
                    fan = yaw_fan_for_travel(
                        distance_since_full_search, obs.ts - last_full_search_ts
                    )
                    T, fitness = track(
                        premap.pointcloud, local.pointcloud, seed_T, yaw_fan_deg=fan
                    )
                    attempt_mode = "tracking"
                    threshold = tracking_fitness_threshold
                else:
                    # Original baseline path -- unchanged, still what runs on every
                    # attempt when mode="global" (the default).
                    T, fitness = relocalize(premap.pointcloud, local.pointcloud)
                    attempt_mode = "search"
                    threshold = fitness_threshold
                wall_s = time.monotonic() - wall0
                cpu_s = time.process_time() - cpu0

                accepted = fitness >= threshold
                if mode == "tracking":
                    if accepted:
                        miss_count = 0
                        last_good = (T, fitness, obs.ts)
                        if attempt_mode == "search":
                            last_full_search_ts = obs.ts
                            last_full_search_pos = latest_odom_xy
                    elif attempt_mode == "tracking":
                        miss_count += 1

                trans_err, yaw_err = _pose_error(T, graph.correction_at(obs.ts))
                a = Attempt(
                    obs.ts,
                    len(local),
                    fitness,
                    wall_s,
                    cpu_s,
                    trans_err,
                    yaw_err,
                    accepted,
                    mode=attempt_mode,
                )
                attempts.append(a)
                print(
                    f"t={a.ts:8.2f}  n_pts={a.n_pts:7d}  mode={a.mode:8s}  fitness={a.fitness:.3f}  "
                    f"wall={a.wall_s:5.2f}s  cpu={a.cpu_s:5.2f}s  "
                    f"trans_err={a.trans_err_m:.3f}m  yaw_err={a.yaw_err_deg:6.2f}deg  "
                    f"{'ACCEPT' if a.accepted else 'reject'}"
                )
                if max_attempts is not None and len(attempts) >= max_attempts:
                    break
        finally:
            grid.dispose()
    return attempts


def _summarize_group(label: str, group: list[Attempt]) -> None:
    if not group:
        return
    accepted = [a for a in group if a.accepted]
    wall = [a.wall_s for a in group]
    cpu = [a.cpu_s for a in group]

    print()
    print(f"-- {label} (n={len(group)}) --")
    print(f"accepted={len(accepted)} ({100 * len(accepted) / len(group):.0f}%)")
    print(f"latency wall   median={np.median(wall):.2f}s  p90={np.percentile(wall, 90):.2f}s")
    print(f"latency cpu    median={np.median(cpu):.2f}s  p90={np.percentile(cpu, 90):.2f}s")

    if accepted:
        trans = [a.trans_err_m for a in accepted]
        yaw = [a.yaw_err_deg for a in accepted]
        print(f"trans_err (accepted)  median={np.median(trans):.3f}m  p90={np.percentile(trans, 90):.3f}m")
        print(f"yaw_err   (accepted)  median={np.median(yaw):.2f}deg  p90={np.percentile(yaw, 90):.2f}deg")

    false_accepts = [a for a in accepted if a.trans_err_m > 0.5]
    if false_accepts:
        print(
            f"WARNING: {len(false_accepts)}/{len(accepted)} accepted attempts have >0.5m "
            "ground-truth error — fitness_threshold alone did not catch these"
        )


def _summarize(attempts: list[Attempt]) -> None:
    if not attempts:
        print("no attempts recorded (recording too short, or MIN_LOCAL_POINTS never reached)")
        return

    _summarize_group("overall", attempts)
    search = [a for a in attempts if a.mode == "search"]
    tracking = [a for a in attempts if a.mode == "tracking"]
    if search and tracking:
        _summarize_group("search (bootstrap / reacquire / sanity-check)", search)
        _summarize_group("tracking (seeded, cheap)", tracking)


def main(
    recording: str = typer.Argument(..., help="Recording stem, e.g. go2_hongkong_office"),
    map_file: str = typer.Option(
        ..., "--map-file", help="Premap stem, built FROM `recording` itself (see module docstring)"
    ),
    mode: str = typer.Option(
        "global",
        "--mode",
        help="'global': every attempt uses the full search (original baseline). "
        "'tracking': exercises the live SEARCHING/TRACKING gate.",
    ),
    sample_interval: float = typer.Option(
        RELOC_INTERVAL, "--sample-interval", help="Seconds between relocalize() attempts"
    ),
    fitness_threshold: float = typer.Option(0.45, "--fitness-threshold"),
    tracking_fitness_threshold: float = typer.Option(
        0.5, "--tracking-fitness-threshold", help="Only used when --mode=tracking"
    ),
    voxel_size: float = typer.Option(0.05, "--voxel-size"),
    max_attempts: int = typer.Option(
        0, "--max-attempts", help="Stop after N attempts (0 = run the whole recording)"
    ),
) -> None:
    attempts = run_eval(
        recording,
        map_file,
        sample_interval=sample_interval,
        fitness_threshold=fitness_threshold,
        tracking_fitness_threshold=tracking_fitness_threshold,
        voxel_size=voxel_size,
        max_attempts=max_attempts or None,
        mode=mode,
    )
    _summarize(attempts)


if __name__ == "__main__":
    typer.run(main)
