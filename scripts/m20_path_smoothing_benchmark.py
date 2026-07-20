#!/usr/bin/env python3
"""Deterministic benchmark for constrained M20 path smoothing."""

import argparse
import csv
import json
from pathlib import Path as FilePath
import resource
from statistics import median
from time import perf_counter, process_time
from unittest.mock import patch

import numpy as np

from dimos.mapping.occupancy.path_resampling import (
    ConstrainedPathSmoothingConfig,
    constrained_smooth_resample_path,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path

LENGTHS_M = (2.0, 5.0, 10.0, 20.0, 40.0)
PHASE_FIELDS = (
    "raw_validation_ms",
    "reference_costs_ms",
    "smoothing_loop_ms",
    "raw_resample_metrics_ms",
    "candidate_1_0_ms",
    "candidate_0_5_ms",
    "candidate_0_25_ms",
    "candidate_0_125_ms",
    "final_path_message_ms",
    "optimizer_total_ms",
)


def _distribution(values: list[float]) -> dict[str, float]:
    return {
        "p50": median(values),
        "p95": float(np.percentile(values, 95)),
        "max": max(values),
    }


def _fixture(length_m: float, seed: int) -> tuple[Path, Pose, OccupancyGrid]:
    rng = np.random.default_rng(seed)
    grid = rng.integers(0, 16, size=(220, 1000), dtype=np.int8)
    grid[:2, :] = CostValues.OCCUPIED
    grid[-2:, :] = CostValues.OCCUPIED
    grid[:, :2] = CostValues.OCCUPIED
    grid[:, -2:] = CostValues.OCCUPIED
    grid[35:42, 120:150] = CostValues.UNKNOWN
    grid[175:182, 650:690] = CostValues.UNKNOWN
    costmap = OccupancyGrid(grid=grid, resolution=0.05)

    point_count = int(length_m * 18.55) + 1
    xs = np.linspace(2.0, 2.0 + length_m, point_count)
    phase = np.linspace(0.0, length_m * 2.1, point_count)
    ys = 5.5 + 0.035 * np.sin(phase) + 0.018 * np.sin(phase * 2.7)
    path = Path(
        frame_id="world",
        poses=[
            PoseStamped(frame_id="world", position=[float(x), float(y), 0.0])
            for x, y in zip(xs, ys, strict=True)
        ],
    )
    return path, Pose(position=path.poses[-1].position), costmap


def _run_case(
    length_m: float,
    warmups: int,
    repetitions: int,
    seed: int,
) -> dict[str, object]:
    path, goal, costmap = _fixture(length_m, seed)
    config = ConstrainedPathSmoothingConfig()
    runs: list[dict[str, float | int]] = []
    wall_times: list[float] = []
    cpu_times: list[float] = []
    output_points: set[int] = set()

    def quiet(*args: object, **kwargs: object) -> None:
        return None

    with (
        patch("dimos.mapping.occupancy.path_resampling.logger.info", quiet),
        patch("dimos.mapping.occupancy.path_resampling.logger.warning", quiet),
    ):
        for _ in range(warmups):
            constrained_smooth_resample_path(path, goal, costmap, config)
        for _ in range(repetitions):
            timing: dict[str, float | int] = {}
            wall_started = perf_counter()
            cpu_started = process_time()
            result = constrained_smooth_resample_path(path, goal, costmap, config, timing)
            cpu_times.append((process_time() - cpu_started) * 1000)
            wall_times.append((perf_counter() - wall_started) * 1000)
            output_points.add(len(result.poses))
            runs.append(timing)

    return {
        "length_m": length_m,
        "raw_points": len(path.poses),
        "output_points": sorted(output_points),
        "wall_ms": _distribution(wall_times),
        "cpu_ms": _distribution(cpu_times),
        "peak_rss_kib": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        "phases_ms": {
            field: _distribution([float(run[field]) for run in runs]) for field in PHASE_FIELDS
        },
    }


def _write_csv(path: FilePath, cases: list[dict[str, object]]) -> None:
    fields = (
        "length_m",
        "raw_points",
        "wall_p50_ms",
        "wall_p95_ms",
        "wall_max_ms",
        "cpu_p50_ms",
        "cpu_p95_ms",
        "cpu_max_ms",
        "peak_rss_kib",
    )
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for case in cases:
            wall = case["wall_ms"]
            cpu = case["cpu_ms"]
            assert isinstance(wall, dict) and isinstance(cpu, dict)
            writer.writerow(
                {
                    "length_m": case["length_m"],
                    "raw_points": case["raw_points"],
                    "wall_p50_ms": wall["p50"],
                    "wall_p95_ms": wall["p95"],
                    "wall_max_ms": wall["max"],
                    "cpu_p50_ms": cpu["p50"],
                    "cpu_p95_ms": cpu["p95"],
                    "cpu_max_ms": cpu["max"],
                    "peak_rss_kib": case["peak_rss_kib"],
                }
            )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--warmups", type=int, default=5)
    parser.add_argument("--repetitions", type=int, default=30)
    parser.add_argument("--seed", type=int, default=20260717)
    parser.add_argument("--output-dir", type=FilePath, required=True)
    args = parser.parse_args()
    if args.warmups < 0 or args.repetitions < 1:
        parser.error("warmups must be non-negative and repetitions must be positive")

    cases = [_run_case(length, args.warmups, args.repetitions, args.seed) for length in LENGTHS_M]
    result = {
        "protocol": {
            "lengths_m": LENGTHS_M,
            "warmups": args.warmups,
            "repetitions": args.repetitions,
            "seed": args.seed,
            "costmap_shape": [220, 1000],
            "costmap_resolution_m": 0.05,
            "resample_spacing_m": 0.1,
            "collision_sample_spacing_m": 0.05,
        },
        "cases": cases,
    }
    args.output_dir.mkdir(parents=True, exist_ok=True)
    json_path = args.output_dir / "benchmark.json"
    csv_path = args.output_dir / "benchmark.csv"
    json_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    _write_csv(csv_path, cases)
    print(json.dumps({"json": str(json_path), "csv": str(csv_path)}))


if __name__ == "__main__":
    main()
