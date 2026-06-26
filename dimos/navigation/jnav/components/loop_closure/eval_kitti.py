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

"""Evaluate one PGO on a KITTI db with the official KITTI odometry error.

Parallels eval.py, but for the kitti_to_db.py recordings. Runs the module over
the db's drifty ICP odometry (fastlio_odometry) + registered scans (fastlio_lidar),
reads the ground truth from the db's gt_odometry stream, and reports translational
(%) and rotational (deg/m) error — the leaderboard metric — for the corrected
trajectory and the raw-odometry baseline.

Usage:
    uv run python dimos/navigation/jnav/components/loop_closure/eval_kitti.py \
        --db-path ~/datasets/kitti/kitti_seq07/mem2.db \
        --module-path dimos/navigation/jnav/components/loop_closure/gsc_pgo/module.py \
        [--module-name PGO] [--pgo-config-json '{...}']
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.navigation.jnav.components.loop_closure.eval import run_module_graph
from dimos.navigation.jnav.utils.kitti import kitti_odometry_error
from dimos.navigation.jnav.utils.module_loading import (
    filter_config_for_module,
    load_module_class,
)
from dimos.navigation.jnav.utils.recording_db import iterate_stream

DEFAULT_CONFIG = {"use_scan_context": True, "global_map_publish_rate": 0}

# Stream names produced by kitti_to_db (overridable on the CLI).
DEFAULT_LIDAR_STREAM = "fastlio_lidar"
DEFAULT_ODOM_STREAM = "fastlio_odometry"
DEFAULT_GT_STREAM = "gt_odometry"


def poses_from_stream(db_path: Path, stream: str) -> tuple[list[np.ndarray], list[float]]:
    """Read an Odometry stream as (4x4 poses, timestamps)."""
    poses: list[np.ndarray] = []
    times: list[float] = []
    for timestamp, message in iterate_stream(db_path, stream):
        orientation = message.pose.orientation
        position = message.pose.position
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_quat(
            [orientation.x, orientation.y, orientation.z, orientation.w]
        ).as_matrix()
        transform[:3, 3] = [position.x, position.y, position.z]
        poses.append(transform)
        times.append(timestamp)
    return poses, times


def _gt_at(
    gt_poses: list[np.ndarray], gt_times: list[float], query: list[float]
) -> list[np.ndarray]:
    times = np.asarray(gt_times)
    return [gt_poses[int(np.argmin(np.abs(times - t)))] for t in query]


def corrected_trajectory(
    db_path: Path,
    module_path: Path,
    module_name: str,
    config: dict[str, Any],
    *,
    lidar_stream: str,
    odom_stream: str,
) -> tuple[list[Any], list[Any], Any]:
    module_class = load_module_class(module_path, module_name)
    config = filter_config_for_module(module_class, config)
    graph, closures, _ = run_module_graph(
        db_path,
        module_class,
        config,
        lidar_stream=lidar_stream,
        odom_stream=odom_stream,
        lockstep=True,
    )
    poses, times = [], []
    for node in graph:
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_quat(node[4:8]).as_matrix()
        transform[:3, 3] = node[1:4]
        poses.append(transform)
        times.append(node[0])
    return poses, times, closures


def evaluate(
    db_path: Path,
    module_path: Path,
    module_name: str,
    config: dict[str, Any],
    results_suffix: str = "",
    *,
    lidar_stream: str = DEFAULT_LIDAR_STREAM,
    odom_stream: str = DEFAULT_ODOM_STREAM,
    gt_stream: str = DEFAULT_GT_STREAM,
) -> dict[str, Any]:
    gt_poses, gt_times = poses_from_stream(db_path, gt_stream)
    odom_poses, _ = poses_from_stream(db_path, odom_stream)
    if not gt_poses:
        raise SystemExit(f"no {gt_stream!r} stream in {db_path}")
    baseline = kitti_odometry_error(odom_poses, gt_poses)

    corrected, corrected_times, closures = corrected_trajectory(
        db_path,
        module_path,
        module_name,
        config,
        lidar_stream=lidar_stream,
        odom_stream=odom_stream,
    )
    error = kitti_odometry_error(corrected, _gt_at(gt_poses, gt_times, corrected_times))

    print(
        f"raw odometry:      {baseline['translational_percent']:.2f}% transl / "
        f"{baseline['rotational_deg_per_m']:.4f} deg/m"
    )
    print(
        f"after {module_path.parent.name}: {error['translational_percent']:.2f}% transl / "
        f"{error['rotational_deg_per_m']:.4f} deg/m  ({closures} closures)"
    )

    module_key = module_path.parent.name + (f".{results_suffix}" if results_suffix else "")
    summary = {
        "db": str(db_path),
        "module": module_key,
        "scores": {
            "translational_percent": error["translational_percent"],
            "rotational_deg_per_m": error["rotational_deg_per_m"],
            "baseline_translational_percent": baseline["translational_percent"],
            "baseline_rotational_deg_per_m": baseline["rotational_deg_per_m"],
            "closures": closures,
            "keyframes": len(corrected),
        },
    }
    recording = db_path.parent.name
    out_dir = Path(__file__).resolve().parent / "eval_results" / f"{recording}__{module_key}"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / "kitti_summary.json").write_text(json.dumps(summary, indent=2))
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--db-path", type=Path, required=True)
    parser.add_argument("--module-path", type=Path, required=True)
    parser.add_argument("--module-name", default="PGO")
    parser.add_argument("--pgo-config-json", default="")
    parser.add_argument("--results-suffix", default="")
    parser.add_argument("--lidar-stream", default=DEFAULT_LIDAR_STREAM)
    parser.add_argument("--odom-stream", default=DEFAULT_ODOM_STREAM)
    parser.add_argument("--gt-stream", default=DEFAULT_GT_STREAM)
    args = parser.parse_args()
    config = dict(DEFAULT_CONFIG)
    if args.pgo_config_json:
        config.update(json.loads(args.pgo_config_json))
    evaluate(
        args.db_path.expanduser(),
        args.module_path,
        args.module_name,
        config,
        args.results_suffix,
        lidar_stream=args.lidar_stream,
        odom_stream=args.odom_stream,
        gt_stream=args.gt_stream,
    )


if __name__ == "__main__":
    main()
