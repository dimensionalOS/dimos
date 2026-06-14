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

"""Benchmark PGO on KITTI recordings using the official KITTI odometry error.

Each ``<sequences-root>/*/mem2.db`` is replayed through PGO with the lockstep
harness: the recording's drifty ICP odometry (``fastlio_odometry``) + registered
scans (``fastlio_lidar``) are fed scan-by-scan, each scan paced on PGO's
``corrected_odometry`` ack. The corrected pose graph is scored against the db's
``gt_odometry`` with the KITTI leaderboard metric — translational error (%) and
rotational error (deg/m) — alongside the raw-odometry baseline.

Per-sequence summaries are written as JSON under ``--output-dir``.

Usage:
    uv run python -m dimos.navigation.nav_stack.modules.pgo.benchmark_kitti \\
        [--sequences-root ~/datasets/kitti/sequences] [--smoke] [--max-scans N]
"""

from __future__ import annotations

import argparse
import importlib
import json
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from dimos.navigation.nav_stack.modules.pgo.lockstep_harness import (
    iterate_recording_stream,
    run_pgo_graph,
)
from dimos.navigation.nav_stack.modules.pgo.pgo import PGO
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DEFAULT_SEQUENCES_ROOT = "~/datasets/kitti/sequences"
RESULTS_DIR = Path(__file__).resolve().parent / "kitti_results"

# Stream names produced by kitti_to_db (overridable on the CLI).
DEFAULT_LIDAR_STREAM = "fastlio_lidar"
DEFAULT_ODOM_STREAM = "fastlio_odometry"
DEFAULT_GT_STREAM = "gt_odometry"

# --smoke caps each sequence to a quick liveness-sized prefix.
SMOKE_MAX_SCANS = 300

# Tuned PGO config for KITTI urban sequences.
DEFAULT_PGO_KWARGS: dict[str, Any] = {
    "use_scan_context": True,
    "scan_context_match_threshold": 0.4,
    # ICP fitness on KITTI urban submaps has a 5-50 m² noise floor for TRUE
    # loops; the 0.15 default rejects nearly all of them. 50.0 sits at the top
    # of that floor: true revisits pass, egregious misalignments are rejected.
    "loop_score_thresh": 50.0,
    "loop_search_radius": 1.0,
    "loop_candidate_max_distance_m": 10.0,
    "loop_time_thresh": 50.0,
    # No closure cooldown: the benchmark scores the whole trajectory, and on a
    # fast platform even a small cooldown suppresses keyframes for tens of
    # metres after each accepted loop. Cooldown is a deploy anti-spam knob.
    "min_loop_detect_duration": 0.0,
    "key_pose_delta_trans": 0.5,
    # CMU's 0.1 m + half=5 overflows PCL VoxelGrid int32 on KITTI.
    "submap_resolution": 0.5,
    "loop_submap_half_range": 2,
    # The global map is a big cloud the benchmark never reads; keep it off the bus.
    "global_map_publish_rate": 0.001,
    "drain_stale_scans": False,
    # KITTI urban scenes are structured; the open-grass loop-acceptance gates
    # don't apply and would only suppress real closures.
    "loop_min_degeneracy": 0.0,
    "loop_min_occupancy": 0,
}

# KITTI leaderboard metric (devkit evaluate_odometry).
LENGTHS = (100.0, 200.0, 300.0, 400.0, 500.0, 600.0, 700.0, 800.0)
STEP = 10


def _trajectory_distances(poses: list[np.ndarray]) -> list[float]:
    distances = [0.0]
    for index in range(1, len(poses)):
        delta = poses[index][:3, 3] - poses[index - 1][:3, 3]
        distances.append(distances[-1] + float(np.linalg.norm(delta)))
    return distances


def _last_frame_for_length(distances: list[float], first: int, length: float) -> int:
    target = distances[first] + length
    for index in range(first, len(distances)):
        if distances[index] >= target:
            return index
    return -1


def _rotation_error(pose_error: np.ndarray) -> float:
    trace = pose_error[0, 0] + pose_error[1, 1] + pose_error[2, 2]
    return float(np.arccos(np.clip((trace - 1.0) / 2.0, -1.0, 1.0)))


def _translation_error(pose_error: np.ndarray) -> float:
    return float(np.linalg.norm(pose_error[:3, 3]))


def kitti_odometry_error(
    estimated: list[np.ndarray], ground_truth: list[np.ndarray]
) -> dict[str, float]:
    """Average translational (%) and rotational (deg/m) error, devkit-style."""
    count = min(len(estimated), len(ground_truth))
    estimated, ground_truth = estimated[:count], ground_truth[:count]
    distances = _trajectory_distances(ground_truth)

    translational_errors: list[float] = []
    rotational_errors: list[float] = []
    for first in range(0, count, STEP):
        for length in LENGTHS:
            last = _last_frame_for_length(distances, first, length)
            if last < 0:
                continue
            gt_delta = np.linalg.inv(ground_truth[first]) @ ground_truth[last]
            estimated_delta = np.linalg.inv(estimated[first]) @ estimated[last]
            pose_error = np.linalg.inv(gt_delta) @ estimated_delta
            translational_errors.append(_translation_error(pose_error) / length)
            rotational_errors.append(_rotation_error(pose_error) / length)

    if not translational_errors:
        return {"translational_percent": float("nan"), "rotational_deg_per_m": float("nan")}
    return {
        "translational_percent": float(np.mean(translational_errors)) * 100.0,
        "rotational_deg_per_m": float(np.degrees(np.mean(rotational_errors))),
    }


def poses_from_stream(db_path: Path, stream: str) -> tuple[list[np.ndarray], list[float]]:
    """Read an Odometry stream as (4x4 body->world poses, timestamps)."""
    poses: list[np.ndarray] = []
    times: list[float] = []
    for timestamp, message in iterate_recording_stream(db_path, stream):
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
    gt_poses: list[np.ndarray], gt_times: list[float], query_times: list[float]
) -> list[np.ndarray]:
    times = np.asarray(gt_times)
    return [gt_poses[int(np.argmin(np.abs(times - t)))] for t in query_times]


def _graph_to_poses(graph: list[list[float]]) -> tuple[list[np.ndarray], list[float]]:
    poses: list[np.ndarray] = []
    times: list[float] = []
    for node in graph:
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_quat(node[4:8]).as_matrix()
        transform[:3, 3] = node[1:4]
        poses.append(transform)
        times.append(node[0])
    return poses, times


def evaluate_sequence(
    db_path: Path,
    *,
    pgo_kwargs: dict[str, Any],
    max_scans: int | None,
    lidar_stream: str,
    odom_stream: str,
    gt_stream: str,
) -> dict[str, Any]:
    """Replay one sequence through PGO and score it against ground truth."""
    gt_poses, gt_times = poses_from_stream(db_path, gt_stream)
    if not gt_poses:
        raise SystemExit(f"no {gt_stream!r} stream in {db_path}")
    odom_poses, _ = poses_from_stream(db_path, odom_stream)
    baseline = kitti_odometry_error(odom_poses, gt_poses)

    result = run_pgo_graph(
        PGO.blueprint(**pgo_kwargs),
        db_path,
        lidar_stream=lidar_stream,
        odometry_stream=odom_stream,
        max_scans=max_scans,
    )
    if result["replay_error"] is not None:
        raise RuntimeError(f"replay failed on {db_path}: {result['replay_error']}")
    corrected, corrected_times = _graph_to_poses(result["graph"])
    if not corrected:
        raise RuntimeError(f"{db_path}: PGO produced an empty pose graph")
    error = kitti_odometry_error(corrected, _gt_at(gt_poses, gt_times, corrected_times))

    return {
        "db": str(db_path),
        "sequence": db_path.parent.name,
        "scores": {
            "translational_percent": error["translational_percent"],
            "rotational_deg_per_m": error["rotational_deg_per_m"],
            "baseline_translational_percent": baseline["translational_percent"],
            "baseline_rotational_deg_per_m": baseline["rotational_deg_per_m"],
            "closures": result["closures"],
            "keyframes": result["keyframes"],
            "scans_skipped": result["scans_skipped"],
        },
    }


def discover_sequences(sequences_root: Path, selected: list[str] | None) -> list[Path]:
    """All ``<root>/*/mem2.db`` (optionally filtered to named sequence dirs)."""
    dbs = sorted(sequences_root.glob("*/mem2.db"))
    if selected:
        wanted = set(selected)
        dbs = [db for db in dbs if db.parent.name in wanted]
    return dbs


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sequences-root", type=Path, default=Path(DEFAULT_SEQUENCES_ROOT))
    parser.add_argument(
        "--sequences",
        default="",
        help="comma-separated sequence dir names to run (default: all under root)",
    )
    parser.add_argument(
        "--smoke",
        action="store_true",
        help=f"quick liveness mode: cap each sequence to {SMOKE_MAX_SCANS} scans",
    )
    parser.add_argument("--max-scans", type=int, default=None)
    parser.add_argument("--output-dir", type=Path, default=RESULTS_DIR)
    parser.add_argument("--pgo-config-json", default="", help="inline JSON of PGO config overrides")
    parser.add_argument("--lidar-stream", default=DEFAULT_LIDAR_STREAM)
    parser.add_argument("--odom-stream", default=DEFAULT_ODOM_STREAM)
    parser.add_argument("--gt-stream", default=DEFAULT_GT_STREAM)
    args = parser.parse_args()

    sequences_root = args.sequences_root.expanduser()
    selected = [name.strip() for name in args.sequences.split(",") if name.strip()]
    dbs = discover_sequences(sequences_root, selected)
    if not dbs:
        raise SystemExit(f"no */mem2.db under {sequences_root}")

    max_scans = args.max_scans
    if max_scans is None and args.smoke:
        max_scans = SMOKE_MAX_SCANS

    pgo_kwargs = dict(DEFAULT_PGO_KWARGS)
    if args.pgo_config_json:
        pgo_kwargs.update(json.loads(args.pgo_config_json))

    output_dir = args.output_dir.expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)

    summaries: list[dict[str, Any]] = []
    for db_path in dbs:
        sequence = db_path.parent.name
        logger.info(f"=== {sequence}: replaying through PGO (max_scans={max_scans}) ===")
        summary = evaluate_sequence(
            db_path,
            pgo_kwargs=pgo_kwargs,
            max_scans=max_scans,
            lidar_stream=args.lidar_stream,
            odom_stream=args.odom_stream,
            gt_stream=args.gt_stream,
        )
        scores = summary["scores"]
        print(
            f"{sequence}: raw {scores['baseline_translational_percent']:.2f}%/"
            f"{scores['baseline_rotational_deg_per_m']:.4f} -> corrected "
            f"{scores['translational_percent']:.2f}%/{scores['rotational_deg_per_m']:.4f} "
            f"({scores['closures']} closures, {scores['keyframes']} keyframes)"
        )
        (output_dir / f"{sequence}.json").write_text(json.dumps(summary, indent=2) + "\n")
        summaries.append(summary)

    (output_dir / "summary.json").write_text(json.dumps(summaries, indent=2) + "\n")
    print(f"\n{len(summaries)} sequence(s) -> {output_dir}")


if __name__ == "__main__":
    # Re-import under the canonical dotted name so the harness module classes
    # deploy into workers with a picklable __module__.
    importlib.import_module("dimos.navigation.nav_stack.modules.pgo.benchmark_kitti").main()
