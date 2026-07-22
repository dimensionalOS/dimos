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

"""Evaluate a loop-closure module against a recording.

The raw-baseline robot trajectory is the odometry the module is actually fed
(``--odom-stream``, e.g. ``fastlio_odometry``), read straight from its stored
poses. Scoring never consults a separate "answer key" (the tf tree, pointlio, or
any other odom source): every metric is the SAME odometry with vs without the
module's correction, so a positive score means the module improved the map it was
given rather than measuring the gap between two unrelated trajectories.

Two ground-truth-free scores, before vs after correction:
  * April-tag agreement — a fixed tag re-seen along the run should map to one
    world position; the spread of its per-visit robot positions measures drift.
    A tag sighting is placed at the robot's baseline pose nearest its time.
  * Lidar-voxel agreement — re-anchoring the registered scans onto the
    corrected trajectory should collapse double walls, so the corrected map
    should occupy FEWER voxels than the raw one.

Pipeline:
  1. April tags: read the db's `april_tags` stream (ts + marker_id only), or
     detect them with sane defaults (medoid, blur/reproj/size/distance gates).
  2. Raw agreement over the raw odometry.
  3. Replay lidar + odom through the module (loaded dynamically from
     --module-path/--module-name), capture its optimized pose graph.
  4. Corrected agreement + voxel agreement, written to
     eval_results/<recording>__<module>/summary.json (and an eval.rrd with the
     raw + corrected trajectories when --with-rrd true).

Usage:
    uv run python dimos/navigation/jnav/components/loop_closure/eval.py \\
        --db-path ~/datasets/RECORDINGS_DIR/2026-06-04_12-56pm-PST/mem2.db \\
        --odom-stream fastlio_odometry \\
        --camera-stream color_image \\
        --camera-intrinsics-json-path \\
            ~/datasets/RECORDINGS_DIR/2026-06-04_12-56pm-PST/camera_intrinsics.json \\
        --module-path dimos/navigation/jnav/components/loop_closure/gsc_pgo/module.py \\
        --module-name PGO \\
        --pgo-config-json '{"use_scan_context": true}' \\
        --with-rrd true
"""

from __future__ import annotations

import argparse
from collections.abc import Iterable, Iterator
import json
from pathlib import Path
import time
from typing import Any

import numpy as np

from dimos.navigation.jnav.components.loop_closure.gsc_pgo.utils.replay import run_module_graph
from dimos.navigation.jnav.utils.apriltags import (
    VISIT_GAP_S,
    AgreementReport,
    agreement_improvement,
    agreement_report,
    detect_apriltags,
    load_intrinsics_json,
    load_or_detect_sightings,
    paired_tag_visit_positions,
)
from dimos.navigation.jnav.utils.module_loading import (
    filter_config_for_module,
    load_module_class,
)
from dimos.navigation.jnav.utils.recording_db import (
    ODOM_MATCH_TOLERANCE_S,
    iterate_stream,
    list_streams,
    payload_pose,
    store,
    stream_count,
)
from dimos.navigation.jnav.utils.trajectory_metrics import (
    drifted_lookup,
    graph_lookup,
    has_drift,
    lidar_voxel_agreement,
    pose7_lookup,
    trajectory_lookup,
    trajectory_recovery_error,
    write_trajectory_rrd,
)

RESULTS_DIR = Path(__file__).resolve().parent / "eval_results"
APRIL_TAGS_STREAM = "april_tags"

# Cap replayed scans fed to voxel agreement so the map fits in memory.
VOXEL_MAX_SCANS = 300

# Bump to invalidate every cached cell (scoring/replay semantics changed).
# v3: raw baseline is the fed odom stream's stored poses again (reverting the v2
# tf baseline). The tf tree's world->base_link is built by add_tf from whatever
# odom it prefers (pointlio), which diverges from the fastlio the benchmark feeds
# the module, so v2 scored the correction against an unrelated trajectory.
EVAL_VERSION = 3


def cell_fingerprint(
    db_path: Path,
    pgo_config: dict[str, Any],
    lidar_stream: str,
    odom_stream: str,
    drift_per_sec: list[float] | None = None,
) -> dict[str, Any]:
    """Identity of a completed cell — the driver re-runs only when this changes
    (db edited, config changed, streams changed, drift changed, or version)."""
    stat = db_path.stat()
    return {
        "db_bytes": stat.st_size,
        "db_mtime": int(stat.st_mtime),
        "pgo_config": pgo_config,
        "lidar_stream": lidar_stream,
        "odom_stream": odom_stream,
        "drift_per_sec": list(drift_per_sec or [0.0, 0.0, 0.0]),
        "version": EVAL_VERSION,
    }


def odom_pose_samples(db_path: Path, odom_stream: str) -> tuple[np.ndarray, np.ndarray]:
    """Robot trajectory straight from the fed odom stream's stored poses.

    This is the raw baseline the module's correction is scored against: the SAME
    odometry with vs without loop closure. Handles both the ``Odometry`` and
    ``PoseStamped`` payload shapes found in recordings."""
    times: list[float] = []
    poses: list[list[float]] = []
    for timestamp, payload in iterate_stream(db_path, odom_stream):
        pose = payload_pose(payload)
        times.append(timestamp)
        poses.append(
            [
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        )
    if not times:
        raise SystemExit(f"{db_path}: odom stream {odom_stream!r} produced no poses")
    return (
        np.asarray(times, dtype=np.float64),
        np.asarray(poses, dtype=np.float64).reshape(-1, 7),
    )


def _report_dict(report: AgreementReport) -> dict[str, Any]:
    return {
        "mean_spread_m": report.mean_spread,
        "total_observations": report.total_observations,
        "per_tag": [
            {"tag_id": tag.tag_id, "observations": tag.observations, "spread_m": tag.spread}
            for tag in report.per_tag
        ],
    }


def evaluate(
    db_path: Path,
    *,
    odom_stream: str,
    camera_stream: str | None,
    intrinsics_json: Path | None,
    module_path: Path,
    module_name: str,
    pgo_config: dict[str, Any],
    with_rrd: bool,
    lidar_stream: str,
    lockstep: bool = True,
    results_suffix: str = "",
    recording_name: str | None = None,
    drift_per_sec: list[float] | None = None,
    ignore_tags: set[int] | None = None,
) -> dict[str, Any]:
    streams = list_streams(db_path)
    for required in (odom_stream, lidar_stream):
        if required not in streams:
            raise SystemExit(f"no stream {required!r} in {db_path} (have: {streams})")

    module_class = load_module_class(module_path, module_name)
    pgo_config = filter_config_for_module(module_class, pgo_config)

    # Artificial drift: the module is fed odom+lidar with a constant-velocity
    # world offset added at each time; the raw-baseline scoring must apply the
    # SAME offset so it compares against what the module actually saw.
    drift_per_sec = drift_per_sec or [0.0, 0.0, 0.0]
    drift_t0 = 0.0
    if has_drift(drift_per_sec):
        first_odom = next(iterate_stream(db_path, odom_stream), None)
        if first_odom is None:
            raise SystemExit(f"{db_path}: {odom_stream!r} is empty — cannot anchor drift start")
        drift_t0 = first_odom[0]

    # April-tag agreement needs a camera + intrinsics; voxel agreement does not.
    # Datasets without either (kitti-360, bare lidar recordings) still score on
    # voxel agreement alone, so the same harness fills every table cell.
    sightings: dict[int, list[float]] = {}
    tag_source = "none"
    have_camera = camera_stream is not None and camera_stream in streams
    if have_camera and intrinsics_json is not None and intrinsics_json.exists():
        assert camera_stream is not None  # narrowed by have_camera
        camera = camera_stream
        intrinsics_config = load_intrinsics_json(intrinsics_json)
        db_store = store(db_path)
        stored_stream: Iterable[Any] = (
            db_store.stream(APRIL_TAGS_STREAM)
            if APRIL_TAGS_STREAM in db_store.list_streams()
            else []
        )
        stored = ((int(obs.tags["marker_id"]), float(obs.ts)) for obs in stored_stream)

        def detect() -> Iterator[tuple[int, float]]:
            detections = detect_apriltags(
                db_store,
                intrinsics_config["intrinsics"],
                intrinsics_config["distortion"],
                image_stream=camera,
                stream_name=APRIL_TAGS_STREAM,
                marker_length=intrinsics_config.get("marker_length", 0.10),
                dictionary=intrinsics_config.get("dictionary", "DICT_APRILTAG_36h11"),
            )
            return ((int(d["marker_id"]), float(d["ts"])) for d in detections)

        sightings, tag_source = load_or_detect_sightings(stored, detect)
    # Drop dynamic/unreliable tags (e.g. a tag on a moving object) so their
    # motion isn't mistaken for trajectory drift. huge_loop_realsense tag #17 is
    # dynamic; all others are static.
    if ignore_tags:
        dropped = sorted(tag_id for tag_id in sightings if tag_id in ignore_tags)
        for tag_id in dropped:
            del sightings[tag_id]
        if dropped:
            print(f"ignoring tags {dropped} (declared dynamic/unreliable)")
    n_sightings = sum(len(times) for times in sightings.values())
    if sightings:
        print(f"april tags ({tag_source}): {n_sightings} sightings across ids {sorted(sightings)}")
    else:
        print("no April tags (camera/intrinsics absent or none detected) — voxel agreement only")

    started = time.monotonic()
    graph, closures, replay_stats = run_module_graph(
        db_path,
        module_class,
        pgo_config,
        lidar_stream=lidar_stream,
        odom_stream=odom_stream,
        lockstep=lockstep,
        drift_per_sec=drift_per_sec,
        drift_t0=drift_t0,
    )
    runtime_s = time.monotonic() - started
    if not graph:
        raise SystemExit(f"{module_name} produced an empty pose graph")

    # Raw-baseline trajectory: the fed odom stream's own poses (fastlio, etc.).
    # The correction is scored against the SAME odometry it was built from — no
    # tf tree / pointlio / other odom "answer key" (see odom_pose_samples).
    raw_times, raw_poses7 = odom_pose_samples(db_path, odom_stream)
    print(f"raw baseline from odom stream {odom_stream!r} ({len(raw_times)} samples)")
    raw_xyz_base = trajectory_lookup(raw_times, raw_poses7[:, :3], ODOM_MATCH_TOLERANCE_S)
    raw_pose7_base = pose7_lookup(raw_times, raw_poses7, ODOM_MATCH_TOLERANCE_S)

    # The module solved on drifted input, so its graph lives in the drifted
    # world; the raw baselines must be drifted to match (see drift_per_sec).
    raw_xyz_lookup = drifted_lookup(raw_xyz_base, drift_per_sec, drift_t0)
    raw_pose7_lookup = drifted_lookup(raw_pose7_base, drift_per_sec, drift_t0)

    xyz_graph = [(node[0], node[1], node[2], node[3]) for node in graph]
    if sightings:
        raw_tag_positions, corrected_tag_positions = paired_tag_visit_positions(
            sightings,
            raw_xyz_lookup,
            graph_lookup(xyz_graph),
            gap_s=VISIT_GAP_S,
        )
        raw_report = agreement_report(raw_tag_positions)
        corrected_report = agreement_report(corrected_tag_positions)
        improvement: float | None = agreement_improvement(raw_report, corrected_report)
    else:
        raw_report = agreement_report({})
        corrected_report = agreement_report({})
        improvement = None  # no tags — tag agreement is N/A for this cell

    voxel_stride = max(1, -(-stream_count(db_path, lidar_stream) // VOXEL_MAX_SCANS))
    voxel = lidar_voxel_agreement(
        (
            (timestamp, cloud.points_f32())
            for timestamp, cloud in iterate_stream(db_path, lidar_stream, stride=voxel_stride)
        ),
        raw_pose7_lookup,
        graph,
        drift_per_sec=drift_per_sec,
        drift_t0=drift_t0,
    )

    # Drift-recovery ATE: corrected trajectory vs the UN-drifted ground truth
    # (the odom before drift was injected). Only meaningful with --drift-per-sec;
    # the right metric where tag/voxel agreement is weak (e.g. KITTI's long loop).
    trajectory = trajectory_recovery_error(graph, raw_xyz_base, drift_per_sec, drift_t0)
    if trajectory is not None:
        print(
            f"  drift recovery:    {trajectory['drifted_ate_m']:.2f}"
            f" -> {trajectory['corrected_ate_m']:.2f} m ATE"
            f" ({trajectory['trajectory_improvement']:+.3f})"
        )

    # Key by package + class — several loop-closure modules are all named PGO.
    # results_suffix (dot-joined, NOT "__" which delimits the recording name)
    # separates runs that differ in inputs, e.g. fastlio vs pointlio odometry.
    module_package = module_class.__module__.rsplit(".", 2)[-2]
    module_key = f"{module_package}.{module_name}" + (
        f".{results_suffix}" if results_suffix else ""
    )
    # db.parent.name is the recording dir for go2; LFS dbs (hk_village) sit
    # directly in data/, so an explicit recording_name avoids cell collisions.
    out_dir = RESULTS_DIR / f"{recording_name or db_path.parent.name}__{module_key}"
    out_dir.mkdir(parents=True, exist_ok=True)
    rrd_path = out_dir / "eval.rrd"
    if with_rrd:
        write_trajectory_rrd(rrd_path, raw_poses7[:, :3], graph)

    summary = {
        "db": str(db_path),
        "odom_stream": odom_stream,
        "camera_stream": camera_stream,
        "lidar_stream": lidar_stream,
        "module": {"path": str(module_path), "name": module_name},
        "pgo_config": pgo_config,
        "drift_per_sec": list(drift_per_sec),
        "fingerprint": cell_fingerprint(
            db_path, pgo_config, lidar_stream, odom_stream, drift_per_sec
        ),
        "replay": replay_stats,
        "pose_source": "odom",
        "april_tags": {
            "source": tag_source,
            "sightings": n_sightings,
            "ids": sorted(sightings),
        },
        "scores": {
            "raw_spread_m": raw_report.mean_spread if sightings else None,
            "corrected_spread_m": corrected_report.mean_spread if sightings else None,
            "tag_improvement": improvement,
            "voxel_improvement": voxel.get("improvement"),
            "trajectory_improvement": trajectory["trajectory_improvement"] if trajectory else None,
            "drifted_ate_m": trajectory["drifted_ate_m"] if trajectory else None,
            "corrected_ate_m": trajectory["corrected_ate_m"] if trajectory else None,
            "closures": closures,
            "keyframes": len(graph),
            "runtime_s": round(runtime_s, 1),
        },
        "raw_agreement": _report_dict(raw_report),
        "corrected_agreement": _report_dict(corrected_report),
        "voxel_agreement": voxel,
        "rrd": str(rrd_path) if with_rrd else None,
        "evaluated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")

    print(f"\nresults -> {out_dir / 'summary.json'}")
    if sightings:
        print(
            f"  tag spread:        {raw_report.mean_spread:.3f}"
            f" -> {corrected_report.mean_spread:.3f} m"
        )
        print(f"  tag improvement:   {improvement:+.3f} (1.0 = perfect)")
    else:
        print("  tag improvement:   n/a (no tags)")
    if voxel.get("status") == "ok":
        print(
            f"  voxel agreement:   {voxel['raw_voxels']} -> {voxel['corrected_voxels']} voxels"
            f" ({voxel['improvement']:+.3f}, {voxel['scans_used']} scans @ {voxel['voxel_size_m']}m)"
        )
    else:
        print(f"  voxel agreement:   {voxel.get('status')}")
    print(f"  closures:          {closures}, keyframes: {len(graph)}")
    if with_rrd:
        print(f"  rrd:               {rrd_path}")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--db-path", type=Path, required=True)
    parser.add_argument("--odom-stream", required=True)
    parser.add_argument(
        "--camera-stream", default=None, help="omit for tagless datasets (voxel agreement only)"
    )
    parser.add_argument(
        "--camera-intrinsics-json-path",
        type=Path,
        default=None,
        help="omit for tagless datasets (voxel agreement only)",
    )
    parser.add_argument("--module-path", type=Path, required=True)
    parser.add_argument("--module-name", required=True)
    parser.add_argument(
        "--pgo-config-json",
        help="inline JSON of module config overrides (default: scan_context variant)",
    )
    parser.add_argument("--with-rrd", default="false", choices=["true", "false"])
    parser.add_argument(
        "--lidar-stream",
        default="fastlio_lidar",
        help="lidar stream replayed into the module alongside the odometry",
    )
    parser.add_argument(
        "--lockstep",
        default="true",
        choices=["true", "false"],
        help="pace scans on corrected_odometry acks (machine-independent); false = fixed-rate",
    )
    parser.add_argument(
        "--results-suffix",
        default="",
        help="extra results-dir key for runs with different inputs (e.g. pointlio)",
    )
    parser.add_argument(
        "--recording-name",
        default=None,
        help="results-dir recording key (default: db parent dir name)",
    )
    parser.add_argument(
        "--drift-per-sec",
        default=None,
        help="inject odom drift as a constant world velocity 'x,y,z' in m/s "
        "(offset = this * (t - t0), added to odom+lidar). e.g. '0.01,0,0'",
    )
    parser.add_argument(
        "--ignore-tags",
        default=None,
        help="comma-separated April-tag ids to drop from scoring (dynamic/unreliable "
        "tags whose motion would look like drift). e.g. '17'",
    )
    args = parser.parse_args()

    drift_per_sec = (
        [float(v) for v in args.drift_per_sec.split(",")] if args.drift_per_sec else None
    )
    if drift_per_sec is not None and len(drift_per_sec) != 3:
        raise SystemExit(f"--drift-per-sec must be 'x,y,z', got {args.drift_per_sec!r}")

    ignore_tags = (
        {int(tag_id) for tag_id in args.ignore_tags.split(",")} if args.ignore_tags else None
    )

    db_path = args.db_path.expanduser()
    if not db_path.exists():
        raise SystemExit(f"no such db: {db_path}")
    intrinsics_json = (
        args.camera_intrinsics_json_path.expanduser()
        if args.camera_intrinsics_json_path is not None
        else None
    )
    if intrinsics_json is not None and not intrinsics_json.exists():
        raise SystemExit(f"no such intrinsics json: {intrinsics_json}")

    pgo_config = json.loads(args.pgo_config_json) if args.pgo_config_json else {}

    evaluate(
        db_path,
        odom_stream=args.odom_stream,
        camera_stream=args.camera_stream,
        intrinsics_json=intrinsics_json,
        module_path=args.module_path,
        module_name=args.module_name,
        pgo_config=pgo_config,
        with_rrd=args.with_rrd == "true",
        lidar_stream=args.lidar_stream,
        lockstep=args.lockstep == "true",
        results_suffix=args.results_suffix,
        recording_name=args.recording_name,
        drift_per_sec=drift_per_sec,
        ignore_tags=ignore_tags,
    )


if __name__ == "__main__":
    main()
