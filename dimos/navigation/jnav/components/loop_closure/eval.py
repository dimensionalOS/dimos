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

"""Loop-closure evaluation by physical self-consistency (no world frame).

A recording carries sensor data, one algorithm's odometry, and the static tf
tree — but no global pose and no ground truth. The only physical anchor is that
an AprilTag is the same physical object every time it is seen. So evaluation
asks a single question: does the module's correction make the recording more
self-consistent than the raw odometry it was fed?

The raw baseline is the odometry stream the module is actually fed
(``--odom-stream``), read as the map<-base_link pose over time. The module is
replayed on that odometry + lidar and hands back its optimized keyframe graph.
The corrected trajectory is the raw odometry warped by the module's own
per-keyframe deformation (Δ_i = corrected_i ∘ raw_i⁻¹, interpolated between
keyframes — "plain stretch"), so raw and corrected differ only by what the
module changed.

Two agreement scores, raw vs corrected:
  * AprilTag spread — each tag is placed in the map by real camera geometry
    (map<-base_link(t) ∘ base_link<-camera_optical ∘ camera_optical<-tag), NOT a
    robot-position proxy. A fixed tag re-seen along the run should map to ONE
    map position; the spread of its per-visit placements measures drift. Tags in
    ``--dynamic-tags`` are held out (they move, so their spread is not drift).
  * Lidar-voxel agreement — re-anchoring the registered scans onto the corrected
    trajectory should collapse doubled walls, so the corrected map occupies
    fewer voxels.

Also writes before/after visuals so the correction can be eyeballed
(contraction-gaming and tf-chain errors are obvious to the eye, invisible to a
scalar): a top-down PNG, an isometric PNG, and a rerun rrd — each toggleable via
``--topdown-png`` / ``--isometric-png`` / ``--rrd`` (all on by default).
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
from typing import Any

import numpy as np

from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.navigation.jnav.components.loop_closure.gsc_pgo.utils.db_fallback import resolve_db_path
from dimos.navigation.jnav.components.loop_closure.gsc_pgo.utils.go2_legacy import (
    normalize_go2_legacy,
)
from dimos.navigation.jnav.components.loop_closure.gsc_pgo.utils.replay import (
    EDGE_LOOP_CLOSURE,
    run_module_graph,
)
from dimos.navigation.jnav.components.loop_closure.utils import (
    MAP_MAX_SCANS,
    accumulate_maps,
    load_tag_detections,
    registered_scans,
    report_dict,
    score_tags,
    write_comparison_rrd,
    write_isometric_png,
    write_topdown_png,
)
from dimos.navigation.jnav.utils.module_loading import (
    filter_config_for_module,
    load_module_class,
)
from dimos.navigation.jnav.utils.recording_tf import RecordingTF
from dimos.navigation.jnav.utils.trajectory_metrics import (
    deform_path,
    drift_delta_lookup,
    lidar_voxel_agreement,
    pose_lookup,
)

RESULTS_DIR = Path(__file__).resolve().parent / "eval_results"
DEFAULT_TAG_FRAME = "camera_optical"
DEFAULT_BASE_FRAME = "base_link"
DEFAULT_WORLD_FRAME = "world"


def _git_state() -> tuple[str | None, bool | None]:
    """(commit hash, unstaged-changes flag) of the repo this file lives in."""
    try:
        result = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=Path(__file__).resolve().parent,
            capture_output=True,
            text=True,
            check=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return None, None
    commit = result.stdout.strip()
    dirty = subprocess.run(
        ["git", "status", "--porcelain", "--untracked-files=no"],
        cwd=Path(__file__).resolve().parent,
        capture_output=True,
        text=True,
    ).stdout.strip()
    return commit, bool(dirty)


def evaluate(
    db_path: Path,
    *,
    odom_stream: str,
    lidar_stream: str,
    camera_stream: str | None,
    camera_info_stream: str,
    module_path: Path,
    module_name: str,
    pgo_config: dict[str, Any],
    dynamic_tags: set[int],
    tag_frame: str,
    odom_parent: str,
    odom_child: str,
    recording_name: str | None,
    write_topdown: bool,
    write_isometric: bool,
    write_rrd: bool,
    tf_failure_budget: int = 30,
) -> dict[str, Any]:
    evaluate_started = time.monotonic()
    with SqliteStore(path=db_path, must_exist=True) as store:
        # legacy go2 recordings are massaged into the generic shape here; every other rig is a no-op
        odom_tf, odom_stream, lidar_stream = normalize_go2_legacy(
            store, f"{odom_parent}:{odom_child}", odom_stream, lidar_stream
        )
        odom_parent, _, odom_child = odom_tf.partition(":")
        streams = store.list_streams()
        for required in (odom_stream, lidar_stream, "tf"):
            if required not in streams:
                raise SystemExit(f"no stream {required!r} in {db_path} (have: {streams})")

        recording_name = recording_name or db_path.parent.name
        module_class = load_module_class(module_path, module_name)
        pgo_config = filter_config_for_module(module_class, pgo_config)

        odom_row_list: list[tuple[float, float, float, float, float, float, float, float]] = []
        for observation in store.stream(odom_stream, Odometry).order_by("ts"):
            pose = observation.data.pose.pose
            odom_row_list.append(
                (
                    float(observation.ts),
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                )
            )
        odom_rows = np.asarray(odom_row_list, dtype=np.float64).reshape(-1, 8)
        if not len(odom_rows):
            raise SystemExit(f"odom stream {odom_stream!r} produced no poses in {db_path}")
        lidar_count = int(store.stream(lidar_stream).count())
        raw_times, raw_poses = odom_rows[:, 0], odom_rows[:, 1:]
        tf = RecordingTF.from_store(store)
        tf.override_edge(odom_parent, odom_child, raw_times, raw_poses)

        detections = load_tag_detections(
            db_path, camera_stream, camera_info_stream, streams, dynamic_tags
        )
        if detections:
            # probe mid-recording: startup detections may predate odom coverage (tolerated
            # per-detection in place_tags), but an unreachable camera frame here means the
            # tf tree can never place tags
            probe_ts = float(raw_times[len(raw_times) // 2])
            try:
                tf.get(odom_parent, tag_frame, probe_ts)
            except LookupError as error:
                print(
                    f"WARNING: no tf path {odom_parent} <- {tag_frame} — "
                    f"skipping tag scoring, voxel agreement only ({error})",
                    flush=True,
                )
                detections = []

        started = time.monotonic()
        graph, closures, loop_edges, replay_stats, graph_detail = run_module_graph(
            db_path,
            module_class,
            pgo_config,
            lidar_stream=lidar_stream,
            odom_stream=odom_stream,
        )
        runtime_s = time.monotonic() - started
        if not graph:
            raise SystemExit(f"{module_name} produced an empty pose graph")

        raw_pose = pose_lookup(raw_times, raw_poses, tolerance=float("inf"))
        delta_lookup = drift_delta_lookup(list(graph), raw_pose)

        raw_report, corrected_report, improvement, raw_tag_medians, corrected_tag_medians = (
            score_tags(detections, tf, odom_parent, tag_frame, delta_lookup)
        )

        raw_map, corrected_map = accumulate_maps(
            registered_scans(db_path, lidar_stream, 1, tf, odom_parent, tf_failure_budget),
            delta_lookup,
        )
        metric_stride = max(1, -(-lidar_count // MAP_MAX_SCANS))
        voxel = lidar_voxel_agreement(
            registered_scans(
                db_path, lidar_stream, metric_stride, tf, odom_parent, tf_failure_budget
            ),
            raw_pose,
            list(graph),
        )

    raw_path, corrected_path = deform_path(raw_times, raw_poses, delta_lookup)
    closure_segments = np.array(
        [
            [
                np.interp(start_ts, raw_times, raw_poses[:, 0]),
                np.interp(start_ts, raw_times, raw_poses[:, 1]),
                np.interp(start_ts, raw_times, raw_poses[:, 2]),
                np.interp(end_ts, raw_times, raw_poses[:, 0]),
                np.interp(end_ts, raw_times, raw_poses[:, 1]),
                np.interp(end_ts, raw_times, raw_poses[:, 2]),
            ]
            for start_ts, end_ts in loop_edges
        ]
    ).reshape(-1, 6)

    out_dir = RESULTS_DIR / f"{recording_name}__{module_name}"
    out_dir.mkdir(parents=True, exist_ok=True)
    # cached render inputs: lets plot styling be iterated without replaying the module
    np.savez_compressed(
        out_dir / "topdown_inputs.npz",
        raw_map=raw_map,
        corrected_map=corrected_map,
        raw_path=raw_path,
        corrected_path=corrected_path,
        closure_segments=closure_segments,
    )
    png_path = out_dir / "topdown_before_after.png"
    if write_topdown:
        write_topdown_png(
            png_path,
            raw_map,
            corrected_map,
            raw_tag_medians,
            corrected_tag_medians,
            raw_path,
            corrected_path,
            recording_name,
            closure_segments,
        )
    iso_png_path = out_dir / "isometric_before_after.png"
    if write_isometric:
        write_isometric_png(
            iso_png_path,
            raw_map,
            corrected_map,
            raw_path,
            corrected_path,
            recording_name,
            closure_segments,
        )
    rrd_path = out_dir / "comparison.rrd"
    if write_rrd:
        write_comparison_rrd(
            rrd_path,
            raw_map,
            corrected_map,
            raw_tag_medians,
            corrected_tag_medians,
            raw_path,
            corrected_path,
            recording_name,
            closure_segments,
        )

    git_commit, git_unstaged_changes = _git_state()
    node_fields = ["id", "ts", "x", "y", "z", "qx", "qy", "qz", "qw"]
    final_pose_graph = {
        "node_fields": node_fields,
        "nodes": graph_detail["nodes"],
        "edges": [
            [start_id, end_id, "loop_closure" if metadata_id == EDGE_LOOP_CLOSURE else "odom"]
            for start_id, end_id, metadata_id in graph_detail["edges"]
        ],
    }
    raw_pose_graph = {
        "node_fields": node_fields,
        "nodes": [
            [node_id, node_ts, *np.asarray(raw_pose(node_ts)).tolist()]
            for node_id, node_ts, *_ in graph_detail["nodes"]
        ],
    }

    summary = {
        "db": str(db_path),
        "odom_stream": odom_stream,
        "lidar_stream": lidar_stream,
        "camera_stream": camera_stream,
        "module": {"path": str(module_path), "name": module_name},
        "pgo_config": pgo_config,
        "dynamic_tags": sorted(dynamic_tags),
        "argv": sys.argv,
        "replay": replay_stats,
        "scores": {
            "raw_spread_m": raw_report.mean_spread if detections else None,
            "corrected_spread_m": corrected_report.mean_spread if detections else None,
            "tag_improvement": improvement,
            "voxel_improvement": voxel.get("improvement"),
            "closures": closures,
            "keyframes": len(graph),
            "runtime_s": round(runtime_s, 1),
        },
        "raw_agreement": report_dict(raw_report),
        "corrected_agreement": report_dict(corrected_report),
        "voxel_agreement": voxel,
        "raw_path": raw_path.tolist(),
        "corrected_path": corrected_path.tolist(),
        "raw_pose_graph": raw_pose_graph,
        "final_pose_graph": final_pose_graph,
        "topdown_png": str(png_path) if write_topdown else None,
        "isometric_png": str(iso_png_path) if write_isometric else None,
        "rrd": str(rrd_path) if write_rrd else None,
        "evaluated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "total_runtime_s": round(time.monotonic() - evaluate_started, 1),
        "git_commit": git_commit,
        "git_unstaged_changes": git_unstaged_changes,
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")

    print(f"\nresults -> {out_dir / 'summary.json'}")
    if detections:
        print(
            f"  tag spread:      {raw_report.mean_spread:.3f}"
            f" -> {corrected_report.mean_spread:.3f} m ({improvement:+.3f})"
        )
    else:
        print("  tag spread:      n/a (no tags)")
    if voxel.get("status") == "ok":
        print(
            f"  voxel agreement: {voxel['raw_voxels']} -> {voxel['corrected_voxels']} voxels"
            f" ({voxel['improvement']:+.3f}, {voxel['scans_used']} scans)"
        )
    print(f"  closures: {closures}, keyframes: {len(graph)}")
    if write_topdown:
        print(f"  top-down map:    {png_path}")
    if write_isometric:
        print(f"  isometric map:   {iso_png_path}")
    if write_rrd:
        print(f"  rrd:             {rrd_path}")
    return summary


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--db-path", type=Path, required=True)
    parser.add_argument("--odom-stream", required=True)
    parser.add_argument("--lidar-stream", default="fastlio_lidar")
    parser.add_argument("--camera-stream", default="color_image")
    parser.add_argument(
        "--camera-info-stream",
        default="camera_info",
        help="stream carrying CameraInfo (K + distortion), used to detect tags when "
        "raw_april_tags isn't already in the db.",
    )
    parser.add_argument("--module-path", type=Path, required=True)
    parser.add_argument("--module-name", required=True)
    parser.add_argument("--pgo-config-json", default=None)
    parser.add_argument(
        "--dynamic-tags",
        default="",
        help="comma-separated tag ids to hold OUT of agreement (moving tags). e.g. '17'",
    )
    parser.add_argument("--tag-frame", default=DEFAULT_TAG_FRAME)
    parser.add_argument(
        "--odom-tf",
        default=f"{DEFAULT_WORLD_FRAME}:{DEFAULT_BASE_FRAME}",
        help="parent:child frame edge the odom stream publishes, e.g. 'world:l1_link'. "
        "Lidar scans resolve through the tf tree with this dynamic edge added.",
    )
    parser.add_argument("--recording-name", default=None)
    parser.add_argument("--topdown-png", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--isometric-png", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--rrd", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument(
        "--tf-failure-budget",
        type=int,
        default=30,
        help="how many scans may fail tf registration (startup/shutdown odom-coverage noise) "
        "before erroring out",
    )
    args = parser.parse_args()

    try:
        db_path = resolve_db_path(args.db_path)
    except (FileNotFoundError, RuntimeError) as error:
        raise SystemExit(f"no such db: {args.db_path.expanduser()} ({error})")
    dynamic_tags = {int(tag) for tag in args.dynamic_tags.split(",") if tag.strip()}
    pgo_config = json.loads(args.pgo_config_json) if args.pgo_config_json else {}
    odom_parent, _, odom_child = args.odom_tf.partition(":")
    if not odom_parent or not odom_child:
        raise SystemExit(f"--odom-tf must be 'parent:child', got {args.odom_tf!r}")

    evaluate(
        db_path,
        odom_stream=args.odom_stream,
        lidar_stream=args.lidar_stream,
        camera_stream=args.camera_stream,
        camera_info_stream=args.camera_info_stream,
        module_path=args.module_path,
        module_name=args.module_name,
        pgo_config=pgo_config,
        dynamic_tags=dynamic_tags,
        tag_frame=args.tag_frame,
        odom_parent=odom_parent,
        odom_child=odom_child,
        recording_name=args.recording_name,
        write_topdown=args.topdown_png,
        write_isometric=args.isometric_png,
        write_rrd=args.rrd,
        tf_failure_budget=args.tf_failure_budget,
    )


if __name__ == "__main__":
    main()
