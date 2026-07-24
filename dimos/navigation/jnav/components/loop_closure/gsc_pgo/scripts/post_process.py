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

"""AprilTag-loop-closed + ICP-refined ground-truth post-processing for a recording.

NOTE: all of this should eventually be merged into `dimos map global`, its just harder to get the PR merged if I do that

Two-stage solve turns drifty odometry into a ground-truth trajectory:
  1. GTSAM tag PGO: anisotropic odometry between-factors (stiff roll/pitch + gravity z
     anchor, loose yaw) + quality-weighted AprilTag landmark factors fix macro drift.
  2. ICP loop closures between spatially-close / temporally-distant lidar submaps anchor
     local geometry, then re-solve.

Outputs written back into the recording db: <odom>_corrected, <lidar>_corrected,
tf_deformation_nodes_corrected, pose_graph, and raycast-accumulated maps; plus an
aggregated <lidar>_corrected.pc2.lcm and a comparison rrd opened in rerun.

--rec is a recording dir (mem2.db + camera_intrinsics.json sidecar) or a bare .db file.
Stream/frame defaults auto-detect the rig. With no camera_intrinsics.json the AprilTag
stage is skipped and ICP loop closures alone drive the PGO.

Usage:
  python .../gsc_pgo/scripts/post_process.py --rec PATH [--no-odom | --no-lidar] [options]
"""

import argparse
from pathlib import Path
import sys

import numpy as np

from dimos.memory2.store.sqlite import SqliteStore
from dimos.navigation.jnav.components.loop_closure.gsc_pgo.utils.artifacts import (
    raycast_accumulate,
    write_corrected_lidar,
    write_corrected_odom,
    write_deformation_nodes,
    write_pose_graph,
)
from dimos.navigation.jnav.components.loop_closure.gsc_pgo.utils.go2_legacy import (
    normalize_go2_legacy,
)
from dimos.navigation.jnav.components.loop_closure.gsc_pgo.utils.offline_pgo import (
    add_icp_closures,
    best_factor_per_keyframe_marker,
    build_tag_graph,
    report_revisits,
    select_keyframes,
    solve,
)
from dimos.navigation.jnav.components.loop_closure.gsc_pgo.utils.recording import (
    build_and_open_rrd,
    load_optical_transform,
    resolve_recording,
)
from dimos.navigation.jnav.components.loop_closure.gsc_pgo.utils.recording_scans import (
    default_odom_edge,
    resolve_streams,
    world_register,
)
from dimos.navigation.jnav.utils.apriltags import (
    ensure_raw_tag_stream,
    filter_glimpses,
    read_raw_tag_stream,
)
from dimos.navigation.jnav.utils.recording_tf import RecordingTF
from dimos.navigation.jnav.utils.trajectory_metrics import nearest_index


def parse_args():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--rec", type=Path, required=True, help="recording dir or .db path")
    parser.add_argument("--lidar", default="", help="input lidar stream (auto if unset)")
    parser.add_argument("--odom", default="", help="input odometry stream (auto if unset)")
    parser.add_argument("--tags", default="raw_april_tags", help="unfiltered AprilTag stream")
    parser.add_argument("--camera", default="color_image", help="image stream to detect tags on")
    parser.add_argument("--tag-size", type=float, default=0.10, help="AprilTag edge length (m)")
    parser.add_argument("--dict", dest="dictionary", default="DICT_APRILTAG_36h11")
    parser.add_argument("--ignore-tags", default="", help="comma/space-separated moving tag ids")
    parser.add_argument("--corrected-suffix", default="_corrected")
    parser.add_argument("--suffix", default="")
    parser.add_argument("--lidar-frame", default="", help="fallback frame for frame-less scans")
    parser.add_argument("--world-frame", default="world")
    parser.add_argument("--odom-tf", default="", help="'parent:child' edge the odom overrides")
    parser.add_argument(
        "--closure-spacing",
        type=float,
        default=2.0,
        help="max one ICP loop closure per this many meters of odom path (<=0 disables thinning)",
    )
    parser.add_argument("--no-odom", dest="write_odom", action="store_false")
    parser.add_argument("--no-lidar", dest="write_lidar", action="store_false")
    parser.add_argument("--no-icp", dest="icp", action="store_false")
    parser.add_argument("--no-lcm", dest="lcm", action="store_false")
    parser.add_argument("--no-rrd", dest="rrd", action="store_false")
    parser.add_argument("--no-accum", dest="accum", action="store_false")
    parser.add_argument("--no-tf", dest="tf", action="store_false")
    parser.add_argument("--lcm-voxel", type=float, default=0.05)
    parser.add_argument("--accum-voxel", type=float, default=0.05)
    parser.add_argument("--accum-max-range", type=float, default=20.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rec_dir, db_path = resolve_recording(args.rec)
    if not db_path.exists():
        sys.exit(f"no db at {db_path}")
    store = SqliteStore(path=db_path, must_exist=True)
    store.start()

    # resolve stream/frame defaults from what the recording actually has
    odom_stream, lidar_stream = resolve_streams(store.list_streams(), args.odom, args.lidar)
    odom_tf = args.odom_tf or default_odom_edge(store, odom_stream)
    # legacy go2 recordings are massaged into the generic shape here; every other rig is a no-op
    odom_tf, odom_stream, lidar_stream = normalize_go2_legacy(
        store, odom_tf, odom_stream, lidar_stream
    )
    lidar_frame = args.lidar_frame or (odom_tf.split(":", 1)[1] if odom_tf else args.world_frame)
    ignore_tags = {int(token) for token in args.ignore_tags.replace(",", " ").split()}

    base_optical, intrinsics = load_optical_transform(rec_dir)
    tags_available = ensure_raw_tag_stream(
        store,
        intrinsics,
        raw_stream=args.tags,
        image_stream=args.camera,
        marker_length=args.tag_size,
        dictionary=args.dictionary,
    )
    if not tags_available:
        print(
            f"no AprilTag data ({args.tags!r} absent) -- running tag-free (odom + ICP only)",
            flush=True,
        )

    store_tf = (
        RecordingTF.from_store(store, odom_tf=odom_tf or None, odom_stream=odom_stream)
        if args.tf
        else None
    )

    def world_points(observation):
        points, _origin = world_register(observation, store_tf, args.world_frame, lidar_frame)
        return points

    print(f"recording: {rec_dir}", flush=True)
    print(
        f"streams: tags={args.tags} odom={odom_stream} lidar={lidar_stream} -> {args.corrected_suffix}{args.suffix}",
        flush=True,
    )

    # gate tags, pick keyframes, keep one best factor per keyframe x marker
    raw_detections = read_raw_tag_stream(store, args.tags) if tags_available else []
    detections = filter_glimpses(raw_detections, exclude_tags=ignore_tags)
    odom_row_list: list[tuple[float, ...]] = []
    for observation in store.stream(odom_stream).order_by("ts"):
        odom_pose = observation.data.pose.pose
        odom_row_list.append(
            (
                float(observation.ts),
                odom_pose.position.x,
                odom_pose.position.y,
                odom_pose.position.z,
                odom_pose.orientation.x,
                odom_pose.orientation.y,
                odom_pose.orientation.z,
                odom_pose.orientation.w,
            )
        )
    odom_rows = np.asarray(odom_row_list, dtype=np.float64).reshape(-1, 8)
    if not len(odom_rows):
        sys.exit(f"odom stream {odom_stream!r} is empty in {db_path}")
    _indices, keyframe_poses, keyframe_times = select_keyframes(odom_rows)
    best_factors = best_factor_per_keyframe_marker(detections, keyframe_times)
    if raw_detections:
        report_revisits(raw_detections, best_factors)

    # stage 1: tag PGO
    print(f"building factor graph over {len(keyframe_poses)} keyframes...", flush=True)
    graph, values, seen_markers = build_tag_graph(keyframe_poses, best_factors, base_optical)
    print("solving stage 1 (tag PGO)...", flush=True)
    estimate = solve(graph, values)
    raw_keyframe_poses = list(keyframe_poses)

    # stage 2: ICP loop closures
    if args.icp:
        accepted = add_icp_closures(
            graph,
            estimate,
            store,
            lidar_stream,
            keyframe_poses,
            keyframe_times,
            world_points,
            args.closure_spacing,
        )
        if accepted:
            print("solving stage 2 (tag PGO + ICP closures)...", flush=True)
            estimate = solve(graph, estimate)

    # per-keyframe corrections
    corrections = [
        estimate.atPose3(index).compose(raw_keyframe_poses[index].inverse())
        for index in range(len(keyframe_poses))
    ]
    max_shift = max(float(np.linalg.norm(np.asarray(c.translation()))) for c in corrections)
    print(
        f"PGO: {len(keyframe_poses)} keyframes, {len(best_factors)} tag factors over "
        f"{len(seen_markers)} markers, max correction shift {max_shift:.1f} m",
        flush=True,
    )

    # persist PGO artifacts
    write_deformation_nodes(
        store,
        f"tf_deformation_nodes{args.corrected_suffix}{args.suffix}",
        keyframe_times,
        raw_keyframe_poses,
        estimate,
    )
    write_pose_graph(store, f"pose_graph{args.suffix}", keyframe_times, estimate)

    if args.write_odom:
        write_corrected_odom(
            store,
            f"{odom_stream}{args.corrected_suffix}{args.suffix}",
            odom_rows,
            keyframe_times,
            corrections,
        )

    if args.write_lidar:
        lidar_out = f"{lidar_stream}{args.corrected_suffix}{args.suffix}"
        lcm_path = (rec_dir / f"{lidar_out}.pc2.lcm") if args.lcm else None
        write_corrected_lidar(
            store,
            lidar_out,
            lidar_stream,
            odom_rows,
            keyframe_times,
            corrections,
            world_points,
            lcm_path,
            args.lcm_voxel,
        )
        if args.accum:
            odom_times = odom_rows[:, 0]

            def origin_from_odom(ts):
                row = odom_rows[nearest_index(odom_times, ts)]
                return float(row[1]), float(row[2]), float(row[3])

            raycast_accumulate(
                store,
                lidar_stream,
                store_tf,
                args.world_frame,
                lidar_frame,
                origin_from_odom,
                args.accum_voxel,
                args.accum_max_range,
            )
            raycast_accumulate(
                store,
                lidar_out,
                store_tf,
                "odom",
                "",
                None,
                args.accum_voxel,
                args.accum_max_range,
            )
        if args.rrd and intrinsics is not None:
            build_and_open_rrd(db_path, lidar_stream, odom_stream, args.tags)
    store.stop()


if __name__ == "__main__":
    main()
