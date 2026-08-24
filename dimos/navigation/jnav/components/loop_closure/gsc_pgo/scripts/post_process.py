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

--db is the recording .db file; its parent dir is where the .pc2.lcm outputs land. Camera
intrinsics come from the recording's CameraInfo stream (auto-detected) and the base<-optical
extrinsic from its tf tree; stream/frame defaults auto-detect the rig. With no CameraInfo
stream the AprilTag stage is skipped and ICP loop closures alone drive the PGO.

Usage:
  python .../gsc_pgo/scripts/post_process.py --db PATH.db [--no-odom | --no-lidar] [options]
"""

import argparse
from dataclasses import fields
from pathlib import Path
import sys
from typing import Any

from gtsam import Point3, Pose3, Rot3
import numpy as np

from dimos.memory.store.sqlite import SqliteStore
from dimos.navigation.jnav.components.loop_closure.gsc_pgo.utils.artifacts import (
    raycast_accumulate,
    write_corrected_lidar,
    write_corrected_odom,
    write_deformation_nodes,
    write_pose_graph,
)
from dimos.navigation.jnav.components.loop_closure.gsc_pgo.utils.db_fallback import (
    resolve_db_path,
)
from dimos.navigation.jnav.components.loop_closure.gsc_pgo.utils.go2_legacy import (
    normalize_go2_legacy,
)
from dimos.navigation.jnav.components.loop_closure.gsc_pgo.utils.offline_pgo import (
    Tuning,
    add_icp_closures,
    best_factor_per_keyframe_marker,
    build_tag_graph,
    report_revisits,
    select_keyframes,
    solve,
)
from dimos.navigation.jnav.components.loop_closure.gsc_pgo.utils.recording import (
    build_and_open_rrd,
)
from dimos.navigation.jnav.components.loop_closure.gsc_pgo.utils.recording_scans import (
    default_odom_edge,
    resolve_streams,
)
from dimos.navigation.jnav.components.loop_closure.utils import resolve_camera_info
from dimos.navigation.jnav.utils.apriltags import (
    ensure_raw_tag_stream,
    filter_glimpses,
    read_raw_tag_stream,
)
from dimos.navigation.jnav.utils.recording_tf import RecordingTF


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--db",
        type=Path,
        required=True,
        help="recording .db file, or an LFS dataset name (e.g. go2_china_office.db) "
        "which is git-pulled and decompressed from LFS when not present locally",
    )
    parser.add_argument("--lidar", default="", help="input lidar stream (auto if unset)")
    parser.add_argument("--odom", default="", help="input odometry stream (auto if unset)")
    parser.add_argument("--tags", default="raw_april_tags", help="unfiltered AprilTag stream")
    parser.add_argument("--camera", default="color_image", help="image stream to detect tags on")
    parser.add_argument(
        "--camera-info-stream",
        default="",
        help="CameraInfo stream (K + distortion); when unset, tries '<camera>_camera_info' "
        "then 'camera_info'",
    )
    parser.add_argument(
        "--tag-frame",
        default="camera_optical",
        help="optical frame the tag detections sit in; the base<-optical extrinsic is read from"
        " the tf tree (falls back to this when CameraInfo carries no frame_id)",
    )
    parser.add_argument(
        "--base-optical",
        default="",
        help="base<-optical camera extrinsic 'x y z qx qy qz qw' (meters + quaternion), used only "
        "when the recording has no tf tree to resolve it (e.g. hk_village-style recordings with "
        "camera_info but no tf/json). Mid360 rig: '0.3 0 0 -0.5 0.5 -0.5 0.5'",
    )
    parser.add_argument("--tag-size", type=float, default=0.10, help="AprilTag edge length (m)")
    parser.add_argument("--dict", dest="dictionary", default="DICT_APRILTAG_36h11")
    parser.add_argument("--ignore-tags", default="", help="comma/space-separated moving tag ids")
    parser.add_argument("--corrected-suffix", default="_corrected")
    parser.add_argument("--suffix", default="")
    parser.add_argument("--world-frame", default="world")
    parser.add_argument(
        "--corrected-odom-frame",
        default="corrected_odom",
        help="child frame the corrected odom/lidar hang on (tf-driven, not world-baked)",
    )
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
    parser.add_argument("--lcm-voxel", type=float, default=0.05)
    parser.add_argument("--accum-voxel", type=float, default=0.05)
    parser.add_argument("--accum-max-range", type=float, default=20.0)
    tuning_group = parser.add_argument_group(
        "solve tuning", "keyframe / factor-noise / ICP knobs (see offline_pgo.Tuning)"
    )
    for field in fields(Tuning):
        tuning_group.add_argument(
            f"--{field.name.replace('_', '-')}",
            type=type(field.default),
            default=field.default,
            help=f"default {field.default:g}",
        )
    return parser.parse_args()


def tuning_from_args(args: argparse.Namespace) -> Tuning:
    return Tuning(**{field.name: getattr(args, field.name) for field in fields(Tuning)})


def parse_base_optical(spec: str) -> Pose3 | None:
    """Parse a ``'x y z qx qy qz qw'`` base<-optical extrinsic into a ``Pose3`` (None if empty)."""
    if not spec.strip():
        return None
    values = [float(token) for token in spec.replace(",", " ").split()]
    if len(values) != 7:
        sys.exit(f"--base-optical needs 7 numbers 'x y z qx qy qz qw', got {len(values)}: {spec!r}")
    x, y, z, qx, qy, qz, qw = values
    return Pose3(Rot3.Quaternion(qw, qx, qy, qz), Point3(x, y, z))


def resolve_base_optical(
    store_tf: RecordingTF, body_frame: str, optical_frame: str, ts: float, cli_spec: str
) -> Pose3:
    """base<-optical extrinsic: explicit ``--base-optical``, else the recording's tf tree."""
    override = parse_base_optical(cli_spec)
    if override is not None:
        return override
    try:
        return Pose3(store_tf.get(body_frame, optical_frame, ts).to_matrix())
    except LookupError as error:
        sys.exit(
            f"cannot resolve the camera extrinsic {body_frame!r} <- {optical_frame!r} from the "
            f"tf tree; pass --base-optical 'x y z qx qy qz qw' for this rig. ({error})"
        )


def main() -> None:
    args = parse_args()
    tuning = tuning_from_args(args)
    db_path = resolve_db_path(args.db)
    if db_path.is_dir():
        sys.exit(f"--db must be a .db file, not a directory: {db_path}")
    rec_dir = db_path.parent
    with SqliteStore(path=db_path, must_exist=True) as store:
        # resolve stream/frame defaults from what the recording actually has
        odom_stream, lidar_stream = resolve_streams(store.list_streams(), args.odom, args.lidar)
        odom_tf = args.odom_tf or default_odom_edge(store, odom_stream)
        # legacy go2 recordings are massaged into the generic shape here; every other rig is a no-op
        odom_tf, odom_stream, lidar_stream = normalize_go2_legacy(
            store, odom_tf, odom_stream, lidar_stream
        )
        body_frame = odom_tf.split(":", 1)[1] if odom_tf else args.world_frame
        ignore_tags = {int(token) for token in args.ignore_tags.replace(",", " ").split()}

        camera_info, camera_info_tried = resolve_camera_info(
            store, args.camera, args.camera_info_stream
        )
        if camera_info is None:
            print(
                f"WARNING: no CameraInfo stream among {camera_info_tried} "
                "-- AprilTag stage skipped; ICP + odom only. If this is a go2 "
                "recording, add the static front-camera intrinsics first with "
                "scripts/add_camera_info.py, then re-run.",
                flush=True,
            )
            camera_matrix, distortion = None, None
            optical_frame = args.tag_frame
        else:
            camera_matrix, distortion, optical_frame = camera_info
            optical_frame = optical_frame or args.tag_frame
        tags_available = ensure_raw_tag_stream(
            store,
            camera_matrix,
            distortion,
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

        store_tf = RecordingTF.from_store(store, odom_tf=odom_tf or None, odom_stream=odom_stream)

        def world_points(observation: Any) -> np.ndarray:
            points = np.asarray(observation.data.points_f32())
            scan_frame = observation.data.frame_id
            transform = store_tf.get(args.world_frame, scan_frame, float(observation.ts), None)
            rotation = np.asarray(transform.rotation.to_rotation_matrix(), float).reshape(3, 3)
            translation = np.array(
                [transform.translation.x, transform.translation.y, transform.translation.z], float
            )
            world: np.ndarray = points @ rotation.T + translation
            return world.astype(np.float32)

        print(f"recording: {rec_dir}", flush=True)
        print(
            f"streams: tags={args.tags} odom={odom_stream} lidar={lidar_stream} -> {args.corrected_suffix}{args.suffix}",
            flush=True,
        )

        # gate tags, pick keyframes, keep one best factor per keyframe x marker
        raw_detections = read_raw_tag_stream(store, args.tags) if tags_available else []
        detections = filter_glimpses(raw_detections, exclude_tags=ignore_tags)
        odom_row_list: list[tuple[float, ...]] = []
        observation: Any
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
        _indices, keyframe_poses, keyframe_times = select_keyframes(odom_rows, tuning)
        best_factors = best_factor_per_keyframe_marker(detections, keyframe_times)
        if raw_detections:
            report_revisits(raw_detections, best_factors)

        # base<-optical camera extrinsic: --base-optical override, else the tf tree (was the json's
        # optical_in_base), else the known Go2/Mid360 rig mount geometry for tf-less recordings.
        base_optical = Pose3()
        if best_factors:
            base_optical = resolve_base_optical(
                store_tf,
                body_frame,
                optical_frame,
                # mid-run, not odom_rows[0]: tf typically starts a fraction of a second after
                # odometry, and a past-only lookup before the first tf sample breaks the chain
                float(np.median(odom_rows[:, 0])),
                args.base_optical,
            )

        # stage 1: tag PGO
        print(f"building factor graph over {len(keyframe_poses)} keyframes...", flush=True)
        graph, values, seen_markers = build_tag_graph(
            keyframe_poses, best_factors, base_optical, tuning
        )
        print("solving stage 1 (tag PGO)...", flush=True)
        estimate = solve(graph, values, tuning)
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
                tuning,
            )
            if accepted:
                print("solving stage 2 (tag PGO + ICP closures)...", flush=True)
                estimate = solve(graph, estimate, tuning)

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
            args.world_frame,
            body_frame,
        )
        write_pose_graph(
            store, f"pose_graph{args.suffix}", keyframe_times, estimate, args.world_frame
        )

        corrected_odom_out = f"{odom_stream}{args.corrected_suffix}{args.suffix}"
        if args.write_odom:
            write_corrected_odom(
                store,
                corrected_odom_out,
                odom_rows,
                keyframe_times,
                corrections,
                args.world_frame,
                args.corrected_odom_frame,
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
                args.world_frame,
                args.corrected_odom_frame,
            )
            if args.accum:
                raycast_accumulate(
                    store,
                    lidar_stream,
                    store_tf,
                    args.world_frame,
                    args.accum_voxel,
                    args.accum_max_range,
                )
                if corrected_odom_out in store.list_streams():
                    # tf that places the corrected-odom-framed per-scan clouds back into the
                    # world; also supplies the ray origin for the corrected raycast.
                    corrected_store_tf = RecordingTF.from_store(
                        store,
                        odom_tf=f"{args.world_frame}:{args.corrected_odom_frame}",
                        odom_stream=corrected_odom_out,
                    )
                    raycast_accumulate(
                        store,
                        lidar_out,
                        corrected_store_tf,
                        args.world_frame,
                        args.accum_voxel,
                        args.accum_max_range,
                    )
                else:
                    print(
                        "WARNING: no corrected odom stream (--no-odom?) -- skipping corrected lidar accumulation",
                        flush=True,
                    )
            if args.rrd:
                build_and_open_rrd(
                    db_path,
                    lidar_stream,
                    odom_stream,
                    args.tags,
                    args.world_frame,
                    camera_stream=args.camera,
                    camera_info_stream=args.camera_info_stream,
                )


if __name__ == "__main__":
    main()
