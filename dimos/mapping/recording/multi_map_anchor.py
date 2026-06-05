#!/usr/bin/env python
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

"""Anchor one recording's trajectory onto another via shared AprilTags.

Takes two recording dirs of the same route -- a stable mid360_realsense map (the
anchor) and a shakier go2_mid360 map (the target). It:

  1. solves the anchor's drift-corrected trajectory (AprilTag landmark SLAM),
     ignoring the dog-mounted tag, and keeps the optimized world poses of the
     static environment tags as a tag map (this defines the world frame),
  2. solves the target the same way but pins the *shared* tags to the anchor's
     tag map, so the target lands in the anchor's world frame, drift-corrected,
  3. writes one .rrd overlaying the original FAST-LIO odom and the corrected
     odom from both recordings (and, by default, the re-anchored lidar maps so
     you can see whether the two corrected maps line up).

    uv run --no-sync python \
        dimos/mapping/recording/multi_map_anchor.py DIR_A DIR_B [--out PATH]

DIR_A/DIR_B may be in any order; the mid360_realsense recording (detected by its
`realsense_camera_info` stream) is always used as the anchor.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import cast

import numpy as np
import rerun as rr
import rerun.blueprint as rrb
from scipy.spatial.transform import Rotation

from dimos.mapping.recording.go2_mid360.post_process import load_camera as load_go2_camera
from dimos.mapping.recording.mid360_realsense.post_process import (
    load_camera as load_realsense_camera,
)
from dimos.mapping.recording.utils.apriltags import detect_apriltags
from dimos.mapping.recording.utils.build_rrd import _log_map, _log_path_gradient
from dimos.mapping.recording.utils.gtsam_gt import build_gtsam_gt, write_gtsam_odom
from dimos.mapping.recording.utils.lidar_reanchor import reanchor_stream
from dimos.mapping.recording.utils.post_process import CameraParams
from dimos.memory2.store.sqlite import SqliteStore

TagMap = dict[int, list[float]]
Trajectory = list[tuple[float, list[float]]]
MIN_SHARED_TAGS = 3  # rigid SE3 needs >=3 non-collinear correspondences

DB_NAME = "mem2.db"
DOG_TAG_ID = 17  # mounted on the robot dog -> not a static landmark, ignored
GTSAM_STREAM = "gtsam_odom"
FASTLIO_LIDAR = "fastlio_lidar"
FASTLIO_ODOM = "fastlio_odometry"
LOOP_LIDAR = "livox_lidar"  # raw sensor-frame cloud (loop closure needs sensor, not world, frame)
CORRECTED_LIDAR = "fastlio_lidar_corrected"
REALSENSE_INFO_STREAM = "realsense_camera_info"


def _resolve_db(arg: str) -> Path:
    path = Path(arg)
    if path.name == DB_NAME:
        return path
    if (path / DB_NAME).exists():
        return path / DB_NAME
    raise SystemExit(f"no {DB_NAME} found at {arg}")


def _is_realsense(db: Path) -> bool:
    # recordings drop a `type.<rig>` marker file; prefer it over opening the
    # (multi-GB) db, falling back to a stream check if absent.
    if (db.parent / "type.mid360_realsense").exists():
        return True
    if (db.parent / "type.go2_mid360").exists():
        return False
    with SqliteStore(path=str(db)) as store:
        return REALSENSE_INFO_STREAM in store.list_streams()


def _load_camera(db: Path) -> CameraParams:
    return load_realsense_camera(db) if _is_realsense(db) else load_go2_camera(db)


def _mat_from_pose7(pose7: list[float]) -> np.ndarray:
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(pose7[3:7]).as_matrix()
    matrix[:3, 3] = pose7[:3]
    return matrix


def _pose7_from_mat(matrix: np.ndarray) -> list[float]:
    quaternion = Rotation.from_matrix(matrix[:3, :3]).as_quat()
    return [*matrix[:3, 3].tolist(), *quaternion.tolist()]


def _rigid_align(source_points: np.ndarray, target_points: np.ndarray) -> np.ndarray:
    """Kabsch: the rigid SE3 (4x4) that best maps `source_points` onto
    `target_points` (no scale). Position-only — tag orientation is untrusted."""
    source_centroid = source_points.mean(axis=0)
    target_centroid = target_points.mean(axis=0)
    covariance = (source_points - source_centroid).T @ (target_points - target_centroid)
    u_matrix, _singular, vt_matrix = np.linalg.svd(covariance)
    reflection = np.diag([1.0, 1.0, np.sign(np.linalg.det(vt_matrix.T @ u_matrix.T))])
    rotation = vt_matrix.T @ reflection @ u_matrix.T
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = target_centroid - rotation @ source_centroid
    return transform


def _align_to_anchor(target_map: TagMap, anchor_map: TagMap) -> np.ndarray:
    """Rigid SE3 placing the target's frame into the anchor's frame, fit from the
    tags both maps share."""
    shared = sorted(set(target_map) & set(anchor_map))
    if len(shared) < MIN_SHARED_TAGS:
        raise SystemExit(
            f"only {len(shared)} shared tags {shared} (need >={MIN_SHARED_TAGS}) — cannot anchor"
        )
    source_points = np.array([target_map[marker_id][:3] for marker_id in shared])
    target_points = np.array([anchor_map[marker_id][:3] for marker_id in shared])
    transform = _rigid_align(source_points, target_points)
    residuals = np.linalg.norm(
        (source_points @ transform[:3, :3].T + transform[:3, 3]) - target_points, axis=1
    )
    print(
        f"   align: {len(shared)} shared tags {shared} | residual "
        f"mean {residuals.mean():.3f} m, max {residuals.max():.3f} m"
    )
    return transform


def _solve(
    db: Path,
    *,
    image_stream: str,
    marker_length: float,
    dictionary: str,
    add_loop_closures: bool,
) -> tuple[Trajectory, TagMap]:
    """Detect tags and run the GTSAM solve (ignoring the dog tag), each map in its
    own frame. Returns its corrected trajectory and optimized static tag map."""
    intrinsics, distortion, optical_in_base, _resolution = _load_camera(db)
    with SqliteStore(path=str(db)) as store:
        detections = detect_apriltags(
            store, intrinsics, distortion, image_stream, "april_tags", marker_length, dictionary
        )
    if not detections:
        raise SystemExit(f"no AprilTags detected in {db} -- cannot anchor")

    return cast(
        "tuple[Trajectory, TagMap]",
        build_gtsam_gt(
            str(db),
            detections,
            optical_in_base,
            exclude_marker_ids=(DOG_TAG_ID,),
            pose_stream=FASTLIO_ODOM,
            loop_lidar_stream=LOOP_LIDAR,
            add_loop_closures=add_loop_closures,
            return_landmarks=True,
        ),
    )


def _write_corrected(db: Path, trajectory: Trajectory) -> None:
    """Write `gtsam_odom` (+ .tum) and re-anchor the fastlio lidar onto it."""
    with SqliteStore(path=str(db)) as store:
        write_gtsam_odom(store, trajectory, GTSAM_STREAM, db.parent / "gtsam_odom.tum")
        stream_names = store.list_streams()
        if FASTLIO_LIDAR in stream_names and FASTLIO_ODOM in stream_names:
            try:
                reanchor_stream(
                    store,
                    str(db),
                    lidar_stream=FASTLIO_LIDAR,
                    odom_stream=FASTLIO_ODOM,
                    gtsam_stream=GTSAM_STREAM,
                    out_stream=CORRECTED_LIDAR,
                )
            except Exception as error:
                print(f"   re-anchor {CORRECTED_LIDAR} failed: {error}")


def build_combined_rrd(
    out_path: str, anchor_db: Path, target_db: Path, tag_map: TagMap, *, with_maps: bool
) -> None:
    """One Rerun recording overlaying both recordings in the shared world frame:
    raw FAST-LIO odom + corrected odom for each, the anchor tag map, and
    (optionally) the re-anchored lidar maps."""
    rr.init("multi_map_anchor", recording_id=str(out_path))
    rr.save(str(out_path))

    map_entities = ["anchor/map", "target/map"]
    hide = {f"/world/{entity}": rrb.EntityBehavior(visible=False) for entity in map_entities}
    rr.send_blueprint(
        rrb.Blueprint(
            rrb.Spatial3DView(origin="/world", name="3D", overrides=hide),
            rrb.BlueprintPanel(state=rrb.PanelState.Expanded),
            rrb.TimePanel(state=rrb.PanelState.Collapsed),
        )
    )

    # raw odom dim, corrected odom bright; anchor in greens/cyan, target in warms.
    _log_path_gradient(str(anchor_db), FASTLIO_ODOM, "world/anchor/fastlio_raw", (0, 110, 150))
    _log_path_gradient(str(anchor_db), GTSAM_STREAM, "world/anchor/corrected", (0, 220, 120))
    _log_path_gradient(str(target_db), FASTLIO_ODOM, "world/target/fastlio_raw", (150, 110, 0))
    _log_path_gradient(str(target_db), GTSAM_STREAM, "world/target/corrected", (255, 150, 60))

    for marker_id, pose7 in sorted(tag_map.items()):
        rr.log(
            f"world/tags/marker_{marker_id}",
            rr.Points3D(
                [pose7[:3]],
                colors=(255, 220, 60),
                radii=0.25,
                labels=[f"tag {marker_id}"],
                show_labels=True,
            ),
            static=True,
        )

    if with_maps:
        with SqliteStore(path=str(anchor_db)) as store:
            _log_map(store, CORRECTED_LIDAR, "world/anchor/map", 0.1, (0, 180, 170))
        with SqliteStore(path=str(target_db)) as store:
            _log_map(store, CORRECTED_LIDAR, "world/target/map", 0.1, (240, 160, 40))

    print(f"   wrote {out_path}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("dir_a", help="a recording dir or mem2.db")
    parser.add_argument("dir_b", help="a recording dir or mem2.db")
    parser.add_argument("--out", default=None, help="output .rrd (default: next to the anchor)")
    parser.add_argument("--image-stream", default="color_image")
    parser.add_argument("--marker-length", type=float, default=0.10)
    parser.add_argument("--dictionary", default="DICT_APRILTAG_36h11")
    parser.add_argument(
        "--loop",
        action="store_true",
        help="add lidar loop-closure constraints (off by default: with fastlio_odometry as the "
        "pose chain the livox_lidar revisits are unreliable and currently degrade the solve)",
    )
    parser.add_argument("--no-maps", action="store_true", help="skip the re-anchored lidar maps")
    args = parser.parse_args()

    db_a, db_b = _resolve_db(args.dir_a), _resolve_db(args.dir_b)
    if _is_realsense(db_a):
        anchor_db, target_db = db_a, db_b
    elif _is_realsense(db_b):
        anchor_db, target_db = db_b, db_a
    else:
        raise SystemExit("neither recording is a mid360_realsense (no realsense_camera_info)")
    print(f"anchor (realsense): {anchor_db.parent}")
    print(f"target (go2):       {target_db.parent}")

    common = {
        "image_stream": args.image_stream,
        "marker_length": args.marker_length,
        "dictionary": args.dictionary,
        "add_loop_closures": args.loop,
    }

    # The anchor (realsense) defines the world frame: solve it, write it as-is.
    print(">> solving anchor (defines the world frame)")
    anchor_trajectory, anchor_map = _solve(anchor_db, **common)
    _write_corrected(anchor_db, anchor_trajectory)

    # The target (go2) is solved in its own frame, then rigidly placed into the
    # anchor frame via the tags both maps share.
    print(">> solving target")
    target_trajectory, target_map = _solve(target_db, **common)
    print(">> aligning target onto the anchor frame")
    transform = _align_to_anchor(target_map, anchor_map)
    target_trajectory = [
        (timestamp, _pose7_from_mat(transform @ _mat_from_pose7(pose7)))
        for timestamp, pose7 in target_trajectory
    ]
    _write_corrected(target_db, target_trajectory)

    out_path = args.out or str(anchor_db.parent / "multi_map_anchor.rrd")
    print(">> building combined .rrd")
    build_combined_rrd(out_path, anchor_db, target_db, anchor_map, with_maps=not args.no_maps)
    print("done")


if __name__ == "__main__":
    main()
