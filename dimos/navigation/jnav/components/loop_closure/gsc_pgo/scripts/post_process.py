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

# Untyped analysis script: gtsam/open3d/cv2 lack type stubs.
# mypy: ignore-errors
"""AprilTag-loop-closed + ICP-refined ground-truth post-processing for a go2 recording.

Source of tags = the UNFILTERED tag stream (build it first with add_april.py). Each raw
detection carries its gate diagnostics, so gates are applied here post-hoc (no re-detection)
and are easy to relax. Factors: one robust (best-reproj) observation per keyframe x marker ->
denser, balanced loop closure than one-medoid-per-visit.

Two-stage solve: (1) GTSAM tag PGO -- anisotropic odometry between-factors (stiff roll/pitch + z
anchor gravity, loose yaw) + quality-weighted AprilTag landmark factors fix macro drift;
(2) ICP loop closures between spatially-close / temporally-distant lidar submaps anchor local
geometry. Writes <lidar>_corrected / <odom>_corrected back into the recording db, plus raycast-
accumulated maps (<lidar>_accumulated original + <lidar>_corrected_accumulated), optionally a
.pc2.lcm log of the corrected cloud, and opens a comparison rrd.

Usage: python dimos/navigation/jnav/components/loop_closure/gsc_pgo/scripts/post_process.py [odom|lidar|both] --rec=PATH
       [--lidar=pointlio_lidar] [--odom=pointlio_odometry] [--tags=raw_april_tags]
       [--corrected-suffix=_corrected] [--suffix=...] [--ignore-tags=17]
       [--no-icp] [--no-lcm] [--no-rrd] [--no-accum]
"""

import json
from pathlib import Path
import re
import sqlite3
import sys
import time

from gtsam import (
    BetweenFactorPose3,
    LevenbergMarquardtOptimizer,
    LevenbergMarquardtParams,
    NonlinearFactorGraph,
    Point3,
    Pose3,
    PriorFactorPose3,
    Rot3,
    Symbol,
    Values,
    noiseModel,
)
import numpy as np

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.jnav.components.loop_closure.gsc_pgo.scripts.accumulate import (
    accumulate_stream,
)
from dimos.navigation.jnav.msgs.DeformationNode import DeformationNode, tf_id_for
from dimos.navigation.jnav.msgs.Graph3D import Graph3D
from dimos.navigation.jnav.utils import recording_db as rdb
from dimos.navigation.jnav.utils.apriltags import (
    DEFAULT_MAX_ANGULAR_SPEED_DPS,
    DEFAULT_MAX_DISTANCE_M,
    DEFAULT_MAX_LINEAR_SPEED_MPS,
    DEFAULT_MAX_REPROJ_PX,
    DEFAULT_MAX_VIEW_ANGLE_DEG,
    DEFAULT_MIN_SHARPNESS,
    DEFAULT_MIN_TAG_PX,
    _write_tag_stream,
    detect_raw_detections,
    view_quality,
)
from dimos.navigation.jnav.utils.recording_tf import RecordingTF

VISIT_GAP_S = 30.0
WHAT = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("-") else "both"


def arg(flag, default=""):
    return next(
        (item.split("=", 1)[1] for item in sys.argv if item.startswith(flag + "=")), default
    )


REC_ARG = arg("--rec")
SUFFIX = arg("--suffix")
LIDAR_STREAM = arg("--lidar", "pointlio_lidar")  # input lidar stream (world-registered scans)
ODOM_STREAM = arg("--odom", "pointlio_odometry")  # input odometry stream (keyframe source)
# ODOM_STREAM is interpolated as a table name (SQLite can't parameterize those);
# reject anything that isn't a plain identifier to keep that injection-free.
if not re.match(r"^[A-Za-z_][A-Za-z0-9_]*$", ODOM_STREAM):
    raise ValueError(f"unsafe --odom stream name: {ODOM_STREAM!r}")
RAW_STREAM = arg(
    "--tags", "raw_april_tags"
)  # unfiltered AprilTag stream (auto-detected if missing)
CAMERA = arg("--camera", "color_image")  # image stream to detect on when RAW_STREAM is missing
MARKER_LENGTH_M = float(arg("--tag-size", "0.10"))  # AprilTag edge length (m), for auto-detect
DICTIONARY = arg("--dict", "DICT_APRILTAG_36h11")  # AprilTag dictionary, for auto-detect
IGNORE_TAGS = {
    int(marker_id) for marker_id in arg("--ignore-tags").replace(",", " ").split()
}  # dynamic/moving tags
CORRECTED_SUFFIX = arg("--corrected-suffix", "_corrected")  # corrected streams: <input><suffix>
WRITE_LCM = "--no-lcm" not in sys.argv  # also emit <lidar>_corrected.pc2.lcm of the corrected cloud
OPEN_RRD = "--no-rrd" not in sys.argv  # build + open a comparison rrd at the end
LCM_VOXEL = float(arg("--lcm-voxel", "0.05"))  # voxel size for the aggregated .pc2.lcm cloud
LCM_OUTLIER_NN = 20  # statistical outlier removal: neighbor count
LCM_OUTLIER_STD = 2.0  # ...and std-ratio threshold (lower = more aggressive)
LIDAR_FRAME = arg("--lidar-frame", "mid360_link")  # frame the raw lidar scans live in
WORLD_FRAME = arg("--world-frame", "world")  # frame to register scans into
USE_TF = "--no-tf" not in sys.argv  # world-register via recording tf (fallback: obs.pose)
WRITE_ACCUM = "--no-accum" not in sys.argv  # raycast-accumulated maps: original + corrected
ACCUM_VOXEL = float(arg("--accum-voxel", "0.05"))  # accumulation resolution / final voxel size (m)
ACCUM_MAX_RANGE = float(arg("--accum-max-range", "20"))  # ignore returns/clearing beyond this (m)

# Per-glimpse gates (speed == -1 means "unknown" and always passes).
GATE = dict(
    min_sharpness=DEFAULT_MIN_SHARPNESS,
    max_reproj_px=DEFAULT_MAX_REPROJ_PX,
    min_tag_px=DEFAULT_MIN_TAG_PX,
    max_distance_m=DEFAULT_MAX_DISTANCE_M,
    max_view_angle_deg=DEFAULT_MAX_VIEW_ANGLE_DEG,
    max_lin_speed=DEFAULT_MAX_LINEAR_SPEED_MPS,
    max_ang_speed=DEFAULT_MAX_ANGULAR_SPEED_DPS,
)

if not REC_ARG:
    sys.exit(
        "usage: python dimos/navigation/jnav/components/loop_closure/gsc_pgo/scripts/post_process.py [odom|lidar|both] --rec=PATH "
        "[--lidar=...] [--odom=...] [--tags=...] [--corrected-suffix=...] [--suffix=...] "
        "[--no-icp] [--no-lcm] [--no-rrd] [--no-accum]   (--rec is required: recording dir)"
    )
REC = Path(REC_ARG).expanduser()
DB = REC / "mem2.db"
intrinsics = json.loads((REC / "camera_intrinsics.json").read_text())
optical_in_base = np.array(intrinsics["optical_in_base"], float)
T_base_optical = Pose3(
    Rot3.Quaternion(optical_in_base[6], optical_in_base[3], optical_in_base[4], optical_in_base[5]),
    Point3(optical_in_base[0], optical_in_base[1], optical_in_base[2]),
)
store = rdb.store(DB)
if RAW_STREAM not in store.list_streams():
    # No tag stream yet -> detect them now (so a fresh recording only needs post_process).
    if CAMERA not in store.list_streams():
        sys.exit(
            f"!! {RAW_STREAM} missing and can't auto-detect: camera stream {CAMERA!r} not in db."
        )
    print(
        f"{RAW_STREAM} missing -- detecting AprilTags over {CAMERA} "
        f"(tag_size={MARKER_LENGTH_M} m, dict={DICTIONARY})...",
        flush=True,
    )
    camera_matrix = np.array(intrinsics["intrinsics"], float).reshape(3, 3)
    distortion = np.array(intrinsics.get("distortion", []), float)
    raw_detections, _, n_images = detect_raw_detections(
        store,
        camera_matrix,
        distortion,
        image_stream=CAMERA,
        marker_length=MARKER_LENGTH_M,
        dictionary=DICTIONARY,
    )
    _write_tag_stream(store, RAW_STREAM, raw_detections, diagnostics=True)
    print(
        f"wrote {RAW_STREAM}: {len(raw_detections)} raw detections over {n_images} frames",
        flush=True,
    )


def transform_matrix(transform):
    """``(R, t)`` (3x3, 3) for a Transform so ``p_target = p_source @ R.T + t``."""
    rotation = np.asarray(transform.rotation.to_rotation_matrix(), float).reshape(3, 3)
    translation = np.array(
        [transform.translation.x, transform.translation.y, transform.translation.z], float
    )
    return rotation, translation


_STORE_TF = RecordingTF.from_store(store) if USE_TF else None
_TF_AVAILABLE = _STORE_TF is not None


def world_points(observation):
    """Nx3 world-registered points for a lidar observation.

    The scan's own ``frame_id`` decides what to do: a scan already in
    WORLD_FRAME is returned untouched (transforming it again double-registers
    it), otherwise it's brought into world via tf (``world <- frame_id``) at the
    scan time. Falls back to LIDAR_FRAME when the scan carries no frame, then
    to the observation's stored pose, then to assuming it's already world.
    """
    points = np.asarray(observation.data.points_f32())
    if not len(points):
        return points
    scan_frame = getattr(observation.data, "frame_id", "") or LIDAR_FRAME
    if scan_frame == WORLD_FRAME:
        return points  # already world-registered per its own header
    if _STORE_TF is not None:
        # tolerance=None -> nearest recorded sample per edge; RecordingTF keeps the
        # whole recording buffered, so one-shot static frames stay resolvable and the
        # densely-sampled odom->base_link edge lands within a few ms of the scan.
        transform = _STORE_TF.get(WORLD_FRAME, scan_frame, float(observation.ts), None)
        if transform is not None:
            rotation, translation = transform_matrix(transform)
            return points @ rotation.T + translation
    pose = getattr(observation, "pose", None)
    if isinstance(pose, (tuple, list)) and len(pose) >= 7:
        rotation = Rot3.Quaternion(pose[6], pose[3], pose[4], pose[5]).matrix()
        return points @ rotation.T + np.array(pose[:3], float)
    return points  # unknown frame, assume already world-registered


print(f"recording: {REC}", flush=True)
if _TF_AVAILABLE:
    print(
        f"world-registering {LIDAR_STREAM} via tf into {WORLD_FRAME} "
        f"(per-scan frame_id; already-{WORLD_FRAME} scans left as-is)",
        flush=True,
    )
print(
    f"streams: tags={RAW_STREAM} odom={ODOM_STREAM} lidar={LIDAR_STREAM} "
    f"-> corrected suffix {CORRECTED_SUFFIX}{SUFFIX}",
    flush=True,
)


def passes(detection):
    return (
        detection["sharpness"] >= GATE["min_sharpness"]
        and detection["reproj_px"] <= GATE["max_reproj_px"]
        and detection["tag_px"] >= GATE["min_tag_px"]
        and detection["distance_m"] <= GATE["max_distance_m"]
        and detection["view_angle_deg"] <= GATE["max_view_angle_deg"]
        and (detection["lin_speed"] < 0 or detection["lin_speed"] <= GATE["max_lin_speed"])
        and (detection["ang_speed"] < 0 or detection["ang_speed"] <= GATE["max_ang_speed"])
    )


# read raw detections (pose + diagnostics)
print("reading tag detections...", flush=True)
raw_detections = []
for observation in store.stream(RAW_STREAM):
    pose = observation.data
    tags = observation.tags
    # distance_m / view_angle_deg are derived from the tag pose (not required in the
    # tag stream), so any detector's raw stream works as long as it carries the pose
    # + the image-only diagnostics (sharpness/reproj_px/tag_px). speeds default to -1
    # ("unknown", always passes) when a stream doesn't record them.
    tag_pose = [
        pose.x,
        pose.y,
        pose.z,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]
    distance_m, view_angle_deg = view_quality(tag_pose)
    raw_detections.append(
        dict(
            ts=float(observation.ts),
            marker_id=int(tags["marker_id"]),
            T_cam_tag=Pose3(
                Rot3.Quaternion(
                    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z
                ),
                Point3(pose.x, pose.y, pose.z),
            ),
            reproj_px=float(tags["reproj_px"]),
            sharpness=float(tags["sharpness"]),
            tag_px=float(tags["tag_px"]),
            distance_m=float(distance_m),
            view_angle_deg=float(view_angle_deg),
            lin_speed=float(tags.get("lin_speed", -1.0)),
            ang_speed=float(tags.get("ang_speed", -1.0)),
        )
    )
gated_detections = [
    detection
    for detection in raw_detections
    if passes(detection) and detection["marker_id"] not in IGNORE_TAGS
]

# keyframes from raw odometry
odom_connection = sqlite3.connect(f"file:{DB}?mode=ro", uri=True)
odom_rows = np.array(
    list(
        odom_connection.execute(
            "select ts,pose_x,pose_y,pose_z,pose_qx,pose_qy,pose_qz,pose_qw "
            f"from {ODOM_STREAM} order by ts"
        )
    ),
    float,
)
odom_connection.close()


def row_pose(row):
    return Rot3.Quaternion(row[7], row[4], row[5], row[6]), np.array(row[1:4])


keyframe_indices = [0]
prev_rot, prev_pos = row_pose(odom_rows[0])
for row_index in range(1, len(odom_rows)):
    rot, pos = row_pose(odom_rows[row_index])
    if (
        np.linalg.norm(pos - prev_pos) > 0.5
        or np.degrees(np.linalg.norm(Rot3.Logmap(prev_rot.inverse() * rot))) > 10
    ):
        keyframe_indices.append(row_index)
        prev_rot, prev_pos = rot, pos
keyframe_poses = [row_pose(odom_rows[index]) for index in keyframe_indices]
keyframe_times = odom_rows[keyframe_indices, 0]
num_keyframes = len(keyframe_poses)

# one factor per keyframe x marker: keep the best-reproj detection in each bucket
best_per_keyframe_marker = {}
for detection in gated_detections:
    keyframe = int(np.argmin(np.abs(keyframe_times - detection["ts"])))
    key = (keyframe, detection["marker_id"])
    if (
        key not in best_per_keyframe_marker
        or detection["reproj_px"] < best_per_keyframe_marker[key]["reproj_px"]
    ):
        best_per_keyframe_marker[key] = detection

# revisit report
raw_count_by_marker, visit_times_by_marker = {}, {}
for detection in raw_detections:
    raw_count_by_marker.setdefault(detection["marker_id"], 0)
    raw_count_by_marker[detection["marker_id"]] += 1
for (_keyframe, marker_id), detection in best_per_keyframe_marker.items():
    visit_times_by_marker.setdefault(marker_id, []).append(detection["ts"])


def n_visits(times):
    times = sorted(times)
    visits = [[times[0]]]
    for time_value in times[1:]:
        (
            visits[-1].append(time_value)
            if time_value - visits[-1][-1] <= VISIT_GAP_S
            else visits.append([time_value])
        )
    return len(visits)


print(f"gates: {GATE}")
print(
    f"raw detections {len(raw_detections)} -> {len(gated_detections)} pass gates -> "
    f"{len(best_per_keyframe_marker)} keyframe-tag factors\n"
)
print(f"{'tag':>4} | {'raw viewings':>12} | {'filtered revisits':>17}")
not_revisited = []
for marker_id in sorted(raw_count_by_marker):
    visit_count = (
        n_visits(visit_times_by_marker[marker_id]) if marker_id in visit_times_by_marker else 0
    )
    flag = "" if visit_count >= 2 else "   <-- NOT REVISITED"
    print(
        f"{marker_id:>4} | {raw_count_by_marker[marker_id]:>12} | {visit_count:>10} visit(s){flag}"
    )
    if visit_count < 2:
        not_revisited.append(marker_id)
print(
    f"\ntags NOT revisited (no loop-closure constraint): {not_revisited if not_revisited else 'none'}\n"
)

# factor graph + solve
odom_noise = noiseModel.Diagonal.Variances(np.array([1e-8, 1e-8, 1e-5, 1e-4, 1e-4, 1e-6]))
gravity_anchor_noise = noiseModel.Diagonal.Variances(np.array([1e-8, 1e-8, 1e-6, 1e-8, 1e-8, 1e-8]))


# quality weighting: planar-PnP pose error grows ~quadratically with range, and reproj_px is a
# direct misfit proxy. Inflate a glimpse's covariance by (dist/REF_D)^2 * (reproj/REF_R)^2 so a
# far/oblique/blurry tag pose contributes almost nothing while close, sharp ones dominate.
REF_D, REF_R = 0.4, 1.0


def tag_noise(tag_rotation, distance_m=REF_D, reproj_px=REF_R):
    scale = max((max(distance_m, 0.2) / REF_D) ** 2 * (max(reproj_px, 0.5) / REF_R) ** 2, 0.25)
    rotation_matrix = tag_rotation.matrix()
    covariance = np.zeros((6, 6))
    covariance[:3, :3] = rotation_matrix @ np.diag([0.04, 0.04, 0.0025]) @ rotation_matrix.T
    covariance[3:, 3:] = rotation_matrix @ np.diag([0.0025, 0.0025, 0.25]) @ rotation_matrix.T
    return noiseModel.Gaussian.Covariance(covariance * scale)


print(f"building factor graph over {num_keyframes} keyframes...", flush=True)
graph = NonlinearFactorGraph()
initial_values = Values()
for keyframe_index in range(num_keyframes):
    rot, pos = keyframe_poses[keyframe_index]
    initial_values.insert(keyframe_index, Pose3(rot, Point3(pos)))
    if keyframe_index == 0:
        graph.add(PriorFactorPose3(0, Pose3(rot, Point3(pos)), gravity_anchor_noise))
    else:
        rot_prev, pos_prev = keyframe_poses[keyframe_index - 1]
        graph.add(
            BetweenFactorPose3(
                keyframe_index - 1,
                keyframe_index,
                Pose3(
                    rot_prev.inverse() * rot,
                    Point3(rot_prev.inverse().rotate(Point3(pos - pos_prev))),
                ),
                odom_noise,
            )
        )
seen_markers = set()
for (keyframe, marker_id), detection in sorted(best_per_keyframe_marker.items()):
    rot, pos = keyframe_poses[keyframe]
    keyframe_pose = Pose3(rot, Point3(pos))
    T_base_tag = T_base_optical.compose(detection["T_cam_tag"])
    if marker_id not in seen_markers:
        seen_markers.add(marker_id)
        initial_values.insert(Symbol("l", marker_id).key(), keyframe_pose.compose(T_base_tag))
    graph.add(
        BetweenFactorPose3(
            keyframe,
            Symbol("l", marker_id).key(),
            T_base_tag,
            tag_noise(T_base_tag.rotation(), detection["distance_m"], detection["reproj_px"]),
        )
    )

print("solving stage 1 (tag PGO)...", flush=True)
lm_params = LevenbergMarquardtParams()
lm_params.setMaxIterations(200)
estimate = LevenbergMarquardtOptimizer(graph, initial_values, lm_params).optimize()
raw_keyframe_poses = [
    Pose3(keyframe_poses[index][0], Point3(keyframe_poses[index][1]))
    for index in range(num_keyframes)
]

# STAGE 2: ICP loop-closure refinement (tags=macro, lidar ICP=local anchor)
# Tag PGO has pulled revisits roughly together; now ICP the lidar submaps of spatially-close,
# temporally-distant keyframe pairs to add precise 6-DOF relative constraints, then re-solve.
ICP = "--no-icp" not in sys.argv
if ICP:
    import open3d as o3d
    from scipy.spatial import cKDTree

    ICP_RADIUS_M = 4.0  # tag-corrected positions must be within this to be a revisit candidate
    ICP_MIN_DT_S = 25.0  # ...and at least this far apart in time (a real revisit, not adjacency)
    ICP_MAX_CORR_M = 0.6  # ICP correspondence distance
    ICP_VOXEL = 0.15
    ICP_FIT_MIN, ICP_RMSE_MAX = 0.45, 0.25
    SUBMAP_HALF_S = 1.0  # accumulate scans within +/- this of a keyframe time into its submap

    corrected_poses = [estimate.atPose3(index) for index in range(num_keyframes)]
    corrected_positions = np.array([np.asarray(pose.translation()) for pose in corrected_poses])

    # revisit candidate pairs
    position_tree = cKDTree(corrected_positions)
    candidate_pairs = set()
    for first_index, second_index in position_tree.query_pairs(ICP_RADIUS_M):
        if abs(keyframe_times[first_index] - keyframe_times[second_index]) >= ICP_MIN_DT_S:
            candidate_pairs.add((min(first_index, second_index), max(first_index, second_index)))
    candidate_pairs = sorted(
        candidate_pairs,
        key=lambda pair: np.linalg.norm(
            corrected_positions[pair[0]] - corrected_positions[pair[1]]
        ),
    )
    involved_keyframes = {index for pair in candidate_pairs for index in pair}
    print(
        f"ICP stage: {len(candidate_pairs)} revisit candidate pairs "
        f"over {len(involved_keyframes)} keyframes",
        flush=True,
    )

    # build per-(involved)-keyframe body-frame submaps from the input lidar (odom-registered)
    print("ICP stage: reading lidar submaps...", flush=True)
    submap_chunks = {index: [] for index in involved_keyframes}
    scan_count = 0
    submap_start_time = time.time()
    for observation in store.stream(LIDAR_STREAM):
        scan_count += 1
        if scan_count % 20000 == 0:
            print(f"  read {scan_count} scans, {time.time() - submap_start_time:.0f}s", flush=True)
        scan_ts = float(observation.ts)
        keyframe = int(np.argmin(np.abs(keyframe_times - scan_ts)))
        if keyframe not in submap_chunks or abs(keyframe_times[keyframe] - scan_ts) > SUBMAP_HALF_S:
            continue
        keyframe_rot, keyframe_pos = keyframe_poses[keyframe]
        world = world_points(observation)
        submap_chunks[keyframe].append(
            (world - keyframe_pos) @ keyframe_rot.matrix()
        )  # world -> kf-keyframe body frame
    submap_clouds = {}
    for keyframe, chunks in submap_chunks.items():
        if not chunks:
            continue
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(np.concatenate(chunks, 0).astype(np.float64))
        cloud = cloud.voxel_down_sample(ICP_VOXEL)
        cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))
        submap_clouds[keyframe] = cloud
    print(
        f"ICP stage: built {len(submap_clouds)} submaps, "
        f"registering {len(candidate_pairs)} pairs...",
        flush=True,
    )

    icp_noise = noiseModel.Robust.Create(
        noiseModel.mEstimator.Huber.Create(1.345),
        noiseModel.Diagonal.Variances(np.array([4e-4, 4e-4, 4e-4, 2.5e-3, 2.5e-3, 2.5e-3])),
    )
    accepted_count = 0
    icp_start_time = time.time()
    for pair_index, (first_index, second_index) in enumerate(candidate_pairs):
        if pair_index and pair_index % 5000 == 0:
            print(
                f"  registered {pair_index}/{len(candidate_pairs)} pairs, "
                f"{accepted_count} accepted, {time.time() - icp_start_time:.0f}s",
                flush=True,
            )
        if first_index not in submap_clouds or second_index not in submap_clouds:
            continue
        initial_guess = (
            corrected_poses[first_index].inverse() * corrected_poses[second_index]
        ).matrix()  # first<-second initial guess from tag correction
        result = o3d.pipelines.registration.registration_icp(
            submap_clouds[second_index],
            submap_clouds[first_index],
            ICP_MAX_CORR_M,
            initial_guess,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        )
        if result.fitness >= ICP_FIT_MIN and result.inlier_rmse <= ICP_RMSE_MAX:
            transform = result.transformation
            graph.add(
                BetweenFactorPose3(
                    first_index,
                    second_index,
                    Pose3(Rot3(transform[:3, :3]), Point3(transform[:3, 3])),
                    icp_noise,
                )
            )
            accepted_count += 1
    print(
        f"ICP stage: accepted {accepted_count}/{len(candidate_pairs)} loop closures "
        f"(fit>={ICP_FIT_MIN}, rmse<={ICP_RMSE_MAX}m)",
        flush=True,
    )
    if accepted_count:
        # estimate already holds every key (keyframe + landmark poses); warm-start from it
        print("solving stage 2 (tag PGO + ICP closures)...", flush=True)
        estimate = LevenbergMarquardtOptimizer(graph, estimate, lm_params).optimize()

corrections = [
    estimate.atPose3(index).compose(raw_keyframe_poses[index].inverse())
    for index in range(num_keyframes)
]
max_correction_shift = max(
    float(np.linalg.norm(np.asarray(corrections[index].translation())))
    for index in range(num_keyframes)
)
print(
    f"PGO: N={num_keyframes} keyframes, {len(best_per_keyframe_marker)} tag factors "
    f"over {len(seen_markers)} markers, "
    f"max correction shift {max_correction_shift:.1f} m",
    flush=True,
)


def correction_at(ts):
    if ts <= keyframe_times[0]:
        return corrections[0]
    if ts >= keyframe_times[-1]:
        return corrections[-1]
    insert_index = int(np.searchsorted(keyframe_times, ts))
    before_index, after_index = insert_index - 1, insert_index
    alpha = (ts - keyframe_times[before_index]) / (
        keyframe_times[after_index] - keyframe_times[before_index]
    )
    return corrections[before_index].compose(
        Pose3.Expmap(
            alpha * Pose3.Logmap(corrections[before_index].between(corrections[after_index]))
        )
    )


def pose_tuple(pose):
    translation = pose.translation()
    quaternion = pose.rotation().toQuaternion()
    return (
        translation[0],
        translation[1],
        translation[2],
        quaternion.x(),
        quaternion.y(),
        quaternion.z(),
        quaternion.w(),
    )


# Persist the PGO's internal artifacts as REAL streams (true payload types):
#   tf_deformation_nodes_corrected (DeformationNode) -- per keyframe, the raw pose (original)
#     then the optimized pose (current); a deformation-aware tf.get can replay the
#     loop-closure correction from these exactly like the online gsc_pgo stream.
#   pose_graph (Graph3D) -- the optimized keyframe nodes + sequential odom edges.
_tf_edge_id = tf_id_for("map", "odom")
_deform_name = f"tf_deformation_nodes{CORRECTED_SUFFIX}{SUFFIX}"
if _deform_name in store.list_streams():
    store.delete_stream(_deform_name)
_deform_stream = store.stream(_deform_name, DeformationNode)
for index in range(num_keyframes):
    node_ts = float(keyframe_times[index])
    for keyframe_pose in (raw_keyframe_poses[index], estimate.atPose3(index)):  # original, current
        px, py, pz, qx, qy, qz, qw = pose_tuple(keyframe_pose)
        _deform_stream.append(
            DeformationNode(
                id=index,
                tf_id=_tf_edge_id,
                pose=PoseStamped(
                    ts=node_ts, frame_id="map", position=[px, py, pz], orientation=[qx, qy, qz, qw]
                ),
            ),
            ts=node_ts,
            pose=None,
            tags={"tf_id": str(_tf_edge_id), "id": str(index)},
        )
print(f"wrote {_deform_name}: {num_keyframes} keyframes (raw+optimized)", flush=True)

_graph_name = f"pose_graph{SUFFIX}"
if _graph_name in store.list_streams():
    store.delete_stream(_graph_name)
_graph_nodes = []
for index in range(num_keyframes):
    px, py, pz, qx, qy, qz, qw = pose_tuple(estimate.atPose3(index))
    _graph_nodes.append(
        Graph3D.Node3D(
            pose=PoseStamped(
                ts=float(keyframe_times[index]),
                frame_id="map",
                position=[px, py, pz],
                orientation=[qx, qy, qz, qw],
            ),
            id=index,
        )
    )
_graph_edges = [
    Graph3D.Edge(index, index + 1, float(keyframe_times[index + 1]))
    for index in range(num_keyframes - 1)
]
_graph_ts = float(keyframe_times[-1])
store.stream(_graph_name, Graph3D).append(
    Graph3D(ts=_graph_ts, nodes=_graph_nodes, edges=_graph_edges), ts=_graph_ts, pose=None
)
print(f"wrote {_graph_name}: {num_keyframes} nodes, {len(_graph_edges)} edges", flush=True)

if WHAT in ("odom", "both"):
    out_name = f"{ODOM_STREAM}{CORRECTED_SUFFIX}{SUFFIX}"
    if out_name in store.list_streams():
        store.delete_stream(out_name)
    out_stream = store.stream(out_name, Odometry)
    print(f"writing {out_name} ({len(odom_rows)} poses)...", flush=True)
    written_count = 0
    write_start_time = time.time()
    for row in odom_rows:
        ts = float(row[0])
        corrected_pose = correction_at(ts).compose(
            Pose3(Rot3.Quaternion(row[7], row[4], row[5], row[6]), Point3(row[1], row[2], row[3]))
        )
        x, y, z, qx, qy, qz, qw = pose_tuple(corrected_pose)
        out_stream.append(
            Odometry(
                ts=ts,
                frame_id="odom",
                child_frame_id="base_link",
                pose=Pose(x, y, z, qx, qy, qz, qw),
            ),
            ts=ts,
            pose=(x, y, z, qx, qy, qz, qw),
        )
        written_count += 1
        if written_count % 20000 == 0:
            print(
                f"  {written_count}/{len(odom_rows)} poses, {time.time() - write_start_time:.0f}s",
                flush=True,
            )
    print(
        f"wrote {out_name}: {len(odom_rows)} poses in {time.time() - write_start_time:.0f}s",
        flush=True,
    )

if WHAT in ("lidar", "both"):
    out_name = f"{LIDAR_STREAM}{CORRECTED_SUFFIX}{SUFFIX}"
    odom_times = odom_rows[:, 0]

    def base_pose(ts):
        index = int(np.searchsorted(odom_times, ts))
        index = min(max(index, 0), len(odom_rows) - 1)
        if index > 0 and abs(odom_times[index - 1] - ts) < abs(odom_times[index] - ts):
            index -= 1
        row = odom_rows[index]
        return Pose3(
            Rot3.Quaternion(row[7], row[4], row[5], row[6]), Point3(row[1], row[2], row[3])
        )

    if out_name in store.list_streams():
        store.delete_stream(out_name)
    out_stream = store.stream(out_name, PointCloud2)

    # The db stream stays per-scan, but the .pc2.lcm is ONE aggregated cloud (voxel-downsampled +
    # statistical-outlier-removed), not 5184 per-scan events. Intensity rides through open3d's
    # voxel averaging via the color channel. Chunks are collapsed every CHUNK scans to bound memory.
    if WRITE_LCM:
        import open3d as o3d

    CHUNK = 1000
    aggregated_points, aggregated_intensities = [], []  # incrementally voxel-downsampled chunks
    buffered_points, buffered_intensities = [], []
    have_intensities = False

    def collapse(points_chunks, intensity_chunks, voxel):
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(np.concatenate(points_chunks).astype(np.float64))
        carry_intensities = bool(intensity_chunks)
        if carry_intensities:
            intensity_column = np.concatenate(intensity_chunks).astype(np.float64)[:, None]
            cloud.colors = o3d.utility.Vector3dVector(np.repeat(intensity_column, 3, axis=1))
        cloud = cloud.voxel_down_sample(voxel)
        downsampled_points = np.asarray(cloud.points, np.float32)
        downsampled_intensities = (
            np.asarray(cloud.colors, np.float32)[:, 0] if carry_intensities else None
        )
        return downsampled_points, downsampled_intensities

    print(f"writing {out_name} (corrected lidar)...", flush=True)
    written_count = 0
    write_start_time = time.time()
    for observation in store.stream(LIDAR_STREAM):
        ts = float(observation.ts)
        correction = correction_at(ts)
        rotation_matrix = correction.rotation().matrix()
        translation = np.asarray(correction.translation())
        points = world_points(observation)
        intensities = observation.data.intensities_f32()
        corrected_points = (points @ rotation_matrix.T + translation).astype(np.float32)
        cloud_msg = PointCloud2.from_numpy(
            corrected_points,
            frame_id="odom",
            intensities=(np.asarray(intensities) if intensities is not None else None),
        )
        cloud_msg.ts = ts  # stamp the cloud (lcm_encode needs a non-None ts)
        out_stream.append(cloud_msg, ts=ts, pose=pose_tuple(correction.compose(base_pose(ts))))
        if WRITE_LCM:
            buffered_points.append(corrected_points)
            if intensities is not None:
                have_intensities = True
                buffered_intensities.append(np.asarray(intensities, np.float32))
            if len(buffered_points) >= CHUNK:
                downsampled_points, downsampled_intensities = collapse(
                    buffered_points, buffered_intensities if have_intensities else [], LCM_VOXEL
                )
                aggregated_points.append(downsampled_points)
                if downsampled_intensities is not None:
                    aggregated_intensities.append(downsampled_intensities)
                buffered_points, buffered_intensities = [], []
        written_count += 1
        if written_count % 2000 == 0:
            print(f"  {written_count} scans, {time.time() - write_start_time:.0f}s", flush=True)
    print(
        f"wrote {out_name}: {written_count} scans in {time.time() - write_start_time:.0f}s",
        flush=True,
    )

    if WRITE_LCM:
        if buffered_points:  # flush remainder
            downsampled_points, downsampled_intensities = collapse(
                buffered_points, buffered_intensities if have_intensities else [], LCM_VOXEL
            )
            aggregated_points.append(downsampled_points)
            if downsampled_intensities is not None:
                aggregated_intensities.append(downsampled_intensities)
        # final unified voxel pass over the per-chunk results, then statistical outlier removal
        merged_intensity_chunks = aggregated_intensities if have_intensities else []
        downsampled_points, downsampled_intensities = collapse(
            aggregated_points, merged_intensity_chunks, LCM_VOXEL
        )
        print(
            f"aggregating .pc2.lcm: {len(downsampled_points):,} pts after voxel, "
            f"removing outliers...",
            flush=True,
        )
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(downsampled_points.astype(np.float64))
        if downsampled_intensities is not None:
            cloud.colors = o3d.utility.Vector3dVector(
                np.repeat(downsampled_intensities.astype(np.float64)[:, None], 3, axis=1)
            )
        cloud, _keep = cloud.remove_statistical_outlier(LCM_OUTLIER_NN, LCM_OUTLIER_STD)
        merged_xyz = np.asarray(cloud.points, np.float32)
        merged_inten = (
            np.asarray(cloud.colors, np.float32)[:, 0]
            if downsampled_intensities is not None
            else None
        )
        merged = PointCloud2.from_numpy(merged_xyz, frame_id="odom", intensities=merged_inten)
        merged.ts = float(odom_times[0])
        lcm_path = REC / f"{out_name}.pc2.lcm"
        lcm_path.write_bytes(merged.lcm_encode())
        print(
            f"wrote {lcm_path}: 1 aggregated cloud, {len(merged_xyz):,} pts "
            f"(voxel {LCM_VOXEL} m, outlier nn={LCM_OUTLIER_NN}/std={LCM_OUTLIER_STD})",
            flush=True,
        )

    if WRITE_ACCUM:
        # Raycast-accumulated global maps. Original: the input lidar (sensor-frame,
        # tf-registered; already-world Point-LIO clouds get their ray origin from
        # the odom trajectory). Corrected: the gt stream just written (already world in "odom").
        accumulate_stream(
            store,
            LIDAR_STREAM,
            f"{LIDAR_STREAM}_accumulated",
            world_frame=WORLD_FRAME,
            lidar_frame=LIDAR_FRAME,
            use_tf=USE_TF,
            voxel_size=ACCUM_VOXEL,
            max_range=ACCUM_MAX_RANGE,
            origin_stream=ODOM_STREAM,
        )
        accumulate_stream(
            store,
            out_name,
            f"{out_name}_accumulated",
            world_frame="odom",
            voxel_size=ACCUM_VOXEL,
            max_range=ACCUM_MAX_RANGE,
        )

# build + open the comparison rrd
if OPEN_RRD and WHAT in ("lidar", "both"):
    import subprocess

    from dimos.navigation.jnav.components.loop_closure.gsc_pgo.scripts import make_rrd

    print("building comparison rrd...", flush=True)
    rrd_path = make_rrd.build(
        REC, lidar_stream=LIDAR_STREAM, odom_stream=ODOM_STREAM, tag_stream=RAW_STREAM
    )
    rerun_bin = Path(sys.executable).parent / "rerun"
    if rerun_bin.exists():
        subprocess.Popen([str(rerun_bin), str(rrd_path)])
        print(f"opened {rrd_path}", flush=True)
    else:
        print(f"rerun binary not found at {rerun_bin}; open manually: rerun {rrd_path}", flush=True)
