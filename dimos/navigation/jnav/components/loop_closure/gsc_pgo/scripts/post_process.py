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
import json
from pathlib import Path
import subprocess
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
import open3d as o3d
from scipy.spatial import cKDTree

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.jnav.components.loop_closure.gsc_pgo.scripts import make_rrd
from dimos.navigation.jnav.components.loop_closure.gsc_pgo.scripts.make_rrd import (
    pose3_from_xyzquat,
)
from dimos.navigation.jnav.msgs.DeformationNode import DeformationNode, tf_id_for
from dimos.navigation.jnav.msgs.Graph3D import Graph3D
from dimos.navigation.jnav.utils import recording_db as rdb
from dimos.navigation.jnav.utils.apriltags import (
    ensure_raw_tag_stream,
    filter_glimpses,
    read_raw_tag_stream,
)
from dimos.navigation.jnav.utils.recording_tf import (
    RecordingTF,
    default_odom_edge,
    resolve_streams,
    world_register,
)
from dimos.navigation.jnav.utils.trajectory_metrics import (
    nearest_index,
    thin_pairs_by_path_section,
)

# keyframe selection
KEYFRAME_TRANSLATION_M = 0.5
KEYFRAME_ROTATION_DEG = 10.0

# tag revisit report
VISIT_GAP_S = 30.0
MIN_VISITS_FOR_LOOP = 2

# factor graph
LM_MAX_ITERATIONS = 200
ODOM_NOISE = noiseModel.Diagonal.Variances(np.array([1e-8, 1e-8, 1e-5, 1e-4, 1e-4, 1e-6]))
GRAVITY_ANCHOR_NOISE = noiseModel.Diagonal.Variances(np.array([1e-8, 1e-8, 1e-6, 1e-8, 1e-8, 1e-8]))
# Quality weighting: planar-PnP pose error grows ~quadratically with range, and reproj_px is a
# direct misfit proxy. A glimpse's covariance is inflated by (dist/REF)^2 * (reproj/REF)^2 so
# far/oblique/blurry tags contribute almost nothing while close, sharp ones dominate.
REF_DISTANCE_M = 0.4
REF_REPROJ_PX = 1.0

# ICP loop closures
ICP_RADIUS_M = 4.0  # tag-corrected positions must be within this to be a revisit candidate
ICP_MIN_DT_S = 25.0  # ...and at least this far apart in time (a real revisit, not adjacency)
ICP_MAX_CORR_M = 0.6  # ICP correspondence distance
ICP_VOXEL_M = 0.15
ICP_FITNESS_MIN = 0.45
ICP_RMSE_MAX_M = 0.25
SUBMAP_HALF_S = 1.0  # accumulate scans within +/- this of a keyframe time into its submap
ICP_HUBER_DELTA = 1.345
ICP_NOISE = noiseModel.Robust.Create(
    noiseModel.mEstimator.Huber.Create(ICP_HUBER_DELTA),
    noiseModel.Diagonal.Variances(np.array([4e-4, 4e-4, 4e-4, 2.5e-3, 2.5e-3, 2.5e-3])),
)

# aggregated .pc2.lcm
LCM_CHUNK_SCANS = 1000  # collapse buffered scans this often to bound memory
LCM_OUTLIER_NN = 20  # statistical outlier removal: neighbor count
LCM_OUTLIER_STD = 2.0  # ...and std-ratio threshold (lower = more aggressive)

# progress logging cadence
ODOM_LOG_EVERY = 20000
SCAN_LOG_EVERY = 2000


def resolve_recording(rec_arg):
    """--rec is a recording dir (mem2.db + sidecar) or a bare .db; return (dir, db)."""
    rec_path = Path(rec_arg).expanduser()
    if rec_path.is_dir():
        return rec_path, rec_path / "mem2.db"
    return rec_path.parent, rec_path


def load_optical_transform(rec_dir):
    """Base<-optical camera transform + parsed intrinsics, or (identity, None) if absent."""
    intrinsics_path = rec_dir / "camera_intrinsics.json"
    if not intrinsics_path.exists():
        return Pose3(), None
    intrinsics = json.loads(intrinsics_path.read_text())
    return pose3_from_xyzquat(np.array(intrinsics["optical_in_base"], float)), intrinsics


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


def select_keyframes(odom_rows):
    """Keyframe indices where the robot moved past the translation/rotation thresholds."""
    indices = [0]
    previous = pose3_from_xyzquat(odom_rows[0][1:])
    for row_index in range(1, len(odom_rows)):
        current = pose3_from_xyzquat(odom_rows[row_index][1:])
        moved = np.linalg.norm(
            np.asarray(current.translation()) - np.asarray(previous.translation())
        )
        turned = np.degrees(np.linalg.norm(Pose3.Logmap(previous.between(current))[:3]))
        if moved > KEYFRAME_TRANSLATION_M or turned > KEYFRAME_ROTATION_DEG:
            indices.append(row_index)
            previous = current
    poses = [pose3_from_xyzquat(odom_rows[index][1:]) for index in indices]
    times = odom_rows[indices, 0]
    return indices, poses, times


def best_factor_per_keyframe_marker(detections, keyframe_times):
    """One factor per (keyframe, marker): the filtered detection with the lowest reproj error."""
    best = {}
    for detection in detections:
        keyframe = int(np.argmin(np.abs(keyframe_times - detection["ts"])))
        key = (keyframe, detection["marker_id"])
        if key not in best or detection["reproj_px"] < best[key]["reproj_px"]:
            best[key] = detection
    return best


def count_visits(times):
    """Number of temporally-separated visits in a sorted-able list of timestamps."""
    times = sorted(times)
    visits = [[times[0]]]
    for value in times[1:]:
        if value - visits[-1][-1] <= VISIT_GAP_S:
            visits[-1].append(value)
        else:
            visits.append([value])
    return len(visits)


def report_revisits(detections, best_factors):
    """Print per-marker raw viewings + filtered revisits, flagging tags with no loop closure."""
    raw_by_marker = {}
    for detection in detections:
        raw_by_marker[detection["marker_id"]] = raw_by_marker.get(detection["marker_id"], 0) + 1
    visit_times = {}
    for (_keyframe, marker_id), detection in best_factors.items():
        visit_times.setdefault(marker_id, []).append(detection["ts"])
    print(f"{'tag':>4} | {'raw viewings':>12} | {'filtered revisits':>17}")
    not_revisited = []
    for marker_id in sorted(raw_by_marker):
        visits = count_visits(visit_times[marker_id]) if marker_id in visit_times else 0
        flag = "" if visits >= MIN_VISITS_FOR_LOOP else "   <-- NOT REVISITED"
        print(f"{marker_id:>4} | {raw_by_marker[marker_id]:>12} | {visits:>10} visit(s){flag}")
        if visits < MIN_VISITS_FOR_LOOP:
            not_revisited.append(marker_id)
    print(f"\ntags with no loop-closure constraint: {not_revisited or 'none'}\n")


def tag_noise(tag_rotation, distance_m, reproj_px):
    """Range/reproj-inflated Gaussian covariance for a single AprilTag landmark factor."""
    scale = max(
        (max(distance_m, 0.2) / REF_DISTANCE_M) ** 2 * (max(reproj_px, 0.5) / REF_REPROJ_PX) ** 2,
        0.25,
    )
    rotation_matrix = tag_rotation.matrix()
    covariance = np.zeros((6, 6))
    covariance[:3, :3] = rotation_matrix @ np.diag([0.04, 0.04, 0.0025]) @ rotation_matrix.T
    covariance[3:, 3:] = rotation_matrix @ np.diag([0.0025, 0.0025, 0.25]) @ rotation_matrix.T
    return noiseModel.Gaussian.Covariance(covariance * scale)


def build_tag_graph(keyframe_poses, best_factors, base_optical):
    """Stage-1 graph: sequential odom between-factors + AprilTag landmark factors."""
    graph = NonlinearFactorGraph()
    values = Values()
    for index, pose in enumerate(keyframe_poses):
        values.insert(index, pose)
        if index == 0:
            graph.add(PriorFactorPose3(0, pose, GRAVITY_ANCHOR_NOISE))
        else:
            graph.add(
                BetweenFactorPose3(
                    index - 1, index, keyframe_poses[index - 1].between(pose), ODOM_NOISE
                )
            )
    seen_markers = set()
    for (keyframe, marker_id), detection in sorted(best_factors.items()):
        base_tag = base_optical.compose(pose3_from_xyzquat(detection["t_cam_marker"]))
        landmark_key = Symbol("l", marker_id).key()
        if marker_id not in seen_markers:
            seen_markers.add(marker_id)
            values.insert(landmark_key, keyframe_poses[keyframe].compose(base_tag))
        graph.add(
            BetweenFactorPose3(
                keyframe,
                landmark_key,
                base_tag,
                tag_noise(base_tag.rotation(), detection["distance_m"], detection["reproj_px"]),
            )
        )
    return graph, values, seen_markers


def solve(graph, values):
    params = LevenbergMarquardtParams()
    params.setMaxIterations(LM_MAX_ITERATIONS)
    return LevenbergMarquardtOptimizer(graph, values, params).optimize()


def build_submaps(
    store, lidar_stream, keyframe_indices, keyframe_poses, keyframe_times, world_points
):
    """Body-frame, voxel-downsampled, normal-estimated lidar submap per involved keyframe."""
    chunks = {index: [] for index in keyframe_indices}
    scan_count = 0
    started = time.time()
    for observation in store.stream(lidar_stream):
        scan_count += 1
        if scan_count % ODOM_LOG_EVERY == 0:
            print(f"  read {scan_count} scans, {time.time() - started:.0f}s", flush=True)
        scan_ts = float(observation.ts)
        keyframe = int(np.argmin(np.abs(keyframe_times - scan_ts)))
        if keyframe not in chunks or abs(keyframe_times[keyframe] - scan_ts) > SUBMAP_HALF_S:
            continue
        pose = keyframe_poses[keyframe]
        world = world_points(observation)
        chunks[keyframe].append((world - np.asarray(pose.translation())) @ pose.rotation().matrix())
    clouds = {}
    for keyframe, keyframe_chunks in chunks.items():
        if not keyframe_chunks:
            continue
        cloud = o3d.geometry.PointCloud()
        cloud.points = o3d.utility.Vector3dVector(
            np.concatenate(keyframe_chunks, 0).astype(np.float64)
        )
        cloud = cloud.voxel_down_sample(ICP_VOXEL_M)
        cloud.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))
        clouds[keyframe] = cloud
    return clouds


def add_icp_closures(
    graph,
    estimate,
    store,
    lidar_stream,
    keyframe_poses,
    keyframe_times,
    world_points,
    closure_spacing,
):
    """Stage 2: register spatially-close / temporally-distant submaps, add loop factors."""
    num_keyframes = len(keyframe_poses)
    corrected_poses = [estimate.atPose3(index) for index in range(num_keyframes)]
    positions = np.array([np.asarray(pose.translation()) for pose in corrected_poses])

    candidate_pairs = set()
    for first, second in cKDTree(positions).query_pairs(ICP_RADIUS_M):
        if abs(keyframe_times[first] - keyframe_times[second]) >= ICP_MIN_DT_S:
            candidate_pairs.add((min(first, second), max(first, second)))
    candidate_pairs = sorted(
        candidate_pairs, key=lambda pair: np.linalg.norm(positions[pair[0]] - positions[pair[1]])
    )
    total_candidates = len(candidate_pairs)
    candidate_pairs = thin_pairs_by_path_section(candidate_pairs, positions, closure_spacing)
    print(
        f"ICP stage: thinned {total_candidates} -> {len(candidate_pairs)} pairs "
        f"(one per {closure_spacing:g} m of path)",
        flush=True,
    )
    involved = {index for pair in candidate_pairs for index in pair}
    print(
        f"ICP stage: {len(candidate_pairs)} candidate pairs over {len(involved)} keyframes",
        flush=True,
    )
    if not candidate_pairs:
        return 0

    print("ICP stage: reading lidar submaps...", flush=True)
    clouds = build_submaps(
        store, lidar_stream, involved, keyframe_poses, keyframe_times, world_points
    )
    print(
        f"ICP stage: built {len(clouds)} submaps, registering {len(candidate_pairs)} pairs...",
        flush=True,
    )

    accepted = 0
    started = time.time()
    for pair_index, (first, second) in enumerate(candidate_pairs):
        if pair_index and pair_index % 5000 == 0:
            print(
                f"  {pair_index}/{len(candidate_pairs)} pairs, {accepted} accepted, {time.time() - started:.0f}s",
                flush=True,
            )
        if first not in clouds or second not in clouds:
            continue
        initial_guess = (corrected_poses[first].inverse() * corrected_poses[second]).matrix()
        result = o3d.pipelines.registration.registration_icp(
            clouds[second],
            clouds[first],
            ICP_MAX_CORR_M,
            initial_guess,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        )
        if result.fitness >= ICP_FITNESS_MIN and result.inlier_rmse <= ICP_RMSE_MAX_M:
            transform = result.transformation
            graph.add(
                BetweenFactorPose3(
                    first,
                    second,
                    Pose3(Rot3(transform[:3, :3]), Point3(transform[:3, 3])),
                    ICP_NOISE,
                )
            )
            accepted += 1
    print(f"ICP stage: accepted {accepted}/{len(candidate_pairs)} loop closures", flush=True)
    return accepted


def interpolate_correction(keyframe_times, corrections, ts):
    """SE(3) interpolation of the keyframe corrections at an arbitrary timestamp."""
    if ts <= keyframe_times[0]:
        return corrections[0]
    if ts >= keyframe_times[-1]:
        return corrections[-1]
    after = int(np.searchsorted(keyframe_times, ts))
    before = after - 1
    alpha = (ts - keyframe_times[before]) / (keyframe_times[after] - keyframe_times[before])
    step = Pose3.Expmap(alpha * Pose3.Logmap(corrections[before].between(corrections[after])))
    return corrections[before].compose(step)


def write_deformation_nodes(store, name, keyframe_times, raw_poses, estimate):
    """Per keyframe: the raw pose then the optimized pose, so tf.get can replay the correction."""
    if name in store.list_streams():
        store.delete_stream(name)
    stream = store.stream(name, DeformationNode)
    edge_id = tf_id_for("map", "odom")
    for index in range(len(keyframe_times)):
        node_ts = float(keyframe_times[index])
        for pose in (raw_poses[index], estimate.atPose3(index)):
            px, py, pz, qx, qy, qz, qw = pose_tuple(pose)
            stream.append(
                DeformationNode(
                    id=index,
                    tf_id=edge_id,
                    pose=PoseStamped(
                        ts=node_ts,
                        frame_id="map",
                        position=[px, py, pz],
                        orientation=[qx, qy, qz, qw],
                    ),
                ),
                ts=node_ts,
                pose=None,
                tags={"tf_id": str(edge_id), "id": str(index)},
            )
    print(f"wrote {name}: {len(keyframe_times)} keyframes (raw+optimized)", flush=True)


def write_pose_graph(store, name, keyframe_times, estimate):
    """The optimized keyframe nodes + sequential odom edges as a Graph3D."""
    if name in store.list_streams():
        store.delete_stream(name)
    num_keyframes = len(keyframe_times)
    nodes = []
    for index in range(num_keyframes):
        px, py, pz, qx, qy, qz, qw = pose_tuple(estimate.atPose3(index))
        nodes.append(
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
    edges = [
        Graph3D.Edge(index, index + 1, float(keyframe_times[index + 1]))
        for index in range(num_keyframes - 1)
    ]
    graph_ts = float(keyframe_times[-1])
    store.stream(name, Graph3D).append(
        Graph3D(ts=graph_ts, nodes=nodes, edges=edges), ts=graph_ts, pose=None
    )
    print(f"wrote {name}: {num_keyframes} nodes, {len(edges)} edges", flush=True)


def write_corrected_odom(store, name, odom_rows, keyframe_times, corrections):
    if name in store.list_streams():
        store.delete_stream(name)
    stream = store.stream(name, Odometry)
    print(f"writing {name} ({len(odom_rows)} poses)...", flush=True)
    started = time.time()
    for count, row in enumerate(odom_rows, 1):
        ts = float(row[0])
        corrected = interpolate_correction(keyframe_times, corrections, ts).compose(
            pose3_from_xyzquat(row[1:])
        )
        x, y, z, qx, qy, qz, qw = pose_tuple(corrected)
        stream.append(
            Odometry(
                ts=ts,
                frame_id="odom",
                child_frame_id="base_link",
                pose=Pose(x, y, z, qx, qy, qz, qw),
            ),
            ts=ts,
            pose=(x, y, z, qx, qy, qz, qw),
        )
        if count % ODOM_LOG_EVERY == 0:
            print(f"  {count}/{len(odom_rows)} poses, {time.time() - started:.0f}s", flush=True)
    print(f"wrote {name}: {len(odom_rows)} poses in {time.time() - started:.0f}s", flush=True)


def voxel_downsample(points_chunks, intensity_chunks, voxel):
    """Merge chunks, voxel-downsample, carrying intensity through open3d's color channel."""
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(np.concatenate(points_chunks).astype(np.float64))
    carry = bool(intensity_chunks)
    if carry:
        column = np.concatenate(intensity_chunks).astype(np.float64)[:, None]
        cloud.colors = o3d.utility.Vector3dVector(np.repeat(column, 3, axis=1))
    cloud = cloud.voxel_down_sample(voxel)
    points = np.asarray(cloud.points, np.float32)
    intensities = np.asarray(cloud.colors, np.float32)[:, 0] if carry else None
    return points, intensities


def write_corrected_lidar(
    store,
    name,
    lidar_stream,
    odom_rows,
    keyframe_times,
    corrections,
    world_points,
    lcm_path,
    lcm_voxel,
):
    """Per-scan corrected clouds into the db; if lcm_path, also one aggregated .pc2.lcm."""
    if name in store.list_streams():
        store.delete_stream(name)
    stream = store.stream(name, PointCloud2)
    odom_times = odom_rows[:, 0]
    aggregated_points, aggregated_intensities = [], []
    buffered_points, buffered_intensities = [], []
    have_intensities = False

    print(f"writing {name} (corrected lidar)...", flush=True)
    started = time.time()
    scan_count = 0
    for observation in store.stream(lidar_stream):
        scan_count += 1
        ts = float(observation.ts)
        correction = interpolate_correction(keyframe_times, corrections, ts)
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
        cloud_msg.ts = ts
        stream.append(
            cloud_msg,
            ts=ts,
            pose=pose_tuple(
                correction.compose(pose3_from_xyzquat(odom_rows[nearest_index(odom_times, ts)][1:]))
            ),
        )
        if lcm_path:
            buffered_points.append(corrected_points)
            if intensities is not None:
                have_intensities = True
                buffered_intensities.append(np.asarray(intensities, np.float32))
            if len(buffered_points) >= LCM_CHUNK_SCANS:
                points_out, intensities_out = voxel_downsample(
                    buffered_points, buffered_intensities if have_intensities else [], lcm_voxel
                )
                aggregated_points.append(points_out)
                if intensities_out is not None:
                    aggregated_intensities.append(intensities_out)
                buffered_points, buffered_intensities = [], []
        if scan_count % SCAN_LOG_EVERY == 0:
            print(f"  {scan_count} scans, {time.time() - started:.0f}s", flush=True)
    print(f"wrote {name}: {scan_count} scans in {time.time() - started:.0f}s", flush=True)

    if lcm_path:
        if buffered_points:
            points_out, intensities_out = voxel_downsample(
                buffered_points, buffered_intensities if have_intensities else [], lcm_voxel
            )
            aggregated_points.append(points_out)
            if intensities_out is not None:
                aggregated_intensities.append(intensities_out)
        write_aggregated_lcm(
            aggregated_points,
            aggregated_intensities if have_intensities else [],
            lcm_voxel,
            lcm_path,
            float(odom_times[0]),
        )


def write_aggregated_lcm(points_chunks, intensity_chunks, voxel, lcm_path, stamp):
    """Final unified voxel pass + statistical outlier removal into a single .pc2.lcm cloud."""
    points, intensities = voxel_downsample(points_chunks, intensity_chunks, voxel)
    print(
        f"aggregating .pc2.lcm: {len(points):,} pts after voxel, removing outliers...", flush=True
    )
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    if intensities is not None:
        cloud.colors = o3d.utility.Vector3dVector(
            np.repeat(intensities.astype(np.float64)[:, None], 3, axis=1)
        )
    cloud, _keep = cloud.remove_statistical_outlier(LCM_OUTLIER_NN, LCM_OUTLIER_STD)
    merged_xyz = np.asarray(cloud.points, np.float32)
    merged_intensities = (
        np.asarray(cloud.colors, np.float32)[:, 0] if intensities is not None else None
    )
    merged = PointCloud2.from_numpy(merged_xyz, frame_id="odom", intensities=merged_intensities)
    merged.ts = stamp
    lcm_path.write_bytes(merged.lcm_encode())
    print(
        f"wrote {lcm_path}: 1 aggregated cloud, {len(merged_xyz):,} pts (voxel {voxel} m)",
        flush=True,
    )


def raycast_accumulate(
    store, in_stream, store_tf, world_frame, lidar_frame, origin_lookup, voxel, max_range
):
    """Raycast ``in_stream`` into one ``<in_stream>_accumulated`` cloud, carving free space
    along every ray so dynamic objects and registration ghosts get cleared, not smeared."""
    out_stream = f"{in_stream}_accumulated"
    # imported here: needs the dimos_voxel_ray_tracing native ext, absent on plain CI runners
    from dimos.mapping.ray_tracing.voxel_map import VoxelRayMapper

    mapper = VoxelRayMapper(voxel_size=voxel, max_range=max_range)
    scan_count = 0
    last_ts = 0.0
    start_time = time.time()
    for observation in store.stream(in_stream):
        points, origin = world_register(
            observation, store_tf, world_frame, lidar_frame, origin_lookup
        )
        if origin is None or not len(points):
            continue
        mapper.add_frame(points, origin)
        last_ts = float(observation.ts)
        scan_count += 1
        if scan_count % SCAN_LOG_EVERY == 0:
            print(
                f"  {scan_count} scans, {mapper.voxel_count():,} voxels, "
                f"{time.time() - start_time:.0f}s",
                flush=True,
            )
    accumulated = np.asarray(mapper.global_map(), np.float32)
    if out_stream in store.list_streams():
        store.delete_stream(out_stream)
    cloud = PointCloud2.from_numpy(accumulated, frame_id=world_frame, timestamp=last_ts)
    store.stream(out_stream, PointCloud2).append(cloud, ts=last_ts, pose=None)
    print(
        f"wrote {out_stream}: {len(accumulated):,} pts from {scan_count} scans "
        f"in {time.time() - start_time:.0f}s",
        flush=True,
    )


def build_and_open_rrd(db_path, lidar_stream, odom_stream, tag_stream):
    print("building comparison rrd...", flush=True)
    rrd_path = make_rrd.build(
        db_path, lidar_stream=lidar_stream, odom_stream=odom_stream, tag_stream=tag_stream
    )
    rerun_bin = Path(sys.executable).parent / "rerun"
    if rerun_bin.exists():
        subprocess.Popen([str(rerun_bin), str(rrd_path)])
        print(f"opened {rrd_path}", flush=True)
    else:
        print(f"rerun not found at {rerun_bin}; open manually: rerun {rrd_path}", flush=True)


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
    store = rdb.store(db_path)

    # resolve stream/frame defaults from what the recording actually has
    odom_stream, lidar_stream = resolve_streams(store.list_streams(), args.odom, args.lidar)
    odom_tf = args.odom_tf or default_odom_edge(store, odom_stream)
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
    odom_rows = rdb.odometry_rows(db_path, odom_stream)
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


if __name__ == "__main__":
    main()
