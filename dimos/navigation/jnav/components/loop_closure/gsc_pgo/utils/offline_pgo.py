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

"""GTSAM tag-PGO + ICP-loop-closure solve pipeline for recording post-processing."""

from __future__ import annotations

from collections.abc import Callable
import time
from typing import TYPE_CHECKING, Any

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

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.jnav.components.loop_closure.gsc_pgo.scripts.make_rrd import (
    pose3_from_xyzquat,
)
from dimos.navigation.jnav.utils.trajectory_metrics import thin_pairs_by_path_section

if TYPE_CHECKING:
    from dimos.memory2.store.base import Store
    from dimos.memory2.type.observation import Observation

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

# progress logging cadence
ODOM_LOG_EVERY = 20000


def select_keyframes(odom_rows: np.ndarray) -> tuple[list[int], list[Pose3], np.ndarray]:
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


def best_factor_per_keyframe_marker(
    detections: list[dict[str, Any]], keyframe_times: np.ndarray
) -> dict[tuple[int, int], dict[str, Any]]:
    """One factor per (keyframe, marker): the filtered detection with the lowest reproj error."""
    best: dict[tuple[int, int], dict[str, Any]] = {}
    for detection in detections:
        keyframe = int(np.argmin(np.abs(keyframe_times - detection["ts"])))
        key = (keyframe, detection["marker_id"])
        if key not in best or detection["reproj_px"] < best[key]["reproj_px"]:
            best[key] = detection
    return best


def count_visits(times: list[float]) -> int:
    """Number of temporally-separated visits in a sorted-able list of timestamps."""
    times = sorted(times)
    visits = [[times[0]]]
    for value in times[1:]:
        if value - visits[-1][-1] <= VISIT_GAP_S:
            visits[-1].append(value)
        else:
            visits.append([value])
    return len(visits)


def report_revisits(
    detections: list[dict[str, Any]], best_factors: dict[tuple[int, int], dict[str, Any]]
) -> None:
    """Print per-marker raw viewings + filtered revisits, flagging tags with no loop closure."""
    raw_by_marker: dict[int, int] = {}
    for detection in detections:
        raw_by_marker[detection["marker_id"]] = raw_by_marker.get(detection["marker_id"], 0) + 1
    visit_times: dict[int, list[float]] = {}
    for (_keyframe, marker_id), detection in best_factors.items():
        visit_times.setdefault(marker_id, []).append(detection["ts"])
    print(f"{'tag':>4} | {'raw viewings':>12} | {'filtered revisits':>17}")
    not_revisited: list[int] = []
    for marker_id in sorted(raw_by_marker):
        visits = count_visits(visit_times[marker_id]) if marker_id in visit_times else 0
        flag = "" if visits >= MIN_VISITS_FOR_LOOP else "   <-- NOT REVISITED"
        print(f"{marker_id:>4} | {raw_by_marker[marker_id]:>12} | {visits:>10} visit(s){flag}")
        if visits < MIN_VISITS_FOR_LOOP:
            not_revisited.append(marker_id)
    print(f"\ntags with no loop-closure constraint: {not_revisited or 'none'}\n")


def tag_noise(tag_rotation: Rot3, distance_m: float, reproj_px: float) -> noiseModel.Gaussian:
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


def build_tag_graph(
    keyframe_poses: list[Pose3],
    best_factors: dict[tuple[int, int], dict[str, Any]],
    base_optical: Pose3,
) -> tuple[NonlinearFactorGraph, Values, set[int]]:
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


def solve(graph: NonlinearFactorGraph, values: Values) -> Values:
    params = LevenbergMarquardtParams()
    params.setMaxIterations(LM_MAX_ITERATIONS)
    return LevenbergMarquardtOptimizer(graph, values, params).optimize()


def build_submaps(
    store: Store,
    lidar_stream: str,
    keyframe_indices: set[int],
    keyframe_poses: list[Pose3],
    keyframe_times: np.ndarray,
    world_points: Callable[[Observation[PointCloud2]], np.ndarray],
) -> dict[int, o3d.geometry.PointCloud]:
    """Body-frame, voxel-downsampled, normal-estimated lidar submap per involved keyframe."""
    chunks: dict[int, list[np.ndarray]] = {index: [] for index in keyframe_indices}
    scan_count = 0
    started = time.time()
    for observation in store.stream(lidar_stream, PointCloud2):
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
    clouds: dict[int, o3d.geometry.PointCloud] = {}
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
    graph: NonlinearFactorGraph,
    estimate: Values,
    store: Store,
    lidar_stream: str,
    keyframe_poses: list[Pose3],
    keyframe_times: np.ndarray,
    world_points: Callable[[Observation[PointCloud2]], np.ndarray],
    closure_spacing: float,
) -> int:
    """Stage 2: register spatially-close / temporally-distant submaps, add loop factors."""
    num_keyframes = len(keyframe_poses)
    corrected_poses = [estimate.atPose3(index) for index in range(num_keyframes)]
    positions = np.array([np.asarray(pose.translation()) for pose in corrected_poses])

    candidate_set: set[tuple[int, int]] = set()
    for first, second in cKDTree(positions).query_pairs(ICP_RADIUS_M):
        if abs(keyframe_times[first] - keyframe_times[second]) >= ICP_MIN_DT_S:
            candidate_set.add((min(first, second), max(first, second)))
    candidate_pairs = sorted(
        candidate_set, key=lambda pair: np.linalg.norm(positions[pair[0]] - positions[pair[1]])
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
