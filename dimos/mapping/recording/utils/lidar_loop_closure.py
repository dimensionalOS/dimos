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

"""LiDAR loop closure for the AprilTag groundtruth solve.

AprilTags only pin drift where a tag is re-seen; between tags the FAST-LIO chain
drifts uncorrected. This module finds *revisits* from the lidar alone and turns
each into a relative-pose constraint the GTSAM solve can add as a loop closure:

  1. sample keyframes along the odom chain (one per `keyframe_gap_m` of travel),
  2. describe each keyframe cloud with a rotation-invariant Scan Context
     descriptor (Kim & Kim, IROS'18) so revisits match by appearance, not by the
     drifted odom position,
  3. shortlist candidates by ring-key nearest neighbour, confirm with the
     column-aligned Scan Context distance,
  4. verify each candidate with Generalized-ICP and keep only well-aligned pairs.

Returns `LoopClosure` measurements indexed against the same node array the solve
uses. The keyframe clouds and the odom backbone are both in the lidar (mid360)
frame, so the GICP transform *is* the relative-pose measurement — no extrinsics.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
from sklearn.neighbors import NearestNeighbors

from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


@dataclass
class LoopClosure:
    """A revisit constraint: node `j` (later) seen again at node `i` (earlier).

    `relative_pose7` is the pose of frame `j` expressed in frame `i`
    ([x y z qx qy qz qw]); `fitness`/`rmse` are the GICP alignment quality.
    """

    i: int
    j: int
    relative_pose7: list[float]
    fitness: float
    rmse: float


def scan_context(points: np.ndarray, n_ring: int, n_sector: int, max_radius: float) -> np.ndarray:
    """Bird's-eye Scan Context: an `n_ring` x `n_sector` grid whose cells hold the
    max height of the points falling in each (range, azimuth) bin."""
    descriptor = np.zeros((n_ring, n_sector), dtype=np.float64)
    xy_range = np.linalg.norm(points[:, :2], axis=1)
    in_range = (xy_range > 1e-3) & (xy_range < max_radius)
    if not np.any(in_range):
        return descriptor
    points = points[in_range]
    xy_range = xy_range[in_range]
    azimuth = np.arctan2(points[:, 1], points[:, 0])
    ring_index = np.clip((xy_range / max_radius * n_ring).astype(int), 0, n_ring - 1)
    sector_index = np.clip(
        ((azimuth + np.pi) / (2 * np.pi) * n_sector).astype(int), 0, n_sector - 1
    )
    flat_index = ring_index * n_sector + sector_index
    np.maximum.at(descriptor.reshape(-1), flat_index, points[:, 2])
    return descriptor


def ring_key(descriptor: np.ndarray) -> np.ndarray:
    """Rotation-invariant summary of a descriptor (mean height per ring)."""
    return descriptor.mean(axis=1)


def scan_context_distance(query: np.ndarray, candidate: np.ndarray) -> tuple[float, int]:
    """Minimum column-aligned cosine distance over all azimuth shifts.

    Returns (distance in [0, 1], best column shift). The shift recovers the yaw
    between the two views and seeds GICP."""
    n_sector = query.shape[1]
    query_norm = np.linalg.norm(query, axis=0)
    best_distance, best_shift = 2.0, 0
    for shift in range(n_sector):
        shifted = np.roll(candidate, shift, axis=1)
        shifted_norm = np.linalg.norm(shifted, axis=0)
        denominator = query_norm * shifted_norm
        valid = denominator > 1e-9
        if not np.any(valid):
            continue
        cosine = (query * shifted).sum(axis=0)[valid] / denominator[valid]
        distance = 1.0 - float(cosine.mean())
        if distance < best_distance:
            best_distance, best_shift = distance, shift
    return best_distance, best_shift


def _yaw_transform(yaw_radians: float) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler("z", yaw_radians).as_matrix()
    return transform


def _world_rotation(pose7: list[float]) -> np.ndarray:
    """Rotation-only 4x4 of a [x y z qx qy qz qw] pose (its world orientation)."""
    rotation = np.eye(4)
    rotation[:3, :3] = Rotation.from_quat(pose7[3:7]).as_matrix()
    return rotation


def _to_open3d(points: np.ndarray, voxel: float) -> o3d.geometry.PointCloud:
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    return cloud.voxel_down_sample(voxel) if voxel > 0 else cloud


def _gicp(
    source: np.ndarray,
    target: np.ndarray,
    init: np.ndarray,
    max_correspondence: float,
    voxel: float,
) -> tuple[np.ndarray, float, float]:
    """Generalized-ICP of `source` onto `target`. Returns (target_from_source 4x4,
    fitness, inlier_rmse). The transform maps a source-frame point into target."""
    source_cloud = _to_open3d(source, voxel)
    target_cloud = _to_open3d(target, voxel)
    result = o3d.pipelines.registration.registration_generalized_icp(
        source_cloud,
        target_cloud,
        max_correspondence,
        init,
        o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=60),
    )
    return result.transformation, result.fitness, result.inlier_rmse


def _matrix_to_pose7(transform: np.ndarray) -> list[float]:
    quaternion = Rotation.from_matrix(transform[:3, :3]).as_quat()  # x, y, z, w
    translation = transform[:3, 3]
    return [float(value) for value in (*translation, *quaternion)]


def _load_keyframe_clouds(
    db_path: str, lidar_stream: str, keyframe_timestamps: np.ndarray
) -> dict[int, np.ndarray]:
    """Stream `lidar_stream` once, keeping the nearest-in-time cloud per keyframe."""
    best_delta = {index: float("inf") for index in range(len(keyframe_timestamps))}
    clouds: dict[int, np.ndarray] = {}
    with SqliteStore(path=db_path) as store:
        if lidar_stream not in store.list_streams():
            return clouds
        for observation in store.stream(lidar_stream, PointCloud2):
            keyframe_index = int(np.argmin(np.abs(keyframe_timestamps - observation.ts)))
            delta = abs(float(observation.ts) - keyframe_timestamps[keyframe_index])
            if delta < best_delta[keyframe_index]:
                points = observation.data.points_f32()
                if len(points):
                    best_delta[keyframe_index] = delta
                    clouds[keyframe_index] = np.asarray(points, dtype=np.float64)
    return clouds


def _select_keyframes(node_poses7: list[list[float]], keyframe_gap_m: float) -> list[int]:
    """Node indices spaced ~`keyframe_gap_m` apart along the travelled path."""
    keyframe_nodes = [0]
    last_position = np.array(node_poses7[0][:3])
    for node_index in range(1, len(node_poses7)):
        position = np.array(node_poses7[node_index][:3])
        if np.linalg.norm(position - last_position) >= keyframe_gap_m:
            keyframe_nodes.append(node_index)
            last_position = position
    return keyframe_nodes


def find_loop_closures(
    db_path: str,
    node_timestamps: np.ndarray,
    node_poses7: list[list[float]],
    *,
    lidar_stream: str = "livox_lidar",
    keyframe_gap_m: float = 1.0,
    n_ring: int = 20,
    n_sector: int = 60,
    max_radius: float = 20.0,
    num_candidates: int = 10,
    sc_distance_max: float = 0.5,
    min_time_gap_s: float = 15.0,
    gicp_max_correspondence: float = 2.0,
    gicp_voxel: float = 0.3,
    gicp_fitness_min: float = 0.7,
    gicp_rmse_max: float = 0.55,
    max_translation_m: float = 2.5,
    max_vertical_m: float = 1.0,
) -> list[LoopClosure]:
    """Detect lidar revisits among the solve's odom nodes and verify them with GICP."""
    keyframe_nodes = _select_keyframes(node_poses7, keyframe_gap_m)
    if len(keyframe_nodes) < 3:
        print(f"   loop: too few keyframes ({len(keyframe_nodes)}) — skipping")
        return []
    keyframe_timestamps = node_timestamps[keyframe_nodes]
    clouds = _load_keyframe_clouds(db_path, lidar_stream, keyframe_timestamps)
    usable = [index for index in range(len(keyframe_nodes)) if index in clouds]
    if len(usable) < 3:
        print(f"   loop: no usable '{lidar_stream}' clouds at keyframes — skipping")
        return []

    # The mid360 is mounted pitched, so raw scans aren't a level bird's-eye. Rotate
    # each keyframe into its world orientation (gravity up) for Scan Context and
    # GICP, so a revisit from any heading differs only by a yaw column-shift. The
    # GICP transform is converted back to the sensor frame at the end.
    world_rotation = {
        index: _world_rotation(node_poses7[keyframe_nodes[index]]) for index in usable
    }
    aligned = {index: clouds[index] @ world_rotation[index][:3, :3].T for index in usable}

    descriptors = {
        index: scan_context(aligned[index], n_ring, n_sector, max_radius) for index in usable
    }
    ring_keys = np.array([ring_key(descriptors[index]) for index in usable])
    neighbours = NearestNeighbors(n_neighbors=min(num_candidates + 1, len(usable))).fit(ring_keys)

    loops: list[LoopClosure] = []
    for position, query_index in enumerate(usable):
        _, candidate_positions = neighbours.kneighbors(ring_keys[position : position + 1])
        best: LoopClosure | None = None
        best_distance = sc_distance_max
        for candidate_position in candidate_positions[0]:
            candidate_index = usable[candidate_position]
            if (
                keyframe_timestamps[query_index] - keyframe_timestamps[candidate_index]
                < min_time_gap_s
            ):
                continue  # not a revisit: too close in time (or in the future)
            distance, shift = scan_context_distance(
                descriptors[query_index], descriptors[candidate_index]
            )
            if distance >= best_distance:
                continue
            # Scan Context recovers the yaw magnitude but not its sign — seed GICP
            # from both and keep the better-aligned result.
            yaw = shift * (2 * np.pi / n_sector)
            transform, fitness, rmse = max(
                (
                    _gicp(
                        aligned[query_index],
                        aligned[candidate_index],
                        _yaw_transform(signed_yaw),
                        gicp_max_correspondence,
                        gicp_voxel,
                    )
                    for signed_yaw in (yaw, -yaw)
                ),
                key=lambda result: result[1],
            )
            if fitness < gicp_fitness_min or rmse > gicp_rmse_max:
                continue
            # A genuine revisit puts the two scans at nearly the same place; a large
            # (especially vertical) offset is perceptual aliasing — drop it. GT is
            # corrupted far worse by one bad loop than helped by a marginal one.
            if (
                abs(transform[2, 3]) > max_vertical_m
                or np.linalg.norm(transform[:3, 3]) > max_translation_m
            ):
                continue
            # convert aligned-frame transform back to the sensor (mid360) node frame
            sensor_transform = (
                np.linalg.inv(world_rotation[candidate_index])
                @ transform
                @ world_rotation[query_index]
            )
            best_distance = distance
            best = LoopClosure(
                i=keyframe_nodes[candidate_index],
                j=keyframe_nodes[query_index],
                relative_pose7=_matrix_to_pose7(sensor_transform),
                fitness=float(fitness),
                rmse=float(rmse),
            )
        if best is not None:
            loops.append(best)

    print(
        f"   loop: {len(keyframe_nodes)} keyframes ({len(usable)} with clouds) "
        f"-> {len(loops)} lidar loop closures (stream '{lidar_stream}')"
    )
    return loops
