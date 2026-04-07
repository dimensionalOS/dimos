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

"""Pure-Python reference implementation of the PGO algorithm.

Uses scipy KDTree for neighbor search, open3d for ICP, and gtsam Python
bindings for pose graph optimization. Tests the LOGIC independent of the
C++ binary.
"""

from __future__ import annotations

from dataclasses import dataclass

import gtsam
import numpy as np
import open3d as o3d
from scipy.spatial import KDTree


@dataclass
class PGOConfig:
    """PGO algorithm configuration."""

    key_pose_delta_trans: float = 0.5
    key_pose_delta_deg: float = 10.0
    loop_search_radius: float = 15.0
    loop_time_thresh: float = 60.0
    loop_score_thresh: float = 0.3
    loop_submap_half_range: int = 5
    submap_resolution: float = 0.1
    min_loop_detect_duration: float = 5.0
    global_map_voxel_size: float = 0.15
    max_icp_iterations: int = 50
    max_icp_correspondence_dist: float = 10.0


@dataclass
class KeyPose:
    """Stored keyframe with local and global poses."""

    r_local: np.ndarray  # 3x3 rotation matrix (local/odometry frame)
    t_local: np.ndarray  # 3 translation vector (local/odometry frame)
    r_global: np.ndarray  # 3x3 rotation matrix (optimized global frame)
    t_global: np.ndarray  # 3 translation vector (optimized global frame)
    time: float  # timestamp
    body_cloud: np.ndarray  # Nx3 point cloud in body frame


@dataclass
class LoopPair:
    """Detected loop closure between two keyframes."""

    source_id: int
    target_id: int
    r_offset: np.ndarray  # 3x3 relative rotation
    t_offset: np.ndarray  # 3 relative translation
    score: float


def _rotation_to_quat(R: np.ndarray) -> np.ndarray:
    """Convert 3x3 rotation matrix to quaternion [x,y,z,w]."""
    from scipy.spatial.transform import Rotation

    return Rotation.from_matrix(R).as_quat()  # [x,y,z,w]


def _angular_distance_deg(R1: np.ndarray, R2: np.ndarray) -> float:
    """Compute angular distance in degrees between two rotation matrices."""
    R_diff = R1.T @ R2
    # Clamp to avoid numerical issues with arccos
    trace = np.clip((np.trace(R_diff) - 1.0) / 2.0, -1.0, 1.0)
    return np.degrees(np.arccos(trace))


def _voxel_downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Voxel-grid downsample an Nx3 point cloud."""
    if len(points) == 0 or voxel_size <= 0:
        return points
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points.astype(np.float64))
    pcd = pcd.voxel_down_sample(voxel_size)
    return np.asarray(pcd.points)


class SimplePGOReference:
    """Pure-Python reference implementation of SimplePGO.

    Mirrors the C++ SimplePGO class for testing purposes.
    """

    def __init__(self, config: PGOConfig | None = None) -> None:
        self.config = config or PGOConfig()
        self.key_poses: list[KeyPose] = []
        self.history_pairs: list[tuple[int, int]] = []
        self._cache_pairs: list[LoopPair] = []
        self._r_offset = np.eye(3)
        self._t_offset = np.zeros(3)

        # GTSAM iSAM2
        params = gtsam.ISAM2Params()
        params.setRelinearizeThreshold(0.01)
        params.relinearizeSkip = 1
        self._isam2 = gtsam.ISAM2(params)
        self._graph = gtsam.NonlinearFactorGraph()
        self._initial_values = gtsam.Values()

    def is_key_pose(self, r: np.ndarray, t: np.ndarray) -> bool:
        """Check if a pose qualifies as a new keyframe."""
        if len(self.key_poses) == 0:
            return True
        last = self.key_poses[-1]
        delta_trans = np.linalg.norm(t - last.t_local)
        delta_deg = _angular_distance_deg(last.r_local, r)
        return (
            delta_trans > self.config.key_pose_delta_trans
            or delta_deg > self.config.key_pose_delta_deg
        )

    def add_key_pose(
        self, r_local: np.ndarray, t_local: np.ndarray, timestamp: float, body_cloud: np.ndarray
    ) -> bool:
        """Add a keyframe if it passes the keyframe test. Returns True if added."""
        if not self.is_key_pose(r_local, t_local):
            return False

        idx = len(self.key_poses)
        init_r = self._r_offset @ r_local
        init_t = self._r_offset @ t_local + self._t_offset

        # Add initial value to GTSAM
        pose = gtsam.Pose3(gtsam.Rot3(init_r), gtsam.Point3(init_t))
        self._initial_values.insert(idx, pose)

        if idx == 0:
            # Prior factor
            noise = gtsam.noiseModel.Diagonal.Variances(np.ones(6) * 1e-12)
            self._graph.addPriorPose3(idx, pose, noise)
        else:
            # Odometry (between) factor
            last = self.key_poses[-1]
            r_between = last.r_local.T @ r_local
            t_between = last.r_local.T @ (t_local - last.t_local)
            noise = gtsam.noiseModel.Diagonal.Variances(
                np.array([1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-6])
            )
            delta = gtsam.Pose3(gtsam.Rot3(r_between), gtsam.Point3(t_between))
            self._graph.add(gtsam.BetweenFactorPose3(idx - 1, idx, delta, noise))

        kp = KeyPose(
            r_local=r_local.copy(),
            t_local=t_local.copy(),
            r_global=init_r.copy(),
            t_global=init_t.copy(),
            time=timestamp,
            body_cloud=body_cloud.copy() if len(body_cloud) > 0 else body_cloud,
        )
        self.key_poses.append(kp)
        return True

    def get_submap(self, idx: int, half_range: int, resolution: float) -> np.ndarray:
        """Build a submap around a keyframe by transforming nearby body clouds."""
        min_idx = max(0, idx - half_range)
        max_idx = min(len(self.key_poses) - 1, idx + half_range)

        all_pts = []
        for i in range(min_idx, max_idx + 1):
            kp = self.key_poses[i]
            if len(kp.body_cloud) == 0:
                continue
            # Transform body cloud to global frame
            global_pts = (kp.r_global @ kp.body_cloud.T).T + kp.t_global
            all_pts.append(global_pts)

        if not all_pts:
            return np.zeros((0, 3))
        combined = np.vstack(all_pts)
        if resolution > 0:
            combined = _voxel_downsample(combined, resolution)
        return combined

    def search_for_loop_pairs(self) -> None:
        """Search for loop closure candidates using KD-tree radius search + ICP."""
        if len(self.key_poses) < 10:
            return

        # Rate limiting
        if self.config.min_loop_detect_duration > 0.0 and self.history_pairs:
            current_time = self.key_poses[-1].time
            last_time = self.key_poses[self.history_pairs[-1][1]].time
            if current_time - last_time < self.config.min_loop_detect_duration:
                return

        cur_idx = len(self.key_poses) - 1
        last_item = self.key_poses[-1]

        # Build KD-tree from all previous keyframe positions
        positions = np.array([kp.t_global for kp in self.key_poses[:-1]])
        kdtree = KDTree(positions)

        # Radius search
        indices = kdtree.query_ball_point(last_item.t_global, self.config.loop_search_radius)
        if not indices:
            return

        # Sort by distance
        dists = [np.linalg.norm(last_item.t_global - positions[i]) for i in indices]
        sorted_indices = [indices[i] for i in np.argsort(dists)]

        # Find candidate far enough in time
        loop_idx = -1
        for idx in sorted_indices:
            if abs(last_item.time - self.key_poses[idx].time) > self.config.loop_time_thresh:
                loop_idx = idx
                break

        if loop_idx == -1:
            return

        # ICP verification
        target_cloud = self.get_submap(
            loop_idx, self.config.loop_submap_half_range, self.config.submap_resolution
        )
        source_cloud = self.get_submap(cur_idx, 0, self.config.submap_resolution)

        if len(target_cloud) < 10 or len(source_cloud) < 10:
            return

        transform, score = self._run_icp(source_cloud, target_cloud)
        if score > self.config.loop_score_thresh:
            return

        # Compute loop closure constraint
        r_transform = transform[:3, :3]
        t_transform = transform[:3, 3]
        r_refined = r_transform @ self.key_poses[cur_idx].r_global
        t_refined = r_transform @ self.key_poses[cur_idx].t_global + t_transform
        r_offset = self.key_poses[loop_idx].r_global.T @ r_refined
        t_offset = self.key_poses[loop_idx].r_global.T @ (
            t_refined - self.key_poses[loop_idx].t_global
        )

        pair = LoopPair(
            source_id=cur_idx,
            target_id=loop_idx,
            r_offset=r_offset,
            t_offset=t_offset,
            score=score,
        )
        self._cache_pairs.append(pair)
        self.history_pairs.append((loop_idx, cur_idx))

    def _run_icp(self, source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, float]:
        """Run ICP between source and target point clouds.

        Returns (4x4 transform, fitness score).
        """
        src_pcd = o3d.geometry.PointCloud()
        src_pcd.points = o3d.utility.Vector3dVector(source.astype(np.float64))
        tgt_pcd = o3d.geometry.PointCloud()
        tgt_pcd.points = o3d.utility.Vector3dVector(target.astype(np.float64))

        result = o3d.pipelines.registration.registration_icp(
            src_pcd,
            tgt_pcd,
            max_correspondence_distance=self.config.max_icp_correspondence_dist,
            init=np.eye(4),
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=self.config.max_icp_iterations,
            ),
        )
        # Reject matches with zero/near-zero correspondences (fitness=0 means
        # no points were within max_correspondence_distance). In this case
        # inlier_rmse is 0.0 which would incorrectly pass the score threshold.
        if result.fitness < 0.05:
            return result.transformation, float("inf")
        return result.transformation, result.inlier_rmse

    def smooth_and_update(self) -> None:
        """Run iSAM2 optimization and update keyframe poses."""
        has_loop = len(self._cache_pairs) > 0

        # Add loop closure factors
        if has_loop:
            for pair in self._cache_pairs:
                noise = gtsam.noiseModel.Diagonal.Variances(np.ones(6) * pair.score)
                delta = gtsam.Pose3(gtsam.Rot3(pair.r_offset), gtsam.Point3(pair.t_offset))
                self._graph.add(
                    gtsam.BetweenFactorPose3(pair.target_id, pair.source_id, delta, noise)
                )
            self._cache_pairs.clear()

        # iSAM2 update
        self._isam2.update(self._graph, self._initial_values)
        self._isam2.update()
        if has_loop:
            for _ in range(4):
                self._isam2.update()
        self._graph = gtsam.NonlinearFactorGraph()
        self._initial_values = gtsam.Values()

        # Update keyframe poses from estimates
        estimates = self._isam2.calculateBestEstimate()
        for i in range(len(self.key_poses)):
            pose = estimates.atPose3(i)
            self.key_poses[i].r_global = pose.rotation().matrix()
            self.key_poses[i].t_global = pose.translation()

        # Update offset
        last = self.key_poses[-1]
        self._r_offset = last.r_global @ last.r_local.T
        self._t_offset = last.t_global - self._r_offset @ last.t_local

    def get_corrected_pose(
        self, r_local: np.ndarray, t_local: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """Get corrected pose for a local pose."""
        r_corrected = self._r_offset @ r_local
        t_corrected = self._r_offset @ t_local + self._t_offset
        return r_corrected, t_corrected

    def build_global_map(self, voxel_size: float | None = None) -> np.ndarray:
        """Build global map from all corrected keyframes."""
        if voxel_size is None:
            voxel_size = self.config.global_map_voxel_size

        all_pts = []
        for kp in self.key_poses:
            if len(kp.body_cloud) == 0:
                continue
            global_pts = (kp.r_global @ kp.body_cloud.T).T + kp.t_global
            all_pts.append(global_pts)

        if not all_pts:
            return np.zeros((0, 3))
        combined = np.vstack(all_pts)
        if voxel_size > 0:
            combined = _voxel_downsample(combined, voxel_size)
        return combined

    @property
    def r_offset(self) -> np.ndarray:
        return self._r_offset

    @property
    def t_offset(self) -> np.ndarray:
        return self._t_offset
