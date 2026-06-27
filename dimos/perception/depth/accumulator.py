# Copyright 2025-2026 Dimensional Inc.
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

"""Accumulates per-frame depth clouds into a drift-corrected 3D map with SLAM and mesh export.

Pipeline per frame
------------------
1. ICP odometry  — align current frame against a sliding window of recent frames.
2. Loop closure  — every ``lc_interval`` frames, query all stored keyframes with ICP;
                   if a match is found, add a loop-closure edge.
3. Pose-graph optimisation — minimise
       E(T) = Σ_{(i,j)∈E} ‖log(T_i⁻¹ Tⱼ Ω_{ij}⁻¹)‖²_Σ
   using Open3D's Levenberg-Marquardt solver on SE(3).
4. Map rebuild  — after optimisation, reproject all keyframes through their
                  updated poses and voxel-downsample.

On shutdown ``stop()`` exports:
  • ``<export_path>.ply``       — coloured point cloud
  • ``<export_path>_mesh.ply``  — Poisson surface mesh (Kazhdan 2006)
"""

from __future__ import annotations

import threading
import time
from typing import Any

import numpy as np
import open3d as o3d
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.depth.monocular_depth_module import _make_colored_cloud
from dimos.spec.mapping import GlobalPointcloud
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


# ---------------------------------------------------------------------------
# ICP helpers
# ---------------------------------------------------------------------------

def _icp(
    source: o3d.geometry.PointCloud,
    target: o3d.geometry.PointCloud,
    max_distance: float,
    max_iter: int,
    init: np.ndarray | None = None,
) -> tuple[np.ndarray, float]:
    """Point-to-point ICP. Returns (4×4 transform, fitness). Identity on failure."""
    result = o3d.pipelines.registration.registration_icp(
        source,
        target,
        max_distance,
        np.eye(4) if init is None else init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter),
    )
    return result.transformation, result.fitness


def _merge(frames: list[o3d.geometry.PointCloud]) -> o3d.geometry.PointCloud:
    merged = o3d.geometry.PointCloud()
    for f in frames:
        merged += f
    return merged


_MAX_UNCOMPACTED = 1_000_000


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

class Config(ModuleConfig):
    voxel_size: float = 0.03
    world_frame: str = "world"
    # Odometry ICP (sliding window)
    icp_window: int = 5
    icp_min_points: int = 300
    icp_max_correspondence: float = 0.2
    icp_max_iter: int = 20
    merge_interval: int = 10
    publish_freq: float = 5.0
    # Pose-graph SLAM
    lc_interval: int = 15          # check loop closure every N frames
    lc_min_gap: int = 15           # minimum keyframe gap to consider for loop closure
    lc_max_correspondence: float = 0.12
    lc_fitness_threshold: float = 0.45
    # Export
    export_path: str = "map"
    poisson_depth: int = 9         # octree depth; higher = finer mesh, slower


# ---------------------------------------------------------------------------
# Module
# ---------------------------------------------------------------------------

class DepthAccumulatorModule(Module, GlobalPointcloud):
    """Builds a drift-corrected 3D world map from per-frame depth clouds.

    Works with both monocular (no external odometry) and stereo (VIO TF) inputs —
    the pose graph handles drift correction in either case.
    """

    config: Config

    frame_cloud: In[PointCloud2]
    global_map: Out[PointCloud2]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._map = o3d.geometry.PointCloud()
        # Sliding window for odometry ICP
        self._window: list[o3d.geometry.PointCloud] = []
        # Keyframes for SLAM pose graph
        self._keyframes: list[o3d.geometry.PointCloud] = []
        self._poses: list[np.ndarray] = []          # absolute pose T_i in world frame
        self._pose_graph = o3d.pipelines.registration.PoseGraph()
        self._frame_count = 0
        self._last_publish = 0.0

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(self.frame_cloud.subscribe(self._on_frame_cloud))
        )

    @rpc
    def stop(self) -> None:
        with self._lock:
            self._export_map()
        super().stop()

    @rpc
    def reset(self) -> None:
        with self._lock:
            self._map = o3d.geometry.PointCloud()
            self._window.clear()
            self._keyframes.clear()
            self._poses.clear()
            self._pose_graph = o3d.pipelines.registration.PoseGraph()
            self._frame_count = 0

    # ------------------------------------------------------------------
    # Per-frame processing
    # ------------------------------------------------------------------

    def _on_frame_cloud(self, cloud: PointCloud2) -> None:
        pts, cols = cloud.as_numpy()
        if len(pts) == 0:
            return

        frame_pcd = o3d.geometry.PointCloud()
        frame_pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
        if cols is not None and len(cols) == len(pts):
            frame_pcd.colors = o3d.utility.Vector3dVector(cols.astype(np.float64))

        frame_down = frame_pcd.voxel_down_sample(self.config.voxel_size * 2)

        with self._lock:
            # --- Odometry ICP against sliding window ---
            odom_T = np.eye(4)
            window_pts = sum(len(f.points) for f in self._window)
            if self._window and window_pts >= self.config.icp_min_points:
                target = _merge(self._window).voxel_down_sample(self.config.voxel_size * 3)
                odom_T, _ = _icp(
                    frame_down, target,
                    self.config.icp_max_correspondence,
                    self.config.icp_max_iter,
                )
                frame_pcd.transform(odom_T)
                frame_down.transform(odom_T)

            self._window.append(frame_down)
            if len(self._window) > self.config.icp_window:
                self._window.pop(0)

            # --- Pose graph: add node and odometry edge ---
            idx = len(self._keyframes)
            abs_pose = (self._poses[-1] @ odom_T) if self._poses else np.eye(4)
            self._poses.append(abs_pose)
            self._keyframes.append(frame_down)

            node = o3d.pipelines.registration.PoseGraphNode(np.linalg.inv(abs_pose))
            self._pose_graph.nodes.append(node)
            if idx > 0:
                info = np.eye(6)  # uniform information matrix
                edge = o3d.pipelines.registration.PoseGraphEdge(
                    idx - 1, idx, odom_T, info, uncertain=False
                )
                self._pose_graph.edges.append(edge)

            # --- Loop closure check ---
            if (self._frame_count > 0
                    and self._frame_count % self.config.lc_interval == 0
                    and len(self._keyframes) > self.config.lc_min_gap + 1):
                self._check_loop_closures(idx)

            # --- Accumulate map ---
            self._map += frame_pcd
            self._frame_count += 1

            if (self._frame_count % self.config.merge_interval == 0
                    or len(self._map.points) > _MAX_UNCOMPACTED):
                self._map = self._map.voxel_down_sample(self.config.voxel_size)

            # --- Throttled publish ---
            now = time.monotonic()
            if now - self._last_publish < 1.0 / self.config.publish_freq:
                return
            self._last_publish = now

            pts_out = np.asarray(self._map.points, dtype=np.float32)
            cols_out = (
                np.asarray(self._map.colors, dtype=np.float32)
                if self._map.has_colors() else None
            )

        logger.debug("DepthAccumulatorModule: %d points in global map", len(pts_out))
        self.global_map.publish(
            _make_colored_cloud(
                pts_out,
                cols_out if cols_out is not None else np.zeros((len(pts_out), 3), dtype=np.float32),
                self.config.world_frame,
                cloud.ts,
            )
        )

    # ------------------------------------------------------------------
    # Loop closure + pose graph optimisation
    # ------------------------------------------------------------------

    def _check_loop_closures(self, current_idx: int) -> None:
        """ICP query against old keyframes; optimise pose graph on any match."""
        current_kf = self._keyframes[current_idx]
        found_any = False

        for j in range(current_idx - self.config.lc_min_gap):
            T_lc, fitness = _icp(
                current_kf,
                self._keyframes[j],
                self.config.lc_max_correspondence,
                self.config.icp_max_iter,
            )
            if fitness < self.config.lc_fitness_threshold:
                continue

            logger.info(
                "DepthAccumulatorModule: loop closure %d→%d  fitness=%.3f",
                current_idx, j, fitness,
            )
            info = np.eye(6) * fitness  # weight edge by match quality
            edge = o3d.pipelines.registration.PoseGraphEdge(
                current_idx, j, T_lc, info, uncertain=True
            )
            self._pose_graph.edges.append(edge)
            found_any = True

        if found_any:
            self._optimise_graph()

    def _optimise_graph(self) -> None:
        """Levenberg-Marquardt optimisation on SE(3) pose graph, then rebuild map."""
        option = o3d.pipelines.registration.GlobalOptimizationOption(
            max_correspondence_distance=self.config.lc_max_correspondence,
            edge_prune_threshold=0.25,
            reference_node=0,
        )
        o3d.pipelines.registration.global_optimization(
            self._pose_graph,
            o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
            o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
            option,
        )
        self._rebuild_map()

    def _rebuild_map(self) -> None:
        """Reproject all keyframes through their optimised poses and merge."""
        merged = o3d.geometry.PointCloud()
        for i, kf in enumerate(self._keyframes):
            T_opt = np.linalg.inv(self._pose_graph.nodes[i].pose)
            self._poses[i] = T_opt
            merged += kf.transform(T_opt)
        self._map = merged.voxel_down_sample(self.config.voxel_size)
        logger.info(
            "DepthAccumulatorModule: map rebuilt after graph optimisation — %d points",
            len(self._map.points),
        )

    # ------------------------------------------------------------------
    # Export
    # ------------------------------------------------------------------

    def _export_map(self) -> None:
        """Export coloured point cloud and Poisson surface mesh at shutdown."""
        if len(self._map.points) < 100:
            logger.warning("DepthAccumulatorModule: map too sparse to export, skipping")
            return

        path = self.config.export_path

        # --- Raw coloured cloud ---
        pcd = self._map.voxel_down_sample(self.config.voxel_size)
        o3d.io.write_point_cloud(f"{path}.ply", pcd)
        logger.info(
            "DepthAccumulatorModule: exported %d points → %s.ply",
            len(pcd.points), path,
        )

        # --- Poisson surface reconstruction ---
        # Normal estimation: radius covers ~4 voxels, max 30 neighbours.
        # orient_normals_consistent_tangent_plane propagates orientation
        # through a minimum spanning tree of the neighbourhood graph.
        pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.config.voxel_size * 4,
                max_nn=30,
            )
        )
        pcd.orient_normals_consistent_tangent_plane(k=15)

        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pcd, depth=self.config.poisson_depth
        )
        densities = np.asarray(densities)
        # Trim boundary artefacts: remove vertices whose reconstruction density
        # falls below the 5th percentile.
        mesh.remove_vertices_by_mask(densities < np.quantile(densities, 0.05))
        mesh.compute_vertex_normals()

        o3d.io.write_triangle_mesh(f"{path}_mesh.ply", mesh)
        logger.info(
            "DepthAccumulatorModule: Poisson mesh %d triangles → %s_mesh.ply",
            len(mesh.triangles), path,
        )
