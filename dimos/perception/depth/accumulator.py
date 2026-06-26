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

"""Accumulates per-frame depth clouds into a globally consistent 3D map.

Map building strategy for a moving robot with no LiDAR:

  1. ICP registration against a sliding window of the last N frames, not the
     full map. This bounds ICP complexity to O(N · icp_window) regardless of
     how large the total map grows.

  2. Voxel compaction runs every merge_interval frames, not every frame.
     Between compactions, frames are concatenated cheaply. The map never
     deletes observations — it only grows, bounded by the voxel resolution.

  3. Publishing to Rerun is throttled to publish_freq Hz so the Rerun bridge
     is never the bottleneck on a fast-moving robot.

ICP minimises:  E(T) = Σ_i ‖T p_i − q_{j(i)}‖²

where p_i are points in the new frame (already in approximate world frame via
odometry/VIO), and q_{j(i)} their nearest neighbours in the sliding window.
T is applied only if ICP fitness > 0.3 (sufficient point overlap).
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
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _icp_refine(
    source: o3d.geometry.PointCloud,
    target: o3d.geometry.PointCloud,
    max_distance: float,
    max_iter: int,
) -> np.ndarray:
    """Return the ICP refinement transform, or identity if fitness is too low."""
    result = o3d.pipelines.registration.registration_icp(
        source,
        target,
        max_distance,
        np.eye(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter),
    )
    return result.transformation if result.fitness > 0.3 else np.eye(4)


def _merge_window(frames: list[o3d.geometry.PointCloud]) -> o3d.geometry.PointCloud:
    merged = o3d.geometry.PointCloud()
    for f in frames:
        merged += f
    return merged


class Config(ModuleConfig):
    voxel_size: float = 0.03
    world_frame: str = "world"
    # ICP: uses the last icp_window frames as target so complexity stays bounded
    # as the map grows. Starts once the window has icp_min_points total.
    icp_window: int = 5
    icp_min_points: int = 300
    icp_max_correspondence: float = 0.2
    icp_max_iter: int = 20
    # Map compaction: voxel-downsample the full accumulated map every
    # merge_interval frames (not every frame) to amortise the cost.
    merge_interval: int = 10
    # Rerun publish rate cap — keeps the bridge from becoming the bottleneck.
    publish_freq: float = 5.0


class DepthAccumulatorModule(Module):
    """Builds a persistent, drift-corrected 3D world map from per-frame depth clouds.

    Designed for moving robots with camera-only sensing. Optimised so per-frame
    cost stays constant regardless of map size: ICP targets a sliding window of
    recent frames, and full-map compaction is amortised across merge_interval frames.
    """

    config: Config

    frame_cloud: In[PointCloud2]
    global_map: Out[PointCloud2]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._map = o3d.geometry.PointCloud()
        self._window: list[o3d.geometry.PointCloud] = []  # sliding ICP target
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
        super().stop()

    @rpc
    def reset(self) -> None:
        with self._lock:
            self._map = o3d.geometry.PointCloud()
            self._window.clear()
            self._frame_count = 0

    def _on_frame_cloud(self, cloud: PointCloud2) -> None:
        pts, cols = cloud.as_numpy()
        if len(pts) == 0:
            return

        frame_pcd = o3d.geometry.PointCloud()
        frame_pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
        if cols is not None and len(cols) == len(pts):
            frame_pcd.colors = o3d.utility.Vector3dVector(cols.astype(np.float64))

        # Downsampled copy used for ICP source and for the sliding window store.
        frame_down = frame_pcd.voxel_down_sample(self.config.voxel_size * 2)

        with self._lock:
            # --- ICP against sliding window (bounded cost) ---
            window_pts = sum(len(f.points) for f in self._window)
            if self._window and window_pts >= self.config.icp_min_points:
                target = _merge_window(self._window).voxel_down_sample(self.config.voxel_size * 3)
                corr = _icp_refine(
                    frame_down, target,
                    self.config.icp_max_correspondence,
                    self.config.icp_max_iter,
                )
                frame_pcd.transform(corr)
                frame_down.transform(corr)

            # Advance sliding window
            self._window.append(frame_down)
            if len(self._window) > self.config.icp_window:
                self._window.pop(0)

            # --- Accumulate (never delete, cheap concatenation) ---
            self._map += frame_pcd
            self._frame_count += 1

            # Compact periodically to bound memory; not every frame
            if self._frame_count % self.config.merge_interval == 0:
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
