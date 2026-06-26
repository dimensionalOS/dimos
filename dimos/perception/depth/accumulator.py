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

Each incoming frame is registered against the existing map via point-to-point
ICP before being added. ICP uses the odometry-based world-frame position as
the initial estimate and refines it by minimising point-pair residuals:

    E(T) = Σ_i ‖T p_i − q_{j(i)}‖²

where p_i are source points, q_{j(i)} their nearest neighbours in the map,
and T is the refinement transform (close to identity when odometry is good).
This corrects per-frame drift without requiring a full pose-graph pass.

For long traversals with revisits, run correct_map.py after recording to apply
global PGO loop closure on top.
"""

from __future__ import annotations

import threading
from typing import Any

import numpy as np
import open3d as o3d
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.pointclouds.accumulators.general import GeneralPointCloudAccumulator
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
    # fitness = fraction of source points with a correspondence; too low means
    # insufficient overlap and the transform would be unreliable.
    return result.transformation if result.fitness > 0.3 else np.eye(4)


class Config(ModuleConfig):
    voxel_size: float = 0.05
    world_frame: str = "world"
    # ICP registration: frames are registered against the accumulated map to
    # correct odometry drift before being added. Disabled until the map reaches
    # icp_min_points so the first frames don't register against noise.
    icp_min_points: int = 500
    icp_max_correspondence: float = 0.2  # metres; max allowed point-pair distance
    icp_max_iter: int = 30


class DepthAccumulatorModule(Module):
    """Accumulates camera-frame clouds into a persistent, drift-corrected global map."""

    config: Config

    frame_cloud: In[PointCloud2]
    global_map: Out[PointCloud2]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._accum: GeneralPointCloudAccumulator | None = None

    @rpc
    def start(self) -> None:
        super().start()
        from dimos.core.global_config import GlobalConfig
        self._accum = GeneralPointCloudAccumulator(
            voxel_size=self.config.voxel_size,
            global_config=GlobalConfig(),
        )
        self.register_disposable(
            Disposable(self.frame_cloud.subscribe(self._on_frame_cloud))
        )

    @rpc
    def stop(self) -> None:
        super().stop()

    @rpc
    def reset(self) -> None:
        from dimos.core.global_config import GlobalConfig
        with self._lock:
            self._accum = GeneralPointCloudAccumulator(
                voxel_size=self.config.voxel_size,
                global_config=GlobalConfig(),
            )

    def _on_frame_cloud(self, cloud: PointCloud2) -> None:
        pts, cols = cloud.as_numpy()
        if len(pts) == 0:
            return

        frame_pcd = o3d.geometry.PointCloud()
        frame_pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
        if cols is not None and len(cols) == len(pts):
            frame_pcd.colors = o3d.utility.Vector3dVector(cols.astype(np.float64))

        with self._lock:
            map_pcd = self._accum.get_point_cloud()

            if len(map_pcd.points) >= self.config.icp_min_points:
                # Downsample both sides to keep ICP fast (2× and 4× voxel respectively).
                src = frame_pcd.voxel_down_sample(self.config.voxel_size * 2)
                tgt = map_pcd.voxel_down_sample(self.config.voxel_size * 4)
                corr = _icp_refine(
                    src, tgt,
                    self.config.icp_max_correspondence,
                    self.config.icp_max_iter,
                )
                frame_pcd.transform(corr)

            self._accum.add(frame_pcd)
            global_pcd = self._accum.get_point_cloud()
            pts_out = np.asarray(global_pcd.points, dtype=np.float32)
            cols_out = (
                np.asarray(global_pcd.colors, dtype=np.float32)
                if global_pcd.has_colors()
                else np.zeros((len(pts_out), 3), dtype=np.float32)
            )

        logger.debug("DepthAccumulatorModule: %d points in global map", len(pts_out))
        self.global_map.publish(
            _make_colored_cloud(pts_out, cols_out, self.config.world_frame, cloud.ts)
        )
