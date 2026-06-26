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

"""Thin Module wrapper around GeneralPointCloudAccumulator for camera pipelines.

``GeneralPointCloudAccumulator`` (in ``dimos.mapping.pointclouds.accumulators``)
already handles voxel-based accumulation with cylinder-splice: each new frame
removes stale points in its visible region and replaces them with fresh
observations.  This prevents ghost copies as the camera moves — better than
time-based decay.

This module simply adapts the ``frame_cloud`` port from
``MonocularDepthModule`` to the ``global_map`` port expected by ``CostMapper``,
using that existing accumulator internally::

    autoconnect(
        MonocularDepthModule.blueprint(),
        DepthAccumulatorModule.blueprint(),
        CostMapper.blueprint(algo="height_cost"),
    )

For loop closure on longer traversals, compose with the existing ``PGO``
transformer from ``dimos.mapping.loop_closure.pgo``.
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


class Config(ModuleConfig):
    """Configuration for DepthAccumulatorModule.

    Attributes:
        voxel_size: Cell size in metres — match CostMapper resolution (0.05 m).
        world_frame: Frame ID written into ``global_map``.
    """

    voxel_size: float = 0.05
    world_frame: str = "world"


class DepthAccumulatorModule(Module):
    """Accumulates camera-frame clouds into a persistent global map.

    Wraps ``GeneralPointCloudAccumulator`` which uses a cylinder-splice
    strategy: each new frame removes existing map points in its visible
    region before adding the fresh observations.  This prevents ghost
    copies when the camera or robot moves without requiring an explicit
    time-based decay.

    Ports
    -----
    Inputs
        frame_cloud : Per-frame coloured cloud from ``MonocularDepthModule``.
    Outputs
        global_map  : Accumulated cloud; satisfies the ``GlobalPointcloud``
            spec consumed by ``CostMapper``.
    """

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
        """Clear the accumulated map."""
        from dimos.core.global_config import GlobalConfig
        with self._lock:
            self._accum = GeneralPointCloudAccumulator(
                voxel_size=self.config.voxel_size,
                global_config=GlobalConfig(),
            )
        logger.info("DepthAccumulatorModule: map cleared")

    def _on_frame_cloud(self, cloud: PointCloud2) -> None:
        pts, cols = cloud.as_numpy()
        if len(pts) == 0:
            return

        frame_pcd = o3d.geometry.PointCloud()
        frame_pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
        if cols is not None and len(cols) == len(pts):
            frame_pcd.colors = o3d.utility.Vector3dVector(cols.astype(np.float64))

        with self._lock:
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
