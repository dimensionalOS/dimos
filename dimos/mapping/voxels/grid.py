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

from __future__ import annotations

import time
from typing import TYPE_CHECKING

from dimos.mapping.voxels.impl.o3d import O3dVoxels
from dimos.mapping.voxels.impl.packed import PackedVoxels
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.decorators.decorators import simple_mcache
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    import open3d as o3d  # type: ignore[import-untyped]

logger = setup_logger()


class VoxelGrid:
    """Voxel grid accumulator with column carving.

    Backend picked at construction: Open3D VoxelBlockGrid on CUDA, packed
    sorted-key numpy store on CPU (the Open3D CPU hashmap path degrades
    linearly with map size — full-map scan + gather per frame, ~150 ms at
    300k voxels). Both produce identical voxel sets.

    No Module/framework dependency. Can be used standalone or wrapped
    by VoxelGridMapper (Module) or VoxelMapTransformer (memory Transformer).
    """

    def __init__(
        self,
        voxel_size: float = 0.05,
        block_count: int = 2_000_000,
        device: str = "CUDA:0",
        carve_columns: bool = True,
        frame_id: str = "world",
        show_startup_log: bool = True,
    ) -> None:
        import open3d.core as o3c  # type: ignore[import-untyped]

        self._voxel_size = voxel_size
        self._frame_id = frame_id

        use_cuda = device.startswith("CUDA") and o3c.cuda.is_available()
        self._impl: O3dVoxels | PackedVoxels = (
            O3dVoxels(voxel_size, block_count, carve_columns, o3c.Device(device))
            if use_cuda
            else PackedVoxels(voxel_size, carve_columns)
        )

        if show_startup_log:
            logger.info(f"VoxelGrid using device: {device if use_cuda else 'CPU:0 (packed-numpy)'}")

        self._latest_frame_ts: float = 0.0
        self._disposed = False

    def _check_disposed(self) -> None:
        if self._disposed:
            raise RuntimeError("VoxelGrid has been disposed and cannot be used")

    def add_frame(self, frame: PointCloud2) -> None:
        self._check_disposed()
        if frame.ts is not None:
            self._latest_frame_ts = frame.ts

        self._impl.add_frame(frame)
        self.get_global_pointcloud.invalidate_cache(self)
        self.get_global_pointcloud2.invalidate_cache(self)

    @simple_mcache
    def get_global_pointcloud2(self) -> PointCloud2:
        self._check_disposed()
        return PointCloud2(
            ensure_legacy_pcd(self.get_global_pointcloud()),
            frame_id=self._frame_id,
            ts=self._latest_frame_ts if self._latest_frame_ts else time.time(),
        )

    @simple_mcache
    def get_global_pointcloud(self) -> o3d.t.geometry.PointCloud:
        import open3d as o3d  # type: ignore[import-untyped]
        import open3d.core as o3c  # type: ignore[import-untyped]

        self._check_disposed()
        cpu = o3c.Device("CPU:0")
        out = o3d.t.geometry.PointCloud(device=cpu)
        out.point["positions"] = o3c.Tensor(self._impl.points(), device=cpu)
        return out

    def size(self) -> int:
        self._check_disposed()
        return self._impl.size()

    def __len__(self) -> int:
        return self.size()

    def dispose(self) -> None:
        """Free backend resources. The object is unusable after this call."""
        if self._disposed:
            return
        self._disposed = True
        self.get_global_pointcloud.invalidate_cache(self)  # type: ignore[attr-defined]
        self.get_global_pointcloud2.invalidate_cache(self)  # type: ignore[attr-defined]
        self._impl.dispose()


def ensure_legacy_pcd(
    pcd_any: o3d.t.geometry.PointCloud | o3d.geometry.PointCloud,
) -> o3d.geometry.PointCloud:
    import open3d as o3d  # type: ignore[import-untyped]

    if isinstance(pcd_any, o3d.geometry.PointCloud):
        return pcd_any

    assert isinstance(pcd_any, o3d.t.geometry.PointCloud), (
        "Input must be a legacy PointCloud or a tensor PointCloud"
    )

    return pcd_any.to_legacy()
