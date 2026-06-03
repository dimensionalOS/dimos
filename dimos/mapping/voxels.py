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
from typing import TYPE_CHECKING, Any

import open3d as o3d  # type: ignore[import-untyped]
import open3d.core as o3c  # type: ignore[import-untyped]

from dimos.core.core import rpc
from dimos.core.module import ModuleConfig
from dimos.core.stream import In, Out
from dimos.memory2.module import StreamModule
from dimos.memory2.stream import Stream
from dimos.memory2.transform import Transformer
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.decorators.decorators import simple_mcache
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from collections.abc import Iterator

    from dimos.memory2.type.observation import Observation

logger = setup_logger()


class VoxelGrid:
    """Pure voxel grid accumulator using Open3D VoxelBlockGrid.

    No Module/framework dependency. Can be used standalone or wrapped
    by VoxelGridMapper (Module) or VoxelMapTransformer (memory2 Transformer).
    """

    def __init__(
        self,
        voxel_size: float = 0.05,
        block_count: int = 2_000_000,
        device: str = "CUDA:0",
        carve_columns: bool = True,
        frame_id: str = "world",
    ) -> None:
        self._voxel_size = voxel_size
        self._carve_columns = carve_columns
        self._frame_id = frame_id

        dev = (
            o3c.Device(device)
            if (device.startswith("CUDA") and o3c.cuda.is_available())
            else o3c.Device("CPU:0")
        )

        logger.info(f"VoxelGrid using device: {dev}")

        self.vbg: o3d.t.geometry.VoxelBlockGrid | None = o3d.t.geometry.VoxelBlockGrid(
            attr_names=("dummy",),
            attr_dtypes=(o3c.uint8,),
            attr_channels=(o3c.SizeVector([1]),),
            voxel_size=voxel_size,
            block_resolution=1,
            block_count=block_count,
            device=dev,
        )

        self._dev = dev
        self._voxel_hashmap = self.vbg.hashmap()
        self._key_dtype = self._voxel_hashmap.key_tensor().dtype
        self._latest_frame_ts: float = 0.0
        self._disposed = False

    def _check_disposed(self) -> None:
        if self._disposed:
            raise RuntimeError("VoxelGrid has been disposed and cannot be used")

    def add_frame(self, frame: PointCloud2) -> None:
        self._check_disposed()
        if frame.ts is not None:
            self._latest_frame_ts = frame.ts

        pcd = ensure_tensor_pcd(frame.pointcloud, self._dev)

        if pcd.is_empty():
            return

        pts = pcd.point["positions"].to(self._dev, o3c.float32)
        vox = (pts / self._voxel_size).floor().to(self._key_dtype)
        keys_Nx3 = vox.contiguous()

        if self._carve_columns:
            self._carve_and_insert(keys_Nx3)
        else:
            self._voxel_hashmap.activate(keys_Nx3)

        self.get_global_pointcloud.invalidate_cache(self)
        self.get_global_pointcloud2.invalidate_cache(self)

    def _carve_and_insert(self, new_keys: o3c.Tensor) -> None:
        """Column carving: remove all existing voxels sharing (X,Y) with new_keys, then insert."""
        if new_keys.shape[0] == 0:
            self._voxel_hashmap.activate(new_keys)
            return

        xy_keys = new_keys[:, :2].contiguous()

        xy_hashmap = o3c.HashMap(
            init_capacity=xy_keys.shape[0],
            key_dtype=self._key_dtype,
            key_element_shape=o3c.SizeVector([2]),
            value_dtypes=[o3c.uint8],
            value_element_shapes=[o3c.SizeVector([1])],
            device=self._dev,
        )
        dummy_vals = o3c.Tensor.zeros((xy_keys.shape[0], 1), o3c.uint8, self._dev)
        xy_hashmap.insert(xy_keys, dummy_vals)

        active_indices = self._voxel_hashmap.active_buf_indices()
        if active_indices.shape[0] == 0:
            self._voxel_hashmap.activate(new_keys)
            return

        existing_keys = self._voxel_hashmap.key_tensor()[active_indices]
        existing_xy = existing_keys[:, :2].contiguous()

        _, found_mask = xy_hashmap.find(existing_xy)

        to_erase = existing_keys[found_mask]
        if to_erase.shape[0] > 0:
            self._voxel_hashmap.erase(to_erase)

        self._voxel_hashmap.activate(new_keys)

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
        self._check_disposed()
        assert self.vbg is not None
        voxel_coords, _ = self.vbg.voxel_coordinates_and_flattened_indices()
        # Move to CPU immediately to avoid holding a large duplicate on GPU.
        cpu = o3c.Device("CPU:0")
        pts = voxel_coords.to(cpu) + (self._voxel_size * 0.5)
        out = o3d.t.geometry.PointCloud(device=cpu)
        out.point["positions"] = pts
        return out

    def size(self) -> int:
        self._check_disposed()
        return self._voxel_hashmap.size()  # type: ignore[no-any-return]

    def __len__(self) -> int:
        return self.size()

    def dispose(self) -> None:
        """Free GPU resources. The object is unusable after this call."""
        if self._disposed:
            return
        self._disposed = True
        self.get_global_pointcloud.invalidate_cache(self)  # type: ignore[attr-defined]
        self.get_global_pointcloud2.invalidate_cache(self)  # type: ignore[attr-defined]
        self.vbg = None
        self._voxel_hashmap = None


class VoxelMapTransformer(Transformer[PointCloud2, PointCloud2]):
    """Accumulate PointCloud2 observations into a global voxel map.

    Assumes input clouds are already in world frame.
    All keyword arguments except ``emit_every`` are forwarded to
    :class:`VoxelGrid`.

    Args:
        emit_every: Yield the current accumulated map every *n* frames.
            ``1`` (default) = yield after every frame (live-compatible).
            ``0`` = yield only when upstream exhausts (batch mode).
        **grid_kwargs: Forwarded to ``VoxelGrid()``.
    """

    def __init__(self, *, emit_every: int = 1, **grid_kwargs: Any) -> None:
        self.emit_every = emit_every
        self._grid_kwargs = grid_kwargs

    def _make_obs(
        self, grid: VoxelGrid, last_obs: Observation[PointCloud2], count: int
    ) -> Observation[PointCloud2]:
        # pose=None: the global map is in world frame, per-observation pose is meaningless
        return last_obs.derive(
            data=grid.get_global_pointcloud2(),
            pose=None,
            tags={**last_obs.tags, "frame_count": count},
        )

    def __call__(
        self, upstream: Iterator[Observation[PointCloud2]]
    ) -> Iterator[Observation[PointCloud2]]:
        grid = VoxelGrid(**self._grid_kwargs)
        try:
            last_obs: Observation[PointCloud2] | None = None
            count = 0

            for obs in upstream:
                grid.add_frame(obs.data)
                last_obs = obs
                count += 1

                if self.emit_every > 0 and count % self.emit_every == 0:
                    yield self._make_obs(grid, last_obs, count)

            # Yield on exhaustion: always in batch mode, or if there are un-emitted frames
            if last_obs is not None and (self.emit_every == 0 or count % self.emit_every != 0):
                yield self._make_obs(grid, last_obs, count)
        finally:
            grid.dispose()


class VoxelGridMapperConfig(ModuleConfig):
    """Configuration for VoxelGridMapper."""

    voxel_size: float = 0.05
    block_count: int = 2_000_000
    device: str = "CUDA:0"
    carve_columns: bool = True
    frame_id: str = "world"


class VoxelGridMapper(StreamModule[PointCloud2, PointCloud2]):
    """Accumulate lidar point clouds into a global voxel map."""

    config: VoxelGridMapperConfig

    def pipeline(self, stream: Stream[PointCloud2]) -> Stream[PointCloud2]:
        cfg = self.config.model_dump(
            include=set(VoxelGridMapperConfig.model_fields) - set(ModuleConfig.model_fields)
        )
        return stream.transform(VoxelMapTransformer(**cfg))

    lidar: In[PointCloud2]
    global_map: Out[PointCloud2]

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()


def ensure_tensor_pcd(
    pcd_any: o3d.t.geometry.PointCloud | o3d.geometry.PointCloud,
    device: o3c.Device,
) -> o3d.t.geometry.PointCloud:
    """Convert legacy / cuda.pybind point clouds into o3d.t.geometry.PointCloud on `device`."""

    if isinstance(pcd_any, o3d.t.geometry.PointCloud):
        return pcd_any.to(device)

    assert isinstance(pcd_any, o3d.geometry.PointCloud), (
        "Input must be a legacy PointCloud or a tensor PointCloud"
    )

    return o3d.t.geometry.PointCloud.from_legacy(pcd_any, o3c.float32, device)


def ensure_legacy_pcd(
    pcd_any: o3d.t.geometry.PointCloud | o3d.geometry.PointCloud,
) -> o3d.geometry.PointCloud:
    if isinstance(pcd_any, o3d.geometry.PointCloud):
        return pcd_any

    assert isinstance(pcd_any, o3d.t.geometry.PointCloud), (
        "Input must be a legacy PointCloud or a tensor PointCloud"
    )

    return pcd_any.to_legacy()
