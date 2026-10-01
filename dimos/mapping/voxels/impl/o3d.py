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

from typing import TYPE_CHECKING

from dimos_generated.sensor_msgs.msg import PointCloud2
import numpy as np
from numpy.typing import NDArray

from dimos.msgs.pointcloud import pointcloud_xyz

if TYPE_CHECKING:
    import open3d as o3d  # type: ignore[import-untyped]
    import open3d.core as o3c  # type: ignore[import-untyped]


class O3dVoxels:
    """CUDA voxel store: Open3D VoxelBlockGrid hashmap."""

    def __init__(
        self, voxel_size: float, block_count: int, carve_columns: bool, dev: o3c.Device
    ) -> None:
        import open3d as o3d  # type: ignore[import-untyped]
        import open3d.core as o3c  # type: ignore[import-untyped]

        self._voxel_size = voxel_size
        self._carve_columns = carve_columns
        self._dev = dev
        self.vbg: o3d.t.geometry.VoxelBlockGrid | None = o3d.t.geometry.VoxelBlockGrid(
            attr_names=("dummy",),
            attr_dtypes=(o3c.uint8,),
            attr_channels=(o3c.SizeVector([1]),),
            voxel_size=voxel_size,
            block_resolution=1,
            block_count=block_count,
            device=dev,
        )
        self._hashmap = self.vbg.hashmap()
        self._key_dtype = self._hashmap.key_tensor().dtype

    def add_frame(self, frame: PointCloud2) -> None:
        import open3d.core as o3c  # type: ignore[import-untyped]

        values = pointcloud_xyz(frame).astype(np.float32)
        values = values[np.isfinite(values).all(axis=1)]
        if not len(values):
            return

        pts = o3c.Tensor(values, device=self._dev, dtype=o3c.float32)
        vox = (pts / self._voxel_size).floor().to(self._key_dtype)
        keys_Nx3 = vox.contiguous()

        if self._carve_columns:
            self._carve_and_insert(keys_Nx3)
        else:
            self._hashmap.activate(keys_Nx3)

        # Return Open3D's CUDA caching pool to the driver. The ops above
        # (HashMap construction in carving, key_tensor()[idx], find(),
        # activate()) allocate per-call device buffers; Open3D's caching
        # allocator holds them in pool indefinitely once the Python wrappers
        # are released. Without this call, VRAM grows ~0.8 MB/call until OOM.
        o3c.cuda.release_cache()

    def _carve_and_insert(self, new_keys: o3c.Tensor) -> None:
        """Column carving: remove all existing voxels sharing (X,Y) with new_keys, then insert."""
        import open3d.core as o3c  # type: ignore[import-untyped]

        if new_keys.shape[0] == 0:
            self._hashmap.activate(new_keys)
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

        active_indices = self._hashmap.active_buf_indices()
        if active_indices.shape[0] == 0:
            self._hashmap.activate(new_keys)
            return

        existing_keys = self._hashmap.key_tensor()[active_indices]
        existing_xy = existing_keys[:, :2].contiguous()

        _, found_mask = xy_hashmap.find(existing_xy)

        to_erase = existing_keys[found_mask]
        if to_erase.shape[0] > 0:
            self._hashmap.erase(to_erase)

        self._hashmap.activate(new_keys)

    def points(self) -> NDArray[np.float32]:
        """Voxel centers, (N, 3) float32."""
        import open3d.core as o3c  # type: ignore[import-untyped]

        assert self.vbg is not None
        voxel_coords, _ = self.vbg.voxel_coordinates_and_flattened_indices()
        # Move to CPU immediately to avoid holding a large duplicate on GPU.
        cpu = voxel_coords.to(o3c.Device("CPU:0"))
        return cpu.numpy() + np.float32(self._voxel_size * 0.5)  # type: ignore[no-any-return]

    def size(self) -> int:
        return self._hashmap.size()  # type: ignore[no-any-return]

    def dispose(self) -> None:
        """Free GPU resources."""
        self.vbg = None
        self._hashmap = None
