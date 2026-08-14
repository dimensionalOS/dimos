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

import numpy as np

from dimos.mapping.voxels.keys import FIELD_BITS as _BITS, FIELD_MASK as _MASK, KEY_OFFSET as _BIAS
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class PackedVoxels:
    """CPU voxel store: sorted int64 keys, 21 bits/axis, (x,y) in the high bits.

    Column carving is contiguous-range deletion (searchsorted), insertion is a
    sorted merge — O(map) at memcpy speed per frame, single-threaded, no Open3D
    ops. ~25x faster than the Open3D CPU hashmap path on recorded go2 data.
    """

    def __init__(self, voxel_size: float, carve_columns: bool) -> None:
        self._voxel_size = voxel_size
        self._carve_columns = carve_columns
        self._keys = np.empty(0, dtype=np.int64)

    def add_frame(self, frame: PointCloud2) -> None:
        pts = frame.points_f32()
        if not len(pts):
            return
        vox = np.floor(pts / np.float32(self._voxel_size)).astype(np.int64)
        if np.abs(vox).max(initial=0) >= _BIAS:
            raise ValueError(f"point outside +-{_BIAS * self._voxel_size:.0f} m packed range")
        vox += _BIAS
        new = np.unique((vox[:, 0] << (2 * _BITS)) | (vox[:, 1] << _BITS) | vox[:, 2])

        keys = self._keys
        if self._carve_columns and len(keys):
            # drop every existing voxel whose (x,y) column is touched by `new`;
            # each column is the contiguous key range [xy<<21, (xy+1)<<21)
            cols = np.unique(new >> _BITS)
            starts = np.searchsorted(keys, cols << _BITS, side="left")
            ends = np.searchsorted(keys, (cols + 1) << _BITS, side="left")
            delta = np.zeros(len(keys) + 1, dtype=np.int32)
            np.add.at(delta, starts, 1)
            np.add.at(delta, ends, -1)
            keys = keys[np.cumsum(delta[:-1]) == 0]

        # merge two sorted arrays; carving already emptied `new`'s columns, so
        # duplicates only need filtering in union mode
        pos = np.searchsorted(keys, new)
        if not self._carve_columns and len(keys):
            fresh = (pos == len(keys)) | (keys[np.minimum(pos, len(keys) - 1)] != new)
            pos, new = pos[fresh], new[fresh]
        self._keys = np.insert(keys, pos, new)

    def points(self) -> np.ndarray:
        """Voxel centers, (N, 3) float32."""
        k = self._keys
        vox = np.empty((len(k), 3), dtype=np.float32)
        vox[:, 0] = (k >> (2 * _BITS)) - _BIAS
        vox[:, 1] = ((k >> _BITS) & _MASK) - _BIAS
        vox[:, 2] = (k & _MASK) - _BIAS
        return (vox + np.float32(0.5)) * np.float32(self._voxel_size)

    def size(self) -> int:
        return len(self._keys)

    def dispose(self) -> None:
        self._keys = np.empty(0, dtype=np.int64)
