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

from collections.abc import Callable

from dimos_generated.sensor_msgs.msg import PointCloud2
from dimos_generated.std_msgs.msg import Header
import numpy as np

from dimos.mapping.voxels.keys import FIELD_BITS as _BITS, FIELD_MASK as _MASK, KEY_OFFSET as _BIAS
from dimos.msgs.pointcloud import pointcloud_from_xyz, pointcloud_xyz
from dimos.msgs.time import time_from_seconds, to_seconds


class PackedVoxels:
    """CPU voxel store: sorted int64 keys, 21 bits/axis, (x,y) in the high bits.

    Column carving is contiguous-range deletion (searchsorted), insertion is a
    sorted merge — O(map) at memcpy speed per frame, single-threaded, no Open3D
    ops. ~25x faster than the Open3D CPU hashmap path on recorded go2 data.

    `stamped` also remembers, per voxel, the ts of the frame that wrote it and
    the key it was inserted at, so `reproject` can move the map later.
    """

    def __init__(self, voxel_size: float, carve_columns: bool, stamped: bool = False) -> None:
        self._voxel_size = voxel_size
        self._carve_columns = carve_columns
        self.stamped = stamped
        self._keys = np.empty(0, dtype=np.int64)
        self._ts = np.empty(0, dtype=np.float64)
        self._raw = np.empty(0, dtype=np.int64)

    def _pack(self, pts: np.ndarray) -> np.ndarray:
        vox = np.floor(pts / np.float32(self._voxel_size)).astype(np.int64)
        if np.abs(vox).max(initial=0) >= _BIAS:
            raise ValueError(f"point outside +-{_BIAS * self._voxel_size:.0f} m packed range")
        vox += _BIAS
        return (vox[:, 0] << (2 * _BITS)) | (vox[:, 1] << _BITS) | vox[:, 2]

    def _centers(self, k: np.ndarray) -> np.ndarray:
        vox = np.empty((len(k), 3), dtype=np.float32)
        vox[:, 0] = (k >> (2 * _BITS)) - _BIAS
        vox[:, 1] = ((k >> _BITS) & _MASK) - _BIAS
        vox[:, 2] = (k & _MASK) - _BIAS
        return (vox + np.float32(0.5)) * np.float32(self._voxel_size)

    def add_frame(self, frame: PointCloud2) -> None:
        pts = pointcloud_xyz(frame).astype(np.float32)
        pts = pts[np.isfinite(pts).all(axis=1)]
        if not len(pts):
            return
        new = np.unique(self._pack(pts))

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
            keep = np.cumsum(delta[:-1]) == 0
            keys = keys[keep]
            if self.stamped:
                self._ts, self._raw = self._ts[keep], self._raw[keep]

        # merge two sorted arrays; carving already emptied `new`'s columns, so
        # duplicates only need filtering in union mode
        pos = np.searchsorted(keys, new)
        if not self._carve_columns and len(keys):
            fresh = (pos == len(keys)) | (keys[np.minimum(pos, len(keys) - 1)] != new)
            pos, new = pos[fresh], new[fresh]
        self._keys = np.insert(keys, pos, new)
        if self.stamped:
            self._ts = np.insert(self._ts, pos, to_seconds(frame.header.stamp))
            self._raw = np.insert(self._raw, pos, new)

    def reproject(self, place: Callable[[PointCloud2], PointCloud2]) -> None:
        """Move every voxel to where `place` puts its stamped original position."""
        if not self.stamped:
            raise RuntimeError("reproject needs a stamped store")
        if not len(self._keys):
            return
        keys = self._pack(
            pointcloud_xyz(
                place(
                    pointcloud_from_xyz(
                        self._centers(self._raw),
                        header=Header(stamp=time_from_seconds(0.0), frame_id=""),
                        stamps=self._ts,
                    )
                )
            )
        )
        order = np.lexsort((-self._ts, keys))  # by key, latest stamp first
        keys, ts, raw = keys[order], self._ts[order], self._raw[order]
        if self._carve_columns:
            # carving invariant: a column belongs to the latest frame that landed in it
            col = keys >> _BITS
            starts = np.flatnonzero(np.r_[True, col[1:] != col[:-1]])
            latest = np.repeat(np.maximum.reduceat(ts, starts), np.diff(np.r_[starts, len(keys)]))
            keep = ts == latest
            keys, ts, raw = keys[keep], ts[keep], raw[keep]
        first = np.r_[True, keys[1:] != keys[:-1]]
        self._keys, self._ts, self._raw = keys[first], ts[first], raw[first]

    def points(self) -> np.ndarray:
        """Voxel centers, (N, 3) float32."""
        return self._centers(self._keys)

    def size(self) -> int:
        return len(self._keys)

    def dispose(self) -> None:
        self._keys = np.empty(0, dtype=np.int64)
        self._ts = np.empty(0, dtype=np.float64)
        self._raw = np.empty(0, dtype=np.int64)
