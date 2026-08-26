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

"""Voxel indices packed into sortable int64 keys, so membership is a search."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from dimos.mapping.voxels.keys import (
    KEY_OFFSET,
    X_SHIFT,
    Y_SHIFT,
    pack_indices,
    unpack_keys,
)

if TYPE_CHECKING:
    from numpy.typing import NDArray


def voxel_keys(points: NDArray[np.float32], voxel_size: float) -> NDArray[np.int64]:
    """Pack voxel indices into sortable int64 keys, one per point."""
    # Quantized in float64, so float32 rounding cannot shift a point a voxel over.
    idx = np.floor(points.astype(np.float64) / voxel_size).astype(np.int64) + KEY_OFFSET
    return pack_indices(idx)


def key_centers(keys: NDArray[np.int64], voxel_size: float) -> NDArray[np.float32]:
    """Voxel center positions for packed keys, the inverse of voxel_keys."""
    return ((unpack_keys(keys) + 0.5) * voxel_size).astype(np.float32)


def keys_contain(sorted_keys: NDArray[np.int64], query: NDArray[np.int64]) -> NDArray[np.bool_]:
    if len(sorted_keys) == 0:
        return np.zeros(len(query), dtype=bool)
    # Unsorted keys would report almost nothing occupied, which passes every
    # gate and inflates the score instead of failing.
    assert np.all(sorted_keys[:-1] <= sorted_keys[1:]), "map keys must be sorted"
    pos = np.clip(np.searchsorted(sorted_keys, query), 0, len(sorted_keys) - 1)
    return np.asarray(sorted_keys[pos] == query)


def cylinder_offsets(
    radius: float, z_lo: float, z_hi: float, voxel_size: float
) -> NDArray[np.int64]:
    """Integer voxel offsets forming a vertical cylinder."""
    r_vox = int(np.ceil(radius / voxel_size))
    span = np.arange(-r_vox, r_vox + 1)
    dx, dy = np.meshgrid(span, span, indexing="ij")
    in_disc = (dx * voxel_size) ** 2 + (dy * voxel_size) ** 2 <= radius**2
    dz = np.arange(int(np.floor(z_lo / voxel_size)), int(np.ceil(z_hi / voxel_size)) + 1)
    disc = np.stack([dx[in_disc], dy[in_disc]], axis=1)
    out = np.concatenate([np.hstack([disc, np.full((len(disc), 1), z)]) for z in dz])
    return np.asarray(out, dtype=np.int64)


def offset_deltas(offsets: NDArray[np.int64]) -> NDArray[np.int64]:
    """Packed key deltas for integer voxel offsets."""
    # The fields sit far from their bounds, so a packed add carries no bits over.
    return np.asarray((offsets[:, 0] << X_SHIFT) + (offsets[:, 1] << Y_SHIFT) + offsets[:, 2])
