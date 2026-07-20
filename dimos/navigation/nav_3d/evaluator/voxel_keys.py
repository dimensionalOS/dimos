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

"""Voxel indices packed into sortable int64 keys.

Membership tests over a map run as a sorted-array search on these keys rather
than a per-point spatial query, which is what makes the gates cheap enough to
sweep a whole path.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from numpy.typing import NDArray

_KEY_OFFSET = 1 << 20


def voxel_keys(points: NDArray[np.float32], voxel_size: float) -> NDArray[np.int64]:
    """Pack voxel indices into sortable int64 keys, one per point."""
    idx = np.floor(points.astype(np.float64) / voxel_size).astype(np.int64) + _KEY_OFFSET
    return (idx[:, 0] << 42) | (idx[:, 1] << 21) | idx[:, 2]


def key_centers(keys: NDArray[np.int64], voxel_size: float) -> NDArray[np.float32]:
    """Voxel center positions for packed keys, the inverse of voxel_keys."""
    mask = (1 << 21) - 1
    idx = np.stack([keys >> 42, (keys >> 21) & mask, keys & mask], axis=1) - _KEY_OFFSET
    return ((idx + 0.5) * voxel_size).astype(np.float32)


def keys_contain(sorted_keys: NDArray[np.int64], query: NDArray[np.int64]) -> NDArray[np.bool_]:
    if len(sorted_keys) == 0:
        return np.zeros(len(query), dtype=bool)
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


def offset_keys(
    points: NDArray[np.float32], offsets: NDArray[np.int64], voxel_size: float
) -> NDArray[np.int64]:
    """Keys of every (point voxel + offset) pair, shape (P * O,)."""
    idx = np.floor(points.astype(np.float64) / voxel_size).astype(np.int64) + _KEY_OFFSET
    swept = idx[:, None, :] + offsets[None, :, :]
    return np.asarray((swept[..., 0] << 42) | (swept[..., 1] << 21) | swept[..., 2])
