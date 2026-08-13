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

"""The int64 voxel key layout shared by every packed voxel store.

Three 21-bit signed-biased indices, x in the high bits, so keys sort in the
same order as (x, y, z) and membership is a sorted-array search.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from numpy.typing import NDArray

FIELD_BITS = 21
# Voxel coords in [-2^20, 2^20): +-52 km at 5 cm voxels.
KEY_OFFSET = 1 << (FIELD_BITS - 1)
X_SHIFT = 2 * FIELD_BITS
Y_SHIFT = FIELD_BITS
FIELD_MASK = (1 << FIELD_BITS) - 1


def pack_indices(idx: NDArray[np.int64]) -> NDArray[np.int64]:
    """Pack biased (N, 3) integer voxel indices into sortable int64 keys."""
    return (idx[:, 0] << X_SHIFT) | (idx[:, 1] << Y_SHIFT) | idx[:, 2]


def unpack_keys(keys: NDArray[np.int64]) -> NDArray[np.int64]:
    """Unbiased (N, 3) integer voxel indices, the inverse of pack_indices."""
    return (
        np.stack([keys >> X_SHIFT, (keys >> Y_SHIFT) & FIELD_MASK, keys & FIELD_MASK], axis=1)
        - KEY_OFFSET
    )
