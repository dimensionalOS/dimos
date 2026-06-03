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

"""Smoke tests for the pyo3-bound VoxelRayMap."""

from __future__ import annotations

import numpy as np
import pytest

from dimos.mapping.ray_tracing.voxel_map import VoxelRayMap


def _mapper() -> VoxelRayMap:
    return VoxelRayMap(
        voxel_size=1.0,
        max_range=100.0,
        ray_subsample=1,
        shadow_depth=0.0,
        grace_depth=0.0,
        min_health=0,
        max_health=1,
    )


def test_add_frame_round_trip() -> None:
    mapper = _mapper()
    points = np.array(
        [
            [5.5, 0.5, 0.5],
            [0.5, 5.5, 0.5],
        ],
        dtype=np.float32,
    )
    mapper.add_frame(points, (0.0, 0.0, 0.0))

    voxels = mapper.global_map()
    assert voxels.dtype == np.float32
    assert voxels.shape == (2, 3)

    centers = {tuple(row) for row in voxels.tolist()}
    assert (5.5, 0.5, 0.5) in centers
    assert (0.5, 5.5, 0.5) in centers


def test_add_frame_rejects_wrong_shape() -> None:
    mapper = _mapper()
    bad = np.array([1.0, 2.0, 3.0], dtype=np.float32)
    with pytest.raises(ValueError, match="N, 3"):
        mapper.add_frame(bad, (0.0, 0.0, 0.0))
