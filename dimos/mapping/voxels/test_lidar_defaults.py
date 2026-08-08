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

import pytest

from dimos.mapping.voxels.lidar_defaults import (
    DEFAULT_FINE_VOXEL,
    DEFAULT_RERANK_DIST,
    DEFAULT_VOXEL_SIZE,
    MID360_FINE_VOXEL,
    MID360_RERANK_DIST,
    MID360_VOXEL_SIZE,
    fine_voxel_for_lidar,
    rerank_dist_for_lidar,
    voxel_size_for_lidar,
)


def test_default_lidar_presets() -> None:
    assert voxel_size_for_lidar(None) == DEFAULT_VOXEL_SIZE
    assert voxel_size_for_lidar("default") == DEFAULT_VOXEL_SIZE
    assert fine_voxel_for_lidar(None) == DEFAULT_FINE_VOXEL
    assert fine_voxel_for_lidar("default") == DEFAULT_FINE_VOXEL
    assert rerank_dist_for_lidar(None) == DEFAULT_RERANK_DIST
    assert rerank_dist_for_lidar("default") == DEFAULT_RERANK_DIST


def test_mid360_lidar_presets() -> None:
    assert voxel_size_for_lidar("mid360") == MID360_VOXEL_SIZE
    assert fine_voxel_for_lidar("mid360") == MID360_FINE_VOXEL
    assert rerank_dist_for_lidar("mid360") == MID360_RERANK_DIST
    assert MID360_FINE_VOXEL == 0.06
    assert MID360_RERANK_DIST == 0.12


def test_unknown_lidar_config_raises() -> None:
    with pytest.raises(ValueError, match="unknown lidar_config"):
        voxel_size_for_lidar("hesai")
    with pytest.raises(ValueError, match="unknown lidar_config"):
        fine_voxel_for_lidar("hesai")
    with pytest.raises(ValueError, match="unknown lidar_config"):
        rerank_dist_for_lidar("hesai")
