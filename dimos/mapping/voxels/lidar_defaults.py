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

"""Lidar-config presets for voxel map and relocalization resolution.

``default`` keeps the historical 5 cm voxels (Go2 L1 / generic). ``mid360``
uses 3 cm voxels to better preserve Mid-360 detail in premap build and live
relocalization maps, plus tighter ICP fine-voxel / rerank distance.
"""

from __future__ import annotations

from typing import Literal

LidarConfigName = Literal["default", "mid360"]

DEFAULT_VOXEL_SIZE = 0.05
MID360_VOXEL_SIZE = 0.03
DEFAULT_FINE_VOXEL = 0.1
MID360_FINE_VOXEL = 0.06
DEFAULT_RERANK_DIST = DEFAULT_FINE_VOXEL * 1.5
MID360_RERANK_DIST = 0.12

_VOXEL_SIZE_BY_LIDAR: dict[str, float] = {
    "default": DEFAULT_VOXEL_SIZE,
    "mid360": MID360_VOXEL_SIZE,
}
_FINE_VOXEL_BY_LIDAR: dict[str, float] = {
    "default": DEFAULT_FINE_VOXEL,
    "mid360": MID360_FINE_VOXEL,
}
_RERANK_DIST_BY_LIDAR: dict[str, float] = {
    "default": DEFAULT_RERANK_DIST,
    "mid360": MID360_RERANK_DIST,
}


def _preset(table: dict[str, float], lidar_config: str | None, default: float) -> float:
    if lidar_config is None:
        return default
    try:
        return table[lidar_config]
    except KeyError as e:
        known = ", ".join(sorted(table))
        raise ValueError(f"unknown lidar_config {lidar_config!r}; expected one of: {known}") from e


def voxel_size_for_lidar(lidar_config: str | None) -> float:
    """Return the default voxel size for a lidar-config name."""
    return _preset(_VOXEL_SIZE_BY_LIDAR, lidar_config, DEFAULT_VOXEL_SIZE)


def fine_voxel_for_lidar(lidar_config: str | None) -> float:
    """Return the relocalization ICP fine voxel for a lidar-config name."""
    return _preset(_FINE_VOXEL_BY_LIDAR, lidar_config, DEFAULT_FINE_VOXEL)


def rerank_dist_for_lidar(lidar_config: str | None) -> float:
    """Return the relocalization rerank / ICP inlier distance for a lidar-config name."""
    return _preset(_RERANK_DIST_BY_LIDAR, lidar_config, DEFAULT_RERANK_DIST)
