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

"""Lidar-config presets for voxel map resolution.

``default`` keeps the historical 5 cm voxels (Go2 L1 / generic). ``mid360``
uses 3 cm voxels to better preserve Mid-360 detail in premap build and live
relocalization maps.
"""

from __future__ import annotations

from typing import Literal

LidarConfigName = Literal["default", "mid360"]

DEFAULT_VOXEL_SIZE = 0.05
MID360_VOXEL_SIZE = 0.03

_VOXEL_SIZE_BY_LIDAR: dict[str, float] = {
    "default": DEFAULT_VOXEL_SIZE,
    "mid360": MID360_VOXEL_SIZE,
}


def voxel_size_for_lidar(lidar_config: str | None) -> float:
    """Return the default voxel size for a lidar-config name."""
    if lidar_config is None:
        return DEFAULT_VOXEL_SIZE
    try:
        return _VOXEL_SIZE_BY_LIDAR[lidar_config]
    except KeyError as e:
        known = ", ".join(sorted(_VOXEL_SIZE_BY_LIDAR))
        raise ValueError(f"unknown lidar_config {lidar_config!r}; expected one of: {known}") from e
