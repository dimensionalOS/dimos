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

"""TerrainAnalysis NativeModule: C++ terrain processing for obstacle detection.

Ported from terrainAnalysis.cpp. Processes registered point clouds to produce
a terrain cost map with obstacle classification.
"""

from __future__ import annotations

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.change_detect import Glob, PathEntry


class TerrainAnalysisConfig(NativeModuleConfig):
    """Config for the terrain analysis native module."""

    cwd: str | None = "."
    executable: str = "result/bin/terrain_analysis"
    build_command: str | None = "nix build github:dimensionalOS/dimos-module-terrain-analysis/v0.1.0 --no-write-lock-file"

    # Terrain analysis parameters
    sensor_range: float = 20.0
    obstacle_height_threshold: float = 0.15
    ground_height_threshold: float = 0.1
    voxel_size: float = 0.05
    terrain_voxel_size: float = 1.0
    terrain_voxel_half_width: int = 10
    terrain_voxel_width: int = 21


class TerrainAnalysis(NativeModule):
    """Terrain analysis native module for obstacle cost map generation.

    Processes registered point clouds from SLAM to classify terrain as
    ground/obstacle, outputting a cost-annotated point cloud.

    Ports:
        registered_scan (In[PointCloud2]): World-frame registered point cloud.
        odometry (In[Odometry]): Vehicle state for local frame reference.
        terrain_map (Out[PointCloud2]): Terrain cost map (intensity=obstacle cost).
    """

    default_config: type[TerrainAnalysisConfig] = TerrainAnalysisConfig  # type: ignore[assignment]

    registered_scan: In[PointCloud2]
    odometry: In[Odometry]
    terrain_map: Out[PointCloud2]
