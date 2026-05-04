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

from __future__ import annotations

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class TerrainAnalysisConfig(NativeModuleConfig):
    cwd: str | None = "."
    executable: str = "result/bin/terrain_analysis"
    build_command: str | None = (
        "nix build github:dimensionalOS/dimos-module-terrain-analysis/v0.1.1 --no-write-lock-file"
    )
    cli_name_override: dict[str, str] = {
        "sensor_range": "sensorRange",
        "scan_voxel_size": "scanVoxelSize",
        "terrain_voxel_size": "terrainVoxelSize",
        "terrain_voxel_half_width": "terrainVoxelHalfWidth",
        "obstacle_height_threshold": "obstacleHeightThre",
        "ground_height_threshold": "groundHeightThre",
        "vehicle_height": "vehicleHeight",
        "min_relative_z": "minRelZ",
        "max_relative_z": "maxRelZ",
        "use_sorting": "useSorting",
        "quantile_z": "quantileZ",
        "decay_time": "decayTime",
        "no_decay_distance": "noDecayDis",
        "clearing_distance": "clearingDis",
        "clear_dynamic_obstacles": "clearDyObs",
        "no_data_obstacle": "noDataObstacle",
        "no_data_block_skip_count": "noDataBlockSkipNum",
        "min_block_point_count": "minBlockPointNum",
        "voxel_point_update_threshold": "voxelPointUpdateThre",
        "voxel_time_update_threshold": "voxelTimeUpdateThre",
        "min_dynamic_obstacle_distance": "minDyObsDis",
        "abs_dynamic_obstacle_relative_z_threshold": "absDyObsRelZThre",
        "min_dynamic_obstacle_vfov": "minDyObsVFOV",
        "max_dynamic_obstacle_vfov": "maxDyObsVFOV",
        "min_dynamic_obstacle_point_count": "minDyObsPointNum",
        "min_out_of_fov_point_count": "minOutOfFovPointNum",
        "consider_drop": "considerDrop",
        "limit_ground_lift": "limitGroundLift",
        "max_ground_lift": "maxGroundLift",
        "distance_ratio_z": "disRatioZ",
    }

    sensor_range: float = 20.0  # m
    scan_voxel_size: float = 0.05  # m
    terrain_voxel_size: float = 1.0  # m
    terrain_voxel_half_width: int = 10  # cells (full grid = 2*N+1)

    obstacle_height_threshold: float = 0.15  # m
    ground_height_threshold: float = 0.1  # m
    vehicle_height: float | None = None  # m
    min_relative_z: float | None = None  # m
    max_relative_z: float | None = None  # m

    use_sorting: bool | None = None
    quantile_z: float | None = None

    decay_time: float | None = None  # s
    no_decay_distance: float | None = None  # m
    clearing_distance: float | None = None  # m
    clear_dynamic_obstacles: bool | None = None
    no_data_obstacle: bool | None = None
    no_data_block_skip_count: int | None = None
    min_block_point_count: int | None = None

    voxel_point_update_threshold: int | None = None
    voxel_time_update_threshold: float | None = None  # s

    min_dynamic_obstacle_distance: float | None = None  # m
    abs_dynamic_obstacle_relative_z_threshold: float | None = None  # m
    min_dynamic_obstacle_vfov: float | None = None  # deg
    max_dynamic_obstacle_vfov: float | None = None  # deg
    min_dynamic_obstacle_point_count: int | None = None
    min_out_of_fov_point_count: int | None = None

    consider_drop: bool | None = None
    limit_ground_lift: bool | None = None
    max_ground_lift: float | None = None  # m
    distance_ratio_z: float | None = None


class TerrainAnalysis(NativeModule):
    config: TerrainAnalysisConfig

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    registered_scan: In[PointCloud2]
    odometry: In[Odometry]
    terrain_map: Out[PointCloud2]
