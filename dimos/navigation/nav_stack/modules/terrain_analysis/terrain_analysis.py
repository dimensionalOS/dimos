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
    # C++ binary uses camelCase CLI args (with VFOV all-caps).
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

    # Maximum range of lidar sensor used for terrain analysis (m).
    sensor_range: float = 20.0
    # Voxel size for downsampling the input registered scan (m).
    scan_voxel_size: float = 0.05
    # Terrain grid cell size (m).
    terrain_voxel_size: float = 1.0
    # Terrain grid radius in cells (full grid is 2*N+1 on a side).
    terrain_voxel_half_width: int = 10

    # Points higher than this above ground are classified as obstacles (m).
    obstacle_height_threshold: float = 0.15
    # Points lower than this are considered ground in cost-map mode (m).
    ground_height_threshold: float = 0.1
    # Ignore points above this height relative to the vehicle (m).
    vehicle_height: float | None = None
    # Height-band filter: minimum z relative to robot (m).
    min_relative_z: float | None = None
    # Height-band filter: maximum z relative to robot (m).
    max_relative_z: float | None = None

    # Use quantile-based sorting for ground height estimation.
    use_sorting: bool | None = None
    # Quantile of z-values used to estimate ground height (0–1).
    quantile_z: float | None = None

    # How long terrain points persist before expiring (s).
    decay_time: float | None = None
    # Radius around robot where points never decay (m).
    no_decay_distance: float | None = None
    # Dynamic clearing distance — points beyond this from new obs are removed (m).
    clearing_distance: float | None = None
    # Whether to actively clear dynamic obstacles.
    clear_dynamic_obstacles: bool | None = None
    # Treat unseen (no-data) voxels as obstacles.
    no_data_obstacle: bool | None = None
    # Number of no-data blocks to skip before treating as obstacle.
    no_data_block_skip_count: int | None = None
    # Minimum points per terrain block for valid classification.
    min_block_point_count: int | None = None

    # Reprocess a voxel after this many new points accumulate.
    voxel_point_update_threshold: int | None = None
    # Cull a voxel after this many seconds since last update (s).
    voxel_time_update_threshold: float | None = None

    # Minimum distance from sensor for dynamic obstacle detection (m).
    min_dynamic_obstacle_distance: float | None = None
    # Absolute z-threshold for dynamic obstacle classification (m).
    abs_dynamic_obstacle_relative_z_threshold: float | None = None
    # Minimum vertical FOV angle for dynamic obstacle detection (deg).
    min_dynamic_obstacle_vfov: float | None = None
    # Maximum vertical FOV angle for dynamic obstacle detection (deg).
    max_dynamic_obstacle_vfov: float | None = None
    # Minimum number of points to qualify as a dynamic obstacle.
    min_dynamic_obstacle_point_count: int | None = None
    # Minimum out-of-FOV points before classifying as dynamic.
    min_out_of_fov_point_count: int | None = None

    # Whether to consider terrain drops (negative slopes).
    consider_drop: bool | None = None
    # Limit how much the estimated ground plane can lift between frames.
    limit_ground_lift: bool | None = None
    # Maximum ground plane lift per frame (m).
    max_ground_lift: float | None = None
    # Distance-to-z ratio used for slope-based point filtering.
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
