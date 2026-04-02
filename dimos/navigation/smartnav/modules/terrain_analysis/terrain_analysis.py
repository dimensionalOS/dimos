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

from pydantic import Field

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class TerrainAnalysisConfig(NativeModuleConfig):
    """Config for the terrain analysis native module.

    Field names map to C++ CLI args via snake_case → camelCase conversion
    (e.g. ``obstacle_height_thre`` → ``--obstacleHeightThre``).
    Fields with ``None`` default are omitted from the CLI, letting the
    C++ binary use its own built-in default.
    """

    cwd: str | None = "."
    executable: str = "result/bin/terrain_analysis"
    build_command: str | None = (
        "nix build github:dimensionalOS/dimos-module-terrain-analysis/v0.1.0 --no-write-lock-file"
    )
    cli_name_override: dict[str, str] = Field(
        default={
            "min_dy_obs_vfov": "minDyObsVFOV",
            "max_dy_obs_vfov": "maxDyObsVFOV",
        }
    )

    # --- Sensor / input filtering ---

    # Maximum range of lidar sensor used for terrain analysis (m).
    sensor_range: float = 20.0
    # Voxel size for downsampling the input registered scan (m).
    scan_voxel_size: float = 0.05
    # Terrain grid cell size (m).
    terrain_voxel_size: float = 1.0
    # Terrain grid radius in cells (full grid is 2*N+1 on a side).
    terrain_voxel_half_width: int = 10

    # --- Obstacle / ground classification ---

    # Points higher than this above ground are classified as obstacles (m).
    obstacle_height_thre: float = 0.15
    # Points lower than this are considered ground in cost-map mode (m).
    ground_height_thre: float = 0.1
    # Ignore points above this height relative to the vehicle (m).
    vehicle_height: float | None = None
    # Height-band filter: minimum z relative to robot (m).
    min_rel_z: float | None = None
    # Height-band filter: maximum z relative to robot (m).
    max_rel_z: float | None = None

    # --- Sorting / quantile ground estimation ---

    # Use quantile-based sorting for ground height estimation.
    use_sorting: bool | None = None
    # Quantile of z-values used to estimate ground height (0–1).
    quantile_z: float | None = None

    # --- Decay and clearing ---

    # How long terrain points persist before expiring (s).
    decay_time: float | None = None
    # Radius around robot where points never decay (m).
    no_decay_dis: float | None = None
    # Dynamic clearing distance — points beyond this from new obs are removed (m).
    clearing_dis: float | None = None
    # Whether to actively clear dynamic obstacles.
    clear_dy_obs: bool | None = None
    # Treat unseen (no-data) voxels as obstacles.
    no_data_obstacle: bool | None = None
    # Number of no-data blocks to skip before treating as obstacle.
    no_data_block_skip_num: int | None = None
    # Minimum points per terrain block for valid classification.
    min_block_point_num: int | None = None

    # --- Voxel culling ---

    # Reprocess a voxel after this many new points accumulate.
    voxel_point_update_thre: int | None = None
    # Cull a voxel after this many seconds since last update (s).
    voxel_time_update_thre: float | None = None

    # --- Dynamic obstacle filtering ---

    # Minimum distance from sensor for dynamic obstacle detection (m).
    min_dy_obs_dis: float | None = None
    # Absolute z-threshold for dynamic obstacle classification (m).
    abs_dy_obs_rel_z_thre: float | None = None
    # Minimum vertical FOV angle for dynamic obstacle detection (deg).
    min_dy_obs_vfov: float | None = None
    # Maximum vertical FOV angle for dynamic obstacle detection (deg).
    max_dy_obs_vfov: float | None = None
    # Minimum number of points to qualify as a dynamic obstacle.
    min_dy_obs_point_num: int | None = None
    # Minimum out-of-FOV points before classifying as dynamic.
    min_out_of_fov_point_num: int | None = None

    # --- Ground lift limits ---

    # Whether to consider terrain drops (negative slopes).
    consider_drop: bool | None = None
    # Limit how much the estimated ground plane can lift between frames.
    limit_ground_lift: bool | None = None
    # Maximum ground plane lift per frame (m).
    max_ground_lift: float | None = None
    # Distance-to-z ratio used for slope-based point filtering.
    dis_ratio_z: float | None = None


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
