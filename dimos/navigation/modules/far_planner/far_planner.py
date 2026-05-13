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

from pathlib import Path

from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.nav_msgs.ContourPolygons3D import ContourPolygons3D
from dimos.msgs.nav_msgs.GraphNodes3D import GraphNodes3D
from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class FarPlannerConfig(NativeModuleConfig):
    cwd: str | None = str(Path(__file__).resolve().parent)
    executable: str = "result/bin/far_planner_native"
    build_command: str | None = (
        "nix build github:dimensionalOS/dimos-module-far-planner/v0.5.0 --no-write-lock-file"
    )

    # C++ binary uses snake_case CLI args.
    cli_name_override: dict[str, str] = {
        "robot_dimension": "robot_dim",
    }

    update_rate: float = 5.0
    robot_dimension: float = 0.5
    voxel_dim: float = 0.1
    sensor_range: float = 30.0
    terrain_range: float = 7.5
    local_planner_range: float = 2.5
    vehicle_height: float = 0.75
    is_static_env: bool = True
    is_viewpoint_extend: bool = True
    is_multi_layer: bool = False
    is_debug_output: bool = False
    is_attempt_autoswitch: bool = True
    world_frame: str = "map"

    converge_dist: float = 1.5
    goal_adjust_radius: float = 10.0
    free_counter_thred: int = 5
    reach_goal_vote_size: int = 5
    path_momentum_thred: int = 5

    floor_height: float = 2.0
    cell_length: float = 5.0
    map_grid_max_length: float = 1000.0
    map_grad_max_height: float = 100.0

    connect_votes_size: int = 10
    clear_dumper_thred: int = 3
    node_finalize_thred: int = 3
    filter_pool_size: int = 12

    resize_ratio: float = 5.0
    filter_count_value: int = 5

    angle_noise: float = 15.0
    accept_max_align_angle: float = 15.0
    new_intensity_thred: float = 2.0
    dynamic_obs_decay_time: float = 10.0
    new_points_decay_time: float = 2.0
    dyobs_update_thred: int = 4
    new_point_counter: int = 10
    obs_inflate_size: int = 2
    visualize_ratio: float = 0.4
    # 0 → republish way_point every cycle (matches OG ros-nav behaviour).
    # The C++ default suppresses way_point updates until the goal moves by
    # this many metres, which makes the live sim feel stuck.
    wp_churn_dist: float = 0.0


class FarPlanner(NativeModule):
    """Note: 2D planner, supposed to be really good at large maps"""

    config: FarPlannerConfig

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        super().stop()

    terrain_map_ext: In[PointCloud2]
    terrain_map: In[PointCloud2]
    registered_scan: In[PointCloud2]
    odometry: In[Odometry]
    goal: In[PointStamped]
    stop_movement: In[Bool]
    way_point: Out[PointStamped]
    goal_path: Out[NavPath]
    graph_nodes: Out[GraphNodes3D]
    graph_edges: Out[LineSegments3D]
    contour_polygons: Out[ContourPolygons3D]
    nav_boundary: Out[LineSegments3D]
