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

"""FarPlanner NativeModule: C++ visibility-graph route planner.

Ported from far_planner + boundary_handler + graph_decoder. Builds a
visibility graph from the classified terrain map, finds routes to goals,
and outputs intermediate waypoints for the local planner.
"""

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
    """Config for the FAR planner native module."""

    cwd: str | None = str(Path(__file__).resolve().parent)
    executable: str = "result/bin/far_planner_native"
    build_command: str | None = (
        "nix build github:dimensionalOS/dimos-module-far-planner/v0.4.0 --no-write-lock-file"
    )

    # C++ binary uses snake_case CLI args.
    cli_name_override: dict[str, str] = {
        "robot_dimension": "robot_dim",
    }

    # --- Core planner parameters (mirrors LoadROSParams) ---
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

    # --- Graph planner params ---
    converge_dist: float = 1.5
    goal_adjust_radius: float = 10.0
    free_counter_thred: int = 5
    reach_goal_vote_size: int = 5
    path_momentum_thred: int = 5

    # --- Map handler params ---
    floor_height: float = 2.0
    cell_length: float = 5.0
    map_grid_max_length: float = 1000.0
    map_grad_max_height: float = 100.0

    # --- Dynamic graph params ---
    connect_votes_size: int = 10
    clear_dumper_thred: int = 3
    node_finalize_thred: int = 3
    filter_pool_size: int = 12

    # --- Contour detector params ---
    resize_ratio: float = 5.0
    filter_count_value: int = 5

    # --- Utility params ---
    angle_noise: float = 15.0
    accept_max_align_angle: float = 15.0
    new_intensity_thred: float = 2.0
    dynamic_obs_decay_time: float = 10.0
    new_points_decay_time: float = 2.0
    dyobs_update_thred: int = 4
    new_point_counter: int = 10
    obs_inflate_size: int = 2
    visualize_ratio: float = 0.4


class FarPlanner(NativeModule):
    """FAR planner: visibility-graph global route planner.

    Builds and maintains a visibility graph from classified terrain maps,
    then finds shortest paths through the graph to navigation goals.
    Outputs intermediate waypoints for the local planner.

    Ports:
        terrain_map_ext (In[PointCloud2]): Extended terrain map (classified obstacles).
        terrain_map (In[PointCloud2]): Scan-based terrain map (alternative input).
        registered_scan (In[PointCloud2]): Raw lidar scan (for dynamic obs detection).
        odometry (In[Odometry]): Vehicle state (corrected by PGO).
        goal (In[PointStamped]): User-specified navigation goal.
        stop_movement (In[Bool]): Cancel active goal and go idle.
        way_point (Out[PointStamped]): Intermediate waypoint for local planner.
        goal_path (Out[NavPath]): Full planned path to goal.
    """

    config: FarPlannerConfig

    @rpc
    def start(self) -> None:
        super().start()

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
