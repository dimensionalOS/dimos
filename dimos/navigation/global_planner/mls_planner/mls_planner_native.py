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

"""Rust multi-level surface path planner."""

from __future__ import annotations

from pydantic import Field

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.reform import ReformConfig
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation import spec


class MLSPlannerNativeConfig(NativeModuleConfig):
    source_dir: str | None = "dimos/navigation/global_planner/mls_planner/rust"
    executable: str = "result/bin/mls_planner"
    build_command: str | None = "nix build -L path:."
    stdin_config: bool = True

    world_frame: str = "odom"
    # Frame whose tf pose in the world frame is the planning start.
    base_frame: str = "base_link"
    voxel_size: float = 0.08
    robot_height: float = 0.3
    # Height of base_frame above the ground while standing. Subtracted from
    # the start pose z before snapping to a surface.
    start_z_offset_m: float = 0.0
    max_overhead_m: float = 2.0

    surface_closing_radius: float = 0.3
    node_spacing_m: float = 1.0
    wall_clearance_m: float = 0.1
    wall_buffer_m: float = 0.75
    wall_buffer_weight: float = 100.0
    step_threshold_m: float = 0.16
    step_penalty_weight: float = 4.0
    goal_tolerance: float = 0.3
    viz_publish_hz: float = 2.0
    # The surface and edge viz publish by square cells of this edge, only the
    # changed ones each tick, plus this many unchanged ones round robin.
    viz_cell_m: float = 4.0
    viz_sweep_cells: int = 32
    # Worker threads for parallel planner work.
    worker_threads: int = 4
    # Whether and how the graph warps through each loop_closure_event.
    reform: ReformConfig = Field(default_factory=ReformConfig)
    # How far around the start, goal and route a plan re-keys cells a warp moved, m.
    reform_rekey_radius_m: float = 3.0
    # With no full route publish no path, instead of following the cached route to a standoff short of the blockage.
    require_full_route: bool = False
    # A region update spanning more columns is a bad pose or bound (10M is ~126 m of radius at 8 cm), refused before it allocates.
    max_region_columns: int = 10_000_000
    # A surface cluster spanning a box of more cells is left unclosed, as only a stray voxel makes one.
    max_closing_box_cells: int = 250_000_000


class MLSPlannerNative(NativeModule, spec.GlobalPlanner):
    """Rust-backed MLS planner.

    Feed either global_map, which rebuilds fully per message, or the local_map
    plus region_bounds pair from RayTracingVoxelMap for incremental updates.
    """

    config: MLSPlannerNativeConfig

    global_map: In[PointCloud2]
    local_map: In[PointCloud2]
    region_bounds: In[PoseStamped]
    # A seeded map's regions as RayTracingVoxelMap lands them, applied through
    # the region pipeline between live updates. Live updates keep priority.
    seed_map: In[PointCloud2]
    seed_bounds: In[PoseStamped]
    goal: In[PointStamped]
    # With config.reform.enabled: GscPGO's keyframes and per-closure moves, as RayTracingVoxelMap takes them.
    pose_graph: In[PointCloud2]
    loop_closure_event: In[PointCloud2]
    tf: In[TFMessage]

    path: Out[Path]
    surface_map: Out[PointCloud2]
    nodes: Out[PointCloud2]
    node_edges: Out[LineSegments3D]
