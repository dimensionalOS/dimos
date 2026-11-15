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

import math

from dimos_lcm.actionlib_msgs import GoalStatus
from dimos_lcm.std_msgs import Time
from pydantic import BaseModel, ConfigDict, Field

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation import spec


class MLSReformConfig(BaseModel):
    """How the planner follows GscPGO's loop closures (its new_keyframe and loop_closure_event inputs)."""

    model_config = ConfigDict(extra="forbid")

    # follow closures; off ignores new_keyframe and loop_closure_event
    enabled: bool = False
    # edge of a version 0 tile, the extent of a submesh, m
    tile_m: float = 10.0
    # extra tiles around each area a lookup maps to version 0
    tile_pad: int = 1
    # edge of the coarse grid keyframe boxes are filed in, m
    box_grid_m: float = 10.0
    # how far each keyframe box grows before it is filed: the most a reform moves a point off its keyframe's rigid motion, m
    box_pad_m: float = 1.0
    # past layers whose keyframe poses are kept, for clouds stamped with an older version
    version_history: int = 4
    # a closure starts a new layer when it closes a loop or moves some keyframe's box corner this far from the layer's frame, m (0 = half a voxel)
    min_shift_m: float = 0.0


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
    # Steepest voxel plane, in degrees from level, a cell takes its height from.
    max_surface_slope_deg: float = 45.0
    # How far fit noise alone moves a plane's height; smaller moves and rises are ignored.
    plane_noise_m: float = 0.01
    step_penalty_weight: float = 4.0
    goal_tolerance: float = 0.3
    goal_z_tolerance: float = 0.5
    # Seconds a goal may go without a safe path before the planner aborts it.
    blocked_timeout_s: float = 2.0
    # Seconds a followed goal may go without the robot moving before it is aborted.
    progress_timeout_s: float = 10.0
    # Ground-plane meters the robot must move within the progress timeout window.
    progress_min_m: float = 0.2
    viz_publish_hz: float = 2.0
    # The surface and edge viz publish by square cells of this edge, only the
    # changed ones each tick, plus this many unchanged ones round robin.
    viz_cell_m: float = 4.0
    viz_sweep_cells: int = 32
    # Worker threads for parallel planner work.
    worker_threads: int = 4
    # Whether and how the planner follows GscPGO's loop closures.
    reform: MLSReformConfig = Field(default_factory=MLSReformConfig)
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
    Callers drive it through the go_to and stop_navigation RPCs.
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
    # With config.reform.enabled: GscPGO's new keyframes (raw pose, scan box) and versioned closures.
    new_keyframe: In[PointCloud2]
    loop_closure_event: In[PointCloud2]
    tf: In[TFMessage]

    path: Out[Path]
    nav_status: Out[GoalStatus]
    surface_map: Out[PointCloud2]
    nodes: Out[PointCloud2]
    node_edges: Out[LineSegments3D]

    @rpc
    def stop(self) -> None:
        # A planner that was never deployed has no goal transport to cancel on.
        if self.goal.transport is not None:
            self.stop_navigation()
        super().stop()

    @rpc
    def go_to(self, pose: PoseStamped) -> Time:
        """Start driving to a pose and return the goal's stamp at once.

        Replaces any goal under way. The heading is not used and an empty frame is the
        world frame. `nav_status` reports carry the stamp in `goal_id.stamp`, and a report
        with a status other than PENDING or ACTIVE says the goal is over.
        """
        goal = PointStamped(pose.x, pose.y, pose.z, frame_id=pose.frame_id)
        self.goal.transport.publish(goal)
        return Time(*goal.ros_timestamp())

    @rpc
    def stop_navigation(self) -> None:
        """Cancel the current goal, whoever sent it, and return at once."""
        self.goal.transport.publish(PointStamped(math.nan, math.nan, math.nan))
