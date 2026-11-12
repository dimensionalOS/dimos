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
from typing import Any

from dimos_lcm.actionlib_msgs import GoalStatus
from dimos_lcm.std_msgs import Time

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
    # Seconds a go_to waits for the robot pose to show up on tf.
    pose_wait_s: float = 0.5

    def to_config_dict(self) -> dict[str, Any]:
        return {k: v for k, v in super().to_config_dict().items() if k != "pose_wait_s"}


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

    def _pose(self, wait_s: float = 0.0) -> PoseStamped | None:
        return self.tfbuffer.get_pose(
            self.config.world_frame, self.config.base_frame, forward_tolerance=wait_s
        )

    @rpc
    def go_to(self, x: float, y: float) -> Time:
        """Start driving to a point in the world frame and return the goal's stamp at once.
        Replaces any goal under way. `nav_status` reports carry the stamp in `goal_id.stamp`,
        and a report with a status other than PENDING or ACTIVE says the goal is over.
        Raises LookupError when tf has no robot pose.

        Args:
            x: target x in meters, world frame.
            y: target y in meters, world frame.
        """
        pose = self._pose(wait_s=self.config.pose_wait_s)
        if pose is None:
            raise LookupError(f"no {self.config.world_frame} -> {self.config.base_frame} on tf")
        # The planner picks the surface nearest the goal's z, so send the floor height under the robot.
        floor_z = pose.z - self.config.start_z_offset_m
        goal = PointStamped(x, y, floor_z, frame_id=self.config.world_frame)
        self.goal.transport.publish(goal)
        return Time(*goal.ros_timestamp())

    @rpc
    def stop_navigation(self) -> None:
        """Cancel the current goal, whoever sent it, and return at once."""
        self.goal.transport.publish(PointStamped(math.nan, math.nan, math.nan))
