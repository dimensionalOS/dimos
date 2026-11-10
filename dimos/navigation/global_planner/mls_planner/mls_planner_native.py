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
import threading
from typing import Any

from dimos_lcm.actionlib_msgs import GoalStatus
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.message_utils import time as message_time
from dimos_messages.dimos_msgs.msg import LineSegments3DRegion, PointCloudRegion, RegionBounds
from dimos_messages.geometry_msgs.msg import Point, PointStamped, PoseStamped
from dimos_messages.nav_msgs.msg import Path
from dimos_messages.sensor_msgs.msg import PointCloud2
from dimos_messages.tf2_msgs.msg import TFMessage
from dimos.navigation import spec


def reports_goal(msg: GoalStatus, goal: PointStamped) -> bool:
    """Whether a nav_status report is about this goal, which its stamp says."""
    return [msg.goal_id.stamp.sec, msg.goal_id.stamp.nsec] == [
        goal.header.stamp.sec,
        goal.header.stamp.nanosec,
    ]


def holds_goal(status: int) -> bool:
    """Whether a status means the planner still holds the goal."""
    return status in (GoalStatus.PENDING, GoalStatus.ACTIVE)


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
    The agent drives it through go_to and stop_navigation.
    """

    config: MLSPlannerNativeConfig

    global_map: In[PointCloud2]
    local_map: In[PointCloud2]
    region_bounds: In[RegionBounds]
    # A seeded map's regions as RayTracingVoxelMap lands them, applied through
    # the region pipeline between live updates. Live updates keep priority.
    seed_map: In[PointCloudRegion]
    seed_bounds: In[RegionBounds]
    goal: In[PointStamped]
    tf: IO[TFMessage]
    tf_static: In[TFMessage]

    path: Out[Path]
    nav_status: Out[GoalStatus]
    surface_map: Out[PointCloudRegion]
    nodes: Out[PointCloud2]
    node_edges: Out[LineSegments3DRegion]

    _goal: PointStamped | None = None
    """The goal of the go_to under way."""

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._goal_lock = threading.Lock()

    @rpc
    def start(self) -> None:
        super().start()
        # The binary publishes nav_status. The wrapper reads it back to end go_to.
        self.register_disposable(
            Disposable(self.nav_status.transport.subscribe(self._on_nav_status, self.nav_status))
        )

    @rpc
    def stop(self) -> None:
        self._finish("Canceled, module stopping")
        super().stop()

    def _on_nav_status(self, msg: GoalStatus) -> None:
        with self._goal_lock:
            goal = self._goal
        if goal is None or not reports_goal(msg, goal) or holds_goal(msg.status):
            return
        if msg.status == GoalStatus.SUCCEEDED:
            outcome = "Reached the target"
        elif msg.status == GoalStatus.PREEMPTED:
            outcome = msg.text.capitalize()
        else:
            outcome = f"Failed: {msg.text}"
        self._finish(outcome, cancel=False, ended=goal)

    def _pose(self, wait_s: float = 0.0) -> PoseStamped | None:
        return self.tfbuffer.get_pose(
            self.config.world_frame, self.config.base_frame, forward_tolerance=wait_s
        )

    @skill(uses=[CAP_MOVEMENT], lifecycle="background")
    async def go_to(self, x: float, y: float) -> str:
        """Start driving to a point in the world frame and return immediately.

        Replaces any goal under way. A tool update reports where the robot is and how far
        from the point when it stops, which can be short of it.

        Args:
            x: target x in meters, world frame.
            y: target y in meters, world frame.
        """
        pose = self._pose(wait_s=self.config.pose_wait_s)
        # Close the old go_to's stream first so start_tool opens a new one for this call.
        self._finish("Replaced by another goal", cancel=pose is None)
        # Opened before any return so the movement claim always has a stream to release it.
        self.start_tool("go_to")
        if pose is None:
            self.stop_tool("go_to")
            return f"No goal sent: no {self.config.world_frame} -> {self.config.base_frame} on tf."

        # The planner picks the surface nearest the goal's z, so send the floor height under the robot.
        floor_z = pose.pose.position.z - self.config.start_z_offset_m
        # Its stamp names the goal in the nav_status reports that follow.
        goal = PointStamped(
            header=message_time.header_now(self.config.world_frame),
            point=Point(x, y, floor_z),
        )
        with self._goal_lock:
            self._goal = goal
        self.goal.transport.publish(goal)
        return "Navigating. A tool update reports the robot's position when it stops."

    def _finish(
        self, outcome: str, *, cancel: bool = True, ended: PointStamped | None = None
    ) -> None:
        with self._goal_lock:
            if ended is not None and self._goal is not ended:
                return
            goal, self._goal = self._goal, None
        if goal is None:
            return
        pose = self._pose()
        if pose is None:
            where = "robot position unknown"
        else:
            at = pose.pose.position
            distance = math.dist((goal.point.x, goal.point.y), (at.x, at.y))
            where = f"robot at ({at.x:.2f}, {at.y:.2f}), {distance:.2f} m from the target"
        self.tool_update("go_to", f"{outcome}; {where}.")
        self.stop_tool("go_to")
        if cancel:
            self._cancel()

    def _cancel(self) -> None:
        self.goal.transport.publish(
            PointStamped(
                header=message_time.header_now(), point=Point(math.nan, math.nan, math.nan)
            )
        )

    @skill
    async def stop_navigation(self) -> str:
        """Cancel the current goal and stop moving."""
        # The goal may have come from a click, so cancel whether or not a go_to is under way.
        self._finish("Canceled", cancel=False)
        self._cancel()
        return "Stopped."
