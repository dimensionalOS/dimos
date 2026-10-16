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

"""Multi-level surface path planner."""

from __future__ import annotations

import math
import threading

from dimos_lcm.actionlib_msgs import GoalStatus
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
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
from dimos.types.timestamped import Timestamped

PREEMPT_REPLACED = "replaced"
PREEMPT_CANCELED = "canceled"


def holds_goal(status: int) -> bool:
    """Whether a status means the planner still holds the goal."""
    return status in (GoalStatus.PENDING, GoalStatus.ACTIVE)


def goal_id(goal: Timestamped) -> str:
    """The id nav_status reports carry for a goal, which is its stamp."""
    sec, nsec = goal.ros_timestamp()
    return f"{sec}.{nsec:09d}"


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
    # Ground-plane distance the robot must cover to count as moving.
    progress_min_m: float = 0.2
    viz_publish_hz: float = 2.0
    # The surface and edge viz publish by square cells of this edge, only the
    # changed ones each tick, plus this many unchanged ones round robin.
    viz_region_m: float = 4.0
    viz_sweep_regions: int = 2
    # Worker threads for parallel planner work.
    worker_threads: int = 4


class MLSPlannerNative(NativeModule, spec.GlobalPlanner):
    """Rust-backed MLS planner.

    Feed either global_map, which rebuilds fully per message, or the local_map
    plus region_bounds pair from RayTracingVoxelMap for incremental updates.
    The agent drives it through go_to and stop_navigation.
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
    # Each goal's status under the id of the message that set it.
    nav_status: Out[GoalStatus]
    surface_map: Out[PointCloud2]
    nodes: Out[PointCloud2]
    node_edges: Out[LineSegments3D]

    _goal: PoseStamped | None = None
    """The goal of the go_to under way. None when there is none."""

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._goal_lock = threading.Lock()

    @rpc
    def start(self) -> None:
        super().start()
        # Subscribe to tf now so the first go_to already has a pose.
        self.tfbuffer  # noqa: B018

        # so this is weird to subscribe to the output of this module, but this is how the python
        # wrapper gets the updated nav status from the binary, so it can end the skill call
        # correctly.
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
        if goal is None or msg.goal_id.id != goal_id(goal) or holds_goal(msg.status):
            return
        if msg.status == GoalStatus.SUCCEEDED:
            outcome = "Reached the target"
        elif msg.status == GoalStatus.PREEMPTED:
            replaced = msg.text == PREEMPT_REPLACED
            outcome = "Replaced by another goal" if replaced else "Canceled"
        elif msg.status == GoalStatus.REJECTED:
            outcome = f"Failed: {msg.text}"
        else:
            outcome = f"Blocked: {msg.text}"
        self._finish(outcome, cancel=False, ended=goal)

    def _pose(self) -> PoseStamped | None:
        return self.tfbuffer.get_pose(self.config.world_frame, self.config.base_frame)

    @skill(uses=[CAP_MOVEMENT], lifecycle="background")
    async def go_to(self, x: float, y: float) -> str:
        """Start driving to a point in the world frame and return immediately.

        A tool update reports where the robot is and how far from the point when it stops,
        which can be short of it. Call stop_navigation before sending another go_to.

        Args:
            x: target x in meters, world frame.
            y: target y in meters, world frame.
        """
        # Opened before any return so the movement claim always has a stream to release it.
        self.start_tool("go_to")
        with self._goal_lock:
            busy = self._goal is not None
        if busy:
            return "Already navigating. Call stop_navigation first."

        pose = self._pose()
        if pose is None:
            self.stop_tool("go_to")
            return f"No goal sent: no {self.config.world_frame} -> {self.config.base_frame} on tf."

        # One x, y can have walkable surfaces at several heights, e.g. downstairs and upstairs.
        # The planner picks the one nearest the goal's z, so send the height of the floor under
        # the robot.
        floor_z = pose.z - self.config.start_z_offset_m
        goal = PoseStamped(position=(x, y, floor_z), frame_id=self.config.world_frame)
        with self._goal_lock:
            self._goal = goal
        self.goal.transport.publish(
            PointStamped(goal.x, goal.y, goal.z, ts=goal.ts, frame_id=goal.frame_id)
        )
        return "Navigating. A tool update reports the robot's position when it stops."

    def _finish(
        self, outcome: str, *, cancel: bool = True, ended: PoseStamped | None = None
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
            distance = math.dist((goal.x, goal.y), (pose.x, pose.y))
            where = f"robot at ({pose.x:.2f}, {pose.y:.2f}), {distance:.2f} m from the target"
        self.tool_update("go_to", f"{outcome}; {where}.")
        self.stop_tool("go_to")
        if cancel:
            self.goal.transport.publish(PointStamped(math.nan, math.nan, math.nan))

    @skill
    async def stop_navigation(self) -> str:
        """Cancel the current goal and stop moving."""
        self._finish("Canceled")
        return "Stopped."
