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

from dimos_lcm.actionlib_msgs import GoalStatus
from reactivex.disposable import Disposable

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

_HOLDS_GOAL = frozenset({GoalStatus.PENDING, GoalStatus.ACTIVE})


def _answers(assumed: GoalStatus, msg: GoalStatus) -> bool:
    """Whether the planner's report covers the goal message a status was assumed for."""
    same_goal = msg.goal_id.id == assumed.goal_id.id
    if assumed.status == GoalStatus.PENDING:
        return bool(same_goal)
    return not same_goal or msg.status not in _HOLDS_GOAL


class MLSPlannerNativeConfig(NativeModuleConfig):
    source_dir: str | None = "dimos/navigation/global_planner/mls_planner/rust"
    # The crate is a workspace member, so cargo builds into the repo-root target dir.
    executable: str = "../../../../../target/release/mls_planner"
    build_command: str | None = "cargo build --release"
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
    viz_publish_hz: float = 2.0
    # The surface and edge viz publish by square cells of this edge, only the
    # changed ones each tick, plus this many unchanged ones round robin.
    viz_region_m: float = 4.0
    viz_sweep_regions: int = 2
    # Worker threads for parallel planner work.
    worker_threads: int = 4


class MLSPlannerNative(NativeModule, spec.GlobalPlanner, spec.NavigationInterfaceSpec):
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
    tf: In[TFMessage]

    path: Out[Path]
    nav_status: Out[GoalStatus]
    surface_map: Out[PointCloud2]
    nodes: Out[PointCloud2]
    node_edges: Out[LineSegments3D]

    _status: GoalStatus | None = None
    # The status of a sent goal message until the planner reports on it.
    _assumed: GoalStatus | None = None

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self._status_lock = threading.Lock()

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(self.nav_status.transport.subscribe(self._on_nav_status, self.nav_status))
        )

    def _on_nav_status(self, msg: GoalStatus) -> None:
        with self._status_lock:
            self._status = msg
            if self._assumed is not None and _answers(self._assumed, msg):
                self._assumed = None

    def _send(self, point: PointStamped, assumed: GoalStatus) -> None:
        with self._status_lock:
            self._assumed = assumed
        self.goal.transport.publish(point)

    @rpc
    def set_goal(self, goal: PoseStamped) -> bool:
        """Send the pose's position as the goal. The planner has no goal heading."""
        self._send(
            PointStamped(goal.x, goal.y, goal.z, ts=goal.ts, frame_id=goal.frame_id),
            spec.goal_status(goal, GoalStatus.PENDING, "planning"),
        )
        return True

    @rpc
    def cancel_goal(self) -> bool:
        """Cancel the goal. False when the planner held none."""
        cancel = PointStamped(math.nan, math.nan, math.nan)
        status = self.get_status()
        if status is None or status.status not in _HOLDS_GOAL:
            self.goal.transport.publish(cancel)
            return False
        self._send(
            cancel,
            GoalStatus(goal_id=status.goal_id, status=GoalStatus.PREEMPTED, text="canceled"),
        )
        return True

    @rpc
    def get_status(self) -> GoalStatus | None:
        with self._status_lock:
            return self._status if self._assumed is None else self._assumed
