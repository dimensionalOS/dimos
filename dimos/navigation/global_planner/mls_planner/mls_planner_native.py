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
from typing import NamedTuple

from dimos_lcm.actionlib_msgs import GoalStatus
from reactivex.disposable import Disposable

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.LineSegments3D import LineSegments3D
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation import spec
from dimos.navigation.spec import cancel_goal, position_goal
from dimos.utils.trigonometry import angle_diff

# Seconds a go_to waits for the robot pose to show up on tf.
POSE_WAIT_S = 0.5


def reports_goal(msg: GoalStatus, route: Path) -> bool:
    """Whether a nav_status report is about this route, which its stamp says."""
    return [msg.goal_id.stamp.sec, msg.goal_id.stamp.nsec] == route.ros_timestamp()


class Underway(NamedTuple):
    """The route a skill sent and the skill, which names its tool stream."""

    route: Path
    tool: str


def waypoint(w: dict[str, float]) -> tuple[float, float]:
    """The x and y of a waypoint argument, or a ValueError naming what is wrong with it."""
    try:
        x, y = float(w["x"]), float(w["y"])
    except (KeyError, TypeError, ValueError) as e:
        raise ValueError(f"a waypoint needs x and y, got {w!r}") from e
    if not (math.isfinite(x) and math.isfinite(y)):
        raise ValueError(f"a waypoint must be finite, got {w!r}")
    return x, y


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
    # Heading error under which a goal with a heading counts as reached.
    goal_yaw_tolerance: float = 0.25
    # Seconds a goal may go without a safe path before the planner aborts it.
    blocked_timeout_s: float = 2.0
    # Seconds a followed goal may go without the robot moving before it is aborted.
    progress_timeout_s: float = 10.0
    # Ground-plane distance the robot must cover to count as moving.
    progress_min_m: float = 0.2
    viz_publish_hz: float = 2.0
    # The surface and edge viz publish by square cells of this edge, only the
    # changed ones each tick, plus this many unchanged ones round robin.
    viz_cell_m: float = 4.0
    viz_sweep_cells: int = 32
    # Worker threads for parallel planner work.
    worker_threads: int = 4


class MLSPlannerNative(NativeModule, spec.GlobalPlanner):
    """Rust-backed MLS planner.

    Feed either global_map, which rebuilds fully per message, or the local_map
    plus region_bounds pair from RayTracingVoxelMap for incremental updates.
    The agent drives it through go_to, go_through and stop_navigation.
    """

    config: MLSPlannerNativeConfig

    global_map: In[PointCloud2]
    local_map: In[PointCloud2]
    region_bounds: In[PoseStamped]
    # A seeded map's regions as RayTracingVoxelMap lands them, applied through
    # the region pipeline between live updates. Live updates keep priority.
    seed_map: In[PointCloud2]
    seed_bounds: In[PoseStamped]
    goal: In[PoseStamped]
    # A route to drive through in one go, under its header stamp as the goal id.
    waypoints: In[Path]
    tf: In[TFMessage]

    path: Out[Path]
    nav_status: Out[GoalStatus]
    surface_map: Out[PointCloud2]
    nodes: Out[PointCloud2]
    node_edges: Out[LineSegments3D]

    _goal: Underway | None = None

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
        if goal is None or not reports_goal(msg, goal.route) or holds_goal(msg.status):
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
    async def go_to(self, x: float, y: float, degrees: float | None = None) -> str:
        """Start driving to a point in the world frame and return immediately.

        Replaces any goal under way. A tool update reports where the robot is and how far
        from the point when it stops, which can be short of it.

        Args:
            x: target x in meters, world frame.
            y: target y in meters, world frame.
            degrees: heading to face there, world frame, 0 = +x and 90 = +y. Omit for any.
        """
        return self._send("go_to", [(x, y)], degrees)

    @skill(uses=[CAP_MOVEMENT], lifecycle="background")
    async def go_through(
        self, waypoints: list[dict[str, float]], degrees: float | None = None
    ) -> str:
        """Drive through the waypoints in order without stopping, and return immediately.

        Replaces any goal under way. The robot passes within reach of every waypoint and stops
        at the last one. A tool update reports where the robot is and how far from the last
        waypoint when it stops, which can be short of it.

        Args:
            waypoints: points in the world frame, in order, each {"x": meters, "y": meters}.
            degrees: heading to face at the last waypoint, world frame, 0 = +x and 90 = +y. Omit for any.
        """
        points = [waypoint(w) for w in waypoints]
        if not points:
            raise ValueError("go_through needs at least one waypoint")
        return self._send("go_through", points, degrees)

    def _send(self, tool: str, points: list[tuple[float, float]], degrees: float | None) -> str:
        pose = self._pose(wait_s=POSE_WAIT_S)
        # Close the old goal's stream first so start_tool opens a new one for this call.
        self._finish("Replaced by another goal", cancel=pose is None)
        # Opened before any return so the movement claim always has a stream to release it.
        self.start_tool(tool)
        if pose is None:
            self.stop_tool(tool)
            return f"No goal sent: no {self.config.world_frame} -> {self.config.base_frame} on tf."

        # The planner picks the surface nearest each point's z, so send the floor height under the robot.
        floor_z = pose.z - self.config.start_z_offset_m
        frame = self.config.world_frame
        poses = [position_goal(x, y, floor_z, frame) for x, y in points]
        if degrees is not None:
            x, y = points[-1]
            facing = Quaternion.from_euler(Vector3(0.0, 0.0, math.radians(degrees)))
            poses[-1] = PoseStamped(frame_id=frame, position=[x, y, floor_z], orientation=facing)
        route = Path(frame_id=frame, poses=poses)
        with self._goal_lock:
            self._goal = Underway(route, tool)
        self.waypoints.transport.publish(route)
        return "Navigating. A tool update reports the robot's position when it stops."

    def _finish(self, outcome: str, *, cancel: bool = True, ended: Underway | None = None) -> None:
        with self._goal_lock:
            if ended is not None and self._goal is not ended:
                return
            goal, self._goal = self._goal, None
        if goal is None:
            return
        target = goal.route.poses[-1]
        pose = self._pose()
        if pose is None:
            where = "robot position unknown"
        else:
            distance = math.dist((target.x, target.y), (pose.x, pose.y))
            where = f"robot at ({pose.x:.2f}, {pose.y:.2f}), {distance:.2f} m from the target"
            if not target.orientation.is_zero():
                off = math.degrees(abs(angle_diff(target.yaw, pose.yaw)))
                where += f", {off:.0f} degrees off its heading"
        self.tool_update(goal.tool, f"{outcome}; {where}.")
        self.stop_tool(goal.tool)
        if cancel:
            self._cancel()

    def _cancel(self) -> None:
        self.goal.transport.publish(cancel_goal(self.config.world_frame))

    @skill
    async def stop_navigation(self) -> str:
        """Cancel the current goal and stop moving."""
        # The goal may have come from a click, so cancel whether or not a go_to is under way.
        self._finish("Canceled", cancel=False)
        self._cancel()
        return "Stopped."
