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

import asyncio
from collections.abc import AsyncIterator
import math

from dimos_lcm.actionlib_msgs import GoalStatus

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.navigation.spec import NavigationInterfaceSpec, goal_id

# The planner retries a blocked goal on every map update, so a block only
# counts once it has lasted this long.
BLOCKED_SETTLE_S = 2.0


class PointNavSkillContainerConfig(ModuleConfig):
    world_frame: str = "world"
    """Frame of the goal coordinates."""
    base_frame: str = "base_link"
    """The robot's body frame on tf."""
    base_height_m: float = 0.0
    """Height of base_frame above the floor; matches the planner's start_z_offset_m."""


class PointNavSkillContainer(Module):
    """Drive to world points through a planner that reports each goal's outcome on nav_status."""

    config: PointNavSkillContainerConfig

    _navigation: NavigationInterfaceSpec

    tf: In[TFMessage]
    nav_status: In[GoalStatus]

    _timeout: asyncio.TimerHandle | None = None
    """Set while a go_to is under way; fires when it runs out of time."""
    _blocked: asyncio.TimerHandle | None = None
    """Set while the planner reports the goal blocked. Fires if it stays blocked."""
    _target: tuple[float, float] = (0.0, 0.0)
    """The x, y of the go_to under way."""
    _goal: PoseStamped | None = None
    """The goal of the go_to under way."""

    async def main(self) -> AsyncIterator[None]:
        # Subscribe to tf now so the first go_to already has a pose.
        self.tfbuffer  # noqa: B018
        yield
        self._finish("Cancelled, module stopping")

    async def handle_nav_status(self, msg: GoalStatus) -> None:
        goal = self._goal
        if self._timeout is None or goal is None:
            return
        if msg.goal_id.id != goal_id(goal):
            # A newer goal means the report that ended this one was missed.
            if [msg.goal_id.stamp.sec, msg.goal_id.stamp.nsec] > goal.ros_timestamp():
                self._finish("Replaced by another goal", cancel=False)
            return

        if msg.status == GoalStatus.PREEMPTED:
            replaced = msg.text == "replaced"
            self._finish("Replaced by another goal" if replaced else "Cancelled", cancel=False)
        elif msg.status == GoalStatus.SUCCEEDED:
            self._finish("Reached the target", cancel=False)
        elif msg.status == GoalStatus.REJECTED:
            self._finish(f"Failed: {msg.text}", cancel=False)
        elif msg.status == GoalStatus.ABORTED:
            if self._blocked is None:
                self._blocked = asyncio.get_running_loop().call_later(
                    BLOCKED_SETTLE_S, self._finish, f"Blocked: {msg.text}"
                )
        elif self._blocked is not None:
            self._blocked.cancel()
            self._blocked = None

    def _pose(self) -> PoseStamped | None:
        return self.tfbuffer.get_pose(self.config.world_frame, self.config.base_frame)

    @skill(uses=[CAP_MOVEMENT], lifecycle="background")
    async def go_to(self, x: float, y: float, timeout_s: float = 90.0) -> str:
        """Start driving to a point in the world frame and return immediately.

        A tool update reports where the robot is and how far from the point when it stops,
        which can be short of it. Call stop_navigation before sending another go_to.

        Args:
            x: target x in metres, world frame.
            y: target y in metres, world frame.
            timeout_s: seconds allowed before the goal is cancelled and the robot stops.
        """
        # Opened before any return so the movement claim always has a stream to release it.
        self.start_tool("go_to")
        if self._timeout is not None:
            return "Already navigating. Call stop_navigation first."

        pose = self._pose()
        if pose is None:
            self.stop_tool("go_to")
            return f"No goal sent: no {self.config.world_frame} -> {self.config.base_frame} on tf."

        # One x, y can have walkable surfaces at several heights, e.g. downstairs and upstairs.
        # The planner picks the one nearest the goal's z, so send the height of the floor under
        # the robot.
        floor_z = pose.z - self.config.base_height_m
        self._target = (x, y)
        self._goal = PoseStamped(position=(x, y, floor_z), frame_id=self.config.world_frame)
        self._timeout = asyncio.get_running_loop().call_later(
            timeout_s, self._finish, f"Gave up after {timeout_s:g}s"
        )
        self._navigation.set_goal(self._goal)
        return "Navigating. A tool update reports the robot's position when it stops."

    def _finish(self, outcome: str, *, cancel: bool = True) -> None:
        if self._timeout is None:
            return
        self._timeout.cancel()
        self._timeout = None
        if self._blocked is not None:
            self._blocked.cancel()
            self._blocked = None
        pose = self._pose()
        if pose is None:
            where = "robot position unknown"
        else:
            distance = math.dist(self._target, (pose.x, pose.y))
            where = f"robot at ({pose.x:.2f}, {pose.y:.2f}), {distance:.2f} m from the target"
        self.tool_update("go_to", f"{outcome}; {where}.")
        self.stop_tool("go_to")
        # Last, so a planner that does not answer cannot hold up the report.
        if cancel:
            self._navigation.cancel_goal()

    @skill
    async def stop_navigation(self) -> str:
        """Cancel the current goal and stop moving."""
        self._finish("Cancelled")
        return "Stopped."
