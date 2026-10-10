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

from dimos_generated.geometry_msgs.msg import Point, PointStamped, PoseStamped
from dimos_generated.std_msgs.msg import Bool
from dimos_generated.tf2_msgs.msg import TFMessage

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.time import header_now


class PointNavSkillContainerConfig(ModuleConfig):
    world_frame: str = "world"
    """Frame of the goal coordinates."""
    base_frame: str = "base_link"
    """The robot's body frame on tf."""
    base_height_m: float = 0.0
    """Height of base_frame above the floor; matches the planner's start_z_offset_m."""
    arrival_radius_m: float = 0.3
    """A goal nearer than this is already reached; matches the planner's goal_tolerance."""


class PointNavSkillContainer(Module):
    """Drive to world points through a planner that takes a ``goal`` point and replans until arrival."""

    config: PointNavSkillContainerConfig

    tf: In[TFMessage]
    goal_reached: In[Bool]

    goal: Out[PointStamped]
    stop_movement: Out[Bool]

    _timeout: asyncio.TimerHandle | None = None
    """Set while a go_to is under way; fires when it runs out of time."""
    _target: tuple[float, float] = (0.0, 0.0)
    """The x, y of the go_to under way."""

    async def main(self) -> AsyncIterator[None]:
        # Subscribe to tf now so the first go_to already has a pose.
        self.tfbuffer  # noqa: B018
        yield
        self._finish("Cancelled, module stopping")

    async def handle_goal_reached(self, msg: Bool) -> None:
        if msg.data:
            self._finish("Path following ended")

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

        # TODO: fix this in the planner. It drops a goal the robot is already within its
        # goal_tolerance of without publishing anything, so no goal_reached would ever end this
        # go_to. Until the planner announces such a goal, repeat its check here.
        distance = math.dist((x, y), (pose.pose.position.x, pose.pose.position.y))
        if distance < self.config.arrival_radius_m:
            self.stop_tool("go_to")
            return (
                f"Already there: robot at ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}), "
                f"{distance:.2f} m from the target."
            )

        # One x, y can have walkable surfaces at several heights, e.g. downstairs and upstairs.
        # The planner picks the one nearest the goal's z, so send the height of the floor under
        # the robot.
        floor_z = pose.pose.position.z - self.config.base_height_m
        self.goal.publish(
            PointStamped(
                header=header_now(self.config.world_frame), point=Point(x=x, y=y, z=floor_z)
            )
        )
        self._target = (x, y)
        self._timeout = asyncio.get_running_loop().call_later(
            timeout_s, self._finish, f"Gave up after {timeout_s:g}s"
        )
        return "Navigating. A tool update reports the robot's position when it stops."

    def _finish(self, outcome: str) -> None:
        if self._timeout is None:
            return
        self._timeout.cancel()
        self._timeout = None
        # A NaN goal cancels the planner's goal
        self.goal.publish(
            PointStamped(
                header=header_now(self.config.world_frame),
                point=Point(x=math.nan, y=math.nan, z=math.nan),
            )
        )
        # Tell the BasicPathFollower to stop
        self.stop_movement.publish(Bool(True))
        pose = self._pose()
        if pose is None:
            where = "robot position unknown"
        else:
            distance = math.dist(self._target, (pose.pose.position.x, pose.pose.position.y))
            where = f"robot at ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}), {distance:.2f} m from the target"
        self.tool_update("go_to", f"{outcome}; {where}.")
        self.stop_tool("go_to")

    @skill
    async def stop_navigation(self) -> str:
        """Cancel the current goal and stop moving."""
        self._finish("Cancelled")
        return "Stopped."
