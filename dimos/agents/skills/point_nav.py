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

import math
from threading import Event
import time
from typing import Any

from dimos_lcm.std_msgs import Bool

from dimos.agents.annotation import skill
from dimos.agents.capabilities import CAP_MOVEMENT
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.tf2_msgs.TFMessage import TFMessage


class PointNavSkillContainerConfig(ModuleConfig):
    world_frame: str = "world"
    """Frame of the goal coordinates."""
    base_frame: str = "base_link"
    """The robot's body frame on tf."""
    base_height_m: float = 0.0
    """Height of base_frame above the floor; matches the planner's start_z_offset_m."""
    goal_tolerance: float = 0.5
    """Arrival distance in xy, wider than BasicPathFollower's 0.3 m stop radius."""
    max_wait_s: float = 100.0
    """Longest go_to waits before giving up; under the 120 s MCP client and RPC call timeouts."""
    poll_interval_s: float = 0.25
    """Seconds between position checks while go_to waits."""


class PointNavSkillContainer(Module):
    """Drive to world points through a planner that takes a ``goal`` point and replans until arrival."""

    config: PointNavSkillContainerConfig

    tf: In[TFMessage]

    goal: Out[PointStamped]
    stop_movement: Out[Bool]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._cancelled = Event()

    @rpc
    def start(self) -> None:
        super().start()
        # Subscribe to tf now so the first go_to already has a pose.
        self.tfbuffer  # noqa: B018

    def _pose(self) -> PoseStamped | None:
        return self.tfbuffer.get_pose(self.config.world_frame, self.config.base_frame)

    def _cancel_goal(self) -> None:
        # A NaN goal cancels the planner's goal
        self.goal.publish(
            PointStamped(math.nan, math.nan, math.nan, frame_id=self.config.world_frame)
        )
        # Tell the BasicPathFollower to stop
        self.stop_movement.publish(Bool(True))

    @skill(uses=[CAP_MOVEMENT])
    def go_to(
        self, x: float, y: float, wait_s: float = 90.0, poll_interval_s: float | None = None
    ) -> str:
        """Drive to a point in the world frame and wait until the robot is there.

        Args:
            x: target x in metres, world frame.
            y: target y in metres, world frame.
            wait_s: seconds to wait for arrival. When it runs out the goal is cancelled and the
                robot stops.
            poll_interval_s: seconds between position checks while waiting. Leave unset for the
                module's default.
        """
        pose = self._pose()
        if pose is None:
            return f"No goal sent: no {self.config.world_frame} -> {self.config.base_frame} on tf."

        self._cancelled.clear()

        # One x, y can have walkable surfaces at several heights, e.g. downstairs and upstairs.
        # The planner picks the one nearest the goal's z, so send the height of the floor under
        # the robot.
        floor_z = pose.z - self.config.base_height_m
        self.goal.publish(PointStamped(x, y, floor_z, frame_id=self.config.world_frame))

        wait_s = min(wait_s, self.config.max_wait_s)
        deadline = time.monotonic() + wait_s
        if poll_interval_s is None or poll_interval_s <= 0:
            poll_interval_s = self.config.poll_interval_s

        while time.monotonic() < deadline and not self._cancelled.is_set():
            # Check if we've reached the target
            pose = self._pose() or pose
            if math.hypot(x - pose.x, y - pose.y) <= self.config.goal_tolerance:
                return f"Reached ({x:.2f}, {y:.2f}); robot at ({pose.x:.2f}, {pose.y:.2f})."

            self._cancelled.wait(poll_interval_s)

        if self._cancelled.is_set():
            outcome = "Cancelled"
        else:
            self._cancel_goal()
            outcome = f"Gave up after {wait_s:.0f}s and stopped"
        return f"{outcome}; robot was at ({pose.x:.2f}, {pose.y:.2f})."

    @skill
    def stop_navigation(self) -> str:
        """Cancel the current goal and stop moving."""
        # stop the loop in self.go_to
        self._cancel_goal()
        # stop the Planner and BasicPathFollower
        self._cancelled.set()
        return "Stopped."
