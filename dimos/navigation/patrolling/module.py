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
from collections.abc import AsyncGenerator

from dimos_lcm.std_msgs import Bool

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.navigation.patrolling.constants import EXTRA_CLEARANCE
from dimos.navigation.patrolling.create_patrol_router import create_patrol_router
from dimos.navigation.patrolling.routers.patrol_router import PatrolRouter
from dimos.navigation.replanning_a_star.module_spec import ReplanningAStarPlannerSpec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class PatrollingModule(Module):
    odom: In[PoseStamped]
    global_costmap: In[OccupancyGrid]
    goal_reached: In[Bool]
    goal_request: Out[PoseStamped]

    _global_config: GlobalConfig
    _router: PatrolRouter
    _planner_spec: ReplanningAStarPlannerSpec
    _latest_pose: PoseStamped | None = None
    _patrol_task: asyncio.Task[None] | None = None

    _clearance_multiplier = 0.5

    def __init__(self, g: GlobalConfig = global_config) -> None:
        super().__init__()
        self._global_config = g
        clearance_radius_m = self._global_config.robot_width * self._clearance_multiplier
        self._router = create_patrol_router("coverage", clearance_radius_m)
        self._goal_reached_event = asyncio.Event()

    async def main(self) -> AsyncGenerator[None, None]:
        yield
        await self._stop_patrolling()

    async def handle_odom(self, msg: PoseStamped) -> None:
        self._latest_pose = msg
        self._router.handle_odom(msg)

    async def handle_global_costmap(self, msg: OccupancyGrid) -> None:
        self._router.handle_occupancy_grid(msg)

    async def handle_goal_reached(self, _msg: Bool) -> None:
        self._goal_reached_event.set()

    @skill
    async def start_patrol(self) -> str:
        """Start patrolling the known area. The robot will continuously pick patrol goals from the router and navigate to them until `stop_patrol` is called."""
        if self._patrol_task is not None and not self._patrol_task.done():
            return "Patrol is already running. Use `stop_patrol` to stop."

        self._router.reset()
        self._planner_spec.set_replanning_enabled(False)
        self._planner_spec.set_safe_goal_clearance(
            self._global_config.robot_rotation_diameter / 2 + EXTRA_CLEARANCE
        )
        self._patrol_task = asyncio.create_task(self._patrol_loop())
        return "Patrol started. Use `stop_patrol` to stop."

    @rpc
    def is_patrolling(self) -> bool:
        return self._patrol_task is not None and not self._patrol_task.done()

    @skill
    async def stop_patrol(self) -> str:
        """Stop the ongoing patrol."""
        await self._stop_patrolling()
        return "Patrol stopped."

    async def _stop_patrolling(self) -> None:
        if self._patrol_task is not None and not self._patrol_task.done():
            self._patrol_task.cancel()
            try:
                await self._patrol_task
            except asyncio.CancelledError:
                pass
        self._patrol_task = None
        self._planner_spec.set_replanning_enabled(True)
        self._planner_spec.reset_safe_goal_clearance()
        if self._latest_pose is not None:
            self.goal_request.publish(self._latest_pose)

    async def _patrol_loop(self) -> None:
        while True:
            goal = self._router.next_goal()
            if goal is None:
                logger.info("No patrol goal available, retrying in 2s")
                await asyncio.sleep(2.0)
                continue

            self._goal_reached_event.clear()
            self.goal_request.publish(goal)
            await self._goal_reached_event.wait()
