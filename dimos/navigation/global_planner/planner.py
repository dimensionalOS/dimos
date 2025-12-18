# Copyright 2025 Dimensional Inc.
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


from reactivex.disposable import Disposable

from dimos.core import In, Module, Out, rpc
from dimos.core.global_config import GlobalConfig
from dimos.mapping.occupancy.path_map import make_navigation_map
from dimos.mapping.occupancy.path_resampling import simple_resample_path
from dimos.msgs.geometry_msgs import Pose, PoseStamped
from dimos.msgs.nav_msgs import OccupancyGrid, Path
from dimos.navigation.global_planner.astar import astar
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class AstarPlanner(Module):
    # LCM inputs
    target: In[PoseStamped]
    global_costmap: In[OccupancyGrid]
    odom: In[PoseStamped]

    # LCM outputs
    path: Out[Path]

    _global_config: GlobalConfig

    def __init__(
        self,
        global_config: GlobalConfig | None = None,
    ) -> None:
        super().__init__()

        self.latest_costmap: OccupancyGrid | None = None
        self.latest_odom: PoseStamped | None = None

        self._global_config = global_config or GlobalConfig()

    @rpc
    def start(self) -> None:
        super().start()

        self._disposables.add(Disposable(self.target.subscribe(self._on_target)))
        self._disposables.add(Disposable(self.global_costmap.subscribe(self._on_costmap)))
        self._disposables.add(Disposable(self.odom.subscribe(self._on_odom)))

    @rpc
    def stop(self) -> None:
        super().stop()

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        self.latest_costmap = msg

    def _on_odom(self, msg: PoseStamped) -> None:
        self.latest_odom = msg

    def _on_target(self, goal_pose: PoseStamped) -> None:
        if self.latest_costmap is None or self.latest_odom is None:
            logger.warning("Cannot plan: missing costmap or odometry data")
            return

        path = self.plan(goal_pose)
        if path:
            self.path.publish(path)

    def plan(self, goal_pose: Pose) -> Path | None:
        """Plan a path from current position to goal."""

        if self.latest_costmap is None or self.latest_odom is None:
            logger.warning("Cannot plan: missing costmap or odometry data")
            return None

        logger.debug(f"Planning path to goal {goal_pose}")

        robot_pos = self.latest_odom.position

        costmap = make_navigation_map(
            self.latest_costmap,
            self._global_config.robot_width,
            strategy=self._global_config.planner_strategy,
        )

        path = astar(self._global_config.astar_algorithm, costmap, goal_pose.position, robot_pos)

        if not path:
            logger.warning("No path found to the goal.")
            return None

        return simple_resample_path(path, goal_pose, 0.1)


astar_planner = AstarPlanner.blueprint

__all__ = ["AstarPlanner", "astar_planner"]
