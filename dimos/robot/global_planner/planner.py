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

from dimos.robot.module_utils import robot_module
from dimos.robot.capabilities import Move, Lidar, Odometry
from abc import abstractmethod
from typing import Callable, Optional
import threading

from dimos.types.path import Path
from dimos.types.costmap import Costmap
from dimos.types.vector import VectorLike, to_vector, Vector
from dimos.robot.global_planner.algo import astar
from dimos.utils.logging_config import setup_logger
from dimos.web.websocket_vis.helpers import Visualizable
from dimos.utils.reactive import getter_streaming
from dimos.robot.unitree_webrtc.type.map import Map

logger = setup_logger("dimos.robot.unitree.global_planner")


class Planner(Visualizable):
    """Base class for path planners."""

    # Instance variable declaration - cleaner than in __init__
    set_local_nav: Optional[Callable[[Path, Optional[threading.Event]], bool]] = None

    def __init__(self):
        # Note: I originally moved this to __init__ when removing @dataclass
        # but it's cleaner as a class attribute with type hint above
        pass

    @abstractmethod
    def plan(self, goal: VectorLike) -> Optional[Path]: ...

    def set_goal(
        self,
        goal: VectorLike,
        goal_theta: Optional[float] = None,
        stop_event: Optional[threading.Event] = None,
    ) -> bool:
        path = self.plan(goal)
        if not path:
            logger.warning("No path found to the goal.")
            return False

        print("pathing success", path)
        if self.set_local_nav:
            return self.set_local_nav(path, stop_event=stop_event, goal_theta=goal_theta)
        return False


@robot_module
class AstarPlanner(Planner):
    REQUIRES = (Move, Lidar, Odometry)

    def __init__(self, conservativism: int = 8):
        super().__init__()
        self.conservativism = conservativism
        # Initialize placeholder values
        self.get_costmap = None
        self.get_robot_pos = None
        self.set_local_nav = None

    def setup(self, robot):
        # Build lambdas that access robot state lazily
        from dimos.utils.reactive import getter_streaming
        from dimos.robot.unitree_webrtc.type.map import Map
        from dimos.robot.local_planner.local_planner import navigate_path_local

        # Create map and streams
        lidar_stream = robot.lidar_stream()
        odom_stream = robot.odom_stream()
        self.map = Map(voxel_size=0.2)
        self.map_stream = self.map.consume(lidar_stream)
        self.odom = getter_streaming(odom_stream)

        # Set up callbacks
        self.get_costmap = lambda: self.map.costmap
        self.get_robot_pos = lambda: self.odom().pos
        self.set_local_nav = lambda path, stop_event=None, goal_theta=None: navigate_path_local(
            robot, path, timeout=120.0, goal_theta=goal_theta, stop_event=stop_event
        )

    def plan(self, goal: VectorLike) -> Optional[Path]:
        if not self.get_costmap or not self.get_robot_pos:
            logger.error("AstarPlanner not properly initialized")
            return None

        goal = to_vector(goal).to_2d()
        pos = self.get_robot_pos().to_2d()
        costmap = self.get_costmap().smudge()

        # self.vis("costmap", costmap)
        self.vis("target", goal)

        print("ASTAR ", costmap, goal, pos)
        path = astar(costmap, goal, pos)

        if path:
            path = path.resample(0.1)
            self.vis("a*", path)
            return path

        logger.warning("No path found to the goal.")
        return None
