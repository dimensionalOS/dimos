from dataclasses import dataclass
from abc import abstractmethod

from typing import Callable, Optional
from dimos.types.path import Path
from dimos.types.costmap import Costmap
from dimos.types.vector import VectorLike, to_vector, Vector
from dimos.types.position import PositionLike, to_position, Position
from dimos.robot.global_planner.algo import astar
from dimos.utils.logging_config import setup_logger
from dimos.web.websocket_vis.helpers import Visualizable


def obstacles_from_costmap(costmap: Costmap) -> list[Vector]:
    """Get the obstacle positions from the costmap."""
    obstacles = []
    for i in range(costmap.width):
        for j in range(costmap.height):
            if costmap[i, j] == 1:
                obstacles.append(Vector(i, j))
    return obstacles


@dataclass
class LocalPlanner(Visualizable):
    get_costmap: Callable[[], Costmap]
    get_robot_pos: Callable[[], Position]
    set_robot_vel: Callable[[Position], None]

    goal: Position

    def set_goal(self, goal: PositionLike):
        self.goal = to_position(goal)

    @abstractmethod
    def move(self, goal: PositionLike) -> bool:
        ...
        # calculate the most free direction given obstacles
