import math
from dataclasses import dataclass
from typing import Callable, Optional

import reactivex as rx
from reactivex import operators as ops
from plum import dispatch

from dimos.robot.local_planner.local_planner import LocalPlanner
from dimos.types.costmap import Costmap
from dimos.types.position import Position
from dimos.types.path import Path
from dimos.types.vector import Vector, VectorLike, to_vector
from dimos.utils.threadpool import get_scheduler
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.robot.unitree.global_planner")


@dataclass
class SimplePlanner(LocalPlanner):
    get_costmap: Callable[[], Costmap]
    get_robot_pos: Callable[[], Position]
    goal: Optional[Vector] = None
    speed: float = 0.2

    @dispatch
    def set_goal(self, goal: Path, stop_event=None, goal_theta=None) -> bool:
        self.goal = goal.last().to_2d()
        logger.info(f"Setting goal: {self.goal}")
        return True

    @dispatch
    def set_goal(self, goal: VectorLike, stop_event=None, goal_theta=None) -> bool:
        self.goal = to_vector(goal).to_2d()
        logger.info(f"Setting goal: {self.goal}")
        return True

    def _transform_to_robot_frame(self, global_vector: Vector, robot_position: Position) -> Vector:
        """Transform a global coordinate vector to robot-relative coordinates.

        Args:
            global_vector: Vector in global coordinates
            robot_position: Robot's position and orientation

        Returns:
            Vector in robot coordinates where X is forward/backward, Y is left/right
        """
        # Get the robot's yaw angle (rotation around Z-axis)
        robot_yaw = robot_position.rot.z

        # Create rotation matrix to transform from global to robot frame
        cos_yaw = math.cos(-robot_yaw)  # Negative because we're rotating coordinate frame
        sin_yaw = math.sin(-robot_yaw)

        # Apply 2D rotation transformation
        robot_x = global_vector.x * cos_yaw - global_vector.y * sin_yaw  # Forward/backward
        robot_y = global_vector.x * sin_yaw + global_vector.y * cos_yaw  # Left/right

        return Vector(robot_x, robot_y, global_vector.z)

    def calc_move(self, direction: Vector) -> Vector:
        """Calculate the movement vector based on the direction to the goal.

        Args:
            direction: Direction vector towards the goal

        Returns:
            Movement vector scaled by speed
        """
        try:
            # Normalize the direction vector and scale by speed
            normalized_direction = direction.normalize()
            move_vector = normalized_direction * self.speed
            print("CALC MOVE", direction, normalized_direction, move_vector)
            return move_vector
        except Exception as e:
            print("Error calculating move vector:", e)

    def spy(self, name: str):
        def spyfun(x):
            print(f"SPY {name}:", x)
            return x

        return ops.map(spyfun)

    def get_move_stream(self, frequency: float = 40.0) -> rx.Observable:
        return rx.interval(1.0 / frequency, scheduler=get_scheduler()).pipe(
            # do we have a goal?
            ops.filter(lambda _: self.goal is not None),
            # calculate direction vector
            ops.map(lambda _: self.get_robot_pos().pos.to_2d() - self.goal),
            ops.filter(lambda direction: direction.length() > 0.1),
            # ops.map(self.calc_move),
            self.spy("direction"),
            ops.map(
                lambda direction: self._transform_to_robot_frame(
                    direction.normalize() * self.speed,
                    self.get_robot_pos(),
                )
            ),
        )
