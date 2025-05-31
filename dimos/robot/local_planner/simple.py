import math
import time
from datetime import datetime
from dataclasses import dataclass
from typing import Callable, Optional

import reactivex as rx
from plum import dispatch
from reactivex import operators as ops

from dimos.robot.local_planner.local_planner import LocalPlanner
from dimos.types.costmap import Costmap
from dimos.types.path import Path
from dimos.types.position import Position
from dimos.types.vector import Vector, VectorLike, to_vector
from dimos.utils.logging_config import setup_logger
from dimos.utils.threadpool import get_scheduler

logger = setup_logger("dimos.robot.unitree.global_planner")


@dataclass
class SimplePlanner(LocalPlanner):
    get_costmap: Callable[[], Costmap]
    get_robot_pos: Callable[[], Position]
    goal: Optional[Vector] = None
    speed: float = 0.3

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

    def spy(self, name: str):
        def spyfun(x):
            print(f"SPY {name}:", x)
            return x

        return ops.map(spyfun)

    def get_move_stream(self, frequency: float = 40.0) -> rx.Observable:
        return rx.interval(1.0 / frequency, scheduler=get_scheduler()).pipe(
            ops.filter(lambda _: self.goal is not None),
            ops.map(lambda _: self.get_robot_pos()),
            ops.map(lambda odom: odom.vector_to(self.goal)),
            ops.filter(lambda direction: direction.length() > 0.1),
            ops.map(lambda direction: direction.normalize() * self.speed),
            ops.map(lambda direction: Vector(-direction.y, direction.x)),
        )

    def rotate_to_target(self, direction_to_goal: Vector) -> Vector:
        """Calculate the rotation needed for the robot to face the target.

        Args:
            direction_to_goal: Vector pointing from robot position to goal in global coordinates

        Returns:
            Vector with (x=0, y=0, z=angular_velocity) for rotation only
        """
        # Calculate the desired yaw angle to face the target
        desired_yaw = math.atan2(direction_to_goal.y, direction_to_goal.x)

        # Get current robot yaw
        current_yaw = self.get_robot_pos().yaw

        # Calculate the yaw error using a more robust method to avoid oscillation
        yaw_error = math.atan2(
            math.sin(desired_yaw - current_yaw), math.cos(desired_yaw - current_yaw)
        )

        # print(
        #     f"DEBUG: direction_to_goal={direction_to_goal}, desired_yaw={math.degrees(desired_yaw):.1f}°, current_yaw={math.degrees(current_yaw):.1f}°"
        # )
        # print(
        #     f"DEBUG: yaw_error={math.degrees(yaw_error):.1f}°, abs_error={abs(yaw_error):.3f}, tolerance=0.1"
        # )

        # Calculate angular velocity (proportional control)
        max_angular_speed = 0.2  # rad/s
        raw_angular_velocity = 100.0 / yaw_error
        angular_velocity = max(-max_angular_speed, min(max_angular_speed, raw_angular_velocity))

        # Stop rotating if we're close enough to the target angle
        if abs(yaw_error) < 0.1:  # ~5.7 degrees tolerance
            angular_velocity = 0.0

        # print(
        #     f"Rotation control: current_yaw={math.degrees(current_yaw):.1f}°, desired_yaw={math.degrees(desired_yaw):.1f}°, error={math.degrees(yaw_error):.1f}°, ang_vel={angular_velocity:.3f}"
        # )

        # Return movement command: no translation (x=0, y=0), only rotation (z=angular_velocity)
        # Try flipping the sign in case the rotation convention is opposite
        return Vector(0, 0, -angular_velocity)
