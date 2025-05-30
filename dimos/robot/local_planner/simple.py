import math
import time
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


def transform_to_robot_frame(global_vector: Vector, robot_position: Position) -> Vector:
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
    # We need to rotate the coordinate system by -robot_yaw to get robot-relative coordinates
    cos_yaw = math.cos(-robot_yaw)
    sin_yaw = math.sin(-robot_yaw)

    # Apply 2D rotation transformation
    # This transforms a global direction vector into the robot's coordinate frame
    # In robot frame: X=forward/backward, Y=left/right
    # In global frame: X=east/west, Y=north/south
    robot_x = global_vector.x * cos_yaw - global_vector.y * sin_yaw  # Forward/backward
    robot_y = global_vector.x * sin_yaw + global_vector.y * cos_yaw  # Left/right

    return Vector(robot_y, robot_x, 0)


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
            self.spy("goal"),
            ops.map(lambda _: self.get_robot_pos().pos.to_2d() - self.goal),
            ops.filter(lambda direction: direction.length() > 0.1),
            # ops.map(self.calc_move),
            self.spy("direction"),
            # ops.map(
            #    lambda direction: transform_to_robot_frame(
            #        direction.normalize() * self.speed,
            #        self.get_robot_pos(),
            #    )
            # ),
            ops.map(self.rotate_to_target),
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
        current_yaw = self.get_robot_pos().rot.z

        # Calculate the yaw error using a more robust method to avoid oscillation
        yaw_error = math.atan2(
            math.sin(desired_yaw - current_yaw), math.cos(desired_yaw - current_yaw)
        )

        print(
            f"DEBUG: direction_to_goal={direction_to_goal}, desired_yaw={math.degrees(desired_yaw):.1f}°, current_yaw={math.degrees(current_yaw):.1f}°"
        )
        print(
            f"DEBUG: yaw_error={math.degrees(yaw_error):.1f}°, abs_error={abs(yaw_error):.3f}, tolerance=0.1"
        )

        # Calculate angular velocity (proportional control)
        max_angular_speed = 0.3  # rad/s
        raw_angular_velocity = yaw_error * 2.0
        angular_velocity = max(-max_angular_speed, min(max_angular_speed, raw_angular_velocity))

        print(
            f"DEBUG: raw_ang_vel={raw_angular_velocity:.3f}, clamped_ang_vel={angular_velocity:.3f}"
        )

        # Stop rotating if we're close enough to the target angle
        if abs(yaw_error) < 0.1:  # ~5.7 degrees tolerance
            print("DEBUG: Within tolerance - stopping rotation")
            angular_velocity = 0.0
        else:
            print("DEBUG: Outside tolerance - continuing rotation")

        print(
            f"Rotation control: current_yaw={math.degrees(current_yaw):.1f}°, desired_yaw={math.degrees(desired_yaw):.1f}°, error={math.degrees(yaw_error):.1f}°, ang_vel={angular_velocity:.3f}"
        )

        # Return movement command: no translation (x=0, y=0), only rotation (z=angular_velocity)
        # Try flipping the sign in case the rotation convention is opposite
        return Vector(0, 0, -angular_velocity)
