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

import math
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional

import reactivex as rx
from plum import dispatch
from reactivex import operators as ops

from dimos.core import In, Module, Out, rpc
from dimos.msgs.geometry_msgs import PoseStamped, Vector3
from dimos.robot.unitree_webrtc.type.odometry import Odometry

# from dimos.robot.local_planner.local_planner import LocalPlanner
from dimos.types.costmap import Costmap
from dimos.types.path import Path
from dimos.types.pose import Pose
from dimos.types.vector import Vector, VectorLike, to_vector
from dimos.utils.logging_config import setup_logger
from dimos.utils.threadpool import get_scheduler

logger = setup_logger("dimos.robot.unitree.global_planner")


def transform_to_robot_frame(global_vector: Vector, robot_position: Pose) -> Vector:
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

    return Vector(-robot_x, robot_y, 0)


class SimplePlanner(Module):
    path: In[Path] = None
    odom: In[PoseStamped] = None
    movecmd: Out[Vector3] = None

    get_costmap: Callable[[], Costmap]

    latest_odom: PoseStamped = None

    goal: Optional[Vector] = None
    speed: float = 0.3

    def __init__(
        self,
        get_costmap: Callable[[], Costmap],
    ):
        Module.__init__(self)
        self.get_costmap = get_costmap

    def get_move_stream(self, frequency: float = 40.0) -> rx.Observable:
        return rx.interval(1.0 / frequency, scheduler=get_scheduler()).pipe(
            # do we have a goal?
            ops.filter(lambda _: self.goal is not None),
            # For testing: make robot move left/right instead of rotating
            ops.map(lambda _: self._test_translational_movement()),
            self.frequency_spy("movement_test"),
        )

    @rpc
    def start(self):
        self.path.subscribe(self.set_goal)

        def setodom(odom: Odometry):
            self.latest_odom = odom

        self.odom.subscribe(setodom)
        self.get_move_stream(frequency=20.0).subscribe(self.movecmd.publish)

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

    def frequency_spy(self, name: str, window_size: int = 10):
        """Create a frequency spy that logs message rate over a sliding window.

        Args:
            name: Name for the spy output
            window_size: Number of messages to average frequency over
        """
        timestamps = []

        def freq_spy_fun(x):
            current_time = time.time()
            timestamps.append(current_time)
            print(x)
            # Keep only the last window_size timestamps
            if len(timestamps) > window_size:
                timestamps.pop(0)

            # Calculate frequency if we have enough samples
            if len(timestamps) >= 2:
                time_span = timestamps[-1] - timestamps[0]
                if time_span > 0:
                    frequency = (len(timestamps) - 1) / time_span
                    print(f"FREQ SPY {name}: {frequency:.2f} Hz ({len(timestamps)} samples)")
                else:
                    print(f"FREQ SPY {name}: calculating... ({len(timestamps)} samples)")
            else:
                print(f"FREQ SPY {name}: warming up... ({len(timestamps)} samples)")

            return x

        return ops.map(freq_spy_fun)

    def _test_translational_movement(self) -> Vector:
        """Test translational movement by alternating left and right movement.

        Returns:
            Vector with (x=0, y=left/right, z=0) for testing left-right movement
        """
        # Use time to alternate between left and right movement every 3 seconds
        current_time = time.time()
        cycle_time = 6.0  # 6 second cycle (3 seconds each direction)
        phase = (current_time % cycle_time) / cycle_time

        if phase < 0.5:
            # First half: move LEFT (positive X according to our documentation)
            movement = Vector3(0.2, 0, 0)  # Move left at 0.2 m/s
            direction = "LEFT (positive X)"
        else:
            # Second half: move RIGHT (negative X according to our documentation)
            movement = Vector3(-0.2, 0, 0)  # Move right at 0.2 m/s
            direction = "RIGHT (negative X)"

        print("=== LEFT-RIGHT MOVEMENT TEST ===")
        print(f"Phase: {phase:.2f}, Direction: {direction}")
        print(f"Sending movement command: {movement}")
        print(f"Expected: Robot should move {direction.split()[0]} relative to its body")
        print("===================================")
        return movement

    def _calculate_rotation_to_target(self, direction_to_goal: Vector) -> Vector:
        """Calculate the rotation needed for the robot to face the target.

        Args:
            direction_to_goal: Vector pointing from robot position to goal in global coordinates

        Returns:
            Vector with (x=0, y=0, z=angular_velocity) for rotation only
        """
        # Calculate the desired yaw angle to face the target
        desired_yaw = math.atan2(direction_to_goal.y, direction_to_goal.x)

        # Get current robot yaw
        current_yaw = self.latest_odom.orientation.z

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
        max_angular_speed = 0.15  # rad/s
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

    def _debug_direction(self, name: str, direction: Vector) -> Vector:
        """Debug helper to log direction information"""
        robot_pos = self.latest_odom
        print(
            f"DEBUG {name}: direction={direction}, robot_pos={robot_pos.position.to_2d()}, robot_yaw={math.degrees(robot_pos.rot.z):.1f}°, goal={self.goal}"
        )
        return direction

    def _debug_robot_command(self, robot_cmd: Vector) -> Vector:
        """Debug helper to log robot command information"""
        print(
            f"DEBUG robot_command: x={robot_cmd.x:.3f}, y={robot_cmd.y:.3f} (forward/backward, left/right)"
        )
        return robot_cmd
