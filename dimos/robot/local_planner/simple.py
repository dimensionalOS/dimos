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
import traceback
from typing import Callable, Optional

import reactivex as rx
from plum import dispatch
from reactivex import operators as ops

from dimos.core import In, Module, Out, rpc

# from dimos.robot.local_planner.local_planner import LocalPlanner
from dimos.msgs.geometry_msgs import (
    Pose,
    PoseLike,
    PoseStamped,
    Transform,
    Twist,
    Vector3,
    VectorLike,
    to_pose,
)
from dimos.msgs.tf2_msgs import TFMessage
from dimos.types.costmap import Costmap
from dimos.types.path import Path
from dimos.utils.logging_config import setup_logger
from dimos.utils.threadpool import get_scheduler

logger = setup_logger("dimos.robot.unitree.local_planner")


def vector_to_twist(vector: Vector3) -> Twist:
    """Convert a Vector3 to a Twist message."""
    twist = Twist()
    twist.linear.x = vector.x
    twist.linear.y = vector.y
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = vector.z
    return twist


class SimplePlanner(Module):
    path: In[Path] = None
    odom: In[PoseStamped] = None
    movecmd: Out[Twist] = None

    tf: Out[Transform] = None

    get_costmap: Callable[[], Costmap]

    latest_odom: Optional[PoseStamped] = None

    goal: Optional[PoseStamped] = None
    speed: float = 0.3

    def __init__(
        self,
        get_costmap: Callable[[], Costmap],
    ):
        Module.__init__(self)
        self.get_costmap = get_costmap

    def _odom_to_tf(self, odom: PoseStamped) -> Transform:
        """Convert Odometry to Transform."""
        return Transform(
            translation=odom.position,
            rotation=odom.orientation,
            frame_id="world",
            child_frame_id="base_link",
            ts=odom.ts,
        )

    def transform_to_robot_frame(
        self, global_target: Vector3, global_robot_position: PoseStamped
    ) -> Vector3:
        tf1 = self._odom_to_tf(global_robot_position)
        tf2 = global_robot_position.find_transform(global_target)
        tf2.child_frame_id = "target"
        tf2.frame_id = "base_link"
        self.tf.publish(TFMessage(tf1, tf2))
        return tf2.translation

    def get_move_stream(self, frequency: float = 20.0) -> rx.Observable:
        return rx.interval(1.0 / frequency, scheduler=get_scheduler()).pipe(
            # do we have a goal?
            ops.filter(lambda _: self.goal is not None),
            # do we have odometry data?
            ops.filter(lambda _: self.latest_odom is not None),
            # transform goal to robot frame with error handling
            ops.map(lambda _: self._safe_transform_goal()),
            # filter out None results (errors)
            ops.filter(lambda result: result is not None),
            ops.map(self._calculate_rotation_to_target),
            # filter out None results from calc_move
            ops.filter(lambda result: result is not None),
        )

    def _safe_transform_goal(self) -> Optional[Vector3]:
        """Safely transform goal to robot frame with error handling."""
        try:
            return self.transform_to_robot_frame(self.goal, self.latest_odom)
        except Exception as e:
            logger.error(f"Error transforming goal to robot frame: {e}")
            print(traceback.format_exc())
            return None

    @rpc
    def start(self):
        print(self.path.connection, self.path.transport)
        self.path.subscribe(self.set_goal)

        def setodom(odom: PoseStamped):
            self.latest_odom = odom

        def pubmove(move: Vector3):
            print("PUBMOVE", move, "\n\n")
            # print(self.movecmd)
            try:
                self.movecmd.publish(move)
            except Exception as e:
                print(traceback.format_exc())
                print(e)

        self.odom.subscribe(setodom)
        self.get_move_stream(frequency=20.0).subscribe(pubmove)

    @dispatch
    def set_goal(self, goal: Path, stop_event=None, goal_theta=None) -> bool:
        self.goal = to_pose(goal.last())
        logger.info(f"Setting goal: {self.goal}")
        return True

    @dispatch
    def set_goal(self, goal: VectorLike, stop_event=None, goal_theta=None) -> bool:
        self.goal = to_pose(goal.last())
        logger.info(f"Setting goal: {self.goal}")
        return True

    def calc_move(self, direction: Vector3) -> Optional[Vector3]:
        """Calculate the movement vector based on the direction to the goal.

        Args:
            direction: Direction vector towards the goal

        Returns:
            Movement vector scaled by speed, or None if error occurs
        """
        try:
            # Normalize the direction vector and scale by speed
            normalized_direction = direction.normalize()
            move_vector = normalized_direction * self.speed
            print("CALC MOVE", direction, normalized_direction, move_vector)
            return move_vector
        except Exception as e:
            logger.error(f"Error calculating move vector: {e}")
            return None

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

    def _test_translational_movement(self) -> Vector3:
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

    def _calculate_rotation_to_target(self, direction_to_goal: Vector3) -> Vector3:
        """Calculate the rotation needed for the robot to face the target.

        Args:
            direction_to_goal: Vector pointing from robot position to goal in global coordinates

        Returns:
            Vector with (x=0, y=0, z=angular_velocity) for rotation only
        """
        if self.latest_odom is None:
            logger.warning("No odometry data available for rotation calculation")
            return Vector3(0, 0, 0)

        # Calculate the desired yaw angle to face the target
        desired_yaw = math.atan2(direction_to_goal.y, direction_to_goal.x)

        # Get current robot yaw
        current_yaw = self.latest_odom.yaw

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
        return Vector3(0, 0, -angular_velocity)

    def _debug_direction(self, name: str, direction: Vector3) -> Vector3:
        """Debug helper to log direction information"""
        robot_pos = self.latest_odom
        if robot_pos is None:
            print(f"DEBUG {name}: direction={direction}, robot_pos=None, goal={self.goal}")
            return direction

        print(
            f"DEBUG {name}: direction={direction}, robot_pos={robot_pos.position.to_2d()}, robot_yaw={math.degrees(robot_pos.orientation.z):.1f}°, goal={self.goal}"
        )
        return direction

    def _debug_robot_command(self, robot_cmd: Vector3) -> Vector3:
        """Debug helper to log robot command information"""
        print(
            f"DEBUG robot_command: x={robot_cmd.x:.3f}, y={robot_cmd.y:.3f} (forward/backward, left/right)"
        )
        return robot_cmd
