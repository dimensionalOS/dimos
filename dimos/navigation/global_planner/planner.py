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

from abc import abstractmethod
from dataclasses import dataclass
from typing import Optional

from dimos.core import In, Module, Out, rpc
from dimos.msgs.geometry_msgs import Pose, PoseStamped
from dimos.msgs.nav_msgs import OccupancyGrid, Path
from dimos.navigation.global_planner.algo import astar
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import euler_to_quaternion

logger = setup_logger("dimos.robot.unitree.global_planner")

import math
from dimos.msgs.geometry_msgs import Quaternion, Vector3


def add_orientations_to_path(path: Path, goal_orientation: Quaternion = None) -> Path:
    """Add orientations to path poses based on direction of movement.

    Args:
        path: Path with poses to add orientations to
        goal_orientation: Desired orientation for the final pose

    Returns:
        Path with orientations added to all poses
    """
    if not path.poses or len(path.poses) < 2:
        return path

    # Calculate orientations for all poses except the last one
    for i in range(len(path.poses) - 1):
        current_pose = path.poses[i]
        next_pose = path.poses[i + 1]

        # Calculate direction to next point
        dx = next_pose.position.x - current_pose.position.x
        dy = next_pose.position.y - current_pose.position.y

        # Calculate yaw angle
        yaw = math.atan2(dy, dx)

        # Convert to quaternion (roll=0, pitch=0, yaw)
        orientation = euler_to_quaternion(Vector3(0, 0, yaw))
        current_pose.orientation = orientation

    # Set last pose orientation
    identity_quat = Quaternion(0, 0, 0, 1)
    if goal_orientation is not None and goal_orientation != identity_quat:
        # Use the provided goal orientation if it's not the identity
        path.poses[-1].orientation = goal_orientation
    elif len(path.poses) > 1:
        # Use the previous pose's orientation
        path.poses[-1].orientation = path.poses[-2].orientation
    else:
        # Single pose with identity goal orientation
        path.poses[-1].orientation = identity_quat

    return path


def resample_path(path: Path, spacing: float) -> Path:
    """Resample a path to have approximately uniform spacing between poses.

    Args:
        path: The original Path
        spacing: Desired distance between consecutive poses

    Returns:
        A new Path with resampled poses
    """
    if len(path) < 2 or spacing <= 0:
        return path

    resampled = []
    resampled.append(path.poses[0])

    accumulated_distance = 0.0

    for i in range(1, len(path.poses)):
        current = path.poses[i]
        prev = path.poses[i - 1]

        # Calculate segment distance
        dx = current.x - prev.x
        dy = current.y - prev.y
        segment_length = (dx**2 + dy**2) ** 0.5

        if segment_length < 1e-10:
            continue

        # Direction vector
        dir_x = dx / segment_length
        dir_y = dy / segment_length

        # Add points along this segment
        while accumulated_distance + segment_length >= spacing:
            # Distance along segment for next point
            dist_along = spacing - accumulated_distance
            if dist_along < 0:
                break

            # Create new pose
            new_x = prev.x + dir_x * dist_along
            new_y = prev.y + dir_y * dist_along
            new_pose = PoseStamped(
                frame_id=path.frame_id,
                position=[new_x, new_y, 0.0],
                orientation=prev.orientation,  # Keep same orientation
            )
            resampled.append(new_pose)

            # Update for next iteration
            accumulated_distance = 0
            segment_length -= dist_along
            prev = new_pose

        accumulated_distance += segment_length

    # Add last pose if not already there
    if len(path.poses) > 1:
        last = path.poses[-1]
        if not resampled or (resampled[-1].x != last.x or resampled[-1].y != last.y):
            resampled.append(last)

    return Path(frame_id=path.frame_id, poses=resampled)


@dataclass
class Planner(Module):
    target: In[PoseStamped] = None
    path: Out[Path] = None

    def __init__(self):
        Module.__init__(self)


class AstarPlanner(Planner):
    # LCM inputs
    target: In[PoseStamped] = None
    global_costmap: In[OccupancyGrid] = None
    odom: In[PoseStamped] = None

    # LCM outputs
    path: Out[Path] = None

    def __init__(self):
        super().__init__()

        # Latest data
        self.latest_costmap: Optional[OccupancyGrid] = None
        self.latest_odom: Optional[PoseStamped] = None

    @rpc
    def start(self):
        # Subscribe to inputs
        self.target.subscribe(self._on_target)
        self.global_costmap.subscribe(self._on_costmap)
        self.odom.subscribe(self._on_odom)

        logger.info("A* planner started")

    def _on_costmap(self, msg: OccupancyGrid):
        """Handle incoming costmap messages."""
        self.latest_costmap = msg

    def _on_odom(self, msg: PoseStamped):
        """Handle incoming odometry messages."""
        self.latest_odom = msg

    def _on_target(self, msg: PoseStamped):
        """Handle incoming target messages and trigger planning."""
        if self.latest_costmap is None or self.latest_odom is None:
            logger.warning("Cannot plan: missing costmap or odometry data")
            return

        path = self.plan(msg)
        if path:
            # Add orientations to the path, using the goal's orientation for the final pose
            path = add_orientations_to_path(path, msg.orientation)
            self.path.publish(path)

    def plan(self, goal: Pose) -> Optional[Path]:
        """Plan a path from current position to goal."""
        if self.latest_costmap is None or self.latest_odom is None:
            logger.warning("Cannot plan: missing costmap or odometry data")
            return None

        logger.debug(f"Planning path to goal {goal}")

        # Get current position from odometry
        robot_pos = self.latest_odom.position
        costmap = self.latest_costmap.inflate(0.2).gradient(max_distance=1.5)

        # Run A* planning
        path = astar(costmap, goal.position, robot_pos)

        if path:
            path = resample_path(path, 0.1)
            logger.debug(f"Path found with {len(path.poses)} waypoints")
            return path

        logger.warning("No path found to the goal.")
        return None
