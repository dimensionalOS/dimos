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

import threading
from abc import abstractmethod
from dataclasses import dataclass
from typing import Callable, Optional

from dimos.core import In, Module, Out, rpc
from dimos.msgs.geometry_msgs import Pose, PoseLike, PoseStamped, Vector3, VectorLike, to_pose
from dimos.msgs.nav_msgs import OccupancyGrid, Path
from dimos.robot.global_planner.algo import astar
from dimos.utils.logging_config import setup_logger
from dimos.web.websocket_vis.helpers import Visualizable

logger = setup_logger("dimos.robot.unitree.global_planner")


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
class Planner(Visualizable, Module):
    target: In[PoseStamped] = None
    path: Out[Path] = None

    def __init__(self):
        Module.__init__(self)
        Visualizable.__init__(self)

    @rpc
    def set_goal(
        self,
        goal: VectorLike,
        goal_theta: Optional[float] = None,
        stop_event: Optional[threading.Event] = None,
    ):
        path = self.plan(goal)
        if not path:
            logger.warning("No path found to the goal.")
            return False

        print("pathing success", path)


class AstarPlanner(Planner):
    target: In[Vector3] = None
    path: Out[Path] = None

    get_costmap: Callable[[], OccupancyGrid]
    get_robot_pos: Callable[[], Vector3]
    set_local_nav: Callable[[Path, Optional[threading.Event], Optional[float]], bool] = None

    conservativism: int = 8

    def __init__(
        self,
        get_costmap: Callable[[], OccupancyGrid],
        get_robot_pos: Callable[[], Vector3],
        set_local_nav: Callable[[Path, Optional[threading.Event], Optional[float]], bool] = None,
    ):
        super().__init__()
        self.get_costmap = get_costmap
        self.get_robot_pos = get_robot_pos
        self.set_local_nav = set_local_nav

    @rpc
    def start(self):
        self.target.subscribe(self.plan)

    def plan(self, goallike: PoseLike) -> Path:
        goal = to_pose(goallike)
        logger.info(f"planning path to goal {goal}")
        pos = self.get_robot_pos()
        costmap = self.get_costmap().gradient()

        self.vis("target", goal)

        path = astar(costmap, goal.position, pos)

        if path:
            path = resample_path(path, 0.1)
            self.path.publish(path)
            if hasattr(self, "set_local_nav") and self.set_local_nav:
                self.set_local_nav(path)
                logger.warning(f"Path found: {path}")
            return path
        logger.warning("No path found to the goal.")
