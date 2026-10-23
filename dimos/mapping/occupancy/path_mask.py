# Copyright 2025-2026 Dimensional Inc.
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

import numpy as np
from numpy.typing import NDArray

from dimos.mapping.occupancy.operations import _grid_coordinates, _grid_values
from dimos.message_utils.occupancy import CostValues
from dimos.messages.geometry_msgs.msg import PoseStamped as CdrPoseStamped
from dimos.messages.nav_msgs.msg import OccupancyGrid as CdrOccupancyGrid, Path as CdrPath
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path


def make_path_mask(
    occupancy_grid: OccupancyGrid | CdrOccupancyGrid,
    path: Path | CdrPath,
    robot_width: float,
    pose_index: int = 0,
    max_length: float = float("inf"),
) -> NDArray[np.bool_]:
    """Generate a numpy mask of path cells the robot will travel through.

    Creates a boolean mask where True indicates cells that the robot will
    occupy while following the path, accounting for the robot's width.

    Args:
        occupancy_grid: The occupancy grid providing dimensions and resolution.
        path: The path containing poses the robot will follow.
        robot_width: The width of the robot in meters.
        pose_index: The index in path.poses to start drawing from. Defaults to 0.
        max_length: Maximum cumulative length to draw. Defaults to infinity.

    Returns:
        A 2D boolean numpy array (height x width) where True indicates
        cells the robot will pass through.
    """
    import cv2

    mask = np.zeros((occupancy_grid.info.height, occupancy_grid.info.width), dtype=np.uint8)

    line_width_pixels = max(1, int(robot_width / occupancy_grid.info.resolution))

    poses = path.poses
    if len(poses) < pose_index + 2:
        return mask.astype(np.bool_)

    # Draw lines between consecutive points
    cumulative_length = 0.0
    for i in range(pose_index, len(poses) - 1):
        first, second = poses[i], poses[i + 1]
        pos1 = first.pose.position if isinstance(first, CdrPoseStamped) else first.position
        pos2 = second.pose.position if isinstance(second, CdrPoseStamped) else second.position

        segment_length = np.sqrt(
            (pos2.x - pos1.x) ** 2 + (pos2.y - pos1.y) ** 2 + (pos2.z - pos1.z) ** 2
        )

        if cumulative_length + segment_length > max_length:
            break

        cumulative_length += segment_length

        grid_pt1 = _grid_coordinates(occupancy_grid, pos1)
        grid_pt2 = _grid_coordinates(occupancy_grid, pos2)

        pt1 = (round(grid_pt1[0]), round(grid_pt1[1]))
        pt2 = (round(grid_pt2[0]), round(grid_pt2[1]))

        cv2.line(mask, pt1, pt2, (255.0,), thickness=line_width_pixels)

    bool_mask = mask.astype(np.bool_)

    total_points = np.sum(bool_mask)

    if total_points == 0:
        return bool_mask

    occupied_mask = _grid_values(occupancy_grid) >= CostValues.OCCUPIED
    occupied_in_path = bool_mask & occupied_mask
    occupied_count = np.sum(occupied_in_path)

    if occupied_count / total_points > 0.05:
        raise ValueError(
            f"More than 5% of path points are occupied: "
            f"{occupied_count}/{total_points} ({100 * occupied_count / total_points:.1f}%)"
        )

    # Some of the points on the edge of the path may be occupied due to
    # rounding. Remove them.
    bool_mask = bool_mask & ~occupied_mask

    return bool_mask
