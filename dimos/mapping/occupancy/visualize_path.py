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

from typing import overload

import numpy as np

from dimos.mapping.occupancy.operations import _grid_coordinates
from dimos.mapping.occupancy.visualizations import _map_image, visualize_occupancy_grid
from dimos.message_utils.geometry import yaw
from dimos.message_utils.image import image_view
from dimos.messages.geometry_msgs.msg import PoseStamped as CdrPoseStamped
from dimos.messages.nav_msgs.msg import OccupancyGrid as CdrOccupancyGrid, Path as CdrPath
from dimos.messages.sensor_msgs.msg import Image as CdrImage
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.msgs.sensor_msgs.Image import Image


@overload
def visualize_path(
    occupancy_grid: CdrOccupancyGrid,
    path: Path | CdrPath,
    robot_width: float,
    robot_length: float,
    thickness: int = 1,
    scale: int = 8,
) -> CdrImage: ...


@overload
def visualize_path(
    occupancy_grid: OccupancyGrid,
    path: Path | CdrPath,
    robot_width: float,
    robot_length: float,
    thickness: int = 1,
    scale: int = 8,
) -> Image: ...


def visualize_path(
    occupancy_grid: OccupancyGrid | CdrOccupancyGrid,
    path: Path | CdrPath,
    robot_width: float,
    robot_length: float,
    thickness: int = 1,
    scale: int = 8,
) -> Image | CdrImage:
    import cv2

    image = visualize_occupancy_grid(occupancy_grid, "rainbow")
    bgr = image_view(image) if isinstance(image, CdrImage) else image.data

    bgr = cv2.resize(
        bgr,
        (bgr.shape[1] * scale, bgr.shape[0] * scale),
        interpolation=cv2.INTER_NEAREST,
    ).astype(np.uint8, copy=False)

    # Convert robot dimensions from meters to grid cells, then to scaled pixels
    resolution = occupancy_grid.info.resolution
    robot_width_px = int((robot_width / resolution) * scale)
    robot_length_px = int((robot_length / resolution) * scale)

    # Draw robot rectangle at each path point
    for pose in path.poses:
        # Convert world coordinates to grid coordinates
        position = pose.pose.position if isinstance(pose, CdrPoseStamped) else pose.position
        x, y = _grid_coordinates(occupancy_grid, position)
        cx = int(x * scale)
        cy = int(y * scale)

        # Get yaw angle from pose orientation
        angle = yaw(pose.pose.orientation) if isinstance(pose, CdrPoseStamped) else pose.yaw

        # Define rectangle corners centered at origin (length along x, width along y)
        half_length = robot_length_px / 2
        half_width = robot_width_px / 2
        corners = np.array(
            [
                [-half_length, -half_width],
                [half_length, -half_width],
                [half_length, half_width],
                [-half_length, half_width],
            ],
            dtype=np.float32,
        )

        # Rotate corners by yaw angle
        cos_yaw = np.cos(angle)
        sin_yaw = np.sin(angle)
        rotation_matrix = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
        rotated_corners = corners @ rotation_matrix.T

        # Translate to center position
        rotated_corners[:, 0] += cx
        rotated_corners[:, 1] += cy

        # Draw the rotated rectangle
        pts = rotated_corners.astype(np.int32).reshape((-1, 1, 2))
        cv2.polylines(bgr, [pts], isClosed=True, color=(0, 0, 0), thickness=thickness)

    return _map_image(occupancy_grid, bgr)
