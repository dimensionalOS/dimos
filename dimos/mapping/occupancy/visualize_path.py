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

import cv2
from dimos_generated.nav_msgs.msg import OccupancyGrid, Path
from dimos_generated.sensor_msgs.msg import Image
import numpy as np

from dimos.mapping.occupancy.visualizations import visualize_occupancy_grid
from dimos.msgs.geometry import pose_matrix
from dimos.msgs.image import image_from_array, image_view
from dimos.msgs.occupancy import occupancy_extent


def visualize_path(
    occupancy_grid: OccupancyGrid,
    path: Path,
    robot_width: float,
    robot_length: float,
    thickness: int = 1,
    scale: int = 8,
) -> Image:
    image = visualize_occupancy_grid(occupancy_grid, "rainbow")
    bgr = image_view(image)

    bgr = cv2.resize(
        bgr,
        (bgr.shape[1] * scale, bgr.shape[0] * scale),
        interpolation=cv2.INTER_NEAREST,
    )

    # Convert robot dimensions from meters to grid cells, then to scaled pixels
    occupancy_extent(occupancy_grid)
    resolution = occupancy_grid.info.resolution
    world_to_grid = np.linalg.inv(pose_matrix(occupancy_grid.info.origin))
    robot_width_px = int((robot_width / resolution) * scale)
    robot_length_px = int((robot_length / resolution) * scale)

    # Draw robot rectangle at each path point
    for pose in path.poses:
        # Convert world coordinates to grid coordinates
        local = world_to_grid @ pose_matrix(pose.pose)
        cx = int(local[0, 3] / resolution * scale)
        cy = int(local[1, 3] / resolution * scale)

        # Get heading relative to the grid's origin orientation.
        yaw = np.arctan2(local[1, 0], local[0, 0])

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
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        rotation_matrix = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
        rotated_corners = corners @ rotation_matrix.T

        # Translate to center position
        rotated_corners[:, 0] += cx
        rotated_corners[:, 1] += cy

        # Draw the rotated rectangle
        pts = rotated_corners.astype(np.int32).reshape((-1, 1, 2))
        cv2.polylines(bgr, [pts], isClosed=True, color=(0, 0, 0), thickness=thickness)

    return image_from_array(bgr, encoding="bgr8", header=occupancy_grid.header)
