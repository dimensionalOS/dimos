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

from __future__ import annotations

from enum import IntEnum
import time
from typing import TYPE_CHECKING, BinaryIO

import cv2
import numpy as np
from scipy import ndimage  # type: ignore[import-untyped]

from dimos.msgs.geometry_msgs import Pose, Quaternion, Vector3, VectorLike
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.sensor_msgs.image_impls.AbstractImage import (
    ImageFormat,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

import numpy as np


# Encode occupancy grid as image, with helper methods
class OccupancyGridImage:
    def __init__(
        self,
        occupancy_grid: OccupancyGrid,
        size: tuple[int, int] | None = None,
        flip_vertical: bool = True,
        robot_pose: Pose | None = None,
    ):
        self.occupancy_grid = occupancy_grid

        self.size = size
        if self.size is None:
            self.size = (1024, int(1024 * (self.occupancy_grid.height / self.occupancy_grid.width)))

        self.flip_vertical = flip_vertical
        self.robot_pose = robot_pose

    def encode(self) -> Image:
        """Encode the occupancy grid as image."""
        return self._grid_to_image()

    def _grid_to_image(self) -> Image:
        """Convert the occupancy grid to Image."""
        # convert to RGB image:
        # - unknown as yellow
        # - free space as blue
        # - obstacles as red shades
        grid = self.occupancy_grid.grid
        image_arr = np.zeros((*grid.shape, 3), dtype=np.uint8)

        unknown_mask = grid == CostValues.UNKNOWN
        # unknown as yellow
        image_arr[unknown_mask] = [255, 255, 0]

        known_mask = grid != CostValues.UNKNOWN
        if np.any(known_mask):
            free_mask = grid == 0

            # free space as blue
            image_arr[free_mask] = [0, 0, 200]
            obstacle_mask = (grid > 0) & (grid <= 100)
            # obstaceles as red shades
            if np.any(obstacle_mask):
                # map cost values 1 - 100 from 255 to 100 (dark to bright red)
                red_intensity = (255 - (grid[obstacle_mask] * 155 // 100)).astype(np.uint8)
                image_arr[obstacle_mask] = np.stack(
                    [red_intensity, np.zeros_like(red_intensity), np.zeros_like(red_intensity)],
                    axis=1,
                )

        # add robot pose if available
        if self.robot_pose:
            image_arr = self._overlay_robot_pose(image_arr)

        # flip vertically for correct orientation
        if self.flip_vertical:
            image_arr = cv2.flip(image_arr, 0)

        # keep original aspect ratio if size not specified
        if self.size is None:
            (1024, int(1024 * (self.occupancy_grid.height / self.occupancy_grid.width)))

        # resize
        image_arr_resized = cv2.resize(image_arr, self.size, interpolation=cv2.INTER_NEAREST)

        image = Image(data=image_arr_resized, format=ImageFormat.RGB)

        return image

    def _overlay_robot_pose(self, image_arr: np.ndarray) -> np.ndarray:  # type: ignore[type-arg]
        """Augment the occupancy grid image with the robot's pose.

        args:
            image_arr: Numpy array representing the occupancy grid image
        """

        # robot position
        robot_grid_pos = self.occupancy_grid.world_to_grid(self.robot_pose.position)
        rgx = round(robot_grid_pos.x)
        rgy = round(robot_grid_pos.y)

        # yaw
        yaw = self.quat_to_yaw(self.robot_pose.orientation)
        halfarrow_length = 2  # pixels
        arrow_dx = int(halfarrow_length * np.cos(yaw))
        arrow_dy = -int(
            halfarrow_length * np.sin(yaw)
        )  # account for poisitive y down in image space

        # draw on image
        height, width = image_arr.shape[:2]
        min_dimension = min(height, width)

        robot_radius = max(3, int(min_dimension * 0.015))  # At least 2 pixels
        arrow_length = max(7, int(min_dimension * 0.035))

        line_thickness = max(1, int(min_dimension * 0.005))

        # robot position marker
        cv2.circle(image_arr, (rgx, rgy), robot_radius, (0, 255, 0), -1)

        # orientation arrow
        arrow_dx = int(arrow_length * np.cos(yaw))
        arrow_dy = -int(arrow_length * np.sin(yaw))  # account for y + down in image space

        cv2.arrowedLine(
            image_arr,
            (rgx, rgy),
            (rgx + arrow_dx, rgy + arrow_dy),
            (0, 255, 0),
            line_thickness,
            tipLength=0.4,
        )

        return image_arr

    def quat_to_yaw(self, q: Quaternion) -> np.ndarray:
        """Convert quaternion to yaw angle in radians."""
        return np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def is_free_space(
        self,
        pixel_x: int,
        pixel_y: int,
        size: tuple[int, int] | None = None,
        flip_vertical: bool | None = None,
    ) -> bool:
        """Get the type of point (free, occupied, unknown) at given pixel coordinates in the occupancy grid image.
        args:
            pixel_x: X coordinate in pixels (image space)
            pixel_y: Y coordinate in pixels (image space)
        returns:
            cost value at the specified pixel
        """

        size = size or self.size
        flip_vertical = flip_vertical if flip_vertical is not None else self.flip_vertical

        grid_x, grid_y = self.pixel_to_grid(
            pixel_x, pixel_y, size=size, flip_vertical=flip_vertical
        )

        if self.occupancy_grid.grid[grid_y, grid_x] == CostValues.FREE:
            return True
        else:
            return False

    def get_closest_free_point(
        self,
        pixel_x: int,
        pixel_y: int,
        size: tuple[int, int] | None = None,
        flip_vertical: bool | None = None,
        max_search_radius: int = 10,
    ) -> tuple[int, int] | None:
        """Find the closest free point in the occupancy grid to the given grid coordinates.

        args:
            pixel_x: X coordinate in pixels (image space)
            pixel_y: Y coordinate in pixels (image space)
            max_search_radius: Maximum search radius in grid cells
        returns:
            (x, y) grid coordinates of the closest free point, or None if not found
        """
        size = size or self.size
        flip_vertical = flip_vertical if flip_vertical is not None else self.flip_vertical

        grid_x, grid_y = self.pixel_to_grid(
            pixel_x, pixel_y, size=size, flip_vertical=flip_vertical
        )

        y_min = max(0, grid_y - max_search_radius)
        y_max = min(self.occupancy_grid.height - 1, grid_y + max_search_radius)
        x_min = max(0, grid_x - max_search_radius)
        x_max = min(self.occupancy_grid.width - 1, grid_x + max_search_radius)

        search_area = self.occupancy_grid.grid[y_min : y_max + 1, x_min : x_max + 1]

        # TODO: buffer free space?
        free_positions = np.argwhere(search_area == CostValues.FREE)
        if free_positions.size > 0:
            distances = np.linalg.norm(
                free_positions - np.array([grid_y - y_min, grid_x - x_min]), axis=1
            )
            closest_idx = np.argmin(distances)
            closest_free_pos = free_positions[closest_idx]
            closest_x = closest_free_pos[1] + x_min
            closest_y = closest_free_pos[0] + y_min
            return self.grid_to_pixel(closest_x, closest_y, size=size, flip_vertical=flip_vertical)
        return None

    def grid_to_pixel(
        self,
        grid_x: int,
        grid_y: int,
        size: tuple[int, int] | None = None,
        flip_vertical: bool | None = None,
    ) -> tuple[int, int]:
        """Convert grid coordinates to pixel coordinates in the occupancy grid image.

        args:
            grid_x: X coordinate in grid
            grid_y: Y coordinate in grid
        returns:
            (pixel_x, pixel_y)
        """
        size = size or self.size
        flip_vertical = flip_vertical if flip_vertical is not None else self.flip_vertical

        pixel_x = round((grid_x / self.occupancy_grid.width) * size[0])
        pixel_y = round((grid_y / self.occupancy_grid.height) * size[1])

        if flip_vertical:
            pixel_y = size[1] - pixel_y

        return (pixel_x, pixel_y)

    def pixel_to_grid(
        self,
        pixel_x: int,
        pixel_y: int,
        size: tuple[int, int] | None = None,
        flip_vertical: bool | None = None,
    ) -> tuple[int, int]:
        """Convert pixel coordinates in the occupancy grid image to grid coordinates.

        args:
            pixel_x: X coordinate in pixels (image space)
            pixel_y: Y coordinate in pixels (image space)
        returns:
            (x, y)
        """
        size = size or self.size
        flip_vertical = flip_vertical if flip_vertical is not None else self.flip_vertical

        if flip_vertical:
            pixel_y = size[1] - pixel_y

        grid_x = round((pixel_x / size[0]) * self.occupancy_grid.width)
        grid_y = round((pixel_y / size[1]) * self.occupancy_grid.height)

        return (grid_x, grid_y)

    def pixel_to_world(
        self,
        pixel_x: int,
        pixel_y: int,
        size: tuple[int, int] | None = None,
        flip_vertical: bool | None = None,
    ) -> Vector3:
        """Convert pixel coordinates in the occupancy grid image to world coordinates.

        args:
            pixel_x: X coordinate in pixels (image space)
            pixel_y: Y coordinate in pixels (image space)
        returns:
            World position as Vector3
        """
        size = size or self.size
        flip_vertical = flip_vertical if flip_vertical is not None else self.flip_vertical

        if flip_vertical:
            pixel_y = size[1] - pixel_y

        grid_x = (pixel_x / size[0]) * self.occupancy_grid.width
        grid_y = (pixel_y / size[1]) * self.occupancy_grid.height

        return self.occupancy_grid.grid_to_world(Vector3(grid_x, grid_y, 0.0))
