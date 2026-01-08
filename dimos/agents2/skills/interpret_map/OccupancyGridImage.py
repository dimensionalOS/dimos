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

from __future__ import annotations

import time
from typing import TYPE_CHECKING, BinaryIO

import cv2
import numpy as np

from dimos.msgs.geometry_msgs import Pose, Quaternion, Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import CostValues, OccupancyGrid
from dimos.msgs.sensor_msgs import Image
from dimos.msgs.sensor_msgs.image_impls.AbstractImage import (
    ImageFormat,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

import numpy as np

if TYPE_CHECKING:
    from numpy.typing import NDArray


# Encode occupancy grid as image, with helper methods
class OccupancyGridImage:
    def __init__(
        self,
        image: Image,
        occupancy_grid: OccupancyGrid,
        size: tuple[int, int] | None = None,
        robot_pose: Pose | None = None,
        flip_vertical: bool = True,
    ):
        self.image = image
        self.occupancy_grid = occupancy_grid

        self.size = size
        if self.size is None:
            self.size = self._get_encoded_image_size(image.data)

        self.flip_vertical = flip_vertical
        self.robot_pose = robot_pose
        self.grid_to_pix_matrix: NDArray[np.float64] | None = None
        self.rotated_image_size: tuple[int, int] | None = None
        self.robot_pixel_coord = self._get_robot_pixel(image.data, color=[0, 255, 0])

    @classmethod
    def from_occupancygrid(
        cls,
        occupancy_grid: OccupancyGrid,
        size: tuple[int, int] | None = None,
        flip_vertical: bool = True,
        robot_pose: Pose | None = None,
    ) -> OccupancyGridImage:
        """
        Create an OccupancyGridImage from OccupancyGrid

        args:
            occupancy_grid: OccupancyGrid message
            size: Size of the image
            flip_vertical: Flip image to account for orientation during grid building
            robot_pose: Robot pose in world frame to be added on the image

        returns:
            OccupancyGridImage with occupancy grid encoded as image
        """
        # convert to RGB image:
        # - unknown as gray
        # - free space as white
        # - obstacles as black
        grid = occupancy_grid.grid
        image_arr = np.zeros((*grid.shape, 3), dtype=np.uint8)

        unknown_mask = grid == CostValues.UNKNOWN
        # unknown as gray
        image_arr[unknown_mask] = [127, 127, 127]

        known_mask = grid != CostValues.UNKNOWN
        if np.any(known_mask):
            free_mask = grid == 0

            # free space as white
            image_arr[free_mask] = [255, 255, 255]
            obstacle_mask = (grid > 0) & (grid <= 100)
            # obstaceles as black
            if np.any(obstacle_mask):
                image_arr[obstacle_mask] = [0, 0, 0]

        # overlay robot pose and rotate image keeping robot pointing upwards
        if robot_pose:
            image_arr = cls._overlay_robot_pose(image_arr, occupancy_grid, robot_pose)

            # rotate image to align robot heading upward
            yaw = robot_pose.orientation.euler[2]
            image_arr, rotation_matrix, rotated_size = cls._rotate_image_to_align_robot(
                image_arr, yaw
            )

        # flip vertically for correct orientation
        if flip_vertical:
            image_arr = cv2.flip(image_arr, 0).astype(np.uint8)

        # keep original aspect ratio if size not specified
        if size is None:
            size = cls._get_encoded_image_size(image_arr)

        # resize
        image_arr_resized = cv2.resize(image_arr, size, interpolation=cv2.INTER_NEAREST)

        image = Image(data=image_arr_resized, format=ImageFormat.RGB)

        occupancy_grid_image = cls(
            image=image,
            occupancy_grid=occupancy_grid,
            size=size,
            robot_pose=robot_pose,
            flip_vertical=flip_vertical,
        )

        # store rotation info
        if rotation_matrix is not None:
            occupancy_grid_image.grid_to_pix_matrix = rotation_matrix
            occupancy_grid_image.rotated_image_size = rotated_size

        return occupancy_grid_image

    @staticmethod
    def _get_encoded_image_size(image_arr: NDArray[np.uint8]) -> tuple[int, int]:
        # keep max dimension 1024 for encoding
        MAX_IMAGE_DIMENSION = 1024
        aspect_ratio = image_arr.shape[1] / image_arr.shape[0]
        if aspect_ratio >= 1.0:
            width = MAX_IMAGE_DIMENSION
            height = int(MAX_IMAGE_DIMENSION / aspect_ratio)
        else:
            height = MAX_IMAGE_DIMENSION
            width = int(MAX_IMAGE_DIMENSION * aspect_ratio)
        return (width, height)

    @staticmethod
    def _overlay_robot_pose(
        image_arr: NDArray[np.uint8], occupancy_grid: OccupancyGrid, robot_pose: Pose
    ) -> NDArray[np.uint8]:
        """Augment the occupancy grid image with the robot's pose.

        args:
            image_arr: Numpy array representing the occupancy grid image
            occupancy_grid: OccupancyGrid instance used to generate this image
            robot_pose: Robot pose in world frame as Pose
        returns:
            image array with robot pose overlayed
        """

        # robot position
        robot_grid_pos = occupancy_grid.world_to_grid(robot_pose.position)
        rgx = round(robot_grid_pos.x)
        rgy = round(robot_grid_pos.y)

        # check bounds
        if not (0 <= rgx < image_arr.shape[1] and 0 <= rgy < image_arr.shape[0]):
            logger.warning(f"Robot position ({rgx}, {rgy}) is outside image bounds")
            return image_arr

        # draw on image
        height, width = image_arr.shape[:2]
        min_dimension = min(height, width)

        robot_radius = max(3, int(min_dimension * 0.015))  # At least 3 pixels
        max(10, int(min_dimension * 0.035))

        max(1, int(min_dimension * 0.005))

        # robot marker
        cv2.circle(image_arr, (rgx, rgy), robot_radius, (0, 255, 0), -1)

        return image_arr

    @staticmethod
    def _rotate_image_to_align_robot(
        image_arr: NDArray[np.uint8], yaw: float
    ) -> tuple[NDArray[np.uint8], NDArray[np.float64], tuple[int, int]]:
        """Rotate image to align robot heading upward.

        args:
            image_arr: Image array to rotate
            yaw: Robot yaw angle in radians
        returns:
            tuple of (rotated image, rotation matrix, rotated image size)
        """
        height, width = image_arr.shape[:2]

        # new dimensions to fit rotated image
        abs_yaw = abs(yaw)
        new_height = int(height * abs(np.cos(abs_yaw)) + width * abs(np.sin(abs_yaw)))
        new_width = int(height * abs(np.sin(abs_yaw)) + width * abs(np.cos(abs_yaw)))

        # rotation matrix
        rotation_matrix = cv2.getRotationMatrix2D(
            center=(width / 2, height / 2), angle=-np.rad2deg(yaw), scale=1
        )

        # adjust new canvas size
        rotation_matrix[0, 2] += (new_width - width) / 2
        rotation_matrix[1, 2] += (new_height - height) / 2

        # Apply rotation
        rotated_image = cv2.warpAffine(
            image_arr, rotation_matrix, (new_width, new_height), borderValue=(127, 127, 127)
        )

        return (
            rotated_image.astype(np.uint8),
            rotation_matrix.astype(np.float64),
            (new_width, new_height),
        )

    @staticmethod
    def _get_robot_pixel(image_arr: NDArray[np.uint8], color: list[int]) -> tuple[int, int]:
        """
        Find the pixel coordinates of the robot in the image.

        args:
            image_arr: RGB image array of shape (height, width, 3)
            color: RGB color values [R, G, B] that denote the robot

        returns:
            Tuple of (row, col) pixel coordinates where the robot is found
        """
        # create a mask where all three color channels match
        mask = np.all(image_arr == color, axis=-1)
        rows, cols = np.where(mask)

        # return center pixel
        if len(rows) > 0:
            return (int(np.mean(cols)), int(np.mean(rows)))  # x, y

        raise ValueError(f"Robot with color {color} not found in image")

    def is_free_space(
        self,
        pixel_x: int,
        pixel_y: int,
        size: tuple[int, int] = (1024, 1024),
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

        # check bounds
        if not (
            0 <= grid_x < self.occupancy_grid.width and 0 <= grid_y < self.occupancy_grid.height
        ):
            return False

        if self.occupancy_grid.grid[grid_y, grid_x] == CostValues.FREE:
            return True
        else:
            return False

    def get_closest_free_point(
        self,
        pixel_x: int,
        pixel_y: int,
        size: tuple[int, int] = (1024, 1024),
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
        size: tuple[int, int] = (1024, 1024),
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
        size: tuple[int, int],
        flip_vertical: bool | None = None,
    ) -> tuple[int, int]:
        """Convert pixel coordinates in the occupancy grid image to grid coordinates.

        args:
            pixel_x: X coordinate in pixels (image space)
            pixel_y: Y coordinate in pixels (image space)
        returns:
            (x, y)
        """
        pix_to_grid_matrix = cv2.invertAffineTransform(self.grid_to_pix_matrix)  # type: ignore[arg-type]

        size = size or self.size
        flip_vertical = flip_vertical if flip_vertical is not None else self.flip_vertical

        # account for image resize before sending to agent
        scaled_x = round((pixel_x / size[0]) * self.rotated_image_size[0])  # type: ignore[index]
        scaled_y = round((pixel_y / size[1]) * self.rotated_image_size[1])  # type: ignore[index]

        # unrotate
        point = np.array([scaled_x, scaled_y, 1])
        unrotated = pix_to_grid_matrix @ point

        if flip_vertical:
            unrotated[1] = self.occupancy_grid.height - unrotated[1]

        grid_x = round(unrotated[0])
        grid_y = round(unrotated[1])

        return (grid_x, grid_y)

    def pixel_to_world(
        self,
        pixel_x: int,
        pixel_y: int,
        size: tuple[int, int] = (1024, 1024),
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

        grid_x, grid_y = self.pixel_to_grid(pixel_x, pixel_y, size, flip_vertical)

        return self.occupancy_grid.grid_to_world(Vector3(grid_x, grid_y, 0.0))
