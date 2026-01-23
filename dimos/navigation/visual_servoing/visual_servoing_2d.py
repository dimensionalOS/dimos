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

from dimos.msgs.geometry_msgs import Twist, Vector3
from dimos.msgs.sensor_msgs import CameraInfo


class VisualServoing2D:
    """2D visual servoing controller for tracking objects using bounding boxes.

    Uses camera intrinsics to convert pixel coordinates to normalized camera
    coordinates and estimates distance based on known object width.
    """

    # Target distance to maintain from object (meters).
    _target_distance: float = 1.5

    # Minimum distance before backing up (meters).
    _min_distance: float = 0.8

    # Maximum forward/backward speed (m/s).
    _max_linear_speed: float = 0.5

    # Maximum turning speed (rad/s).
    _max_angular_speed: float = 0.8

    # Assumed real-world width of tracked object (meters).
    _assumed_object_width: float = 0.45

    # Proportional gain for angular velocity control.
    _angular_gain: float = 1.0

    # Proportional gain for linear velocity control.
    _linear_gain: float = 0.8

    # Speed factor when backing up (multiplied by max_linear_speed).
    _backup_speed_factor: float = 0.6

    # Multiplier for x_norm when calculating turn factor.
    _turn_factor_multiplier: float = 2.0

    # Maximum speed reduction due to turning (turn_factor ranges from 1-this to 1).
    _turn_factor_max_reduction: float = 0.7

    _rotation_requires_linear_movement: bool = False

    # Camera intrinsics for coordinate conversion.
    _camera_info: CameraInfo

    def __init__(
        self, camera_info: CameraInfo, rotation_requires_linear_movement: bool = False
    ) -> None:
        self._camera_info = camera_info
        self._rotation_requires_linear_movement = rotation_requires_linear_movement

    def compute_twist(
        self,
        bbox: tuple[float, float, float, float],
        image_width: int,
    ) -> Twist:
        """Compute twist command to servo towards the tracked object.

        Args:
            bbox: Bounding box (x1, y1, x2, y2) in pixels.
            image_width: Width of the image.

        Returns:
            Twist command for the robot.
        """
        x1, _, x2, _ = bbox
        bbox_center_x = (x1 + x2) / 2.0

        # Get normalized x coordinate using inverse K matrix
        # Positive = object is to the right of optical center
        x_norm = self._get_normalized_x(bbox_center_x)

        estimated_distance = self._estimate_distance(bbox)

        if estimated_distance is None:
            return Twist.zero()

        # Calculate distance error (positive = too far, need to move forward)
        distance_error = estimated_distance - self._target_distance

        # Compute angular velocity (turn towards object)
        # Negative because positive angular.z is counter-clockwise (left turn)
        angular_z = -x_norm * self._angular_gain
        angular_z = float(np.clip(angular_z, -self._max_angular_speed, self._max_angular_speed))

        # Compute linear velocity - ALWAYS move forward/backward based on distance.
        # Reduce forward speed when turning sharply to maintain stability.
        turn_factor = 1.0 - min(
            abs(x_norm) * self._turn_factor_multiplier, self._turn_factor_max_reduction
        )

        if estimated_distance < self._min_distance:
            # Too close, back up (don't reduce speed for backing up)
            linear_x = -self._max_linear_speed * self._backup_speed_factor
        else:
            # Move forward based on distance error with proportional gain
            linear_x = distance_error * self._linear_gain * turn_factor
            linear_x = float(np.clip(linear_x, -self._max_linear_speed, self._max_linear_speed))

        # Enforce minimum linear speed when turning
        if self._rotation_requires_linear_movement and abs(angular_z) < 0.02:
            linear_x = max(linear_x, 0.1)

        return Twist(
            linear=Vector3(linear_x, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, angular_z),
        )

    def _get_normalized_x(self, pixel_x: float) -> float:
        """Convert pixel x coordinate to normalized camera coordinate.

        Uses inverse K matrix: x_norm = (pixel_x - cx) / fx

        Args:
            pixel_x: x coordinate in pixels

        Returns:
            Normalized x coordinate (tan of angle from optical center)
        """
        fx = self._camera_info.K[0]  # focal length x
        cx = self._camera_info.K[2]  # optical center x
        return (pixel_x - cx) / fx

    def _estimate_distance(self, bbox: tuple[float, float, float, float]) -> float | None:
        """Estimate distance to object based on bounding box size and camera intrinsics.

        Uses the pinhole camera model:
            pixel_width / fx = real_width / distance
            distance = (real_width * fx) / pixel_width

        Uses bbox width instead of height because ground robot can't see full
        person height when close. Width (shoulders) is more consistently visible.

        Args:
            bbox: Bounding box (x1, y1, x2, y2) in pixels.

        Returns:
            Estimated distance in meters, or None if bbox is invalid.
        """
        bbox_width = bbox[2] - bbox[0]  # x2 - x1

        if bbox_width <= 0:
            return None

        # Pinhole camera model: distance = (real_width * fx) / pixel_width
        fx = self._camera_info.K[0]  # focal length x in pixels
        estimated_distance = (self._assumed_object_width * fx) / bbox_width

        return estimated_distance
