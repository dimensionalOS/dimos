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

import numpy as np
from typing import Dict, Any, Optional, List

from dimos.msgs.geometry_msgs import Pose, Vector3, Quaternion


def parse_zed_pose(zed_pose_data: Dict[str, Any]) -> Optional[Pose]:
    """
    Parse ZED pose data dictionary into a Pose object.

    Args:
        zed_pose_data: Dictionary from ZEDCamera.get_pose() containing:
            - position: [x, y, z] in meters
            - rotation: [x, y, z, w] quaternion
            - euler_angles: [roll, pitch, yaw] in radians
            - valid: Whether pose is valid

    Returns:
        Pose object with position and orientation, or None if invalid
    """
    if not zed_pose_data or not zed_pose_data.get("valid", False):
        return None

    # Extract position
    position = zed_pose_data.get("position", [0, 0, 0])
    pos_vector = Vector3(position[0], position[1], position[2])

    quat = zed_pose_data["rotation"]
    orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
    return Pose(pos_vector, orientation)


def estimate_object_depth(
    depth_image: np.ndarray, segmentation_mask: Optional[np.ndarray], bbox: List[float]
) -> float:
    """
    Estimate object depth dimension using segmentation mask and depth data.
    Optimized for real-time performance.

    Args:
        depth_image: Depth image in meters
        segmentation_mask: Binary segmentation mask for the object
        bbox: Bounding box [x1, y1, x2, y2]

    Returns:
        Estimated object depth in meters
    """
    x1, y1, x2, y2 = int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])

    # Extract depth ROI once
    roi_depth = depth_image[y1:y2, x1:x2]

    if segmentation_mask is not None and segmentation_mask.size > 0:
        # Extract mask ROI efficiently
        mask_roi = (
            segmentation_mask[y1:y2, x1:x2]
            if segmentation_mask.shape != roi_depth.shape
            else segmentation_mask
        )

        # Fast mask application using boolean indexing
        valid_mask = mask_roi > 0
        if np.sum(valid_mask) > 10:  # Early exit if not enough points
            masked_depths = roi_depth[valid_mask]

            # Fast percentile calculation using numpy's optimized functions
            depth_90 = np.percentile(masked_depths, 90)
            depth_10 = np.percentile(masked_depths, 10)
            depth_range = depth_90 - depth_10

            # Clamp to reasonable bounds with single operation
            return np.clip(depth_range, 0.02, 0.5)

    # Fast fallback using area calculation
    bbox_area = (x2 - x1) * (y2 - y1)

    # Vectorized area-based estimation
    if bbox_area > 10000:
        return 0.15
    elif bbox_area > 5000:
        return 0.10
    else:
        return 0.05
