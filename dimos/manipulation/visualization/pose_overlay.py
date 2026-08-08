# Copyright 2026 Dimensional Inc.
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

"""Project 3D poses onto camera images."""

import cv2

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image


def draw_pose_axes(
    image: Image,
    pose: PoseStamped,
    camera_from_pose_frame: Transform,
    camera_info: CameraInfo,
    axis_length_m: float = 0.05,
) -> Image | None:
    """Draw a pose midpoint and projected RGB coordinate axes onto an image."""
    center_px = _project_point(
        _transform_point(camera_from_pose_frame, pose.position),
        camera_info,
    )
    if center_px is None:
        return None

    overlay = image.to_opencv().copy()
    axes = (
        (Vector3(1.0, 0.0, 0.0), (0, 0, 255), "X"),
        (Vector3(0.0, 1.0, 0.0), (0, 255, 0), "Y"),
        (Vector3(0.0, 0.0, 1.0), (255, 0, 0), "Z"),
    )
    for axis, color, label in axes:
        endpoint = pose.position + pose.orientation.rotate_vector(axis * axis_length_m)
        endpoint_px = _project_point(
            _transform_point(camera_from_pose_frame, endpoint), camera_info
        )
        if endpoint_px is None:
            continue
        cv2.arrowedLine(overlay, center_px, endpoint_px, color, 2, tipLength=0.2)
        cv2.putText(overlay, label, endpoint_px, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    cv2.circle(overlay, center_px, 4, (255, 255, 255), -1)
    return Image.from_opencv(overlay, frame_id=image.frame_id, ts=image.ts)


def _transform_point(transform: Transform, point: Vector3) -> Vector3:
    return transform.rotation.rotate_vector(point) + transform.translation


def _project_point(point: Vector3, camera_info: CameraInfo) -> tuple[int, int] | None:
    if point.z <= 0:
        return None
    fx, fy, cx, cy = camera_info.K[0], camera_info.K[4], camera_info.K[2], camera_info.K[5]
    return (round(fx * point.x / point.z + cx), round(fy * point.y / point.z + cy))
