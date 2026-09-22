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

"""Frame-level fiducial marker detection orchestration."""

from __future__ import annotations

from typing import Any

import cv2
from dimos_generated.geometry_msgs.msg import TransformStamped, Vector3
from dimos_generated.sensor_msgs.msg import CameraInfo, Image
from dimos_generated.std_msgs.msg import Header
import numpy as np

from dimos.msgs.geometry import compose_transforms
from dimos.msgs.image import image_to_bgr
from dimos.msgs.time import to_seconds
from dimos.perception.detection.type.detection3d.marker import Detection3DMarker
from dimos.perception.fiducial.marker_pose import (
    camera_info_to_cv_matrices,
    camera_optical_frame_id,
    create_aruco_detector,
    estimate_marker_pose,
    marker_corners_to_bbox,
    marker_reprojection_error,
    rvec_tvec_to_transform,
)


def detect_markers_in_image(
    image: Image,
    *,
    camera_info: CameraInfo,
    world_T_optical: TransformStamped,
    marker_length_m: float,
    aruco_dictionary: str,
    world_frame: str = "world",
    detect_inverted: bool = False,
    detector: Any | None = None,
    camera_matrix: np.ndarray[Any, np.dtype[Any]] | None = None,
    dist_coeffs: np.ndarray[Any, np.dtype[Any]] | None = None,
) -> list[Detection3DMarker]:
    """Detect markers in one image and return rich world-frame 3D detections."""
    if marker_length_m <= 0:
        raise ValueError(f"marker_length_m must be > 0, got {marker_length_m}")
    if (
        camera_info.width
        and camera_info.height
        and (image.width != camera_info.width or image.height != camera_info.height)
    ):
        return []

    if detector is None:
        detector = create_aruco_detector(aruco_dictionary, detect_inverted=detect_inverted)
    if (camera_matrix is None) != (dist_coeffs is None):
        raise ValueError("camera_matrix and dist_coeffs must be provided together")
    if camera_matrix is None or dist_coeffs is None:
        camera_matrix, dist_coeffs = camera_info_to_cv_matrices(camera_info)

    gray = cv2.cvtColor(image_to_bgr(image), cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None or len(ids) == 0:
        return []

    optical_frame = camera_optical_frame_id(image, camera_info)
    t_world_optical = TransformStamped(
        header=Header(stamp=image.header.stamp, frame_id=world_frame),
        child_frame_id=optical_frame,
        transform=world_T_optical.transform,
    )
    marker_size = Vector3(x=marker_length_m, y=marker_length_m)
    detections: list[Detection3DMarker] = []

    for corner_set, mid_arr in zip(corners, ids, strict=True):
        mid = int(mid_arr[0])
        pose = estimate_marker_pose(
            corner_set,
            marker_length_m,
            camera_matrix,
            dist_coeffs,
            distortion_model=camera_info.distortion_model,
        )
        if pose is None:
            continue

        rvec, tvec = pose
        t_optical_marker = rvec_tvec_to_transform(
            rvec,
            tvec,
            header=Header(stamp=image.header.stamp, frame_id=optical_frame),
            child_frame_id=f"marker_{mid}",
        )
        t_world_marker = compose_transforms(t_world_optical, t_optical_marker)

        corners_2d = corner_set.reshape(4, 2).astype(np.float32)
        bbox = marker_corners_to_bbox(corners_2d)
        reprojection_error = marker_reprojection_error(
            corners_2d,
            marker_length_m,
            camera_matrix,
            dist_coeffs,
            rvec,
            tvec,
            distortion_model=camera_info.distortion_model,
        )

        detections.append(
            Detection3DMarker(
                bbox=bbox,
                track_id=-1,
                class_id=mid,
                confidence=1.0,
                name="",
                ts=to_seconds(image.header.stamp),
                image=image,
                center=t_world_marker.transform.translation,
                size=marker_size,
                transform=t_world_optical,
                frame_id=world_frame,
                orientation=t_world_marker.transform.rotation,
                marker_id=mid,
                corners_px=corners_2d,
                dictionary=aruco_dictionary,
                reprojection_error=reprojection_error,
            )
        )

    return detections
