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
from dimos_generated.sensor_msgs.msg import CameraInfo
from dimos_generated.std_msgs.msg import Header
import numpy as np

from dimos.msgs.image import image_from_array
from dimos.perception.fiducial.marker_pose import (
    camera_optical_frame_id,
    estimate_marker_pose,
    marker_corners_to_bbox,
    marker_reprojection_error,
)


def test_camera_optical_frame_id_resolution() -> None:
    info_named = CameraInfo(header=Header(frame_id="cam_info_optical"))
    info_empty = CameraInfo()
    pixels = np.zeros((480, 640, 3), dtype=np.uint8)
    img_custom = image_from_array(pixels, encoding="bgr8", header=Header(frame_id="custom_optical"))
    img_whitespace = image_from_array(
        pixels, encoding="bgr8", header=Header(frame_id="  custom_optical  ")
    )
    img_empty = image_from_array(pixels, encoding="bgr8")

    assert camera_optical_frame_id(img_custom, info_named) == "custom_optical"
    assert camera_optical_frame_id(img_whitespace, info_named) == "custom_optical"
    assert camera_optical_frame_id(img_empty, info_named) == "cam_info_optical"
    assert camera_optical_frame_id(img_empty, info_empty) == "camera_optical"


def test_estimate_marker_pose_roundtrip() -> None:
    marker_length = 0.2
    h = marker_length / 2.0
    obj = np.array(
        [[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]],
        dtype=np.float32,
    )
    k = np.array([[400.0, 0.0, 320.0], [0.0, 400.0, 240.0], [0.0, 0.0, 1.0]])
    dist = np.zeros((5, 1), dtype=np.float64)
    rvec0 = np.array([[0.1], [0.05], [-0.02]], dtype=np.float64)
    tvec0 = np.array([[0.2], [-0.15], [2.5]], dtype=np.float64)
    img_pts, _jac = cv2.projectPoints(obj, rvec0, tvec0, k, dist)
    corners = img_pts.reshape(4, 2).astype(np.float32)
    result = estimate_marker_pose(corners, marker_length, k, dist)
    assert result is not None
    rvec, tvec = result
    np.testing.assert_allclose(rvec.reshape(3), rvec0.reshape(3), atol=1e-3)
    np.testing.assert_allclose(tvec.reshape(3), tvec0.reshape(3), atol=1e-3)
    assert marker_reprojection_error(corners, marker_length, k, dist, rvec, tvec) < 0.01


def test_marker_corners_to_bbox_accepts_aruco_shapes() -> None:
    corners = np.array([[[10.0, 20.0], [50.0, 18.0], [48.0, 60.0], [9.0, 58.0]]])
    assert marker_corners_to_bbox(corners) == (9.0, 18.0, 50.0, 60.0)
