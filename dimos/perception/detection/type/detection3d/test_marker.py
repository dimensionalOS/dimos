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
import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.vision_msgs.Detection3D import Detection3D
from dimos.perception.detection.type.detection3d.marker import Detection3DMarker
from dimos.perception.fiducial.marker_pose import marker_reprojection_error


def _project_synthetic_marker_corners(
    marker_length_m: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
) -> np.ndarray:
    h = marker_length_m / 2.0
    obj = np.array(
        [[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]],
        dtype=np.float32,
    )
    projected, _jac = cv2.projectPoints(obj, rvec, tvec, camera_matrix, dist_coeffs)
    return projected.reshape(4, 2).astype(np.float32)


def test_marker_detection3d_msg_preserves_marker_identity_on_wire() -> None:
    center = Vector3(1.0, 2.0, 3.0)
    orientation = Quaternion(0.1, 0.2, 0.3, 0.9)
    size = Vector3(0.16, 0.16, 0.0)
    marker_length_m = 0.16
    camera_matrix = np.array(
        [[420.0, 0.0, 60.0], [0.0, 420.0, 50.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    dist_coeffs = np.zeros((5, 1), dtype=np.float64)
    rvec = np.array([[0.05], [-0.02], [0.01]], dtype=np.float64)
    tvec = np.array([[0.1], [-0.05], [1.8]], dtype=np.float64)
    corners_px = _project_synthetic_marker_corners(
        marker_length_m,
        camera_matrix,
        dist_coeffs,
        rvec,
        tvec,
    )
    corners_px[1] += np.array([3.0, -4.0], dtype=np.float32)
    reprojection_error = marker_reprojection_error(
        corners_px,
        marker_length_m,
        camera_matrix,
        dist_coeffs,
        rvec,
        tvec,
    )
    image = Image(
        data=np.zeros((100, 120, 3), dtype=np.uint8),
        frame_id="camera_optical",
        ts=123.456,
    )

    det = Detection3DMarker(
        bbox=(10.0, 20.0, 50.0, 80.0),
        track_id=7,
        class_id=999,
        confidence=1.0,
        name="marker_42",
        ts=image.ts,
        image=image,
        center=center,
        size=size,
        frame_id="world",
        orientation=orientation,
        marker_id=42,
        corners_px=corners_px,
        dictionary="DICT_APRILTAG_36h11",
        reprojection_error=reprojection_error,
    )

    assert det.name == "DICT_APRILTAG_36h11:42"
    assert det.reprojection_error == pytest.approx(2.5)

    msg = det.to_detection3d_msg()

    assert msg.header.frame_id == "world"
    assert msg.id == "42"
    assert msg.results_length == 1
    assert msg.results[0].hypothesis.class_id == "DICT_APRILTAG_36h11:42"
    assert msg.results[0].hypothesis.score == pytest.approx(1.0)
    assert msg.bbox.center.position.x == pytest.approx(center.x)
    assert msg.bbox.center.position.y == pytest.approx(center.y)
    assert msg.bbox.center.position.z == pytest.approx(center.z)
    assert msg.bbox.center.orientation.x == pytest.approx(orientation.x)
    assert msg.bbox.center.orientation.y == pytest.approx(orientation.y)
    assert msg.bbox.center.orientation.z == pytest.approx(orientation.z)
    assert msg.bbox.center.orientation.w == pytest.approx(orientation.w)
    assert msg.bbox.size.x == pytest.approx(size.x)
    assert msg.bbox.size.y == pytest.approx(size.y)
    assert msg.bbox.size.z == pytest.approx(size.z)

    decoded = Detection3D.lcm_decode(msg.lcm_encode())
    assert decoded.id == "42"
    assert decoded.results_length == 1
    assert decoded.results[0].hypothesis.class_id == "DICT_APRILTAG_36h11:42"
