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

"""Synthetic observations shared by hand-eye solver and workflow tests."""

import json

import cv2
import numpy as np
import pytest

from dimos.calibration.hand_eye import HandEyeSample
from dimos.calibration.hand_eye_session import open_session
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat


@pytest.fixture
def make_transform():
    def make(rotation, translation):
        result = np.eye(4)
        result[:3, :3] = cv2.Rodrigues(np.asarray(rotation, dtype=np.float64))[0]
        result[:3, 3] = translation
        return result

    return make


@pytest.fixture
def observations(make_transform):
    rng = np.random.default_rng(17)
    camera_to_ee = make_transform([0.2, -0.3, 0.1], [0.04, 0.02, 0.08])
    base_from_target = make_transform([0.1, 0.15, -0.1], [0.2, 0.0, 0.9])
    samples = []
    for _ in range(20):
        base_from_ee = make_transform(rng.uniform(-0.6, 0.6, 3), rng.uniform(-0.1, 0.1, 3))
        camera_from_target = np.linalg.inv(base_from_ee @ camera_to_ee) @ base_from_target
        samples.append(HandEyeSample(base_from_ee, camera_from_target, 0.3))
    return samples, camera_to_ee, base_from_target


@pytest.fixture
def marker_image(make_transform):
    info = CameraInfo.from_intrinsics(600, 600, 320, 240, 640, 480)
    target = make_transform([2.9, 0.15, -0.2], [0.01, -0.02, 0.6])
    points = np.array(
        [[-0.075, 0.075, 0], [0.075, 0.075, 0], [0.075, -0.075, 0], [-0.075, -0.075, 0]]
    )
    pixels, _ = cv2.projectPoints(
        points, cv2.Rodrigues(target[:3, :3])[0], target[:3, 3], info.get_K_matrix(), None
    )
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    tag = cv2.aruco.generateImageMarker(dictionary, 0, 200)
    source = np.array([[0, 0], [199, 0], [199, 199], [0, 199]], dtype=np.float32)
    homography = cv2.getPerspectiveTransform(source, pixels.reshape(4, 2).astype(np.float32))
    pixels_image = cv2.warpPerspective(tag, homography, (640, 480), borderValue=255)
    return (
        Image(
            data=pixels_image,
            format=ImageFormat.GRAY,
            frame_id="camera_color_optical_frame",
            ts=123.0,
        ),
        info,
        target,
    )


@pytest.fixture
def saved_session(tmp_path, observations, marker_image):
    samples, _, _ = observations
    _, info, _ = marker_image
    data = open_session(tmp_path, info, ee_frame="arm_link6")
    data["camera_frame"] = "camera_color_optical_frame"
    data["samples"] = [
        {
            "id": f"{i + 1:04d}",
            "role": "train" if i < 15 else "validation",
            "image_timestamp": float(i),
            "base_from_ee": sample.base_from_ee.tolist(),
            "camera_from_target": sample.camera_from_target.tolist(),
            "reprojection_error_px": sample.reprojection_error_px,
        }
        for i, sample in enumerate(samples)
    ]
    (tmp_path / "session.json").write_text(json.dumps(data))
    return tmp_path
