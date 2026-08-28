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

from unittest.mock import call

import cv2
import numpy as np
import pytest_mock

from dimos.hardware.sensors.camera.webcam import Webcam
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo


def test_start_requests_and_reports_camera_mode(mocker: pytest_mock.MockerFixture) -> None:
    capture = mocker.patch("cv2.VideoCapture").return_value
    capture.isOpened.return_value = True
    capture.get.side_effect = [30.0, 640.0, 480.0]
    thread = mocker.patch("dimos.hardware.sensors.camera.webcam.threading.Thread").return_value
    log = mocker.patch("dimos.hardware.sensors.camera.webcam.logger.info")
    webcam = Webcam(camera_index=2, width=640, height=480, fps=30.0)

    webcam.start()

    assert capture.set.call_args_list == [
        call(cv2.CAP_PROP_FRAME_WIDTH, 640),
        call(cv2.CAP_PROP_FRAME_HEIGHT, 480),
        call(cv2.CAP_PROP_FPS, 30.0),
    ]
    log.assert_called_once_with(
        "Webcam %s requested %.1fHz %dx%d; negotiated %.1fHz %dx%d",
        2,
        30.0,
        640,
        480,
        30.0,
        640,
        480,
    )
    thread.start.assert_called_once_with()


def test_capture_timestamp_is_taken_immediately_after_read(
    mocker: pytest_mock.MockerFixture,
) -> None:
    frame = np.zeros((2, 3, 3), dtype=np.uint8)
    capture = mocker.MagicMock()
    capture.read.return_value = (True, frame)
    webcam = Webcam(camera_index=2, width=3, height=2, fps=30.0)
    webcam._capture = capture
    now = mocker.patch("dimos.hardware.sensors.camera.webcam.time.time", return_value=42.5)

    image = webcam.capture_frame()

    now.assert_called_once_with()
    assert image.ts == 42.5
    assert image.data.shape == (2, 3, 3)


def test_webcam_without_intrinsics_reports_a_nominal_pinhole() -> None:
    info = Webcam(width=1280, height=720).camera_info
    assert (info.width, info.height) == (1280, 720)
    assert info.K[0] > 0
    assert info.frame_id == "camera_optical"


def test_webcam_keeps_configured_intrinsics() -> None:
    configured = CameraInfo.from_fov(90.0, 640, 480)
    assert Webcam(camera_info=configured).camera_info is configured


def test_webcam_with_size_but_no_focal_length_gets_a_nominal_pinhole() -> None:
    info = Webcam(camera_info=CameraInfo(width=1280, height=720)).camera_info
    assert info.K[0] > 0


def test_webcam_stereo_slice_halves_the_nominal_pinhole_width() -> None:
    info = Webcam(width=1280, height=720, stereo_slice="left").camera_info
    assert (info.width, info.height) == (640, 720)
