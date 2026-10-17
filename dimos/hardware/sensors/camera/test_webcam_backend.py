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

import cv2
import pytest

from dimos.hardware.sensors.camera.webcam import Webcam


@pytest.fixture
def webcam(mocker, request):
    camera = Webcam(camera_index=request.param)
    mocker.patch("dimos.hardware.sensors.camera.webcam.threading.Thread")
    yield camera
    camera.stop()


@pytest.mark.parametrize(
    ("platform", "webcam", "device", "backend"),
    [
        ("linux", "/dev/video0", "/dev/video0", cv2.CAP_V4L2),
        ("linux", "/dev/v4l/by-id/usb-camera", "/dev/v4l/by-id/usb-camera", cv2.CAP_V4L2),
        ("linux", 2, 2, cv2.CAP_ANY),
        ("linux", "2", 2, cv2.CAP_ANY),
        ("linux", "-1", -1, cv2.CAP_ANY),
        ("linux", "-2", -2, cv2.CAP_ANY),
        ("linux", "+2", 2, cv2.CAP_ANY),
        ("linux", " 2 ", 2, cv2.CAP_ANY),
        ("linux", " -1 ", -1, cv2.CAP_ANY),
        ("linux", "rtsp://camera/stream", "rtsp://camera/stream", cv2.CAP_ANY),
        ("darwin", 0, 0, cv2.CAP_ANY),
        ("darwin", "/dev/video0", "/dev/video0", cv2.CAP_ANY),
    ],
    indirect=["webcam"],
)
def test_capture_backend(platform, webcam, device, backend, mocker):
    video_capture = mocker.patch("cv2.VideoCapture")
    mocker.patch("sys.platform", platform)

    webcam.start()

    video_capture.assert_called_once_with(device, backend)
