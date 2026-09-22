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

from dimos_generated.sensor_msgs.msg import CameraInfo, Image
from dimos_generated.std_msgs.msg import Header
import numpy as np
import pytest

from dimos.hardware.sensors.camera.webcam import Webcam
from dimos.msgs.camera_info import camera_info_from_fov
from dimos.msgs.image import image_view


def test_webcam_without_intrinsics_reports_a_nominal_pinhole() -> None:
    info = Webcam(width=1280, height=720).camera_info
    assert (info.width, info.height) == (1280, 720)
    assert info.k[0] > 0
    assert info.header.frame_id == "camera_optical"


def test_webcam_keeps_configured_intrinsics() -> None:
    configured = camera_info_from_fov(90.0, 640, 480, header=Header())
    assert Webcam(camera_info=configured).camera_info is configured


def test_webcam_with_size_but_no_focal_length_gets_a_nominal_pinhole() -> None:
    info = Webcam(camera_info=CameraInfo(width=1280, height=720)).camera_info
    assert info.k[0] > 0


def test_webcam_stereo_slice_halves_the_nominal_pinhole_width() -> None:
    info = Webcam(width=1280, height=720, stereo_slice="left").camera_info
    assert (info.width, info.height) == (640, 720)


@pytest.mark.parametrize("side,start", [("left", 0), ("right", 2)])
def test_capture_converts_bgr_and_crops_before_cdr_encoding(side, start, mocker):
    camera = Webcam(stereo_slice=side, frame_id_prefix="robot")
    pixels = np.arange(2 * 4 * 3, dtype=np.uint8).reshape(2, 4, 3)
    capture = mocker.Mock()
    capture.read.return_value = (True, pixels)
    camera._capture = capture
    mocker.patch("dimos.msgs.time.time.time_ns", return_value=1700000000123456789)
    try:
        message = Image.decode(camera.capture_frame().encode())
    finally:
        camera.stop()
    assert message.encoding == "rgb8"
    assert (message.width, message.height, message.step) == (2, 2, 6)
    assert message.header.frame_id == "robot/camera_optical"
    assert (message.header.stamp.sec, message.header.stamp.nanosec) == (1700000000, 123456789)
    np.testing.assert_array_equal(image_view(message), pixels[:, start : start + 2, ::-1])
    capture.release.assert_called_once()
