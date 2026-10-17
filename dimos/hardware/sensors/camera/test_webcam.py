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

from dimos.hardware.sensors.camera.webcam import Webcam
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo


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
