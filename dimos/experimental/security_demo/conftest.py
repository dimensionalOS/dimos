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

from __future__ import annotations

import pytest

from dimos.experimental.security_demo.security_module import SecurityModule
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.detectors.yolo import Yolo2DDetector
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.utils.data import get_data


@pytest.fixture(scope="session")
def yolo_detector():
    detector = Yolo2DDetector(device="cpu")
    yield detector
    detector.stop()


@pytest.fixture(scope="session")
def person_image():
    return Image.from_file(get_data("security_detection.png"))


@pytest.fixture(scope="session")
def empty_image():
    return Image.from_file(get_data("security_no_detection.png"))


@pytest.fixture()
def security_module(mocker):
    mocker.patch("dimos.experimental.security_demo.security_module._create_router")
    mocker.patch("dimos.experimental.security_demo.security_module._create_visual_servo")
    mocker.patch("dimos.experimental.security_demo.security_module.YoloPersonDetector")
    mocker.patch("dimos.experimental.security_demo.security_module.EdgeTAMProcessor")
    mocker.patch("dimos.experimental.security_demo.security_module.DepthEstimator")

    module = SecurityModule(camera_info=CameraInfo())

    # Replace output streams with mocks for test assertions
    module.detection = mocker.MagicMock()
    module.security_state = mocker.MagicMock()
    module.goal_request = mocker.MagicMock()
    module.cmd_vel = mocker.MagicMock()

    # These are set by framework wiring, not __init__
    module._planner_spec = mocker.MagicMock()
    module._speak_skill = mocker.MagicMock()

    yield module

    module.stop()


@pytest.fixture()
def make_detection(person_image):
    def _make(
        bbox=(100.0, 50.0, 300.0, 400.0),
        track_id=1,
        class_id=0,
        confidence=0.9,
        name="person",
    ):
        return Detection2DBBox(
            bbox=bbox,
            track_id=track_id,
            class_id=class_id,
            confidence=confidence,
            name=name,
            ts=0.0,
            image=person_image,
        )

    return _make
