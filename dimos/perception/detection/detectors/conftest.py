# Copyright 2025 Dimensional Inc.
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

import pytest

from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection.detectors.person.yolo import YoloPersonDetector
from dimos.perception.detection.detectors.yolo import Yolo2DDetector
from dimos.utils.data import get_data


@pytest.fixture(scope="session")
def test_image():
    """Load the test image used for detector tests."""
    return Image.from_file(get_data("cafe.jpg"))


@pytest.fixture(scope="session")
def person_detector():
    """Create a YoloPersonDetector instance."""
    return YoloPersonDetector()


@pytest.fixture(scope="session")
def bbox_detector():
    """Create a Yolo2DDetector instance for general object detection."""
    return Yolo2DDetector()
