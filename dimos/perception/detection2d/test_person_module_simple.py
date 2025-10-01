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

"""Simple test for PersonDetection2DModule configuration."""

import sys

sys.path.insert(0, "/home/lesh/coding/dimensional/dimos")

from dimos.perception.detection2d.module2D import Config
from dimos.perception.detection2d.detectors.person.yolo import YoloPersonDetector


def test_config():
    """Test that default config uses YoloPersonDetector."""
    config = Config()
    detector = config.detector()

    print(f"Config detector type: {type(detector)}")
    print(f"Is YoloPersonDetector: {isinstance(detector, YoloPersonDetector)}")

    assert isinstance(detector, YoloPersonDetector)
    print("\n✓ PersonDetection2DModule correctly configured to use YoloPersonDetector by default")


def test_detector_methods():
    """Test that the detector has the expected methods."""
    detector = YoloPersonDetector()

    # Check it has both generic and person-specific methods
    assert hasattr(detector, "process_image")
    assert hasattr(detector, "detect_people")

    print("\n✓ YoloPersonDetector has both process_image and detect_people methods")


if __name__ == "__main__":
    test_config()
    test_detector_methods()
