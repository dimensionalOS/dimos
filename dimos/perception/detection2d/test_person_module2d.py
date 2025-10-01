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

"""Test PersonDetection2DModule with person detection."""

import pytest
from unittest.mock import MagicMock
from dimos.perception.detection2d.module2D import PersonDetection2DModule, Config
from dimos.msgs.sensor_msgs import Image
from dimos.utils.data import get_data


def test_person_detection_module_config():
    """Test that PersonDetection2DModule uses YoloPersonDetector by default."""
    from dimos.perception.detection2d.detectors.person.yolo import YoloPersonDetector

    config = Config()
    detector = config.detector()
    assert isinstance(detector, YoloPersonDetector)


def test_person_detection_module_process():
    """Test that PersonDetection2DModule processes person detections correctly."""
    # Create module with mocked outputs
    module = PersonDetection2DModule()
    module.detections = MagicMock()
    module.annotations = MagicMock()

    # Load test image
    image = Image.from_file(get_data("cafe.jpg"))

    # Process image
    detections = module.process_image_frame(image)

    # Check that we get ImageDetections2D with Person objects
    assert detections is not None
    assert len(detections.detections) > 0

    # Check that detections are Person objects with keypoints
    from dimos.perception.detection2d.type.person import Person

    for det in detections.detections:
        assert isinstance(det, Person)
        assert hasattr(det, "keypoints")
        assert hasattr(det, "keypoint_scores")
        assert det.keypoints.shape == (17, 2)

    print(
        f"\n✓ PersonDetection2DModule detected {len(detections.detections)} people with keypoints"
    )


def test_backward_compatibility():
    """Test that Detection2DModule alias still works."""
    from dimos.perception.detection2d.module2D import Detection2DModule

    assert Detection2DModule is PersonDetection2DModule


if __name__ == "__main__":
    test_person_detection_module_config()
    test_person_detection_module_process()
    test_backward_compatibility()
