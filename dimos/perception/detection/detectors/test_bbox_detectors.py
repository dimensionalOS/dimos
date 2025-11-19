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

from dimos.perception.detection.type import Detection2D, ImageDetections2D


@pytest.fixture(params=["bbox_detector", "person_detector"], scope="session")
def detector(request):
    """Parametrized fixture that provides both bbox and person detectors."""
    return request.getfixturevalue(request.param)


@pytest.fixture(scope="session")
def detections(detector, test_image):
    """Get ImageDetections2D from any detector."""
    return detector.process_image(test_image)


def test_detection_basic(detections) -> None:
    """Test that we can detect objects with all detectors."""
    assert len(detections.detections) > 0

    # Check first detection
    detection = detections.detections[0]
    assert isinstance(detection, Detection2D)
    assert detection.confidence > 0
    assert len(detection.bbox) == 4  # bbox is a tuple (x1, y1, x2, y2)
    assert detection.class_id >= 0
    assert detection.name is not None


def test_detection_bbox_properties(detections) -> None:
    """Test Detection2D bbox properties work for all detectors."""
    detection = detections.detections[0]

    # Test bounding box is valid
    x1, y1, x2, y2 = detection.bbox
    assert x2 > x1, "x2 should be greater than x1"
    assert y2 > y1, "y2 should be greater than y1"
    assert all(coord >= 0 for coord in detection.bbox), "Coordinates should be non-negative"

    # Test bbox volume
    volume = detection.bbox_2d_volume()
    assert volume > 0
    expected_volume = (x2 - x1) * (y2 - y1)
    assert abs(volume - expected_volume) < 0.01

    # Test center calculation
    center_x, center_y, width, height = detection.get_bbox_center()
    assert center_x == (x1 + x2) / 2.0
    assert center_y == (y1 + y2) / 2.0
    assert width == x2 - x1
    assert height == y2 - y1


def test_detection_cropped_image(detections, test_image) -> None:
    """Test cropping image to detection bbox."""
    detection = detections.detections[0]

    # Test cropped image
    cropped = detection.cropped_image(padding=20)
    assert cropped is not None

    # Cropped image should be smaller than original (usually)
    if test_image.shape:
        assert cropped.shape[0] <= test_image.shape[0]
        assert cropped.shape[1] <= test_image.shape[1]


def test_detection_annotations(detections) -> None:
    """Test annotation generation for detections."""
    detection = detections.detections[0]

    # Test text annotations - all detections should have at least 2
    text_annotations = detection.to_text_annotation()
    assert len(text_annotations) >= 2  # confidence and name/track_id (person has keypoints too)

    # Test points annotations - at least bbox
    points_annotations = detection.to_points_annotation()
    assert len(points_annotations) >= 1  # At least the bbox polygon

    # Test image annotations
    annotations = detection.to_image_annotations()
    assert annotations.texts_length >= 2
    assert annotations.points_length >= 1


def test_detection_ros_conversion(detections) -> None:
    """Test conversion to ROS Detection2D message."""
    detection = detections.detections[0]

    ros_det = detection.to_ros_detection2d()

    # Check bbox conversion
    center_x, center_y, width, height = detection.get_bbox_center()
    assert abs(ros_det.bbox.center.position.x - center_x) < 0.01
    assert abs(ros_det.bbox.center.position.y - center_y) < 0.01
    assert abs(ros_det.bbox.size_x - width) < 0.01
    assert abs(ros_det.bbox.size_y - height) < 0.01

    # Check confidence and class_id
    assert len(ros_det.results) > 0
    assert ros_det.results[0].hypothesis.score == detection.confidence
    assert ros_det.results[0].hypothesis.class_id == detection.class_id


def test_detection_is_valid(detections) -> None:
    """Test bbox validation."""
    detection = detections.detections[0]

    # Detection from real detector should be valid
    assert detection.is_valid()


def test_image_detections2d_structure(detections) -> None:
    """Test that process_image returns ImageDetections2D."""
    assert isinstance(detections, ImageDetections2D)
    assert len(detections.detections) > 0
    assert all(isinstance(d, Detection2D) for d in detections.detections)


def test_multiple_detections(detections) -> None:
    """Test that multiple objects can be detected."""
    print(f"\nDetected {len(detections.detections)} objects in test image")

    for i, detection in enumerate(detections.detections[:5]):  # Show first 5
        print(f"\nDetection {i}:")
        print(f"  Class: {detection.name} (id: {detection.class_id})")
        print(f"  Confidence: {detection.confidence:.3f}")
        print(
            f"  Bbox: ({detection.bbox[0]:.1f}, {detection.bbox[1]:.1f}, {detection.bbox[2]:.1f}, {detection.bbox[3]:.1f})"
        )
        print(f"  Track ID: {detection.track_id}")


def test_detection_string_representation(detections) -> None:
    """Test string representation of detections."""
    detection = detections.detections[0]
    str_repr = str(detection)

    # Should contain class name (either Detection2DBBox or Detection2DPerson)
    assert "Detection2D" in str_repr

    # Should show object name
    assert detection.name in str_repr or f"class_{detection.class_id}" in str_repr
