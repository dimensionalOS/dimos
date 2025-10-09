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


def test_detection_basic_properties(detection2d):
    """Test basic detection properties."""
    assert detection2d.track_id >= 0
    assert detection2d.class_id >= 0
    assert 0.0 <= detection2d.confidence <= 1.0
    assert detection2d.name is not None
    assert detection2d.ts > 0


def test_bounding_box_format(detection2d):
    """Test bounding box format and validity."""
    bbox = detection2d.bbox
    assert len(bbox) == 4, "Bounding box should have 4 values"

    x1, y1, x2, y2 = bbox
    assert x2 > x1, "x2 should be greater than x1"
    assert y2 > y1, "y2 should be greater than y1"
    assert x1 >= 0, "x1 should be non-negative"
    assert y1 >= 0, "y1 should be non-negative"


def test_bbox_2d_volume(detection2d):
    """Test bounding box volume calculation."""
    volume = detection2d.bbox_2d_volume()
    assert volume > 0, "Bounding box volume should be positive"

    # Calculate expected volume
    x1, y1, x2, y2 = detection2d.bbox
    expected_volume = (x2 - x1) * (y2 - y1)
    assert volume == pytest.approx(expected_volume, abs=0.001)


def test_bbox_center_calculation(detection2d):
    """Test bounding box center calculation."""
    center_bbox = detection2d.get_bbox_center()
    assert len(center_bbox) == 4, "Center bbox should have 4 values"

    center_x, center_y, width, height = center_bbox
    x1, y1, x2, y2 = detection2d.bbox

    # Verify center calculations
    assert center_x == pytest.approx((x1 + x2) / 2.0, abs=0.001)
    assert center_y == pytest.approx((y1 + y2) / 2.0, abs=0.001)
    assert width == pytest.approx(x2 - x1, abs=0.001)
    assert height == pytest.approx(y2 - y1, abs=0.001)


def test_cropped_image(detection2d):
    """Test cropped image generation."""
    padding = 20
    cropped = detection2d.cropped_image(padding=padding)

    assert cropped is not None, "Cropped image should not be None"

    # The actual cropped image is (260, 192, 3)
    assert cropped.width == 192
    assert cropped.height == 260
    assert cropped.shape == (260, 192, 3)


def test_to_ros_bbox(detection2d):
    """Test ROS bounding box conversion."""
    ros_bbox = detection2d.to_ros_bbox()

    assert ros_bbox is not None
    assert hasattr(ros_bbox, "center")
    assert hasattr(ros_bbox, "size_x")
    assert hasattr(ros_bbox, "size_y")

    # Verify values match
    center_x, center_y, width, height = detection2d.get_bbox_center()
    assert ros_bbox.center.position.x == pytest.approx(center_x, abs=0.001)
    assert ros_bbox.center.position.y == pytest.approx(center_y, abs=0.001)
    assert ros_bbox.size_x == pytest.approx(width, abs=0.001)
    assert ros_bbox.size_y == pytest.approx(height, abs=0.001)


def test_to_text_annotation(detection2d):
    """Test text annotation generation."""
    text_annotations = detection2d.to_text_annotation()

    # Actually returns 2 annotations
    assert len(text_annotations) == 2, "Should have two text annotations"

    # First annotation: confidence
    text0 = text_annotations[0]
    assert text0.text == "confidence: 0.815"
    assert text0.position.x == pytest.approx(503.437, abs=0.001)
    assert text0.position.y == pytest.approx(489.829, abs=0.001)

    # Second annotation: name_class_track
    text1 = text_annotations[1]
    assert text1.text == "suitcase_28_1"
    assert text1.position.x == pytest.approx(503.437, abs=0.001)
    assert text1.position.y == pytest.approx(249.894, abs=0.001)


def test_to_points_annotation(detection2d):
    """Test points annotation generation for bbox corners."""
    points_annotations = detection2d.to_points_annotation()

    assert len(points_annotations) == 1, "Should have one points annotation"
    points = points_annotations[0]

    # Actually has 4 points forming a LINE_LOOP
    assert points.points_length == 4
    assert points.type == 2  # LINE_LOOP

    x1, y1, x2, y2 = detection2d.bbox
    expected_corners = [
        (x1, y1),  # Top-left
        (x1, y2),  # Bottom-left
        (x2, y2),  # Bottom-right
        (x2, y1),  # Top-right
    ]

    for i, (expected_x, expected_y) in enumerate(expected_corners):
        point = points.points[i]
        assert point.x == pytest.approx(expected_x, abs=0.001)
        assert point.y == pytest.approx(expected_y, abs=0.001)


def test_to_image_annotations(detection2d):
    """Test complete image annotations generation."""
    annotations = detection2d.to_image_annotations()

    assert annotations is not None
    assert hasattr(annotations, "points")
    assert hasattr(annotations, "texts")

    # Has 1 points annotation and 2 text annotations
    assert annotations.points_length == 1
    assert annotations.texts_length == 2


def test_to_repr_dict(detection2d):
    """Test dictionary representation for display."""
    repr_dict = detection2d.to_repr_dict()

    assert "name" in repr_dict
    assert "class" in repr_dict
    assert "track" in repr_dict
    assert "conf" in repr_dict
    assert "bbox" in repr_dict

    # Verify string formatting
    assert repr_dict["class"] == str(detection2d.class_id)
    assert repr_dict["track"] == str(detection2d.track_id)
    assert repr_dict["conf"] == f"{detection2d.confidence:.2f}"


def test_image_detections2d_properties(image_detections2d):
    """Test ImageDetections2D container properties."""
    assert hasattr(image_detections2d, "detections")
    assert hasattr(image_detections2d, "image")
    assert hasattr(image_detections2d, "ts")

    # Check that all detections share the same image and timestamp
    for det in image_detections2d.detections:
        assert det.image is image_detections2d.image
        assert det.ts == image_detections2d.ts


def test_ros_detection2d_conversion(detection2d):
    """Test conversion to ROS Detection2D message."""
    ros_det = detection2d.to_ros_detection2d()

    assert ros_det is not None
    assert hasattr(ros_det, "header")
    assert hasattr(ros_det, "results")
    assert hasattr(ros_det, "bbox")
    assert hasattr(ros_det, "id")

    # Check header
    assert ros_det.header.stamp.sec > 0
    assert ros_det.header.frame_id == "camera_link"

    # Check detection ID
    assert ros_det.id == str(detection2d.track_id)

    # Currently results_length is 0 in the implementation
    assert ros_det.results_length == 0

    # Check bbox is set
    assert ros_det.bbox is not None
