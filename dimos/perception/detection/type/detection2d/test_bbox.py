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


def test_detection2d(detection2d) -> None:
    # def test_detection_basic_properties(detection2d):
    """Test basic detection properties."""
    assert detection2d.track_id >= 0
    assert detection2d.class_id >= 0
    assert 0.0 <= detection2d.confidence <= 1.0
    assert detection2d.name is not None
    assert detection2d.ts > 0

    # def test_bounding_box_format(detection2d):
    """Test bounding box format and validity."""
    bbox = detection2d.bbox
    assert len(bbox) == 4, "Bounding box should have 4 values"

    x1, y1, x2, y2 = bbox
    assert x2 > x1, "x2 should be greater than x1"
    assert y2 > y1, "y2 should be greater than y1"
    assert x1 >= 0, "x1 should be non-negative"
    assert y1 >= 0, "y1 should be non-negative"

    # def test_bbox_2d_volume(detection2d):
    """Test bounding box volume calculation."""
    volume = detection2d.bbox_2d_volume()
    assert volume > 0, "Bounding box volume should be positive"

    # Calculate expected volume
    x1, y1, x2, y2 = detection2d.bbox
    expected_volume = (x2 - x1) * (y2 - y1)
    assert volume == pytest.approx(expected_volume, abs=0.001)

    # def test_bbox_center_calculation(detection2d):
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

    # def test_cropped_image(detection2d):
    """Test cropped image generation."""
    padding = 20
    cropped = detection2d.cropped_image(padding=padding)

    assert cropped is not None, "Cropped image should not be None"

    # The actual cropped image is (260, 192, 3)
    assert cropped.width == 192
    assert cropped.height == 260
    assert cropped.shape == (260, 192, 3)

    # def test_to_ros_bbox(detection2d):
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
