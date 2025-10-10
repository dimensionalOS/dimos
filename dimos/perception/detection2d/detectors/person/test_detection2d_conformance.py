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
from dimos.perception.detection2d.detectors.person.yolo import YoloPersonDetector
from dimos.perception.detection2d.type.person import Person
from dimos.utils.data import get_data


def test_person_detection2d_bbox_conformance():
    """Test that Person conforms to Detection2DBBox interface."""
    image = Image.from_file(get_data("cafe.jpg"))
    detector = YoloPersonDetector()
    people = detector.detect_people(image)

    assert len(people) > 0
    person = people[0]

    # Test Detection2DBBox methods
    # Test bbox operations
    assert hasattr(person, "bbox")
    assert len(person.bbox) == 4
    assert all(isinstance(x, float) for x in person.bbox)

    # Test inherited properties
    assert hasattr(person, "get_bbox_center")
    center_bbox = person.get_bbox_center()
    assert len(center_bbox) == 4  # center_x, center_y, width, height

    # Test volume calculation
    volume = person.bbox_2d_volume()
    assert volume > 0

    # Test cropped image
    cropped = person.cropped_image(padding=10)
    assert isinstance(cropped, Image)

    # Test annotation methods
    text_annotations = person.to_text_annotation()
    assert len(text_annotations) == 3  # confidence, name/track_id, and keypoints count

    points_annotations = person.to_points_annotation()
    # Should have: 1 bbox + 1 keypoints + multiple skeleton lines
    assert len(points_annotations) > 1
    print(f"  - Points annotations: {len(points_annotations)} (bbox + keypoints + skeleton)")

    # Test image annotations
    annotations = person.to_image_annotations()
    assert annotations.texts_length == 3
    assert annotations.points_length > 1

    # Test ROS conversion
    ros_det = person.to_ros_detection2d()
    assert ros_det.bbox.size_x == person.width
    assert ros_det.bbox.size_y == person.height

    # Test string representation
    str_repr = str(person)
    assert "Person" in str_repr
    assert "person" in str_repr  # name field

    print("\n✓ Person class fully conforms to Detection2DBBox interface")
    print(f"  - Detected {len(people)} people")
    print(f"  - First person confidence: {person.confidence:.3f}")
    print(f"  - Bbox volume: {volume:.1f}")
    print(f"  - Has {len(person.get_visible_keypoints(0.5))} visible keypoints")


if __name__ == "__main__":
    test_person_detection2d_bbox_conformance()
