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

"""Test person annotations work correctly."""

import sys

from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection2d.detectors.person.yolo import YoloPersonDetector
from dimos.utils.data import get_data


def test_person_annotations():
    """Test that Person annotations include keypoints and skeleton."""
    image = Image.from_file(get_data("cafe.jpg"))
    detector = YoloPersonDetector()
    people = detector.detect_people(image)

    assert len(people) > 0
    person = people[0]

    # Test text annotations
    text_anns = person.to_text_annotation()
    print(f"\nText annotations: {len(text_anns)}")
    for i, ann in enumerate(text_anns):
        print(f"  {i}: {ann.text}")
    assert len(text_anns) == 3  # confidence, name/track_id, keypoints count
    assert any("keypoints:" in ann.text for ann in text_anns)

    # Test points annotations
    points_anns = person.to_points_annotation()
    print(f"\nPoints annotations: {len(points_anns)}")

    # Count different types (use actual LCM constants)
    from dimos_lcm.foxglove_msgs.ImageAnnotations import PointsAnnotation

    bbox_count = sum(1 for ann in points_anns if ann.type == PointsAnnotation.LINE_LOOP)  # 2
    keypoint_count = sum(1 for ann in points_anns if ann.type == PointsAnnotation.POINTS)  # 1
    skeleton_count = sum(1 for ann in points_anns if ann.type == PointsAnnotation.LINE_LIST)  # 4

    print(f"  - Bounding boxes: {bbox_count}")
    print(f"  - Keypoint circles: {keypoint_count}")
    print(f"  - Skeleton lines: {skeleton_count}")

    assert bbox_count >= 1  # At least the person bbox
    assert keypoint_count >= 1  # At least some visible keypoints
    assert skeleton_count >= 1  # At least some skeleton connections

    # Test full image annotations
    img_anns = person.to_image_annotations()
    assert img_anns.texts_length == len(text_anns)
    assert img_anns.points_length == len(points_anns)

    print(f"\n✓ Person annotations working correctly!")
    print(f"  - {len(person.get_visible_keypoints(0.5))}/17 visible keypoints")


if __name__ == "__main__":
    test_person_annotations()
