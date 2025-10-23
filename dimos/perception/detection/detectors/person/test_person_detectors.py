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

from dimos.perception.detection.type import Detection2DPerson, ImageDetections2D


@pytest.fixture(scope="session")
def people(person_detector, test_image):
    return person_detector.process_image(test_image)


@pytest.fixture(scope="session")
def person(people):
    return people[0]


def test_person_detection(people) -> None:
    """Test that we can detect people with pose keypoints."""
    assert len(people) > 0

    # Check first person
    person = people[0]
    assert isinstance(person, Detection2DPerson)
    assert person.confidence > 0
    assert len(person.bbox) == 4  # bbox is now a tuple
    assert person.keypoints.shape == (17, 2)
    assert person.keypoint_scores.shape == (17,)


def test_person_properties(people) -> None:
    """Test Detection2DPerson object properties and methods."""
    person = people[0]

    # Test bounding box properties
    assert person.width > 0
    assert person.height > 0
    assert len(person.center) == 2

    # Test keypoint access
    nose_xy, nose_conf = person.get_keypoint("nose")
    assert nose_xy.shape == (2,)
    assert 0 <= nose_conf <= 1

    # Test visible keypoints
    visible = person.get_visible_keypoints(threshold=0.5)
    assert len(visible) > 0
    assert all(isinstance(name, str) for name, _, _ in visible)
    assert all(xy.shape == (2,) for _, xy, _ in visible)
    assert all(0 <= conf <= 1 for _, _, conf in visible)


def test_person_normalized_coords(people) -> None:
    """Test normalized coordinates if available."""
    person = people[0]

    if person.keypoints_normalized is not None:
        assert person.keypoints_normalized.shape == (17, 2)
        # Check all values are in 0-1 range
        assert (person.keypoints_normalized >= 0).all()
        assert (person.keypoints_normalized <= 1).all()

    if person.bbox_normalized is not None:
        assert person.bbox_normalized.shape == (4,)
        assert (person.bbox_normalized >= 0).all()
        assert (person.bbox_normalized <= 1).all()


def test_multiple_people(people) -> None:
    """Test that multiple people can be detected."""
    print(f"\nDetected {len(people)} people in test image")

    for i, person in enumerate(people[:3]):  # Show first 3
        print(f"\nPerson {i}:")
        print(f"  Confidence: {person.confidence:.3f}")
        print(f"  Size: {person.width:.1f} x {person.height:.1f}")

        visible = person.get_visible_keypoints(threshold=0.8)
        print(f"  High-confidence keypoints (>0.8): {len(visible)}")
        for name, xy, conf in visible[:5]:
            print(f"    {name}: ({xy[0]:.1f}, {xy[1]:.1f}) conf={conf:.3f}")


def test_image_detections2d_structure(people) -> None:
    """Test that process_image returns ImageDetections2D."""
    assert isinstance(people, ImageDetections2D)
    assert len(people.detections) > 0
    assert all(isinstance(d, Detection2DPerson) for d in people.detections)


def test_invalid_keypoint(test_image) -> None:
    """Test error handling for invalid keypoint names."""
    # Create a dummy Detection2DPerson
    import numpy as np

    person = Detection2DPerson(
        # Detection2DBBox fields
        bbox=(0.0, 0.0, 100.0, 100.0),
        track_id=0,
        class_id=0,
        confidence=0.9,
        name="person",
        ts=test_image.ts,
        image=test_image,
        # Detection2DPerson fields
        keypoints=np.zeros((17, 2)),
        keypoint_scores=np.zeros(17),
    )

    with pytest.raises(ValueError):
        person.get_keypoint("invalid_keypoint")


def test_person_annotations(person) -> None:
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

    print("\n✓ Person annotations working correctly!")
    print(f"  - {len(person.get_visible_keypoints(0.5))}/17 visible keypoints")
