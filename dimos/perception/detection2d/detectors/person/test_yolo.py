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


@pytest.fixture()
def detector():
    return YoloPersonDetector()


@pytest.fixture()
def test_image():
    return Image.from_file(get_data("cafe.jpg"))


@pytest.fixture()
def people(detector, test_image):
    return detector.detect_people(test_image)


def test_person_detection(people):
    """Test that we can detect people with pose keypoints."""
    assert len(people) > 0

    # Check first person
    person = people[0]
    assert isinstance(person, Person)
    assert person.confidence > 0
    assert len(person.bbox) == 4  # bbox is now a tuple
    assert person.keypoints.shape == (17, 2)
    assert person.keypoint_scores.shape == (17,)


def test_person_properties(people):
    """Test Person object properties and methods."""
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


def test_person_normalized_coords(people):
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


def test_multiple_people(people):
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


def test_invalid_keypoint(test_image):
    """Test error handling for invalid keypoint names."""
    # Create a dummy person
    import numpy as np

    person = Person(
        # Detection2DBBox fields
        bbox=(0.0, 0.0, 100.0, 100.0),
        track_id=0,
        class_id=0,
        confidence=0.9,
        name="person",
        ts=test_image.ts,
        image=test_image,
        # Person fields
        keypoints=np.zeros((17, 2)),
        keypoint_scores=np.zeros(17),
    )

    with pytest.raises(ValueError):
        person.get_keypoint("invalid_keypoint")
