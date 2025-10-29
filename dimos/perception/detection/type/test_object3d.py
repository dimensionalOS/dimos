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

from dimos.perception.detection.moduleDB import Object3D
from dimos.perception.detection.type.detection3d import ImageDetections3DPC


def test_first_object(first_object) -> None:
    # def test_object3d_properties(first_object):
    """Test basic properties of an Object3D."""
    assert first_object.track_id is not None
    assert isinstance(first_object.track_id, str)
    assert first_object.name is not None
    assert first_object.class_id >= 0
    assert 0.0 <= first_object.confidence <= 1.0
    assert first_object.ts > 0
    assert first_object.frame_id is not None
    assert first_object.best_detection is not None

    # def test_object3d_center(first_object):
    """Test Object3D center calculation."""
    assert first_object.center is not None
    assert hasattr(first_object.center, "x")
    assert hasattr(first_object.center, "y")
    assert hasattr(first_object.center, "z")

    # Center should be within reasonable bounds
    assert -10 < first_object.center.x < 10
    assert -10 < first_object.center.y < 10
    assert -10 < first_object.center.z < 10


def test_object3d_repr_dict(first_object) -> None:
    """Test to_repr_dict method."""
    repr_dict = first_object.to_repr_dict()

    assert "object_id" in repr_dict
    assert "detections" in repr_dict
    assert "center" in repr_dict

    assert repr_dict["object_id"] == first_object.track_id
    assert repr_dict["detections"] == first_object.detections

    # Center should be formatted as string with coordinates
    assert isinstance(repr_dict["center"], str)
    assert repr_dict["center"].startswith("[")
    assert repr_dict["center"].endswith("]")

    # def test_object3d_scene_entity_label(first_object):
    """Test scene entity label generation."""
    label = first_object.scene_entity_label()

    assert isinstance(label, str)
    assert first_object.name in label
    assert f"({first_object.detections})" in label

    # def test_object3d_agent_encode(first_object):
    """Test agent encoding."""
    encoded = first_object.agent_encode()

    assert isinstance(encoded, dict)
    assert "id" in encoded
    assert "name" in encoded
    assert "detections" in encoded
    assert "last_seen" in encoded

    assert encoded["id"] == first_object.track_id
    assert encoded["name"] == first_object.name
    assert encoded["detections"] == first_object.detections
    assert encoded["last_seen"].endswith("s ago")

    # def test_object3d_image_property(first_object):
    """Test get_image method returns best_detection's image."""
    assert first_object.get_image() is not None
    assert first_object.get_image() is first_object.best_detection.image


def test_all_objeects(all_objects) -> None:
    # def test_object3d_multiple_detections(all_objects):
    """Test objects that have been built from multiple detections."""
    # Find objects with multiple detections
    multi_detection_objects = [obj for obj in all_objects if obj.detections > 1]

    if multi_detection_objects:
        obj = multi_detection_objects[0]

        # Since detections is now a counter, we can only test that we have multiple detections
        # and that best_detection exists
        assert obj.detections > 1
        assert obj.best_detection is not None
        assert obj.confidence is not None
        assert obj.ts > 0

        # Test that best_detection has reasonable properties
        assert obj.best_detection.bbox_2d_volume() > 0

    # def test_object_db_module_objects_structure(all_objects):
    """Test the structure of objects in the database."""
    for obj in all_objects:
        assert isinstance(obj, Object3D)
        assert hasattr(obj, "track_id")
        assert hasattr(obj, "detections")
        assert hasattr(obj, "best_detection")
        assert hasattr(obj, "center")
        assert obj.detections >= 1


def test_objectdb_module(object_db_module) -> None:
    # def test_object_db_module_populated(object_db_module):
    """Test that ObjectDBModule is properly populated."""
    assert len(object_db_module.objects) > 0, "Database should contain objects"
    assert object_db_module.cnt > 0, "Object counter should be greater than 0"

    # def test_object3d_addition(object_db_module):
    """Test Object3D addition operator."""
    # Get existing objects from the database
    objects = list(object_db_module.objects.values())
    if len(objects) < 2:
        pytest.skip("Not enough objects in database")

    # Get detections from two different objects
    det1 = objects[0].best_detection
    det2 = objects[1].best_detection

    # Create a new object with the first detection
    obj = Object3D("test_track_combined", det1)

    # Add the second detection from a different object
    combined = obj + det2

    assert combined.track_id == "test_track_combined"
    assert combined.detections == 2

    # Since detections is now a counter, we can't check if specific detections are in the list
    # We can only verify the count and that best_detection is properly set

    # Best detection should be determined by the Object3D logic
    assert combined.best_detection is not None

    # Center should be valid (no specific value check since we're using real detections)
    assert hasattr(combined, "center")
    assert combined.center is not None

    # def test_image_detections3d_scene_update(object_db_module):
    """Test ImageDetections3DPC to Foxglove scene update conversion."""
    # Get some detections
    objects = list(object_db_module.objects.values())
    if not objects:
        pytest.skip("No objects in database")

    detections = [obj.best_detection for obj in objects[:3]]  # Take up to 3

    image_detections = ImageDetections3DPC(image=detections[0].image, detections=detections)

    scene_update = image_detections.to_foxglove_scene_update()

    assert scene_update is not None
    assert scene_update.entities_length == len(detections)

    for i, entity in enumerate(scene_update.entities):
        assert entity.id == str(detections[i].track_id)
        assert entity.frame_id == detections[i].frame_id
        assert entity.cubes_length == 1
        assert entity.texts_length == 1
