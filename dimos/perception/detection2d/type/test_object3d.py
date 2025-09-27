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

from dimos.perception.detection2d import testing
from dimos.perception.detection2d.module2D import Detection2DModule
from dimos.perception.detection2d.module3D import Detection3DModule
from dimos.perception.detection2d.moduleDB import Object3D, ObjectDBModule
from dimos.perception.detection2d.type.detection3d import ImageDetections3D
from dimos.protocol.service import lcmservice as lcm
from dimos.robot.unitree_webrtc.modular.connection_module import ConnectionModule


@pytest.fixture(scope="session", autouse=True)
def setup_lcm():
    """Configure LCM for the test session."""
    lcm.autoconf()


@pytest.fixture(scope="session")
def object_db_module():
    """Create and populate an ObjectDBModule with detections from multiple frames."""
    module2d = Detection2DModule()
    module3d = Detection3DModule(camera_info=ConnectionModule._camera_info())
    moduleDB = ObjectDBModule(
        camera_info=ConnectionModule._camera_info(),
        goto=lambda obj_id: None,  # No-op for testing
    )

    # Process 5 frames to build up object history
    for i in range(5):
        seek_value = 10.0 + (i * 2)
        moment = testing.get_moment(seek=seek_value)

        # Process 2D detections
        imageDetections2d = module2d.process_image_frame(moment["image_frame"])

        # Get camera transform
        camera_transform = moment["tf"].get("camera_optical", moment.get("lidar_frame").frame_id)

        # Process 3D detections
        imageDetections3d = module3d.process_frame(
            imageDetections2d, moment["lidar_frame"], camera_transform
        )

        # Add to database
        moduleDB.add_detections(imageDetections3d)

    return moduleDB


@pytest.fixture(scope="session")
def first_object(object_db_module):
    """Get the first object from the database."""
    objects = list(object_db_module.objects.values())
    assert len(objects) > 0, "No objects found in database"
    return objects[0]


@pytest.fixture(scope="session")
def all_objects(object_db_module):
    """Get all objects from the database."""
    return list(object_db_module.objects.values())


def test_object_db_module_populated(object_db_module):
    """Test that ObjectDBModule is properly populated."""
    assert len(object_db_module.objects) > 0, "Database should contain objects"
    assert object_db_module.cnt > 0, "Object counter should be greater than 0"


def test_object_db_module_objects_structure(all_objects):
    """Test the structure of objects in the database."""
    for obj in all_objects:
        assert isinstance(obj, Object3D)
        assert hasattr(obj, "track_id")
        assert hasattr(obj, "detections")
        assert hasattr(obj, "best_detection")
        assert hasattr(obj, "center")
        assert len(obj.detections) >= 1


def test_object3d_properties(first_object):
    """Test basic properties of an Object3D."""
    assert first_object.track_id is not None
    assert isinstance(first_object.track_id, str)
    assert first_object.name is not None
    assert first_object.class_id >= 0
    assert 0.0 <= first_object.confidence <= 1.0
    assert first_object.ts > 0
    assert first_object.frame_id is not None
    assert first_object.best_detection is not None


def test_object3d_multiple_detections(all_objects):
    """Test objects that have been built from multiple detections."""
    # Find objects with multiple detections
    multi_detection_objects = [obj for obj in all_objects if len(obj.detections) > 1]

    if multi_detection_objects:
        obj = multi_detection_objects[0]

        # Test that confidence is the max of all detections
        max_conf = max(d.confidence for d in obj.detections)
        assert obj.confidence == max_conf

        # Test that timestamp is the max (most recent)
        max_ts = max(d.ts for d in obj.detections)
        assert obj.ts == max_ts

        # Test that best_detection has the largest bbox volume
        best_volume = obj.best_detection.bbox_2d_volume()
        for det in obj.detections:
            assert det.bbox_2d_volume() <= best_volume


def test_object3d_center(first_object):
    """Test Object3D center calculation."""
    assert first_object.center is not None
    assert hasattr(first_object.center, "x")
    assert hasattr(first_object.center, "y")
    assert hasattr(first_object.center, "z")

    # Center should be within reasonable bounds
    assert -10 < first_object.center.x < 10
    assert -10 < first_object.center.y < 10
    assert -10 < first_object.center.z < 10


def test_object3d_repr_dict(first_object):
    """Test to_repr_dict method."""
    repr_dict = first_object.to_repr_dict()

    assert "object_id" in repr_dict
    assert "detections" in repr_dict
    assert "center" in repr_dict

    assert repr_dict["object_id"] == first_object.track_id
    assert repr_dict["detections"] == len(first_object.detections)

    # Center should be formatted as string with coordinates
    assert isinstance(repr_dict["center"], str)
    assert repr_dict["center"].startswith("[")
    assert repr_dict["center"].endswith("]")


def test_object3d_scene_entity_label(first_object):
    """Test scene entity label generation."""
    label = first_object.scene_entity_label()

    assert isinstance(label, str)
    assert first_object.name in label
    assert f"({len(first_object.detections)})" in label


def test_object3d_agent_encode(first_object):
    """Test agent encoding."""
    encoded = first_object.agent_encode()

    assert isinstance(encoded, dict)
    assert "id" in encoded
    assert "name" in encoded
    assert "detections" in encoded
    assert "last_seen" in encoded

    assert encoded["id"] == first_object.track_id
    assert encoded["name"] == first_object.name
    assert encoded["detections"] == len(first_object.detections)
    assert encoded["last_seen"].endswith("s ago")


def test_object3d_image_property(first_object):
    """Test image property returns best_detection's image."""
    assert first_object.image is not None
    assert first_object.image is first_object.best_detection.image


def test_object3d_addition(object_db_module):
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
    assert len(combined.detections) == 2

    # The combined object should have properties from both detections
    assert det1 in combined.detections
    assert det2 in combined.detections

    # Best detection should be determined by the Object3D logic
    assert combined.best_detection is not None

    # Center should be valid (no specific value check since we're using real detections)
    assert hasattr(combined, "center")
    assert combined.center is not None


def test_image_detections3d_scene_update(object_db_module):
    """Test ImageDetections3D to Foxglove scene update conversion."""
    # Get some detections
    objects = list(object_db_module.objects.values())
    if not objects:
        pytest.skip("No objects in database")

    detections = [obj.best_detection for obj in objects[:3]]  # Take up to 3

    image_detections = ImageDetections3D(image=detections[0].image, detections=detections)

    scene_update = image_detections.to_foxglove_scene_update()

    assert scene_update is not None
    assert scene_update.entities_length == len(detections)

    for i, entity in enumerate(scene_update.entities):
        assert entity.id == str(detections[i].track_id)
        assert entity.frame_id == detections[i].frame_id
        assert entity.cubes_length == 1
        assert entity.texts_length == 1
