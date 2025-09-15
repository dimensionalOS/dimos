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

import os
import pickle

import pytest

from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection2d.type import Detection2D, Detection3D
from dimos.perception.detection2d.yolo_2d_det import Yolo2DDetector
from dimos.utils.data import get_data


@pytest.fixture
def loaded_detection3d():
    """Fixture to load a pickled Detection3D object."""
    TEST_DIR = os.path.dirname(__file__)
    detection3d_pkl = os.path.join(TEST_DIR, "detection3d.pkl")

    # Check if pickle file exists
    if not os.path.exists(detection3d_pkl):
        pytest.skip("Detection3D pickle file not found. Run test_module.py::test_basic first.")

    # Load the pickled Detection3D
    with open(detection3d_pkl, "rb") as f:
        detection3d = pickle.load(f)

    return detection3d


def test_detection2dtype():
    detector = Yolo2DDetector()

    image = Image.from_file(get_data("cafe.jpg"))
    raw_detections = detector.process_image(image.to_opencv())

    detections = Detection2D.from_detector(raw_detections, image=image)

    for det in detections:
        print(det)


def test_load_detection3d(loaded_detection3d):
    """Test loading a pickled Detection3D object."""
    detection3d = loaded_detection3d

    # Verify it's a Detection3D object
    assert isinstance(detection3d, Detection3D)
    assert hasattr(detection3d, "bbox")
    assert hasattr(detection3d, "pointcloud")
    assert hasattr(detection3d, "transform")
    assert hasattr(detection3d, "image")

    # Print detection info
    print(f"\nLoaded Detection3D: {detection3d}")
    print(f"Name: {detection3d.name}")
    print(f"Confidence: {detection3d.confidence}")
    print(f"Bbox: {detection3d.bbox}")
    print(f"Track ID: {detection3d.track_id}")
    print(f"Class ID: {detection3d.class_id}")

    # Verify pointcloud
    if detection3d.pointcloud:
        num_points = len(detection3d.pointcloud.pointcloud.points)
        print(f"Points in pointcloud: {num_points}")
        assert num_points > 0

    # Verify transform
    if detection3d.transform:
        print(f"Transform: {detection3d.transform}")

    # Verify image
    if detection3d.image:
        print(f"Image shape: {detection3d.image.shape}")
        print(f"Image frame_id: {detection3d.image.frame_id}")

    # Test conversion methods
    annotations = detection3d.to_imageannotations()
    assert annotations is not None
    print(f"Converted to ImageAnnotations successfully")

    ros_det = detection3d.to_ros_detection2d()
    assert ros_det is not None
    print(f"Converted to ROS Detection2D successfully")

    # Test that Detection3D inherits from Detection2D
    assert isinstance(detection3d, Detection2D)

    # Test get_bbox_center method
    center = detection3d.get_bbox_center()
    print(f"Bbox center: {center}")


def test_to_ros_detection2d(loaded_detection3d):
    """Test converting Detection3D to ROS Detection2D message."""
    detection3d = loaded_detection3d

    # Convert to ROS Detection2D
    ros_det = detection3d.to_ros_detection2d()

    # Verify the conversion produced a valid object
    assert ros_det is not None

    # Check header
    assert hasattr(ros_det, "header")
    # Convert ROS time back to float for comparison
    from dimos.types.timestamped import to_timestamp

    assert abs(to_timestamp(ros_det.header.stamp) - detection3d.ts) < 0.001
    assert ros_det.header.frame_id == "camera_link"

    # Check bounding box
    assert hasattr(ros_det, "bbox")
    bbox = ros_det.bbox

    # Verify bbox has correct structure
    assert hasattr(bbox, "center")
    assert hasattr(bbox.center, "position")
    assert hasattr(bbox.center.position, "x")
    assert hasattr(bbox.center.position, "y")
    assert hasattr(bbox, "size_x")
    assert hasattr(bbox, "size_y")

    # Calculate expected center from original bbox
    x1, y1, x2, y2 = detection3d.bbox
    expected_center_x = (x1 + x2) / 2.0
    expected_center_y = (y1 + y2) / 2.0
    expected_width = x2 - x1
    expected_height = y2 - y1

    # Verify bbox values
    assert abs(bbox.center.position.x - expected_center_x) < 0.001
    assert abs(bbox.center.position.y - expected_center_y) < 0.001
    assert abs(bbox.size_x - expected_width) < 0.001
    assert abs(bbox.size_y - expected_height) < 0.001

    # Check results (hypotheses)
    assert hasattr(ros_det, "results")
    assert len(ros_det.results) == 1

    hypothesis = ros_det.results[0]
    assert hasattr(hypothesis, "hypothesis")
    assert hypothesis.hypothesis.class_id == detection3d.class_id
    assert hypothesis.hypothesis.score == detection3d.confidence

    # Check ID
    assert hasattr(ros_det, "id")
    assert ros_det.id == str(detection3d.track_id)

    print(f"\nSuccessfully converted Detection3D to ROS Detection2D")
    print(f"Header timestamp: {to_timestamp(ros_det.header.stamp)}")
    print(f"Header frame_id: {ros_det.header.frame_id}")
    print(f"Bbox center: ({bbox.center.position.x}, {bbox.center.position.y})")
    print(f"Bbox size: {bbox.size_x} x {bbox.size_y}")
    print(f"Class ID: {hypothesis.hypothesis.class_id}")
    print(f"Confidence: {hypothesis.hypothesis.score}")
    print(f"Track ID: {ros_det.id}")


def test_from_ros_detection2d(loaded_detection3d):
    """Test converting from ROS Detection2D message back to Detection2D object."""
    # First convert to ROS format
    ros_det = loaded_detection3d.to_ros_detection2d()

    # Then convert back to Detection2D
    detection2d_converted = Detection2D.from_ros_detection2d(ros_det, name=loaded_detection3d.name)

    # Verify all fields match
    assert isinstance(detection2d_converted, Detection2D)

    # Check bbox (allowing small floating point differences)
    for i in range(4):
        assert abs(detection2d_converted.bbox[i] - loaded_detection3d.bbox[i]) < 0.001

    # Check other fields
    assert detection2d_converted.track_id == loaded_detection3d.track_id
    assert detection2d_converted.class_id == loaded_detection3d.class_id
    assert abs(detection2d_converted.confidence - loaded_detection3d.confidence) < 0.001
    assert detection2d_converted.ts == loaded_detection3d.ts
    assert detection2d_converted.name == loaded_detection3d.name

    print(f"\nSuccessfully converted ROS Detection2D back to Detection2D")
    print(f"Original bbox: {loaded_detection3d.bbox}")
    print(f"Converted bbox: {detection2d_converted.bbox}")
    print(f"Track ID: {detection2d_converted.track_id}")
    print(f"Class ID: {detection2d_converted.class_id}")
    print(f"Confidence: {detection2d_converted.confidence}")
    print(f"Timestamp: {detection2d_converted.ts}")
    print(f"Name: {detection2d_converted.name}")


def test_roundtrip_conversion():
    """Test roundtrip conversion: Detection2D -> ROS -> Detection2D."""
    # Create a Detection2D object
    original = Detection2D(
        bbox=(100.0, 200.0, 300.0, 400.0),
        track_id=42,
        class_id=3,
        confidence=0.95,
        name="test_object",
        ts=1234567890.123,
    )

    # Convert to ROS
    ros_det = original.to_ros_detection2d()

    # Convert back
    converted = Detection2D.from_ros_detection2d(ros_det, name=original.name)

    # Verify roundtrip
    assert converted.bbox == original.bbox
    assert converted.track_id == original.track_id
    assert converted.class_id == original.class_id
    assert converted.confidence == original.confidence
    assert converted.ts == original.ts
    assert converted.name == original.name

    print("\nRoundtrip conversion successful!")
    print(f"Original: {original}")
    print(f"Converted: {converted}")
