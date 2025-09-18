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

from dimos.perception.detection2d.conftest import detections2d, detections3d
from dimos.perception.detection2d.type import (
    Detection2D,
    Detection3D,
    ImageDetections2D,
    ImageDetections3D,
)
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.msgs.geometry_msgs import Transform, Vector3, PoseStamped


def test_detections2d(detections2d):
    print(f"\n=== ImageDetections2D Test ===")
    print(f"Type: {type(detections2d)}")
    print(f"Number of detections: {len(detections2d)}")
    print(f"Image timestamp: {detections2d.image.ts}")
    print(f"Image shape: {detections2d.image.shape}")
    print(f"Image frame_id: {detections2d.image.frame_id}")

    print(f"\nFull detections object:")
    print(detections2d)

    # Basic type assertions
    assert isinstance(detections2d, ImageDetections2D)
    assert hasattr(detections2d, "image")
    assert hasattr(detections2d, "detections")

    # Image assertions
    assert detections2d.image is not None
    assert detections2d.image.ts == 1757960670.490248
    assert detections2d.image.shape == (720, 1280, 3)
    assert detections2d.image.frame_id == "camera_optical"

    # Detection count assertions
    assert len(detections2d) == 1
    assert isinstance(detections2d.detections, list)
    assert len(detections2d.detections) == 1

    # Test first detection with literal checks
    det = detections2d.detections[0]
    print(f"\n--- Detection 0 (literal checks) ---")
    print(f"Type: {type(det)}")
    print(f"Name: {det.name}")
    print(f"Class ID: {det.class_id}")
    print(f"Track ID: {det.track_id}")
    print(f"Confidence: {det.confidence}")
    print(f"Bbox: {det.bbox}")
    print(f"Timestamp: {det.ts}")

    # Detection type assertions
    assert isinstance(det, Detection2D)

    # Literal value assertions
    assert det.name == "suitcase"
    assert det.class_id == 28  # COCO class 28 is suitcase
    assert det.track_id == 1
    assert 0.814 < det.confidence < 0.815  # Allow small floating point variance

    # Data type assertions
    assert isinstance(det.name, str)
    assert isinstance(det.class_id, int)
    assert isinstance(det.track_id, int)
    assert isinstance(det.confidence, float)
    assert isinstance(det.bbox, (tuple, list)) and len(det.bbox) == 4
    assert isinstance(det.ts, float)

    # Bbox literal checks (with tolerance for float precision)
    x1, y1, x2, y2 = det.bbox
    assert 503.4 < x1 < 503.5
    assert 249.8 < y1 < 250.0
    assert 655.9 < x2 < 656.0
    assert 469.8 < y2 < 470.0

    # Bbox format assertions
    assert all(isinstance(coord, (int, float)) for coord in det.bbox)
    assert x2 > x1, f"x2 ({x2}) should be greater than x1 ({x1})"
    assert y2 > y1, f"y2 ({y2}) should be greater than y1 ({y1})"
    assert x1 >= 0 and y1 >= 0, "Bbox coordinates should be non-negative"

    # Bbox width/height checks
    width = x2 - x1
    height = y2 - y1
    assert 152.0 < width < 153.0  # Expected width ~152.5
    assert 219.0 < height < 221.0  # Expected height ~219.9

    # Confidence assertions
    assert 0.0 <= det.confidence <= 1.0, (
        f"Confidence should be between 0 and 1, got {det.confidence}"
    )

    # Image reference assertion
    assert det.image is detections2d.image, "Detection should reference the same image"

    # Timestamp consistency
    assert det.ts == detections2d.image.ts
    assert det.ts == 1757960670.490248


def test_detections3d(detections3d):
    print(f"\n=== ImageDetections3D Test ===")
    print(f"Type: {type(detections3d)}")
    print(f"Number of detections: {len(detections3d)}")
    print(f"Image timestamp: {detections3d.image.ts}")
    print(f"Image shape: {detections3d.image.shape}")
    print(f"Image frame_id: {detections3d.image.frame_id}")

    print(f"\nFull detections object:")
    print(detections3d)

    # Basic type assertions
    assert isinstance(detections3d, ImageDetections3D)
    assert hasattr(detections3d, "image")
    assert hasattr(detections3d, "detections")

    # Image assertions
    assert detections3d.image is not None
    assert detections3d.image.ts == 1757960670.490248
    assert detections3d.image.shape == (720, 1280, 3)
    assert detections3d.image.frame_id == "camera_optical"

    # Detection count assertions
    assert len(detections3d) == 1
    assert isinstance(detections3d.detections, list)
    assert len(detections3d.detections) == 1

    # Test first 3D detection with literal checks
    det = detections3d.detections[0]
    print(f"\n--- Detection3D 0 (literal checks) ---")
    print(f"Type: {type(det)}")
    print(f"Name: {det.name}")
    print(f"Class ID: {det.class_id}")
    print(f"Track ID: {det.track_id}")
    print(f"Confidence: {det.confidence}")
    print(f"Bbox: {det.bbox}")
    print(f"Timestamp: {det.ts}")
    print(f"Has pointcloud: {hasattr(det, 'pointcloud')}")
    print(f"Has transform: {hasattr(det, 'transform')}")
    if hasattr(det, "pointcloud"):
        print(f"Pointcloud points: {len(det.pointcloud)}")
        print(f"Pointcloud frame_id: {det.pointcloud.frame_id}")

    # Detection type assertions
    assert isinstance(det, Detection3D)

    # Detection3D should have all Detection2D fields plus pointcloud and transform
    assert hasattr(det, "bbox")
    assert hasattr(det, "track_id")
    assert hasattr(det, "class_id")
    assert hasattr(det, "confidence")
    assert hasattr(det, "name")
    assert hasattr(det, "ts")
    assert hasattr(det, "image")
    assert hasattr(det, "pointcloud")
    assert hasattr(det, "transform")

    # Literal value assertions (should match Detection2D)
    assert det.name == "suitcase"
    assert det.class_id == 28  # COCO class 28 is suitcase
    assert det.track_id == 1
    assert 0.814 < det.confidence < 0.815  # Allow small floating point variance

    # Data type assertions
    assert isinstance(det.name, str)
    assert isinstance(det.class_id, int)
    assert isinstance(det.track_id, int)
    assert isinstance(det.confidence, float)
    assert isinstance(det.bbox, (tuple, list)) and len(det.bbox) == 4
    assert isinstance(det.ts, float)

    # Bbox literal checks (should match Detection2D)
    x1, y1, x2, y2 = det.bbox
    assert 503.4 < x1 < 503.5
    assert 249.8 < y1 < 250.0
    assert 655.9 < x2 < 656.0
    assert 469.8 < y2 < 470.0

    # 3D-specific assertions
    assert isinstance(det.pointcloud, PointCloud2)
    assert isinstance(det.transform, Transform)

    # Pointcloud assertions
    assert len(det.pointcloud) == 81  # Based on the output we saw
    assert det.pointcloud.frame_id == "world"  # Pointcloud should be in world frame

    # Test center calculation
    center = det.center
    print(f"\nDetection center: {center}")
    assert isinstance(center, Vector3)
    assert hasattr(center, "x")
    assert hasattr(center, "y")
    assert hasattr(center, "z")

    # Test pose property
    pose = det.pose
    print(f"Detection pose: {pose}")
    assert isinstance(pose, PoseStamped)
    assert pose.frame_id == "world"
    assert pose.ts == det.ts
    assert pose.position == center  # Pose position should match center

    # Check distance calculation (from to_repr_dict)
    repr_dict = det.to_repr_dict()
    print(f"\nRepr dict: {repr_dict}")
    assert "dist" in repr_dict
    assert repr_dict["dist"] == "0.88m"  # Based on the output
    assert repr_dict["points"] == "81"
    assert repr_dict["name"] == "suitcase"
    assert repr_dict["class"] == "28"
    assert repr_dict["track"] == "1"

    # Image reference assertion
    assert det.image is detections3d.image, "Detection should reference the same image"

    # Timestamp consistency
    assert det.ts == detections3d.image.ts
    assert det.ts == 1757960670.490248


def test_detection3d_to_pose(detections3d):
    det = detections3d[0]
    pose = det.pose

    # Check that pose is valid
    assert pose.frame_id == "world"
    assert pose.ts == det.ts
