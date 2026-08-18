# Copyright 2025-2026 Dimensional Inc.
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

from typing import Any

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg
from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC

pytestmark = pytest.mark.self_hosted


def _projection_context() -> tuple[Image, CameraInfo, Transform]:
    image = Image.from_numpy(np.zeros((10, 10, 3), dtype=np.uint8), ts=3.0)
    camera_info = CameraInfo.from_intrinsics(
        1.0,
        1.0,
        0.0,
        0.0,
        image.width,
        image.height,
        frame_id="camera_optical",
    )
    world_to_optical = Transform(
        frame_id="camera_optical",
        child_frame_id="world",
        ts=image.ts,
    )
    return image, camera_info, world_to_optical


def test_from_2d_selects_bbox_points_and_preserves_cloud_metadata() -> None:
    image, camera_info, world_to_optical = _projection_context()
    detection = Detection2DBBox(
        bbox=(0.0, 0.0, 1.0, 1.0),
        track_id=4,
        class_id=2,
        confidence=0.9,
        name="object",
        ts=image.ts,
        image=image,
    )
    cloud = PointCloud2.from_numpy(
        np.asarray(((1.0, 1.0, 1.0), (6.0, 6.0, 1.0), (8.0, 8.0, 1.0))),
        frame_id="world",
        timestamp=2.0,
    )

    result = Detection3DPC.from_2d(
        detection,
        cloud,
        camera_info,
        world_to_optical,
        filters=[],
    )

    assert result is not None
    points, _ = result.pointcloud.as_numpy()
    assert np.allclose(points, ((1.0, 1.0, 1.0), (6.0, 6.0, 1.0)))
    assert result.pointcloud.frame_id == "world"
    assert result.pointcloud.ts == 2.0
    assert result.ts == image.ts


def test_from_2d_prefers_segmentation_mask_over_bbox() -> None:
    image, camera_info, world_to_optical = _projection_context()
    mask = np.zeros((image.height, image.width), dtype=np.uint8)
    mask[2, 2] = 255
    detection = Detection2DSeg(
        bbox=(0.0, 0.0, 9.0, 9.0),
        track_id=0,
        class_id=0,
        confidence=1.0,
        name="object",
        ts=image.ts,
        image=image,
        mask=mask,
    )
    cloud = PointCloud2.from_numpy(
        np.asarray(((1.0, 1.0, 1.0), (2.0, 2.0, 1.0))),
        timestamp=2.0,
    )

    result = Detection3DPC.from_2d(
        detection,
        cloud,
        camera_info,
        world_to_optical,
        filters=[],
    )

    assert result is not None
    points, _ = result.pointcloud.as_numpy()
    assert np.allclose(points, ((2.0, 2.0, 1.0),))


@pytest.mark.skipif_macos_bug
def test_detection3dpc(detection3dpc: Any) -> None:
    # def test_oriented_bounding_box(detection3dpc):
    """Test oriented bounding box calculation and values."""
    obb = detection3dpc.get_oriented_bounding_box()
    assert obb is not None, "Oriented bounding box should not be None"

    # Verify OBB center values
    assert obb.center[0] == pytest.approx(-3.36002, abs=0.1)
    assert obb.center[1] == pytest.approx(-0.196446, abs=0.1)
    assert obb.center[2] == pytest.approx(0.220184, abs=0.1)

    # Verify OBB extent values
    assert obb.extent[0] == pytest.approx(0.531275, abs=0.12)
    assert obb.extent[1] == pytest.approx(0.461054, abs=0.1)
    assert obb.extent[2] == pytest.approx(0.155, abs=0.1)

    # def test_bounding_box_dimensions(detection3dpc):
    """Test bounding box dimension calculation."""
    dims = detection3dpc.get_bounding_box_dimensions()
    assert len(dims) == 3, "Bounding box dimensions should have 3 values"
    assert dims[0] == pytest.approx(0.350, abs=0.1)
    assert dims[1] == pytest.approx(0.250, abs=0.1)
    assert dims[2] == pytest.approx(0.550, abs=0.1)

    # def test_axis_aligned_bounding_box(detection3dpc):
    """Test axis-aligned bounding box calculation."""
    aabb = detection3dpc.get_bounding_box()
    assert aabb is not None, "Axis-aligned bounding box should not be None"

    # Verify AABB min values
    assert aabb.min_bound[0] == pytest.approx(-3.575, abs=0.2)
    assert aabb.min_bound[1] == pytest.approx(-0.375, abs=0.2)
    assert aabb.min_bound[2] == pytest.approx(-0.075, abs=0.2)

    # Verify AABB max values
    assert aabb.max_bound[0] == pytest.approx(-3.075, abs=0.2)
    assert aabb.max_bound[1] == pytest.approx(-0.125, abs=0.2)
    assert aabb.max_bound[2] == pytest.approx(0.475, abs=0.2)

    # def test_point_cloud_properties(detection3dpc):
    """Test point cloud data and boundaries."""
    points, _ = detection3dpc.pointcloud.as_numpy()
    assert len(points) > 60
    assert detection3dpc.pointcloud.frame_id == "world", (
        f"Expected frame_id 'world', got '{detection3dpc.pointcloud.frame_id}'"
    )

    min_pt = np.min(points, axis=0)
    max_pt = np.max(points, axis=0)
    center = np.mean(points, axis=0)

    # Verify point cloud boundaries
    assert min_pt[0] == pytest.approx(-3.575, abs=0.2)
    assert min_pt[1] == pytest.approx(-0.375, abs=0.2)
    assert min_pt[2] == pytest.approx(-0.075, abs=0.2)

    assert max_pt[0] == pytest.approx(-3.075, abs=0.2)
    assert max_pt[1] == pytest.approx(-0.125, abs=0.2)
    assert max_pt[2] == pytest.approx(0.475, abs=0.2)

    assert center[0] == pytest.approx(-3.326, abs=0.1)
    assert center[1] == pytest.approx(-0.202, abs=0.1)
    assert center[2] == pytest.approx(0.160, abs=0.1)

    # def test_detection_pose(detection3dpc):
    """Test detection pose and frame information."""
    assert detection3dpc.pose.x == pytest.approx(-3.327, abs=0.1)
    assert detection3dpc.pose.y == pytest.approx(-0.202, abs=0.1)
    assert detection3dpc.pose.z == pytest.approx(0.160, abs=0.1)
    assert detection3dpc.pose.frame_id == "world", (
        f"Expected frame_id 'world', got '{detection3dpc.pose.frame_id}'"
    )
