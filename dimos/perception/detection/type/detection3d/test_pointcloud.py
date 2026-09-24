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

import cv2
import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg
from dimos.perception.detection.type.detection3d.imageDetections3DPC import ImageDetections3DPC
from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC, lattice_quantum
from dimos.perception.detection.type.detection3d.pointcloud_filters import range_cluster

pytestmark = pytest.mark.self_hosted

PITCH = 0.05
K = [100.0, 0.0, 320.0, 0.0, 100.0, 240.0, 0.0, 0.0, 1.0]
CAMERA_INFO = CameraInfo(height=480, width=640, K=K)


def _image() -> Image:
    return Image(
        data=np.zeros((480, 640, 3), dtype=np.uint8),
        format=ImageFormat.BGR,
        frame_id="camera_optical",
        ts=1.0,
    )


def _lattice() -> np.ndarray:
    x, y = np.meshgrid(np.arange(-10, 11) * PITCH, np.arange(-10, 11) * PITCH)
    return np.column_stack((x.ravel(), y.ravel(), np.ones(x.size)))


def _sighting(ts: float, confidence: float, name: str, points: int) -> Detection3DPC:
    return Detection3DPC(
        image=_image(),
        bbox=(ts, ts, ts + 10.0, ts + 10.0),
        track_id=0,
        class_id=0,
        confidence=confidence,
        name=name,
        ts=ts,
        pointcloud=PointCloud2.from_numpy(np.full((points, 3), ts), frame_id="world", timestamp=ts),
        frame_id="world",
    )


@pytest.mark.skipif_macos_bug
def test_detection3dpc(detection3dpc) -> None:
    # def test_oriented_bounding_box(detection3dpc):
    """Test oriented bounding box calculation and values."""
    obb = detection3dpc.get_oriented_bounding_box()
    assert obb is not None, "Oriented bounding box should not be None"

    # Verify OBB center values
    assert obb.center[0] == pytest.approx(-3.316207, abs=0.1)
    assert obb.center[1] == pytest.approx(-0.300175, abs=0.1)
    assert obb.center[2] == pytest.approx(0.240114, abs=0.1)

    # Verify OBB extent values
    assert obb.extent[0] == pytest.approx(0.593476, abs=0.12)
    assert obb.extent[1] == pytest.approx(0.470315, abs=0.1)
    assert obb.extent[2] == pytest.approx(0.164996, abs=0.1)

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


@pytest.mark.parametrize(
    ("points", "quantum"),
    [
        (_lattice(), PITCH),
        (np.random.default_rng(0).uniform(-1.0, 1.0, (500, 3)), None),
        (
            np.column_stack((np.arange(500) % 20 * PITCH, np.linspace(0, 1, 1000).reshape(500, 2))),
            None,
        ),
        (_lattice()[:5], None),
    ],
    ids=["lattice", "continuous", "gridded-in-x-only", "too-few-columns"],
)
def test_lattice_quantum(points: np.ndarray, quantum: float | None) -> None:
    # grid pitch for lattice clouds, None otherwise
    assert lattice_quantum(points) == pytest.approx(quantum)


def test_add() -> None:
    # identity from the stronger sighting, time from the later
    strong, later = _sighting(1.0, 0.9, "cup", 4), _sighting(2.0, 0.5, "mug", 6)

    merged = strong + later

    assert len(merged.pointcloud.as_numpy()[0]) == 10
    assert (merged.name, merged.confidence) == ("cup", 0.9)
    assert (merged.ts, merged.bbox) == (2.0, later.bbox)


@pytest.mark.parametrize(
    "D", [[0.0] * 5, [-0.1, 0.05, 0.001, -0.002, 0.01]], ids=["undistorted", "radtan"]
)
def test_project_pixels(D: list[float]) -> None:
    # matches OpenCV with and without distortion
    points = np.array([[0.3, -0.2, 1.0], [-0.6, 0.4, 1.5]])
    info = CameraInfo(height=480, width=640, distortion_model="plumb_bob", D=D, K=K)
    expected, _ = cv2.projectPoints(
        points, np.zeros(3), np.zeros(3), np.reshape(K, (3, 3)), np.array(D)
    )

    np.testing.assert_allclose(Detection3DPC.project_pixels(points, info), expected.reshape(-1, 2))


def test_project_cloud_empty() -> None:
    # nothing in front of the camera
    cloud = PointCloud2.from_numpy(np.array([[0.0, 0.0, -1.0]]), frame_id="world", timestamp=1.0)

    world_points, pixels = Detection3DPC.project_cloud(cloud, CAMERA_INFO, Transform.identity())

    assert len(world_points) == len(pixels) == 0


def test_from_2d_splat() -> None:
    # a mask between cell centers (columns 320 and 325) is hit via the lattice splat
    image = _image()
    mask = np.zeros((480, 640), dtype=np.uint8)
    mask[:, 322] = 255
    stripe = Detection2DSeg(
        bbox=(322.0, 0.0, 323.0, 480.0),
        track_id=0,
        class_id=0,
        confidence=0.9,
        name="stripe",
        ts=image.ts,
        image=image,
        mask=mask,
    )
    cloud = PointCloud2.from_numpy(_lattice(), frame_id="world", timestamp=1.0)

    lifted = ImageDetections3DPC.from_2d(
        ImageDetections2D(image=image, detections=[stripe]),
        cloud,
        CAMERA_INFO,
        Transform.identity(),
        [],
    )

    columns = np.unique(np.round(lifted.detections[0].pointcloud.as_numpy()[0][:, 0], 3))
    np.testing.assert_allclose(columns, [0.0, PITCH])


def test_range_cluster() -> None:
    # keeps the near cluster, drops background past the range gap
    near = np.column_stack((np.zeros((30, 2)), np.linspace(0.9, 1.1, 30)))
    far = np.column_stack((np.linspace(-0.5, 0.5, 10), np.zeros(10), np.full(10, 3.0)))
    det = Detection2DBBox(
        bbox=(0.0, 0.0, 640.0, 480.0),
        track_id=0,
        class_id=0,
        confidence=0.9,
        name="object",
        ts=1.0,
        image=_image(),
    )

    def keep(points: np.ndarray) -> PointCloud2 | None:
        cloud = PointCloud2.from_numpy(points, frame_id="world", timestamp=1.0)
        return range_cluster()(det, cloud, CAMERA_INFO, Transform.identity())

    kept = keep(np.vstack((near, far)))
    assert kept is not None
    np.testing.assert_allclose(np.sort(kept.as_numpy()[0][:, 2]), near[:, 2], rtol=1e-6)
    assert keep(np.empty((0, 3))) is None
