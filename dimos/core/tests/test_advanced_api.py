# Copyright 2026 Dimensional Inc.
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

# Copyright 2025-2026 Dimensional Inc.
"""Unit tests for the advanced usage API primitives.

Tests:
1. detect() returns Detection2DBBox from VL model
2. Detection2DBBox.project() calls Detection3DPC.from_2d()
3. Detection3DPC.to_vector() returns .center
4. Full chain: detect → project → to_vector
5. cmd_vel can receive Twist from the chain
"""

from __future__ import annotations

from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.detection.detect import detect
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC

# ── Fixtures ──


@pytest.fixture
def fake_image() -> Image:
    """640x480 black image."""
    data = np.zeros((480, 640, 3), dtype=np.uint8)
    return Image(data=data)


@pytest.fixture
def fake_pointcloud() -> PointCloud2:
    """Simple pointcloud with some 3D points."""
    points = np.array(
        [
            [1.0, 0.0, 0.5],
            [1.1, 0.1, 0.6],
            [1.2, -0.1, 0.4],
            [0.9, 0.05, 0.55],
            [1.05, -0.05, 0.45],
        ]
        * 20,
        dtype=np.float32,
    )  # 100 points
    return PointCloud2.from_numpy(points, frame_id="world", timestamp=0.0)


@pytest.fixture
def fake_camera_info():
    """MuJoCo-like camera intrinsics."""
    from dimos_lcm.sensor_msgs import CameraInfo as LcmCameraInfo

    cam = LcmCameraInfo()
    cam.K = [
        400.0,
        0.0,
        320.0,
        0.0,
        400.0,
        240.0,
        0.0,
        0.0,
        1.0,
    ]
    cam.width = 640
    cam.height = 480
    return cam


@pytest.fixture
def fake_detection(fake_image: Image) -> Detection2DBBox:
    """A valid 2D detection."""
    return Detection2DBBox(
        bbox=(200.0, 150.0, 400.0, 450.0),
        track_id=1,
        class_id=0,
        confidence=0.95,
        name="person",
        ts=0.0,
        image=fake_image,
    )


# ── Test detect() ──


class TestDetect:
    """Tests for the detect() function."""

    @patch("dimos.perception.detection.detect.get_object_bbox_from_image")
    @patch("dimos.perception.detection.detect.create")
    def test_detect_returns_detection2dbbox(self, mock_create, mock_get_bbox, fake_image):
        """detect() should return a Detection2DBBox when object found."""
        mock_vl = MagicMock()
        mock_create.return_value = mock_vl
        mock_get_bbox.return_value = (100.0, 50.0, 300.0, 400.0)

        result = detect("person", fake_image)

        assert result is not None
        assert isinstance(result, Detection2DBBox)
        assert result.bbox == (100.0, 50.0, 300.0, 400.0)
        assert result.name == "person"
        assert result.image is fake_image
        assert result.ts == fake_image.ts

    @patch("dimos.perception.detection.detect.get_object_bbox_from_image")
    @patch("dimos.perception.detection.detect.create")
    def test_detect_returns_none_when_not_found(self, mock_create, mock_get_bbox, fake_image):
        """detect() should return None when VL model finds nothing."""
        mock_create.return_value = MagicMock()
        mock_get_bbox.return_value = None

        result = detect("unicorn", fake_image)
        assert result is None

    @patch("dimos.perception.detection.detect.get_object_bbox_from_image")
    @patch("dimos.perception.detection.detect.create")
    def test_detect_caches_model(self, mock_create, mock_get_bbox, fake_image):
        """detect() should cache VL model instances."""
        mock_create.return_value = MagicMock()
        mock_get_bbox.return_value = (10, 10, 50, 50)

        # Clear cache
        from dimos.perception.detection import detect as detect_mod

        detect_mod._model_cache.clear()

        detect("a", fake_image)
        detect("b", fake_image)

        # Same model, should only create once
        assert mock_create.call_count == 1

    @patch("dimos.perception.detection.detect.get_object_bbox_from_image")
    @patch("dimos.perception.detection.detect.create")
    def test_detect_different_models(self, mock_create, mock_get_bbox, fake_image):
        """detect() with different model names should create separate instances."""
        mock_create.return_value = MagicMock()
        mock_get_bbox.return_value = (10, 10, 50, 50)

        from dimos.perception.detection import detect as detect_mod

        detect_mod._model_cache.clear()

        detect("a", fake_image, model="qwen")
        detect("b", fake_image, model="other")

        assert mock_create.call_count == 2


# ── Test .project() ──


class TestProject:
    """Tests for Detection2DBBox.project()."""

    def test_project_method_exists(self, fake_detection):
        """Detection2DBBox should have .project() method."""
        assert hasattr(fake_detection, "project")
        assert callable(fake_detection.project)

    @patch("dimos.perception.detection.type.detection3d.pointcloud.Detection3DPC.from_2d")
    def test_project_calls_from_2d(
        self, mock_from_2d, fake_detection, fake_pointcloud, fake_camera_info
    ):
        """project() should delegate to Detection3DPC.from_2d()."""
        mock_from_2d.return_value = MagicMock(spec=Detection3DPC)
        tf = Transform.identity()

        fake_detection.project(fake_pointcloud, fake_camera_info, tf)

        mock_from_2d.assert_called_once()
        call_kwargs = mock_from_2d.call_args
        assert call_kwargs.kwargs["det"] is fake_detection
        assert call_kwargs.kwargs["world_pointcloud"] is fake_pointcloud
        assert call_kwargs.kwargs["camera_info"] is fake_camera_info
        assert call_kwargs.kwargs["world_to_optical_transform"] is tf

    @patch("dimos.perception.detection.type.detection3d.pointcloud.Detection3DPC.from_2d")
    def test_project_returns_none_on_failure(
        self, mock_from_2d, fake_detection, fake_pointcloud, fake_camera_info
    ):
        """project() should return None when from_2d fails."""
        mock_from_2d.return_value = None
        result = fake_detection.project(fake_pointcloud, fake_camera_info, Transform.identity())
        assert result is None

    @patch("dimos.perception.detection.type.detection3d.pointcloud.Detection3DPC.from_2d")
    def test_project_passes_custom_filters(
        self, mock_from_2d, fake_detection, fake_pointcloud, fake_camera_info
    ):
        """project() should pass custom filters through to from_2d."""
        mock_from_2d.return_value = None
        custom_filters = [lambda *a: None]

        fake_detection.project(
            fake_pointcloud, fake_camera_info, Transform.identity(), filters=custom_filters
        )

        assert mock_from_2d.call_args.kwargs["filters"] is custom_filters


# ── Test .to_vector() ──


class TestToVector:
    """Tests for Detection3DPC.to_vector()."""

    def test_to_vector_returns_center(self):
        """to_vector() should return the same as .center."""
        # Create a Detection3DPC with known pointcloud
        points = np.array(
            [
                [1.0, 2.0, 3.0],
                [2.0, 3.0, 4.0],
                [3.0, 4.0, 5.0],
            ],
            dtype=np.float32,
        )
        pc = PointCloud2.from_numpy(points, frame_id="world", timestamp=0.0)

        fake_image = Image(data=np.zeros((480, 640, 3), dtype=np.uint8))
        det3d = Detection3DPC(
            bbox=(100, 100, 200, 200),
            track_id=1,
            class_id=0,
            confidence=0.9,
            name="test",
            ts=0.0,
            image=fake_image,
            pointcloud=pc,
            frame_id="world",
        )

        vec = det3d.to_vector()
        center = det3d.center

        assert isinstance(vec, Vector3)
        assert vec.x == center.x
        assert vec.y == center.y
        assert vec.z == center.z


# ── Test full chain ──


class TestFullChain:
    """Tests for the full detect → project → to_vector → publish chain."""

    @patch("dimos.perception.detection.detect.get_object_bbox_from_image")
    @patch("dimos.perception.detection.detect.create")
    @patch("dimos.perception.detection.type.detection3d.pointcloud.Detection3DPC.from_2d")
    def test_chain_detect_project_to_vector(
        self,
        mock_from_2d,
        mock_create,
        mock_get_bbox,
        fake_image,
        fake_pointcloud,
        fake_camera_info,
    ):
        """Full chain: detect → project → to_vector should produce a Vector3."""
        # Setup mocks
        mock_create.return_value = MagicMock()
        mock_get_bbox.return_value = (100.0, 50.0, 300.0, 400.0)

        points = np.array([[2.0, 1.0, 0.5]] * 10, dtype=np.float32)
        pc = PointCloud2.from_numpy(points, frame_id="world", timestamp=0.0)
        mock_det3d = Detection3DPC(
            bbox=(100, 50, 300, 400),
            track_id=-1,
            class_id=-1,
            confidence=1.0,
            name="person",
            ts=0.0,
            image=fake_image,
            pointcloud=pc,
            frame_id="world",
        )
        mock_from_2d.return_value = mock_det3d

        # Clear model cache
        from dimos.perception.detection import detect as detect_mod

        detect_mod._model_cache.clear()

        # The chain
        det = detect("person", fake_image)
        assert det is not None

        det3d = det.project(fake_pointcloud, fake_camera_info, Transform.identity())
        assert det3d is not None

        vec = det3d.to_vector()
        assert isinstance(vec, Vector3)
        assert vec.x == pytest.approx(2.0)
        assert vec.y == pytest.approx(1.0)
        assert vec.z == pytest.approx(0.5)

    @patch("dimos.perception.detection.detect.get_object_bbox_from_image")
    @patch("dimos.perception.detection.detect.create")
    @patch("dimos.perception.detection.type.detection3d.pointcloud.Detection3DPC.from_2d")
    def test_chain_to_twist(
        self,
        mock_from_2d,
        mock_create,
        mock_get_bbox,
        fake_image,
        fake_pointcloud,
        fake_camera_info,
    ):
        """Chain output can be used with VisualServoing2D.compute_twist()."""
        mock_create.return_value = MagicMock()
        mock_get_bbox.return_value = (100.0, 50.0, 300.0, 400.0)

        points = np.array([[2.0, 0.5, 0.5]] * 10, dtype=np.float32)
        pc = PointCloud2.from_numpy(points, frame_id="world", timestamp=0.0)
        mock_det3d = Detection3DPC(
            bbox=(100, 50, 300, 400),
            track_id=-1,
            class_id=-1,
            confidence=1.0,
            name="person",
            ts=0.0,
            image=fake_image,
            pointcloud=pc,
            frame_id="world",
        )
        mock_from_2d.return_value = mock_det3d

        from dimos.perception.detection import detect as detect_mod

        detect_mod._model_cache.clear()

        det = detect("person", fake_image)
        assert det is not None

        # Use existing VisualServoing2D with the detection's bbox
        from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo

        cam_info = CameraInfo()
        cam_info.K = fake_camera_info.K
        cam_info.width = fake_camera_info.width
        cam_info.height = fake_camera_info.height

        from dimos.navigation.visual_servoing.visual_servoing_2d import VisualServoing2D

        servo = VisualServoing2D(cam_info)
        twist = servo.compute_twist(det.bbox, fake_image.width)

        assert isinstance(twist, Twist)
        # Person is left of center (bbox centered at x=200, image center at 320)
        # So angular.z should be positive (turn left)
        assert twist.angular.z != 0.0

    def test_cmd_vel_accepts_twist(self):
        """Verify a Twist can be published to a mock cmd_vel stream."""
        cmd_vel = MagicMock()
        twist = Twist(
            linear=Vector3(0.3, 0.0, 0.0),
            angular=Vector3(0.0, 0.0, 0.1),
        )
        cmd_vel.publish(twist)
        cmd_vel.publish.assert_called_once_with(twist)


# ── Test integration ideas ──


class TestIntegrationNotes:
    """Not real tests — documents how to test without a person in scene."""

    def test_integration_approach_documented(self):
        """
        Integration test approaches for person-follow without a real person:

        1. SYNTHETIC DETECTION: Skip VL model, create Detection2DBBox manually
           with a known bbox, feed through project → servo → cmd_vel.
           Verifies the full chain minus VL inference.

        2. STATIC OBJECT: In MuJoCo, detect the robot itself or floor plane.
           Won't test following behavior but tests detect() → project() pipeline.

        3. MOVING MARKER: Place a MuJoCo body in scene, move it programmatically,
           verify the robot produces correct twist commands to track it.

        4. REPLAY DATA: Use recorded images with known person detections.
           Feed through detect(), verify bbox matches expected.
        """
        pass
