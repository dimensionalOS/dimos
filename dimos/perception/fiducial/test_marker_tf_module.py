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

from unittest.mock import MagicMock, patch

import cv2
from dimos_lcm.vision_msgs import BoundingBox3D, ObjectHypothesis, ObjectHypothesisWithPose
import numpy as np
import pytest

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.std_msgs.Header import Header
from dimos.msgs.vision_msgs.Detection3D import Detection3D
from dimos.msgs.vision_msgs.Detection3DArray import Detection3DArray
from dimos.perception.fiducial.marker_pose import (
    _camera_optical_frame_id,
    estimate_marker_pose,
    marker_corners_to_bbox,
    marker_reprojection_error,
)
from dimos.perception.fiducial.marker_tf_module import (
    MarkerTfModule,
    deploy,
)

pytest.importorskip("cv2.aruco")


@pytest.fixture
def dimos():
    coord = ModuleCoordinator()
    coord.start()
    try:
        yield coord
    finally:
        coord.stop()


def test_deploy_calls_coordinator_deploy_and_wires_streams(dimos) -> None:
    proxy = MagicMock()
    source = MagicMock()
    source.detections = MagicMock()

    with patch.object(dimos, "deploy", return_value=proxy) as mock_deploy:
        result = deploy(dimos, source)

    mock_deploy.assert_called_once_with(
        MarkerTfModule,
        marker_namespace_prefix="marker_tf",
    )
    assert result is proxy
    proxy.detections.connect.assert_called_once_with(source.detections)
    proxy.start.assert_called_once()


def test_deploy_explicit_marker_namespace_prefix_overrides_prefix(dimos) -> None:
    proxy = MagicMock()
    source = MagicMock()
    source.detections = MagicMock()

    with patch.object(dimos, "deploy", return_value=proxy) as mock_deploy:
        deploy(
            dimos,
            source,
            prefix="/ignored",
            marker_namespace_prefix="bot_a",
        )

    mock_deploy.assert_called_once_with(
        MarkerTfModule,
        marker_namespace_prefix="bot_a",
    )
    proxy.detections.connect.assert_called_once_with(source.detections)


def test_deploy_empty_prefix_skips_auto_namespace(dimos) -> None:
    proxy = MagicMock()
    source = MagicMock()
    source.detections = MagicMock()

    with patch.object(dimos, "deploy", return_value=proxy) as mock_deploy:
        deploy(dimos, source, prefix="")

    mock_deploy.assert_called_once_with(
        MarkerTfModule,
        marker_namespace_prefix=None,
    )
    proxy.detections.connect.assert_called_once_with(source.detections)


def test_camera_optical_frame_id_resolution() -> None:
    ts = 1.0
    fx, fy, cx, cy = 600.0, 600.0, 320.0, 240.0
    info_named = CameraInfo.from_intrinsics(fx, fy, cx, cy, 640, 480, frame_id="cam_info_optical")
    info_named.ts = ts
    info_empty = CameraInfo.from_intrinsics(fx, fy, cx, cy, 640, 480)
    info_empty.ts = ts
    img_custom = Image(
        data=np.zeros((480, 640, 3), dtype=np.uint8),
        format=ImageFormat.BGR,
        ts=ts,
        frame_id="custom_optical",
    )
    img_whitespace = Image(
        data=np.zeros((480, 640, 3), dtype=np.uint8),
        format=ImageFormat.BGR,
        ts=ts,
        frame_id="  custom_optical  ",
    )
    img_empty = Image(data=np.zeros((480, 640, 3), dtype=np.uint8), format=ImageFormat.BGR, ts=ts)

    assert _camera_optical_frame_id(img_custom, info_named) == "custom_optical"
    assert _camera_optical_frame_id(img_whitespace, info_named) == "custom_optical"
    assert _camera_optical_frame_id(img_empty, info_named) == "cam_info_optical"
    assert _camera_optical_frame_id(img_empty, info_empty) == "camera_optical"


def test_estimate_marker_pose_roundtrip() -> None:
    marker_length = 0.2
    h = marker_length / 2.0
    obj = np.array(
        [[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]],
        dtype=np.float32,
    )
    k = np.array([[400.0, 0.0, 320.0], [0.0, 400.0, 240.0], [0.0, 0.0, 1.0]])
    dist = np.zeros((5, 1), dtype=np.float64)
    rvec0 = np.array([[0.1], [0.05], [-0.02]], dtype=np.float64)
    tvec0 = np.array([[0.2], [-0.15], [2.5]], dtype=np.float64)
    img_pts, _jac = cv2.projectPoints(obj, rvec0, tvec0, k, dist)
    corners = img_pts.reshape(4, 2).astype(np.float32)
    result = estimate_marker_pose(corners, marker_length, k, dist)
    assert result is not None
    rvec, tvec = result
    np.testing.assert_allclose(rvec.reshape(3), rvec0.reshape(3), atol=1e-3)
    np.testing.assert_allclose(tvec.reshape(3), tvec0.reshape(3), atol=1e-3)
    assert marker_reprojection_error(corners, marker_length, k, dist, rvec, tvec) < 0.01


def test_marker_corners_to_bbox_accepts_aruco_shapes() -> None:
    corners = np.array([[[10.0, 20.0], [50.0, 18.0], [48.0, 60.0], [9.0, 58.0]]])
    assert marker_corners_to_bbox(corners) == (9.0, 18.0, 50.0, 60.0)


def _detection_array(
    *,
    ts: float,
    marker_id: str = "0",
    class_id: str = "DICT_APRILTAG_36h11:0",
    center: Vector3 | None = None,
    orientation: Quaternion | None = None,
) -> Detection3DArray:
    if center is None:
        center = Vector3(1.2, -0.3, 0.8)
    if orientation is None:
        orientation = Quaternion(0.0, 0.0, 0.0, 1.0)

    det = Detection3D()
    det.header = Header(ts, "world")
    det.id = marker_id
    det.results = [
        ObjectHypothesisWithPose(
            hypothesis=ObjectHypothesis(
                class_id=class_id,
                score=1.0,
            )
        )
    ]
    det.results_length = len(det.results)
    det.bbox = BoundingBox3D(
        center=Pose(
            position=center,
            orientation=orientation,
        ),
        size=Vector3(0.18, 0.18, 0.0),
    )
    return Detection3DArray(
        header=Header(ts, "world"),
        detections=[det],
        detections_length=1,
    )


def test_marker_tf_module_publishes_world_markers_chain() -> None:
    ts = 1_000_000.0
    center = Vector3(1.3, -0.2, 0.4)
    orientation = Quaternion(0.1, 0.2, 0.3, 0.9)

    mod = MarkerTfModule()
    try:
        mod._process_detections(
            _detection_array(
                ts=ts,
                marker_id="7",
                class_id="DICT_APRILTAG_36h11:99",
                center=center,
                orientation=orientation,
            )
        )

        wm = mod.tf.get("world", "markers", ts, 1.0)
        assert wm is not None
        assert abs(wm.translation.x) < 1e-6
        assert abs(wm.translation.y) < 1e-6
        assert abs(wm.translation.z) < 1e-6

        w_m7 = mod.tf.get("world", "marker_7", ts, 1.0)
        assert w_m7 is not None
        assert w_m7.translation.x == pytest.approx(center.x)
        assert w_m7.translation.y == pytest.approx(center.y)
        assert w_m7.translation.z == pytest.approx(center.z)
        assert w_m7.rotation.x == pytest.approx(orientation.x)
        assert w_m7.rotation.y == pytest.approx(orientation.y)
        assert w_m7.rotation.z == pytest.approx(orientation.z)
        assert w_m7.rotation.w == pytest.approx(orientation.w)
        assert mod.tf.get("world", "marker_99", ts, 0.1) is None
    finally:
        mod.stop()


def test_marker_tf_parses_class_id_when_detection_id_empty() -> None:
    ts = 700_000.0

    mod = MarkerTfModule()
    try:
        mod._process_detections(
            _detection_array(ts=ts, marker_id="", class_id="DICT_APRILTAG_36h11:42")
        )

        assert mod.tf.get("world", "marker_42", ts, 1.0) is not None
    finally:
        mod.stop()


def test_marker_tf_empty_array_skips_publication() -> None:
    ts = 600_000.0
    mod = MarkerTfModule()
    try:
        mod._process_detections(
            Detection3DArray(
                header=Header(ts, "world"),
                detections=[],
                detections_length=0,
            )
        )

        assert mod.tf.get("world", "markers", ts, 0.1) is None
    finally:
        mod.stop()


def test_marker_tf_non_empty_array_without_marker_id_skips_publication() -> None:
    ts = 650_000.0
    mod = MarkerTfModule()
    try:
        mod._process_detections(_detection_array(ts=ts, marker_id="", class_id="marker"))

        assert mod.tf.get("world", "markers", ts, 0.1) is None
    finally:
        mod.stop()


def test_marker_tf_does_not_recompute_marker_pose() -> None:
    ts = 800_000.0
    mod = MarkerTfModule()
    try:
        with patch("dimos.perception.fiducial.marker_pose.estimate_marker_pose") as mock_estimate:
            mod._process_detections(_detection_array(ts=ts, marker_id="4"))

        mock_estimate.assert_not_called()
        assert mod.tf.get("world", "marker_4", ts, 1.0) is not None
    finally:
        mod.stop()


def test_marker_namespace_prefix_child_frames() -> None:
    ts = 500_000.0

    mod = MarkerTfModule(marker_namespace_prefix="r1")
    try:
        mod._process_detections(_detection_array(ts=ts))

        assert mod.tf.get("world", "r1/markers", ts, 1.0) is not None
        assert mod.tf.get("world", "r1/marker_0", ts, 1.0) is not None
    finally:
        mod.stop()
