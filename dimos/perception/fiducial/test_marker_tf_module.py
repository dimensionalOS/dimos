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
import numpy as np
import pytest

pytest.importorskip("cv2.aruco")

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.perception.fiducial.marker_tf_module import (
    MarkerTfModule,
    _camera_optical_frame_id,
    deploy,
    estimate_marker_pose,
)


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
    camera = MagicMock()
    camera.color_image = MagicMock()
    camera.camera_info = MagicMock()

    with patch.object(dimos, "deploy", return_value=proxy) as mock_deploy:
        result = deploy(dimos, camera, marker_length_m=0.18)

    mock_deploy.assert_called_once_with(
        MarkerTfModule,
        marker_length_m=0.18,
        marker_namespace_prefix="marker_tf",
    )
    assert result is proxy
    proxy.color_image.connect.assert_called_once_with(camera.color_image)
    proxy.camera_info.connect.assert_called_once_with(camera.camera_info)
    proxy.start.assert_called_once()


def test_deploy_explicit_marker_namespace_prefix_overrides_prefix(dimos) -> None:
    proxy = MagicMock()
    camera = MagicMock()
    camera.color_image = MagicMock()
    camera.camera_info = MagicMock()

    with patch.object(dimos, "deploy", return_value=proxy) as mock_deploy:
        deploy(
            dimos,
            camera,
            prefix="/ignored",
            marker_length_m=0.1,
            marker_namespace_prefix="bot_a",
        )

    mock_deploy.assert_called_once_with(
        MarkerTfModule,
        marker_length_m=0.1,
        marker_namespace_prefix="bot_a",
    )


def test_deploy_empty_prefix_skips_auto_namespace(dimos) -> None:
    proxy = MagicMock()
    camera = MagicMock()
    camera.color_image = MagicMock()
    camera.camera_info = MagicMock()

    with patch.object(dimos, "deploy", return_value=proxy) as mock_deploy:
        deploy(dimos, camera, prefix="", marker_length_m=0.15)

    mock_deploy.assert_called_once_with(
        MarkerTfModule,
        marker_length_m=0.15,
        marker_namespace_prefix=None,
    )


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


def test_marker_tf_uses_image_frame_when_camera_info_frame_empty() -> None:
    """Regression: TF must use Image.frame_id when CameraInfo.frame_id is unset."""
    ts = 1_000_000.0
    fx, fy, cx, cy = 600.0, 600.0, 320.0, 240.0
    cam_info = CameraInfo.from_intrinsics(fx, fy, cx, cy, 640, 480)
    cam_info.ts = ts
    bgr = _synthetic_marker_bgr(0)
    image = Image(data=bgr, format=ImageFormat.BGR, ts=ts, frame_id="custom_optical")

    mod = MarkerTfModule(marker_length_m=0.18, max_freq=30.0)
    try:
        mod.tf.publish(
            Transform(
                translation=Vector3(1.0, 0.0, 0.0),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                frame_id="world",
                child_frame_id="base_link",
                ts=ts,
            ),
            Transform(
                translation=Vector3(0.3, 0.0, 0.0),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                frame_id="base_link",
                child_frame_id="custom_optical",
                ts=ts,
            ),
        )
        mod._latest_camera_info = cam_info
        mod._process_color_image(image)

        assert mod.tf.get("world", "marker_0", ts, 1.0) is not None
    finally:
        mod.stop()


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


def _synthetic_marker_bgr(marker_id: int = 0) -> np.ndarray:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    side_px = 220
    tile = np.zeros((side_px, side_px), dtype=np.uint8)
    cv2.aruco.generateImageMarker(dictionary, marker_id, side_px, tile)
    canvas = np.full((480, 640), 255, dtype=np.uint8)
    yo = (480 - side_px) // 2
    xo = (640 - side_px) // 2
    canvas[yo : yo + side_px, xo : xo + side_px] = tile
    return cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)


def test_marker_tf_module_publishes_world_markers_chain() -> None:
    ts = 1_000_000.0
    fx, fy, cx, cy = 600.0, 600.0, 320.0, 240.0
    cam_info = CameraInfo(
        height=480,
        width=640,
        distortion_model="plumb_bob",
        D=[0.0, 0.0, 0.0, 0.0, 0.0],
        K=[fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
        R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        P=[fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        frame_id="camera_optical",
        ts=ts,
    )
    bgr = _synthetic_marker_bgr(0)
    image = Image(data=bgr, format=ImageFormat.BGR, ts=ts, frame_id="camera_optical")

    mod = MarkerTfModule(marker_length_m=0.18, max_freq=30.0)
    try:
        mod.tf.publish(
            Transform(
                translation=Vector3(1.0, 0.0, 0.0),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                frame_id="world",
                child_frame_id="base_link",
                ts=ts,
            ),
            Transform(
                translation=Vector3(0.3, 0.0, 0.0),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                frame_id="base_link",
                child_frame_id="camera_optical",
                ts=ts,
            ),
        )
        mod._latest_camera_info = cam_info
        mod._process_color_image(image)

        wm = mod.tf.get("world", "markers", ts, 1.0)
        assert wm is not None
        assert abs(wm.translation.x) < 1e-6
        assert abs(wm.translation.y) < 1e-6
        assert abs(wm.translation.z) < 1e-6

        w_m0 = mod.tf.get("world", "marker_0", ts, 1.0)
        assert w_m0 is not None
        assert w_m0.translation.x > 1.1
        assert w_m0.translation.z > 0.2
    finally:
        mod.stop()


def test_marker_namespace_prefix_child_frames() -> None:
    ts = 500_000.0
    fx, fy, cx, cy = 600.0, 600.0, 320.0, 240.0
    cam_info = CameraInfo(
        height=480,
        width=640,
        distortion_model="plumb_bob",
        D=[0.0] * 5,
        K=[fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0],
        R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        P=[fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0],
        frame_id="camera_optical",
        ts=ts,
    )
    image = Image(data=_synthetic_marker_bgr(0), format=ImageFormat.BGR, ts=ts)

    mod = MarkerTfModule(marker_length_m=0.18, marker_namespace_prefix="r1", max_freq=30.0)
    try:
        mod.tf.publish(
            Transform(
                translation=Vector3(0.0, 0.0, 0.0),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                frame_id="world",
                child_frame_id="base_link",
                ts=ts,
            ),
            Transform(
                translation=Vector3(0.0, 0.0, 0.0),
                rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                frame_id="base_link",
                child_frame_id="camera_optical",
                ts=ts,
            ),
        )
        mod._latest_camera_info = cam_info
        mod._process_color_image(image)

        assert mod.tf.get("world", "r1/markers", ts, 1.0) is not None
        assert mod.tf.get("world", "r1/marker_0", ts, 1.0) is not None
    finally:
        mod.stop()
