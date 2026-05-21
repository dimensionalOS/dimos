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

"""Integration test for ``deploy`` + ``MarkerTfModule`` (worker, LCM, TF).

The oracle is **OpenCV** ``solvePnP`` (same planar object layout and
``SOLVEPNP_IPPE_SQUARE`` as the module) plus a **NumPy** SE(3) chain. Nothing is
imported from ``marker_tf_module`` except ``deploy`` (the package entrypoint
under test). A flat synthetic marker image keeps detection reliable; depth is
whatever ``solvePnP`` recovers for that view, and we assert the running module
matches that same OpenCV reference on the same pixels.
"""

from __future__ import annotations

import time
from typing import cast

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as SciRotation

from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.perception.fiducial.marker_tf_module import deploy
from dimos.perception.fiducial.testing.manual_frame_camera import ManualFrameCameraModule
from dimos.protocol.tf.tf import LCMTF
from dimos.spec.perception import Camera


def _marker_square_object_points_m(marker_length_m: float) -> np.ndarray:
    """Planar Z=0 square; corner order matches OpenCV ArUco / IPPE_SQUARE usage."""
    h = marker_length_m / 2.0
    return np.array(
        [
            [-h, h, 0.0],
            [h, h, 0.0],
            [h, -h, 0.0],
            [-h, -h, 0.0],
        ],
        dtype=np.float32,
    )


def _se3_to_4x4(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R.astype(np.float64)
    T[:3, 3] = t.reshape(3).astype(np.float64)
    return T


def _opencv_pnp_cam_marker(
    corners_1x4x2: np.ndarray,
    marker_length_m: float,
    cam_mtx: np.ndarray,
    dist: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    obj = _marker_square_object_points_m(marker_length_m)
    img = corners_1x4x2.reshape(4, 1, 2).astype(np.float32)
    ok, rvec, tvec = cv2.solvePnP(
        obj,
        img,
        cam_mtx,
        dist,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not ok:
        raise RuntimeError("solvePnP failed for oracle")
    return rvec, tvec


def _quaternion_xyzw_close(
    a: np.ndarray, bx: float, by: float, bz: float, bw: float, atol: float
) -> None:
    b = np.array([bx, by, bz, bw], dtype=np.float64)
    da = np.linalg.norm(a - b)
    db = np.linalg.norm(a + b)
    assert min(da, db) < atol, f"quaternion mismatch min_err={min(da, db)}"


def _synthetic_flat_marker_bgr(marker_id: int = 0) -> np.ndarray:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    side_px = 200
    tile = np.zeros((side_px, side_px), dtype=np.uint8)
    cv2.aruco.generateImageMarker(dictionary, marker_id, side_px, tile)
    height, width = 480, 640
    canvas = np.full((height, width), 255, dtype=np.uint8)
    yo = (height - side_px) // 2
    xo = (width - side_px) // 2
    canvas[yo : yo + side_px, xo : xo + side_px] = tile
    return cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)


def test_marker_tf_deploy_lcm_tf_integration() -> None:
    """Exercise deployed ``MarkerTfModule``; oracle = OpenCV solvePnP + NumPy SE3."""
    height, width = 480, 640
    marker_length_m = 0.18
    fx, fy, cx, cy = 600.0, 600.0, 320.0, 240.0
    K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
    dist = np.zeros((5, 1), dtype=np.float64)

    bgr = _synthetic_flat_marker_bgr(0)
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    det = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
    corners, ids, _ = det.detectMarkers(gray)
    assert ids is not None and len(ids) >= 1
    assert int(np.asarray(ids).reshape(-1)[0]) == 0

    rvec, tvec = _opencv_pnp_cam_marker(corners[0], marker_length_m, K, dist)
    R_cm, _ = cv2.Rodrigues(rvec)
    T_cam_marker = _se3_to_4x4(R_cm, tvec.reshape(3))

    ts = time.time()
    cam_info = CameraInfo.from_intrinsics(fx, fy, cx, cy, width, height, frame_id="camera_optical")
    cam_info.ts = ts

    t_world_base = Transform(
        translation=Vector3(0.5, -0.2, 0.05),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id="world",
        child_frame_id="base_link",
        ts=ts,
    )
    t_base_optical = Transform(
        translation=Vector3(0.1, 0.0, 0.0),
        rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
        frame_id="base_link",
        child_frame_id="camera_optical",
        ts=ts,
    )

    T_wb = _se3_to_4x4(
        np.eye(3),
        np.array(
            [t_world_base.translation.x, t_world_base.translation.y, t_world_base.translation.z]
        ),
    )
    T_bo = _se3_to_4x4(
        np.eye(3),
        np.array(
            [
                t_base_optical.translation.x,
                t_base_optical.translation.y,
                t_base_optical.translation.z,
            ]
        ),
    )
    T_wm = T_wb @ T_bo @ T_cam_marker
    exp_t = T_wm[:3, 3]
    exp_q = SciRotation.from_matrix(T_wm[:3, :3]).as_quat()

    coord = ModuleCoordinator()
    coord.start()
    host_tf = LCMTF()
    color_transport: LCMTransport[Image] | None = None
    info_transport: LCMTransport[CameraInfo] | None = None
    try:
        cam = coord.deploy(ManualFrameCameraModule)
        color_transport = LCMTransport(
            "/integration_marker_tf/color_image",
            Image,
        )
        info_transport = LCMTransport(
            "/integration_marker_tf/camera_info",
            CameraInfo,
        )
        cam.color_image.transport = color_transport
        cam.camera_info.transport = info_transport

        deploy(
            coord,
            cast(Camera, cam),  # noqa: TC006
            prefix="/marker_tf",
            marker_length_m=marker_length_m,
            max_freq=60.0,
        )

        time.sleep(0.35)
        image = Image(data=bgr, format=ImageFormat.BGR, ts=ts, frame_id="camera_optical")
        w_m: Transform | None = None
        w_markers: Transform | None = None
        for _ in range(40):
            host_tf.publish(t_world_base, t_base_optical)
            info_transport.publish(cam_info)
            color_transport.publish(image)
            time.sleep(0.08)
            w_markers = host_tf.get("world", "marker_tf/markers", ts, 1.0)
            w_m = host_tf.get("world", "marker_tf/marker_0", ts, 1.0)
            if w_m is not None and w_markers is not None:
                break

        assert w_markers is not None, "Timed out waiting for world -> marker_tf/markers"
        assert w_m is not None, "Timed out waiting for world -> marker_tf/marker_0"

        assert w_markers.frame_id == "world"
        assert w_markers.child_frame_id == "marker_tf/markers"
        assert w_m.frame_id == "world"
        assert w_m.child_frame_id == "marker_tf/marker_0"

        assert abs(w_markers.translation.x) < 1e-5
        assert abs(w_markers.translation.y) < 1e-5
        assert abs(w_markers.translation.z) < 1e-5

        np.testing.assert_allclose(
            [w_m.translation.x, w_m.translation.y, w_m.translation.z],
            exp_t,
            rtol=0.02,
            atol=0.02,
        )
        _quaternion_xyzw_close(
            exp_q,
            w_m.rotation.x,
            w_m.rotation.y,
            w_m.rotation.z,
            w_m.rotation.w,
            atol=0.02,
        )
    finally:
        if info_transport is not None:
            info_transport.stop()
        if color_transport is not None:
            color_transport.stop()
        host_tf.stop()
        coord.stop()
