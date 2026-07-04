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

"""Shared fiducial marker pose helpers."""

from __future__ import annotations

import cv2
import cv2.aruco
import numpy as np

from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image

_FISHEYE_MODELS = frozenset({"equidistant", "fisheye", "kannala_brandt"})


def is_fisheye_model(distortion_model: str | None) -> bool:
    """Return whether a CameraInfo distortion model should use fisheye handling."""
    return (distortion_model or "").strip().lower() in _FISHEYE_MODELS


def camera_info_to_cv_matrices(camera_info: CameraInfo) -> tuple[np.ndarray, np.ndarray]:
    """Build OpenCV ``cameraMatrix`` and ``distCoeffs`` from ``CameraInfo``."""
    k = np.array(camera_info.K, dtype=np.float64).reshape(3, 3)
    d = np.array(camera_info.D if camera_info.D else [], dtype=np.float64).reshape(-1, 1)
    return k, d


def camera_optical_frame_id(image: Image, camera_info: CameraInfo) -> str:
    """Frame in which image pixels and intrinsics apply (optical convention in ROS).

    Prefer ``Image.frame_id`` so TF lookups match the stream that produced the
    pixels. Fall back to ``CameraInfo.frame_id``, then a conventional default.
    """
    for fid in (image.frame_id, camera_info.frame_id):
        if fid and fid.strip():
            return fid.strip()
    return "camera_optical"


def _aruco_marker_object_points(marker_length_m: float) -> np.ndarray:
    """Corner order matches OpenCV ArUco / solvePnP convention (planar square, Z=0)."""
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


def estimate_marker_pose(
    corners_px: np.ndarray,
    marker_length_m: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    *,
    distortion_model: str | None = None,
) -> tuple[np.ndarray, np.ndarray] | None:
    """Return ``(rvec, tvec)`` for camera optical <- marker from undistorted solvePnP.

    For fisheye/equidistant intrinsics, corners are first undistorted into the
    same pinhole ``K`` so the radtan-only ``solvePnP`` sees pinhole-equivalent
    pixels. Otherwise the radtan ``dist_coeffs`` are passed straight through.
    """
    obj = _aruco_marker_object_points(marker_length_m)
    img: np.ndarray = corners_px.reshape(4, 1, 2).astype(np.float32)
    if is_fisheye_model(distortion_model):
        d_flat = np.asarray(dist_coeffs, dtype=np.float64).reshape(-1)
        if d_flat.size < 4:
            raise ValueError(
                f"Fisheye/equidistant distortion model requires at least 4 coefficients; "
                f"got {d_flat.size}. Check CameraInfo.D."
            )
        d_fisheye = d_flat[:4].reshape(4, 1)
        img = cv2.fisheye.undistortPoints(img, camera_matrix, d_fisheye, P=camera_matrix)
        solve_dist: np.ndarray = np.zeros((0, 1), dtype=np.float64)
    else:
        solve_dist = dist_coeffs
    ok, rvec, tvec = cv2.solvePnP(
        obj,
        img,
        camera_matrix,
        solve_dist,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not ok:
        return None
    return rvec, tvec


def rvec_tvec_to_transform(
    rvec: np.ndarray,
    tvec: np.ndarray,
    *,
    frame_id: str,
    child_frame_id: str,
    ts: float,
) -> Transform:
    """Build ``Transform`` for ``frame_id`` <- ``child_frame_id`` (camera <- marker)."""
    rot_mat, _ = cv2.Rodrigues(rvec)
    quat = Quaternion.from_rotation_matrix(rot_mat)
    tx, ty, tz = tvec.reshape(3)
    return Transform(
        translation=Vector3(float(tx), float(ty), float(tz)),
        rotation=quat,
        frame_id=frame_id,
        child_frame_id=child_frame_id,
        ts=ts,
    )


def create_aruco_detector(dictionary_name: str) -> cv2.aruco.ArucoDetector:
    if not hasattr(cv2.aruco, dictionary_name):
        raise ValueError(f"Unknown ArUco dictionary {dictionary_name!r}")
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    parameters = cv2.aruco.DetectorParameters()
    return cv2.aruco.ArucoDetector(dictionary, parameters)


def marker_corners_to_bbox(corners_px: np.ndarray) -> tuple[float, float, float, float]:
    """Return the axis-aligned image bbox around a marker's four pixel corners."""
    corners_2d = np.asarray(corners_px, dtype=np.float32).reshape(4, 2)
    xy_min = corners_2d.min(axis=0)
    xy_max = corners_2d.max(axis=0)
    return (float(xy_min[0]), float(xy_min[1]), float(xy_max[0]), float(xy_max[1]))


def marker_reprojection_error(
    corners_px: np.ndarray,
    marker_length_m: float,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    *,
    distortion_model: str | None = None,
) -> float:
    """Return RMS corner reprojection error in pixels.

    Fisheye/equidistant inputs are compared in the same undistorted pinhole
    pixel space used by :func:`estimate_marker_pose`.
    """
    observed: np.ndarray = np.asarray(corners_px, dtype=np.float32).reshape(4, 1, 2)
    project_dist = dist_coeffs

    if is_fisheye_model(distortion_model):
        d_flat = np.asarray(dist_coeffs, dtype=np.float64).reshape(-1)
        if d_flat.size < 4:
            raise ValueError(
                f"Fisheye/equidistant distortion model requires at least 4 coefficients; "
                f"got {d_flat.size}. Check CameraInfo.D."
            )
        d_fisheye = d_flat[:4].reshape(4, 1)
        observed = cv2.fisheye.undistortPoints(
            observed,
            camera_matrix,
            d_fisheye,
            P=camera_matrix,
        )
        project_dist = np.zeros((0, 1), dtype=np.float64)

    projected, _ = cv2.projectPoints(
        _aruco_marker_object_points(marker_length_m),
        rvec,
        tvec,
        camera_matrix,
        project_dist,
    )
    residual = projected.reshape(4, 2) - observed.reshape(4, 2)
    return float(np.sqrt(np.mean(np.sum(residual * residual, axis=1))))
