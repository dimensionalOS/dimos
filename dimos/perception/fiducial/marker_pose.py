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

from typing import TYPE_CHECKING, Any

from dimos_generated.geometry_msgs.msg import Transform, TransformStamped, Vector3
from dimos_generated.sensor_msgs.msg import CameraInfo, Image
from dimos_generated.std_msgs.msg import Header
import numpy as np

from dimos.msgs.geometry import quaternion_from_matrix

if TYPE_CHECKING:
    import cv2
    import cv2.aruco

_FISHEYE_MODELS = frozenset({"equidistant", "fisheye", "kannala_brandt"})


def is_fisheye_model(distortion_model: str | None) -> bool:
    """Return whether a CameraInfo distortion model should use fisheye handling."""
    return (distortion_model or "").strip().lower() in _FISHEYE_MODELS


def camera_info_to_cv_matrices(
    camera_info: CameraInfo,
) -> tuple[np.ndarray[Any, np.dtype[Any]], np.ndarray[Any, np.dtype[Any]]]:
    """Build OpenCV ``cameraMatrix`` and ``distCoeffs`` from ``CameraInfo``."""
    k = np.array(camera_info.k, dtype=np.float64).reshape(3, 3)
    d = np.array(camera_info.d if camera_info.d else [], dtype=np.float64).reshape(-1, 1)
    return k, d


def camera_optical_frame_id(image: Image, camera_info: CameraInfo) -> str:
    """Frame in which image pixels and intrinsics apply (optical convention in ROS).

    Prefer ``Image.header.frame_id`` so TF lookups match the stream that produced the
    pixels. Fall back to ``CameraInfo.header.frame_id``, then a conventional default.
    """
    for fid in (image.header.frame_id, camera_info.header.frame_id):
        if fid and fid.strip():
            return fid.strip()
    return "camera_optical"


def _aruco_marker_object_points(marker_length_m: float) -> np.ndarray[Any, np.dtype[Any]]:
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
    corners_px: np.ndarray[Any, np.dtype[Any]],
    marker_length_m: float,
    camera_matrix: np.ndarray[Any, np.dtype[Any]],
    dist_coeffs: np.ndarray[Any, np.dtype[Any]],
    *,
    distortion_model: str | None = None,
) -> tuple[np.ndarray[Any, np.dtype[Any]], np.ndarray[Any, np.dtype[Any]]] | None:
    """Return ``(rvec, tvec)`` for camera optical <- marker from undistorted solvePnP.

    For fisheye/equidistant intrinsics, corners are first undistorted into the
    same pinhole ``K`` so the radtan-only ``solvePnP`` sees pinhole-equivalent
    pixels. Otherwise the radtan ``dist_coeffs`` are passed straight through.
    """
    import cv2

    obj = _aruco_marker_object_points(marker_length_m)
    img: np.ndarray[Any, np.dtype[Any]] = corners_px.reshape(4, 1, 2).astype(np.float32)
    if is_fisheye_model(distortion_model):
        d_flat = np.asarray(dist_coeffs, dtype=np.float64).reshape(-1)
        if d_flat.size < 4:
            raise ValueError(
                f"Fisheye/equidistant distortion model requires at least 4 coefficients; "
                f"got {d_flat.size}. Check CameraInfo.d."
            )
        d_fisheye = d_flat[:4].reshape(4, 1)
        img = cv2.fisheye.undistortPoints(img, camera_matrix, d_fisheye, P=camera_matrix)
        solve_dist: np.ndarray[Any, np.dtype[Any]] = np.zeros((0, 1), dtype=np.float64)
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
    rvec: np.ndarray[Any, np.dtype[Any]],
    tvec: np.ndarray[Any, np.dtype[Any]],
    *,
    header: Header,
    child_frame_id: str,
) -> TransformStamped:
    """Build ``Transform`` for ``frame_id`` <- ``child_frame_id`` (camera <- marker)."""
    import cv2

    rot_mat, _ = cv2.Rodrigues(rvec)
    quat = quaternion_from_matrix(np.asarray(rot_mat, dtype=np.float64))
    tx, ty, tz = tvec.reshape(3)
    return TransformStamped(
        header=header,
        child_frame_id=child_frame_id,
        transform=Transform(
            translation=Vector3(x=float(tx), y=float(ty), z=float(tz)), rotation=quat
        ),
    )


def create_aruco_detector(
    dictionary_name: str, *, detect_inverted: bool = False
) -> cv2.aruco.ArucoDetector:
    """Build a detector; `detect_inverted` also accepts light-on-dark (negative) markers."""
    import cv2.aruco

    if not hasattr(cv2.aruco, dictionary_name):
        raise ValueError(f"Unknown ArUco dictionary {dictionary_name!r}")
    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
    parameters = cv2.aruco.DetectorParameters()
    parameters.detectInvertedMarker = detect_inverted
    return cv2.aruco.ArucoDetector(dictionary, parameters)


def marker_corners_to_bbox(
    corners_px: np.ndarray[Any, np.dtype[Any]],
) -> tuple[float, float, float, float]:
    """Return the axis-aligned image bbox around a marker's four pixel corners."""
    corners_2d = np.asarray(corners_px, dtype=np.float32).reshape(4, 2)
    xy_min = corners_2d.min(axis=0)
    xy_max = corners_2d.max(axis=0)
    return (float(xy_min[0]), float(xy_min[1]), float(xy_max[0]), float(xy_max[1]))


def marker_reprojection_error(
    corners_px: np.ndarray[Any, np.dtype[Any]],
    marker_length_m: float,
    camera_matrix: np.ndarray[Any, np.dtype[Any]],
    dist_coeffs: np.ndarray[Any, np.dtype[Any]],
    rvec: np.ndarray[Any, np.dtype[Any]],
    tvec: np.ndarray[Any, np.dtype[Any]],
    *,
    distortion_model: str | None = None,
) -> float:
    """Return RMS corner reprojection error in pixels.

    Fisheye/equidistant inputs are compared in the same undistorted pinhole
    pixel space used by :func:`estimate_marker_pose`.
    """
    import cv2

    observed: np.ndarray[Any, np.dtype[Any]] = np.asarray(corners_px, dtype=np.float32).reshape(
        4, 1, 2
    )
    project_dist = dist_coeffs

    if is_fisheye_model(distortion_model):
        d_flat = np.asarray(dist_coeffs, dtype=np.float64).reshape(-1)
        if d_flat.size < 4:
            raise ValueError(
                f"Fisheye/equidistant distortion model requires at least 4 coefficients; "
                f"got {d_flat.size}. Check CameraInfo.d."
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
