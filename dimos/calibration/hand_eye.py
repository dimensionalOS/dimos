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

"""Eye-in-hand calibration with a stationary AprilTag; distances are metres.

``A_from_B`` maps coordinates in B into A. The solved ``camera_to_ee`` is
``ee_from_camera`` for the color optical frame, not the camera housing frame.
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations
from typing import Any

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from dimos.calibration import DEFAULT_MARKER_SIZE_M, MIN_TRAINING_SAMPLES, MIN_VALIDATION_SAMPLES
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.perception.fiducial.marker_pose import (
    camera_info_to_cv_matrices,
    create_aruco_detector,
    estimate_marker_pose,
    marker_reprojection_error,
)

MAX_REPROJECTION_ERROR_PX = 2.0
MAX_TRANSLATION_ERROR_M = 0.01
MAX_ROTATION_ERROR_DEG = 2.0
MIN_ROTATION_DEG = 15.0
MIN_AXIS_SEPARATION_DEG = 30.0
TAG_DICTIONARY = "DICT_APRILTAG_36h11"


def checked_transform(value: np.ndarray) -> np.ndarray:
    """Return a finite, rigid 4x4 transform or raise a descriptive error."""
    matrix = np.asarray(value, dtype=np.float64)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError("Transforms must be finite 4x4 matrices")
    rotation = matrix[:3, :3]
    if not (
        np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-6)
        and np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
        and np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6)
    ):
        raise ValueError("Transforms must contain a proper rotation and bottom row [0, 0, 0, 1]")
    return matrix


@dataclass(frozen=True)
class HandEyeSample:
    """One stationary image/robot-pose pair after marker pose estimation."""

    base_from_ee: np.ndarray
    camera_from_target: np.ndarray
    reprojection_error_px: float

    def __post_init__(self) -> None:
        checked_transform(self.base_from_ee)
        checked_transform(self.camera_from_target)
        if not np.isfinite(self.reprojection_error_px) or self.reprojection_error_px < 0:
            raise ValueError("Reprojection error must be finite and nonnegative")


class MarkerObservationError(ValueError):
    """A fresh view of the target can be retried at this pose."""


def check_marker(marker_size_m: float, marker_id: int) -> None:
    if not np.isfinite(marker_size_m) or marker_size_m <= 0:
        raise ValueError("Marker size must be a positive finite length in metres")
    if not 0 <= marker_id < 587:
        raise ValueError("tag36h11 ID must be between 0 and 586")


def checked_camera_matrices(info: CameraInfo) -> tuple[np.ndarray, np.ndarray]:
    """Check that the live camera model can be used for marker PnP."""
    if info.distortion_model not in {"plumb_bob", "rational_polynomial"}:
        raise ValueError(
            f"Unsupported color distortion model {info.distortion_model!r}. "
            "Use plumb_bob or rational_polynomial intrinsics."
        )
    k, d = camera_info_to_cv_matrices(info)
    if not np.isfinite(k).all() or not np.isfinite(d).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError("Camera intrinsics must be finite with positive focal lengths")
    if not np.allclose(k[2], [0, 0, 1]) or d.size not in {0, 4, 5, 8, 12, 14}:
        raise ValueError("Invalid pinhole camera matrix or distortion coefficient count")
    return k, d


def check_image_pixels(image: Image, info: CameraInfo) -> None:
    """Require calibrated 8-bit color or grayscale pixels for marker detection."""
    if (image.width, image.height) != (info.width, info.height):
        raise ValueError("Image resolution does not match the camera intrinsics")
    channels = {
        ImageFormat.GRAY: 1,
        ImageFormat.RGB: 3,
        ImageFormat.BGR: 3,
        ImageFormat.RGBA: 4,
        ImageFormat.BGRA: 4,
    }
    if image.dtype != np.uint8 or image.channels != channels.get(image.format):
        raise ValueError("Select an 8-bit color or grayscale image topic for marker detection")


def detect_sample(
    image: Image,
    info: CameraInfo,
    base_from_ee: np.ndarray,
    marker_size_m: float = DEFAULT_MARKER_SIZE_M,
    marker_id: int = 0,
) -> HandEyeSample:
    """Estimate camera <- tag36h11 using existing DimOS marker helpers.

    Marker size is the printed black square's outer edge, excluding white margin.
    Intrinsics must describe these exact color pixels and resolution.
    """
    check_marker(marker_size_m, marker_id)
    check_image_pixels(image, info)
    k, d = checked_camera_matrices(info)
    gray = image.to_grayscale().data
    detector = create_aruco_detector(TAG_DICTIONARY)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        # Reflections on a printed tag can wash out the contrast between its cells.
        corners, ids, _ = detector.detectMarkers(cv2.equalizeHist(gray))
    matches = [] if ids is None else np.flatnonzero(ids.ravel() == marker_id).tolist()
    if len(matches) != 1:
        raise MarkerObservationError(
            f"Expected exactly one tag36h11 ID {marker_id}; detected {len(matches)}"
        )
    pixels = corners[matches[0]]
    pose = estimate_marker_pose(pixels, marker_size_m, k, d)
    if pose is None:
        raise MarkerObservationError("Could not estimate the marker pose")
    rvec, tvec = pose
    if not np.isfinite(rvec).all() or not np.isfinite(tvec).all() or tvec.ravel()[2] <= 0:
        raise MarkerObservationError("Marker pose is nonfinite or behind the camera")
    error = marker_reprojection_error(pixels, marker_size_m, k, d, rvec, tvec)
    if not np.isfinite(error) or error > MAX_REPROJECTION_ERROR_PX:
        raise MarkerObservationError(f"Marker reprojection error {error:.3f} px exceeds 2 px")
    camera_from_target = np.eye(4)
    camera_from_target[:3, :3] = cv2.Rodrigues(rvec)[0]
    camera_from_target[:3, 3] = tvec.ravel()
    return HandEyeSample(np.asarray(base_from_ee, dtype=np.float64), camera_from_target, error)


def _check_rotation_diversity(poses: np.ndarray) -> None:
    axes = []
    for first, second in combinations(poses, 2):
        vector = cv2.Rodrigues(first[:3, :3].T @ second[:3, :3])[0].ravel()
        angle = np.linalg.norm(vector)
        if angle >= np.deg2rad(MIN_ROTATION_DEG):
            axes.append(vector / angle)
    if not any(
        np.linalg.norm(np.cross(a, b)) >= np.sin(np.deg2rad(MIN_AXIS_SEPARATION_DEG))
        for a, b in combinations(axes, 2)
    ):
        raise ValueError(
            "Need rotations of at least 15 degrees about two axes separated by 30 degrees"
        )


def _sample_poses(samples: list[HandEyeSample]) -> tuple[np.ndarray, np.ndarray]:
    robot = np.stack([checked_transform(sample.base_from_ee) for sample in samples])
    target = np.stack([checked_transform(sample.camera_from_target) for sample in samples])
    return robot, target


def _metrics(
    samples: list[HandEyeSample], targets_in_base: np.ndarray, reference: np.ndarray
) -> dict[str, Any]:
    offsets = targets_in_base[:, :3, 3] - reference[:3, 3]
    rotations = Rotation.from_matrix(reference[:3, :3].T @ targets_in_base[:, :3, :3])
    translation = float(np.linalg.norm(offsets, axis=1).max())
    rotation = float(np.rad2deg(rotations.magnitude().max()))
    reprojection = max(sample.reprojection_error_px for sample in samples)
    return {
        "passed": bool(
            translation <= MAX_TRANSLATION_ERROR_M
            and rotation <= MAX_ROTATION_ERROR_DEG
            and reprojection <= MAX_REPROJECTION_ERROR_PX
        ),
        "sample_count": len(samples),
        "max_translation_error_m": translation,
        "max_rotation_error_deg": rotation,
        "max_reprojection_error_px": reprojection,
    }


def fit_hand_eye(samples: list[HandEyeSample]) -> dict[str, Any]:
    """Fit camera -> end effector and the fixed target pose with OpenCV's Park solver."""
    if len(samples) < 3:
        raise ValueError("A hand-eye fit needs at least three poses about two rotation axes")
    robot, target = _sample_poses(samples)
    _check_rotation_diversity(robot)
    _check_rotation_diversity(target)
    rotation, translation = cv2.calibrateHandEye(
        list(robot[:, :3, :3]),
        list(robot[:, :3, 3]),
        list(target[:, :3, :3]),
        list(target[:, :3, 3]),
        method=cv2.CALIB_HAND_EYE_PARK,
    )
    camera_to_ee = np.eye(4)
    camera_to_ee[:3, :3], camera_to_ee[:3, 3] = rotation, translation.ravel()
    checked_transform(camera_to_ee)
    targets_in_base = robot @ camera_to_ee @ target
    reference = np.eye(4)
    reference[:3, :3] = Rotation.from_matrix(targets_in_base[:, :3, :3]).mean().as_matrix()
    reference[:3, 3] = targets_in_base[:, :3, 3].mean(axis=0)
    return {
        "camera_to_ee": camera_to_ee.tolist(),
        "base_from_target": reference.tolist(),
        "training_metrics": _metrics(samples, targets_in_base, reference),
    }


def solve_hand_eye(samples: list[HandEyeSample]) -> dict[str, Any]:
    """Fit the final candidate on all training observations before independent validation."""
    if len(samples) < MIN_TRAINING_SAMPLES:
        raise ValueError(f"Need at least {MIN_TRAINING_SAMPLES} training samples")
    return fit_hand_eye(samples)


def validate_hand_eye(
    samples: list[HandEyeSample], camera_to_ee: np.ndarray, base_from_target: np.ndarray
) -> dict[str, Any]:
    """Check held-out observations against the target pose fixed during fitting.

    Pass requires every held-out observation within 10 mm, 2 degrees and 2 px.
    Never refits either transform to the validation observations.
    """
    if len(samples) < MIN_VALIDATION_SAMPLES:
        raise ValueError(f"Need at least {MIN_VALIDATION_SAMPLES} validation samples")
    robot, target = _sample_poses(samples)
    targets_in_base = robot @ checked_transform(camera_to_ee) @ target
    return _metrics(samples, targets_in_base, checked_transform(base_from_target))
