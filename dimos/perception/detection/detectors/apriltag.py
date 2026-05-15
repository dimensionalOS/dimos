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

"""AprilTag/ArUco fiducial marker detector with 3D pose estimation.

Uses OpenCV's ArUco module for detection and solvePnP for marker pose
estimation given camera calibration (CameraInfo). Returns detected tag
IDs with their 6-DOF poses in the camera optical frame.

The module's output includes:
    - tag_id:          Marker dictionary ID
    - family:          Marker family name (e.g. "tag36h11")
    - pose:            PoseStamped in camera optical frame
    - corners_image:   Pixel coordinates of tag corners in the image
    - corner_count:    Number of corners detected (4 for a valid tag)

Example:
    detector = AprilTagDetector(family="tag36h11", tag_size_m=0.165)
    result = detector.detect(image, camera_info)
    for tag in result:
        print(f"Tag {tag.tag_id} at {tag.pose.position}")
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

import cv2
import numpy as np

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = setup_logger()

# OpenCV predefined marker family dictionary mapping.
_FAMILIES: dict[str, int] = {
    "tag36h11": cv2.aruco.DICT_APRILTAG_36h11,
    "tag25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "tag16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "aruco_original": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "aruco_4x4_50": cv2.aruco.DICT_4X4_50,
    "aruco_4x4_100": cv2.aruco.DICT_4X4_100,
    "aruco_4x4_250": cv2.aruco.DICT_4X4_250,
    "aruco_4x4_1000": cv2.aruco.DICT_4X4_1000,
    "aruco_5x5_50": cv2.aruco.DICT_5X5_50,
    "aruco_5x5_100": cv2.aruco.DICT_5X5_100,
    "aruco_5x5_250": cv2.aruco.DICT_5X5_250,
    "aruco_5x5_1000": cv2.aruco.DICT_5X5_1000,
    "aruco_6x6_50": cv2.aruco.DICT_6X6_50,
    "aruco_6x6_100": cv2.aruco.DICT_6X6_100,
    "aruco_6x6_250": cv2.aruco.DICT_6X6_250,
    "aruco_6x6_1000": cv2.aruco.DICT_6X6_1000,
    "aruco_7x7_50": cv2.aruco.DICT_7X7_50,
    "aruco_7x7_100": cv2.aruco.DICT_7X7_100,
    "aruco_7x7_250": cv2.aruco.DICT_7X7_250,
    "aruco_7x7_1000": cv2.aruco.DICT_7X7_1000,
}

# Default detector parameters for ArUco detector.
# Ref: https: // docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
_DEFAULT_DETECTOR_PARAMS = cv2.aruco.DetectorParameters()
_DEFAULT_REFINEMENT_PARAMS = cv2.aruco.RefineParameters()


def _camera_info_to_cv_intrinsics(camera_info: CameraInfo) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Convert dimos CameraInfo to OpenCV cameraMatrix + distCoeffs."""
    K = np.array(camera_info.K, dtype=np.float64).reshape(3, 3)
    if camera_info.D:
        D = np.array(camera_info.D, dtype=np.float64)
    else:
        D = np.zeros((5,), dtype=np.float64)
    return K, D


def _rotation_matrix_to_quaternion(R: NDArray[np.float64]) -> Quaternion:
    """Convert a 3x3 rotation matrix to a unit quaternion (x, y, z, w)."""
    # Adapted from standard matrix -> quaternion conversion.
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2  # S = 4 * qw
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2  # S = 4 * qx
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2  # S = 4 * qy
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2  # S = 4 * qz
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    return Quaternion(qx, qy, qz, qw)


@dataclass(frozen=True, slots=True)
class AprilTagDetection:
    """A single AprilTag/ArUco detection result."""

    tag_id: int
    family: str
    pose: PoseStamped
    corners_image: NDArray[np.float64]
    corner_count: int

    @property
    def tag_frame_id(self) -> str:
        """Canonical frame ID for this tag instance, e.g. 'marker/tag36h11_42'."""
        return f"marker/{self.family}_{self.tag_id}"


class AprilTagDetector:
    """OpenCV-based AprilTag/ArUco fiducial marker detector.

    Args:
        family: Marker family name. Supported families are the keys of _FAMILIES,
            e.g. ``"tag36h11"`` or ``"aruco_4x4_50"``.
        tag_size_m: Physical side length of the marker in metres (measured from
            the outer edge of the black border). This is the value to pass as
            ``tag_size`` to pose-estimation routines.
        detector_params: Optional OpenCV ``cv2.aruco.DetectorParameters`` for fine
            tuning corner refinement, thresholding, etc.
    """

    def __init__(
        self,
        family: str = "tag36h11",
        tag_size_m: float = 0.165,
        detector_params: cv2.aruco.DetectorParameters | None = None,
    ) -> None:
        if family not in _FAMILIES:
            raise ValueError(
                f"Unknown AprilTag family {family!r}. "
                f"Supported: {list(_FAMILIES.keys())}"
            )
        self.family = family
        self.tag_size_m = float(tag_size_m)
        if self.tag_size_m <= 0:
            raise ValueError("tag_size_m must be > 0")

        dict_id = _FAMILIES[family]
        self._dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
        self._params = detector_params or _DEFAULT_DETECTOR_PARAMS
        self._refinement = _DEFAULT_REFINEMENT_PARAMS
        self._detector = cv2.aruco.ArucoDetector(
            self._dictionary,
            self._params,
            refineParams=self._refinement,
        )

        # Pre-compute object coordinates for a square marker centred at origin.
        # The 3D points represent the four corners in the marker's local frame:
        #   (-s/2, -s/2, 0), (s/2, -s/2, 0), (s/2, s/2, 0), (-s/2, s/2, 0)
        # where s = tag_size_m.
        # These are used directly by solvePnP (no need for solvePnP with axis
        # flag since we already have the corner ordering aligned in object space).
        half = self.tag_size_m / 2.0
        self._object_points = np.array(
            [
                [-half, -half, 0.0],
                [half, -half, 0.0],
                [half, half, 0.0],
                [-half, half, 0.0],
            ],
            dtype=np.float64,
        )

        logger.info(
            "AprilTagDetector initialised: family=%s tag_size_m=%.3f",
            self.family,
            self.tag_size_m,
        )

    def detect(
        self,
        image: Image,
        camera_info: CameraInfo,
    ) -> list[AprilTagDetection]:
        """Detect AprilTags in an image and estimate their 3D poses.

        Args:
            image: Input colour image.
            camera_info: Camera calibration (intrinsics + distortion).

        Returns:
            List of :class:`AprilTagDetection` results, one per detected tag.
        """
        cv_image = image.to_opencv()
        if cv_image is None or cv_image.size == 0:
            logger.debug("AprilTag: empty image received")
            return []

        # Convert to greyscale if needed.
        if len(cv_image.shape) == 3:
            grey = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        else:
            grey = cv_image

        # Detect markers.
        corners, ids, rejected = self._detector.detectMarkers(grey)
        if ids is None or len(ids) == 0:
            return []

        camera_matrix, dist_coeffs = _camera_info_to_cv_intrinsics(camera_info)

        detections: list[AprilTagDetection] = []
        for i, tag_id in enumerate(ids.flatten()):
            tag_corners = corners[i]
            image_points = tag_corners.reshape(-1, 2).astype(np.float64)
            corner_count = len(image_points)

            if corner_count != 4:
                logger.debug(
                    "AprilTag: unexpected corner count %d for tag %d (expected 4)",
                    corner_count,
                    tag_id,
                )
                continue

            # Use solvePnP to estimate R, t of marker in camera frame.
            success, rvec, tvec = cv2.solvePnP(
                self._object_points,
                image_points,
                camera_matrix,
                dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
            if not success:
                logger.debug("AprilTag: solvePnP failed for tag %d", tag_id)
                continue

            # Convert axis-angle to rotation matrix, then to quaternion.
            R_mat, _ = cv2.Rodrigues(rvec)
            quaternion = _rotation_matrix_to_quaternion(R_mat)

            # Translation from solvePnP is the marker origin in camera coords.
            position = Vector3(
                float(tvec[0][0]),
                float(tvec[1][0]),
                float(tvec[2][0]),
            )

            pose = PoseStamped(
                ts=image.ts,
                frame_id=camera_info.frame_id or "camera_optical",
                position=position,
                orientation=quaternion,
            )

            detections.append(
                AprilTagDetection(
                    tag_id=int(tag_id),
                    family=self.family,
                    pose=pose,
                    corners_image=image_points,
                    corner_count=corner_count,
                )
            )
            logger.debug(
                "AprilTag: detected tag %d at (%.2f, %.2f, %.2f) in frame %s",
                tag_id,
                position.x,
                position.y,
                position.z,
                pose.frame_id,
            )

        return detections

    def stop(self) -> None:
        """No-op cleanup for API symmetry."""
        pass
