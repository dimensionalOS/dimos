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

from typing import Any

import cv2
import numpy as np

from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.perception.fiducial.marker_detect import detect_markers_in_image
from dimos.perception.fiducial.test_helpers import (
    blank_image,
    camera_info,
    synthetic_marker_image,
    world_T_optical,
)

# Matches camera_info() in test_helpers (640x480, fx=fy=600, no distortion) so the
# stubbed corners live in the same intrinsics detect_markers_in_image solves with.
_PINHOLE_K = np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]])
_NO_DIST = np.zeros((5, 1), dtype=np.float64)

# Strong-perspective known-truth view (its mirror reprojects far worse, so it clears the
# gate at ratio 2.0). world_T_optical is translation (1,2,3) with identity rotation, so
# the recovered world_T_marker must equal (1,2,3)+tvec and R(rvec) -- the wrapper's frame
# composition has no free parameter to absorb error.
_MARKER_LENGTH_M = 0.2
_RVEC_STRONG = np.array([[0.9], [0.5], [0.1]], dtype=np.float64)  # ~60 deg oblique
_TVEC_STRONG = np.array([[0.05], [-0.03], [0.6]], dtype=np.float64)  # 0.6 m, close


def _project_marker_corners(
    rvec: np.ndarray, tvec: np.ndarray, marker_length_m: float
) -> np.ndarray:
    """Pixel corners of a marker at camera_optical <- marker (rvec, tvec) under _PINHOLE_K."""
    h = marker_length_m / 2.0
    obj = np.array(
        [[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]],
        dtype=np.float32,
    )
    img_pts, _jac = cv2.projectPoints(obj, rvec, tvec, _PINHOLE_K, _NO_DIST)
    return img_pts.reshape(4, 2).astype(np.float32)


def _rotation_angle_deg(rot_a: np.ndarray, rot_b: np.ndarray) -> float:
    """Geodesic angle (deg) between two rotation matrices."""
    cos = (np.trace(rot_a.T @ rot_b) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))


class _StubDetector:
    """detectMarkers stub returning the constructed corner sets with their ids, to drive
    detect_markers_in_image's per-marker gate loop on precise corners -- isolating the
    corners -> ambiguity_gated_pose wiring from the real ArUco image stage. Shapes mirror
    OpenCV: corners a tuple of (1, 4, 2) float32 arrays, ids an (N, 1) int array."""

    def __init__(self, corner_sets: list[np.ndarray], marker_ids: list[int]) -> None:
        self._corners = tuple(c.reshape(1, 4, 2).astype(np.float32) for c in corner_sets)
        self._ids = np.array([[mid] for mid in marker_ids], dtype=np.int32)

    def detectMarkers(self, gray: np.ndarray) -> tuple[Any, np.ndarray, None]:
        return self._corners, self._ids, None


def _run(detector: _StubDetector, ts: float = 20.0) -> list[Any]:
    """Drive detect_markers_in_image with a stub detector at ratio 2.0 under _PINHOLE_K."""
    return detect_markers_in_image(
        blank_image(ts),
        camera_info=camera_info(ts),
        world_T_optical=world_T_optical(ts),
        marker_length_m=_MARKER_LENGTH_M,
        aruco_dictionary="DICT_APRILTAG_36h11",
        ambiguity_ratio_min=2.0,
        detector=detector,
        camera_matrix=_PINHOLE_K,
        dist_coeffs=_NO_DIST,
    )


def test_detect_markers_in_image_decodes_a_real_frame_end_to_end() -> None:
    """Invariant: with no injected detector or intrinsics, detect_markers_in_image builds
    the real ArUco detector and reads K from camera_info, decoding one synthetic frame into
    a named world-frame detection carrying a near-zero reprojection health signal."""
    image = synthetic_marker_image(marker_id=7)
    dets = detect_markers_in_image(
        image,
        camera_info=camera_info(image.ts),
        world_T_optical=world_T_optical(image.ts),
        marker_length_m=0.18,
        aruco_dictionary="DICT_APRILTAG_36h11",
    )
    assert len(dets) == 1
    det = dets[0]
    assert det.marker_id == 7
    assert det.name == "DICT_APRILTAG_36h11:7"
    assert det.reprojection_error < 0.1
    msg = det.to_detection3d_msg()
    assert msg.id == "7"
    assert msg.results[0].hypothesis.class_id == "DICT_APRILTAG_36h11:7"


def test_detect_markers_recovers_known_world_pose_bbox_and_corners() -> None:
    """Invariant: for a noise-free projection of a known camera_optical <- marker pose the
    published detection carries the true world-frame pose (center ==
    world_T_optical.translation + tvec, orientation == R(rvec)), a near-zero
    reprojection_error health signal, and the exact detected corners with their
    axis-aligned bbox -- the frame composition and corner round-trip add no error."""
    corners = _project_marker_corners(_RVEC_STRONG, _TVEC_STRONG, _MARKER_LENGTH_M)
    dets = _run(_StubDetector([corners], marker_ids=[7]))
    assert len(dets) == 1
    det = dets[0]

    expected_center = np.array([1.0, 2.0, 3.0]) + _TVEC_STRONG.reshape(3)
    got_center = np.array([det.center.x, det.center.y, det.center.z])
    np.testing.assert_allclose(got_center, expected_center, atol=1e-4)

    rot_true = cv2.Rodrigues(_RVEC_STRONG)[0]
    rot_world_marker = Transform(rotation=det.orientation).to_matrix()[:3, :3]
    assert _rotation_angle_deg(rot_true, rot_world_marker) < 0.01

    assert det.reprojection_error < 0.01

    np.testing.assert_array_equal(np.asarray(det.corners_px, dtype=np.float32), corners)
    xy_min = corners.min(axis=0)
    xy_max = corners.max(axis=0)
    assert det.bbox == (
        float(xy_min[0]),
        float(xy_min[1]),
        float(xy_max[0]),
        float(xy_max[1]),
    )


def test_detect_markers_two_markers_gated_independently() -> None:
    """Invariant: the gate is applied per marker in the zip loop, so in one frame a
    flip-ambiguous marker is dropped while a strong-perspective marker survives with the
    correct id. The ambiguous marker is listed FIRST, proving its `continue` does not
    skip the marker after it and that ids stay aligned to corners (zip strict)."""
    strong = _project_marker_corners(_RVEC_STRONG, _TVEC_STRONG, _MARKER_LENGTH_M)
    clean = _project_marker_corners(
        np.array([[0.03], [0.02], [0.0]], dtype=np.float64),  # near-frontal
        np.array([[0.0], [0.0], [8.0]], dtype=np.float64),  # 8 m, weak perspective
        _MARKER_LENGTH_M,
    )
    rng = np.random.default_rng(42)
    ambiguous = clean + rng.normal(0.0, 0.3, clean.shape).astype(np.float32)

    dets = _run(_StubDetector([ambiguous, strong], marker_ids=[11, 7]))
    assert len(dets) == 1
    assert dets[0].marker_id == 7
