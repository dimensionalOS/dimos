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
from dimos.perception.fiducial.test_helpers import blank_image, camera_info, world_T_optical

# Matches camera_info() in test_helpers (640x480, fx=fy=600, no distortion) so the
# stubbed corners live in the same intrinsics detect_markers_in_image solves with.
_PINHOLE_K = np.array([[600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]])
_NO_DIST = np.zeros((5, 1), dtype=np.float64)


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


class _CornerStubDetector:
    """detectMarkers stub returning exactly the corners we constructed, one marker id.

    detect_markers_in_image accepts an injected detector so a test can hand it precise
    corners and isolate the ambiguity_ratio_min wiring (corners -> ambiguity_gated_pose)
    from the real ArUco image-detection stage. Shapes mirror OpenCV: corners is a tuple
    of (1, 4, 2) float32 arrays; ids is an (N, 1) int array.
    """

    def __init__(self, corners_px: np.ndarray, marker_id: int) -> None:
        self._corners = corners_px.reshape(1, 4, 2).astype(np.float32)
        self._ids = np.array([[marker_id]], dtype=np.int32)

    def detectMarkers(self, gray: np.ndarray) -> tuple[Any, np.ndarray, None]:
        return (self._corners,), self._ids, None


def _detect(corners: np.ndarray, ambiguity_ratio_min: float, ts: float = 20.0) -> list[Any]:
    """Drive detect_markers_in_image with the stub detector and constructed corners."""
    return detect_markers_in_image(
        blank_image(ts),
        camera_info=camera_info(ts),
        world_T_optical=world_T_optical(ts),
        marker_length_m=_MARKER_LENGTH_M,
        aruco_dictionary="DICT_APRILTAG_36h11",
        ambiguity_ratio_min=ambiguity_ratio_min,
        detector=_CornerStubDetector(corners, marker_id=7),
        camera_matrix=_PINHOLE_K,
        dist_coeffs=_NO_DIST,
    )


# Flip-ambiguous view (see test_marker_pose): a small tag seen near-frontal and far is
# weak perspective, so the mirror pose reprojects nearly as well as the true one.
_MARKER_LENGTH_M = 0.1
_RVEC_AMBIGUOUS = np.array([[0.03], [0.02], [0.0]], dtype=np.float64)
_TVEC_AMBIGUOUS = np.array([[0.0], [0.0], [8.0]], dtype=np.float64)


def _flip_ambiguous_corners() -> np.ndarray:
    """Weak-perspective corners plus 0.3px seeded symmetric noise -- reconstructs the
    exact mirror-ambiguous view the marker_pose gate test rejects at ratio 2.0."""
    clean = _project_marker_corners(_RVEC_AMBIGUOUS, _TVEC_AMBIGUOUS, _MARKER_LENGTH_M)
    rng = np.random.default_rng(42)
    return clean + rng.normal(0.0, 0.3, clean.shape).astype(np.float32)


def test_detect_markers_ambiguity_gate_off_keeps_flip_ambiguous_view() -> None:
    """Invariant: ambiguity_ratio_min=1.0 (the default) can never reject, so detect_
    markers_in_image publishes the mirror-ambiguous view as one detection -- the
    ungated best-solution path. Pairs with the drop test below on IDENTICAL corners,
    isolating the ratio as the only cause of the difference (not a solver failure)."""
    dets = _detect(_flip_ambiguous_corners(), ambiguity_ratio_min=1.0)
    assert len(dets) == 1
    assert dets[0].marker_id == 7


def test_detect_markers_ambiguity_gate_drops_flip_ambiguous_view() -> None:
    """Invariant: with ambiguity_ratio_min=2.0 the same flip-ambiguous corners are
    dropped -- detect_markers_in_image returns []. Proves the ratio kwarg reaches the
    per-glimpse gate and suppresses a confident-wrong mirror pose at the source."""
    dets = _detect(_flip_ambiguous_corners(), ambiguity_ratio_min=2.0)
    assert dets == []


def test_detect_markers_ambiguity_gate_keeps_strong_perspective_view() -> None:
    """Invariant (control): ratio 2.0 does not blanket-drop -- a large tag close and
    strongly oblique has one clearly-best pose (mirror reprojects far worse), so it
    survives the gate and yields one detection. Confirms the drop above is the view's
    ambiguity, not the ratio rejecting everything."""
    rvec_true = np.array([[0.9], [0.5], [0.1]], dtype=np.float64)  # ~60 deg oblique
    tvec_true = np.array([[0.05], [-0.03], [0.6]], dtype=np.float64)  # 0.6 m, close
    corners = _project_marker_corners(rvec_true, tvec_true, marker_length_m=0.2)
    dets = detect_markers_in_image(
        blank_image(21.0),
        camera_info=camera_info(21.0),
        world_T_optical=world_T_optical(21.0),
        marker_length_m=0.2,
        aruco_dictionary="DICT_APRILTAG_36h11",
        ambiguity_ratio_min=2.0,
        detector=_CornerStubDetector(corners, marker_id=7),
        camera_matrix=_PINHOLE_K,
        dist_coeffs=_NO_DIST,
    )
    assert len(dets) == 1
    assert dets[0].marker_id == 7


def _rotation_angle_deg(rot_a: np.ndarray, rot_b: np.ndarray) -> float:
    """Geodesic angle (deg) between two rotation matrices."""
    cos = (np.trace(rot_a.T @ rot_b) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))


class _MultiCornerStubDetector:
    """detectMarkers stub returning several constructed corner sets with their ids, to
    drive detect_markers_in_image's per-marker gate loop over more than one marker in
    a single frame. Shapes mirror OpenCV: corners a tuple of (1, 4, 2) float32 arrays,
    ids an (N, 1) int array."""

    def __init__(self, corner_sets: list[np.ndarray], marker_ids: list[int]) -> None:
        self._corners = tuple(c.reshape(1, 4, 2).astype(np.float32) for c in corner_sets)
        self._ids = np.array([[mid] for mid in marker_ids], dtype=np.int32)

    def detectMarkers(self, gray: np.ndarray) -> tuple[Any, np.ndarray, None]:
        return self._corners, self._ids, None


# Strong-perspective known-truth view (survives the gate at ratio 2.0, see the control
# test above). world_T_optical is translation (1,2,3) with identity rotation, so the
# recovered world_T_marker must equal (1,2,3)+tvec for translation and R(rvec) for
# rotation -- the wrapper's frame composition has no free parameter to absorb error.
_POSE_MARKER_LENGTH_M = 0.2
_RVEC_STRONG = np.array([[0.9], [0.5], [0.1]], dtype=np.float64)  # ~60 deg oblique
_TVEC_STRONG = np.array([[0.05], [-0.03], [0.6]], dtype=np.float64)  # 0.6 m, close


def test_detect_markers_recovers_known_world_pose_with_near_zero_reproj() -> None:
    """Invariant: for a noise-free projection of a known camera_optical <- marker pose,
    detect_markers_in_image publishes the true world-frame marker pose end to end --
    center == world_T_optical.translation + tvec (identity camera rotation) and
    orientation == R(rvec) -- carried through the gate, Rodrigues, and Transform
    composition, and ships a near-zero reprojection_error health signal proving the
    surviving pose is trustworthy, not merely present."""
    corners = _project_marker_corners(_RVEC_STRONG, _TVEC_STRONG, _POSE_MARKER_LENGTH_M)
    dets = detect_markers_in_image(
        blank_image(20.0),
        camera_info=camera_info(20.0),
        world_T_optical=world_T_optical(20.0),
        marker_length_m=_POSE_MARKER_LENGTH_M,
        aruco_dictionary="DICT_APRILTAG_36h11",
        ambiguity_ratio_min=2.0,
        detector=_CornerStubDetector(corners, marker_id=7),
        camera_matrix=_PINHOLE_K,
        dist_coeffs=_NO_DIST,
    )
    assert len(dets) == 1
    det = dets[0]

    expected_center = np.array([1.0, 2.0, 3.0]) + _TVEC_STRONG.reshape(3)
    got_center = np.array([det.center.x, det.center.y, det.center.z])
    np.testing.assert_allclose(got_center, expected_center, atol=1e-4)

    rot_true = cv2.Rodrigues(_RVEC_STRONG)[0]
    rot_world_marker = Transform(rotation=det.orientation).to_matrix()[:3, :3]
    assert _rotation_angle_deg(rot_true, rot_world_marker) < 0.01

    assert det.reprojection_error < 0.01


def test_detect_markers_bbox_and_corners_match_constructed() -> None:
    """Invariant: the published detection carries the exact detected pixel corners and
    an axis-aligned bbox that is their (xmin, ymin, xmax, ymax) -- the constructed
    corners round-trip through the wrapper untouched, so crop/draw helpers see the true
    marker extent."""
    corners = _project_marker_corners(_RVEC_STRONG, _TVEC_STRONG, _POSE_MARKER_LENGTH_M)
    dets = detect_markers_in_image(
        blank_image(20.0),
        camera_info=camera_info(20.0),
        world_T_optical=world_T_optical(20.0),
        marker_length_m=_POSE_MARKER_LENGTH_M,
        aruco_dictionary="DICT_APRILTAG_36h11",
        ambiguity_ratio_min=2.0,
        detector=_CornerStubDetector(corners, marker_id=7),
        camera_matrix=_PINHOLE_K,
        dist_coeffs=_NO_DIST,
    )
    assert len(dets) == 1
    det = dets[0]

    np.testing.assert_array_equal(np.asarray(det.corners_px, dtype=np.float32), corners)
    xy_min = corners.min(axis=0)
    xy_max = corners.max(axis=0)
    expected_bbox = (float(xy_min[0]), float(xy_min[1]), float(xy_max[0]), float(xy_max[1]))
    assert det.bbox == expected_bbox


def test_detect_markers_two_markers_gated_independently() -> None:
    """Invariant: the gate is applied per marker in the zip loop, so in one frame a
    flip-ambiguous marker is dropped while a strong-perspective marker survives with the
    correct id. The ambiguous marker is listed FIRST, proving its `continue` does not
    skip the marker after it and that ids stay aligned to corners (zip strict)."""
    strong = _project_marker_corners(_RVEC_STRONG, _TVEC_STRONG, _POSE_MARKER_LENGTH_M)
    clean = _project_marker_corners(
        np.array([[0.03], [0.02], [0.0]], dtype=np.float64),  # near-frontal
        np.array([[0.0], [0.0], [8.0]], dtype=np.float64),  # 8 m, weak perspective
        _POSE_MARKER_LENGTH_M,
    )
    rng = np.random.default_rng(42)
    ambiguous = clean + rng.normal(0.0, 0.3, clean.shape).astype(np.float32)

    dets = detect_markers_in_image(
        blank_image(20.0),
        camera_info=camera_info(20.0),
        world_T_optical=world_T_optical(20.0),
        marker_length_m=_POSE_MARKER_LENGTH_M,
        aruco_dictionary="DICT_APRILTAG_36h11",
        ambiguity_ratio_min=2.0,
        detector=_MultiCornerStubDetector([ambiguous, strong], marker_ids=[11, 7]),
        camera_matrix=_PINHOLE_K,
        dist_coeffs=_NO_DIST,
    )
    assert len(dets) == 1
    assert dets[0].marker_id == 7
