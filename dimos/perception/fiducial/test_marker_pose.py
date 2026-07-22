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

import cv2
import numpy as np
import pytest

from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.perception.fiducial.marker_pose import (
    ambiguity_gated_pose,
    camera_optical_frame_id,
    create_aruco_detector,
    estimate_marker_pose,
    estimate_marker_pose_candidates,
    marker_corners_to_bbox,
    marker_reprojection_error,
)

# Pinhole intrinsics shared by the IPPE-ambiguity tests (640x480, no distortion).
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


def _rotation_angle_deg(rvec_a: np.ndarray, rot_b: np.ndarray) -> float:
    """Geodesic angle (deg) between rotation rvec_a and rotation matrix rot_b."""
    rot_a, _ = cv2.Rodrigues(rvec_a)
    cos = (np.trace(rot_a.T @ rot_b) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))


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

    assert camera_optical_frame_id(img_custom, info_named) == "custom_optical"
    assert camera_optical_frame_id(img_whitespace, info_named) == "custom_optical"
    assert camera_optical_frame_id(img_empty, info_named) == "cam_info_optical"
    assert camera_optical_frame_id(img_empty, info_empty) == "camera_optical"


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


def test_marker_pose_candidates_recover_true_and_mirror() -> None:
    """Invariant: for a planar square, estimate_marker_pose_candidates returns exactly
    the two IPPE mirror poses (Collins & Bartoli 2014) -- the true pose recovered
    exactly and a second, clearly distinct mirror -- both finite. Corners are
    synthesized noise-free from a known camera_optical <- marker pose, so one
    candidate must equal truth to solver precision; the other must be far off (only
    reprojection error, not the candidate list, tells them apart)."""
    marker_length_m = 0.2
    rvec_true = np.array([[0.9], [0.5], [0.1]], dtype=np.float64)  # ~60 deg oblique
    tvec_true = np.array([[0.05], [-0.03], [0.6]], dtype=np.float64)  # 0.6 m, close
    corners = _project_marker_corners(rvec_true, tvec_true, marker_length_m)

    candidates = estimate_marker_pose_candidates(corners, marker_length_m, _PINHOLE_K, _NO_DIST)
    assert len(candidates) == 2
    for rvec, tvec in candidates:
        assert np.all(np.isfinite(rvec)) and np.all(np.isfinite(tvec))

    scored = sorted(
        (
            marker_reprojection_error(corners, marker_length_m, _PINHOLE_K, _NO_DIST, rvec, tvec),
            rvec,
            tvec,
        )
        for rvec, tvec in candidates
    )
    best_err, best_rvec, best_tvec = scored[0]
    runner_err, runner_rvec, _runner_tvec = scored[1]
    # Best candidate is the true pose to solver precision (noise-free corners).
    assert _rotation_angle_deg(rvec_true, cv2.Rodrigues(best_rvec)[0]) < 1e-2
    np.testing.assert_allclose(best_tvec.reshape(3), tvec_true.reshape(3), atol=1e-4)
    assert best_err < 0.01
    # The mirror is a genuinely distinct pose that reprojects far worse.
    assert _rotation_angle_deg(rvec_true, cv2.Rodrigues(runner_rvec)[0]) > 10.0
    assert runner_err > 1.0


def test_ambiguity_gate_keeps_strong_perspective() -> None:
    """Invariant: a large tag close and strongly oblique has one clearly-best IPPE
    pose (the mirror reprojects far worse), so the gate keeps it and recovers the
    true rotation. Corners are synthesized noise-free from a known
    camera_optical <- marker pose, so the surviving pose must match truth exactly."""
    marker_length_m = 0.2
    rvec_true = np.array([[0.9], [0.5], [0.1]], dtype=np.float64)  # ~60 deg oblique
    tvec_true = np.array([[0.05], [-0.03], [0.6]], dtype=np.float64)  # 0.6 m, close
    corners = _project_marker_corners(rvec_true, tvec_true, marker_length_m)

    # IPPE always yields the two mirror candidates; here they are far apart.
    candidates = estimate_marker_pose_candidates(corners, marker_length_m, _PINHOLE_K, _NO_DIST)
    assert len(candidates) == 2

    result = ambiguity_gated_pose(
        corners, marker_length_m, _PINHOLE_K, _NO_DIST, ambiguity_ratio_min=2.0
    )
    assert result is not None
    pose, reproj_px = result
    assert _rotation_angle_deg(rvec_true, pose[:3, :3]) < 1.0
    np.testing.assert_allclose(pose[:3, 3], tvec_true.reshape(3), atol=1e-4)
    assert reproj_px < 0.01


def test_ambiguity_gate_rejects_flip_ambiguous() -> None:
    """Invariant: a small tag seen near-frontal and far is a weak-perspective view
    where the flipped mirror pose reprojects nearly as well as the true one, so the
    gate must reject it (return None) at the production ratio 2.0. A noise-free
    projection is exact and would resolve, so we add tiny seeded symmetric pixel
    noise (0.3 px) to drop the runner-up/best reprojection ratio below 2.0 --
    the ambiguous view is constructed synthetically. Control: with the gate off
    (ratio_min=1.0) the solver still returns a pose, proving None is the gate's
    doing, not a solver failure."""
    marker_length_m = 0.1
    rvec_true = np.array([[0.03], [0.02], [0.0]], dtype=np.float64)  # near-frontal
    tvec_true = np.array([[0.0], [0.0], [8.0]], dtype=np.float64)  # 8 m, far/weak
    clean_corners = _project_marker_corners(rvec_true, tvec_true, marker_length_m)
    rng = np.random.default_rng(42)
    corners = clean_corners + rng.normal(0.0, 0.3, clean_corners.shape).astype(np.float32)

    # Both mirror candidates survive and their reproj errors are within 2x: ambiguous.
    candidates = estimate_marker_pose_candidates(corners, marker_length_m, _PINHOLE_K, _NO_DIST)
    errors = sorted(
        marker_reprojection_error(corners, marker_length_m, _PINHOLE_K, _NO_DIST, rvec, tvec)
        for rvec, tvec in candidates
    )
    assert len(errors) == 2
    assert errors[1] / errors[0] < 2.0

    assert (
        ambiguity_gated_pose(
            corners, marker_length_m, _PINHOLE_K, _NO_DIST, ambiguity_ratio_min=2.0
        )
        is None
    )
    # Gate off (ratio 1.0) => the same corners still yield a pose; only the gate rejects.
    assert (
        ambiguity_gated_pose(
            corners, marker_length_m, _PINHOLE_K, _NO_DIST, ambiguity_ratio_min=1.0
        )
        is not None
    )


# Fisheye/equidistant intrinsics: corners are undistorted into the pinhole K before
# solvePnP (and before reprojection scoring), so a fisheye-projected marker still
# recovers its pose. 4 equidistant coefficients (k1..k4).
_FISHEYE_K = np.array([[300.0, 0.0, 320.0], [0.0, 300.0, 240.0], [0.0, 0.0, 1.0]])
_FISHEYE_D = np.array([[0.05], [0.01], [0.0], [0.0]], dtype=np.float64)


def _fisheye_marker_corners(
    rvec: np.ndarray, tvec: np.ndarray, marker_length_m: float
) -> np.ndarray:
    """Distorted pixel corners of a marker at camera_optical <- marker (rvec, tvec)
    seen through the _FISHEYE_K/_FISHEYE_D equidistant model."""
    h = marker_length_m / 2.0
    obj = np.array(
        [[[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]]],
        dtype=np.float64,
    )
    img_pts, _jac = cv2.fisheye.projectPoints(obj, rvec, tvec, _FISHEYE_K, _FISHEYE_D.reshape(4))
    return img_pts.reshape(4, 2).astype(np.float32)


def test_estimate_marker_pose_undistorts_fisheye_corners() -> None:
    """Invariant: with an equidistant distortion model, corners are undistorted into
    the pinhole K before solvePnP, so a fisheye-projected marker recovers its true
    pose (rvec/tvec) and the reprojection error -- also computed in undistorted pixel
    space -- is ~0. Corners are synthesized noise-free, so recovery is exact."""
    marker_length_m = 0.2
    rvec_true = np.array([[0.1], [0.05], [-0.02]], dtype=np.float64)
    tvec_true = np.array([[0.1], [-0.1], [1.5]], dtype=np.float64)
    corners = _fisheye_marker_corners(rvec_true, tvec_true, marker_length_m)

    result = estimate_marker_pose(
        corners, marker_length_m, _FISHEYE_K, _FISHEYE_D, distortion_model="equidistant"
    )
    assert result is not None
    rvec, tvec = result
    np.testing.assert_allclose(rvec.reshape(3), rvec_true.reshape(3), atol=1e-3)
    np.testing.assert_allclose(tvec.reshape(3), tvec_true.reshape(3), atol=1e-3)
    reproj = marker_reprojection_error(
        corners, marker_length_m, _FISHEYE_K, _FISHEYE_D, rvec, tvec, distortion_model="equidistant"
    )
    assert reproj < 0.01


def test_fisheye_paths_require_at_least_four_coefficients() -> None:
    """Invariant: a fisheye model with fewer than 4 distortion coefficients is a
    mis-specified CameraInfo; both the pose solve and the reprojection scorer raise
    ValueError naming the deficit rather than undistorting with garbage coefficients."""
    corners = _fisheye_marker_corners(np.zeros((3, 1)), np.array([[0.0], [0.0], [1.5]]), 0.2)
    two_coeffs = np.zeros((2, 1), dtype=np.float64)
    with pytest.raises(ValueError, match="requires at least 4 coefficients"):
        estimate_marker_pose(corners, 0.2, _FISHEYE_K, two_coeffs, distortion_model="fisheye")
    with pytest.raises(ValueError, match="requires at least 4 coefficients"):
        marker_reprojection_error(
            corners,
            0.2,
            _FISHEYE_K,
            two_coeffs,
            np.zeros((3, 1)),
            np.array([[0.0], [0.0], [1.5]]),
            distortion_model="fisheye",
        )


def test_create_aruco_detector_rejects_unknown_dictionary() -> None:
    """Invariant: an unknown ArUco dictionary name fails loudly at construction with
    a ValueError naming the bad name, rather than an opaque AttributeError deeper in
    OpenCV -- a known name (DICT_APRILTAG_36h11) still builds a detector."""
    with pytest.raises(ValueError, match=r"Unknown ArUco dictionary 'DICT_NOT_A_REAL_ONE'"):
        create_aruco_detector("DICT_NOT_A_REAL_ONE")
    assert create_aruco_detector("DICT_APRILTAG_36h11") is not None


def test_estimate_and_gate_return_none_on_degenerate_corners() -> None:
    """Invariant: degenerate corner input (four coincident points -> non-finite
    solver output) yields no usable pose: estimate_marker_pose returns None and the
    ambiguity gate, seeing an empty candidate set, also returns None -- neither
    fabricates a pose from a collapsed quad."""
    coincident = np.full((4, 2), 100.0, dtype=np.float32)
    assert estimate_marker_pose_candidates(coincident, 0.1, _PINHOLE_K, _NO_DIST) == []
    assert estimate_marker_pose(coincident, 0.1, _PINHOLE_K, _NO_DIST) is None
    assert (
        ambiguity_gated_pose(coincident, 0.1, _PINHOLE_K, _NO_DIST, ambiguity_ratio_min=2.0) is None
    )
