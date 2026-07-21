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

"""Known-truth tests for the ChArUco + fisheye (equidistant / Kannala-Brandt) calibration path.

Hermetic and deterministic: a ChArUco board's chessboard corners (object points in meters, taken
straight from ``board.getChessboardCorners()``) are projected through a KNOWN ground-truth fisheye
``K``/``D`` at seeded camera poses via ``cv2.fisheye.projectPoints``. Those per-view
(objectPoints, imagePoints) correspondences are fed through the REAL function the fix added,
``_calibrate_charuco_fisheye`` -- no re-implemented solver -- and the recovered intrinsics are
asserted back against the constructed truth. The routing/emit tests warp a RENDERED board so the
real ``cv2.aruco`` detector runs end-to-end. No live capture, no replay, no display.
"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import pytest
from typer.testing import CliRunner
import yaml

from dimos.utils.cli.cameracalibrate.cameracalibrate import (
    _MIN_CHARUCO_CORNERS_FISHEYE,
    CharucoBoardSpec,
    _calibrate_charuco_fisheye,
    _CharucoDetection,
    app,
    build_charuco_board,
    calibrate_from_frames_charuco,
)

_WIDTH, _HEIGHT = 640, 480
_SQUARES_X, _SQUARES_Y = 8, 6
_SQUARE_SIZE_M = 0.03
_MARKER_RATIO = 0.8
_DICT = "DICT_4X4_50"

# Ground-truth fisheye intrinsics we recover. fx == fy (square pixels), principal point at center.
_K_TRUE = np.array(
    [[400.0, 0.0, 320.0], [0.0, 400.0, 240.0], [0.0, 0.0, 1.0]],
    dtype=np.float64,
)
# 4-coeff Kannala-Brandt distortion (k1..k4); small, physical fisheye barrel.
_D_TRUE = np.array([0.02, -0.01, 0.004, -0.001], dtype=np.float64)
_D_ZERO_PLUMB = np.zeros(5, dtype=np.float64)


def _board_spec() -> CharucoBoardSpec:
    return build_charuco_board(
        dict_name=_DICT,
        squares_x=_SQUARES_X,
        squares_y=_SQUARES_Y,
        square_size_m=_SQUARE_SIZE_M,
        marker_ratio=_MARKER_RATIO,
    )


def _synthetic_charuco_fisheye_detections(
    spec: CharucoBoardSpec,
    *,
    K: np.ndarray = _K_TRUE,
    D: np.ndarray = _D_TRUE,
    count: int = 15,
    seed: int = 0,
) -> list[_CharucoDetection]:
    """Project the board's chessboard corners through a KNOWN fisheye K/D at seeded poses.

    Object points come straight from ``board.getChessboardCorners()`` (meters); pixels come from
    ``cv2.fisheye.projectPoints`` -- no re-implemented projection or marker math. Every fourth view
    is a PARTIAL board (a random id subset, still above the min-corner gate) so the per-view
    variable-N handling in ``_calibrate_charuco_fisheye`` is exercised. No pixel noise: the scene is
    constructed known truth, so recovery is essentially exact and the asserts can be tight.
    """
    obj_all = np.asarray(spec.board.getChessboardCorners(), dtype=np.float64)
    n_corners = obj_all.shape[0]
    all_ids = np.arange(n_corners, dtype=np.int32)
    rng = np.random.default_rng(seed)
    out: list[_CharucoDetection] = []
    while len(out) < count:
        rvec = rng.uniform(-0.35, 0.35, size=3).reshape(1, 1, 3)
        tvec = np.array(
            [rng.uniform(-0.06, 0.06), rng.uniform(-0.05, 0.05), rng.uniform(0.32, 0.55)],
            dtype=np.float64,
        ).reshape(1, 1, 3)
        # Every 4th view: partial board (subset of ids) to test variable-N correspondence handling.
        if len(out) % 4 == 3:
            sel = np.sort(rng.choice(n_corners, size=20, replace=False)).astype(np.int32)
        else:
            sel = all_ids
        objp = obj_all[sel].reshape(-1, 1, 3)
        imgpts, _ = cv2.fisheye.projectPoints(objp, rvec, tvec, K, D)
        px = np.asarray(imgpts, dtype=np.float64).reshape(-1, 2)
        # Keep the whole (partial) board strictly in-frame so the solve stays well-conditioned.
        if px[:, 0].min() < 5 or px[:, 0].max() > _WIDTH - 5:
            continue
        if px[:, 1].min() < 5 or px[:, 1].max() > _HEIGHT - 5:
            continue
        out.append(
            _CharucoDetection(
                px.astype(np.float32).reshape(-1, 1, 2),
                sel.reshape(-1, 1),
                obj_all[sel].astype(np.float32),
            )
        )
    return out


# --- core known-truth recovery through the real fisheye function ------------------------------


def test_calibrate_charuco_fisheye_recovers_known_intrinsics() -> None:
    """Charuco+fisheye recovers the CONSTRUCTED fisheye K/D from projected correspondences.

    Feeds known-truth per-view (objectPoints, imagePoints) through ``_calibrate_charuco_fisheye``
    (the real solver the fix added) and asserts the recovered fx/fy/cx/cy and the 4 Kannala-Brandt
    coefficients land on the constructed truth. Measured on this seed: rms ~1e-5 px, fx/fy err
    <1e-3, cx/cy err <0.1 px, |D err| <1e-5 -- the tolerances below are deliberately looser.
    """
    spec = _board_spec()
    detections = _synthetic_charuco_fisheye_detections(spec, count=15)

    rms, K_est, D_est = _calibrate_charuco_fisheye(detections, (_WIDTH, _HEIGHT))

    assert rms < 0.05, f"noiseless fisheye scene should reproject near-exactly, got rms={rms}"
    # Focal length within 0.1% and principal point within half a pixel of the constructed truth.
    assert K_est[0, 0] == pytest.approx(_K_TRUE[0, 0], rel=1e-3)
    assert K_est[1, 1] == pytest.approx(_K_TRUE[1, 1], rel=1e-3)
    assert abs(K_est[0, 2] - _K_TRUE[0, 2]) < 0.5
    assert abs(K_est[1, 2] - _K_TRUE[1, 2]) < 0.5

    D_flat = np.asarray(D_est, dtype=np.float64).ravel()
    assert D_flat.shape == (4,), "fisheye (Kannala-Brandt) emits exactly 4 coefficients"
    # Each k1..k4 within 5e-3 absolute of the constructed truth [0.02, -0.01, 0.004, -0.001].
    assert np.allclose(D_flat, _D_TRUE, atol=5e-3), f"D recovery {D_flat} vs truth {_D_TRUE}"


def test_calibrate_charuco_fisheye_thin_views_raise_got_vs_want() -> None:
    """Views under the min-corner gate leave nothing to solve -> got-vs-want ``ValueError``.

    Fisheye must constrain 4 distortion + 6 extrinsic DOF per view, so thin partial boards are
    dropped; if none survive, the boundary raises loudly rather than handing an empty list to cv2.
    """
    spec = _board_spec()
    obj_all = np.asarray(spec.board.getChessboardCorners(), dtype=np.float64)
    thin_n = _MIN_CHARUCO_CORNERS_FISHEYE - 1  # one short of the gate on every view
    rng = np.random.default_rng(1)
    thin: list[_CharucoDetection] = []
    for _ in range(4):
        sel = np.sort(rng.choice(obj_all.shape[0], size=thin_n, replace=False)).astype(np.int32)
        px = np.zeros((thin_n, 1, 2), dtype=np.float32)  # positions irrelevant; the gate is on N
        thin.append(_CharucoDetection(px, sel.reshape(-1, 1), obj_all[sel].astype(np.float32)))

    with pytest.raises(ValueError, match="none with enough corners"):
        _calibrate_charuco_fisheye(thin, (_WIDTH, _HEIGHT))


# --- rendered-board end-to-end: model routing (real cv2.aruco detection) ----------------------


def _rendered_detectable_charuco_frames(
    spec: CharucoBoardSpec, count: int = 12
) -> list[np.ndarray]:
    """Warp a RENDERED ChArUco board to seeded poses so the real detector recovers corners.

    Projection uses a near-pinhole K with zero distortion (this exercises the DETECT + solver
    routing, not fisheye recovery accuracy). Returns frames the real ``CharucoDetector`` accepts.
    """
    k_render = np.array(
        [[512.0, 0.0, 318.5], [0.0, 508.0, 242.3], [0.0, 0.0, 1.0]], dtype=np.float64
    )
    board_img = spec.board.generateImage((_WIDTH * 2, _HEIGHT * 2))
    detector = cv2.aruco.CharucoDetector(spec.board)
    flat_corners, flat_ids, _mc, _mi = detector.detectBoard(board_img)
    assert flat_ids is not None and len(flat_ids) >= 6
    src = np.asarray(flat_corners, dtype=np.float32).reshape(-1, 2)
    ids_flat = np.asarray(flat_ids, dtype=np.int32).reshape(-1)
    object_points_m = np.asarray(spec.board.getChessboardCorners(), dtype=np.float32)[ids_flat]

    rng = np.random.default_rng(3)
    frames: list[np.ndarray] = []
    for _ in range(800):
        if len(frames) >= count:
            break
        rvec = rng.uniform(-0.18, 0.18, size=3).astype(np.float64)
        tvec = np.array(
            [rng.uniform(-0.04, 0.04), rng.uniform(-0.04, 0.04), rng.uniform(0.45, 0.6)],
            dtype=np.float64,
        )
        imgpts, _ = cv2.projectPoints(object_points_m, rvec, tvec, k_render, _D_ZERO_PLUMB)
        dst = imgpts.reshape(-1, 2).astype(np.float32)
        if dst[:, 0].min() < 20 or dst[:, 0].max() > _WIDTH - 20:
            continue
        if dst[:, 1].min() < 20 or dst[:, 1].max() > _HEIGHT - 20:
            continue
        homography, _ = cv2.findHomography(src, dst, cv2.RANSAC, 2.0)
        if homography is None:
            continue
        warped = cv2.warpPerspective(board_img, homography, (_WIDTH, _HEIGHT), borderValue=255)
        cc, ci, _mc2, _mi2 = detector.detectBoard(warped)
        if ci is not None and len(ci) >= 6:
            frames.append(warped)
    assert len(frames) >= count, f"only rendered {len(frames)} detectable charuco frames"
    return frames[:count]


def test_charuco_fisheye_routes_to_equidistant_solver_four_coeffs() -> None:
    """``distortion_model=fisheye`` routes the charuco path to the 4-coeff (equidistant) solver."""
    spec = _board_spec()
    frames = _rendered_detectable_charuco_frames(spec, count=12)

    result = calibrate_from_frames_charuco(frames, spec, distortion_model="fisheye")

    D = np.asarray(result["D"], dtype=np.float64).ravel()
    assert D.shape == (4,), "fisheye routing must emit 4 Kannala-Brandt coefficients, not 5"
    assert result["n_used"] == 12
    assert result["image_size"] == (_WIDTH, _HEIGHT)


def test_charuco_plumb_bob_default_is_unchanged_five_coeffs() -> None:
    """Regression: the default (plumb_bob) charuco path still emits 5 coeffs via calibrateCameraCharuco."""
    spec = _board_spec()
    frames = _rendered_detectable_charuco_frames(spec, count=12)

    default = calibrate_from_frames_charuco(frames, spec)
    explicit = calibrate_from_frames_charuco(frames, spec, distortion_model="plumb_bob")

    d_default = np.asarray(default["D"], dtype=np.float64).ravel()
    assert d_default.shape == (5,), "plumb_bob charuco emits 5 radial-tangential coefficients"
    # Default and explicit plumb_bob take the identical solver path -> identical intrinsics.
    assert np.allclose(default["K"], explicit["K"])
    assert np.allclose(default["D"], explicit["D"])


# --- CLI end-to-end: charuco + --distortion-model fisheye writes an "equidistant" YAML --------


def _write_frames(images_dir: Path, frames: list[np.ndarray]) -> None:
    images_dir.mkdir(parents=True, exist_ok=True)
    for i, frame in enumerate(frames):
        assert cv2.imwrite(str(images_dir / f"frame_{i:02d}.png"), frame)


def _cli_calibrate_args(images: Path, out_yaml: Path, model: str) -> list[str]:
    return [
        "--board",
        "charuco",
        "--source",
        "folder",
        "--images",
        str(images),
        "--distortion-model",
        model,
        "--dict",
        _DICT,
        "--squares-x",
        str(_SQUARES_X),
        "--squares-y",
        str(_SQUARES_Y),
        "--square-size-m",
        str(_SQUARE_SIZE_M),
        "--marker-ratio",
        str(_MARKER_RATIO),
        "--out",
        str(out_yaml),
        "--no-display",
    ]


def test_cli_calibrate_charuco_fisheye_emits_equidistant_yaml(tmp_path: Path) -> None:
    """`calibrate --board charuco --distortion-model fisheye` writes ``equidistant`` + 4 coeffs."""
    spec = _board_spec()
    frames = _rendered_detectable_charuco_frames(spec, count=12)
    images = tmp_path / "images"
    _write_frames(images, frames)
    out_yaml = tmp_path / "charuco_fisheye.yaml"

    result = CliRunner().invoke(app, _cli_calibrate_args(images, out_yaml, "fisheye"))

    assert result.exit_code == 0, result.output
    assert out_yaml.exists()
    payload = yaml.safe_load(out_yaml.read_text(encoding="utf-8"))
    assert payload["distortion_model"] == "equidistant"
    assert payload["distortion_coefficients"]["cols"] == 4
    assert len(payload["distortion_coefficients"]["data"]) == 4
    assert payload["image_width"] == _WIDTH


def test_cli_calibrate_charuco_plumb_bob_emits_plumb_bob_yaml(tmp_path: Path) -> None:
    """Regression: charuco default model still writes a ``plumb_bob`` YAML with 5 coeffs."""
    spec = _board_spec()
    frames = _rendered_detectable_charuco_frames(spec, count=12)
    images = tmp_path / "images"
    _write_frames(images, frames)
    out_yaml = tmp_path / "charuco_plumb.yaml"

    result = CliRunner().invoke(app, _cli_calibrate_args(images, out_yaml, "plumb_bob"))

    assert result.exit_code == 0, result.output
    payload = yaml.safe_load(out_yaml.read_text(encoding="utf-8"))
    assert payload["distortion_model"] == "plumb_bob"
    assert payload["distortion_coefficients"]["cols"] == 5
