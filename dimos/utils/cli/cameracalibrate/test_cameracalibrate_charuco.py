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

"""Tests for ChArUco support in the cameracalibrate CLI (calibrate + --check).

All synthetic: a ChArUco board's corners are projected through a KNOWN ground-truth K/D at
deterministic poses (seeded rng); the folder tests warp a rendered board so the REAL cv2.aruco
detector runs. No live capture, no replay, no display.
"""

from __future__ import annotations

import json
from pathlib import Path
import re

import cv2
import numpy as np
import pytest
from typer.testing import CliRunner

from dimos.utils.cli.cameracalibrate.cameracalibrate import (
    CharucoBoardSpec,
    _CharucoDetection,
    app,
    build_charuco_board,
    calibrate_from_frames_charuco,
    check_calibration_charuco,
    write_camera_info_yaml,
)

_WIDTH, _HEIGHT = 640, 480
_SQUARES_X, _SQUARES_Y = 8, 6
_SQUARE_SIZE_M = 0.03
_DICT = "DICT_4X4_50"
_K_TRUE = np.array(
    [[512.0, 0.0, 318.5], [0.0, 508.0, 242.3], [0.0, 0.0, 1.0]],
    dtype=np.float64,
)
_D_TRUE = np.array([-0.08, 0.02, 0.0, 0.0, 0.0], dtype=np.float64)  # small plumb_bob distortion
_D_ZERO = np.zeros(5, dtype=np.float64)


def _board_spec() -> CharucoBoardSpec:
    return build_charuco_board(
        dict_name=_DICT,
        squares_x=_SQUARES_X,
        squares_y=_SQUARES_Y,
        square_size_m=_SQUARE_SIZE_M,
        marker_ratio=0.8,
    )


def _synthetic_charuco_detections(
    spec: CharucoBoardSpec,
    *,
    K: np.ndarray = _K_TRUE,
    D: np.ndarray = _D_TRUE,
    count: int = 14,
    noise_px: float = 0.03,
    seed: int = 7,
) -> list[_CharucoDetection]:
    """Project the FULL ChArUco board through ``K``/``D`` at deterministic poses -> detections.

    Truth object points come straight from ``board.getChessboardCorners()`` and the pixels from
    ``cv2.projectPoints`` -- no re-implemented marker/corner math. All ids are present (full board),
    with a tiny seeded sub-pixel noise so the reprojection RMS is realistically nonzero.
    """
    object_points_m = np.asarray(spec.board.getChessboardCorners(), dtype=np.float32)
    n_corners = object_points_m.shape[0]
    ids = np.arange(n_corners, dtype=np.int32).reshape(-1, 1)
    rng = np.random.default_rng(seed)
    out: list[_CharucoDetection] = []
    while len(out) < count:
        rvec = rng.uniform(-0.25, 0.25, size=3).astype(np.float64)
        tvec = np.array(
            [rng.uniform(-0.05, 0.05), rng.uniform(-0.05, 0.05), rng.uniform(0.4, 0.6)],
            dtype=np.float64,
        )
        imgpts, _ = cv2.projectPoints(object_points_m, rvec, tvec, K, D)
        pts_px = np.asarray(imgpts, dtype=np.float64).reshape(-1, 2)
        if (
            pts_px[:, 0].min() < 10
            or pts_px[:, 0].max() > _WIDTH - 10
            or pts_px[:, 1].min() < 10
            or pts_px[:, 1].max() > _HEIGHT - 10
        ):
            continue
        pts_px += rng.normal(0.0, noise_px, size=pts_px.shape)
        out.append(
            _CharucoDetection(
                pts_px.astype(np.float32).reshape(-1, 1, 2),
                ids.copy(),
                object_points_m.copy(),
            )
        )
    return out


def _check_kwargs(K: np.ndarray, D: np.ndarray, spec: CharucoBoardSpec) -> dict[str, object]:
    return {
        "board_spec": spec,
        "K_deployed": K,
        "D_deployed": D,
        "distortion_model": "plumb_bob",
        "image_size_wh": (_WIDTH, _HEIGHT),
    }


# --- build_charuco_board validation -----------------------------------------------------------


def test_build_charuco_board_metric_object_points() -> None:
    """Board built with meter lengths -> object points span (squares-1) * square_size in meters."""
    spec = _board_spec()
    obj = np.asarray(spec.board.getChessboardCorners())
    assert obj.shape == ((_SQUARES_X - 1) * (_SQUARES_Y - 1), 3)
    # Interior corners span (squares_x - 1) gaps of one square along X.
    assert obj[:, 0].max() == pytest.approx((_SQUARES_X - 1) * _SQUARE_SIZE_M, rel=1e-5)
    assert spec.marker_size_m == pytest.approx(_SQUARE_SIZE_M * 0.8, rel=1e-9)


def test_build_charuco_board_rejects_unknown_dict() -> None:
    with pytest.raises(ValueError, match="unknown aruco dictionary"):
        build_charuco_board(dict_name="DICT_NOPE", squares_x=8, squares_y=6, square_size_m=0.03)


def test_build_charuco_board_rejects_marker_ge_square() -> None:
    with pytest.raises(ValueError, match="0 < marker < square"):
        build_charuco_board(
            dict_name=_DICT, squares_x=8, squares_y=6, square_size_m=0.03, marker_size_m=0.05
        )


# --- check_calibration_charuco: the required verdict flip -------------------------------------


def test_check_charuco_same_intrinsics_is_ok_and_low_rms() -> None:
    """Deployed K/D == ground-truth K/D -> low reprojection RMS and verdict OK."""
    spec = _board_spec()
    detections = _synthetic_charuco_detections(spec, count=14)
    result = check_calibration_charuco(detections, **_check_kwargs(_K_TRUE, _D_TRUE, spec))  # type: ignore[arg-type]

    assert result["verdict"] == "OK"
    assert result["n_frames_used"] == 14
    assert result["median_reproj_rms_px"] < 1.0
    assert result["drift"]["ran"] is True
    assert result["drift"]["drift_small"] is True


def test_check_charuco_perturbed_fx_is_degraded_and_high_rms() -> None:
    """Deployed fx off by 8% -> high reprojection RMS and verdict DEGRADED."""
    spec = _board_spec()
    detections = _synthetic_charuco_detections(spec, count=14)
    K_bad = _K_TRUE.copy()
    K_bad[0, 0] *= 1.08  # fx off by 8%

    result = check_calibration_charuco(detections, **_check_kwargs(K_bad, _D_TRUE, spec))  # type: ignore[arg-type]

    assert result["verdict"] == "DEGRADED"
    assert result["median_reproj_rms_px"] > 1.0
    assert result["recommendation"]
    assert "charuco" in result["recommendation"]


def test_check_charuco_verdict_flips_between_matched_and_perturbed() -> None:
    """Core requirement: same K/D passes, perturbed fx fails -> verdict flips."""
    spec = _board_spec()
    detections = _synthetic_charuco_detections(spec, count=14)
    K_bad = _K_TRUE.copy()
    K_bad[0, 0] *= 1.08

    ok = check_calibration_charuco(detections, **_check_kwargs(_K_TRUE, _D_TRUE, spec))  # type: ignore[arg-type]
    degraded = check_calibration_charuco(detections, **_check_kwargs(K_bad, _D_TRUE, spec))  # type: ignore[arg-type]

    assert ok["verdict"] == "OK"
    assert degraded["verdict"] == "DEGRADED"
    assert ok["verdict"] != degraded["verdict"]
    assert degraded["median_reproj_rms_px"] > ok["median_reproj_rms_px"]


def test_check_charuco_perturbed_intrinsics_reports_large_drift() -> None:
    """A fresh charuco calibration flags the 8% fx error as large drift."""
    spec = _board_spec()
    detections = _synthetic_charuco_detections(spec, count=14)
    K_bad = _K_TRUE.copy()
    K_bad[0, 0] *= 1.08

    result = check_calibration_charuco(detections, **_check_kwargs(K_bad, _D_TRUE, spec))  # type: ignore[arg-type]
    drift = result["drift"]
    assert drift["ran"] is True
    assert drift["drift_small"] is False
    assert drift["max_rel_intrinsics_drift"] > 0.05


def test_check_charuco_empty_detections_raises() -> None:
    with pytest.raises(ValueError, match="non-empty"):
        check_calibration_charuco([], **_check_kwargs(_K_TRUE, _D_TRUE, _board_spec()))  # type: ignore[arg-type]


# --- folder end-to-end over a rendered board (REAL cv2.aruco detection path) -------------------


def _synthetic_detectable_charuco_frames(
    spec: CharucoBoardSpec, count: int = 12
) -> list[np.ndarray]:
    """Warp a RENDERED ChArUco board to known poses so the real detector finds corners."""
    board_img = spec.board.generateImage((_WIDTH * 2, _HEIGHT * 2))
    detector = cv2.aruco.CharucoDetector(spec.board)
    flat_corners, flat_ids, _mc, _mi = detector.detectBoard(board_img)
    assert flat_ids is not None and len(flat_ids) >= 6
    src = np.asarray(flat_corners, dtype=np.float32).reshape(-1, 2)
    ids_flat = np.asarray(flat_ids, dtype=np.int32).reshape(-1)
    object_points_m = np.asarray(spec.board.getChessboardCorners(), dtype=np.float32)[ids_flat]

    rng = np.random.default_rng(3)
    frames: list[np.ndarray] = []
    for _ in range(600):
        if len(frames) >= count:
            break
        rvec = rng.uniform(-0.18, 0.18, size=3).astype(np.float64)
        tvec = np.array(
            [rng.uniform(-0.04, 0.04), rng.uniform(-0.04, 0.04), rng.uniform(0.45, 0.6)],
            dtype=np.float64,
        )
        imgpts, _ = cv2.projectPoints(object_points_m, rvec, tvec, _K_TRUE, _D_ZERO)
        dst = imgpts.reshape(-1, 2).astype(np.float32)
        if (
            dst[:, 0].min() < 20
            or dst[:, 0].max() > _WIDTH - 20
            or dst[:, 1].min() < 20
            or dst[:, 1].max() > _HEIGHT - 20
        ):
            continue
        H, _ = cv2.findHomography(src, dst, cv2.RANSAC, 2.0)
        if H is None:
            continue
        warped = cv2.warpPerspective(board_img, H, (_WIDTH, _HEIGHT), borderValue=255)
        cc, ci, _mc2, _mi2 = detector.detectBoard(warped)
        if ci is not None and len(ci) >= 6:
            frames.append(warped)
    assert len(frames) >= count, f"only rendered {len(frames)} detectable charuco frames"
    return frames[:count]


def test_calibrate_from_frames_charuco_recovers_intrinsics() -> None:
    """Full detect + calibrateCameraCharuco recovers K close to the ground-truth projection K."""
    spec = _board_spec()
    frames = _synthetic_detectable_charuco_frames(spec, count=12)
    result = calibrate_from_frames_charuco(frames, spec)

    assert result["n_used"] == 12
    assert result["image_size"] == (_WIDTH, _HEIGHT)
    K = np.asarray(result["K"], dtype=np.float64)
    # Zero-distortion warped renders -> fx/fy/cx/cy within a few percent of the true projection.
    assert K[0, 0] == pytest.approx(_K_TRUE[0, 0], rel=0.05)
    assert K[1, 1] == pytest.approx(_K_TRUE[1, 1], rel=0.05)


def _write_frames(images_dir: Path, frames: list[np.ndarray]) -> None:
    images_dir.mkdir(parents=True, exist_ok=True)
    for i, frame in enumerate(frames):
        assert cv2.imwrite(str(images_dir / f"frame_{i:02d}.png"), frame)


def _write_deployed_yaml(path: Path, K: np.ndarray, D: np.ndarray) -> None:
    write_camera_info_yaml(
        str(path),
        image_width=_WIDTH,
        image_height=_HEIGHT,
        camera_name="synthetic_charuco_deployed",
        K=K,
        D=D,
        distortion_model="plumb_bob",
    )


def _charuco_cli_args(images: Path, deployed: Path, out_json: Path) -> list[str]:
    return [
        "--check",
        "--board",
        "charuco",
        "--source",
        "folder",
        "--images",
        str(images),
        "--camera-info",
        str(deployed),
        "--dict",
        _DICT,
        "--squares-x",
        str(_SQUARES_X),
        "--squares-y",
        str(_SQUARES_Y),
        "--square-size-m",
        str(_SQUARE_SIZE_M),
        "--marker-ratio",
        "0.8",
        "--out",
        str(out_json),
        "--no-display",
    ]


def test_cli_check_charuco_folder_matched_yaml_is_ok(tmp_path: Path) -> None:
    """`--check --board charuco --source folder` end-to-end: matched deployed YAML -> OK + JSON."""
    spec = _board_spec()
    frames = _synthetic_detectable_charuco_frames(spec, count=12)
    images = tmp_path / "images"
    _write_frames(images, frames)
    deployed = tmp_path / "deployed.yaml"
    _write_deployed_yaml(deployed, _K_TRUE, _D_ZERO)
    out_json = tmp_path / "check.json"

    result = CliRunner().invoke(app, _charuco_cli_args(images, deployed, out_json))

    assert result.exit_code == 0, result.output
    assert "CALIBRATION CHECK: OK" in result.output
    payload = json.loads(out_json.read_text(encoding="utf-8"))
    assert payload["verdict"] == "OK"
    assert payload["n_frames_used"] == 12
    assert payload["median_reproj_rms_px"] < 1.0


def test_cli_check_charuco_folder_perturbed_yaml_is_degraded(tmp_path: Path) -> None:
    """A deployed YAML with fx off by 8% -> DEGRADED verdict from the charuco folder source."""
    spec = _board_spec()
    frames = _synthetic_detectable_charuco_frames(spec, count=12)
    images = tmp_path / "images"
    _write_frames(images, frames)
    K_bad = _K_TRUE.copy()
    K_bad[0, 0] *= 1.08
    deployed = tmp_path / "deployed_bad.yaml"
    _write_deployed_yaml(deployed, K_bad, _D_ZERO)
    out_json = tmp_path / "check.json"

    result = CliRunner().invoke(app, _charuco_cli_args(images, deployed, out_json))

    assert result.exit_code == 0, result.output
    assert "CALIBRATION CHECK: DEGRADED" in result.output
    payload = json.loads(out_json.read_text(encoding="utf-8"))
    assert payload["verdict"] == "DEGRADED"


def test_cli_calibrate_charuco_folder_writes_yaml(tmp_path: Path) -> None:
    """`calibrate --board charuco --source folder` writes a plumb_bob CameraInfo YAML."""
    spec = _board_spec()
    frames = _synthetic_detectable_charuco_frames(spec, count=12)
    images = tmp_path / "images"
    _write_frames(images, frames)
    out_yaml = tmp_path / "charuco_calib.yaml"

    result = CliRunner().invoke(
        app,
        [
            "--board",
            "charuco",
            "--source",
            "folder",
            "--images",
            str(images),
            "--dict",
            _DICT,
            "--squares-x",
            str(_SQUARES_X),
            "--squares-y",
            str(_SQUARES_Y),
            "--square-size-m",
            str(_SQUARE_SIZE_M),
            "--out",
            str(out_yaml),
            "--no-display",
        ],
    )

    assert result.exit_code == 0, result.output
    assert out_yaml.exists()
    import yaml

    payload = yaml.safe_load(out_yaml.read_text(encoding="utf-8"))
    assert payload["distortion_model"] == "plumb_bob"
    assert payload["image_width"] == _WIDTH


def test_cli_help_lists_charuco_flags() -> None:
    result = CliRunner().invoke(app, ["--help"])
    assert result.exit_code == 0
    output_plain = re.sub(r"\x1b\[[0-9;]*m", "", result.output)
    # Rich truncates long option names at narrow widths, so assert on truncation-safe substrings.
    for flag in ["--board", "--dict", "--squares-x", "--squares-y", "--marker-rat"]:
        assert flag in output_plain, output_plain


def test_cli_charuco_check_requires_squares(tmp_path: Path) -> None:
    """--board charuco without --squares-x/-y is a clean BadParameter, not a crash."""
    deployed = tmp_path / "deployed.yaml"
    _write_deployed_yaml(deployed, _K_TRUE, _D_ZERO)
    result = CliRunner().invoke(
        app,
        [
            "--check",
            "--board",
            "charuco",
            "--source",
            "folder",
            "--images",
            str(tmp_path),
            "--camera-info",
            str(deployed),
            "--square-size-m",
            str(_SQUARE_SIZE_M),
            "--no-display",
        ],
    )
    assert result.exit_code != 0
    output_plain = re.sub(r"\x1b\[[0-9;]*m", "", result.output).replace("\n", " ")
    output_plain = re.sub(r"\s+", " ", output_plain)
    assert "squares" in output_plain, output_plain
