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

"""Tests for the --check (calibration-health) mode of the cameracalibrate CLI.

All synthetic: a checkerboard is projected through a KNOWN ground-truth K/D at deterministic
poses (seeded rng); no live capture, no replay, no display.
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
    _board_object_points,
    app,
    check_calibration,
    find_chessboard_corners,
    write_camera_info_yaml,
)

_COLS, _ROWS = 9, 6
_WIDTH, _HEIGHT = 640, 480
_SQUARE_SIZE_M = 0.025
_K_TRUE = np.array(
    [[512.0, 0.0, 318.5], [0.0, 508.0, 242.3], [0.0, 0.0, 1.0]],
    dtype=np.float64,
)
_D_TRUE = np.array([-0.08, 0.02, 0.0, 0.0, 0.0], dtype=np.float64)  # small plumb_bob distortion


def _synthetic_board_corners(
    *,
    count: int = 14,
    K: np.ndarray = _K_TRUE,
    D: np.ndarray = _D_TRUE,
    noise_px: float = 0.03,
    seed: int = 7,
) -> list[np.ndarray]:
    """Project the checkerboard through ``K``/``D`` at deterministic poses -> detected corners.

    Returns one ``(cols * rows, 1, 2)`` corner array per view. A tiny seeded sub-pixel noise makes
    the reprojection RMS realistically nonzero without a real detector.
    """
    objp_m = _board_object_points(_COLS, _ROWS, _SQUARE_SIZE_M)
    rng = np.random.default_rng(seed)
    views: list[np.ndarray] = []
    while len(views) < count:
        rvec = rng.uniform(-0.25, 0.25, size=3).astype(np.float64)
        tvec = np.array(
            [rng.uniform(-0.05, 0.05), rng.uniform(-0.05, 0.05), rng.uniform(0.4, 0.6)],
            dtype=np.float64,
        )
        imgpts, _ = cv2.projectPoints(objp_m, rvec, tvec, K, D)
        pts_px = np.asarray(imgpts, dtype=np.float64).reshape(-1, 2)
        # Keep the whole board inside the frame so solvePnP is well-conditioned.
        if (
            pts_px[:, 0].min() < 10
            or pts_px[:, 0].max() > _WIDTH - 10
            or pts_px[:, 1].min() < 10
            or pts_px[:, 1].max() > _HEIGHT - 10
        ):
            continue
        pts_px += rng.normal(0.0, noise_px, size=pts_px.shape)
        views.append(pts_px.astype(np.float32).reshape(-1, 1, 2))
    return views


def _camera_info_dict(K: np.ndarray, D: np.ndarray) -> dict[str, object]:
    return {
        "K_deployed": K,
        "D_deployed": D,
        "distortion_model": "plumb_bob",
        "image_size_wh": (_WIDTH, _HEIGHT),
        "cols": _COLS,
        "rows": _ROWS,
        "square_size_m": _SQUARE_SIZE_M,
    }


def test_check_calibration_same_intrinsics_is_ok_and_low_rms() -> None:
    """Deployed K/D == ground-truth K/D -> low reprojection RMS and verdict OK."""
    views = _synthetic_board_corners(count=14)
    result = check_calibration(views, **_camera_info_dict(_K_TRUE, _D_TRUE))  # type: ignore[arg-type]

    assert result["verdict"] == "OK"
    assert result["n_frames_used"] == 14
    assert result["median_reproj_rms_px"] < 1.0
    assert result["drift"]["ran"] is True
    assert result["drift"]["drift_small"] is True


def test_check_calibration_perturbed_fx_is_degraded_and_high_rms() -> None:
    """Deployed fx off by 8% -> high reprojection RMS and verdict DEGRADED."""
    views = _synthetic_board_corners(count=14)
    K_bad = _K_TRUE.copy()
    K_bad[0, 0] *= 1.08  # fx off by 8%

    result = check_calibration(views, **_camera_info_dict(K_bad, _D_TRUE))  # type: ignore[arg-type]

    assert result["verdict"] == "DEGRADED"
    assert result["median_reproj_rms_px"] > 1.0
    assert result["recommendation"]  # non-empty recalibrate hint


def test_check_verdict_flips_between_matched_and_perturbed_intrinsics() -> None:
    """The core requirement: same K/D passes, perturbed K/D fails -> verdict flips."""
    views = _synthetic_board_corners(count=14)
    K_bad = _K_TRUE.copy()
    K_bad[0, 0] *= 1.08

    ok = check_calibration(views, **_camera_info_dict(_K_TRUE, _D_TRUE))  # type: ignore[arg-type]
    degraded = check_calibration(views, **_camera_info_dict(K_bad, _D_TRUE))  # type: ignore[arg-type]

    assert ok["verdict"] == "OK"
    assert degraded["verdict"] == "DEGRADED"
    assert ok["verdict"] != degraded["verdict"]
    assert degraded["median_reproj_rms_px"] > ok["median_reproj_rms_px"]


def test_check_perturbed_intrinsics_reports_large_drift() -> None:
    """Drift block flags the 8% fx error even if the RMS gate were relaxed."""
    views = _synthetic_board_corners(count=14)
    K_bad = _K_TRUE.copy()
    K_bad[0, 0] *= 1.08

    result = check_calibration(views, **_camera_info_dict(K_bad, _D_TRUE))  # type: ignore[arg-type]
    drift = result["drift"]
    assert drift["ran"] is True
    assert drift["drift_small"] is False
    assert drift["max_rel_intrinsics_drift"] > 0.05


def test_check_drift_skipped_when_too_few_frames() -> None:
    """With fewer than the minimum frames, drift is skipped but the RMS verdict still holds."""
    views = _synthetic_board_corners(count=2)
    result = check_calibration(views, run_drift=True, **_camera_info_dict(_K_TRUE, _D_TRUE))  # type: ignore[arg-type]

    assert result["drift"]["ran"] is False
    assert "need >=" in result["drift"]["reason"]
    assert result["verdict"] == "OK"  # RMS still low; drift-unavailable does not force DEGRADED


def test_check_empty_views_raises() -> None:
    with pytest.raises(ValueError, match="non-empty"):
        check_calibration([], **_camera_info_dict(_K_TRUE, _D_TRUE))  # type: ignore[arg-type]


# --- CLI end-to-end over a recorded folder (real ingestion + detection path) ---


def _synthetic_chessboard_gray(square_px: int = 40) -> np.ndarray:
    """Binary chessboard; ``cols`` x ``rows`` inner corners need ``cols+1`` x ``rows+1`` squares."""
    img = np.full((_HEIGHT, _WIDTH), 255, dtype=np.uint8)
    board_w = (_COLS + 1) * square_px
    board_h = (_ROWS + 1) * square_px
    ox = (_WIDTH - board_w) // 2
    oy = (_HEIGHT - board_h) // 2
    for yi in range(_ROWS + 1):
        for xi in range(_COLS + 1):
            color = 0 if (xi + yi) % 2 == 0 else 255
            x0 = ox + xi * square_px
            y0 = oy + yi * square_px
            img[y0 : y0 + square_px, x0 : x0 + square_px] = color
    return img


# Homography warping is a pinhole operation, so the folder frames are rendered with ZERO
# distortion (matching the existing calibrate synthetic test); the deployed YAML under test
# uses the same zero distortion.
_D_ZERO = np.zeros(5, dtype=np.float64)


def _synthetic_detectable_frames(count: int = 12) -> tuple[list[np.ndarray], np.ndarray]:
    """Warp a rendered board to known poses so the real detector finds corners (folder source)."""
    gray_flat = _synthetic_chessboard_gray()
    corners_flat = find_chessboard_corners(gray_flat, _COLS, _ROWS)
    assert corners_flat is not None
    src = corners_flat.reshape(-1, 2).astype(np.float32)

    objp_m = _board_object_points(_COLS, _ROWS, _SQUARE_SIZE_M)
    rng = np.random.default_rng(3)
    frames: list[np.ndarray] = []
    for _ in range(400):
        if len(frames) >= count:
            break
        rvec = rng.uniform(-0.2, 0.2, size=3).astype(np.float64)
        tvec = np.array(
            [rng.uniform(-0.04, 0.04), rng.uniform(-0.04, 0.04), rng.uniform(0.4, 0.55)],
            dtype=np.float64,
        )
        imgpts, _ = cv2.projectPoints(objp_m, rvec, tvec, _K_TRUE, _D_ZERO)
        dst = imgpts.reshape(-1, 2).astype(np.float32)
        H, _ = cv2.findHomography(src, dst, cv2.RANSAC, 2.0)
        if H is None:
            continue
        warped = cv2.warpPerspective(gray_flat, H, (_WIDTH, _HEIGHT))
        if find_chessboard_corners(warped, _COLS, _ROWS) is not None:
            frames.append(warped)
    assert len(frames) >= count
    return frames[:count], _K_TRUE


def _write_frames(images_dir: Path, frames: list[np.ndarray]) -> None:
    images_dir.mkdir(parents=True, exist_ok=True)
    for i, frame in enumerate(frames):
        assert cv2.imwrite(str(images_dir / f"frame_{i:02d}.png"), frame)


def _write_deployed_yaml(path: Path, K: np.ndarray, D: np.ndarray) -> None:
    write_camera_info_yaml(
        str(path),
        image_width=_WIDTH,
        image_height=_HEIGHT,
        camera_name="synthetic_deployed",
        K=K,
        D=D,
        distortion_model="plumb_bob",
    )


def test_cli_check_folder_matched_yaml_is_ok_and_writes_json(tmp_path: Path) -> None:
    """`calibrate --check --source folder` end-to-end: matched deployed YAML -> OK + JSON."""
    frames, K_true = _synthetic_detectable_frames(count=12)
    images = tmp_path / "images"
    _write_frames(images, frames)
    deployed = tmp_path / "deployed.yaml"
    _write_deployed_yaml(deployed, K_true, _D_ZERO)
    out_json = tmp_path / "check.json"

    result = CliRunner().invoke(
        app,
        [
            "--check",
            "--source", "folder",
            "--images", str(images),
            "--camera-info", str(deployed),
            "--cols", "9",
            "--rows", "6",
            "--square-size-m", "0.025",
            "--out", str(out_json),
            "--no-display",
        ],
    )

    assert result.exit_code == 0, result.output
    assert "CALIBRATION CHECK: OK" in result.output
    assert out_json.exists()
    payload = json.loads(out_json.read_text(encoding="utf-8"))
    assert payload["verdict"] == "OK"
    assert payload["n_frames_used"] == 12
    assert payload["median_reproj_rms_px"] < 1.0
    assert payload["provenance"]["source"] == "folder"
    assert payload["provenance"]["cols"] == 9


def test_cli_check_folder_perturbed_yaml_is_degraded(tmp_path: Path) -> None:
    """A deployed YAML with fx off by 8% -> DEGRADED verdict from the folder source."""
    frames, K_true = _synthetic_detectable_frames(count=12)
    images = tmp_path / "images"
    _write_frames(images, frames)
    K_bad = K_true.copy()
    K_bad[0, 0] *= 1.08
    deployed = tmp_path / "deployed_bad.yaml"
    _write_deployed_yaml(deployed, K_bad, _D_ZERO)
    out_json = tmp_path / "check.json"

    result = CliRunner().invoke(
        app,
        [
            "--check",
            "--source", "folder",
            "--images", str(images),
            "--camera-info", str(deployed),
            "--cols", "9",
            "--rows", "6",
            "--square-size-m", "0.025",
            "--out", str(out_json),
            "--no-display",
        ],
    )

    assert result.exit_code == 0, result.output
    assert "CALIBRATION CHECK: DEGRADED" in result.output
    assert "recalibrate this unit" in result.output
    payload = json.loads(out_json.read_text(encoding="utf-8"))
    assert payload["verdict"] == "DEGRADED"


def test_cli_help_lists_check_flags() -> None:
    result = CliRunner().invoke(app, ["--help"])
    assert result.exit_code == 0
    output_plain = re.sub(r"\x1b\[[0-9;]*m", "", result.output)
    # Rich truncates long option names at narrow widths (e.g. "--rms-threshold-…"), so assert on
    # substrings that survive truncation.
    for flag in ["--check", "--camera-info", "--rms-threshold", "--no-drift"]:
        assert flag in output_plain, output_plain
