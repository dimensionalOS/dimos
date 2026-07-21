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

"""Tests for three cameracalibrate features: the frame-diversity gate, separate frame saving,
and the interactive recalibrate-from-captured-frames offer on a DEGRADED --check.

All synthetic and deterministic: constructed corner arrays for the pure gate, seeded projected
frames for the folder-source --check path. No live capture, no display.
"""

from __future__ import annotations

from pathlib import Path
import shutil
import tempfile

import cv2
import numpy as np
import pytest

from dimos.utils.cli.cameracalibrate.cameracalibrate import (
    CalibrationCheckRunResultDict,
    _board_object_points,
    find_chessboard_corners,
    is_frame_novel,
    load_frames_from_folder,
    run_check,
    run_check_report,
    save_frames_to_dir,
    write_camera_info_yaml,
    write_recalibration_from_check,
)

_COLS, _ROWS = 9, 6
_WIDTH, _HEIGHT = 640, 480
_SQUARE_SIZE_M = 0.025
_K_TRUE = np.array(
    [[512.0, 0.0, 318.5], [0.0, 508.0, 242.3], [0.0, 0.0, 1.0]],
    dtype=np.float64,
)
_D_ZERO = np.zeros(5, dtype=np.float64)


def _grid_corners(dx: float = 0.0, dy: float = 0.0) -> np.ndarray:
    """A deterministic 3x3 board-corner grid (shape (9, 1, 2)) shifted by (dx, dy) pixels."""
    pts = [[[float(x) + dx, float(y) + dy]] for y in (100.0, 120.0, 140.0) for x in (100.0, 120.0, 140.0)]
    return np.asarray(pts, dtype=np.float32)


# --- Feature 1: frame-diversity gate (pure) -----------------------------------


def test_is_frame_novel_accepts_displaced_and_rejects_near_duplicate() -> None:
    """A frame shifted well past the threshold is novel; one barely shifted is a near-duplicate."""
    base = _grid_corners()
    near_duplicate = _grid_corners(dx=2.0)  # mean per-corner displacement 2.0 px < 8.0
    displaced = _grid_corners(dx=20.0)  # mean per-corner displacement 20.0 px >= 8.0

    assert is_frame_novel(displaced, [base], 8.0) is True
    assert is_frame_novel(near_duplicate, [base], 8.0) is False


def test_is_frame_novel_first_frame_and_disabled_gate_are_always_novel() -> None:
    """The first frame (empty history) and any frame under a <=0 threshold are always kept."""
    base = _grid_corners()
    near_duplicate = _grid_corners(dx=2.0)

    assert is_frame_novel(base, [], 8.0) is True  # nothing accepted yet
    assert is_frame_novel(near_duplicate, [base], 0.0) is True  # gate disabled


def test_is_frame_novel_ignores_uncomparable_corner_counts() -> None:
    """An accepted frame with a different corner count cannot prove a duplicate -> treated novel."""
    base = _grid_corners()
    partial = base[:4].copy()  # 4 corners vs 9 -> shapes differ, not comparable
    # Only the uncomparable frame is in history, so the candidate is novel despite being identical.
    assert is_frame_novel(base, [partial], 8.0) is True


# --- Feature 2: save accepted frames separately -------------------------------


def test_save_frames_to_dir_count_matches_and_reloads() -> None:
    """save_frames_to_dir writes exactly one image per frame, re-loadable in capture order."""
    tmp_dir = Path(tempfile.mkdtemp())
    try:
        frames = [np.full((16, 24, 3), i * 10, dtype=np.uint8) for i in range(5)]
        frames_out = tmp_dir / "frames"

        paths = save_frames_to_dir(frames, frames_out)

        assert len(paths) == len(frames)
        written = sorted(frames_out.iterdir())
        assert len(written) == len(frames)
        assert [p.name for p in written] == [f"frame_{i:03d}.png" for i in range(5)]
        reloaded = load_frames_from_folder(str(frames_out))
        assert len(reloaded) == len(frames)
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)


# --- Feature 3: recalibrate-from-captured-frames on DEGRADED --check ----------


def _synthetic_detectable_frames(count: int = 12) -> list[np.ndarray]:
    """Warp a rendered board to seeded poses so the real detector finds corners (folder source)."""
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
    return frames[:count]


def _synthetic_chessboard_gray(square_px: int = 40) -> np.ndarray:
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


def _write_frames(images_dir: Path, frames: list[np.ndarray]) -> None:
    images_dir.mkdir(parents=True, exist_ok=True)
    for i, frame in enumerate(frames):
        assert cv2.imwrite(str(images_dir / f"src_{i:02d}.png"), frame)


def _write_degraded_deployed_yaml(tmp_path: Path) -> Path:
    """Write a deployed CameraInfo whose fx is 8% off _K_TRUE so --check verdicts DEGRADED."""
    K_bad = _K_TRUE.copy()
    K_bad[0, 0] *= 1.08  # fx off by 8% -> DEGRADED
    deployed = tmp_path / "deployed_bad.yaml"
    write_camera_info_yaml(
        str(deployed),
        image_width=_WIDTH,
        image_height=_HEIGHT,
        camera_name="deployed",
        K=K_bad,
        D=_D_ZERO,
        distortion_model="plumb_bob",
    )
    return deployed


def _run_check_folder_degraded(tmp_path: Path) -> CalibrationCheckRunResultDict:
    """Run the real folder-source --check pipeline on seeded frames against a DEGRADED deployed YAML."""
    frames = _synthetic_detectable_frames(count=12)
    images = tmp_path / "images"
    _write_frames(images, frames)
    deployed = _write_degraded_deployed_yaml(tmp_path)
    return run_check(
        source="folder",
        device_index=0,
        images=images,
        topic=None,
        topic_timeout_sec=1.0,
        cols=_COLS,
        rows=_ROWS,
        square_size_m=_SQUARE_SIZE_M,
        camera_info_path=deployed,
        rms_threshold_px=1.0,
        drift_threshold_frac=0.05,
        run_drift=True,
        out=tmp_path / "check.json",
        target_count=len(list(images.iterdir())),
        no_display=True,
    )


def _run_check_report_folder(images: Path, deployed: Path, out: Path) -> None:
    """Drive the CLI-boundary report on the folder source (no display, no flag)."""
    run_check_report(
        source="folder",
        device_index=0,
        images=images,
        topic=None,
        topic_timeout_sec=1.0,
        cols=_COLS,
        rows=_ROWS,
        square_size_m=_SQUARE_SIZE_M,
        camera_info=deployed,
        rms_threshold_px=1.0,
        drift_threshold_frac=0.05,
        check_drift=True,
        out=out,
        target_count=len(list(images.iterdir())),
        no_display=True,
    )


def test_write_recalibration_from_check_emits_loadable_fresh_yaml(tmp_path: Path) -> None:
    """write_recalibration_from_check re-emits the fresh drift K/D as a loadable CameraInfo YAML."""
    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo

    result = _run_check_folder_degraded(tmp_path)
    assert result["verdict"] == "DEGRADED"  # type: ignore[index]

    target = tmp_path / "recalibrated.yaml"
    written = write_recalibration_from_check(result, target, board="chessboard")

    assert written == target
    assert target.exists()
    info = CameraInfo.from_yaml(str(target))
    K_fresh = info.get_K_matrix()
    K_bad_fx = _K_TRUE[0, 0] * 1.08
    # The fresh solve recovers the true fx, not the deployed (bad) one.
    assert abs(float(K_fresh[0, 0]) - _K_TRUE[0, 0]) < abs(float(K_fresh[0, 0]) - K_bad_fx)


def test_interactive_confirm_yes_writes_fresh_yaml(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """On a DEGRADED TTY --check, confirming the [y/N] offer writes a loadable fresh CameraInfo YAML."""
    from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo

    frames = _synthetic_detectable_frames(count=12)
    images = tmp_path / "images"
    _write_frames(images, frames)
    deployed = _write_degraded_deployed_yaml(tmp_path)
    target = tmp_path / "operator_choice.yaml"

    # Force the interactive branch: TTY present, operator confirms, prompt returns the target path.
    _CC = "dimos.utils.cli.cameracalibrate.cameracalibrate"
    monkeypatch.setattr("sys.stdin.isatty", lambda: True)
    monkeypatch.setattr(f"{_CC}.typer.confirm", lambda *a, **k: True)
    monkeypatch.setattr(f"{_CC}.typer.prompt", lambda *a, **k: str(target))

    _run_check_report_folder(images, deployed, tmp_path / "check.json")

    assert target.exists()
    info = CameraInfo.from_yaml(str(target))
    K_fresh = info.get_K_matrix()
    K_bad_fx = _K_TRUE[0, 0] * 1.08
    assert abs(float(K_fresh[0, 0]) - _K_TRUE[0, 0]) < abs(float(K_fresh[0, 0]) - K_bad_fx)


def test_interactive_confirm_no_writes_nothing(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """Declining the [y/N] offer on a DEGRADED TTY --check writes no recalibration YAML."""
    frames = _synthetic_detectable_frames(count=12)
    images = tmp_path / "images"
    _write_frames(images, frames)
    deployed = _write_degraded_deployed_yaml(tmp_path)
    target = tmp_path / "operator_choice.yaml"
    prompted = False

    def _record_prompt(*a: object, **k: object) -> str:
        nonlocal prompted
        prompted = True
        return str(target)

    _CC = "dimos.utils.cli.cameracalibrate.cameracalibrate"
    monkeypatch.setattr("sys.stdin.isatty", lambda: True)
    monkeypatch.setattr(f"{_CC}.typer.confirm", lambda *a, **k: False)
    monkeypatch.setattr(f"{_CC}.typer.prompt", _record_prompt)

    _run_check_report_folder(images, deployed, tmp_path / "check.json")

    assert not prompted  # declined before any path prompt
    assert not target.exists()
    # No default-named file is written either.
    assert not (tmp_path / "recalibrated.yaml").exists()
