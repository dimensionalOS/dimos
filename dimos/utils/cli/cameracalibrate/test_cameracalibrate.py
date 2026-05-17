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

import os
from pathlib import Path
import re
from unittest.mock import MagicMock

import cv2
import numpy as np
import pytest
from typer.testing import CliRunner

from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo as DimosCameraInfo
from dimos.perception.common.utils import load_camera_info, load_camera_info_opencv
from dimos.utils.cli.cameracalibrate.cameracalibrate import (
    app,
    calibrate_from_frames,
    capture_frames_from_webcam,
    find_chessboard_corners,
    load_frames_from_folder,
    write_camera_info_yaml,
)


def _synthetic_chessboard_gray(
    width: int,
    height: int,
    cols: int,
    rows: int,
    square_px: int,
) -> np.ndarray:
    """Build a binary chessboard; ``cols`` x ``rows`` inner corners need ``cols+1`` x ``rows+1`` squares."""
    img = np.full((height, width), 255, dtype=np.uint8)
    board_w = (cols + 1) * square_px
    board_h = (rows + 1) * square_px
    ox = (width - board_w) // 2
    oy = (height - board_h) // 2
    for yi in range(rows + 1):
        for xi in range(cols + 1):
            color = 0 if (xi + yi) % 2 == 0 else 255
            x0 = ox + xi * square_px
            y0 = oy + yi * square_px
            img[y0 : y0 + square_px, x0 : x0 + square_px] = color
    return img


def _synthetic_calibration_frames(
    *,
    cols: int = 9,
    rows: int = 6,
    width: int = 640,
    height: int = 480,
    square_size_m: float = 0.025,
    count: int = 12,
) -> tuple[list[np.ndarray], np.ndarray]:
    square_px = 40
    K_true = np.array(
        [[512.0, 0.0, 318.5], [0.0, 508.0, 242.3], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    D_zero = np.zeros(5, dtype=np.float64)

    gray_flat = _synthetic_chessboard_gray(width, height, cols, rows, square_px=square_px)
    corners_flat = find_chessboard_corners(gray_flat, cols, rows)
    assert corners_flat is not None
    src = corners_flat.reshape(-1, 2).astype(np.float32)

    objp = np.zeros((rows * cols, 3), dtype=np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2).astype(np.float32)
    objp *= float(square_size_m)

    rng = np.random.default_rng(42)
    frames: list[np.ndarray] = []
    for _ in range(400):
        if len(frames) >= count:
            break
        rvec = rng.uniform(-0.22, 0.22, size=3).astype(np.float64)
        tvec = np.array(
            [
                rng.uniform(-0.04, 0.04),
                rng.uniform(-0.04, 0.04),
                rng.uniform(0.38, 0.52),
            ],
            dtype=np.float64,
        )
        imgpts, _ = cv2.projectPoints(objp, rvec, tvec, K_true, D_zero)
        dst = imgpts.reshape(-1, 2).astype(np.float32)
        H, _ = cv2.findHomography(src, dst, cv2.RANSAC, 2.0)
        if H is None:
            continue
        warped = cv2.warpPerspective(gray_flat, H, (width, height))
        corners_w = find_chessboard_corners(warped, cols, rows)
        if corners_w is not None:
            frames.append(warped)

    assert len(frames) >= count
    return frames[:count], K_true


def test_cli_folder_with_synthetic_images_writes_yaml_preview_and_camera_info(
    tmp_path: Path,
) -> None:
    """Folder mode end-to-end without checked-in JPEG fixtures (CI-friendly)."""
    cols, rows = 9, 6
    frames, _K_true = _synthetic_calibration_frames(cols=cols, rows=rows, count=12)
    images = tmp_path / "images"
    images.mkdir()
    for i, frame in enumerate(frames):
        assert cv2.imwrite(str(images / f"frame_{i:02d}.png"), frame)

    out = tmp_path / "camera_info.yaml"
    preview = tmp_path / "camera_info.preview.png"
    result = CliRunner().invoke(
        app,
        [
            "--source",
            "folder",
            "--images",
            str(images),
            "--cols",
            "9",
            "--rows",
            "6",
            "--square-size-m",
            "0.025",
            "--out",
            str(out),
            "--camera-name",
            "synthetic_folder",
            "--no-display",
            str(preview),
        ],
    )

    assert result.exit_code == 0, result.output
    assert "RMS:" in result.output
    assert "(12 frame(s) used)" in result.output
    assert "Wrote preview overlay PNG" in result.output

    assert out.exists()
    assert preview.exists()
    assert preview.stat().st_size > 0

    preview_image = cv2.imread(str(preview))
    assert preview_image is not None
    assert preview_image.shape == (480, 640, 3)

    info = load_camera_info(str(out), frame_id="camera_optical")
    assert info.width == 640
    assert info.height == 480
    assert info.distortion_model == "plumb_bob"
    assert info.header.frame_id == "camera_optical"

    dimos_info = DimosCameraInfo.from_yaml(str(out))
    assert dimos_info.width == 640
    assert dimos_info.height == 480
    assert dimos_info.distortion_model == "plumb_bob"
    assert dimos_info.frame_id == "camera_optical"
    assert dimos_info.get_K_matrix().shape == (3, 3)
    assert dimos_info.get_D_coeffs().shape == (5,)
    assert dimos_info.get_R_matrix().shape == (3, 3)
    assert dimos_info.get_P_matrix().shape == (3, 4)


def test_cli_help_lists_cameracalibrate_flags() -> None:
    result = CliRunner().invoke(app, ["--help"])
    assert result.exit_code == 0
    output_plain = re.sub(r"\x1b\[[0-9;]*m", "", result.output)
    for flag in [
        "--source",
        "--device-index",
        "--images",
        "--cols",
        "--rows",
        "--square-size-m",
        "--out",
        "--camera-name",
        "--target-count",
        "--no-display",
    ]:
        assert flag in output_plain


def test_cli_folder_writes_only_explicit_yaml_and_prints_rms(tmp_path: Path) -> None:
    cols, rows = 9, 6
    frames, _K_true = _synthetic_calibration_frames(cols=cols, rows=rows)
    images = tmp_path / "fixture"
    images.mkdir()
    for i, frame in enumerate(frames):
        assert cv2.imwrite(str(images / f"{i:02d}.png"), frame)

    out = tmp_path / "camera_info.yaml"
    result = CliRunner().invoke(
        app,
        [
            "--source",
            "folder",
            "--images",
            str(images),
            "--cols",
            "9",
            "--rows",
            "6",
            "--square-size-m",
            "0.025",
            "--out",
            str(out),
            "--camera-name",
            "webcam",
            "--no-display",
        ],
    )

    assert result.exit_code == 0
    assert "RMS:" in result.output
    assert "Wrote camera info YAML" in result.output
    assert "Wrote preview overlay PNG" not in result.output
    assert out.exists()
    preview = tmp_path / "camera_info.preview.png"
    assert not preview.exists()
    info = load_camera_info(str(out), frame_id="camera_optical")
    assert info.width == 640
    assert info.height == 480
    assert info.distortion_model == "plumb_bob"


def test_cli_folder_writes_explicit_yaml_and_preview(tmp_path: Path) -> None:
    cols, rows = 9, 6
    frames, _K_true = _synthetic_calibration_frames(cols=cols, rows=rows)
    images = tmp_path / "fixture"
    images.mkdir()
    for i, frame in enumerate(frames):
        assert cv2.imwrite(str(images / f"{i:02d}.png"), frame)

    out = tmp_path / "camera_info.yaml"
    preview = tmp_path / "camera_info.preview.png"
    result = CliRunner().invoke(
        app,
        [
            "--source",
            "folder",
            "--images",
            str(images),
            "--cols",
            "9",
            "--rows",
            "6",
            "--square-size-m",
            "0.025",
            "--out",
            str(out),
            "--no-display",
            str(preview),
        ],
    )

    assert result.exit_code == 0, result.output
    assert "Wrote camera info YAML" in result.output
    assert "Wrote preview overlay PNG" in result.output
    assert out.exists()
    assert preview.exists()
    assert cv2.imread(str(preview)) is not None


def test_cli_folder_writes_no_outputs_when_paths_are_omitted(tmp_path: Path) -> None:
    cols, rows = 9, 6
    frames, _K_true = _synthetic_calibration_frames(cols=cols, rows=rows)
    images = tmp_path / "fixture"
    images.mkdir()
    for i, frame in enumerate(frames):
        assert cv2.imwrite(str(images / f"{i:02d}.png"), frame)

    result = CliRunner().invoke(
        app,
        [
            "--source",
            "folder",
            "--images",
            str(images),
            "--cols",
            "9",
            "--rows",
            "6",
            "--square-size-m",
            "0.025",
            "--no-display",
        ],
    )

    assert result.exit_code == 0, result.output
    assert "RMS:" in result.output
    assert "Wrote camera info YAML" not in result.output
    assert "Wrote preview overlay PNG" not in result.output
    assert not (tmp_path / "camera_info.yaml").exists()
    assert not (tmp_path / "camera_info.preview.png").exists()


class _MockVideoCapture:
    """Minimal ``cv2.VideoCapture`` stand-in for webcam capture tests."""

    def __init__(self, bgr_frame: np.ndarray) -> None:
        self._frame = np.asarray(bgr_frame)
        self._released = False

    def isOpened(self) -> bool:
        return True

    def read(self) -> tuple[bool, np.ndarray]:
        return True, self._frame.copy()

    def release(self) -> None:
        self._released = True


class _FailingVideoCapture:
    """``cv2.VideoCapture`` stand-in whose reads never produce frames."""

    def __init__(self) -> None:
        self.read_count = 0
        self.released = False

    def isOpened(self) -> bool:
        return True

    def read(self) -> tuple[bool, None]:
        self.read_count += 1
        return False, None

    def release(self) -> None:
        self.released = True


def test_capture_frames_from_webcam_mocked_space_fills_target(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """SPACE accepts frames with chessboard overlay path; ``no_display`` skips GUI."""
    cols, rows = 9, 6
    gray = _synthetic_chessboard_gray(640, 480, cols, rows, square_px=40)
    bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    keys_iter = iter([ord(" ")] * 3)

    def _fake_wait_key(_delay: int = 0) -> int:
        try:
            return next(keys_iter)
        except StopIteration:
            return 0

    monkeypatch.setattr(cv2, "VideoCapture", lambda *_a, **_k: _MockVideoCapture(bgr))
    monkeypatch.setattr(cv2, "waitKey", _fake_wait_key)
    mock_imshow = MagicMock()
    monkeypatch.setattr(cv2, "imshow", mock_imshow)

    out = capture_frames_from_webcam(0, 3, cols, rows, no_display=True)
    assert len(out) == 3
    assert all(np.array_equal(f, bgr) for f in out)
    mock_imshow.assert_not_called()


def test_capture_frames_from_webcam_mocked_quit_raises(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    cols, rows = 9, 6
    gray = _synthetic_chessboard_gray(640, 480, cols, rows, square_px=40)
    bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    keys_iter = iter([ord(" "), ord("q")])

    def _fake_wait_key(_delay: int = 0) -> int:
        try:
            return next(keys_iter)
        except StopIteration:
            return 0

    monkeypatch.setattr(cv2, "VideoCapture", lambda *_a, **_k: _MockVideoCapture(bgr))
    monkeypatch.setattr(cv2, "waitKey", _fake_wait_key)

    with pytest.raises(RuntimeError, match="Capture ended"):
        capture_frames_from_webcam(0, 3, cols, rows, no_display=True)


def test_capture_frames_from_webcam_read_failures_raise(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    cap = _FailingVideoCapture()
    monkeypatch.setattr(cv2, "VideoCapture", lambda *_a, **_k: cap)

    with pytest.raises(RuntimeError, match="Failed to read from camera"):
        capture_frames_from_webcam(0, 1, 9, 6, no_display=True)

    assert cap.read_count == 30
    assert cap.released


def test_capture_frames_from_webcam_no_display_false_calls_imshow(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    cols, rows = 9, 6
    gray = _synthetic_chessboard_gray(320, 240, cols, rows, square_px=20)
    bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    monkeypatch.setattr(cv2, "VideoCapture", lambda *_a, **_k: _MockVideoCapture(bgr))
    monkeypatch.setattr(cv2, "waitKey", lambda _delay=0: ord(" "))
    mock_imshow = MagicMock()
    mock_destroy = MagicMock()
    monkeypatch.setattr(cv2, "imshow", mock_imshow)
    monkeypatch.setattr(cv2, "destroyWindow", mock_destroy)

    capture_frames_from_webcam(0, 1, cols, rows, no_display=False)
    mock_imshow.assert_called()
    mock_destroy.assert_called_once()


@pytest.mark.skipif(
    os.environ.get("DIMOS_TEST_REAL_CAMERA") != "1",
    reason="Set DIMOS_TEST_REAL_CAMERA=1 to run this hardware webcam smoke test.",
)
def test_opencv_video_capture_device_zero_opens_when_camera_available() -> None:
    """Smoke check for a real webcam when explicitly opted in."""
    cap = cv2.VideoCapture(0)
    try:
        assert cap.isOpened()
    finally:
        cap.release()


def test_load_frames_from_folder_count_order_and_pixels(tmp_path: Path) -> None:
    """Sorted ``*.png`` / ``*.jpg`` / ``*.jpeg``; correct count and load order."""
    h, w = 24, 32
    # Write out of lexicographic order; expect sorted basenames: 01, 02, 03.
    cv2.imwrite(str(tmp_path / "02.png"), np.full((h, w, 3), (10, 20, 30), dtype=np.uint8))
    cv2.imwrite(str(tmp_path / "01.jpg"), np.full((h, w, 3), (40, 50, 60), dtype=np.uint8))
    cv2.imwrite(str(tmp_path / "03.jpeg"), np.full((h, w, 3), (70, 80, 90), dtype=np.uint8))
    # Noise file must be ignored.
    (tmp_path / "notes.txt").write_text("ignore", encoding="utf-8")

    frames = load_frames_from_folder(str(tmp_path))
    assert len(frames) == 3
    assert frames[0].shape == (h, w, 3)
    assert np.array_equal(frames[0], np.full((h, w, 3), (40, 50, 60), dtype=np.uint8))
    assert np.array_equal(frames[1], np.full((h, w, 3), (10, 20, 30), dtype=np.uint8))
    assert np.array_equal(frames[2], np.full((h, w, 3), (70, 80, 90), dtype=np.uint8))


def test_find_chessboard_corners_synthetic_board_returns_expected_count() -> None:
    cols, rows = 9, 6
    gray = _synthetic_chessboard_gray(640, 480, cols, rows, square_px=40)
    corners = find_chessboard_corners(gray, cols, rows)
    assert corners is not None
    assert corners.shape == (cols * rows, 1, 2)


def test_calibrate_from_frames_synthetic_twelve_views_rms_and_K_near_truth() -> None:
    """12 OpenCV-synthesized chessboard views from known ``K``; ``rms`` < 1 px; ``K`` ~ truth."""
    cols, rows = 9, 6
    width, height = 640, 480
    square_size_m = 0.025
    frames, K_true = _synthetic_calibration_frames(
        cols=cols,
        rows=rows,
        width=width,
        height=height,
        square_size_m=square_size_m,
        count=12,
    )

    out = calibrate_from_frames(frames, cols, rows, square_size_m)
    assert out["n_used"] == 12
    assert out["image_size"] == (width, height)
    assert isinstance(out["rms"], float)
    assert out["rms"] < 1.0

    K_est = np.asarray(out["K"], dtype=np.float64).reshape(3, 3)
    denom = np.maximum(np.abs(K_true), 1e-9)
    rel = np.abs(K_est - K_true) / denom
    assert np.all(rel < 0.05)


def test_calibrate_from_frames_accepts_square_count_request() -> None:
    """A 12x8-square printed board has 11x7 inner corners."""
    frames, _K_true = _synthetic_calibration_frames(cols=11, rows=7, count=10)

    out = calibrate_from_frames(frames, cols=12, rows=8, square_size_m=0.02)

    assert out["n_used"] == 10
    assert out["pattern_size"] == (11, 7)
    assert out["pattern_label"] == "requested square count"


def test_write_camera_info_yaml_round_trip_matches_k_d_size_and_model(tmp_path: Path) -> None:
    K = np.array([[500.0, 0.0, 320.0], [0.0, 510.0, 240.0], [0.0, 0.0, 1.0]])
    D = np.array([-0.1, 0.05, 0.0, 0.0, 0.0])
    path = str(tmp_path / "camera_info.yaml")
    write_camera_info_yaml(
        path,
        image_width=640,
        image_height=480,
        camera_name="test_cam",
        K=K,
        D=D,
        distortion_model="plumb_bob",
    )
    info = load_camera_info(path, frame_id="camera_link")
    assert info.width == 640
    assert info.height == 480
    assert info.distortion_model == "plumb_bob"
    assert np.allclose(np.asarray(info.K, dtype=np.float64).reshape(3, 3), K)
    assert np.allclose(np.asarray(info.D, dtype=np.float64).ravel(), D.ravel())

    dimos_info = DimosCameraInfo.from_yaml(path)
    assert dimos_info.width == 640
    assert dimos_info.height == 480
    assert dimos_info.distortion_model == "plumb_bob"
    assert np.allclose(dimos_info.get_K_matrix(), K)
    assert np.allclose(dimos_info.get_D_coeffs(), D)


def test_write_camera_info_yaml_round_trip_load_camera_info_and_opencv(tmp_path: Path) -> None:
    """YAML written by ``write_camera_info_yaml`` round-trips through both loaders."""
    K = np.array([[600.0, 0.5, 400.0], [0.0, 605.0, 300.5], [0.0, 0.0, 1.0]], dtype=np.float64)
    D = np.array([-0.12, 0.08, 0.002, -0.001, 0.0], dtype=np.float64)
    R = np.array([[0.999, -0.01, 0.0], [0.01, 0.999, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    P = np.array(
        [[600.0, 0.0, 400.0, 0.1], [0.0, 605.0, 300.5, 0.2], [0.0, 0.0, 1.0, 0.0]],
        dtype=np.float64,
    )
    path = str(tmp_path / "camera_info.yaml")
    write_camera_info_yaml(
        path,
        image_width=800,
        image_height=600,
        camera_name="synthetic",
        K=K,
        D=D,
        R=R,
        P=P,
        distortion_model="plumb_bob",
    )
    info = load_camera_info(path, frame_id="camera_optical")
    K_cv, D_cv = load_camera_info_opencv(path)

    assert info.width == 800
    assert info.height == 600
    assert info.distortion_model == "plumb_bob"
    assert info.header.frame_id == "camera_optical"
    assert np.allclose(np.asarray(info.K, dtype=np.float64).reshape(3, 3), K)
    assert np.allclose(np.asarray(info.D, dtype=np.float64).ravel(), D.ravel())
    assert np.allclose(np.asarray(info.R, dtype=np.float64).reshape(3, 3), R)
    assert np.allclose(np.asarray(info.P, dtype=np.float64).reshape(3, 4), P)
    assert np.allclose(K_cv, K)
    assert np.allclose(np.asarray(D_cv, dtype=np.float64).ravel(), D.ravel())

    dimos_info = DimosCameraInfo.from_yaml(path)
    assert dimos_info.width == 800
    assert dimos_info.height == 600
    assert dimos_info.distortion_model == "plumb_bob"
    assert dimos_info.frame_id == "camera_optical"
    assert np.allclose(dimos_info.get_K_matrix(), K)
    assert np.allclose(dimos_info.get_D_coeffs(), D)
    assert np.allclose(dimos_info.get_R_matrix(), R)
    assert np.allclose(dimos_info.get_P_matrix(), P)


def test_write_camera_info_yaml_custom_r_p_and_distortion_model(tmp_path: Path) -> None:
    K = np.array([[400.0, 1.0, 160.0], [0.0, 401.0, 120.0], [0.0, 0.0, 1.0]])
    D = np.array([-0.05, 0.02, 0.001, -0.0005])
    R = np.eye(3)
    P = np.array([[400.0, 0.0, 160.0, 0.01], [0.0, 401.0, 120.0, 0.02], [0.0, 0.0, 1.0, 0.0]])
    path = str(tmp_path / "camera_info.yaml")
    write_camera_info_yaml(
        path,
        image_width=320,
        image_height=240,
        camera_name="narrow",
        K=K,
        D=D,
        R=R,
        P=P,
        distortion_model="rational_polynomial",
    )
    info = load_camera_info(path)
    assert info.width == 320
    assert info.height == 240
    assert info.distortion_model == "rational_polynomial"
    assert np.allclose(np.asarray(info.K, dtype=np.float64).reshape(3, 3), K)
    assert np.allclose(np.asarray(info.D, dtype=np.float64).ravel(), D.ravel())
    assert np.allclose(np.asarray(info.R, dtype=np.float64).reshape(3, 3), R)
    assert np.allclose(np.asarray(info.P, dtype=np.float64).reshape(3, 4), P)
