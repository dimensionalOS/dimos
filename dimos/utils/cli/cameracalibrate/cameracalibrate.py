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

"""Interactive camera calibration for dimos (ROS CameraInfo YAML output)."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from enum import Enum
import os
from pathlib import Path
import threading
import time
from typing import Any, TypedDict, cast
import warnings

# Default OpenCL off: on Apple Silicon, CPU chessboard detection is often faster and more stable here.
# Use setdefault so an explicit OPENCV_OPENCL_RUNTIME from the environment still wins.
os.environ.setdefault("OPENCV_OPENCL_RUNTIME", "disabled")

import cv2
import numpy as np
import typer
import yaml

_IMAGE_EXTS = frozenset({".png", ".jpg", ".jpeg"})


class CalibrationResultDict(TypedDict):
    """Structured return from ``calibrate_from_frames`` (and base of ``run_calibration``)."""

    K: np.ndarray
    D: np.ndarray
    rms: float
    image_size: tuple[int, int]
    n_used: int
    pattern_size: tuple[int, int]
    pattern_label: str


class CalibrationRunResultDict(CalibrationResultDict, total=False):
    """Optional paths written when ``run_calibration`` is asked to emit files."""

    out_path: Path
    preview_path: Path


class Source(str, Enum):
    """Frame source supported by the calibration CLI."""

    webcam = "webcam"
    folder = "folder"
    topic = "topic"


class DistortionModel(str, Enum):
    """Distortion model selected for ``calibrate_from_frames``.

    - ``plumb_bob``: ``cv2.calibrateCamera`` with 5-coefficient radial-tangential
      model. Good for near-pinhole lenses (narrow webcams, etc).
    - ``fisheye``: ``cv2.fisheye.calibrate`` with the 4-coefficient
      Kannala-Brandt model. Required for genuine fisheye / very wide-angle lenses
      (e.g. the Go2 front camera). The YAML written for this model uses the
      ROS-canonical name ``equidistant``.
    """

    plumb_bob = "plumb_bob"
    fisheye = "fisheye"

    def to_ros_name(self) -> str:
        return "equidistant" if self is DistortionModel.fisheye else self.value


app = typer.Typer(
    help="Calibrate camera intrinsics and write ROS CameraInfo YAML.",
    no_args_is_help=True,
)


def write_camera_info_yaml(
    path: str,
    *,
    image_width: int,
    image_height: int,
    camera_name: str,
    K: np.ndarray,
    D: np.ndarray,
    R: np.ndarray | None = None,
    P: np.ndarray | None = None,
    distortion_model: str = "plumb_bob",
) -> None:
    """Write ROS-style CameraInfo YAML loadable by dimos CameraInfo helpers.

    The emitted schema is accepted by ``CameraInfo.from_yaml``,
    ``load_camera_info``, and ``load_camera_info_opencv``.
    """
    k = np.asarray(K, dtype=np.float64).reshape(3, 3)
    d = np.asarray(D, dtype=np.float64).ravel()
    k_flat = k.ravel(order="C").tolist()
    d_flat = d.tolist()

    if R is None:
        r_flat = np.eye(3, dtype=np.float64).ravel(order="C").tolist()
    else:
        r_flat = np.asarray(R, dtype=np.float64).reshape(3, 3).ravel(order="C").tolist()

    if P is None:
        fx = k_flat[0]
        fy = k_flat[4]
        cx = k_flat[2]
        cy = k_flat[5]
        p_flat = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
    else:
        p_flat = np.asarray(P, dtype=np.float64).reshape(3, 4).ravel(order="C").tolist()

    n_dist = len(d_flat)
    payload = {
        "image_width": int(image_width),
        "image_height": int(image_height),
        "camera_name": camera_name,
        "distortion_model": distortion_model,
        "camera_matrix": {"rows": 3, "cols": 3, "data": k_flat},
        "distortion_coefficients": {"rows": 1, "cols": int(n_dist), "data": d_flat},
        "rectification_matrix": {"rows": 3, "cols": 3, "data": r_flat},
        "projection_matrix": {"rows": 3, "cols": 4, "data": p_flat},
    }

    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(payload, f, default_flow_style=False, sort_keys=False)


def load_frames_from_folder(path: str) -> list[np.ndarray]:
    """Load ``*.png``, ``*.jpg``, and ``*.jpeg`` images from a directory.

    Files are ordered by filename (lexicographic sort of basenames). Raises if the path
    is not a directory or if any matching file fails to decode with ``cv2.imread``.
    """
    root = Path(path)
    if not root.is_dir():
        raise ValueError(f"Not a directory: {path}")

    paths = sorted(p for p in root.iterdir() if p.is_file() and p.suffix.lower() in _IMAGE_EXTS)
    out: list[np.ndarray] = []
    for p in paths:
        img = cv2.imread(str(p))
        if img is None:
            raise ValueError(f"Could not read image: {p}")
        out.append(img)
    return out


_CAMERACALIBRATE_WINDOW = "dimos cameracalibrate"
_MAX_CONSECUTIVE_WEBCAM_READ_FAILURES = 30
_OPENCV_CALIBRATION_RUNTIME_CONFIGURED = False


@dataclass(frozen=True)
class _ChessboardDetection:
    corners: np.ndarray
    cols: int
    rows: int
    label: str


@dataclass(frozen=True)
class _WebcamCapture:
    frames: list[np.ndarray]
    image_points: list[np.ndarray]
    pattern: tuple[int, int, str] | None


def _pattern_candidates(cols: int, rows: int) -> list[tuple[int, int, str]]:
    """Return plausible inner-corner pattern sizes, exact request first."""
    # Board size may be given as inner corners or as square counts; also try swapped axes (portrait).
    candidates = [
        (cols, rows, "requested inner corners"),
        (rows, cols, "requested inner corners, rotated"),
    ]
    if cols > 1 and rows > 1:
        candidates.extend(
            [
                (cols - 1, rows - 1, "requested square count"),
                (rows - 1, cols - 1, "requested square count, rotated"),
            ]
        )

    out: list[tuple[int, int, str]] = []
    seen: set[tuple[int, int]] = set()
    for cand_cols, cand_rows, label in candidates:
        if cand_cols < 1 or cand_rows < 1:
            continue
        key = (cand_cols, cand_rows)
        if key in seen:
            continue
        seen.add(key)
        out.append((cand_cols, cand_rows, label))
    return out


def _as_grayscale_uint8(gray: np.ndarray) -> np.ndarray:
    g = np.asarray(gray)
    if g.ndim == 3:
        g = cv2.cvtColor(g, cv2.COLOR_BGR2GRAY)
    if g.dtype != np.uint8:
        # Corner finders expect uint8 range; normalize wider dtypes before detection.
        out_norm = np.empty(g.shape, dtype=np.uint8)
        g = cv2.normalize(g, out_norm, 0, 255, cv2.NORM_MINMAX)
    return np.ascontiguousarray(g)


def _configure_opencv_calibration_runtime() -> None:
    global _OPENCV_CALIBRATION_RUNTIME_CONFIGURED
    if _OPENCV_CALIBRATION_RUNTIME_CONFIGURED:
        return

    # Process-global OpenCV settings; run once before any chessboard detection in this module.
    cv2.setUseOptimized(True)
    if hasattr(cv2, "ocl"):
        try:
            cv2.ocl.setUseOpenCL(False)
        except cv2.error:
            pass

    _OPENCV_CALIBRATION_RUNTIME_CONFIGURED = True


def _find_chessboard_corners_sb(
    gray: np.ndarray,
    cols: int,
    rows: int,
    *,
    exhaustive: bool,
) -> np.ndarray | None:
    _configure_opencv_calibration_runtime()
    find_sb = getattr(cv2, "findChessboardCornersSB", None)
    if find_sb is None:
        return None

    g = _as_grayscale_uint8(gray)
    pattern_size = (cols, rows)
    sb_flags = cv2.CALIB_CB_NORMALIZE_IMAGE
    if exhaustive:
        # CALIB_CB_EXHAUSTIVE and CALIB_CB_ACCURACY: higher CPU, better recall on difficult frames.
        # Live preview passes exhaustive=False; exhaustive=True is reserved for the offline fallback path.
        if hasattr(cv2, "CALIB_CB_EXHAUSTIVE"):
            sb_flags |= cv2.CALIB_CB_EXHAUSTIVE
        if hasattr(cv2, "CALIB_CB_ACCURACY"):
            sb_flags |= cv2.CALIB_CB_ACCURACY

    ok, corners = cast("Callable[..., tuple[bool, Any]]", find_sb)(g, pattern_size, sb_flags)
    if not ok or corners is None:
        return None
    return np.asarray(corners, dtype=np.float32).reshape(cols * rows, 1, 2)


def _find_chessboard_corners_realtime(
    gray: np.ndarray,
    cols: int,
    rows: int,
) -> np.ndarray | None:
    # Preview path: SB without exhaustive flags so most misses stay cheap (latency budget).
    corners = _find_chessboard_corners_sb(gray, cols, rows, exhaustive=False)
    if corners is not None:
        return corners

    if getattr(cv2, "findChessboardCornersSB", None) is not None:
        return None

    return _find_chessboard_corners_exact(gray, cols, rows)


def _find_chessboard_corners_exact(gray: np.ndarray, cols: int, rows: int) -> np.ndarray | None:
    g = _as_grayscale_uint8(gray)
    pattern_size = (cols, rows)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    g_cv = cast("Any", np.ascontiguousarray(g))
    _find_corners = cast("Callable[..., tuple[bool, Any]]", cv2.findChessboardCorners)
    ok, corners = _find_corners(g_cv, pattern_size, flags)
    if ok and corners is not None:
        # Classic detector already gave pixel locations; refine to sub-pixel accuracy.
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        refined = cv2.cornerSubPix(g_cv, corners, (11, 11), (-1, -1), criteria)
        return cast("np.ndarray", refined)

    # Classic detector missed: try SB again with exhaustive flags (slower, higher recall).
    return _find_chessboard_corners_sb(g_cv, cols, rows, exhaustive=True)


def _find_chessboard_detection(
    gray: np.ndarray,
    cols: int,
    rows: int,
    *,
    realtime: bool = False,
    candidates: list[tuple[int, int, str]] | None = None,
) -> _ChessboardDetection | None:
    detector = _find_chessboard_corners_realtime if realtime else _find_chessboard_corners_exact
    for cand_cols, cand_rows, label in candidates or _pattern_candidates(cols, rows):
        corners = detector(gray, cand_cols, cand_rows)
        if corners is not None:
            return _ChessboardDetection(corners, cand_cols, cand_rows, label)
    return None


def _draw_capture_status(
    preview: np.ndarray,
    *,
    detection: _ChessboardDetection | None,
    accepted_count: int,
    target_count: int,
) -> None:
    status = f"Accepted {accepted_count}/{target_count}"
    if detection is None:
        detail = "No chessboard detected - SPACE ignored"
        color = (0, 0, 255)
    else:
        detail = f"Detected {detection.cols}x{detection.rows} ({detection.label}) - SPACE saves"
        color = (0, 180, 0)

    cv2.rectangle(preview, (0, 0), (preview.shape[1], 58), (0, 0, 0), thickness=-1)
    cv2.putText(preview, status, (12, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(preview, detail, (12, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)


def _interactive_capture(
    next_frame: Callable[[], np.ndarray | None],
    target_count: int,
    cols: int,
    rows: int,
    *,
    no_display: bool,
) -> _WebcamCapture:
    """Interactive chessboard preview + SPACE-accept / q-quit loop.

    ``next_frame()`` returns the latest BGR (or grayscale) frame, or ``None`` to
    skip the iteration without calling ``imshow``/``waitKey``. The caller owns
    any wait-on-no-frame or fail-fast policy. ``no_display`` skips ``imshow``
    and window teardown; ``cv2.waitKey`` is still called so tests can inject keys.
    """
    if target_count < 1:
        raise ValueError("target_count must be >= 1")

    accepted: list[np.ndarray] = []
    accepted_corners: list[np.ndarray] = []
    last_detected: tuple[int, int, str] | None = None
    locked_pattern: tuple[int, int, str] | None = None
    locked_exact_probe = False
    pattern_candidates = _pattern_candidates(cols, rows)
    pattern_candidate_index = 0

    try:
        while len(accepted) < target_count:
            frame = next_frame()
            if frame is None:
                continue

            if frame.ndim == 3:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            else:
                gray = frame

            if locked_pattern is None:
                candidate = pattern_candidates[pattern_candidate_index]
                pattern_candidate_index = (pattern_candidate_index + 1) % len(pattern_candidates)
                # Until pattern lock: one candidate per frame so we do not evaluate every pattern every frame.
                detection = _find_chessboard_detection(
                    gray,
                    cols,
                    rows,
                    realtime=True,
                    candidates=[candidate],
                )
                if detection is not None:
                    locked_pattern = (detection.cols, detection.rows, detection.label)
                    locked_exact_probe = True
            else:
                locked_cols, locked_rows, locked_label = locked_pattern
                if locked_exact_probe:
                    # Locked board: prefer full detector; on miss, drop to realtime until corners reappear.
                    corners = find_chessboard_corners(gray, locked_cols, locked_rows)
                    if corners is None:
                        locked_exact_probe = False
                else:
                    corners = _find_chessboard_corners_realtime(gray, locked_cols, locked_rows)
                    if corners is not None:
                        locked_exact_probe = True
                detection = (
                    _ChessboardDetection(corners, locked_cols, locked_rows, locked_label)
                    if corners is not None
                    else None
                )
            preview = np.asarray(frame).copy()
            if detection is not None:
                detected = (detection.cols, detection.rows, detection.label)
                if detected != last_detected:
                    last_detected = detected
                cv2.drawChessboardCorners(
                    preview,
                    (detection.cols, detection.rows),
                    detection.corners,
                    True,
                )
            _draw_capture_status(
                preview,
                detection=detection,
                accepted_count=len(accepted),
                target_count=target_count,
            )

            if not no_display:
                cv2.imshow(_CAMERACALIBRATE_WINDOW, preview)

            key = cv2.waitKey(1) & 0xFF
            if key == ord(" ") and detection is not None:
                accepted.append(np.asarray(frame).copy())
                accepted_corners.append(np.asarray(detection.corners, dtype=np.float32).copy())
            elif key == ord("q"):
                break

        if len(accepted) < target_count:
            raise RuntimeError(
                f"Capture ended with {len(accepted)} of {target_count} frames "
                f"(quit early, missing detections on SPACE, or read failures)."
            )

        return _WebcamCapture(accepted, accepted_corners, locked_pattern)

    finally:
        if not no_display:
            try:
                cv2.destroyWindow(_CAMERACALIBRATE_WINDOW)
            except cv2.error:
                pass
            cv2.waitKey(1)


def _capture_frames_from_webcam(
    device_index: int,
    target_count: int,
    cols: int,
    rows: int,
    *,
    no_display: bool = False,
) -> _WebcamCapture:
    """Capture ``target_count`` BGR frames from a webcam when the board is visible.

    Shows a live preview (unless ``no_display`` is True, for headless runs and CI).
    When a chessboard is detected, press SPACE to accept the current frame. Press
    ``q`` to quit early (raises if fewer than ``target_count`` frames were accepted).

    ``no_display`` mirrors the CLI ``--no-display`` flag: no ``cv2.imshow`` or window
    teardown; ``cv2.waitKey`` is still used so automated tests can inject key codes.
    """
    if target_count < 1:
        raise ValueError("target_count must be >= 1")

    cap: cv2.VideoCapture | None = None
    consecutive_read_failures = 0

    try:
        cap = cv2.VideoCapture(device_index)
        if not cap.isOpened():
            raise RuntimeError(f"Failed to open camera device_index={device_index!r}")

        def _next() -> np.ndarray | None:
            nonlocal consecutive_read_failures
            assert cap is not None  # narrow for type-checker
            ok, frame = cap.read()
            if not ok or frame is None:
                consecutive_read_failures += 1
                if consecutive_read_failures >= _MAX_CONSECUTIVE_WEBCAM_READ_FAILURES:
                    raise RuntimeError(
                        "Failed to read from camera "
                        f"device_index={device_index!r} for "
                        f"{_MAX_CONSECUTIVE_WEBCAM_READ_FAILURES} consecutive attempts."
                    )
                return None
            consecutive_read_failures = 0
            return frame  # type: ignore[no-any-return]

        return _interactive_capture(_next, target_count, cols, rows, no_display=no_display)

    finally:
        if cap is not None:
            cap.release()


def capture_frames_from_webcam(
    device_index: int,
    target_count: int,
    cols: int,
    rows: int,
    *,
    no_display: bool = False,
) -> list[np.ndarray]:
    """Capture ``target_count`` BGR frames from a webcam when the board is visible."""
    return _capture_frames_from_webcam(
        device_index,
        target_count,
        cols,
        rows,
        no_display=no_display,
    ).frames


def _capture_frames_from_topic(
    topic_uri: str,
    target_count: int,
    cols: int,
    rows: int,
    *,
    no_display: bool = False,
    timeout_sec: float = 60.0,
) -> _WebcamCapture:
    """Capture frames from an LCM/SHM image topic with the same interactive UX.

    ``topic_uri`` follows the pubsub registry format ``"<proto>:<topic>"``, e.g.
    ``"jpeg_lcm:/color_image"`` or ``"pshm:color_image"``. The publisher must
    emit ``sensor_msgs.Image`` messages; ``Image.to_opencv()`` normalizes the
    payload to BGR before detection. Raises ``RuntimeError`` if no frames arrive
    within ``timeout_sec``.
    """
    from dimos.msgs.sensor_msgs.Image import Image
    from dimos.protocol.pubsub.registry import subscribe_pubsub_uri

    if target_count < 1:
        raise ValueError("target_count must be >= 1")

    latest_frame: list[np.ndarray | None] = [None]
    last_received_ts: list[float] = [time.time()]
    lock = threading.Lock()

    def _on_image(msg: Any) -> None:
        try:
            arr = msg.to_opencv()
        except (AttributeError, ValueError):
            return
        with lock:
            latest_frame[0] = np.asarray(arr)
            last_received_ts[0] = time.time()

    transport, unsub = subscribe_pubsub_uri(topic_uri, _on_image, msg_type=Image)

    def _next() -> np.ndarray | None:
        with lock:
            frame = latest_frame[0]
            ts = last_received_ts[0]
        if frame is None:
            if time.time() - ts > timeout_sec:
                raise RuntimeError(
                    f"No frames received on topic {topic_uri!r} within {timeout_sec:.1f}s."
                )
            # Yield so the LCM/SHM callback thread can run; avoid busy spin.
            time.sleep(0.01)
            return None
        return frame

    try:
        return _interactive_capture(_next, target_count, cols, rows, no_display=no_display)
    finally:
        # Best-effort teardown: swallow per-transport quirks so cleanup
        # never masks the original error from _interactive_capture.
        try:
            unsub()
        except Exception:
            pass
        try:
            transport.stop()
        except Exception:
            pass


def capture_frames_from_topic(
    topic_uri: str,
    target_count: int,
    cols: int,
    rows: int,
    *,
    no_display: bool = False,
    timeout_sec: float = 60.0,
) -> list[np.ndarray]:
    """Capture ``target_count`` frames from an LCM/SHM image topic."""
    return _capture_frames_from_topic(
        topic_uri,
        target_count,
        cols,
        rows,
        no_display=no_display,
        timeout_sec=timeout_sec,
    ).frames


def find_chessboard_corners(gray: np.ndarray, cols: int, rows: int) -> np.ndarray | None:
    """Detect inner chessboard corners and refine them with sub-pixel accuracy.

    ``cols`` and ``rows`` are the counts of **inner** corners along each axis, matching
    ``cv2.findChessboardCorners(..., patternSize=(cols, rows))``.

    Returns:
        Float array of shape ``(cols * rows, 1, 2)`` on success, else ``None``.
    """
    return _find_chessboard_corners_exact(gray, cols, rows)


def _select_calibration_pattern(
    frames: list[np.ndarray],
    cols: int,
    rows: int,
) -> tuple[int, int, str]:
    candidates = _pattern_candidates(cols, rows)
    best_cols, best_rows, best_label = cols, rows, "requested inner corners"
    best_count = -1
    for cand_cols, cand_rows, label in candidates:
        count = 0
        for frame in frames:
            # Many frames: score each candidate with the lightweight detector first (same idea as preview).
            corners = _find_chessboard_corners_realtime(frame, cand_cols, cand_rows)
            if corners is not None:
                count += 1
        if count > best_count:
            best_cols, best_rows, best_label = cand_cols, cand_rows, label
            best_count = count

    if best_count <= 0:
        for cand_cols, cand_rows, label in candidates:
            count = 0
            for frame in frames:
                corners = find_chessboard_corners(frame, cand_cols, cand_rows)
                if corners is not None:
                    count += 1
            if count > best_count:
                best_cols, best_rows, best_label = cand_cols, cand_rows, label
                best_count = count

        if best_count <= 0:
            raise ValueError("Chessboard not found in any frame.")
    return best_cols, best_rows, best_label


def _calibrate_pinhole(
    objpoints: list[np.ndarray],
    imgpoints: list[np.ndarray],
    image_size: tuple[int, int],
) -> tuple[float, np.ndarray, np.ndarray]:
    """Run ``cv2.calibrateCamera`` (plumb-bob)."""
    _calibrate = cast("Callable[..., Any]", cv2.calibrateCamera)
    rms, camera_matrix, dist_coeffs, _rvecs, _tvecs = _calibrate(
        objpoints,
        imgpoints,
        image_size,
        None,
        None,
    )
    K = np.asarray(camera_matrix, dtype=np.float64)
    D = np.asarray(dist_coeffs, dtype=np.float64).reshape(-1)
    return float(rms), K, D


def _calibrate_fisheye(
    objpoints: list[np.ndarray],
    imgpoints: list[np.ndarray],
    image_size: tuple[int, int],
) -> tuple[float, np.ndarray, np.ndarray]:
    """Run ``cv2.fisheye.calibrate`` (4-coeff Kannala-Brandt).

    ``objpoints`` must be a list of ``(N, 1, 3)`` arrays and ``imgpoints`` a list of
    ``(N, 1, 2)`` arrays (the fisheye solver is strict about the extra middle axis).
    """
    K = np.zeros((3, 3), dtype=np.float64)
    D = np.zeros((4, 1), dtype=np.float64)
    n_views = len(objpoints)
    rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(n_views)]
    tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(n_views)]
    flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
    _calibrate = cast("Callable[..., Any]", cv2.fisheye.calibrate)
    rms, camera_matrix, dist_coeffs, _rvecs, _tvecs = _calibrate(
        objpoints,
        imgpoints,
        image_size,
        K,
        D,
        rvecs,
        tvecs,
        flags,
        criteria,
    )
    K_out = np.asarray(camera_matrix, dtype=np.float64)
    D_out = np.asarray(dist_coeffs, dtype=np.float64).reshape(-1)
    return float(rms), K_out, D_out


def calibrate_from_frames(
    frames: list[np.ndarray],
    cols: int,
    rows: int,
    square_size_m: float,
    *,
    pattern_hint: tuple[int, int, str] | None = None,
    image_points_hint: list[np.ndarray] | None = None,
    distortion_model: DistortionModel | str = DistortionModel.plumb_bob,
) -> CalibrationResultDict:
    """Calibrate intrinsics from grayscale or BGR frames containing a chessboard.

    Each frame where ``find_chessboard_corners`` succeeds contributes one view.
    All frames must share the same resolution.

    ``distortion_model`` picks the solver: ``plumb_bob`` (``cv2.calibrateCamera``,
    5 coeffs) or ``fisheye`` (``cv2.fisheye.calibrate``, 4 coeffs).

    Returns:
        ``{"K", "D", "rms", "image_size", "n_used"}`` with ``K`` (3x3) and ``D`` (1-d),
        ``rms`` reprojection RMSE from OpenCV, ``image_size`` ``(width, height)``, and
        ``n_used`` the number of frames that yielded detections.
    """
    if not frames:
        raise ValueError("frames must be non-empty")

    model = DistortionModel(distortion_model)

    if pattern_hint is None:
        actual_cols, actual_rows, pattern_label = _select_calibration_pattern(frames, cols, rows)
    else:
        actual_cols, actual_rows, pattern_label = pattern_hint

    # Object points on Z=0 with XY spacing square_size_m. cv2.fisheye.calibrate
    # demands an explicit middle axis on each view; cv2.calibrateCamera is fine
    # with the flat (N, 3) shape.
    objp_flat = np.zeros((actual_rows * actual_cols, 3), dtype=np.float32)
    objp_flat[:, :2] = np.mgrid[0:actual_cols, 0:actual_rows].T.reshape(-1, 2).astype(np.float32)
    objp_flat *= float(square_size_m)
    objp_view = objp_flat.reshape(-1, 1, 3) if model is DistortionModel.fisheye else objp_flat

    objpoints: list[np.ndarray] = []
    imgpoints: list[np.ndarray] = []

    first = np.asarray(frames[0])
    h0, w0 = first.shape[:2]

    if image_points_hint is not None and len(image_points_hint) != len(frames):
        raise ValueError("image_points_hint length must match frames length.")

    for i, frame in enumerate(frames):
        f = np.asarray(frame)
        if f.shape[:2] != (h0, w0):
            raise ValueError("All frames must have the same shape.")

        corners_found: np.ndarray
        if image_points_hint is not None:
            # Corners from the frame at SPACE accept time; avoids re-detecting a different instant.
            corners_found = np.asarray(image_points_hint[i], dtype=np.float32).reshape(
                actual_rows * actual_cols,
                1,
                2,
            )
        else:
            gray_in: np.ndarray
            if f.ndim == 3:
                gray_in = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
            else:
                gray_in = np.asarray(f)
            corners_opt = find_chessboard_corners(gray_in, actual_cols, actual_rows)
            if corners_opt is None:
                continue
            corners_found = corners_opt
        objpoints.append(objp_view)
        imgpoints.append(corners_found.astype(np.float32))

    if not objpoints:
        raise ValueError("Chessboard not found in any frame.")

    if model is DistortionModel.fisheye:
        rms, K, D = _calibrate_fisheye(objpoints, imgpoints, (w0, h0))
    else:
        rms, K, D = _calibrate_pinhole(objpoints, imgpoints, (w0, h0))

    return {
        "K": K,
        "D": D,
        "rms": float(rms),
        "image_size": (int(w0), int(h0)),
        "n_used": len(objpoints),
        "pattern_size": (int(actual_cols), int(actual_rows)),
        "pattern_label": pattern_label,
    }


def write_preview_overlay_png(
    frames: list[np.ndarray],
    cols: int,
    rows: int,
    path: Path,
) -> Path:
    """Write a preview PNG with detected chessboard corners drawn on one input frame."""
    for frame in frames:
        f = np.asarray(frame)
        gray = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY) if f.ndim == 3 else f
        corners = find_chessboard_corners(gray, cols, rows)
        if corners is None:
            continue

        preview = cv2.cvtColor(f, cv2.COLOR_GRAY2BGR) if f.ndim == 2 else np.asarray(frame).copy()
        cv2.drawChessboardCorners(preview, (cols, rows), corners, True)
        path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(path), preview):
            raise ValueError(f"Could not write preview image: {path}")
        return path

    raise ValueError("Chessboard not found in any frame for preview overlay.")


def run_calibration(
    *,
    source: Source | str,
    device_index: int,
    images: Path | None,
    topic: str | None,
    topic_timeout_sec: float,
    cols: int,
    rows: int,
    square_size_m: float,
    out: Path | None,
    preview_out: Path | None,
    camera_name: str,
    target_count: int,
    no_display: bool,
    distortion_model: DistortionModel | str = DistortionModel.plumb_bob,
) -> CalibrationRunResultDict:
    """Run calibration from the requested frame source and write CameraInfo YAML."""
    source_value = Source(source)
    model = DistortionModel(distortion_model)
    if cols < 1:
        raise ValueError("cols must be >= 1")
    if rows < 1:
        raise ValueError("rows must be >= 1")
    if square_size_m <= 0:
        raise ValueError("square_size_m must be > 0")

    if source_value is Source.folder:
        if images is None:
            raise ValueError("--images is required when --source folder")
        frames = load_frames_from_folder(str(images))
        pattern_hint = None
        image_points_hint = None
    elif source_value is Source.topic:
        if topic is None:
            raise ValueError(
                "--topic is required when --source topic (e.g. --topic jpeg_lcm:/color_image)"
            )
        capture = _capture_frames_from_topic(
            topic,
            target_count,
            cols,
            rows,
            no_display=no_display,
            timeout_sec=topic_timeout_sec,
        )
        frames = capture.frames
        pattern_hint = capture.pattern
        image_points_hint = capture.image_points
    else:
        capture = _capture_frames_from_webcam(
            device_index,
            target_count,
            cols,
            rows,
            no_display=no_display,
        )
        frames = capture.frames
        pattern_hint = capture.pattern
        image_points_hint = capture.image_points

    cal = calibrate_from_frames(
        frames,
        cols,
        rows,
        square_size_m,
        pattern_hint=pattern_hint,
        image_points_hint=image_points_hint,
        distortion_model=model,
    )
    result: CalibrationRunResultDict = {
        "K": cal["K"],
        "D": cal["D"],
        "rms": cal["rms"],
        "image_size": cal["image_size"],
        "n_used": cal["n_used"],
        "pattern_size": cal["pattern_size"],
        "pattern_label": cal["pattern_label"],
    }
    image_width, image_height = result["image_size"]

    if out is not None:
        out.parent.mkdir(parents=True, exist_ok=True)
        write_camera_info_yaml(
            str(out),
            image_width=int(image_width),
            image_height=int(image_height),
            camera_name=camera_name,
            K=np.asarray(result["K"], dtype=np.float64),
            D=np.asarray(result["D"], dtype=np.float64),
            distortion_model=model.to_ros_name(),
        )
        result["out_path"] = out

    if preview_out is not None:
        preview_out.parent.mkdir(parents=True, exist_ok=True)
        pattern_cols, pattern_rows = result["pattern_size"]
        # Preview is best-effort: a chessboard-detection failure here must not mask
        # the fact that the YAML was already written above.
        try:
            write_preview_overlay_png(frames, int(pattern_cols), int(pattern_rows), preview_out)
            result["preview_path"] = preview_out
        except ValueError as exc:
            warnings.warn(
                f"Preview PNG skipped ({exc}). Camera info YAML was still written to {out}.",
                stacklevel=2,
            )

    return result


@app.command()
def calibrate(
    source: Source = typer.Option(..., "--source", help="Frame source: webcam, folder, or topic"),
    device_index: int = typer.Option(0, "--device-index", help="Webcam device index"),
    images: Path | None = typer.Option(
        None, "--images", help="Directory of calibration images for --source folder"
    ),
    topic: str | None = typer.Option(
        None,
        "--topic",
        help=(
            "Pubsub URI for --source topic (proto:channel), "
            "e.g. 'jpeg_lcm:/color_image' or 'pshm:color_image'."
        ),
    ),
    topic_timeout_sec: float = typer.Option(
        60.0,
        "--topic-timeout-sec",
        help="Abort --source topic if no frames arrive within this many seconds.",
    ),
    cols: int = typer.Option(..., "--cols", help="Inner chessboard corner columns"),
    rows: int = typer.Option(..., "--rows", help="Inner chessboard corner rows"),
    square_size_m: float = typer.Option(
        ..., "--square-size-m", help="Chessboard square size in meters"
    ),
    out: Path | None = typer.Option(None, "--out", help="Optional ROS CameraInfo YAML output path"),
    preview_out: Path | None = typer.Argument(
        None, help="Optional preview PNG output path. Requires --out."
    ),
    camera_name: str = typer.Option("webcam", "--camera-name", help="Camera name in YAML"),
    target_count: int = typer.Option(20, "--target-count", help="Accepted webcam frame count"),
    no_display: bool = typer.Option(False, "--no-display", help="Disable OpenCV preview windows"),
    distortion_model: DistortionModel = typer.Option(
        DistortionModel.plumb_bob,
        "--distortion-model",
        help=(
            "Lens model: 'plumb_bob' (cv2.calibrateCamera, 5 coeffs) for near-pinhole "
            "lenses, or 'fisheye' (cv2.fisheye.calibrate, 4 coeffs) for wide-angle / "
            "fisheye lenses. Fisheye writes ROS 'equidistant' to the YAML."
        ),
    ),
) -> None:
    """Calibrate camera intrinsics and write ROS CameraInfo YAML."""
    if preview_out is not None and out is None:
        raise typer.BadParameter("preview output requires --out")

    try:
        result = run_calibration(
            source=source,
            device_index=device_index,
            images=images,
            topic=topic,
            topic_timeout_sec=topic_timeout_sec,
            cols=cols,
            rows=rows,
            square_size_m=square_size_m,
            out=out,
            preview_out=preview_out,
            camera_name=camera_name,
            target_count=target_count,
            no_display=no_display,
            distortion_model=distortion_model,
        )
    except (ValueError, RuntimeError) as exc:
        raise typer.BadParameter(str(exc)) from exc

    typer.echo(f"RMS: {float(result['rms']):.6f} px ({int(result['n_used'])} frame(s) used)")
    typer.echo(
        f"Detected pattern: {tuple(result.get('pattern_size', (cols, rows)))} "
        f"({result.get('pattern_label', 'requested inner corners')})"
    )
    if out is not None:
        typer.echo(f"Wrote camera info YAML to {out}")
    if preview_out is not None:
        typer.echo(f"Wrote preview overlay PNG to {preview_out}")


def main(args: list[str] | None = None) -> None:
    """CLI entry point."""
    app(args=args)
