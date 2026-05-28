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

from __future__ import annotations

import base64
from dataclasses import dataclass
from threading import Event, Lock, Thread
import time
from typing import Any

import cv2
import numpy as np

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

DEVICE_TYPE_TRUEDEPTH = 0


@dataclass(frozen=True)
class Record3DFrame:
    image_data_url: str
    jpeg_bytes: bytes
    depth_jpeg_bytes: bytes
    depth_hint: dict[str, Any]
    captured_at: float


def _encode_jpeg_data_url(rgb: np.ndarray, max_width: int = 640) -> tuple[str, bytes]:
    if rgb.ndim != 3 or rgb.shape[2] < 3:
        raise ValueError(f"Expected RGB image with shape HxWx3, got {rgb.shape}")

    image = rgb[:, :, :3]
    height, width = image.shape[:2]
    if width > max_width:
        scale = max_width / float(width)
        image = cv2.resize(image, (max_width, int(height * scale)), interpolation=cv2.INTER_AREA)

    bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    ok, buffer = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 78])
    if not ok:
        raise RuntimeError("Failed to encode Record3D RGB frame")

    jpeg_bytes = buffer.tobytes()
    encoded = base64.b64encode(jpeg_bytes).decode("ascii")
    return f"data:image/jpeg;base64,{encoded}", jpeg_bytes


def _encode_depth_jpeg(
    depth: np.ndarray,
    confidence: np.ndarray | None,
    max_width: int = 640,
) -> bytes:
    depth_f = np.asarray(depth, dtype=np.float32)
    valid = np.isfinite(depth_f) & (depth_f > 0.05) & (depth_f < 10.0)
    if confidence is not None and confidence.size and confidence.shape == depth_f.shape:
        valid &= np.asarray(confidence) > 0

    if valid.any():
        clipped = np.clip(depth_f, 0.05, 6.0)
        normalized = ((6.0 - clipped) / 5.95 * 255.0).astype(np.uint8)
        normalized[~valid] = 0
    else:
        normalized = np.zeros(depth_f.shape[:2], dtype=np.uint8)

    height, width = normalized.shape[:2]
    if width > max_width:
        scale = max_width / float(width)
        normalized = cv2.resize(
            normalized,
            (max_width, int(height * scale)),
            interpolation=cv2.INTER_NEAREST,
        )

    color = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
    color[normalized == 0] = (0, 0, 0)
    ok, buffer = cv2.imencode(".jpg", color, [int(cv2.IMWRITE_JPEG_QUALITY), 78])
    if not ok:
        raise RuntimeError("Failed to encode Record3D depth frame")
    return buffer.tobytes()


def _safe_percentile(values: np.ndarray, percentile: float) -> float | None:
    if values.size == 0:
        return None
    return float(np.percentile(values, percentile))


def _depth_summary(depth: np.ndarray, confidence: np.ndarray | None) -> dict[str, Any]:
    depth_f = np.asarray(depth, dtype=np.float32)
    valid = np.isfinite(depth_f) & (depth_f > 0.05) & (depth_f < 10.0)

    confidence_summary: dict[str, Any] = {"available": False}
    if confidence is not None and confidence.size:
        conf = np.asarray(confidence)
        confidence_summary = {
            "available": True,
            "min": int(np.min(conf)),
            "max": int(np.max(conf)),
            "mean": float(np.mean(conf)),
        }
        if conf.shape == depth_f.shape:
            valid &= conf > 0

    valid_values = depth_f[valid]
    height, width = depth_f.shape[:2]
    cx0 = int(width * 0.4)
    cx1 = int(width * 0.6)
    cy0 = int(height * 0.4)
    cy1 = int(height * 0.65)
    center_values = depth_f[cy0:cy1, cx0:cx1]
    center_valid = center_values[np.isfinite(center_values) & (center_values > 0.05) & (center_values < 10.0)]

    return {
        "source": "record3d",
        "units": "meters",
        "shape": [int(height), int(width)],
        "valid_fraction": float(valid.mean()) if valid.size else 0.0,
        "center_median_m": _safe_percentile(center_valid, 50),
        "center_p10_m": _safe_percentile(center_valid, 10),
        "frame_median_m": _safe_percentile(valid_values, 50),
        "frame_p10_m": _safe_percentile(valid_values, 10),
        "inside_1m_fraction": float((valid_values < 1.0).mean()) if valid_values.size else 0.0,
        "inside_4m_fraction": float((valid_values < 4.0).mean()) if valid_values.size else 0.0,
        "confidence": confidence_summary,
    }


class Record3DSource:
    """Background USB RGBD reader for the Record3D iOS app."""

    def __init__(self, device_index: int = 0) -> None:
        self.device_index = device_index
        self._frame_event = Event()
        self._stop_event = Event()
        self._lock = Lock()
        self._thread: Thread | None = None
        self._session: Any | None = None
        self._latest: Record3DFrame | None = None
        self._last_error: str | None = None
        self._connected_device: dict[str, Any] | None = None
        self._frames_received = 0
        self._empty_frames_received = 0

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = Thread(target=self._run, daemon=True, name="Record3DSource")
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._frame_event.set()
        if self._thread is not None:
            self._thread.join(timeout=3)
            self._thread = None
        self._close_session()

    def restart(self) -> None:
        self.stop()
        with self._lock:
            self._latest = None
            self._last_error = None
            self._connected_device = None
            self._frames_received = 0
            self._empty_frames_received = 0
        self.start()

    def status(self) -> dict[str, Any]:
        with self._lock:
            latest_age_s = time.time() - self._latest.captured_at if self._latest else None
            streaming = latest_age_s is not None and latest_age_s <= 3.0
            waiting_for_frames = (
                self._thread is not None
                and self._thread.is_alive()
                and self._connected_device is not None
                and self._frames_received == 0
                and self._last_error is None
            )
            return {
                "running": self._thread is not None and self._thread.is_alive(),
                "streaming": streaming,
                "stale": latest_age_s is not None and latest_age_s > 3.0,
                "waiting_for_frames": waiting_for_frames,
                "connected_device": self._connected_device,
                "frames_received": self._frames_received,
                "empty_frames_received": self._empty_frames_received,
                "latest_age_s": latest_age_s,
                "latest_depth_hint": self._latest.depth_hint if self._latest else None,
                "last_error": self._last_error,
            }

    def latest(self) -> Record3DFrame | None:
        with self._lock:
            return self._latest

    def _on_new_frame(self) -> None:
        self._frame_event.set()

    def _on_stream_stopped(self) -> None:
        with self._lock:
            self._last_error = "Record3D stream stopped"
        self._frame_event.set()

    def _close_session(self) -> None:
        session = self._session
        self._session = None
        if session is None:
            return
        for method_name in ("disconnect", "stop", "close"):
            method = getattr(session, method_name, None)
            if callable(method):
                try:
                    method()
                except Exception:
                    logger.debug(f"Record3D session {method_name} failed", exc_info=True)
                break

    def _run(self) -> None:
        try:
            from record3d import Record3DStream

            devices = Record3DStream.get_connected_devices()
            if len(devices) <= self.device_index:
                raise RuntimeError(
                    f"No Record3D device at index {self.device_index}; found {len(devices)} device(s)"
                )

            device = devices[self.device_index]
            with self._lock:
                self._connected_device = {
                    "product_id": getattr(device, "product_id", None),
                    "udid": getattr(device, "udid", None),
                }

            self._session = Record3DStream()
            self._session.on_new_frame = self._on_new_frame
            self._session.on_stream_stopped = self._on_stream_stopped
            self._session.connect(device)
            logger.info("Connected to Record3D USB stream")

            while not self._stop_event.is_set():
                if not self._frame_event.wait(timeout=1.0):
                    continue
                self._frame_event.clear()
                try:
                    self._capture_latest_frame()
                except Exception as exc:
                    logger.warning(f"Record3D frame capture failed: {exc}")
                    with self._lock:
                        self._last_error = str(exc)
        except Exception as exc:
            logger.exception("Record3D source failed")
            with self._lock:
                self._last_error = str(exc)

    def _capture_latest_frame(self) -> None:
        if self._session is None:
            return

        depth = self._session.get_depth_frame()
        rgb = self._session.get_rgb_frame()
        confidence = self._session.get_confidence_frame()

        if rgb is None or depth is None:
            with self._lock:
                self._empty_frames_received += 1
                self._last_error = None
            return

        rgb = np.asarray(rgb)
        depth = np.asarray(depth)
        if rgb.size == 0 or depth.size == 0 or rgb.ndim < 3 or depth.ndim < 2:
            with self._lock:
                self._empty_frames_received += 1
                self._last_error = None
            return

        if self._session.get_device_type() == DEVICE_TYPE_TRUEDEPTH:
            depth = cv2.flip(depth, 1)
            rgb = cv2.flip(rgb, 1)
            if confidence is not None and confidence.size:
                confidence = cv2.flip(confidence, 1)

        image_data_url, jpeg_bytes = _encode_jpeg_data_url(rgb)
        depth_jpeg_bytes = _encode_depth_jpeg(depth, confidence)
        depth_hint = _depth_summary(depth, confidence)
        depth_hint["device_type"] = int(self._session.get_device_type())

        intrinsic = self._session.get_intrinsic_mat()
        depth_hint["intrinsics"] = {
            "fx": float(intrinsic.fx),
            "fy": float(intrinsic.fy),
            "tx": float(intrinsic.tx),
            "ty": float(intrinsic.ty),
        }

        captured_at = time.time()
        with self._lock:
            self._latest = Record3DFrame(
                image_data_url=image_data_url,
                jpeg_bytes=jpeg_bytes,
                depth_jpeg_bytes=depth_jpeg_bytes,
                depth_hint=depth_hint,
                captured_at=captured_at,
            )
            self._frames_received += 1
            self._last_error = None
