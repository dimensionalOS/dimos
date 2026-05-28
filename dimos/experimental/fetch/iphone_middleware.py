#!/usr/bin/env python3
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

import argparse
import asyncio
import base64
from datetime import datetime
from io import BytesIO
import os
from pathlib import Path
from threading import Event, Lock, Thread
import time
from typing import Any, Awaitable, Callable, cast

from dotenv import load_dotenv
from fastapi import WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
import cv2
import numpy as np
from openai import OpenAI
from unitree_webrtc_connect.constants import RTC_TOPIC

from dimos.experimental.fetch.policy import (
    DEFAULT_GEMINI_VISION_MODEL,
    DEFAULT_MAX_RETRIES,
    DEFAULT_OPENAI_VISION_MODEL,
    DEFAULT_REQUEST_TIMEOUT_S,
    FetchPolicy,
    FetchPolicyConfig,
    VisionProvider,
)
from dimos.experimental.fetch.record3d_source import Record3DSource
try:
    from dimos.experimental.fetch.tts import (
        DEFAULT_GEMINI_TTS_MODEL,
        TtsProvider,
        cartesia_tts,
        gemini_live_tts,
        map_voice,
    )
except ModuleNotFoundError:
    from tts import (  # type: ignore[no-redef]
        DEFAULT_GEMINI_TTS_MODEL,
        TtsProvider,
        cartesia_tts,
        gemini_live_tts,
        map_voice,
    )
try:
    from dimos.experimental.fetch.conversation import LiveConversationSession
except ModuleNotFoundError:
    from conversation import LiveConversationSession  # type: ignore[no-redef]
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.unitree.connection import UnitreeWebRTCConnection
from dimos.utils.logging_config import setup_logger
from dimos.utils.path_utils import get_project_root
from dimos.web.dimos_interface.api.server import FastAPIServer

logger = setup_logger()

STATIC_DIR = Path(__file__).parent / "static"
CAPTURE_DIR = STATIC_DIR / "captures"
ICLOUD_FETCH_DIR = Path.home() / "Library/Mobile Documents/com~apple~CloudDocs/fetch"
GOOGLE_DRIVE_FETCH_DIR = (
    Path.home()
    / "Library/CloudStorage/GoogleDrive-seifip@gmail.com/My Drive/robodog-fetch"
)
LOGO_PATH = STATIC_DIR / "logo.png"
DEFAULT_PORT = 8455
DEFAULT_OPENAI_TTS_MODEL = "tts-1"
GO2_LIDAR_STARTUP_TIMEOUT_S = 12.0
GO2_LIDAR_STALE_TIMEOUT_S = 8.0
CONVERSATION_IDLE_TIMEOUT_S = 30.0
GO2_CAPTURE_MAX_WIDTH = 960
GO2_ANALYSIS_MAX_WIDTH = 640


def _safe_percentile(values: np.ndarray, percentile: float) -> float | None:
    if values.size == 0:
        return None
    return float(np.percentile(values, percentile))


def _go2_lidar_depth_hint(pointcloud: PointCloud2) -> dict[str, Any]:
    points, _ = pointcloud.as_numpy()
    points = np.asarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] < 3:
        raise ValueError(f"Expected Go2 lidar point cloud with shape Nx3, got {points.shape}")

    points = points[:, :3]
    finite = np.isfinite(points).all(axis=1)
    points = points[finite]
    distances = np.linalg.norm(points, axis=1)
    valid = (distances > 0.05) & (distances < 10.0)
    points = points[valid]
    distances = distances[valid]

    if distances.size == 0:
        return {
            "source": "go2_lidar",
            "units": "meters",
            "point_count": 0,
            "valid_fraction": 0.0,
            "center_median_m": None,
            "center_p10_m": None,
            "frame_median_m": None,
            "frame_p10_m": None,
            "nearest_m": None,
            "inside_1m_fraction": 0.0,
            "inside_4m_fraction": 0.0,
        }

    height_band = np.abs(points[:, 2]) < 1.6
    front_y = (points[:, 1] > 0.1) & (np.abs(points[:, 0]) < 1.2) & height_band
    front_x = (points[:, 0] > 0.1) & (np.abs(points[:, 1]) < 1.2) & height_band

    center_distances = distances
    center_axis = "all"
    if front_y.any():
        center_distances = distances[front_y]
        center_axis = "+y"
    elif front_x.any():
        center_distances = distances[front_x]
        center_axis = "+x"

    return {
        "source": "go2_lidar",
        "units": "meters",
        "point_count": int(distances.size),
        "valid_fraction": float(distances.size / max(1, finite.size)),
        "center_axis": center_axis,
        "center_median_m": _safe_percentile(center_distances, 50),
        "center_p10_m": _safe_percentile(center_distances, 10),
        "frame_median_m": _safe_percentile(distances, 50),
        "frame_p10_m": _safe_percentile(distances, 10),
        "nearest_m": float(np.min(distances)),
        "inside_1m_fraction": float((distances < 1.0).mean()),
        "inside_4m_fraction": float((distances < 4.0).mean()),
        "front_y_point_count": int(front_y.sum()),
        "front_x_point_count": int(front_x.sum()),
    }


def _go2_lidar_preview_jpeg(pointcloud: PointCloud2, width: int = 640, height: int = 480) -> bytes:
    points, _ = pointcloud.as_numpy()
    points = np.asarray(points, dtype=np.float32)
    image = np.zeros((height, width, 3), dtype=np.uint8)
    image[:] = (16, 24, 28)
    robot_center = (width // 2, int(height * 0.66))

    for meters in range(1, 7):
        radius = int(meters * 42)
        cv2.circle(image, robot_center, radius, (42, 64, 68), 1)
    cv2.line(image, (robot_center[0], 0), (robot_center[0], height), (38, 70, 76), 1)
    cv2.line(image, (0, robot_center[1]), (width, robot_center[1]), (38, 70, 76), 1)

    if points.ndim == 2 and points.shape[1] >= 3:
        points = points[:, :3]
        finite = np.isfinite(points).all(axis=1)
        points = points[finite]
        distances = np.linalg.norm(points, axis=1)
        valid = (distances > 0.05) & (distances < 7.0) & (np.abs(points[:, 2]) < 1.8)
        points = points[valid]
        distances = distances[valid]

        if points.size:
            if len(points) > 50000:
                stride = max(1, len(points) // 50000)
                points = points[::stride]
                distances = distances[::stride]

            scale = 42.0
            px = np.rint(robot_center[0] + points[:, 0] * scale).astype(np.int32)
            py = np.rint(robot_center[1] - points[:, 1] * scale).astype(np.int32)
            inside = (px >= 0) & (px < width) & (py >= 0) & (py < height)
            px = px[inside]
            py = py[inside]
            distances = distances[inside]
            if distances.size:
                colors = cv2.applyColorMap(
                    np.clip((255.0 - distances / 7.0 * 255.0), 0, 255).astype(np.uint8),
                    cv2.COLORMAP_TURBO,
                )
                image[py, px] = colors[:, 0, :]

    cv2.circle(image, robot_center, 9, (235, 245, 245), -1)
    cv2.circle(image, robot_center, 16, (80, 220, 220), 2)
    cv2.putText(image, "Go2 LiDAR", (18, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (230, 245, 245), 2)
    cv2.putText(image, "top-down", (18, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (140, 220, 220), 1)

    ok, buffer = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
    if not ok:
        raise RuntimeError("Failed to encode Go2 lidar preview")
    return buffer.tobytes()


class Go2Source:
    """Persistent Go2 WebRTC camera/control bridge for Fetch."""

    def __init__(self, ip: str, connection_method: str = "local_ap") -> None:
        self.ip = ip
        self.connection_method = connection_method
        self._stop_event = Event()
        self._lock = Lock()
        self._thread: Thread | None = None
        self._conn: UnitreeWebRTCConnection | None = None
        self._video_subscription: Any | None = None
        self._lidar_subscription: Any | None = None
        self._latest: Image | None = None
        self._latest_jpeg: bytes | None = None
        self._latest_data_url: str | None = None
        self._latest_at: float | None = None
        self._latest_lidar_hint: dict[str, Any] | None = None
        self._latest_lidar_jpeg: bytes | None = None
        self._latest_lidar_at: float | None = None
        self._connected_at: float | None = None
        self._lidar_switch_sent_at: float | None = None
        self._frames_received = 0
        self._lidar_frames_received = 0
        self._last_error: str | None = None
        self._lidar_last_error: str | None = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = Thread(target=self._run, daemon=True, name="Go2Source")
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=3)
            self._thread = None
        self._disconnect()

    def status(self) -> dict[str, Any]:
        with self._lock:
            now = time.time()
            latest_age_s = now - self._latest_at if self._latest_at is not None else None
            latest_lidar_age_s = (
                now - self._latest_lidar_at if self._latest_lidar_at is not None else None
            )
            camera_streaming = latest_age_s is not None and latest_age_s <= 3.0
            lidar_streaming = latest_lidar_age_s is not None and latest_lidar_age_s <= 3.0
            return {
                "enabled": True,
                "running": self._thread is not None and self._thread.is_alive(),
                "connected": self._conn is not None,
                "connected_age_s": now - self._connected_at if self._connected_at else None,
                "streaming": camera_streaming,
                "stale": latest_age_s is not None and latest_age_s > 3.0,
                "waiting_for_frames": self._conn is not None and self._frames_received == 0,
                "frames_received": self._frames_received,
                "latest_age_s": latest_age_s,
                "last_error": self._last_error,
                "lidar_streaming": lidar_streaming,
                "lidar_stale": latest_lidar_age_s is not None and latest_lidar_age_s > 3.0,
                "waiting_for_lidar": self._conn is not None and self._lidar_frames_received == 0,
                "lidar_frames_received": self._lidar_frames_received,
                "latest_lidar_age_s": latest_lidar_age_s,
                "latest_depth_hint": self._latest_lidar_hint,
                "lidar_switch_sent": self._lidar_switch_sent_at is not None,
                "lidar_switch_age_s": (
                    now - self._lidar_switch_sent_at if self._lidar_switch_sent_at else None
                ),
                "lidar_last_error": self._lidar_last_error,
            }

    def latest_jpeg(self) -> bytes | None:
        with self._lock:
            return self._latest_jpeg

    def latest_data_url(self) -> str | None:
        with self._lock:
            return self._latest_data_url

    def depth_hint(self) -> dict[str, Any]:
        with self._lock:
            if self._latest_lidar_hint is not None:
                return self._latest_lidar_hint
            return {
                "source": "go2_lidar",
                "units": "meters",
                "available": False,
                "reason": self._lidar_last_error or "waiting_for_lidar",
            }

    def latest_lidar_jpeg(self) -> bytes | None:
        with self._lock:
            return self._latest_lidar_jpeg

    def preflight(self) -> dict[str, Any]:
        def run(conn: UnitreeWebRTCConnection) -> dict[str, Any]:
            responses = {"recovery_stand": self._sport(conn, 1006)}
            time.sleep(2.0)
            responses["balance_stand"] = self._sport(conn, 1002)
            time.sleep(0.6)
            responses["switch_joystick"] = self._sport(conn, 1027, {"data": True})
            return {"enabled": True, "ok": True, "responses": responses}

        return self._with_connection(run)

    def action(self, payload: dict[str, Any]) -> dict[str, Any]:
        action = str(payload.get("action") or "").strip()

        def run(conn: UnitreeWebRTCConnection) -> dict[str, Any]:
            if action == "move":
                linear_x = float(payload.get("linear_x") or 0.0)
                angular_z = float(payload.get("angular_z") or 0.0)
                duration_s = max(0.0, min(2.0, float(payload.get("duration_s") or 0.0)))
                twist = Twist(
                    linear=Vector3(linear_x, 0.0, 0.0),
                    angular=Vector3(0.0, 0.0, angular_z),
                )
                return {"enabled": True, "ok": conn.move(twist, duration=duration_s)}
            if action == "wave":
                return {"enabled": True, "ok": True, "response": self._sport(conn, 1016)}
            if action == "dance":
                return {"enabled": True, "ok": True, "response": self._sport(conn, 1022)}
            if action == "stand" or action == "stop":
                return {"enabled": True, "ok": True, "response": self._sport(conn, 1002)}
            return {"enabled": True, "ok": False, "message": f"Unknown robot action {action!r}"}

        return self._with_connection(run)

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                conn = UnitreeWebRTCConnection(self.ip, connection_method=self.connection_method)
                connected_at = time.time()
                with self._lock:
                    self._conn = conn
                    self._connected_at = connected_at
                    self._last_error = None
                    starting_lidar_frames = self._lidar_frames_received

                self._enable_lidar(conn)
                self._lidar_subscription = conn.lidar_stream().subscribe(
                    self._on_lidar,
                    self._on_lidar_error,
                )
                self._video_subscription = conn.video_stream().subscribe(
                    self._on_image,
                    self._on_video_error,
                )
                logger.info("Connected to Go2 WebRTC camera/control")

                while not self._stop_event.wait(timeout=0.5):
                    now = time.time()
                    with self._lock:
                        conn_missing = self._conn is None
                        lidar_frames = self._lidar_frames_received
                        latest_lidar_at = self._latest_lidar_at
                    if conn_missing:
                        break
                    if (
                        lidar_frames == starting_lidar_frames
                        and now - connected_at > GO2_LIDAR_STARTUP_TIMEOUT_S
                    ):
                        reason = "No Go2 LiDAR frames after connection; continuing camera-only"
                        with self._lock:
                            self._lidar_last_error = reason
                    if (
                        lidar_frames > starting_lidar_frames
                        and latest_lidar_at is not None
                        and now - latest_lidar_at > GO2_LIDAR_STALE_TIMEOUT_S
                    ):
                        reason = "Go2 LiDAR stream went stale; continuing camera-only"
                        with self._lock:
                            self._lidar_last_error = reason
            except Exception as exc:
                logger.error(f"Go2 WebRTC source failed: {exc}")
                with self._lock:
                    self._last_error = str(exc)
                    self._conn = None
                self._disconnect()
                self._stop_event.wait(timeout=2.0)

    def _on_image(self, image: Image) -> None:
        capture_encoded = image.to_base64(quality=82, max_width=GO2_CAPTURE_MAX_WIDTH)
        analysis_encoded = image.to_base64(quality=68, max_width=GO2_ANALYSIS_MAX_WIDTH)
        jpeg = base64.b64decode(capture_encoded)
        with self._lock:
            self._latest = image
            self._latest_jpeg = jpeg
            self._latest_data_url = f"data:image/jpeg;base64,{analysis_encoded}"
            self._latest_at = time.time()
            self._frames_received += 1

    def _on_lidar(self, pointcloud: PointCloud2) -> None:
        try:
            depth_hint = _go2_lidar_depth_hint(pointcloud)
            lidar_jpeg = _go2_lidar_preview_jpeg(pointcloud)
        except Exception as exc:
            logger.warning(f"Go2 lidar summary failed: {exc}")
            with self._lock:
                self._lidar_last_error = str(exc)
            return

        with self._lock:
            self._latest_lidar_hint = depth_hint
            self._latest_lidar_jpeg = lidar_jpeg
            self._latest_lidar_at = time.time()
            self._lidar_frames_received += 1
            self._lidar_last_error = None

    def _on_video_error(self, exc: Exception) -> None:
        logger.warning(f"Go2 WebRTC camera stream failed: {exc}")
        with self._lock:
            self._last_error = str(exc)

    def _on_lidar_error(self, exc: Exception) -> None:
        logger.warning(f"Go2 WebRTC lidar stream failed: {exc}")
        with self._lock:
            self._lidar_last_error = str(exc)

    def _with_connection(self, callback: Any) -> dict[str, Any]:
        with self._lock:
            conn = self._conn
            last_error = self._last_error
        if conn is None:
            return {
                "enabled": True,
                "ok": False,
                "message": f"Go2 is not connected yet: {last_error or 'waiting'}",
            }
        try:
            return callback(conn)
        except Exception as exc:
            logger.error(f"Go2 action failed: {exc}")
            return {"enabled": True, "ok": False, "message": str(exc)}

    @staticmethod
    def _sport(
        conn: UnitreeWebRTCConnection,
        api_id: int,
        parameter: dict[str, Any] | None = None,
    ) -> Any:
        payload: dict[str, Any] = {"api_id": api_id}
        if parameter is not None:
            payload["parameter"] = parameter
        return conn.publish_request(RTC_TOPIC["SPORT_MOD"], payload)

    def _enable_lidar(self, conn: UnitreeWebRTCConnection) -> None:
        def publish() -> None:
            conn.conn.datachannel.pub_sub.publish_without_callback(
                RTC_TOPIC["ULIDAR_SWITCH"],
                "ON",
            )
            conn.conn.datachannel.pub_sub.publish_without_callback(
                RTC_TOPIC["ULIDAR_SWITCH"],
                "on",
            )

        conn.loop.call_soon_threadsafe(publish)
        with self._lock:
            self._lidar_switch_sent_at = time.time()
            self._lidar_last_error = None

    def _disconnect(self) -> None:
        subscriptions = (self._video_subscription, self._lidar_subscription)
        self._video_subscription = None
        self._lidar_subscription = None
        for subscription in subscriptions:
            if subscription is None:
                continue
            try:
                subscription.dispose()
            except Exception:
                logger.debug("Go2 subscription dispose failed", exc_info=True)
        with self._lock:
            conn = self._conn
            self._conn = None
            self._connected_at = None
        if conn is not None:
            try:
                conn.stop()
            except Exception:
                logger.debug("Go2 connection stop failed", exc_info=True)

DEFAULT_REALTIME_MODEL = "gpt-realtime-2"
DEFAULT_REALTIME_REASONING_EFFORT = "low"
REALTIME_REASONING_EFFORTS = ("minimal", "low", "medium", "high", "xhigh")
TTS_PROVIDERS = ("openai", "gemini", "cartesia")


def _validate_tts_provider(value: str) -> TtsProvider:
    if value not in TTS_PROVIDERS:
        raise ValueError(f"TTS provider must be one of {', '.join(TTS_PROVIDERS)}")
    return cast(TtsProvider, value)


def _validate_realtime_reasoning_effort(value: str) -> str:
    if value not in REALTIME_REASONING_EFFORTS:
        raise ValueError(
            "Realtime reasoning effort must be one of "
            f"{', '.join(REALTIME_REASONING_EFFORTS)}"
        )
    return value


def _realtime_instructions() -> str:
    return """
You are the voice of Fetch, a small robot dog beach prototype.
Speak only the exact Fetch dog line provided in the current request.
Do not add extra offers, explanations, narration, greetings, labels, or sound effects.
Keep delivery short, friendly, and upbeat.
Do not identify people or mention sensitive traits.
If the provided line is empty or unclear, stay silent.
""".strip()


def _build_realtime_session_config(
    *,
    model: str,
    voice: str,
    reasoning_effort: str,
) -> dict[str, Any]:
    return {
        "type": "realtime",
        "model": model,
        "output_modalities": ["audio"],
        "instructions": _realtime_instructions(),
        "audio": {
            "output": {
                "voice": voice,
            }
        },
        "reasoning": {
            "effort": _validate_realtime_reasoning_effort(reasoning_effort),
        },
    }


def _jsonable_openai_response(response: Any) -> dict[str, Any]:
    if hasattr(response, "model_dump"):
        dumped = response.model_dump(mode="json")
        return dumped if isinstance(dumped, dict) else {"response": dumped}
    if isinstance(response, dict):
        return response
    return {
        key: value
        for key, value in vars(response).items()
        if not key.startswith("_")
    }


def _brand_fetch_photo(image_bytes: bytes) -> bytes:
    """Bake the Fetch logo into a saved photo instead of only overlaying it in UI."""
    try:
        from PIL import Image as PILImage
    except Exception:
        logger.exception("Pillow is unavailable; saving unbranded photo")
        return image_bytes

    try:
        photo = PILImage.open(BytesIO(image_bytes)).convert("RGBA")
        logo = PILImage.open(LOGO_PATH).convert("RGBA")
    except Exception:
        logger.exception("Could not decode photo/logo; saving unbranded photo")
        return image_bytes

    width, height = photo.size
    alpha_mask = logo.getchannel("A").point(lambda value: 255 if value > 12 else 0)
    logo_bbox = alpha_mask.getbbox()
    if logo_bbox is not None:
        logo = logo.crop(logo_bbox)

    logo_width = max(90, min(260, int(width * 0.18)))
    logo_height = max(1, int(logo.height * (logo_width / logo.width)))
    logo = logo.resize((logo_width, logo_height), PILImage.Resampling.LANCZOS)

    margin = max(18, int(width * 0.035))
    x = max(margin, width - logo_width - margin)
    y = max(margin, height - logo_height - margin)

    logo_alpha = logo.getchannel("A").point(lambda value: int(value * 0.82))
    logo.putalpha(logo_alpha)
    photo.alpha_composite(logo, (x, y))

    output = BytesIO()
    photo.convert("RGB").save(output, format="JPEG", quality=94, optimize=True)
    return output.getvalue()


def _save_fetch_photo(image_bytes: bytes, filename: str) -> tuple[Path, Path, Path]:
    branded_bytes = _brand_fetch_photo(image_bytes)
    CAPTURE_DIR.mkdir(parents=True, exist_ok=True)
    ICLOUD_FETCH_DIR.mkdir(parents=True, exist_ok=True)
    GOOGLE_DRIVE_FETCH_DIR.mkdir(parents=True, exist_ok=True)
    capture_path = CAPTURE_DIR / filename
    icloud_path = ICLOUD_FETCH_DIR / filename
    google_drive_path = GOOGLE_DRIVE_FETCH_DIR / filename
    capture_path.write_bytes(branded_bytes)
    icloud_path.write_bytes(branded_bytes)
    google_drive_path.write_bytes(branded_bytes)
    return capture_path, icloud_path, google_drive_path


class FetchIphoneMiddleware:
    """HTTPS phone-camera middleware for testing the Fetch behavior."""

    def __init__(
        self,
        host: str = "0.0.0.0",
        port: int = DEFAULT_PORT,
        model: str | None = None,
        vision_provider: VisionProvider = "openai",
        tts_provider: TtsProvider = "cartesia",
        tts_model: str = DEFAULT_OPENAI_TTS_MODEL,
        tts_voice: str = "echo",
        enable_realtime: bool = False,
        realtime_model: str = DEFAULT_REALTIME_MODEL,
        realtime_reasoning_effort: str = DEFAULT_REALTIME_REASONING_EFFORT,
        record3d: bool = False,
        record3d_device_index: int = 0,
        robot_ip: str | None = None,
        robot_connection_method: str = "local_ap",
        conversation_mode: str = "off",
        conversation_model: str = DEFAULT_GEMINI_TTS_MODEL,
    ) -> None:
        self.host = host
        self.port = port
        self.tts_provider = _validate_tts_provider(tts_provider)
        self.tts_model = tts_model
        self.tts_voice = tts_voice
        self.conversation_enabled = conversation_mode == "gemini_live"
        self.conversation_model = conversation_model
        self._conv_session: Any | None = None
        self._conversation_active = False
        self.realtime_enabled = bool(enable_realtime and self.tts_provider == "openai")
        self.realtime_model = realtime_model.strip()
        if not self.realtime_model:
            raise ValueError("Realtime model must be a non-empty string")
        self.realtime_reasoning_effort = _validate_realtime_reasoning_effort(
            realtime_reasoning_effort
        )
        self.robot_ip = robot_ip
        self.robot_connection_method = robot_connection_method
        self.policy = FetchPolicy(
            FetchPolicyConfig(model=model, vision_provider=vision_provider)
        )
        self.server = FastAPIServer(
            dev_name="Fetch iPhone Middleware",
            edge_type="Bidirectional",
            host=host,
            port=port,
        )
        self._openai_client: OpenAI | None = None
        self._record3d_source = Record3DSource(record3d_device_index) if record3d else None
        self._go2_source = Go2Source(robot_ip, robot_connection_method) if robot_ip else None
        self._setup_routes()

    def _audio_route(self) -> str:
        if self.conversation_enabled:
            return "gemini_live_conversation"
        return "realtime_then_speak" if self.realtime_enabled else "speak"

    def _provider_availability(self) -> dict[str, bool]:
        """Which TTS/Live providers have an API key configured, so the browser
        can enable only the audio modes that will actually work."""
        return {
            "openai_available": bool(os.getenv("OPENAI_API_KEY")),
            "gemini_available": bool(os.getenv("GEMINI_API_KEY") or os.getenv("GOOGLE_API_KEY")),
            "cartesia_available": bool(os.getenv("CARTESIA_API_KEY")),
        }

    async def _robot_action_async(self, payload: dict[str, Any]) -> dict[str, Any]:
        if self._go2_source is None:
            return {"enabled": False, "ok": False, "message": "Robot IP is not configured"}
        return await asyncio.to_thread(self._go2_source.action, payload)

    async def _analyze_message(
        self, message: dict[str, Any], message_type: str
    ) -> dict[str, Any]:
        phase = str(message.get("interaction_phase") or "find_guest")
        if message_type == "dog_frame":
            if self._go2_source is None:
                return {"type": "error", "message": "Go2 camera is not enabled"}
            status = self._go2_source.status()
            if not status["streaming"]:
                return {"type": "error", "message": "Go2 camera is not streaming yet", "dog": status}
            image_data_url = self._go2_source.latest_data_url()
            if image_data_url is None:
                return {"type": "error", "message": "No Go2 camera frame received yet"}
            decision = await asyncio.to_thread(
                self.policy.analyze_frame, image_data_url, self._go2_source.depth_hint(), phase
            )
            decision["frame_id"] = message.get("frame_id")
            decision["dog"] = self._go2_source.status()
            return decision
        if message_type == "record3d_frame":
            if self._record3d_source is None:
                return {"type": "error", "message": "Record3D is not enabled"}
            frame = self._record3d_source.latest()
            if frame is None:
                return {"type": "error", "message": "No Record3D frame received yet"}
            decision = await asyncio.to_thread(
                self.policy.analyze_frame, frame.image_data_url, frame.depth_hint, phase
            )
            decision["frame_id"] = message.get("frame_id")
            decision["record3d"] = self._record3d_source.status()
            return decision
        image_data_url = str(message.get("image") or "")
        depth_hint = message.get("depth_hint")
        decision = await asyncio.to_thread(
            self.policy.analyze_frame,
            image_data_url,
            depth_hint if isinstance(depth_hint, dict) else None,
            phase,
        )
        decision["frame_id"] = message.get("frame_id")
        return decision

    async def _route_frame(
        self,
        send_json: Callable[[dict[str, Any]], Awaitable[None]],
        message: dict[str, Any],
        message_type: str,
    ) -> None:
        result = await self._analyze_message(message, message_type)
        if result.get("type") == "error":
            await send_json(result)
            return
        session = self._conv_session
        if session is not None and not session.finished:
            session.push_frame_state(result)
            cmd = result.get("simulated_cmd_vel") if isinstance(result.get("simulated_cmd_vel"), dict) else {}
            has_centering_motion = (
                result.get("state") == "wait_for_coke"
                and abs(float(cmd.get("angular_z") or 0.0)) > 0.0
                and float(cmd.get("linear_x") or 0.0) == 0.0
            )
            if not has_centering_motion:
                result["simulated_cmd_vel"] = {"linear_x": 0.0, "angular_z": 0.0, "duration_s": 0.0}
        await send_json(result)

    async def _route_frame_safe(
        self,
        send_json: Callable[[dict[str, Any]], Awaitable[None]],
        message: dict[str, Any],
        message_type: str,
    ) -> None:
        # Used for fire-and-forget frame analysis during a conversation. Without
        # this, an exception (e.g. the client disconnects mid-send) would be an
        # unretrieved task exception and would leave the browser's pending-frame
        # gate stuck, stalling framing hints. Best-effort error keeps it moving.
        try:
            await self._route_frame(send_json, message, message_type)
        except Exception:
            logger.exception("conversation frame routing failed")
            try:
                await send_json({"type": "error", "message": "frame analysis failed"})
            except Exception:
                pass

    async def _start_conversation(
        self,
        send_json: Callable[[dict[str, Any]], Awaitable[None]],
        context: str,
        voice: str | None = None,
        model: str | None = None,
    ) -> None:
        if not self.conversation_enabled:
            await send_json({"type": "error", "message": "Conversation mode is disabled"})
            return
        if self._conv_session is not None and not self._conv_session.finished:
            return
        session = LiveConversationSession(
            emit=send_json,
            robot_action=self._robot_action_async,
            voice=voice or self.tts_voice,
            model=model or self.conversation_model,
            system_context=context,
            idle_timeout_s=CONVERSATION_IDLE_TIMEOUT_S,
        )
        try:
            await session.open()
        except Exception as exc:
            logger.exception("Failed to open conversation session")
            await send_json({"type": "error", "message": f"Conversation error: {exc}"})
            await send_json(
                {"type": "conversation_state", "state": "reset", "data": {"reason": "open_failed"}}
            )
            return
        self._conv_session = session
        self._conversation_active = True
        await send_json({"type": "conversation_state", "state": "active", "data": {}})
        asyncio.create_task(self._run_conversation(session))

    async def _run_conversation(self, session: Any) -> None:
        try:
            await session.run()
        except Exception:
            logger.exception("Conversation session ended with error")
        finally:
            if self._conv_session is session:
                self._conv_session = None
                self._conversation_active = False

    async def _stop_conversation(self) -> None:
        session = self._conv_session
        self._conv_session = None
        self._conversation_active = False
        if session is not None:
            await session.close()

    def _get_openai_client(self) -> OpenAI | None:
        if not os.getenv("OPENAI_API_KEY"):
            return None
        if self._openai_client is None:
            self._openai_client = OpenAI(
                timeout=DEFAULT_REQUEST_TIMEOUT_S,
                max_retries=DEFAULT_MAX_RETRIES,
            )
        return self._openai_client

    def _setup_routes(self) -> None:
        self.server.app.router.routes = [
            route for route in self.server.app.router.routes if getattr(route, "path", None) != "/"
        ]

        @self.server.app.get("/", response_class=HTMLResponse)
        @self.server.app.get("/fetch", response_class=HTMLResponse)
        async def index() -> HTMLResponse:
            return HTMLResponse(
                content=(STATIC_DIR / "index.html").read_text(),
                headers={"Cache-Control": "no-store"},
            )

        @self.server.app.get("/health")
        async def health() -> dict[str, Any]:
            dog_enabled = self._go2_source is not None
            record3d_enabled = self._record3d_source is not None
            return {
                "ok": True,
                "service": "fetch-iphone",
                "port": self.port,
                "vision_source": "dog" if dog_enabled else "record3d" if record3d_enabled else "browser",
                "vision_provider": self.policy.config.vision_provider,
                "vision_model": self.policy.config.model,
                "dog_enabled": dog_enabled,
                "robot_enabled": dog_enabled,
                "record3d_enabled": record3d_enabled,
                "tts_provider": self.tts_provider,
                "audio_route": self._audio_route(),
                "realtime_enabled": self.realtime_enabled,
                "conversation_enabled": self.conversation_enabled,
                "conversation_model": self.conversation_model,
                **self._provider_availability(),
            }

        @self.server.app.post("/robot/preflight")
        async def robot_preflight() -> Any:
            if self._go2_source is None:
                return {"enabled": False, "ok": False, "message": "Robot IP is not configured"}
            return await asyncio.to_thread(self._go2_source.preflight)

        @self.server.app.post("/robot/action")
        async def robot_action(payload: dict[str, Any]) -> Any:
            if self._go2_source is None:
                return {"enabled": False, "ok": False, "message": "Robot IP is not configured"}
            return await asyncio.to_thread(self._go2_source.action, payload)

        @self.server.app.get("/dog/status")
        async def dog_status() -> dict[str, Any]:
            if self._go2_source is None:
                return {"enabled": False}
            return self._go2_source.status()

        @self.server.app.get("/dog/latest.jpg")
        async def dog_latest_jpg() -> Response:
            if self._go2_source is None:
                return JSONResponse({"error": "Go2 camera is not enabled"}, status_code=404)
            frame = self._go2_source.latest_jpeg()
            if frame is None:
                return JSONResponse({"error": "No Go2 camera frame received yet"}, status_code=404)
            return Response(content=frame, media_type="image/jpeg")

        @self.server.app.get("/dog/stream.mjpg")
        async def dog_stream() -> Any:
            if self._go2_source is None:
                return JSONResponse({"error": "Go2 camera is not enabled"}, status_code=404)

            async def stream_frames() -> Any:
                previous: bytes | None = None
                while True:
                    frame = self._go2_source.latest_jpeg()
                    if frame is not None and frame != previous:
                        previous = frame
                        yield (
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n"
                            b"Cache-Control: no-cache\r\n\r\n"
                            + frame
                            + b"\r\n"
                        )
                    await asyncio.sleep(0.03)

            return StreamingResponse(
                stream_frames(),
                media_type="multipart/x-mixed-replace; boundary=frame",
                headers={"Cache-Control": "no-cache"},
            )

        @self.server.app.get("/dog/lidar.jpg")
        async def dog_lidar_jpg() -> Response:
            if self._go2_source is None:
                return JSONResponse({"error": "Go2 LiDAR is not enabled"}, status_code=404)
            frame = self._go2_source.latest_lidar_jpeg()
            if frame is None:
                return JSONResponse({"error": "No Go2 LiDAR frame received yet"}, status_code=404)
            return Response(content=frame, media_type="image/jpeg")

        @self.server.app.get("/dog/lidar-stream.mjpg")
        async def dog_lidar_stream() -> Any:
            if self._go2_source is None:
                return JSONResponse({"error": "Go2 LiDAR is not enabled"}, status_code=404)

            async def stream_frames() -> Any:
                previous: bytes | None = None
                while True:
                    frame = self._go2_source.latest_lidar_jpeg()
                    if frame is not None and frame != previous:
                        previous = frame
                        yield (
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n"
                            b"Cache-Control: no-cache\r\n\r\n"
                            + frame
                            + b"\r\n"
                        )
                    await asyncio.sleep(0.03)

            return StreamingResponse(
                stream_frames(),
                media_type="multipart/x-mixed-replace; boundary=frame",
                headers={"Cache-Control": "no-cache"},
            )

        @self.server.app.post("/dog/analyze")
        async def dog_analyze(payload: dict[str, Any] | None = None) -> Any:
            if self._go2_source is None:
                return JSONResponse({"error": "Go2 camera is not enabled"}, status_code=404)
            status = self._go2_source.status()
            if not status["streaming"]:
                return JSONResponse(
                    {"error": "Go2 camera is not streaming yet", "dog": status},
                    status_code=409,
                )
            image_data_url = self._go2_source.latest_data_url()
            if image_data_url is None:
                return JSONResponse({"error": "No Go2 camera frame received yet"}, status_code=404)
            interaction_phase = str((payload or {}).get("interaction_phase") or "find_guest")
            return await asyncio.to_thread(
                self.policy.analyze_frame,
                image_data_url,
                self._go2_source.depth_hint(),
                interaction_phase,
            )

        @self.server.app.get("/record3d/status")
        async def record3d_status() -> dict[str, Any]:
            if self._record3d_source is None:
                return {"enabled": False}
            return {"enabled": True, **self._record3d_source.status()}

        @self.server.app.post("/record3d/restart")
        async def record3d_restart() -> dict[str, Any]:
            if self._record3d_source is None:
                return {"enabled": False, "restarted": False}
            self._record3d_source.restart()
            return {"enabled": True, "restarted": True, **self._record3d_source.status()}

        @self.server.app.get("/record3d/latest.jpg")
        async def record3d_latest_jpg() -> Response:
            if self._record3d_source is None:
                return JSONResponse({"error": "Record3D is not enabled"}, status_code=404)
            frame = self._record3d_source.latest()
            if frame is None:
                return JSONResponse({"error": "No Record3D frame received yet"}, status_code=404)
            return Response(content=frame.jpeg_bytes, media_type="image/jpeg")

        @self.server.app.get("/record3d/latest-depth.jpg")
        async def record3d_latest_depth_jpg() -> Response:
            if self._record3d_source is None:
                return JSONResponse({"error": "Record3D is not enabled"}, status_code=404)
            frame = self._record3d_source.latest()
            if frame is None:
                return JSONResponse({"error": "No Record3D frame received yet"}, status_code=404)
            return Response(content=frame.depth_jpeg_bytes, media_type="image/jpeg")

        def stream_record3d_jpegs(frame_attr: str) -> StreamingResponse | JSONResponse:
            if self._record3d_source is None:
                return JSONResponse({"error": "Record3D is not enabled"}, status_code=404)

            async def stream_frames() -> Any:
                last_captured_at = 0.0
                while True:
                    frame = self._record3d_source.latest()
                    if frame is not None and frame.captured_at != last_captured_at:
                        last_captured_at = frame.captured_at
                        jpeg_bytes = getattr(frame, frame_attr)
                        yield (
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n"
                            b"Cache-Control: no-cache\r\n\r\n"
                            + jpeg_bytes
                            + b"\r\n"
                        )
                    await asyncio.sleep(0.03)

            return StreamingResponse(
                stream_frames(),
                media_type="multipart/x-mixed-replace; boundary=frame",
                headers={"Cache-Control": "no-cache"},
            )

        @self.server.app.get("/record3d/stream.mjpg")
        async def record3d_stream() -> Any:
            return stream_record3d_jpegs("jpeg_bytes")

        @self.server.app.get("/record3d/stream-depth.mjpg")
        async def record3d_depth_stream() -> Any:
            return stream_record3d_jpegs("depth_jpeg_bytes")

        @self.server.app.post("/record3d/analyze")
        async def record3d_analyze(payload: dict[str, Any] | None = None) -> Any:
            if self._record3d_source is None:
                return JSONResponse({"error": "Record3D is not enabled"}, status_code=404)
            frame = self._record3d_source.latest()
            if frame is None:
                return JSONResponse({"error": "No Record3D frame received yet"}, status_code=404)
            interaction_phase = str((payload or {}).get("interaction_phase") or "find_guest")
            return await asyncio.to_thread(
                self.policy.analyze_frame,
                frame.image_data_url,
                frame.depth_hint,
                interaction_phase,
            )

        @self.server.app.post("/photos/save")
        async def save_photo(payload: dict[str, Any]) -> Any:
            image_bytes: bytes | None = None
            image_data_url = str(payload.get("image_data_url") or "")

            if image_data_url:
                try:
                    _, encoded = image_data_url.split(",", 1)
                    image_bytes = base64.b64decode(encoded)
                except (ValueError, base64.binascii.Error):
                    return JSONResponse({"error": "Invalid image_data_url"}, status_code=400)
            elif self._go2_source is not None and str(payload.get("source") or "") == "dog":
                status = self._go2_source.status()
                if not status["streaming"]:
                    return JSONResponse(
                        {"error": "Go2 camera is not streaming yet", "dog": status},
                        status_code=409,
                    )
                image_bytes = self._go2_source.latest_jpeg()
            elif self._record3d_source is not None:
                frame = self._record3d_source.latest()
                if frame is not None:
                    image_bytes = frame.jpeg_bytes

            if image_bytes is None:
                return JSONResponse({"error": "No image available to save"}, status_code=404)

            filename = f"fetch-{datetime.now().strftime('%Y%m%d-%H%M%S-%f')}.jpg"
            path, icloud_path, google_drive_path = _save_fetch_photo(image_bytes, filename)
            return {
                "saved": True,
                "url": f"/fetch/static/captures/{filename}",
                "path": str(path),
                "icloud_path": str(icloud_path),
                "google_drive_path": str(google_drive_path),
            }

        @self.server.app.post("/speak")
        async def speak(payload: dict[str, Any]) -> Response:
            text = str(payload.get("text") or "").strip()
            if not text:
                return JSONResponse({"error": "Missing text"}, status_code=400)
            if len(text) > 240:
                return JSONResponse({"error": "Text is too long"}, status_code=400)

            voice = str(payload.get("voice") or self.tts_voice)
            try:
                provider = _validate_tts_provider(str(payload.get("provider") or self.tts_provider))
            except ValueError:
                return JSONResponse({"error": "Unknown TTS provider"}, status_code=400)

            if provider == "gemini":
                try:
                    wav_bytes = await gemini_live_tts(
                        text, voice=map_voice(voice, "gemini"),
                    )
                except Exception as exc:
                    logger.exception("Gemini Live TTS failed")
                    return JSONResponse(
                        {"error": f"Gemini TTS error: {exc}"},
                        status_code=503,
                    )
                return Response(content=wav_bytes, media_type="audio/wav")

            if provider == "cartesia":
                try:
                    wav_bytes = await cartesia_tts(text, voice=map_voice(voice, "cartesia"))
                except Exception as exc:
                    logger.exception("Cartesia TTS failed")
                    return JSONResponse(
                        {"error": f"Cartesia TTS error: {exc}"},
                        status_code=503,
                    )
                return Response(content=wav_bytes, media_type="audio/wav")

            openai_client = self._get_openai_client()
            if openai_client is None:
                return JSONResponse(
                    {"error": "OPENAI_API_KEY is not set; speech requires OpenAI TTS"},
                    status_code=503,
                )

            speech = openai_client.audio.speech.create(
                model=self.tts_model,
                voice=map_voice(voice, "openai"),
                input=text,
                response_format="mp3",
            )
            return Response(content=speech.content, media_type="audio/mpeg")

        @self.server.app.post("/realtime/client-secret")
        async def realtime_client_secret(payload: dict[str, Any] | None = None) -> Any:
            if not self.realtime_enabled:
                return JSONResponse(
                    {
                        "error": (
                            "OpenAI Realtime is disabled; start with "
                            "--enable-realtime and --tts-provider openai"
                        )
                    },
                    status_code=404,
                )

            openai_client = self._get_openai_client()
            if openai_client is None:
                return JSONResponse(
                    {"error": "OPENAI_API_KEY is not set; realtime speech requires OpenAI"},
                    status_code=503,
                )

            voice = str((payload or {}).get("voice") or self.tts_voice).strip()
            if not voice:
                return JSONResponse({"error": "Missing voice"}, status_code=400)

            session_config = _build_realtime_session_config(
                model=self.realtime_model,
                voice=voice,
                reasoning_effort=self.realtime_reasoning_effort,
            )
            client_secret = await asyncio.to_thread(
                openai_client.realtime.client_secrets.create,
                session=session_config,
            )
            response_payload = _jsonable_openai_response(client_secret)
            response_payload.setdefault("model", self.realtime_model)
            response_payload.setdefault("voice", voice)
            return response_payload

        if STATIC_DIR.is_dir():
            self.server.app.mount(
                "/fetch/static",
                StaticFiles(directory=str(STATIC_DIR)),
                name="fetch_static",
            )

        @self.server.app.websocket("/fetch/ws")
        async def websocket_endpoint(ws: WebSocket) -> None:
            await ws.accept()
            await ws.send_json(
                {
                    "type": "hello",
                    "service": "fetch-iphone",
                    "model": self.policy.config.model,
                    "vision_provider": self.policy.config.vision_provider,
                    "tts_provider": self.tts_provider,
                    "audio_route": self._audio_route(),
                    "realtime_enabled": self.realtime_enabled,
                    "conversation_enabled": self.conversation_enabled,
                    "conversation_model": self.conversation_model,
                    **self._provider_availability(),
                }
            )
            logger.info("Fetch iPhone client connected")
            send_lock = asyncio.Lock()

            async def send_json(payload: dict[str, Any]) -> None:
                async with send_lock:
                    await ws.send_json(payload)

            try:
                while True:
                    message = await ws.receive_json()
                    message_type = message.get("type")

                    if message_type == "conversation_start":
                        await self._start_conversation(
                            send_json,
                            str(message.get("context") or ""),
                            voice=(str(message.get("voice")) if message.get("voice") else None),
                            model=(str(message.get("model")) if message.get("model") else None),
                        )
                        continue
                    if message_type == "conversation_stop":
                        await self._stop_conversation()
                        continue
                    if message_type == "mic_chunk":
                        session = self._conv_session
                        if session is not None and not session.finished:
                            try:
                                pcm = base64.b64decode(message.get("data") or "")
                            except (ValueError, base64.binascii.Error):
                                pcm = b""
                            if pcm:
                                session.push_mic(pcm)
                        continue

                    if message_type in ("frame", "dog_frame", "record3d_frame"):
                        if self._conversation_active:
                            message["interaction_phase"] = "confirm_coke"
                            asyncio.create_task(
                                self._route_frame_safe(send_json, message, message_type)
                            )
                        else:
                            await self._route_frame(send_json, message, message_type)
                        continue

                    await send_json(
                        {
                            "type": "error",
                            "message": "Expected frame, dog_frame, or record3d_frame message",
                        }
                    )
            except WebSocketDisconnect:
                logger.info("Fetch iPhone client disconnected")
            except Exception as exc:
                logger.exception("Fetch WebSocket error")
                try:
                    await send_json({"type": "error", "message": str(exc)})
                except Exception:
                    pass
            finally:
                await self._stop_conversation()

    def run(self, ssl: bool = True) -> None:
        if self._go2_source is not None:
            self._go2_source.start()
        if self._record3d_source is not None:
            self._record3d_source.start()
        if ssl:
            certs_dir = get_project_root() / "assets" / "teleop_certs"
            self.server.run(ssl=True, ssl_certs_dir=certs_dir)
        else:
            self.server.run()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the Fetch iPhone middleware.")
    parser.add_argument("--host", default="0.0.0.0", help="Bind host for the HTTPS server.")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="Bind port.")
    parser.add_argument(
        "--model",
        default=None,
        help=(
            "Vision model. Defaults to "
            f"{DEFAULT_OPENAI_VISION_MODEL} for OpenAI and "
            f"{DEFAULT_GEMINI_VISION_MODEL} for Gemini."
        ),
    )
    parser.add_argument(
        "--vision-provider",
        choices=("openai", "gemini"),
        default="openai",
        help="Provider for image analysis.",
    )
    parser.add_argument(
        "--tts-provider",
        choices=("openai", "gemini", "cartesia"),
        default="cartesia",
        help="TTS provider. Cartesia Sonic is the default low-latency speech route.",
    )
    parser.add_argument("--tts-model", default=DEFAULT_OPENAI_TTS_MODEL, help="OpenAI TTS model.")
    parser.add_argument("--tts-voice", default="echo", help="TTS voice name (OpenAI or Gemini prebuilt).")
    parser.add_argument(
        "--enable-realtime",
        action="store_true",
        help="Try OpenAI Realtime WebRTC before /speak when using --tts-provider openai.",
    )
    parser.add_argument(
        "--realtime-model",
        default=DEFAULT_REALTIME_MODEL,
        help="OpenAI Realtime model for optional browser voice playback.",
    )
    parser.add_argument(
        "--realtime-reasoning-effort",
        choices=REALTIME_REASONING_EFFORTS,
        default=DEFAULT_REALTIME_REASONING_EFFORT,
        help="Reasoning effort for optional Realtime voice playback.",
    )
    parser.add_argument(
        "--conversation-mode",
        choices=("off", "gemini_live"),
        default="off",
        help="Enable persistent Gemini Live bidirectional conversation on greet.",
    )
    parser.add_argument(
        "--conversation-model",
        default=DEFAULT_GEMINI_TTS_MODEL,
        help="Gemini Live model for two-way conversation (must support audio in + tools).",
    )
    parser.add_argument("--record3d", action="store_true", help="Read RGBD frames from Record3D over USB.")
    parser.add_argument("--record3d-device-index", type=int, default=0, help="Record3D device index.")
    parser.add_argument("--robot-ip", default=None, help="Optional live Unitree Go2 IP for Fetch actions.")
    parser.add_argument(
        "--robot-connection-method",
        default="local_ap",
        help="Unitree WebRTC connection method: auto, local_ap, or local_sta.",
    )
    parser.add_argument("--no-ssl", action="store_true", help="Disable HTTPS for local debugging.")
    args = parser.parse_args()
    try:
        resolved_config = FetchPolicyConfig(
            model=args.model,
            vision_provider=cast(VisionProvider, args.vision_provider),
        )
    except ValueError as exc:
        parser.error(str(exc))
    args.model = resolved_config.model
    args.vision_provider = resolved_config.vision_provider
    return args


def main() -> None:
    load_dotenv(get_project_root() / ".env")
    load_dotenv()
    args = _parse_args()
    middleware = FetchIphoneMiddleware(
        host=args.host,
        port=args.port,
        model=args.model,
        vision_provider=cast(VisionProvider, args.vision_provider),
        tts_provider=cast(TtsProvider, args.tts_provider),
        tts_model=args.tts_model,
        tts_voice=args.tts_voice,
        enable_realtime=args.enable_realtime,
        realtime_model=args.realtime_model,
        realtime_reasoning_effort=args.realtime_reasoning_effort,
        record3d=args.record3d,
        record3d_device_index=args.record3d_device_index,
        robot_ip=args.robot_ip,
        robot_connection_method=args.robot_connection_method,
        conversation_mode=args.conversation_mode,
        conversation_model=args.conversation_model,
    )
    scheme = "http" if args.no_ssl else "https"
    logger.info(
        f"Fetch iPhone middleware running at {scheme}://{args.host}:{args.port}/fetch"
        f" (vision={args.vision_provider}, tts={args.tts_provider})"
    )
    middleware.run(ssl=not args.no_ssl)


if __name__ == "__main__":
    main()
