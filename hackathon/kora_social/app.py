from __future__ import annotations

from dataclasses import asdict, dataclass
import base64
from datetime import datetime
from io import BytesIO
import logging
import math
import os
from pathlib import Path
import json
import re
import shutil
import sqlite3
import subprocess
import threading
import time
from typing import Any
from uuid import uuid4

from dotenv import load_dotenv
from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, Response
from fastapi.staticfiles import StaticFiles
from PIL import Image as PILImage
from PIL import ImageOps
from pydantic import BaseModel, Field
import socketio
from unitree_webrtc_connect.constants import RTC_TOPIC, SPORT_CMD

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.unitree.connection import UnitreeWebRTCConnection
from hackathon.kora_social.openrouter_vision import OpenRouterVision
from hackathon.kora_social.x_poster import XPoster

logger = logging.getLogger("kora_social")

APP_DIR = Path(__file__).parent
STATIC_DIR = APP_DIR / "static"
CAPTURE_DIR = APP_DIR / "captures"
PEOPLE_TAG_DIR = APP_DIR / "people_tags"
MOCK_FRAME_DIR = APP_DIR / "mock_frames"
DB_PATH = APP_DIR / "kora_social.sqlite3"
ROBOT_IP = "192.168.12.1"
MAP_VIS_URL = "http://127.0.0.1:7779"
DEFAULT_MENTION_INTERVAL_SEC = 30 * 60
DEFAULT_X_HANDLE = "@DimensionalKora"
DEFAULT_X_MENTION_QUERY = f"{DEFAULT_X_HANDLE} -from:{DEFAULT_X_HANDLE.removeprefix('@')}"
MOCK_ROBOT = os.getenv("MOCK_ROBOT", "0").lower() in {"1", "true", "yes", "on"}

load_dotenv(APP_DIR.parents[1] / ".env", override=True)


def _env(name: str, default: str, *, legacy: str | None = None) -> str:
    value = os.getenv(name)
    if value is not None:
        return value
    if legacy is not None:
        value = os.getenv(legacy)
        if value is not None:
            return value
    return default


def _env_bool(name: str, default: bool, *, legacy: str | None = None) -> bool:
    value = _env(name, "1" if default else "0", legacy=legacy)
    return value.strip().lower() in {"1", "true", "yes", "on"}


@dataclass
class Capture:
    id: str
    created_at: str
    filename: str
    caption: str
    reason: str
    score: float
    status: str = "draft"
    x_post_id: str | None = None
    x_post_url: str | None = None
    posted_at: str | None = None
    post_error: str | None = None


@dataclass
class PeopleTag:
    id: str
    created_at: str
    handle: str
    notes: str
    filename: str
    enabled: bool = True


@dataclass
class Mention:
    id: str
    source: str
    author_handle: str
    text: str
    url: str | None
    created_at: str | None
    received_at: str
    command_type: str
    command_payload: str
    status: str = "new"
    action_status: str = "pending"
    reply_text: str | None = None
    error: str | None = None
    raw_json: str | None = None


class MoveRequest(BaseModel):
    forward: float = Field(default=0.0, ge=-0.6, le=0.6)
    lateral: float = Field(default=0.0, ge=-0.5, le=0.5)
    yaw: float = Field(default=0.0, ge=-0.8, le=0.8)
    duration: float = Field(default=0.25, ge=0.05, le=0.5)


class SportMoveRequest(BaseModel):
    forward: float = Field(default=0.0, ge=-0.5, le=0.5)
    lateral: float = Field(default=0.0, ge=-0.3, le=0.3)
    yaw: float = Field(default=0.0, ge=-0.5, le=0.5)
    duration: float = Field(default=0.2, ge=0.05, le=0.5)


SAFE_SPORT_COMMANDS: dict[str, str] = {
    "Hello": "Hello",
    "Stretch": "Stretch",
    "Dance1": "Dance1",
    "Dance2": "Dance2",
}

PATROL_ROUTE: list[dict[str, Any]] = [
    {"kind": "capture", "label": "start"},
    {"kind": "move", "label": "step forward", "forward": 0.18, "lateral": 0.0, "yaw": 0.0, "duration": 1.1},
    {"kind": "capture", "label": "forward view"},
    {"kind": "move", "label": "scan left", "forward": 0.0, "lateral": 0.0, "yaw": 0.30, "duration": 1.0},
    {"kind": "capture", "label": "left view"},
    {"kind": "move", "label": "scan right", "forward": 0.0, "lateral": 0.0, "yaw": -0.30, "duration": 2.0},
    {"kind": "capture", "label": "right view"},
    {"kind": "move", "label": "center", "forward": 0.0, "lateral": 0.0, "yaw": 0.30, "duration": 1.0},
    {"kind": "move", "label": "step back", "forward": -0.16, "lateral": 0.0, "yaw": 0.0, "duration": 1.0},
    {"kind": "capture", "label": "return view"},
]


class CaptionUpdateRequest(BaseModel):
    caption: str = Field(min_length=1, max_length=280)


class DescribeRequest(BaseModel):
    question: str = Field(default="What do you see?", min_length=1, max_length=240)


class PeopleTagCreateRequest(BaseModel):
    handle: str = Field(min_length=1, max_length=32)
    notes: str = Field(default="", max_length=180)
    image_base64: str = Field(min_length=1)


class ModeRequest(BaseModel):
    mock: bool


class XModeRequest(BaseModel):
    dry_run: bool


class MentionSettingsRequest(BaseModel):
    interval_sec: int = Field(default=0, ge=0, le=3600)


class MentionInjectRequest(BaseModel):
    author_handle: str = Field(default="@demo", min_length=1, max_length=32)
    text: str = Field(min_length=1, max_length=500)


def _distance2(a: list[float], b: list[float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


class Go2CameraService:
    def __init__(
        self, robot_ip: str, mock_mode: bool = False, mock_frame_dir: Path | None = None
    ) -> None:
        self.robot_ip = robot_ip
        self.mock_mode = mock_mode
        self.mock_frame_dir = mock_frame_dir
        self._lock = threading.RLock()
        self._connection: UnitreeWebRTCConnection | None = None
        self._subscriptions: list[Any] = []
        self._latest_jpeg: bytes | None = None
        self._latest_ts: float | None = None
        self._latest_error: str | None = None
        self._connecting = False
        self._mock_started = False
        self._mock_frames = self._load_mock_frames()
        self._latest_pose: list[float] | None = None
        self._latest_pose_ts: float | None = None
        self._path_points: list[list[float]] = []
        self._lidar_points: list[list[float]] = []
        self._latest_lidar_ts: float | None = None

    def start(self) -> dict[str, Any]:
        if self.mock_mode:
            with self._lock:
                self._mock_started = True
                self._connecting = False
                self._latest_error = None
            self._refresh_mock_frame()
            return self.status()

        with self._lock:
            if self._connection is not None:
                return self.status()
            if self._connecting:
                return self.status()
            self._connecting = True

        thread = threading.Thread(target=self._connect, daemon=True)
        thread.start()
        return self.status()

    def set_mock_mode(self, mock_mode: bool) -> dict[str, Any]:
        if mock_mode == self.mock_mode:
            if mock_mode and not self._mock_started:
                return self.start()
            return self.status()

        if mock_mode:
            self._stop_real_connection()
            with self._lock:
                self.mock_mode = True
                self._mock_started = True
                self._connecting = False
                self._latest_error = None
            self._refresh_mock_frame()
            return self.status()

        with self._lock:
            self.mock_mode = False
            self._mock_started = False
            self._latest_jpeg = None
            self._latest_ts = None
            self._latest_error = None
        return self.status()

    def stop(self) -> dict[str, Any]:
        if self.mock_mode:
            with self._lock:
                self._mock_started = False
                self._connecting = False
            return self.status()

        self._stop_real_connection()
        return self.status()

    def status(self) -> dict[str, Any]:
        if self.mock_mode and self._mock_started:
            self._refresh_mock_frame()
        with self._lock:
            age = None if self._latest_ts is None else max(0.0, time.time() - self._latest_ts)
            return {
                "robot_ip": self.robot_ip,
                "mock": self.mock_mode,
                "connecting": self._connecting,
                "connected": self._connection is not None or self._mock_started,
                "has_frame": self._latest_jpeg is not None,
                "frame_age_sec": age,
                "last_error": self._latest_error,
            }

    def map_status(self) -> dict[str, Any]:
        if self.mock_mode:
            return self._mock_map_status()

        with self._lock:
            pose_age = (
                None if self._latest_pose_ts is None else max(0.0, time.time() - self._latest_pose_ts)
            )
            lidar_age = (
                None
                if self._latest_lidar_ts is None
                else max(0.0, time.time() - self._latest_lidar_ts)
            )
            fresh = (pose_age is not None and pose_age < 5.0) or (
                lidar_age is not None and lidar_age < 5.0
            )
            return {
                "source": "go2-webrtc",
                "connected": self._connection is not None and fresh,
                "last_error": self._latest_error,
                "age_sec": min(
                    [age for age in (pose_age, lidar_age) if age is not None],
                    default=None,
                ),
                "robot_pose": (
                    {"type": "vector", "c": self._latest_pose}
                    if self._latest_pose is not None
                    else None
                ),
                "path": {"type": "path", "points": list(self._path_points)},
                "costmap": None,
                "objects": [{"type": "lidar", "c": point} for point in self._lidar_points],
                "pointcloud": {
                    "type": "pointcloud",
                    "frame_id": "world",
                    "fields": ["x", "y", "z", "intensity"],
                    "points": [
                        [
                            point[0],
                            point[1],
                            point[2] if len(point) > 2 else 0.18,
                            0.75,
                        ]
                        for point in self._lidar_points
                    ],
                },
                "lidar_points": len(self._lidar_points),
            }

    def latest_jpeg(self) -> bytes | None:
        if self.mock_mode and self._mock_started:
            self._refresh_mock_frame()
        with self._lock:
            return self._latest_jpeg

    def connection(self) -> UnitreeWebRTCConnection | None:
        with self._lock:
            return self._connection

    def capture(self) -> bytes:
        frame = self.latest_jpeg()
        if frame is None:
            raise RuntimeError("No camera frame is available yet.")
        return frame

    def _connect(self) -> None:
        try:
            connection = UnitreeWebRTCConnection(self.robot_ip)
            subscriptions = [connection.video_stream().subscribe(self._on_image)]
            try:
                subscriptions.append(connection.odom_stream().subscribe(self._on_odom))
            except Exception as exc:
                with self._lock:
                    self._latest_error = f"odom subscribe failed: {exc}"
            try:
                subscriptions.append(connection.lidar_stream().subscribe(self._on_lidar))
            except Exception as exc:
                with self._lock:
                    self._latest_error = f"lidar subscribe failed: {exc}"
        except Exception as exc:
            with self._lock:
                self._latest_error = str(exc)
                self._connecting = False
                self._connection = None
                self._subscriptions = []
            return

        with self._lock:
            self._connection = connection
            self._subscriptions = subscriptions
            self._connecting = False
            self._latest_error = None

    def _stop_real_connection(self) -> None:
        with self._lock:
            subscriptions = self._subscriptions
            connection = self._connection
            self._subscriptions = []
            self._connection = None
            self._connecting = False

        for subscription in subscriptions:
            try:
                subscription.dispose()
            except Exception:
                pass
        if connection is not None:
            try:
                connection.stop()
            except Exception:
                pass

    def _on_image(self, image: Any) -> None:
        try:
            rgb = image.to_rgb().data
            buffer = BytesIO()
            PILImage.fromarray(rgb).save(buffer, format="JPEG", quality=88)
            jpeg = buffer.getvalue()
        except Exception as exc:
            with self._lock:
                self._latest_error = f"frame encode failed: {exc}"
            return

        with self._lock:
            self._latest_jpeg = jpeg
            self._latest_ts = time.time()
            self._latest_error = None

    def _on_odom(self, pose: Any) -> None:
        try:
            point = [
                float(pose.position.x),
                float(pose.position.y),
                float(getattr(pose, "yaw", 0.0)),
            ]
        except Exception as exc:
            with self._lock:
                self._latest_error = f"odom parse failed: {exc}"
            return

        with self._lock:
            self._latest_pose = point
            self._latest_pose_ts = time.time()
            if self._path_points and _distance2(self._path_points[-1], point) > 3.0:
                self._path_points = []
            if not self._path_points or _distance2(self._path_points[-1], point) > 0.12:
                self._path_points.append([point[0], point[1]])
                self._path_points = self._path_points[-50:]

    def _on_lidar(self, cloud: Any) -> None:
        try:
            import numpy as np

            points = np.asarray(cloud.pointcloud.points)
            if points.size == 0:
                sample: list[list[float]] = []
            else:
                points = points[np.isfinite(points).all(axis=1)]
                stride = max(1, len(points) // 120)
                sample = [
                    [round(float(point[0]), 2), round(float(point[1]), 2)]
                    for point in points[::stride][:120]
                    if abs(float(point[0])) < 12 and abs(float(point[1])) < 12
                ]
        except Exception as exc:
            with self._lock:
                self._latest_error = f"lidar parse failed: {exc}"
            return

        with self._lock:
            self._lidar_points = sample
            self._latest_lidar_ts = time.time()

    def _load_mock_frames(self) -> list[Path]:
        if self.mock_frame_dir is None:
            return []
        frames: list[Path] = []
        for pattern in ("*.jpg", "*.jpeg", "*.png", "*.JPG", "*.JPEG", "*.PNG"):
            frames.extend(self.mock_frame_dir.glob(pattern))
        return sorted(set(frames))

    def _refresh_mock_frame(self) -> None:
        if not self._mock_frames:
            with self._lock:
                self._latest_error = f"No mock frames in {self.mock_frame_dir}"
                self._latest_jpeg = None
                self._latest_ts = None
            return
        index = int(time.time() / 3) % len(self._mock_frames)
        frame_path = self._mock_frames[index]
        try:
            jpeg = frame_path.read_bytes()
        except OSError as exc:
            with self._lock:
                self._latest_error = f"mock frame read failed: {exc}"
            return
        with self._lock:
            self._latest_jpeg = jpeg
            self._latest_ts = time.time()
            self._latest_error = None

    def _mock_map_status(self) -> dict[str, Any]:
        now = time.time()
        points: list[list[float]] = []
        sparse_points: list[list[float]] = []

        for xi in range(-32, 33, 4):
            for yi in range(-24, 25, 4):
                x = xi / 10
                y = yi / 10
                z = 0.015 * math.sin(x * 1.4 + now * 0.3) + 0.01 * math.cos(y * 1.7)
                points.append([round(x, 2), round(y, 2), round(z, 2), 0.2])

        wall_specs = [
            (-3.2, -2.4, -3.2, 2.4),
            (3.2, -2.4, 3.2, 1.7),
            (-3.2, 2.4, 2.4, 2.4),
            (-1.8, -2.4, 2.9, -2.4),
        ]
        for x1, y1, x2, y2 in wall_specs:
            for step in range(36):
                t = step / 35
                x = x1 + (x2 - x1) * t
                y = y1 + (y2 - y1) * t
                for z_step in range(1, 8):
                    z = z_step * 0.18 + 0.05 * math.sin(now + step)
                    points.append([round(x, 2), round(y, 2), round(z, 2), 0.9])

        for idx in range(42):
            angle = idx * (math.tau / 42)
            radius = 1.0 + 0.35 * math.sin(now * 0.5 + idx)
            x = math.cos(angle) * radius
            y = math.sin(angle) * radius
            z = 0.18 + 0.5 * ((idx % 5) / 5)
            sparse_points.append([round(x, 2), round(y, 2), round(z, 2)])
            points.append([round(x, 2), round(y, 2), round(z, 2), 0.75])

        return {
            "source": "mock",
            "connected": self._mock_started,
            "last_error": None,
            "age_sec": 0.0 if self._mock_started else None,
            "robot_pose": {"type": "vector", "c": [0.0, 0.0, now * 0.15]},
            "path": {
                "type": "path",
                "points": [[-1.6, -1.1], [-1.0, -0.7], [-0.35, -0.18], [0.0, 0.0]],
            },
            "costmap": None,
            "objects": [{"type": "mock", "c": point} for point in sparse_points],
            "pointcloud": {
                "type": "pointcloud",
                "frame_id": "world",
                "fields": ["x", "y", "z", "intensity"],
                "points": points,
            },
            "lidar_points": len(points),
        }


class MapHudService:
    def __init__(self, url: str) -> None:
        self.url = url
        self._lock = threading.RLock()
        self._client: socketio.Client | None = None
        self._thread: threading.Thread | None = None
        self._running = threading.Event()
        self._connected = False
        self._last_error: str | None = None
        self._last_update_ts: float | None = None
        self._state: dict[str, Any] = {}

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._running.set()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running.clear()
        with self._lock:
            client = self._client
        if client is not None:
            try:
                client.disconnect()
            except Exception:
                pass

    def status(self) -> dict[str, Any]:
        with self._lock:
            age = None if self._last_update_ts is None else max(0.0, time.time() - self._last_update_ts)
            return {
                "source": self.url,
                "connected": self._connected,
                "last_error": self._last_error,
                "age_sec": age,
                "robot_pose": self._state.get("robot_pose"),
                "path": self._state.get("path"),
                "costmap": self._summarize_costmap(self._state.get("costmap")),
                "objects": self._state.get("objects", []),
                "pointcloud": self._state.get("pointcloud"),
            }

    def _run(self) -> None:
        while self._running.is_set():
            client = socketio.Client(reconnection=False, logger=False, engineio_logger=False)
            self._register_handlers(client)
            with self._lock:
                self._client = client
            try:
                client.connect(self.url, transports=["websocket", "polling"], wait_timeout=1.5)
                client.wait()
            except Exception as exc:
                with self._lock:
                    self._connected = False
                    self._last_error = str(exc)
            finally:
                try:
                    client.disconnect()
                except Exception:
                    pass
            if self._running.is_set():
                time.sleep(2.0)

    def _register_handlers(self, client: socketio.Client) -> None:
        @client.event
        def connect() -> None:
            with self._lock:
                self._connected = True
                self._last_error = None

        @client.event
        def disconnect() -> None:
            with self._lock:
                self._connected = False

        @client.on("full_state")
        def full_state(data: dict[str, Any]) -> None:
            self._merge_state(data)

        @client.on("robot_pose")
        def robot_pose(data: dict[str, Any]) -> None:
            self._merge_state({"robot_pose": data})

        @client.on("path")
        def path(data: dict[str, Any]) -> None:
            self._merge_state({"path": data})

        @client.on("costmap")
        def costmap(data: dict[str, Any]) -> None:
            self._merge_state({"costmap": data})

        @client.on("pointcloud")
        def pointcloud(data: dict[str, Any]) -> None:
            self._merge_state({"pointcloud": data})

    def _merge_state(self, data: dict[str, Any]) -> None:
        with self._lock:
            self._state.update(data)
            self._last_update_ts = time.time()
            self._last_error = None

    def _summarize_costmap(self, costmap: Any) -> dict[str, Any] | None:
        if not isinstance(costmap, dict):
            return None
        grid = costmap.get("grid")
        summary: dict[str, Any] = {
            "type": costmap.get("type"),
            "origin": costmap.get("origin"),
            "resolution": costmap.get("resolution"),
            "origin_theta": costmap.get("origin_theta"),
            "available": grid is not None,
        }
        if isinstance(grid, dict):
            for key in ("width", "height", "shape", "encoding", "full"):
                if key in grid:
                    summary[key] = grid[key]
        return summary


class RobotControlService:
    def __init__(
        self, camera_service: Go2CameraService, draft_store: DraftStore, mock_mode: bool = False
    ) -> None:
        self.camera = camera_service
        self.store = draft_store
        self.mock_mode = mock_mode
        self._lock = threading.RLock()
        self._patrol_cancel = threading.Event()
        self._patrol_thread: threading.Thread | None = None
        self._drive_watchdog: threading.Timer | None = None
        self._last_action = "idle"
        self._last_error: str | None = None
        self._last_command: dict[str, Any] | None = None
        self._command_log: list[dict[str, Any]] = []
        self._walk_control_enabled = False
        self._patrol_step: str | None = None
        self._patrol_step_index = 0
        self._patrol_step_total = len(PATROL_ROUTE)

    def status(self) -> dict[str, Any]:
        with self._lock:
            return {
                "mock": self.mock_mode,
                "ready": self.mock_mode or self.camera.connection() is not None,
                "patrol_active": self._patrol_thread is not None and self._patrol_thread.is_alive(),
                "last_action": self._last_action,
                "last_error": self._last_error,
                "last_command": self._last_command,
                "command_log": list(self._command_log),
                "patrol_step": self._patrol_step,
                "patrol_step_index": self._patrol_step_index,
                "patrol_step_total": self._patrol_step_total,
            }

    def set_mock_mode(self, mock_mode: bool) -> dict[str, Any]:
        self.cancel_patrol()
        with self._lock:
            self.mock_mode = mock_mode
            self._walk_control_enabled = False
            self._last_action = "mock mode" if mock_mode else "live mode"
            self._last_error = None
        return self.status()

    def stand(self) -> dict[str, Any]:
        if self.mock_mode:
            return self._mock_command("stand", {})
        connection = self._require_connection()
        self._set_action("standing")
        ok = connection.standup()
        time.sleep(1.0)
        balance_ok = connection.balance_stand()
        joystick_ok = self._enable_walk_control(connection)
        self._set_action("ready" if ok and balance_ok and joystick_ok else "stand requested")
        return self.status()

    def lie_down(self) -> dict[str, Any]:
        self.cancel_patrol()
        if self.mock_mode:
            self._walk_control_enabled = False
            return self._mock_command("lie_down", {})
        connection = self._require_connection()
        self.stop_motion()
        self._walk_control_enabled = False
        self._set_action("lying down")
        connection.liedown()
        self._set_action("resting")
        return self.status()

    def move(self, request: MoveRequest) -> dict[str, Any]:
        if self.mock_mode:
            return self._mock_command("move", request.model_dump())
        connection = self._require_connection()
        self._enable_walk_control(connection)
        self._set_action(
            f"move f={request.forward:.2f} l={request.lateral:.2f} y={request.yaw:.2f}"
        )
        joystick_data = {
            "lx": -request.lateral,
            "ly": request.forward,
            "rx": -request.yaw,
            "ry": 0,
        }
        result = connection.move(
            Twist(
                linear=Vector3(request.forward, request.lateral, 0.0),
                angular=Vector3(0.0, 0.0, request.yaw),
            ),
            duration=request.duration,
        )
        self._set_command(
            {
                "kind": "move",
                "request": request.model_dump(),
                "wireless_controller": joystick_data,
                "result": result,
            }
        )
        self.stop_motion()
        self._set_action("ready")
        return self.status()

    def drive(self, request: MoveRequest) -> dict[str, Any]:
        if self.mock_mode:
            return self._mock_command("drive", request.model_dump())
        connection = self._require_connection()
        self._enable_walk_control(connection)
        self._set_action(
            f"drive f={request.forward:.2f} l={request.lateral:.2f} y={request.yaw:.2f}"
        )
        move_payload = self._sport_move_payload(request.forward, request.lateral, request.yaw)
        response = connection.publish_request(RTC_TOPIC["SPORT_MOD"], move_payload)
        self._set_command(
            {
                "kind": "drive",
                "request": request.model_dump(),
                "sport_payload": move_payload,
                "response": response,
                "result": bool(response),
            }
        )
        self._schedule_drive_watchdog()
        return self.status()

    def sport_move(self, request: SportMoveRequest) -> dict[str, Any]:
        if self.mock_mode:
            return self._mock_command("sport_move", request.model_dump())
        connection = self._require_connection()
        self._enable_walk_control(connection)
        self._set_action(
            f"sport move f={request.forward:.2f} l={request.lateral:.2f} y={request.yaw:.2f}"
        )
        move_payload = self._sport_move_payload(request.forward, request.lateral, request.yaw)
        stop_payload = self._sport_move_payload(0, 0, 0)
        response = connection.publish_request(RTC_TOPIC["SPORT_MOD"], move_payload)
        time.sleep(request.duration)
        stop_response = connection.publish_request(RTC_TOPIC["SPORT_MOD"], stop_payload)
        self._set_command(
            {
                "kind": "sport_move",
                "request": request.model_dump(),
                "sport_payload": move_payload,
                "response": response,
                "stop_response": stop_response,
            }
        )
        self._set_action("ready")
        return self.status()

    def stop_motion(self) -> dict[str, Any]:
        if self.mock_mode:
            return self._mock_command("stop", {"forward": 0, "lateral": 0, "yaw": 0})
        connection = self._require_connection()
        self._cancel_drive_watchdog()
        stop_payload = self._sport_move_payload(0, 0, 0)
        response = connection.publish_request(RTC_TOPIC["SPORT_MOD"], stop_payload)
        self._set_command(
            {
                "kind": "stop",
                "sport_payload": stop_payload,
                "response": response,
                "result": bool(response),
            }
        )
        self._set_action("stopped")
        return self.status()

    def hello(self) -> dict[str, Any]:
        return self.sport_command("Hello")

    def sport_command(self, command_name: str) -> dict[str, Any]:
        command_key = SAFE_SPORT_COMMANDS.get(command_name)
        if command_key is None:
            raise ValueError(f"Unsupported sport command: {command_name}")
        if self.mock_mode:
            return self._mock_command(command_key.lower(), {})
        connection = self._require_connection()
        self._set_action(command_key.lower())
        connection.publish_request(RTC_TOPIC["SPORT_MOD"], {"api_id": SPORT_CMD[command_key]})
        return self.status()


    def start_tiny_patrol(self) -> dict[str, Any]:
        if self.mock_mode and self.camera.mock_mode:
            self.camera.start()
        elif self.mock_mode:
            self.camera.set_mock_mode(True)
        else:
            self._require_connection()
        with self._lock:
            if self._patrol_thread is not None and self._patrol_thread.is_alive():
                return self.status()
            self._patrol_cancel.clear()
            self._patrol_step = "starting"
            self._patrol_step_index = 0
            self._patrol_step_total = len(PATROL_ROUTE)
            self._patrol_thread = threading.Thread(target=self._tiny_patrol_loop, daemon=True)
            self._patrol_thread.start()
        return self.status()

    def cancel_patrol(self) -> dict[str, Any]:
        was_active = self._patrol_thread is not None and self._patrol_thread.is_alive()
        self._patrol_cancel.set()
        if self.camera.connection() is not None:
            try:
                self.stop_motion()
            except Exception:
                pass
        if was_active:
            with self._lock:
                self._patrol_step = "cancelled"
            self._set_action("patrol cancelled")
        return self.status()

    def _tiny_patrol_loop(self) -> None:
        try:
            self._set_action("patrol starting")
            for index, step in enumerate(PATROL_ROUTE, start=1):
                if self._patrol_cancel.is_set():
                    break
                self._set_patrol_step(index, str(step["label"]))
                if step["kind"] == "capture":
                    self._capture_patrol_draft(str(step["label"]))
                elif step["kind"] == "move":
                    self._run_patrol_move(step)
                self._sleep_or_cancel(0.25)
            self.stop_motion()
            if self._patrol_cancel.is_set():
                self._set_action("patrol cancelled")
                with self._lock:
                    self._patrol_step = "cancelled"
            else:
                self._set_action("patrol complete")
                with self._lock:
                    self._patrol_step = "complete"
        except Exception as exc:
            self._set_error(str(exc))
            try:
                self.stop_motion()
            except Exception:
                pass

    def _run_patrol_move(self, step: dict[str, Any]) -> None:
        request = {
            "forward": float(step.get("forward", 0.0)),
            "lateral": float(step.get("lateral", 0.0)),
            "yaw": float(step.get("yaw", 0.0)),
            "duration": float(step.get("duration", 0.0)),
        }
        if self.mock_mode:
            self._set_action(
                f"mock patrol {step['label']} f={request['forward']:.2f} "
                f"l={request['lateral']:.2f} y={request['yaw']:.2f}"
            )
            self._set_command(
                {
                    "kind": "patrol_move",
                    "request": request,
                    "mock": True,
                    "result": True,
                }
            )
            self._sleep_or_cancel(request["duration"])
            return

        connection = self._require_connection()
        self._enable_walk_control(connection)
        move_payload = self._sport_move_payload(
            request["forward"], request["lateral"], request["yaw"]
        )
        response = connection.publish_request(RTC_TOPIC["SPORT_MOD"], move_payload)
        self._set_command(
            {
                "kind": "patrol_move",
                "request": request,
                "sport_payload": move_payload,
                "response": response,
                "result": bool(response),
            }
        )
        self._sleep_or_cancel(request["duration"])
        self.stop_motion()

    def _capture_patrol_draft(self, label: str) -> None:
        try:
            jpeg = self.camera.capture()
        except RuntimeError:
            self._set_action("patrol waiting for frame")
            return

        def create_draft() -> None:
            try:
                self.store.create(jpeg)
                self._set_action(f"patrol captured {label}")
            except Exception as exc:
                self._set_error(f"patrol capture failed: {exc}")

        thread = threading.Thread(target=create_draft, daemon=True)
        thread.start()
        self._set_action(f"patrol queued capture {label}")

    def _set_patrol_step(self, index: int, label: str) -> None:
        with self._lock:
            self._patrol_step_index = index
            self._patrol_step = label

    def _sleep_or_cancel(self, duration: float) -> None:
        deadline = time.time() + duration
        while time.time() < deadline:
            if self._patrol_cancel.is_set():
                return
            time.sleep(min(0.05, max(0.0, deadline - time.time())))

    def _require_connection(self) -> UnitreeWebRTCConnection:
        connection = self.camera.connection()
        if connection is None:
            raise RuntimeError("Start the camera/robot connection first.")
        return connection

    def _mock_command(self, kind: str, payload: dict[str, Any]) -> dict[str, Any]:
        now = time.time()
        with self._lock:
            previous = self._command_log[0] if self._command_log else None
            delta_ms = None if previous is None else round((now - previous["ts"]) * 1000)
            command = {
                "kind": kind,
                "request": payload,
                "mock": True,
                "result": True,
                "ts": now,
                "delta_ms": delta_ms,
            }
            self._last_command = command
            self._last_action = f"mock {kind}"
            self._last_error = None
            self._command_log.insert(0, command)
            self._command_log = self._command_log[:40]
        return self.status()

    def _enable_walk_control(self, connection: UnitreeWebRTCConnection) -> bool:
        if self._walk_control_enabled:
            return True
        try:
            balance_response = connection.balance_stand()
            free_walk_response = connection.free_walk()
            response = connection.publish_request(
                RTC_TOPIC["SPORT_MOD"],
                {
                    "api_id": SPORT_CMD["SwitchJoystick"],
                    "parameter": {"data": True},
                },
            )
            self._set_command(
                {
                    "kind": "enable_walk_control",
                    "balance": balance_response,
                    "free_walk": free_walk_response,
                    "switch_joystick": response,
                }
            )
            self._walk_control_enabled = bool(response)
            return bool(response)
        except Exception as exc:
            self._set_error(f"walk control failed: {exc}")
            return False

    def _schedule_drive_watchdog(self) -> None:
        self._cancel_drive_watchdog()
        self._drive_watchdog = threading.Timer(0.35, self._watchdog_stop_motion)
        self._drive_watchdog.daemon = True
        self._drive_watchdog.start()

    def _cancel_drive_watchdog(self) -> None:
        if self._drive_watchdog is not None:
            self._drive_watchdog.cancel()
            self._drive_watchdog = None

    def _watchdog_stop_motion(self) -> None:
        connection = self.camera.connection()
        if connection is None:
            return
        stop_payload = self._sport_move_payload(0, 0, 0)
        response = connection.publish_request(RTC_TOPIC["SPORT_MOD"], stop_payload)
        self._set_command(
            {
                "kind": "watchdog_stop",
                "sport_payload": stop_payload,
                "response": response,
                "result": bool(response),
            }
        )
        self._set_action("stopped")

    def _sport_move_payload(self, forward: float, lateral: float, yaw: float) -> dict[str, Any]:
        return {
            "api_id": SPORT_CMD["Move"],
            "parameter": {"x": forward, "y": lateral, "z": yaw},
        }

    def _set_action(self, action: str) -> None:
        with self._lock:
            self._last_action = action
            self._last_error = None

    def _set_error(self, error: str) -> None:
        with self._lock:
            self._last_error = error

    def _set_command(self, command: dict[str, Any]) -> None:
        now = time.time()
        with self._lock:
            previous = self._command_log[0] if self._command_log else None
            command = {
                **command,
                "ts": now,
                "delta_ms": None if previous is None else round((now - previous["ts"]) * 1000),
            }
            self._last_command = command
            self._command_log.insert(0, command)
            self._command_log = self._command_log[:40]
        if command.get("kind") in {"drive", "move", "patrol_move", "stop", "watchdog_stop"}:
            req = command.get("request", {})
            wc = command.get("wireless_controller", {})
            sport_payload = command.get("sport_payload", {})
            logger.warning(
                "robot command kind=%s req=%s wc=%s sport=%s result=%s delta_ms=%s",
                command.get("kind"),
                req,
                wc,
                sport_payload,
                command.get("result"),
                command.get("delta_ms"),
            )


class PeopleTagStore:
    def __init__(self, image_dir: Path, db_path: Path) -> None:
        self.image_dir = image_dir
        self.db_path = db_path
        self.image_dir.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._db = sqlite3.connect(self.db_path, check_same_thread=False)
        self._db.row_factory = sqlite3.Row
        with self._lock:
            self._db.execute("PRAGMA journal_mode=WAL")
            self._db.execute("PRAGMA foreign_keys=ON")
            self._db.execute(
                """
                CREATE TABLE IF NOT EXISTS people_tags (
                    id TEXT PRIMARY KEY,
                    created_at TEXT NOT NULL,
                    handle TEXT NOT NULL UNIQUE,
                    notes TEXT NOT NULL DEFAULT '',
                    filename TEXT NOT NULL,
                    enabled INTEGER NOT NULL DEFAULT 1
                )
                """
            )
            self._db.execute(
                "CREATE INDEX IF NOT EXISTS idx_people_tags_handle ON people_tags(handle)"
            )
            self._db.commit()

    def list(self) -> list[dict[str, Any]]:
        with self._lock:
            rows = self._db.execute(
                "SELECT * FROM people_tags ORDER BY enabled DESC, handle ASC"
            ).fetchall()
            return [asdict(_people_tag_from_row(row)) for row in rows]

    def create(self, handle: str, notes: str, image: bytes) -> PeopleTag:
        clean_handle = _clean_handle(handle)
        clean_notes = " ".join(notes.strip().split())[:180]
        tag_id = uuid4().hex[:10]
        filename = f"{tag_id}.jpg"
        _normalize_tag_image(image, self.image_dir / filename)
        tag = PeopleTag(
            id=tag_id,
            created_at=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            handle=clean_handle,
            notes=clean_notes,
            filename=filename,
        )
        with self._lock:
            existing = self._db.execute(
                "SELECT filename FROM people_tags WHERE handle = ?", (tag.handle,)
            ).fetchone()
            if existing is not None:
                raise ValueError(f"Tag {tag.handle} already exists.")
            self._save_locked(tag)
        return tag

    def set_enabled(self, tag_id: str, enabled: bool) -> PeopleTag:
        with self._lock:
            tag = self._get_locked(tag_id)
            tag.enabled = enabled
            self._save_locked(tag)
            return tag

    def delete(self, tag_id: str) -> None:
        with self._lock:
            tag = self._get_locked(tag_id)
            self._db.execute("DELETE FROM people_tags WHERE id = ?", (tag_id,))
            self._db.commit()
        try:
            (self.image_dir / tag.filename).unlink()
        except FileNotFoundError:
            pass

    def enabled_candidates(self) -> list[dict[str, Any]]:
        with self._lock:
            rows = self._db.execute(
                "SELECT * FROM people_tags WHERE enabled = 1 ORDER BY handle ASC"
            ).fetchall()
            tags = [_people_tag_from_row(row) for row in rows]
        candidates: list[dict[str, Any]] = []
        for tag in tags:
            try:
                jpeg = (self.image_dir / tag.filename).read_bytes()
            except FileNotFoundError:
                continue
            candidates.append({"handle": tag.handle, "notes": tag.notes, "jpeg": jpeg})
        return candidates

    def _get_locked(self, tag_id: str) -> PeopleTag:
        row = self._db.execute("SELECT * FROM people_tags WHERE id = ?", (tag_id,)).fetchone()
        if row is not None:
            return _people_tag_from_row(row)
        raise KeyError(tag_id)

    def _save_locked(self, tag: PeopleTag) -> None:
        self._db.execute(
            """
            INSERT INTO people_tags (
                id, created_at, handle, notes, filename, enabled
            )
            VALUES (
                :id, :created_at, :handle, :notes, :filename, :enabled
            )
            ON CONFLICT(id) DO UPDATE SET
                created_at = excluded.created_at,
                handle = excluded.handle,
                notes = excluded.notes,
                filename = excluded.filename,
                enabled = excluded.enabled
            """,
            {**asdict(tag), "enabled": int(tag.enabled)},
        )
        self._db.commit()


class DraftStore:
    def __init__(
        self,
        capture_dir: Path,
        db_path: Path,
        vision: OpenRouterVision | None = None,
        people_tags: PeopleTagStore | None = None,
    ) -> None:
        self.capture_dir = capture_dir
        self.db_path = db_path
        self.vision = vision
        self.people_tags = people_tags
        self.capture_dir.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._db = sqlite3.connect(self.db_path, check_same_thread=False)
        self._db.row_factory = sqlite3.Row
        with self._lock:
            self._db.execute("PRAGMA journal_mode=WAL")
            self._db.execute("PRAGMA foreign_keys=ON")
            self._db.execute(
                """
                CREATE TABLE IF NOT EXISTS captures (
                    id TEXT PRIMARY KEY,
                    created_at TEXT NOT NULL,
                    filename TEXT NOT NULL,
                    caption TEXT NOT NULL,
                    reason TEXT NOT NULL,
                    score REAL NOT NULL,
                    status TEXT NOT NULL DEFAULT 'draft',
                    x_post_id TEXT,
                    x_post_url TEXT,
                    posted_at TEXT,
                    post_error TEXT
                )
                """
            )
            self._db.execute(
                "CREATE INDEX IF NOT EXISTS idx_captures_created_at ON captures(created_at DESC)"
            )
            self._db.execute("DELETE FROM captures WHERE status = 'skipped'")
            self._db.commit()

    def list(self) -> list[dict[str, Any]]:
        with self._lock:
            rows = self._db.execute(
                "SELECT * FROM captures ORDER BY created_at DESC, id DESC"
            ).fetchall()
            return [asdict(_capture_from_row(row)) for row in rows]

    def create(self, jpeg: bytes) -> Capture:
        capture_id = uuid4().hex[:10]
        created_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        filename = f"{capture_id}.jpg"
        path = self.capture_dir / filename
        path.write_bytes(jpeg)

        caption, reason, score, status, post_error = self._draft_post(jpeg)
        capture = Capture(
            id=capture_id,
            created_at=created_at,
            filename=filename,
            caption=caption,
            reason=reason,
            score=score,
            status=status,
            post_error=post_error,
        )
        with self._lock:
            self._save_locked(capture)
        return capture

    def _draft_post(self, jpeg: bytes) -> tuple[str, str, float, str, str | None]:
        if self.vision is None or not self.vision.enabled():
            return (
                "",
                "OpenRouter vision is not configured. Add OPENROUTER_API_KEY to .env.",
                0.0,
                "failed",
                "OpenRouter vision is not configured.",
            )
        try:
            post = self.vision.draft_post(jpeg)
        except Exception as exc:
            return (
                "",
                "Vision caption failed. The image was saved; edit the text manually or retry capture.",
                0.0,
                "failed",
                str(exc),
            )
        caption = post.caption
        if self.people_tags is not None:
            try:
                handles = self.vision.suggest_tags(jpeg, self.people_tags.enabled_candidates())
            except Exception:
                handles = []
            for handle in handles:
                if handle not in caption:
                    next_caption = f"{caption} {handle}".strip()
                    if len(next_caption) <= 280:
                        caption = next_caption
        return caption, post.reason, post.score, "draft", None

    def update_status(self, capture_id: str, status: str) -> Capture:
        if status not in {"draft", "queued", "posted", "posting", "failed"}:
            raise ValueError(f"Unsupported status {status!r}")
        with self._lock:
            capture = self._get_locked(capture_id)
            capture.status = status
            if status != "failed":
                capture.post_error = None
            self._save_locked(capture)
            return capture

    def delete(self, capture_id: str) -> None:
        with self._lock:
            capture = self._get_locked(capture_id)
            self._db.execute("DELETE FROM captures WHERE id = ?", (capture_id,))
            self._db.commit()
        try:
            (self.capture_dir / capture.filename).unlink()
        except FileNotFoundError:
            pass

    def update_caption(self, capture_id: str, caption: str) -> Capture:
        clean_caption = " ".join(caption.strip().split())
        if not clean_caption:
            raise ValueError("Caption cannot be empty.")
        with self._lock:
            capture = self._get_locked(capture_id)
            if capture.status in {"posted", "posting"}:
                raise ValueError(f"Cannot edit a {capture.status} draft.")
            capture.caption = clean_caption
            capture.status = "draft"
            capture.post_error = None
            self._save_locked(capture)
            return capture

    def post(self, capture_id: str, poster: XPoster) -> Capture:
        with self._lock:
            capture = self._get_locked(capture_id)
            if capture.status == "posted":
                return capture
            capture.status = "posting"
            capture.post_error = None
            self._save_locked(capture)
            image_path = self.capture_dir / capture.filename
            caption = capture.caption

        try:
            result = poster.post_image(caption, image_path)
        except Exception as exc:
            with self._lock:
                capture = self._get_locked(capture_id)
                capture.status = "failed"
                capture.post_error = str(exc)
                self._save_locked(capture)
                return capture

        with self._lock:
            capture = self._get_locked(capture_id)
            capture.status = "posted"
            capture.x_post_id = result.post_id
            capture.x_post_url = result.post_url
            capture.posted_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            capture.post_error = "dry run" if result.dry_run else None
            self._save_locked(capture)
            return capture

    def _get_locked(self, capture_id: str) -> Capture:
        row = self._db.execute("SELECT * FROM captures WHERE id = ?", (capture_id,)).fetchone()
        if row is not None:
            return _capture_from_row(row)
        raise KeyError(capture_id)

    def _save_locked(self, capture: Capture) -> None:
        self._db.execute(
            """
            INSERT INTO captures (
                id, created_at, filename, caption, reason, score, status,
                x_post_id, x_post_url, posted_at, post_error
            )
            VALUES (
                :id, :created_at, :filename, :caption, :reason, :score, :status,
                :x_post_id, :x_post_url, :posted_at, :post_error
            )
            ON CONFLICT(id) DO UPDATE SET
                created_at = excluded.created_at,
                filename = excluded.filename,
                caption = excluded.caption,
                reason = excluded.reason,
                score = excluded.score,
                status = excluded.status,
                x_post_id = excluded.x_post_id,
                x_post_url = excluded.x_post_url,
                posted_at = excluded.posted_at,
                post_error = excluded.post_error
            """,
            asdict(capture),
        )
        self._db.commit()


class MentionStore:
    def __init__(self, db_path: Path) -> None:
        self.db_path = db_path
        self._lock = threading.RLock()
        self._db = sqlite3.connect(self.db_path, check_same_thread=False)
        self._db.row_factory = sqlite3.Row
        with self._lock:
            self._db.execute("PRAGMA journal_mode=WAL")
            self._db.execute("PRAGMA foreign_keys=ON")
            self._db.execute(
                """
                CREATE TABLE IF NOT EXISTS mentions (
                    id TEXT PRIMARY KEY,
                    source TEXT NOT NULL,
                    author_handle TEXT NOT NULL,
                    text TEXT NOT NULL,
                    url TEXT,
                    created_at TEXT,
                    received_at TEXT NOT NULL,
                    command_type TEXT NOT NULL,
                    command_payload TEXT NOT NULL,
                    status TEXT NOT NULL DEFAULT 'new',
                    action_status TEXT NOT NULL DEFAULT 'pending',
                    reply_text TEXT,
                    error TEXT,
                    raw_json TEXT
                )
                """
            )
            self._db.execute(
                "CREATE INDEX IF NOT EXISTS idx_mentions_received_at ON mentions(received_at DESC)"
            )
            self._db.execute(
                "CREATE INDEX IF NOT EXISTS idx_mentions_action_status ON mentions(action_status)"
            )
            self._db.commit()

    def list(self, limit: int = 50) -> list[dict[str, Any]]:
        with self._lock:
            rows = self._db.execute(
                "SELECT * FROM mentions ORDER BY received_at DESC, id DESC LIMIT ?",
                (limit,),
            ).fetchall()
            return [asdict(_mention_from_row(row)) for row in rows]

    def upsert_many(self, mentions: list[Mention]) -> int:
        inserted = 0
        with self._lock:
            for mention in mentions:
                existing = self._db.execute(
                    "SELECT id FROM mentions WHERE id = ?", (mention.id,)
                ).fetchone()
                if existing is None:
                    inserted += 1
                self._save_locked(mention)
            self._db.commit()
        return inserted

    def inject(self, author_handle: str, text: str) -> Mention:
        mention_id = f"local-{uuid4().hex[:10]}"
        mention = _build_mention(
            mention_id=mention_id,
            source="local",
            author_handle=_clean_handle(author_handle),
            text=text,
            url=None,
            created_at=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            raw=None,
        )
        self.upsert_many([mention])
        return mention

    def get(self, mention_id: str) -> Mention:
        with self._lock:
            return self._get_locked(mention_id)

    def update_action(
        self,
        mention_id: str,
        *,
        action_status: str,
        status: str = "processed",
        reply_text: str | None = None,
        error: str | None = None,
    ) -> Mention:
        if action_status not in {"pending", "done", "failed", "ignored"}:
            raise ValueError(f"Unsupported action status {action_status!r}")
        with self._lock:
            mention = self._get_locked(mention_id)
            mention.status = status
            mention.action_status = action_status
            mention.reply_text = reply_text
            mention.error = error
            self._save_locked(mention)
            self._db.commit()
            return mention

    def delete(self, mention_id: str) -> None:
        with self._lock:
            self._db.execute("DELETE FROM mentions WHERE id = ?", (mention_id,))
            self._db.commit()

    def _get_locked(self, mention_id: str) -> Mention:
        row = self._db.execute("SELECT * FROM mentions WHERE id = ?", (mention_id,)).fetchone()
        if row is not None:
            return _mention_from_row(row)
        raise KeyError(mention_id)

    def _save_locked(self, mention: Mention) -> None:
        self._db.execute(
            """
            INSERT INTO mentions (
                id, source, author_handle, text, url, created_at, received_at,
                command_type, command_payload, status, action_status, reply_text,
                error, raw_json
            )
            VALUES (
                :id, :source, :author_handle, :text, :url, :created_at, :received_at,
                :command_type, :command_payload, :status, :action_status, :reply_text,
                :error, :raw_json
            )
            ON CONFLICT(id) DO UPDATE SET
                source = excluded.source,
                author_handle = excluded.author_handle,
                text = excluded.text,
                url = excluded.url,
                created_at = excluded.created_at,
                command_type = excluded.command_type,
                command_payload = excluded.command_payload,
                raw_json = excluded.raw_json
            """,
            asdict(mention),
        )


class AppSettingsStore:
    def __init__(self, db_path: Path) -> None:
        self.db_path = db_path
        self._lock = threading.RLock()
        self._db = sqlite3.connect(self.db_path, check_same_thread=False)
        self._db.row_factory = sqlite3.Row
        with self._lock:
            self._db.execute("PRAGMA journal_mode=WAL")
            self._db.execute(
                """
                CREATE TABLE IF NOT EXISTS app_settings (
                    key TEXT PRIMARY KEY,
                    value TEXT NOT NULL,
                    updated_at TEXT NOT NULL
                )
                """
            )
            self._db.commit()

    def get_int(self, key: str, default: int) -> int:
        with self._lock:
            row = self._db.execute(
                "SELECT value FROM app_settings WHERE key = ?",
                (key,),
            ).fetchone()
        if row is None:
            return default
        try:
            return int(row["value"])
        except (TypeError, ValueError):
            return default

    def set_int(self, key: str, value: int) -> None:
        with self._lock:
            self._db.execute(
                """
                INSERT INTO app_settings (key, value, updated_at)
                VALUES (?, ?, ?)
                ON CONFLICT(key) DO UPDATE SET
                    value = excluded.value,
                    updated_at = excluded.updated_at
                """,
                (key, str(value), datetime.now().strftime("%Y-%m-%d %H:%M:%S")),
            )
            self._db.commit()


class MentionPoller:
    def __init__(self, store: MentionStore, settings: AppSettingsStore) -> None:
        self.store = store
        self.settings = settings
        env_interval = int(
            _env(
                "KORA_SOCIAL_X_MENTION_INTERVAL_SEC",
                str(DEFAULT_MENTION_INTERVAL_SEC),
                legacy="INFLUENCER_X_MENTION_INTERVAL_SEC",
            )
        )
        self.interval_sec = self.settings.get_int("mention_poll_interval_sec", env_interval)
        self.max_results = int(
            _env("KORA_SOCIAL_X_MENTION_MAX_RESULTS", "20", legacy="INFLUENCER_X_MENTION_MAX_RESULTS")
        )
        self.query = _env(
            "KORA_SOCIAL_X_MENTION_QUERY",
            DEFAULT_X_MENTION_QUERY,
            legacy="INFLUENCER_X_MENTION_QUERY",
        ).strip()
        self.mode = _env(
            "KORA_SOCIAL_X_MENTION_MODE",
            "search",
            legacy="INFLUENCER_X_MENTION_MODE",
        ).strip().lower()
        self.timeout_sec = float(
            _env("KORA_SOCIAL_X_MENTION_TIMEOUT_SEC", "30", legacy="INFLUENCER_X_MENTION_TIMEOUT_SEC")
        )
        self._lock = threading.RLock()
        self._running = threading.Event()
        self._thread: threading.Thread | None = None
        self._last_poll_at: str | None = None
        self._last_error: str | None = None
        self._last_inserted = 0

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._running.set()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running.clear()

    def status(self) -> dict[str, Any]:
        return {
            "installed": shutil.which("xurl") is not None,
            "interval_sec": self.interval_sec,
            "enabled": self.interval_sec > 0,
            "mode": self.mode,
            "query": self.query or "mentions",
            "last_poll_at": self._last_poll_at,
            "last_error": self._last_error,
            "last_inserted": self._last_inserted,
        }

    def set_interval(self, interval_sec: int) -> dict[str, Any]:
        with self._lock:
            self.interval_sec = interval_sec
            self.settings.set_int("mention_poll_interval_sec", interval_sec)
            self._last_error = None
        return self.status()

    def poll_once(self) -> dict[str, Any]:
        if shutil.which("xurl") is None:
            with self._lock:
                self._last_error = "xurl not installed"
                self._last_inserted = 0
            return self.status()

        result = self._run_xurl()
        if result.returncode != 0:
            detail = _safe_poll_error(result)
            with self._lock:
                self._last_poll_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                self._last_error = detail or "xurl mention poll failed"
                self._last_inserted = 0
            return self.status()

        mentions = _parse_xurl_mentions(result.stdout, source="xurl")
        inserted = self.store.upsert_many(mentions)
        with self._lock:
            self._last_poll_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            self._last_error = None
            self._last_inserted = inserted
        return self.status()

    def _run_xurl(self) -> subprocess.CompletedProcess[str]:
        limit = str(max(5, min(100, self.max_results)))
        if self.mode == "mentions" or not self.query:
            commands = [["xurl", "mentions", "-n", limit]]
        else:
            commands = [
                ["xurl", "search", self.query, "-n", limit],
                ["xurl", "search", "recent", "--query", self.query, "--max-results", limit],
                ["xurl", "search", "--query", self.query, "--max-results", limit],
            ]

        result: subprocess.CompletedProcess[str] | None = None
        errors: list[str] = []
        for cmd in commands:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=self.timeout_sec,
                check=False,
            )
            if result.returncode == 0:
                return result
            errors.append(f"{' '.join(cmd[:3])}: {_safe_poll_error(result)}")
        assert result is not None
        result.stderr = " / ".join(error for error in errors if error)
        return result

    def _run(self) -> None:
        while self._running.is_set():
            if self.interval_sec > 0:
                try:
                    self.poll_once()
                except Exception as exc:
                    with self._lock:
                        self._last_poll_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                        self._last_error = str(exc)
                        self._last_inserted = 0
                wait_sec = max(1, self.interval_sec)
            else:
                wait_sec = 1
            self._running.wait(wait_sec)


def _capture_from_row(row: sqlite3.Row) -> Capture:
    return Capture(
        id=row["id"],
        created_at=row["created_at"],
        filename=row["filename"],
        caption=row["caption"],
        reason=row["reason"],
        score=float(row["score"]),
        status=row["status"],
        x_post_id=row["x_post_id"],
        x_post_url=row["x_post_url"],
        posted_at=row["posted_at"],
        post_error=row["post_error"],
    )


def _mention_from_row(row: sqlite3.Row) -> Mention:
    return Mention(
        id=row["id"],
        source=row["source"],
        author_handle=row["author_handle"],
        text=row["text"],
        url=row["url"],
        created_at=row["created_at"],
        received_at=row["received_at"],
        command_type=row["command_type"],
        command_payload=row["command_payload"],
        status=row["status"],
        action_status=row["action_status"],
        reply_text=row["reply_text"],
        error=row["error"],
        raw_json=row["raw_json"],
    )


def _people_tag_from_row(row: sqlite3.Row) -> PeopleTag:
    return PeopleTag(
        id=row["id"],
        created_at=row["created_at"],
        handle=row["handle"],
        notes=row["notes"],
        filename=row["filename"],
        enabled=bool(row["enabled"]),
    )


def _clean_handle(handle: str) -> str:
    clean = handle.strip()
    if clean and not clean.startswith("@"):
        clean = f"@{clean}"
    name = clean[1:]
    if not name or len(name) > 15 or not all(char.isalnum() or char == "_" for char in name):
        raise ValueError("Use a valid X handle, like @dimensionalos.")
    return f"@{name}"


def _normalize_tag_image(image: bytes, path: Path) -> None:
    try:
        pil_image = ImageOps.exif_transpose(PILImage.open(BytesIO(image))).convert("RGB")
    except Exception as exc:
        raise ValueError("Upload a valid image file.") from exc
    pil_image.thumbnail((640, 640))
    pil_image.save(path, format="JPEG", quality=88)


def _decode_image_base64(value: str) -> bytes:
    payload = value.strip()
    if "," in payload:
        payload = payload.split(",", 1)[1]
    try:
        return base64.b64decode(payload, validate=True)
    except Exception as exc:
        raise ValueError("Upload a valid image file.") from exc


def _build_mention(
    *,
    mention_id: str,
    source: str,
    author_handle: str,
    text: str,
    url: str | None,
    created_at: str | None,
    raw: Any,
) -> Mention:
    command_type, command_payload = _parse_mention_command(text)
    return Mention(
        id=mention_id,
        source=source,
        author_handle=author_handle,
        text=" ".join(text.strip().split()),
        url=url,
        created_at=created_at,
        received_at=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        command_type=command_type,
        command_payload=command_payload,
        action_status="pending" if command_type in {"vision", "capture"} else "ignored",
        raw_json=json.dumps(raw, ensure_ascii=True) if raw is not None else None,
    )


def _parse_mention_command(text: str) -> tuple[str, str]:
    clean = " ".join(text.strip().split())
    without_handles = re.sub(r"@\w{1,15}", "", clean).strip()
    if re.search(r"\bsay\b[:,]?\s+(.+)$", without_handles, flags=re.IGNORECASE):
        return "ignored", ""
    if re.search(r"\b(what do you see|what can you see|look around|describe)\b", without_handles, re.IGNORECASE):
        return "vision", without_handles or "What do you see?"
    if re.search(
        r"\b(take|snap|capture|shoot|post|send|make)\b.*\b(photo|picture|pic|selfie|snapshot|image)\b"
        r"|\b(photo|picture|pic|selfie|snapshot)\b",
        without_handles,
        re.IGNORECASE,
    ):
        return "capture", without_handles or "Take a picture."
    if re.search(
        r"\b(move|walk|run|drive|turn|spin|jump|dance|sit|stand|follow|go to|navigate)\b",
        without_handles,
        re.IGNORECASE,
    ):
        return "ignored", ""
    classified = _classify_mention_command_with_openrouter(clean)
    if classified is not None:
        return classified
    if without_handles:
        return "unknown", without_handles
    return "ignored", ""


def _classify_mention_command_with_openrouter(text: str) -> tuple[str, str] | None:
    try:
        classifier = globals().get("vision")
        if not isinstance(classifier, OpenRouterVision) or not classifier.enabled():
            return None
        result = classifier.classify_mention_command(text, bot_handle=DEFAULT_X_HANDLE)
    except Exception as exc:
        logger.warning("OpenRouter mention classification failed: %s", exc)
        return None
    command_type = str(result.get("command_type", "ignored")).strip().lower()
    if command_type not in {"vision", "capture", "ignored"}:
        return None
    command_payload = " ".join(str(result.get("command_payload", "")).strip().split())[:240]
    if command_type in {"vision", "capture"} and not command_payload:
        command_payload = text
    return command_type, command_payload


def _clean_spoken_text(text: str) -> str:
    clean = re.sub(r"https?://\S+", "", text)
    clean = re.sub(r"@\w{1,15}", "", clean)
    clean = " ".join(clean.strip().split())
    return clean[:240]


def _parse_xurl_mentions(output: str, *, source: str) -> list[Mention]:
    parsed = _json_loads_maybe(output)
    if parsed is not None:
        return _mentions_from_json(parsed, source=source)
    return _mentions_from_text(output, source=source)


def _mentions_from_json(parsed: Any, *, source: str) -> list[Mention]:
    users: dict[str, str] = {}
    if isinstance(parsed, dict):
        includes = parsed.get("includes")
        if isinstance(includes, dict) and isinstance(includes.get("users"), list):
            for user in includes["users"]:
                if isinstance(user, dict) and user.get("id") and user.get("username"):
                    users[str(user["id"])] = _clean_handle(str(user["username"]))
    candidates: list[Any]
    if isinstance(parsed, dict) and isinstance(parsed.get("data"), list):
        candidates = parsed["data"]
    elif isinstance(parsed, list):
        candidates = parsed
    else:
        candidates = [item for item in _walk_json(parsed) if isinstance(item, dict) and item.get("text")]

    mentions: list[Mention] = []
    for item in candidates:
        if not isinstance(item, dict):
            continue
        text = str(item.get("text") or item.get("full_text") or "").strip()
        mention_id = str(item.get("id") or item.get("tweet_id") or item.get("post_id") or "").strip()
        if not text or not mention_id:
            continue
        author = (
            item.get("author_handle")
            or item.get("username")
            or item.get("screen_name")
            or users.get(str(item.get("author_id")))
            or "@unknown"
        )
        url = item.get("url")
        if not url and mention_id and str(author).startswith("@"):
            url = f"https://x.com/{str(author).removeprefix('@')}/status/{mention_id}"
        mentions.append(
            _build_mention(
                mention_id=mention_id,
                source=source,
                author_handle=_clean_handle(str(author)),
                text=text,
                url=str(url) if url else None,
                created_at=str(item.get("created_at")) if item.get("created_at") else None,
                raw=item,
            )
        )
    return mentions


def _mentions_from_text(output: str, *, source: str) -> list[Mention]:
    mentions: list[Mention] = []
    for block in re.split(r"\n\s*\n", output.strip()):
        if not block.strip():
            continue
        id_match = re.search(r"\b(?:id|post|tweet)[:# ]+(\d{6,})\b", block, re.IGNORECASE)
        url_match = re.search(r"https://(?:x|twitter)\.com/[A-Za-z0-9_]+/status/(\d+)", block)
        mention_id = url_match.group(1) if url_match else id_match.group(1) if id_match else ""
        if not mention_id:
            continue
        author_match = re.search(r"@([A-Za-z0-9_]{1,15})", block)
        text_lines = [
            line.strip()
            for line in block.splitlines()
            if line.strip()
            and "http" not in line
            and not re.search(r"\b(?:id|post|tweet)[:# ]+\d{6,}\b", line, re.IGNORECASE)
        ]
        text = " ".join(text_lines)
        if not text:
            continue
        mentions.append(
            _build_mention(
                mention_id=mention_id,
                source=source,
                author_handle=f"@{author_match.group(1)}" if author_match else "@unknown",
                text=text,
                url=url_match.group(0).replace("https://twitter.com/", "https://x.com/")
                if url_match
                else None,
                created_at=None,
                raw={"text": block},
            )
        )
    return mentions


def _json_loads_maybe(value: str) -> Any | None:
    value = value.strip()
    if not value:
        return None
    try:
        return json.loads(value)
    except json.JSONDecodeError:
        return None


def _safe_poll_error(result: subprocess.CompletedProcess[str]) -> str:
    detail = " ".join((result.stderr or result.stdout).strip().split())
    return detail[:500]


def _walk_json(value: Any) -> list[Any]:
    items = [value]
    if isinstance(value, dict):
        for child in value.values():
            items.extend(_walk_json(child))
    elif isinstance(value, list):
        for child in value:
            items.extend(_walk_json(child))
    return items


vision = OpenRouterVision()
camera = Go2CameraService(ROBOT_IP, mock_mode=MOCK_ROBOT, mock_frame_dir=MOCK_FRAME_DIR)
people_tags = PeopleTagStore(PEOPLE_TAG_DIR, DB_PATH)
store = DraftStore(CAPTURE_DIR, DB_PATH, vision=vision, people_tags=people_tags)
robot = RobotControlService(camera, store, mock_mode=MOCK_ROBOT)
map_hud = MapHudService(MAP_VIS_URL)
x_poster = XPoster()
mentions = MentionStore(DB_PATH)
app_settings = AppSettingsStore(DB_PATH)
mention_poller = MentionPoller(mentions, app_settings)
app = FastAPI(title="Kora Social")
app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")
app.mount("/captures", StaticFiles(directory=CAPTURE_DIR), name="captures")
app.mount("/people-tags", StaticFiles(directory=PEOPLE_TAG_DIR), name="people-tags")


def _resolved_map_status() -> dict[str, Any]:
    map_status = map_hud.status()
    camera_map = camera.map_status()
    if not map_status["connected"]:
        return camera_map
    if map_status.get("pointcloud") is None and camera_map.get("pointcloud") is not None:
        map_status = {**map_status, "pointcloud": camera_map["pointcloud"]}
        if not map_status.get("objects"):
            map_status["objects"] = camera_map.get("objects", [])
    return map_status


@app.on_event("startup")
def app_startup() -> None:
    map_hud.start()
    mention_poller.start()


@app.on_event("shutdown")
def app_shutdown() -> None:
    map_hud.stop()
    mention_poller.stop()


@app.get("/")
def index() -> FileResponse:
    return FileResponse(STATIC_DIR / "index.html")


@app.get("/api/status")
def api_status() -> dict[str, Any]:
    return {
        "camera": camera.status(),
        "robot": robot.status(),
        "map": _resolved_map_status(),
        "x": x_poster.status().to_dict(),
        "vision": vision.status(),
        "mention_poller": mention_poller.status(),
        "mentions": mentions.list(),
        "people_tags": people_tags.list(),
        "drafts": store.list(),
    }


@app.get("/api/map")
def api_map() -> dict[str, Any]:
    return _resolved_map_status()


@app.post("/api/mentions/settings")
def api_mentions_settings(request: MentionSettingsRequest) -> dict[str, Any]:
    return mention_poller.set_interval(request.interval_sec)


@app.post("/api/mentions/poll")
def api_mentions_poll() -> dict[str, Any]:
    return mention_poller.poll_once()


@app.post("/api/mentions/inject")
def api_mentions_inject(request: MentionInjectRequest) -> dict[str, Any]:
    try:
        return asdict(mentions.inject(request.author_handle, request.text))
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@app.post("/api/mentions/{mention_id}/run")
def api_run_mention(mention_id: str) -> dict[str, Any]:
    try:
        mention = mentions.get(mention_id)
    except KeyError as exc:
        raise HTTPException(status_code=404, detail="Mention not found") from exc

    try:
        if mention.command_type == "vision":
            jpeg = camera.capture()
            answer = vision.describe(jpeg, mention.command_payload or "What do you see?")
            updated = mentions.update_action(
                mention_id,
                action_status="done",
                reply_text=answer,
            )
        elif mention.command_type == "capture":
            jpeg = camera.capture()
            capture = store.create(jpeg)
            detail = "saved draft"
            if capture.status == "failed" and capture.post_error:
                detail = f"saved image, draft needs edit: {capture.post_error}"
            updated = mentions.update_action(
                mention_id,
                action_status="done" if capture.status != "failed" else "failed",
                reply_text=f"{detail} ({capture.id})",
                error=capture.post_error,
            )
        else:
            updated = mentions.update_action(
                mention_id,
                action_status="ignored",
                status="ignored",
                error=f"No runnable action for command type {mention.command_type!r}.",
            )
    except RuntimeError as exc:
        updated = mentions.update_action(
            mention_id,
            action_status="failed",
            error=str(exc),
        )
    except Exception as exc:
        updated = mentions.update_action(
            mention_id,
            action_status="failed",
            error=str(exc),
        )
    return asdict(updated)


@app.post("/api/mentions/{mention_id}/ignore")
def api_ignore_mention(mention_id: str) -> dict[str, Any]:
    try:
        return asdict(
            mentions.update_action(
                mention_id,
                action_status="ignored",
                status="ignored",
            )
        )
    except KeyError as exc:
        raise HTTPException(status_code=404, detail="Mention not found") from exc


@app.delete("/api/mentions/{mention_id}")
def api_delete_mention(mention_id: str) -> dict[str, str]:
    try:
        mentions.delete(mention_id)
    except KeyError as exc:
        raise HTTPException(status_code=404, detail="Mention not found") from exc
    return {"status": "deleted"}


@app.post("/api/mode")
def api_set_mode(request: ModeRequest) -> dict[str, Any]:
    if request.mock:
        try:
            robot.cancel_patrol()
            if camera.connection() is not None:
                robot.stop_motion()
        except Exception:
            pass
    camera_status = camera.set_mock_mode(request.mock)
    robot_status = robot.set_mock_mode(request.mock)
    return {"camera": camera_status, "robot": robot_status}


@app.post("/api/x/mode")
def api_set_x_mode(request: XModeRequest) -> dict[str, Any]:
    status = x_poster.set_dry_run(request.dry_run)
    if not request.dry_run and not status.installed:
        x_poster.set_dry_run(True)
        raise HTTPException(status_code=409, detail="xurl is not installed.")
    if not request.dry_run and not status.authenticated:
        detail = f" {status.detail}" if status.detail else ""
        x_poster.set_dry_run(True)
        raise HTTPException(status_code=409, detail=f"xurl is not authenticated.{detail}")
    return status.to_dict()


@app.post("/api/camera/start")
def api_camera_start() -> dict[str, Any]:
    return camera.start()


@app.post("/api/camera/stop")
def api_camera_stop() -> dict[str, Any]:
    try:
        robot.cancel_patrol()
    except Exception:
        pass
    return camera.stop()


@app.post("/api/robot/stand")
def api_robot_stand() -> dict[str, Any]:
    try:
        return robot.stand()
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post("/api/robot/lie-down")
def api_robot_lie_down() -> dict[str, Any]:
    try:
        return robot.lie_down()
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post("/api/robot/stop")
def api_robot_stop() -> dict[str, Any]:
    try:
        robot.cancel_patrol()
        return robot.stop_motion()
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post("/api/robot/hello")
def api_robot_hello() -> dict[str, Any]:
    try:
        return robot.hello()
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post("/api/robot/sport-command/{command_name}")
def api_robot_sport_command(command_name: str) -> dict[str, Any]:
    try:
        return robot.sport_command(command_name)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post("/api/robot/move")
def api_robot_move(request: MoveRequest) -> dict[str, Any]:
    try:
        return robot.move(request)
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post("/api/robot/drive")
def api_robot_drive(request: MoveRequest) -> dict[str, Any]:
    try:
        return robot.drive(request)
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post("/api/robot/sport-move")
def api_robot_sport_move(request: SportMoveRequest) -> dict[str, Any]:
    try:
        return robot.sport_move(request)
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post("/api/robot/patrol/start")
def api_robot_patrol_start() -> dict[str, Any]:
    try:
        return robot.start_tiny_patrol()
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc


@app.post("/api/robot/patrol/cancel")
def api_robot_patrol_cancel() -> dict[str, Any]:
    return robot.cancel_patrol()


@app.get("/api/frame.jpg")
def api_frame() -> Response:
    jpeg = camera.latest_jpeg()
    if jpeg is None:
        raise HTTPException(status_code=404, detail="No frame available")
    return Response(
        content=jpeg,
        media_type="image/jpeg",
        headers={"Cache-Control": "no-store, max-age=0"},
    )


@app.post("/api/capture")
def api_capture() -> dict[str, Any]:
    try:
        capture = store.create(camera.capture())
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    return asdict(capture)


@app.post("/api/people-tags")
def api_create_people_tag(request: PeopleTagCreateRequest) -> dict[str, Any]:
    try:
        image_bytes = _decode_image_base64(request.image_base64)
        return asdict(people_tags.create(request.handle, request.notes, image_bytes))
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@app.post("/api/people-tags/{tag_id}/enable")
def api_enable_people_tag(tag_id: str) -> dict[str, Any]:
    try:
        return asdict(people_tags.set_enabled(tag_id, True))
    except KeyError as exc:
        raise HTTPException(status_code=404, detail="Tag not found") from exc


@app.post("/api/people-tags/{tag_id}/disable")
def api_disable_people_tag(tag_id: str) -> dict[str, Any]:
    try:
        return asdict(people_tags.set_enabled(tag_id, False))
    except KeyError as exc:
        raise HTTPException(status_code=404, detail="Tag not found") from exc


@app.delete("/api/people-tags/{tag_id}")
def api_delete_people_tag(tag_id: str) -> dict[str, str]:
    try:
        people_tags.delete(tag_id)
    except KeyError as exc:
        raise HTTPException(status_code=404, detail="Tag not found") from exc
    return {"status": "deleted"}


@app.post("/api/vision/describe")
def api_describe(request: DescribeRequest) -> dict[str, str]:
    try:
        jpeg = camera.capture()
        answer = vision.describe(jpeg, request.question)
    except RuntimeError as exc:
        raise HTTPException(status_code=409, detail=str(exc)) from exc
    except Exception as exc:
        raise HTTPException(status_code=502, detail=f"OpenRouter describe failed: {exc}") from exc
    return {"answer": answer}


@app.post("/api/drafts/{capture_id}/caption")
def api_update_caption(capture_id: str, request: CaptionUpdateRequest) -> dict[str, Any]:
    try:
        return asdict(store.update_caption(capture_id, request.caption))
    except KeyError as exc:
        raise HTTPException(status_code=404, detail="Draft not found") from exc
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


@app.post("/api/drafts/{capture_id}/post")
def api_post_draft(capture_id: str) -> dict[str, Any]:
    try:
        return asdict(store.post(capture_id, x_poster))
    except KeyError as exc:
        raise HTTPException(status_code=404, detail="Draft not found") from exc


@app.delete("/api/drafts/{capture_id}")
def api_delete_draft(capture_id: str) -> dict[str, str]:
    try:
        store.delete(capture_id)
    except KeyError as exc:
        raise HTTPException(status_code=404, detail="Draft not found") from exc
    return {"status": "deleted"}


@app.post("/api/drafts/{capture_id}/{status}")
def api_update_status(capture_id: str, status: str) -> dict[str, Any]:
    try:
        return asdict(store.update_status(capture_id, status))
    except KeyError as exc:
        raise HTTPException(status_code=404, detail="Draft not found") from exc
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
