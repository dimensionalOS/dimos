#!/usr/bin/env python3
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

"""Low-level RoboMaster TT (Tello Talent) SDK adapter."""

from __future__ import annotations

import math
import socket
import threading
import time
from typing import Any, TypeAlias

import cv2
from reactivex import Subject

from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Twist, Vector3
from dimos.msgs.sensor_msgs import Image, ImageFormat
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

TelloScalar: TypeAlias = int | float | str
TelloState: TypeAlias = dict[str, TelloScalar]


def _to_float(value: TelloScalar | None, default: float = 0.0) -> float:
    if value is None:
        return default
    if isinstance(value, (float, int)):
        return float(value)
    try:
        return float(value)
    except ValueError:
        return default


def _clamp(value: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, value))


class TelloSdkClient:
    """SDK client for RoboMaster TT / Tello over UDP."""

    def __init__(
        self,
        tello_ip: str = "192.168.10.1",
        command_port: int = 8889,
        state_port: int = 8890,
        video_port: int = 11111,
        local_ip: str = "",
        local_command_port: int = 9000,
        command_timeout: float = 7.0,
        video_uri: str | None = None,
        velocity_scale: float = 100.0,
        yaw_scale: float = 60.0,
    ) -> None:
        """Initialize the client.

        Args:
            tello_ip: Drone IP, default AP mode address.
            command_port: SDK command UDP port.
            state_port: SDK state UDP port.
            video_port: H264 UDP stream port.
            local_ip: Local bind host. Empty string binds all interfaces.
            local_command_port: Local UDP port used for command/response.
            command_timeout: Timeout waiting for command responses.
            video_uri: Optional explicit URI for OpenCV video capture.
            velocity_scale: Meters/second -> rc units conversion factor.
            yaw_scale: Radians/second -> rc units conversion factor.
        """
        self.tello_ip = tello_ip
        self.command_port = command_port
        self.state_port = state_port
        self.video_port = video_port
        self.local_ip = local_ip
        self.local_command_port = local_command_port
        self.command_timeout = command_timeout
        self.video_uri = video_uri
        self.velocity_scale = velocity_scale
        self.yaw_scale = yaw_scale

        self.connected = False
        self._running = False

        self._tello_addr = (self.tello_ip, self.command_port)
        self._command_socket: socket.socket | None = None
        self._state_socket: socket.socket | None = None
        self._video_capture: cv2.VideoCapture | None = None

        self._command_lock = threading.Lock()
        self._state_lock = threading.RLock()
        self._frame_lock = threading.RLock()
        self._pose_lock = threading.RLock()

        self._state_thread: threading.Thread | None = None
        self._video_thread: threading.Thread | None = None

        self._latest_state: TelloState = {}
        self._latest_frame: Image | None = None

        self._position = Vector3(0.0, 0.0, 0.0)
        self._last_pose_ts = time.time()

        self._telemetry_subject: Subject[TelloState] = Subject()
        self._status_subject: Subject[dict[str, Any]] = Subject()
        self._odom_subject: Subject[PoseStamped] = Subject()
        self._video_subject: Subject[Image] = Subject()

    @staticmethod
    def parse_state_packet(packet: str) -> TelloState:
        """Parse semicolon-separated SDK state packet to a dict."""
        result: TelloState = {}
        for item in packet.strip().split(";"):
            if not item or ":" not in item:
                continue
            key, raw_value = item.split(":", 1)
            value = raw_value.strip()
            if value == "":
                continue
            try:
                if "." in value:
                    result[key] = float(value)
                else:
                    result[key] = int(value)
            except ValueError:
                result[key] = value
        return result

    def connect(self) -> bool:
        """Connect sockets, enter SDK mode, start state/video workers."""
        if self.connected:
            return True

        try:
            self._command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._command_socket.bind((self.local_ip, self.local_command_port))
            self._command_socket.settimeout(self.command_timeout)

            self._state_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._state_socket.bind((self.local_ip, self.state_port))
            self._state_socket.settimeout(0.5)
        except OSError as exc:
            logger.error(f"Failed to bind Tello sockets: {exc}")
            self.disconnect()
            return False

        self._running = True
        self._state_thread = threading.Thread(target=self._state_loop, daemon=True)
        self._state_thread.start()

        if not self.enter_sdk_mode():
            logger.error("Tello did not enter SDK command mode")
            self.disconnect()
            return False

        if not self.send_command("streamon", retries=1).startswith("ok"):
            logger.warning("Tello streamon did not return ok; continuing without video")
        else:
            self._video_thread = threading.Thread(target=self._video_loop, daemon=True)
            self._video_thread.start()

        self.connected = True
        logger.info(f"Tello SDK connected to {self.tello_ip}:{self.command_port}")
        return True

    def disconnect(self) -> None:
        """Stop workers and close sockets."""
        if self.connected:
            try:
                self.send_command_no_wait("rc 0 0 0 0")
            except Exception:
                pass
            try:
                self.send_command("streamoff", retries=0)
            except Exception:
                pass

        self._running = False

        if self._state_thread and self._state_thread.is_alive():
            self._state_thread.join(timeout=1.0)
        if self._video_thread and self._video_thread.is_alive():
            self._video_thread.join(timeout=1.0)

        if self._video_capture is not None:
            self._video_capture.release()
            self._video_capture = None

        if self._state_socket is not None:
            self._state_socket.close()
            self._state_socket = None
        if self._command_socket is not None:
            self._command_socket.close()
            self._command_socket = None

        self.connected = False
        logger.info("Tello SDK disconnected")

    def send_command(self, command: str, retries: int = 2) -> str:
        """Send a raw command and wait for a response."""
        if self._command_socket is None:
            return "error:not_connected"

        attempt = 0
        while attempt <= retries:
            try:
                with self._command_lock:
                    self._command_socket.sendto(command.encode("utf-8"), self._tello_addr)
                    data, _ = self._command_socket.recvfrom(2048)
                response = data.decode("utf-8", errors="ignore").strip().lower()
                return response
            except TimeoutError:
                attempt += 1
                logger.warning(f"Tello command timeout [{command}] attempt {attempt}/{retries + 1}")
            except OSError as exc:
                logger.error(f"Tello command socket error [{command}]: {exc}")
                break
        return "error:timeout"

    def send_command_no_wait(self, command: str) -> bool:
        """Send a raw command without waiting for an SDK response.

        Useful for high-rate commands like `rc` where waiting for a reply
        can add control lag and frequent timeout noise.
        """
        if self._command_socket is None:
            return False
        try:
            with self._command_lock:
                self._command_socket.sendto(command.encode("utf-8"), self._tello_addr)
            return True
        except OSError as exc:
            logger.warning(f"Tello non-blocking send failed [{command}]: {exc}")
            return False

    def enter_sdk_mode(self) -> bool:
        """Enter SDK command mode."""
        return self.send_command("command").startswith("ok")

    def takeoff(self) -> bool:
        """Takeoff command."""
        response = self.send_command("takeoff")
        if response.startswith("ok"):
            return True

        # Some firmware/network conditions drop command ACKs.
        # Treat timeout as success if telemetry shows we actually climbed.
        if response == "error:timeout":
            deadline = time.time() + 4.0
            while time.time() < deadline:
                state = self.get_state()
                if _to_float(state.get("h")) >= 20.0:  # centimeters
                    logger.info("Takeoff timeout recovered by telemetry height check")
                    return True
                time.sleep(0.1)

        return False

    def land(self) -> bool:
        """Land command."""
        return self.send_command("land").startswith("ok")

    def emergency(self) -> bool:
        """Emergency stop command."""
        return self.send_command("emergency", retries=0).startswith("ok")

    def stop(self) -> bool:
        """Stop all motion."""
        return self.send_command("stop", retries=0).startswith("ok")

    def yaw(self, degrees_ccw: float) -> bool:
        """Rotate in place. Positive values rotate counter-clockwise."""
        degrees = round(abs(degrees_ccw))
        if degrees == 0:
            return True
        if degrees_ccw > 0:
            return self.send_command(f"ccw {degrees}").startswith("ok")
        return self.send_command(f"cw {degrees}").startswith("ok")

    def move_relative(self, x: float, y: float, z: float, speed: float = 0.4) -> bool:
        """Move relative in meters (x forward, y left, z up)."""
        x_cm = _clamp(round(x * 100.0), -500, 500)
        y_right_cm = _clamp(round(-y * 100.0), -500, 500)
        z_cm = _clamp(round(z * 100.0), -500, 500)
        speed_cm = _clamp(round(speed * 100.0), 10, 100)
        return self.send_command(f"go {x_cm} {y_right_cm} {z_cm} {speed_cm}").startswith("ok")

    def send_ext(self, ext_command: str) -> bool:
        """Send TT expansion-board command through EXT path."""
        payload = ext_command.strip()
        if not payload:
            return False
        if not payload.lower().startswith("ext "):
            payload = f"EXT {payload}"
        return self.send_command(payload).startswith("ok")

    def rc(self, left_right: int, forward_back: int, up_down: int, yaw: int) -> bool:
        """Send low-level continuous control command in SDK rc units."""
        lr = _clamp(left_right, -100, 100)
        fb = _clamp(forward_back, -100, 100)
        ud = _clamp(up_down, -100, 100)
        yw = _clamp(yaw, -100, 100)
        return self.send_command_no_wait(f"rc {lr} {fb} {ud} {yw}")

    def move(self, linear: Vector3, yaw_rate: float = 0.0, duration: float = 0.0) -> bool:
        """Move from SI units, converted to rc command."""
        lr = round(-linear.y * self.velocity_scale)
        fb = round(linear.x * self.velocity_scale)
        ud = round(linear.z * self.velocity_scale)
        yw = round(-yaw_rate * self.yaw_scale)
        ok = self.rc(lr, fb, ud, yw)
        if duration > 0:
            time.sleep(duration)
            self.rc(0, 0, 0, 0)
        return ok

    def move_twist(self, twist: Twist, duration: float = 0.0, lock_altitude: bool = True) -> bool:
        """Move from Twist command."""
        z = 0.0 if lock_altitude else twist.linear.z
        return self.move(
            Vector3(twist.linear.x, twist.linear.y, z),
            yaw_rate=twist.angular.z,
            duration=duration,
        )

    def get_state(self) -> TelloState:
        """Get the latest parsed state packet."""
        with self._state_lock:
            return dict(self._latest_state)

    def get_video_frame(self) -> Image | None:
        """Get latest decoded video frame."""
        with self._frame_lock:
            return self._latest_frame

    def telemetry_stream(self) -> Subject[TelloState]:
        """Stream of parsed state packets."""
        return self._telemetry_subject

    def status_stream(self) -> Subject[dict[str, Any]]:
        """Stream of compact status dictionaries."""
        return self._status_subject

    def odom_stream(self) -> Subject[PoseStamped]:
        """Stream of odometry estimates from state packets."""
        return self._odom_subject

    def video_stream(self) -> Subject[Image]:
        """Stream of decoded camera frames."""
        return self._video_subject

    def _state_loop(self) -> None:
        while self._running:
            if self._state_socket is None:
                break
            try:
                payload, _ = self._state_socket.recvfrom(2048)
            except TimeoutError:
                continue
            except OSError:
                break

            packet = payload.decode("utf-8", errors="ignore").strip()
            if not packet:
                continue

            state = self.parse_state_packet(packet)
            if not state:
                continue

            ts = time.time()
            with self._state_lock:
                self._latest_state = state

            self._telemetry_subject.on_next(dict(state))
            self._status_subject.on_next(self._build_status(state, ts))
            self._publish_odom_from_state(state, ts)

    def _video_loop(self) -> None:
        bind_host = self.local_ip if self.local_ip else "0.0.0.0"
        uri = self.video_uri or (
            f"udp://@{bind_host}:{self.video_port}?overrun_nonfatal=1&fifo_size=50000000"
        )

        capture = cv2.VideoCapture(uri, cv2.CAP_FFMPEG)
        if not capture.isOpened():
            logger.warning(f"Failed to open Tello video stream URI: {uri}")
            return
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self._video_capture = capture
        logger.info(f"Tello video stream started from {uri}")

        frame_count = 0
        no_frame_count = 0
        while self._running:
            ok, frame = capture.read()
            if not ok:
                no_frame_count += 1
                time.sleep(0.02)
                if no_frame_count > 150:
                    logger.warning("Tello video stalled; reopening capture")
                    capture.release()
                    capture = cv2.VideoCapture(uri, cv2.CAP_FFMPEG)
                    if capture.isOpened():
                        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                        no_frame_count = 0
                continue

            no_frame_count = 0
            frame_count += 1
            if frame_count == 1:
                logger.info(f"First Tello frame received: shape={frame.shape}")

            image = Image.from_opencv(frame, format=ImageFormat.BGR, frame_id="camera_link")
            with self._frame_lock:
                self._latest_frame = image
            self._video_subject.on_next(image)

        capture.release()
        self._video_capture = None
        logger.info("Tello video stream stopped")

    def _build_status(self, state: TelloState, ts: float) -> dict[str, Any]:
        return {
            "connected": self.connected,
            "battery": int(_to_float(state.get("bat"), -1.0)),
            "height_m": _to_float(state.get("h")) / 100.0,
            "tof_m": _to_float(state.get("tof")) / 100.0,
            "yaw_deg": _to_float(state.get("yaw")),
            "pitch_deg": _to_float(state.get("pitch")),
            "roll_deg": _to_float(state.get("roll")),
            "temp_low_c": _to_float(state.get("templ")),
            "temp_high_c": _to_float(state.get("temph")),
            "flight_time_s": _to_float(state.get("time")),
            "ts": ts,
        }

    def _publish_odom_from_state(self, state: TelloState, ts: float) -> None:
        roll_rad = math.radians(_to_float(state.get("roll")))
        pitch_rad = math.radians(_to_float(state.get("pitch")))
        yaw_rad = math.radians(_to_float(state.get("yaw")))
        quaternion = Quaternion.from_euler(Vector3(roll_rad, pitch_rad, yaw_rad))

        vgx = _to_float(state.get("vgx")) / 100.0
        vgy_right = _to_float(state.get("vgy")) / 100.0
        v_left = -vgy_right

        with self._pose_lock:
            dt = max(0.0, min(ts - self._last_pose_ts, 0.2))
            self._last_pose_ts = ts

            vx_world = (vgx * math.cos(yaw_rad)) - (v_left * math.sin(yaw_rad))
            vy_world = (vgx * math.sin(yaw_rad)) + (v_left * math.cos(yaw_rad))

            self._position.x += vx_world * dt
            self._position.y += vy_world * dt
            self._position.z = _to_float(state.get("h")) / 100.0

            pose = PoseStamped(
                position=Vector3(self._position),
                orientation=quaternion,
                frame_id="world",
                ts=ts,
            )

        self._odom_subject.on_next(pose)


__all__ = ["TelloSdkClient", "TelloState"]
