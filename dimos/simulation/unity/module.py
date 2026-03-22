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

"""UnityBridgeModule: TCP bridge to the VLA Challenge Unity simulator.

Implements the ROS-TCP-Endpoint binary protocol to communicate with Unity
directly — no ROS dependency needed, no Unity-side changes.

Unity sends simulated sensor data (lidar PointCloud2, compressed camera images).
We send back vehicle PoseStamped updates so Unity renders the robot position.

Protocol (per message on the TCP stream):
  [4 bytes LE uint32] destination string length
  [N bytes]           destination string (topic name or __syscommand)
  [4 bytes LE uint32] message payload length
  [M bytes]           payload (ROS1-serialized message, or JSON for syscommands)
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import platform
from queue import Empty, Queue
import signal
import socket
import struct
import subprocess
import threading
import time
from typing import Any

import cv2
import numpy as np
from pydantic import Field
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger
from dimos.utils.ros1 import (
    deserialize_compressed_image,
    deserialize_pointcloud2,
    serialize_pose_stamped,
)

logger = setup_logger()
PI = math.pi

# LFS data asset name for the Unity sim binary
_LFS_ASSET = "unity_sim_x86"

# Read timeout for the Unity TCP connection (seconds).  If Unity stops
# sending data for longer than this the bridge treats it as a hung
# connection and drops it.
_BRIDGE_READ_TIMEOUT = 30.0


# TCP protocol helpers


def _recvall(sock: socket.socket, size: int) -> bytes:
    buf = bytearray(size)
    view = memoryview(buf)
    pos = 0
    while pos < size:
        n = sock.recv_into(view[pos:], size - pos)
        if not n:
            raise OSError("Connection closed")
        pos += n
    return bytes(buf)


def _read_tcp_message(sock: socket.socket) -> tuple[str, bytes]:
    dest_len = struct.unpack("<I", _recvall(sock, 4))[0]
    dest = _recvall(sock, dest_len).decode("utf-8").rstrip("\x00")
    msg_len = struct.unpack("<I", _recvall(sock, 4))[0]
    msg_data = _recvall(sock, msg_len) if msg_len > 0 else b""
    return dest, msg_data


def _write_tcp_message(sock: socket.socket, destination: str, data: bytes) -> None:
    dest_bytes = destination.encode("utf-8")
    sock.sendall(
        struct.pack("<I", len(dest_bytes)) + dest_bytes + struct.pack("<I", len(data)) + data
    )


def _write_tcp_command(sock: socket.socket, command: str, params: dict[str, Any]) -> None:
    dest_bytes = command.encode("utf-8")
    json_bytes = json.dumps(params).encode("utf-8")
    sock.sendall(
        struct.pack("<I", len(dest_bytes))
        + dest_bytes
        + struct.pack("<I", len(json_bytes))
        + json_bytes
    )


# Platform validation


def _validate_platform() -> None:
    """Raise if the current platform can't run the Unity x86_64 binary."""
    supported_systems = {"Linux"}
    supported_archs = {"x86_64", "AMD64"}

    system = platform.system()
    arch = platform.machine()

    if system not in supported_systems:
        raise RuntimeError(
            f"Unity simulator requires Linux x86_64 but running on {system} {arch}. "
            f"macOS and Windows are not supported (the binary is a Linux ELF executable). "
            f"Use a Linux VM, Docker, or WSL2."
        )

    if arch not in supported_archs:
        raise RuntimeError(
            f"Unity simulator requires x86_64 but running on {arch}. "
            f"ARM64 Linux is not supported. Use an x86_64 machine or emulation layer."
        )


# Config


class UnityBridgeConfig(ModuleConfig):
    """Configuration for the Unity bridge / vehicle simulator.

    Set ``unity_binary=""`` to auto-resolve from LFS data (default).
    Set to an explicit path to use a custom binary. The LFS asset
    ``unity_sim_x86`` is pulled automatically via ``get_data()``.
    """

    # Path to the Unity x86_64 binary. Leave empty to auto-resolve
    # from LFS data (unity_sim_x86/environment/Model.x86_64).
    unity_binary: str = ""

    # Max seconds to wait for Unity to connect after launch.
    unity_connect_timeout: float = 30.0

    # TCP server settings (we listen; Unity connects to us).
    # Default to loopback — set to "0.0.0.0" explicitly if Unity runs
    # on a different machine.
    unity_host: str = "127.0.0.1"
    unity_port: int = 10000

    # Run Unity with no visible window (set -batchmode -nographics).
    # Note: headless mode may not produce camera images.
    headless: bool = False

    # Extra CLI args to pass to the Unity binary.
    unity_extra_args: list[str] = Field(default_factory=list)

    # Vehicle parameters
    vehicle_height: float = 0.75

    # Initial vehicle pose
    init_x: float = 0.0
    init_y: float = 0.0
    init_z: float = 0.0
    init_yaw: float = 0.0

    # Kinematic sim rate (Hz) for odometry integration
    sim_rate: float = 200.0


# Camera intrinsics constants.
#
# The Unity camera produces a 360° cylindrical panorama (1920×640).
# A true pinhole model cannot represent this, so we approximate with
# a 120° horizontal FOV window.  Both CameraInfo and the Rerun static
# pinhole use the SAME focal length so downstream consumers see
# consistent intrinsics.
_CAM_WIDTH = 1920
_CAM_HEIGHT = 640
_CAM_HFOV_RAD = math.radians(120.0)
_CAM_FX = (_CAM_WIDTH / 2.0) / math.tan(_CAM_HFOV_RAD / 2.0)
_CAM_FY = _CAM_FX
_CAM_CX = _CAM_WIDTH / 2.0
_CAM_CY = _CAM_HEIGHT / 2.0


# Module


class UnityBridgeModule(Module[UnityBridgeConfig]):
    """TCP bridge to the Unity simulator with kinematic odometry integration.

    Ports:
        cmd_vel (In[Twist]): Velocity commands.
        terrain_map (In[PointCloud2]): Terrain for Z adjustment.
        odometry (Out[Odometry]): Vehicle state at sim_rate.
        registered_scan (Out[PointCloud2]): Lidar from Unity.
        color_image (Out[Image]): RGB camera from Unity (1920x640 panoramic).
        semantic_image (Out[Image]): Semantic segmentation from Unity.
        camera_info (Out[CameraInfo]): Camera intrinsics.
    """

    default_config = UnityBridgeConfig

    cmd_vel: In[Twist]
    terrain_map: In[PointCloud2]
    odometry: Out[Odometry]
    registered_scan: Out[PointCloud2]
    color_image: Out[Image]
    semantic_image: Out[Image]
    camera_info: Out[CameraInfo]

    @staticmethod
    def rerun_static_pinhole(rr: Any) -> list[Any]:
        """Static Pinhole + Transform3D for the Unity panoramic camera."""
        return [
            rr.Pinhole(
                resolution=[_CAM_WIDTH, _CAM_HEIGHT],
                focal_length=[_CAM_FX, _CAM_FY],
                principal_point=[_CAM_CX, _CAM_CY],
                camera_xyz=rr.ViewCoordinates.RDF,
            ),
            rr.Transform3D(
                parent_frame="tf#/sensor",
                translation=[0.0, 0.0, 0.1],
                rotation=rr.Quaternion(xyzw=[0.5, -0.5, 0.5, -0.5]),
            ),
        ]

    @staticmethod
    def rerun_suppress_camera_info(_: Any) -> None:
        """Suppress CameraInfo logging — the static pinhole handles 3D projection."""
        return None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._x = self.config.init_x
        self._y = self.config.init_y
        self._z = self.config.init_z + self.config.vehicle_height
        self._roll = 0.0
        self._pitch = 0.0
        self._yaw = self.config.init_yaw
        self._terrain_z = self.config.init_z
        self._fwd_speed = 0.0
        self._left_speed = 0.0
        self._yaw_rate = 0.0
        self._cmd_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._running = threading.Event()
        self._sim_thread: threading.Thread | None = None
        self._unity_thread: threading.Thread | None = None
        self._unity_connected = False
        self._unity_ready = threading.Event()
        self._unity_process: subprocess.Popen | None = None  # type: ignore[type-arg]
        self._send_queue: Queue[tuple[str, bytes]] = Queue()
        self._binary_path = self._resolve_binary()

    @rpc
    def start(self) -> None:
        super().start()
        self._disposables.add(Disposable(self.cmd_vel.subscribe(self._on_cmd_vel)))
        self._disposables.add(Disposable(self.terrain_map.subscribe(self._on_terrain)))
        self._running.set()
        self._sim_thread = threading.Thread(target=self._sim_loop, daemon=True)
        self._sim_thread.start()
        self._unity_thread = threading.Thread(target=self._unity_loop, daemon=True)
        self._unity_thread.start()
        # Launch Unity in a thread to avoid blocking start() for up to
        # unity_connect_timeout seconds (default 30s).
        threading.Thread(target=self._launch_unity, daemon=True).start()

    @rpc
    def stop(self) -> None:
        self._running.clear()
        if self._sim_thread:
            self._sim_thread.join(timeout=2.0)
        if self._unity_thread:
            self._unity_thread.join(timeout=2.0)
        if self._unity_process is not None and self._unity_process.poll() is None:
            logger.info(f"Stopping Unity (pid={self._unity_process.pid})")
            self._unity_process.send_signal(signal.SIGTERM)
            try:
                self._unity_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                logger.warning(
                    f"Unity pid={self._unity_process.pid} did not exit after SIGTERM, killing"
                )
                self._unity_process.kill()
            self._unity_process = None
        super().stop()

    def _resolve_binary(self) -> Path | None:
        """Find the Unity binary from config or LFS data.

        When ``unity_binary`` is empty (default), pulls the LFS asset
        ``unity_sim_x86`` via ``get_data()`` and returns the path to
        ``environment/Model.x86_64``.
        """
        cfg = self.config

        # Explicit path provided
        if cfg.unity_binary:
            p = Path(cfg.unity_binary)
            if not p.is_absolute():
                p = Path.cwd() / p
                if not p.exists():
                    p = (Path(__file__).resolve().parent / cfg.unity_binary).resolve()
            if p.exists():
                return p
            logger.warning(f"Unity binary not found at {p}")
            return None

        # Pull from LFS (auto-downloads + extracts on first use)
        try:
            data_dir = get_data(_LFS_ASSET)
            candidate = data_dir / "environment" / "Model.x86_64"
            if candidate.exists():
                return candidate
            logger.warning(f"LFS asset '{_LFS_ASSET}' extracted but Model.x86_64 not found")
        except Exception as e:
            logger.warning(f"Failed to resolve Unity binary from LFS: {e}")

        return None

    def _launch_unity(self) -> None:
        """Launch the Unity simulator binary as a subprocess."""
        binary_path = self._binary_path
        if binary_path is None:
            logger.info("No Unity binary — TCP server will wait for external connection")
            return

        _validate_platform()

        if not os.access(binary_path, os.X_OK):
            binary_path.chmod(binary_path.stat().st_mode | 0o111)

        cmd = [str(binary_path)]
        if self.config.headless:
            cmd.extend(["-batchmode", "-nographics"])
        cmd.extend(self.config.unity_extra_args)

        logger.info(f"Launching Unity: {' '.join(cmd)}")
        env = {**os.environ}
        if "DISPLAY" not in env and not self.config.headless:
            env["DISPLAY"] = ":0"

        self._unity_process = subprocess.Popen(
            cmd,
            cwd=str(binary_path.parent),
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
        )

        # Read Unity stderr in a background thread for diagnostics.
        def _drain_stderr() -> None:
            assert self._unity_process is not None
            assert self._unity_process.stderr is not None
            for raw in self._unity_process.stderr:
                line = raw.decode("utf-8", errors="replace").rstrip()
                if line:
                    logger.warning(f"Unity stderr: {line}")
            self._unity_process.stderr.close()

        threading.Thread(target=_drain_stderr, daemon=True).start()
        logger.info(f"Unity pid={self._unity_process.pid}, waiting for TCP connection...")

        if self._unity_ready.wait(timeout=self.config.unity_connect_timeout):
            logger.info("Unity connected")
        else:
            # Check if process died
            rc = self._unity_process.poll()
            if rc is not None:
                logger.error(
                    f"Unity process exited with code {rc} before connecting. "
                    f"Check that DISPLAY is set and the binary is not corrupted."
                )
            else:
                logger.warning(
                    f"Unity did not connect within {self.config.unity_connect_timeout}s. "
                    f"The binary may still be loading — it will connect when ready."
                )

    def _on_cmd_vel(self, twist: Twist) -> None:
        with self._cmd_lock:
            self._fwd_speed = twist.linear.x
            self._left_speed = twist.linear.y
            self._yaw_rate = twist.angular.z

    def _on_terrain(self, cloud: PointCloud2) -> None:
        points, _ = cloud.as_numpy()
        if len(points) == 0:
            return
        with self._state_lock:
            cur_x, cur_y = self._x, self._y
        dx = points[:, 0] - cur_x
        dy = points[:, 1] - cur_y
        near = points[np.sqrt(dx * dx + dy * dy) < 0.5]
        if len(near) >= 10:
            with self._state_lock:
                self._terrain_z = 0.8 * self._terrain_z + 0.2 * near[:, 2].mean()

    def _unity_loop(self) -> None:
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_sock.bind((self.config.unity_host, self.config.unity_port))
            server_sock.listen(1)
            server_sock.settimeout(2.0)
            logger.info(f"TCP server on :{self.config.unity_port}")

            while self._running.is_set():
                try:
                    conn, addr = server_sock.accept()
                    logger.info(f"Unity connected from {addr}")
                    try:
                        self._bridge_connection(conn)
                    except Exception as e:
                        logger.warning(f"Unity connection ended: {e}")
                    finally:
                        with self._state_lock:
                            self._unity_connected = False
                        self._unity_ready.clear()
                        conn.close()
                except TimeoutError:
                    continue
                except Exception as e:
                    if self._running.is_set():
                        logger.warning(f"TCP server error: {e}")
                        time.sleep(1.0)
        finally:
            server_sock.close()

    def _bridge_connection(self, sock: socket.socket) -> None:
        # Drain stale messages from a previous session.
        while True:
            try:
                self._send_queue.get_nowait()
            except Empty:
                break

        sock.settimeout(_BRIDGE_READ_TIMEOUT)
        with self._state_lock:
            self._unity_connected = True
        self._unity_ready.set()

        _write_tcp_command(
            sock,
            "__handshake",
            {
                "version": "v0.7.0",
                "metadata": json.dumps({"protocol": "ROS2"}),
            },
        )

        halt = threading.Event()
        sender = threading.Thread(target=self._unity_sender, args=(sock, halt), daemon=True)
        sender.start()

        try:
            while self._running.is_set() and not halt.is_set():
                try:
                    dest, data = _read_tcp_message(sock)
                except TimeoutError:
                    continue
                if dest == "":
                    continue
                elif dest.startswith("__"):
                    self._handle_syscommand(dest, data)
                else:
                    self._handle_unity_message(dest, data)
        finally:
            halt.set()
            sender.join(timeout=2.0)
            with self._state_lock:
                self._unity_connected = False

    def _unity_sender(self, sock: socket.socket, halt: threading.Event) -> None:
        while not halt.is_set():
            try:
                dest, data = self._send_queue.get(timeout=1.0)
                if dest == "__raw__":
                    sock.sendall(data)
                else:
                    _write_tcp_message(sock, dest, data)
            except Empty:
                continue
            except Exception as e:
                logger.warning(f"Unity sender error: {e}")
                halt.set()

    def _handle_syscommand(self, dest: str, data: bytes) -> None:
        payload = data.rstrip(b"\x00")
        try:
            params = json.loads(payload.decode("utf-8"))
        except (json.JSONDecodeError, UnicodeDecodeError):
            params = {}

        cmd = dest[2:]
        logger.info(f"Unity syscmd: {cmd} {params}")

        if cmd == "topic_list":
            resp = json.dumps(
                {
                    "topics": ["/unity_sim/set_model_state", "/tf"],
                    "types": ["geometry_msgs/PoseStamped", "tf2_msgs/TFMessage"],
                }
            ).encode("utf-8")
            hdr = b"__topic_list"
            frame = struct.pack("<I", len(hdr)) + hdr + struct.pack("<I", len(resp)) + resp
            self._send_queue.put(("__raw__", frame))

    def _handle_unity_message(self, topic: str, data: bytes) -> None:
        if topic == "/registered_scan":
            pc_result = deserialize_pointcloud2(data)
            if pc_result is not None:
                points, frame_id, ts = pc_result
                if len(points) > 0:
                    self.registered_scan.publish(
                        PointCloud2.from_numpy(points, frame_id=frame_id, timestamp=ts)
                    )

        elif "image" in topic and "compressed" in topic:
            img_result = deserialize_compressed_image(data)
            if img_result is not None:
                img_bytes, _fmt, _frame_id, ts = img_result
                try:
                    decoded = cv2.imdecode(np.frombuffer(img_bytes, np.uint8), cv2.IMREAD_COLOR)
                    if decoded is not None:
                        img = Image.from_numpy(decoded, frame_id="camera", ts=ts)
                        if "semantic" in topic:
                            self.semantic_image.publish(img)
                        else:
                            self.color_image.publish(img)
                            h, w = decoded.shape[:2]
                            self._publish_camera_info(w, h, ts)
                except Exception as e:
                    logger.warning(f"Image decode failed ({topic}): {e}")

    def _publish_camera_info(self, width: int, height: int, ts: float) -> None:
        # Use the same intrinsics as rerun_static_pinhole (120° HFOV pinhole
        # approximation of the cylindrical panorama).
        self.camera_info.publish(
            CameraInfo(
                height=height,
                width=width,
                distortion_model="plumb_bob",
                D=[0.0, 0.0, 0.0, 0.0, 0.0],
                K=[_CAM_FX, 0.0, _CAM_CX, 0.0, _CAM_FY, _CAM_CY, 0.0, 0.0, 1.0],
                R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                P=[_CAM_FX, 0.0, _CAM_CX, 0.0, 0.0, _CAM_FY, _CAM_CY, 0.0, 0.0, 0.0, 1.0, 0.0],
                frame_id="camera",
                ts=ts,
            )
        )

    def _send_to_unity(self, topic: str, data: bytes) -> None:
        with self._state_lock:
            connected = self._unity_connected
        if connected:
            self._send_queue.put((topic, data))

    def _sim_loop(self) -> None:
        dt = 1.0 / self.config.sim_rate

        while self._running.is_set():
            t0 = time.monotonic()

            with self._cmd_lock:
                fwd, left, yaw_rate = self._fwd_speed, self._left_speed, self._yaw_rate

            with self._state_lock:
                prev_z = self._z

                self._yaw += dt * yaw_rate
                if self._yaw > PI:
                    self._yaw -= 2 * PI
                elif self._yaw < -PI:
                    self._yaw += 2 * PI

                cy, sy = math.cos(self._yaw), math.sin(self._yaw)
                self._x += dt * cy * fwd - dt * sy * left
                self._y += dt * sy * fwd + dt * cy * left
                self._z = self._terrain_z + self.config.vehicle_height

                x, y, z = self._x, self._y, self._z
                yaw = self._yaw
                roll, pitch = self._roll, self._pitch

            now = time.time()
            quat = Quaternion.from_euler(Vector3(roll, pitch, yaw))

            self.odometry.publish(
                Odometry(
                    ts=now,
                    frame_id="map",
                    child_frame_id="sensor",
                    pose=Pose(
                        position=[x, y, z],
                        orientation=[quat.x, quat.y, quat.z, quat.w],
                    ),
                    twist=Twist(
                        linear=[fwd, left, (z - prev_z) * self.config.sim_rate],
                        angular=[0.0, 0.0, yaw_rate],
                    ),
                )
            )

            self.tf.publish(
                Transform(
                    translation=Vector3(x, y, z),
                    rotation=quat,
                    frame_id="map",
                    child_frame_id="sensor",
                    ts=now,
                ),
                Transform(
                    translation=Vector3(0.0, 0.0, 0.0),
                    rotation=Quaternion(0.0, 0.0, 0.0, 1.0),
                    frame_id="map",
                    child_frame_id="world",
                    ts=now,
                ),
            )

            with self._state_lock:
                unity_connected = self._unity_connected
            if unity_connected:
                self._send_to_unity(
                    "/unity_sim/set_model_state",
                    serialize_pose_stamped(x, y, z, quat.x, quat.y, quat.z, quat.w),
                )

            sleep_for = dt - (time.monotonic() - t0)
            if sleep_for > 0:
                time.sleep(sleep_for)
