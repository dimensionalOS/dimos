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

"""Mac-side TCP client for the M20 Mac Bridge.

Drop-in replacement for M20ROSSensors — same public interface so
connection.py can use either interchangeably. Connects to the GOS-side
mac_bridge.py TCP server and reconstructs dimos message types from
the wire frames.

Usage:
    client = M20MacBridgeClient("10.21.31.104")
    client.start()
    client.odom_stream().subscribe(lambda pose: print(pose))
    client.publish_nav_cmd(0.5, 0.0, 0.0)
    client.stop()
"""

import json
import logging
import socket
import struct
import threading
import time
from typing import Any

import numpy as np
from reactivex.observable import Observable
from reactivex.subject import Subject

from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Twist import Twist as DimTwist
from dimos.msgs.nav_msgs.Odometry import Odometry as DimosOdometry
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2 as DimosPointCloud2
from dimos.msgs.sensor_msgs.Imu import Imu as DimosImu

from .ros_sensors import MotionInfoData

logger = logging.getLogger(__name__)

# --- Wire protocol (duplicated from mac_bridge.py — GOS can't import dimos) ---

MSG_ODOM = 0x01
MSG_TF = 0x02
MSG_LIDAR = 0x03
MSG_IMU = 0x04
MSG_MOTION_INFO = 0x05
MSG_NAV_CMD = 0x10
MSG_HEARTBEAT = 0x20

MAGIC = b'\xd1\x05'
MAX_FRAME_SIZE = 1048576  # 1 MB


class FrameProtocol:
    """TCP frame encode/decode for the Mac bridge wire protocol."""

    MAGIC = MAGIC
    MAX_FRAME_SIZE = MAX_FRAME_SIZE

    @staticmethod
    def encode(msg_type: int, payload: bytes) -> bytes:
        header = MAGIC + struct.pack("!I", len(payload)) + struct.pack("B", msg_type)
        return header + payload

    @staticmethod
    def read_frame(sock: socket.socket) -> tuple[int, bytes]:
        """Blocking read of one frame from socket.

        If the first bytes aren't magic, scans forward byte-by-byte to
        resynchronize (handles starting mid-stream after reconnect).
        """
        # Read first byte
        b0 = _recv_exact(sock, 1)
        # Scan for magic bytes if not aligned
        skipped = 0
        while True:
            b1 = _recv_exact(sock, 1)
            if b0 + b1 == MAGIC:
                break
            if skipped > MAX_FRAME_SIZE:
                raise ValueError("Could not find magic bytes after scanning %d bytes" % skipped)
            b0 = b1
            skipped += 1
        if skipped > 0:
            logger.debug("Resynchronized after skipping %d bytes", skipped)
        length_bytes = _recv_exact(sock, 4)
        length = struct.unpack("!I", length_bytes)[0]
        if length > MAX_FRAME_SIZE:
            raise ValueError(f"Frame too large: {length} bytes")
        type_byte = _recv_exact(sock, 1)
        msg_type = struct.unpack("B", type_byte)[0]
        payload = _recv_exact(sock, length) if length > 0 else b''
        return msg_type, payload


def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("Connection closed")
        buf.extend(chunk)
    return bytes(buf)


# --- Message reconstruction ---

STALENESS_THRESHOLD = 2.0  # seconds (monotonic)


def _decode_odom(payload: bytes) -> PoseStamped:
    d = json.loads(payload)
    return PoseStamped(
        position=Vector3(d["x"], d["y"], d["z"]),
        orientation=Quaternion(d["qx"], d["qy"], d["qz"], d["qw"]),
        frame_id=d.get("frame_id", "odom"),
        ts=d["ts"],
    )


def _decode_tf(payload: bytes) -> list[Transform]:
    items = json.loads(payload)
    transforms = []
    for t in items:
        transforms.append(
            Transform(
                translation=Vector3(t["tx"], t["ty"], t["tz"]),
                rotation=Quaternion(t["rx"], t["ry"], t["rz"], t["rw"]),
                frame_id=t["frame_id"],
                child_frame_id=t["child_frame_id"],
                ts=t["ts"],
            )
        )
    return transforms


def _decode_lidar(payload: bytes) -> DimosPointCloud2:
    header_len = struct.unpack("!I", payload[:4])[0]
    header = json.loads(payload[4:4 + header_len])
    n_points = header["n_points"]
    ts = header["ts"]
    frame_id = header.get("frame_id", "world")

    point_data = payload[4 + header_len:]
    if n_points > 0 and len(point_data) >= n_points * 12:
        points = np.frombuffer(point_data[:n_points * 12], dtype=np.float32).reshape(n_points, 3)
    else:
        points = np.empty((0, 3), dtype=np.float32)

    return DimosPointCloud2.from_numpy(points, frame_id=frame_id, timestamp=ts)


def _decode_motion_info(payload: bytes) -> MotionInfoData:
    d = json.loads(payload)
    return MotionInfoData(
        vel_x=d["vel_x"],
        vel_y=d["vel_y"],
        vel_yaw=d["vel_yaw"],
        height=d["height"],
        state=d["state"],
        gait=d["gait"],
        remain_mile=d["remain_mile"],
    )


def _decode_imu(payload: bytes) -> DimosImu:
    d = json.loads(payload)
    return DimosImu(
        angular_velocity=Vector3(d["gx"], d["gy"], d["gz"]),
        linear_acceleration=Vector3(d["ax"], d["ay"], d["az"]),
        orientation=Quaternion(d["ox"], d["oy"], d["oz"], d["ow"]),
        frame_id=d.get("frame_id", "imu_link"),
        ts=d["ts"],
    )


def _decode_odometry(payload: bytes) -> DimosOdometry:
    d = json.loads(payload)
    pose = Pose(
        position=[d["x"], d["y"], d["z"]],
        orientation=[d["qx"], d["qy"], d["qz"], d["qw"]],
    )
    twist = DimTwist(
        linear=[d.get("lx", 0), d.get("ly", 0), d.get("lz", 0)],
        angular=[d.get("ax", 0), d.get("ay", 0), d.get("az", 0)],
    )
    return DimosOdometry(
        ts=d["ts"],
        frame_id=d.get("frame_id", "odom"),
        child_frame_id=d.get("child_frame_id", "base_link"),
        pose=pose,
        twist=twist,
    )


class M20MacBridgeClient:
    """TCP client connecting to the GOS-side Mac bridge.

    Drop-in replacement for M20ROSSensors — same start/stop/stream/publish
    interface so connection.py can use either interchangeably.
    """

    def __init__(
        self,
        bridge_host: str,
        bridge_port: int = 9731,
        reconnect_delay: float = 2.0,
    ) -> None:
        self._host = bridge_host
        self._port = bridge_port
        self._reconnect_delay = reconnect_delay

        # Thread coordination
        self._shutdown = threading.Event()
        self._connected = threading.Event()
        self._send_lock = threading.Lock()
        self._sock: socket.socket | None = None

        # Threads
        self._recv_thread: threading.Thread | None = None
        self._heartbeat_thread: threading.Thread | None = None

        # Observable subjects
        self._odom_subject: Subject[PoseStamped] = Subject()
        self._odometry_subject: Subject[DimosOdometry] = Subject()
        self._tf_subject: Subject[list[Transform]] = Subject()
        self._lidar_subject: Subject[DimosPointCloud2] = Subject()
        self._imu_subject: Subject[DimosImu] = Subject()
        self._motion_info_subject: Subject[MotionInfoData] = Subject()

    def start(self) -> None:
        """Start recv and heartbeat threads (non-blocking)."""
        if self._recv_thread is not None and self._recv_thread.is_alive():
            return  # idempotent

        self._shutdown.clear()
        self._recv_thread = threading.Thread(
            target=self._recv_loop, daemon=True, name="bridge_recv"
        )
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop, daemon=True, name="bridge_hb"
        )
        self._recv_thread.start()
        self._heartbeat_thread.start()
        logger.info("M20MacBridgeClient started — connecting to %s:%d", self._host, self._port)

    def stop(self) -> None:
        """Shut down connection and join threads."""
        self._shutdown.set()
        self._connected.clear()

        # Close socket to unblock recv
        sock = self._sock
        if sock is not None:
            try:
                sock.close()
            except Exception:
                pass

        if self._recv_thread is not None:
            self._recv_thread.join(timeout=5)
        if self._heartbeat_thread is not None:
            self._heartbeat_thread.join(timeout=5)

        self._recv_thread = None
        self._heartbeat_thread = None
        self._sock = None
        logger.info("M20MacBridgeClient stopped")

    # --- Observable streams (same interface as M20ROSSensors) ---

    def odom_stream(self) -> Observable[PoseStamped]:
        return self._odom_subject

    def odometry_stream(self) -> Observable[DimosOdometry]:
        return self._odometry_subject

    def tf_stream(self) -> Observable[list[Transform]]:
        return self._tf_subject

    def lidar_stream(self) -> Observable[DimosPointCloud2]:
        return self._lidar_subject

    def imu_stream(self) -> Observable[DimosImu]:
        return self._imu_subject

    def motion_info_stream(self) -> Observable[MotionInfoData]:
        return self._motion_info_subject

    # --- /NAV_CMD publisher ---

    @property
    def nav_cmd_available(self) -> bool:
        return True

    def publish_nav_cmd(self, x_vel: float, y_vel: float, yaw_vel: float) -> None:
        payload = json.dumps({
            "x_vel": float(x_vel),
            "y_vel": float(y_vel),
            "yaw_vel": float(yaw_vel),
        }).encode("utf-8")
        self._send_frame(MSG_NAV_CMD, payload)

    # --- Internal ---

    def _send_frame(self, msg_type: int, payload: bytes) -> None:
        if not self._connected.is_set():
            return
        frame = FrameProtocol.encode(msg_type, payload)
        with self._send_lock:
            sock = self._sock
            if sock is None:
                return
            try:
                sock.sendall(frame)
            except (OSError, BrokenPipeError):
                self._connected.clear()

    def _recv_loop(self) -> None:
        """Connect with auto-reconnect, read frames, dispatch to Subjects."""
        while not self._shutdown.is_set():
            # Connect
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)
                sock.connect((self._host, self._port))
                sock.settimeout(1.0)
            except (OSError, ConnectionRefusedError) as e:
                logger.debug("Connect failed (%s), retrying in %.1fs", e, self._reconnect_delay)
                try:
                    sock.close()
                except Exception:
                    pass
                # Interruptible backoff sleep
                if self._shutdown.wait(self._reconnect_delay):
                    return
                continue

            self._sock = sock
            self._connected.set()
            logger.info("Connected to bridge at %s:%d", self._host, self._port)

            # Receive loop
            try:
                while not self._shutdown.is_set():
                    try:
                        msg_type, payload = FrameProtocol.read_frame(sock)
                    except socket.timeout:
                        continue
                    except (ConnectionError, ValueError) as e:
                        logger.info("Bridge read error: %s", e)
                        break

                    recv_mono = time.monotonic()
                    self._dispatch(msg_type, payload, recv_mono)
            finally:
                self._connected.clear()
                try:
                    sock.close()
                except Exception:
                    pass
                self._sock = None

            if not self._shutdown.is_set():
                logger.info("Disconnected from bridge, reconnecting in %.1fs", self._reconnect_delay)
                if self._shutdown.wait(self._reconnect_delay):
                    return

    def _dispatch(self, msg_type: int, payload: bytes, recv_mono: float) -> None:
        """Decode frame and emit to appropriate Subject."""
        try:
            if msg_type == MSG_ODOM:
                self._odom_subject.on_next(_decode_odom(payload))
                self._odometry_subject.on_next(_decode_odometry(payload))
            elif msg_type == MSG_TF:
                self._tf_subject.on_next(_decode_tf(payload))
            elif msg_type == MSG_LIDAR:
                self._lidar_subject.on_next(_decode_lidar(payload))
            elif msg_type == MSG_IMU:
                self._imu_subject.on_next(_decode_imu(payload))
            elif msg_type == MSG_MOTION_INFO:
                self._motion_info_subject.on_next(_decode_motion_info(payload))
            else:
                logger.debug("Unknown msg_type=0x%02x, skipping", msg_type)
        except Exception:
            logger.debug("Error decoding msg_type=0x%02x", msg_type, exc_info=True)

    def _heartbeat_loop(self) -> None:
        """Send heartbeat every 5s while connected."""
        while not self._shutdown.is_set():
            if self._connected.is_set():
                self._send_frame(MSG_HEARTBEAT, b'')
            # Sleep in 1s increments so shutdown is responsive
            if self._shutdown.wait(5.0):
                return
