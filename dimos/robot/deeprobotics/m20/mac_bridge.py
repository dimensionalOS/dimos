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

"""M20 Mac Bridge — GOS-side TCP server forwarding ROS2 topics.

Standalone Python 3.8 script. No dimos imports. Uses only stdlib + rclpy + drdds.

Subscribes to M20 ROS2 topics on GOS and forwards them over TCP to a Mac client
running dimos. Also receives /NAV_CMD velocity commands from the Mac and publishes
them to ROS2.

Usage:
    python3 mac_bridge.py --port 9731
    python3 mac_bridge.py --port 9731 --node-name my_bridge

Reference: M20 Software Development Guide sections 2.1-2.3
"""

import argparse
import json
import logging
import signal
import socket
import struct
import threading
import time
from typing import Any, Callable, Dict, Optional, Tuple

try:
    import numpy as np
    _NUMPY = True
except ImportError:
    _NUMPY = False

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from tf2_msgs.msg import TFMessage

try:
    from drdds.msg import MotionInfo, NavCmd
    _DRDDS = True
except ImportError:
    _DRDDS = False

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
logger = logging.getLogger("mac_bridge")

# --- Wire protocol ---

MSG_ODOM = 0x01
MSG_TF = 0x02
MSG_LIDAR = 0x03
MSG_IMU = 0x04
MSG_MOTION_INFO = 0x05
MSG_NAV_CMD = 0x10
MSG_HEARTBEAT = 0x20

MAGIC = b'\xd1\x05'
MAX_FRAME_SIZE = 1048576  # 1 MB


def _encode_frame(msg_type, payload):
    # type: (int, bytes) -> bytes
    header = MAGIC + struct.pack("!I", len(payload)) + struct.pack("B", msg_type)
    return header + payload


def _recv_exact(sock, n):
    # type: (socket.socket, int) -> bytes
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("Connection closed")
        buf.extend(chunk)
    return bytes(buf)


def _read_frame(sock):
    # type: (socket.socket) -> Tuple[int, bytes]
    magic = _recv_exact(sock, 2)
    if magic != MAGIC:
        raise ValueError("Bad magic bytes: %r" % magic)
    length_bytes = _recv_exact(sock, 4)
    length = struct.unpack("!I", length_bytes)[0]
    if length > MAX_FRAME_SIZE:
        raise ValueError("Frame too large: %d bytes" % length)
    type_byte = _recv_exact(sock, 1)
    msg_type = struct.unpack("B", type_byte)[0]
    payload = _recv_exact(sock, length) if length > 0 else b''
    return msg_type, payload


# --- ROS adapter ---

class _ROSAdapter(object):
    """Wraps rclpy node, subscriptions, and /NAV_CMD publisher."""

    def __init__(self, node_name, callbacks):
        # type: (str, Dict[str, Callable]) -> None
        self._node_name = node_name
        self._callbacks = callbacks
        self._node = None  # type: Optional[Node]
        self._executor = None  # type: Optional[SingleThreadedExecutor]
        self._spin_thread = None  # type: Optional[threading.Thread]
        self._nav_pub = None

    def start(self):
        # type: () -> None
        rclpy.init()
        self._node = rclpy.create_node(self._node_name)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10,
        )

        self._node.create_subscription(Odometry, "/ODOM", self._callbacks["odom"], qos)
        self._node.create_subscription(TFMessage, "/tf", self._callbacks["tf"], qos)
        self._node.create_subscription(PointCloud2, "/ALIGNED_POINTS", self._callbacks["lidar"], qos)
        self._node.create_subscription(Imu, "/IMU", self._callbacks["imu"], qos)

        if _DRDDS:
            self._node.create_subscription(MotionInfo, "/MOTION_INFO", self._callbacks["motion_info"], qos)
            self._nav_pub = self._node.create_publisher(NavCmd, "/NAV_CMD", qos)
            logger.info("drdds available — /NAV_CMD publisher and /MOTION_INFO subscription enabled")
        else:
            logger.warning("drdds not available — /NAV_CMD and /MOTION_INFO disabled")

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._spin, daemon=True, name="ros_spin")
        self._spin_thread.start()
        logger.info("ROS adapter started: subscriptions active")

    def _spin(self):
        # type: () -> None
        try:
            self._executor.spin()
        except Exception:
            logger.exception("ROS spin exited with error")

    def stop(self):
        # type: () -> None
        # Shutdown order critical for Foxy — wrong order causes segfault
        if self._executor is not None:
            self._executor.shutdown()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=5)
        if self._node is not None:
            self._node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
        logger.info("ROS adapter stopped")

    def publish_nav_cmd(self, x_vel, y_vel, yaw_vel):
        # type: (float, float, float) -> None
        if self._nav_pub is None:
            return
        msg = NavCmd()
        msg.data.x_vel = float(x_vel)
        msg.data.y_vel = float(y_vel)
        msg.data.yaw_vel = float(yaw_vel)
        now = time.time()
        msg.header.stamp.sec = int(now)
        msg.header.stamp.nanosec = int((now - int(now)) * 1e9)
        self._nav_pub.publish(msg)


# --- Bridge server ---

class M20MacBridge(object):
    """TCP server forwarding M20 ROS2 topics to a Mac client."""

    def __init__(self, port=9731, ros_node_name="dimos_mac_bridge"):
        # type: (int, str) -> None
        self._port = port
        self._shutdown = threading.Event()

        # Per-topic latest-value buffers: msg_type -> encoded frame bytes
        self._buffers = {}  # type: Dict[int, bytes]
        self._buf_lock = threading.Lock()

        # Client state
        self._client_connected = False
        self._client_sock = None  # type: Optional[socket.socket]
        self._client_lock = threading.Lock()

        # NAV_CMD watchdog
        self._last_nav_cmd_time = 0.0  # monotonic
        self._watchdog_interval = 0.5  # 500ms

        # Heartbeat timeout
        self._last_heartbeat_time = 0.0  # monotonic
        self._heartbeat_timeout = 15.0

        # ROS adapter
        self._ros = _ROSAdapter(ros_node_name, {
            "odom": self._on_odom,
            "tf": self._on_tf,
            "lidar": self._on_lidar,
            "imu": self._on_imu,
            "motion_info": self._on_motion_info,
        })

        self._sender_thread = None  # type: Optional[threading.Thread]

    def start(self):
        # type: () -> None
        """Start bridge. Blocks in accept loop until stop() is called."""
        self._ros.start()

        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.settimeout(1.0)
        server.bind(("0.0.0.0", self._port))
        server.listen(1)
        logger.info("Mac bridge listening on port %d", self._port)

        try:
            self._accept_loop(server)
        finally:
            server.close()
            self._ros.stop()

    def stop(self):
        # type: () -> None
        self._shutdown.set()

    def _accept_loop(self, server_sock):
        # type: (socket.socket) -> None
        while not self._shutdown.is_set():
            try:
                client, addr = server_sock.accept()
            except socket.timeout:
                continue
            except OSError:
                if self._shutdown.is_set():
                    break
                raise

            logger.info("Client connected from %s:%d", addr[0], addr[1])

            # Clear stale buffers
            with self._buf_lock:
                self._buffers.clear()

            self._last_heartbeat_time = time.monotonic()
            self._last_nav_cmd_time = time.monotonic()

            with self._client_lock:
                self._client_sock = client
                self._client_connected = True

            # Start sender thread for this client
            self._sender_thread = threading.Thread(
                target=self._sender_loop, daemon=True, name="sender"
            )
            self._sender_thread.start()

            try:
                self._handle_client(client)
            except Exception as e:
                logger.info("Client disconnected: %s", e)
            finally:
                with self._client_lock:
                    self._client_connected = False
                    self._client_sock = None
                try:
                    client.close()
                except Exception:
                    pass
                if self._sender_thread is not None:
                    self._sender_thread.join(timeout=2)

    def _handle_client(self, client_sock):
        # type: (socket.socket) -> None
        """Read NAV_CMD and HEARTBEAT frames from client."""
        client_sock.settimeout(1.0)
        while not self._shutdown.is_set():
            # Check heartbeat timeout
            if time.monotonic() - self._last_heartbeat_time > self._heartbeat_timeout:
                raise ConnectionError("Heartbeat timeout (%.0fs)" % self._heartbeat_timeout)

            try:
                msg_type, payload = _read_frame(client_sock)
            except socket.timeout:
                continue
            except (ConnectionError, ValueError) as e:
                raise ConnectionError("Read error: %s" % e)

            if msg_type == MSG_HEARTBEAT:
                self._last_heartbeat_time = time.monotonic()
            elif msg_type == MSG_NAV_CMD:
                self._last_nav_cmd_time = time.monotonic()
                self._last_heartbeat_time = time.monotonic()
                try:
                    data = json.loads(payload)
                    self._ros.publish_nav_cmd(
                        data["x_vel"], data["y_vel"], data["yaw_vel"]
                    )
                except (json.JSONDecodeError, KeyError) as e:
                    logger.warning("Bad NAV_CMD payload: %s", e)
            else:
                logger.debug("Ignoring unknown msg_type=0x%02x from client", msg_type)

    def _sender_loop(self):
        # type: () -> None
        """Sweep per-topic buffers at ~200Hz and send to client."""
        while not self._shutdown.is_set():
            with self._client_lock:
                if not self._client_connected:
                    break
                sock = self._client_sock

            # NAV_CMD watchdog: auto-publish zero velocity if no command for 500ms
            if _DRDDS and time.monotonic() - self._last_nav_cmd_time > self._watchdog_interval:
                self._ros.publish_nav_cmd(0.0, 0.0, 0.0)
                self._last_nav_cmd_time = time.monotonic()

            # Sweep buffers
            frames_to_send = []  # type: list
            with self._buf_lock:
                for msg_type, frame_bytes in self._buffers.items():
                    frames_to_send.append(frame_bytes)
                self._buffers.clear()

            if frames_to_send and sock is not None:
                data = b''.join(frames_to_send)
                try:
                    sock.sendall(data)
                except (OSError, BrokenPipeError):
                    with self._client_lock:
                        self._client_connected = False
                    break

            time.sleep(0.005)  # ~200Hz

    def _buffer_frame(self, msg_type, payload):
        # type: (int, bytes) -> None
        """Write encoded frame to per-topic buffer (called from ROS callbacks)."""
        with self._client_lock:
            if not self._client_connected:
                return
        frame = _encode_frame(msg_type, payload)
        with self._buf_lock:
            self._buffers[msg_type] = frame

    # --- ROS callbacks ---

    def _on_odom(self, msg):
        # type: (Any) -> None
        try:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            stamp = msg.header.stamp
            data = {
                "x": float(p.x), "y": float(p.y), "z": float(p.z),
                "qx": float(q.x), "qy": float(q.y),
                "qz": float(q.z), "qw": float(q.w),
                "ts": float(stamp.sec) + float(stamp.nanosec) * 1e-9,
                "frame_id": msg.header.frame_id or "odom",
            }
            self._buffer_frame(MSG_ODOM, json.dumps(data).encode("utf-8"))
        except Exception:
            logger.debug("Error encoding /ODOM", exc_info=True)

    def _on_tf(self, msg):
        # type: (Any) -> None
        try:
            transforms = []
            for t in msg.transforms:
                stamp = t.header.stamp
                tr = t.transform.translation
                ro = t.transform.rotation
                transforms.append({
                    "frame_id": t.header.frame_id,
                    "child_frame_id": t.child_frame_id,
                    "tx": float(tr.x), "ty": float(tr.y), "tz": float(tr.z),
                    "rx": float(ro.x), "ry": float(ro.y),
                    "rz": float(ro.z), "rw": float(ro.w),
                    "ts": float(stamp.sec) + float(stamp.nanosec) * 1e-9,
                })
            self._buffer_frame(MSG_TF, json.dumps(transforms).encode("utf-8"))
        except Exception:
            logger.debug("Error encoding /tf", exc_info=True)

    def _on_lidar(self, msg):
        # type: (Any) -> None
        try:
            n_points = msg.width * msg.height
            point_step = msg.point_step
            stamp = msg.header.stamp
            ts = float(stamp.sec) + float(stamp.nanosec) * 1e-9
            frame_id = msg.header.frame_id or "world"

            if n_points == 0 or len(msg.data) < n_points * point_step:
                return

            # Find x, y, z field offsets
            offsets = {}
            for f in msg.fields:
                if f.name in ("x", "y", "z"):
                    offsets[f.name] = f.offset

            if not all(k in offsets for k in ("x", "y", "z")):
                return

            if _NUMPY:
                raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
                raw_2d = raw[:n_points * point_step].reshape(n_points, point_step)
                # Extract x, y, z columns — use tobytes + frombuffer for numpy 1.17 compat
                cols = []
                for axis in ("x", "y", "z"):
                    off = offsets[axis]
                    col_bytes = raw_2d[:, off:off + 4].tobytes()
                    cols.append(np.frombuffer(col_bytes, dtype="<f4"))
                points = np.column_stack(cols)
                valid = np.isfinite(points).all(axis=1)
                points = points[valid]
                n_valid = len(points)
                point_bytes = points.astype("<f4").tobytes()
            else:
                # Fallback: struct-based extraction
                raw = bytes(msg.data)
                point_list = []
                for i in range(n_points):
                    base = i * point_step
                    vals = []
                    skip = False
                    for axis in ("x", "y", "z"):
                        off = offsets[axis]
                        v = struct.unpack_from("<f", raw, base + off)[0]
                        if v != v or abs(v) == float("inf"):  # NaN or Inf check
                            skip = True
                            break
                        vals.append(v)
                    if not skip:
                        point_list.append(struct.pack("<fff", *vals))
                n_valid = len(point_list)
                point_bytes = b''.join(point_list)

            header = {"n_points": int(n_valid), "ts": ts, "frame_id": frame_id}
            header_bytes = json.dumps(header).encode("utf-8")
            payload = struct.pack("!I", len(header_bytes)) + header_bytes + point_bytes
            self._buffer_frame(MSG_LIDAR, payload)
        except Exception:
            logger.debug("Error encoding /ALIGNED_POINTS", exc_info=True)

    def _on_imu(self, msg):
        # type: (Any) -> None
        try:
            a = msg.linear_acceleration
            g = msg.angular_velocity
            o = msg.orientation
            stamp = msg.header.stamp
            data = {
                "ax": float(a.x), "ay": float(a.y), "az": float(a.z),
                "gx": float(g.x), "gy": float(g.y), "gz": float(g.z),
                "ox": float(o.x), "oy": float(o.y),
                "oz": float(o.z), "ow": float(o.w),
                "ts": float(stamp.sec) + float(stamp.nanosec) * 1e-9,
            }
            self._buffer_frame(MSG_IMU, json.dumps(data).encode("utf-8"))
        except Exception:
            logger.debug("Error encoding /IMU", exc_info=True)

    def _on_motion_info(self, msg):
        # type: (Any) -> None
        try:
            d = msg.data
            data = {
                "vel_x": float(d.vel_x),
                "vel_y": float(d.vel_y),
                "vel_yaw": float(d.vel_yaw),
                "height": float(d.height),
                "state": int(d.motion_state.state),
                "gait": int(d.gait_state.gait),
                "remain_mile": float(d.remain_mile),
            }
            self._buffer_frame(MSG_MOTION_INFO, json.dumps(data).encode("utf-8"))
        except Exception:
            logger.debug("Error encoding /MOTION_INFO", exc_info=True)


def main():
    # type: () -> None
    parser = argparse.ArgumentParser(description="M20 Mac Bridge (GOS-side TCP server)")
    parser.add_argument("--port", type=int, default=9731, help="TCP listen port")
    parser.add_argument("--node-name", type=str, default="dimos_mac_bridge", help="ROS node name")
    args = parser.parse_args()

    bridge = M20MacBridge(port=args.port, ros_node_name=args.node_name)

    def _signal_handler(signum, frame):
        logger.info("Received signal %d, shutting down...", signum)
        bridge.stop()

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    logger.info("Starting M20 Mac Bridge on port %d", args.port)
    bridge.start()


if __name__ == "__main__":
    main()
