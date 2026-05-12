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

"""Tests for the Unity simulator bridge module.

Markers:
    - No special markers needed for unit tests (all run on any platform).
    - Tests that launch the actual Unity binary should use:
        @pytest.mark.self_hosted
        @pytest.mark.skipif(platform.system() != "Linux" or platform.machine() not in ("x86_64", "AMD64"),
                            reason="Unity binary requires Linux x86_64")
        @pytest.mark.skipif(not os.environ.get("DISPLAY"), reason="Unity requires a display (X11)")
"""

import math
import os
import platform
import socket
import struct
import threading
import time

import numpy as np
import pytest

from dimos.constants import DEFAULT_THREAD_JOIN_TIMEOUT
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.simulation.unity.module import (
    UnityBridgeConfig,
    UnityBridgeModule,
    _validate_platform,
)
from dimos.utils.ros1 import (
    ROS1Reader,
    ROS1Writer,
    deserialize_compressed_image,
    deserialize_pointcloud2,
    read_header,
    serialize_pose_stamped,
)

_is_linux_x86 = platform.system() == "Linux" and platform.machine() in ("x86_64", "AMD64")
_has_display = bool(os.environ.get("DISPLAY"))


class _MockTransport:
    def __init__(self):
        self._messages = []
        self._subscribers = []

    def publish(self, msg):
        self._messages.append(msg)
        for cb in self._subscribers:
            cb(msg)

    def broadcast(self, _s, msg):
        self.publish(msg)

    def subscribe(self, cb, *_a):
        self._subscribers.append(cb)
        return lambda: self._subscribers.remove(cb)

    def stop(self):
        pass


def _wire(module) -> dict[str, _MockTransport]:
    subscribers = {}
    for name in (
        "odometry",
        "registered_scan",
        "cmd_vel",
        "terrain_map",
        "color_image",
        "semantic_image",
        "camera_info",
    ):
        transport = _MockTransport()
        getattr(module, name).transport = transport
        subscribers[name] = transport
    return subscribers


def _find_free_port() -> int:
    with socket.socket() as s:
        s.bind(("", 0))
        return s.getsockname()[1]


def _build_ros1_pointcloud2(points: np.ndarray, frame_id: str = "map") -> bytes:
    w = ROS1Writer()
    w.u32(0)
    w.time()
    w.string(frame_id)
    n = len(points)
    w.u32(1)
    w.u32(n)
    w.u32(4)
    for i, name in enumerate(["x", "y", "z", "intensity"]):
        w.string(name)
        w.u32(i * 4)
        w.u8(7)
        w.u32(1)
    w.u8(0)
    w.u32(16)
    w.u32(16 * n)
    data = np.column_stack([points, np.zeros(n, dtype=np.float32)]).astype(np.float32).tobytes()
    w.u32(len(data))
    w.raw(data)
    w.u8(1)
    return w.bytes()


def _send_tcp(sock, dest: str, data: bytes):
    d = dest.encode()
    sock.sendall(struct.pack("<I", len(d)) + d + struct.pack("<I", len(data)) + data)


def _recv_tcp(sock) -> tuple[str, bytes]:
    dl = struct.unpack("<I", sock.recv(4))[0]
    d = sock.recv(dl).decode().rstrip("\x00")
    ml = struct.unpack("<I", sock.recv(4))[0]
    buf = b""
    while len(buf) < ml:
        buf += sock.recv(ml - len(buf))
    return d, buf


class TestConfig:
    def test_defaults(self):
        config = UnityBridgeConfig()
        assert config.unity_port == 10000
        assert config.sim_rate == pytest.approx(200.0)

    def test_custom_binary_path(self):
        config = UnityBridgeConfig(unity_binary="/custom/path/Model.x86_64")
        assert config.unity_binary == "/custom/path/Model.x86_64"

    def test_headless_mode(self):
        config = UnityBridgeConfig(headless=True)
        assert config.headless is True


class TestPlatformValidation:
    def test_passes_on_linux_x86(self):
        if _is_linux_x86:
            _validate_platform()  # should not raise
        else:
            pytest.skip("Not on Linux x86_64")

    @pytest.mark.skipif(_is_linux_x86, reason="Only tests rejection on unsupported platforms")
    def test_rejects_unsupported_platform(self):
        with pytest.raises(RuntimeError, match="requires"):
            _validate_platform()


class TestROS1Deserialization:
    def test_pointcloud2_round_trip(self):
        points = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
        data = _build_ros1_pointcloud2(points)
        result = deserialize_pointcloud2(data)
        assert result is not None
        decoded_points, frame_id, _ts = result
        np.testing.assert_allclose(decoded_points, points, atol=1e-5)
        assert frame_id == "map"

    def test_pointcloud2_empty(self):
        points = np.zeros((0, 3), dtype=np.float32)
        data = _build_ros1_pointcloud2(points)
        result = deserialize_pointcloud2(data)
        assert result is not None
        decoded_points, _, _ = result
        assert len(decoded_points) == 0

    def test_pointcloud2_truncated(self):
        points = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
        data = _build_ros1_pointcloud2(points)
        assert deserialize_pointcloud2(data[:10]) is None

    def test_pointcloud2_garbage(self):
        assert deserialize_pointcloud2(b"\xff\x00\x01\x02") is None

    def test_compressed_image_truncated(self):
        assert deserialize_compressed_image(b"\x03\x00") is None

    def test_serialize_pose_stamped_round_trip(self):
        data = serialize_pose_stamped(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0, frame_id="odom")
        r = ROS1Reader(data)
        header = read_header(r)
        assert header.frame_id == "odom"
        assert r.f64() == pytest.approx(1.0)
        assert r.f64() == pytest.approx(2.0)
        assert r.f64() == pytest.approx(3.0)
        assert r.f64() == pytest.approx(0.0)  # qx
        assert r.f64() == pytest.approx(0.0)  # qy
        assert r.f64() == pytest.approx(0.0)  # qz
        assert r.f64() == pytest.approx(1.0)  # qw


class TestTCPBridge:
    def test_handshake_and_data_flow(self):
        """Mock Unity connects, sends a PointCloud2, verifies bridge publishes it."""
        port = _find_free_port()
        module = UnityBridgeModule(unity_binary="", unity_port=port)
        subscribers = _wire(module)

        module._running.set()
        module._unity_thread = threading.Thread(target=module._unity_loop, daemon=True)
        module._unity_thread.start()
        time.sleep(0.3)

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.connect(("127.0.0.1", port))

            dest, data = _recv_tcp(sock)
            assert dest == "__handshake"

            points = np.array([[10.0, 20.0, 30.0]], dtype=np.float32)
            _send_tcp(sock, "/registered_scan", _build_ros1_pointcloud2(points))
            time.sleep(0.3)
        finally:
            module._running.clear()
            sock.close()
            module._unity_thread.join(timeout=DEFAULT_THREAD_JOIN_TIMEOUT)
            module.stop()

        assert len(subscribers["registered_scan"]._messages) >= 1
        received_points, _ = subscribers["registered_scan"]._messages[0].as_numpy()
        np.testing.assert_allclose(received_points, points, atol=0.01)


class TestKinematicSim:
    def test_odometry_published(self):
        module = UnityBridgeModule(unity_binary="", sim_rate=100.0)
        subscribers = _wire(module)
        dt_sec = 1.0 / module.config.sim_rate

        for _ in range(10):
            module._sim_step(dt_sec)
        module.stop()

        assert len(subscribers["odometry"]._messages) == 10
        assert subscribers["odometry"]._messages[0].frame_id == "map"

    def test_cmd_vel_moves_robot(self):
        module = UnityBridgeModule(unity_binary="", sim_rate=200.0)
        subscribers = _wire(module)
        dt_sec = 1.0 / module.config.sim_rate

        module._on_cmd_vel(Twist(linear=[1.0, 0.0, 0.0], angular=[0.0, 0.0, 0.0]))
        # 200 steps at dt_sec=0.005s with fwd=1.0 module/s → 200 * 0.005 * 1.0 = 1.0m
        for _ in range(200):
            module._sim_step(dt_sec)
        module.stop()

        last_odom = subscribers["odometry"]._messages[-1]
        assert last_odom.x == pytest.approx(1.0, abs=0.01)


class TestTerrainFit:
    def _feed_terrain(self, module, points):
        cloud = PointCloud2.from_numpy(points.astype(np.float32), frame_id="map", timestamp=0.0)
        module._on_terrain(cloud)

    def test_flat_terrain_returns_zero_tilt(self):
        module = UnityBridgeModule(
            unity_binary="", terrain_inclination_enabled=True, terrain_fit_min_inliers=100
        )
        _wire(module)
        # 30x30 grid of ground points (900) around origin at z=0
        g = np.linspace(-1.0, 1.0, 30)
        xx, yy = np.meshgrid(g, g)
        points = np.column_stack([xx.ravel(), yy.ravel(), np.zeros(xx.size)])
        self._feed_terrain(module, points)
        module.stop()
        assert abs(module._terrain_roll) < 0.01
        assert abs(module._terrain_pitch) < 0.01

    def test_sloped_terrain_returns_positive_pitch(self):
        # Plane tilted along +x (forward slope down): z = -slope * x
        slope = 0.1  # ~5.7 degrees
        module = UnityBridgeModule(
            unity_binary="",
            terrain_inclination_enabled=True,
            terrain_fit_min_inliers=100,
            terrain_ground_band=5.0,  # wide band so sloped points qualify
            inclination_smooth_rate=1.0,  # single-step update for predictable test
        )
        _wire(module)
        g = np.linspace(-1.0, 1.0, 30)
        xx, yy = np.meshgrid(g, g)
        zz = -slope * xx
        points = np.column_stack([xx.ravel(), yy.ravel(), zz.ravel()])
        # Pre-set terrain_z to match mean
        module._terrain_z = 0.0
        self._feed_terrain(module, points)
        module.stop()
        # Fit solves: pitch*(-x) + roll*y = z - z_mean = -slope*x
        # so pitch = slope (positive), roll ≈ 0.
        assert module._terrain_pitch == pytest.approx(slope, abs=0.01)
        assert abs(module._terrain_roll) < 0.01

    def test_insufficient_inliers_no_update(self):
        module = UnityBridgeModule(
            unity_binary="",
            terrain_inclination_enabled=True,
            terrain_fit_min_inliers=500,
        )
        _wire(module)
        # Only 4 ground points — below min_inliers=500
        points = np.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [0.0, 0.1, 0.0], [0.1, 0.1, 0.0]])
        module._terrain_roll = 0.05
        module._terrain_pitch = 0.05
        self._feed_terrain(module, points)
        module.stop()
        # Values unchanged
        assert module._terrain_roll == pytest.approx(0.05)
        assert module._terrain_pitch == pytest.approx(0.05)

    def test_disabled_by_default(self):
        module = UnityBridgeModule(unity_binary="")
        _wire(module)
        assert module.config.terrain_inclination_enabled is False
        # Feed a sloped terrain — tilt should stay at 0
        g = np.linspace(-1.0, 1.0, 30)
        xx, yy = np.meshgrid(g, g)
        points = np.column_stack([xx.ravel(), yy.ravel(), (-0.1 * xx).ravel()])
        self._feed_terrain(module, points)
        module.stop()
        assert module._terrain_roll == pytest.approx(0.0)
        assert module._terrain_pitch == pytest.approx(0.0)


class TestSensorOffset:
    def test_zero_offset_matches_old_behavior(self):
        module = UnityBridgeModule(
            unity_binary="", sim_rate=200.0, sensor_offset_x=0.0, sensor_offset_y=0.0
        )
        _wire(module)
        dt_sec = 1.0 / module.config.sim_rate
        module._on_cmd_vel(Twist(linear=[1.0, 0.0, 0.0], angular=[0.0, 0.0, 0.0]))
        for _ in range(200):
            module._sim_step(dt_sec)
        module.stop()
        assert module._x == pytest.approx(1.0, abs=0.01)
        assert module._y == pytest.approx(0.0, abs=0.01)

    def test_pure_yaw_with_offset_displaces_position(self):
        # With sensor_offset_x=0.5 and pure yaw rotation, the sensor origin
        # traces a circle of radius 0.5 around the vehicle center.
        module = UnityBridgeModule(
            unity_binary="", sim_rate=200.0, sensor_offset_x=0.5, sensor_offset_y=0.0
        )
        _wire(module)
        dt_sec = 1.0 / module.config.sim_rate
        module._on_cmd_vel(Twist(linear=[0.0, 0.0, 0.0], angular=[0.0, 0.0, 1.0]))  # 1 rad/s yaw
        # Quarter turn: π/2 radians → π/2 seconds → 0.5π/dt_sec steps
        steps = int((math.pi / 2.0) / dt_sec)
        for _ in range(steps):
            module._sim_step(dt_sec)
        module.stop()
        # Yaw should be ~π/2
        assert module._yaw == pytest.approx(math.pi / 2.0, abs=0.02)
        # Displacement happened (sensor traversed a quarter-circle of radius 0.5).
        assert abs(module._x - 0.0) > 0.01 or abs(module._y - 0.0) > 0.01

    def test_yaw_rate_roll_published(self):
        # After enabling terrain fit with zero tilt, angular roll/pitch rates
        # in published twist should be ~0.
        module = UnityBridgeModule(
            unity_binary="", sim_rate=100.0, terrain_inclination_enabled=False
        )
        subscribers = _wire(module)
        dt_sec = 1.0 / module.config.sim_rate
        for _ in range(5):
            module._sim_step(dt_sec)
        module.stop()
        last = subscribers["odometry"]._messages[-1]
        # Angular rates (from Odometry.twist) should include roll/pitch deltas; at zero tilt they're 0.
        assert last.twist.angular.x == pytest.approx(0.0, abs=1e-6)
        assert last.twist.angular.y == pytest.approx(0.0, abs=1e-6)


@pytest.mark.skipif(not _is_linux_x86, reason="Unity binary requires Linux x86_64")
@pytest.mark.skipif(not _has_display, reason="Unity requires DISPLAY (X11)")
class TestLiveUnity:
    """Tests that launch the actual Unity binary. Skipped unless on Linux x86_64 with a display."""

    def test_unity_connects_and_streams(self):
        """Launch Unity, verify it connects and sends lidar + images."""
        module = UnityBridgeModule()  # uses auto-download
        subscribers = _wire(module)

        module.start()
        time.sleep(25)

        assert module._unity_connected, "Unity did not connect"
        assert len(subscribers["registered_scan"]._messages) > 5, "No lidar from Unity"
        assert len(subscribers["color_image"]._messages) > 5, "No camera images from Unity"
        assert len(subscribers["odometry"]._messages) > 100, "No odometry"

        module.stop()
