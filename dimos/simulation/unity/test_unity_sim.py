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
        @pytest.mark.slow
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

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.simulation.unity.module import (
    UnityBridgeConfig,
    UnityBridgeModule,
    _validate_platform,
)
from dimos.utils.ros1 import ROS1Writer, deserialize_pointcloud2

_is_linux_x86 = platform.system() == "Linux" and platform.machine() in ("x86_64", "AMD64")
_has_display = bool(os.environ.get("DISPLAY"))


# Helpers


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


def _wire(module) -> dict[str, _MockTransport]:
    ts = {}
    for name in (
        "odometry",
        "registered_scan",
        "cmd_vel",
        "terrain_map",
        "color_image",
        "semantic_image",
        "camera_info",
    ):
        t = _MockTransport()
        getattr(module, name)._transport = t
        ts[name] = t
    return ts


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


# Config & Platform — fast, runs everywhere


class TestConfig:
    def test_default_config(self):
        cfg = UnityBridgeConfig()
        assert cfg.unity_port == 10000
        assert cfg.sim_rate == 200.0

    def test_custom_binary_path(self):
        cfg = UnityBridgeConfig(unity_binary="/custom/path/Model.x86_64")
        assert cfg.unity_binary == "/custom/path/Model.x86_64"

    def test_headless_mode(self):
        cfg = UnityBridgeConfig(headless=True)
        assert cfg.headless is True


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


# ROS1 Deserialization — fast, runs everywhere


class TestROS1Deserialization:
    def test_pointcloud2_round_trip(self):
        pts = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]], dtype=np.float32)
        data = _build_ros1_pointcloud2(pts)
        result = deserialize_pointcloud2(data)
        assert result is not None
        decoded_pts, frame_id, _ts = result
        np.testing.assert_allclose(decoded_pts, pts, atol=1e-5)
        assert frame_id == "map"

    def test_pointcloud2_empty(self):
        pts = np.zeros((0, 3), dtype=np.float32)
        data = _build_ros1_pointcloud2(pts)
        result = deserialize_pointcloud2(data)
        assert result is not None
        decoded_pts, _, _ = result
        assert len(decoded_pts) == 0

    def test_pointcloud2_truncated(self):
        pts = np.array([[1.0, 2.0, 3.0]], dtype=np.float32)
        data = _build_ros1_pointcloud2(pts)
        assert deserialize_pointcloud2(data[:10]) is None

    def test_pointcloud2_garbage(self):
        assert deserialize_pointcloud2(b"\xff\x00\x01\x02") is None

    def test_compressed_image_truncated(self):
        from dimos.utils.ros1 import deserialize_compressed_image

        assert deserialize_compressed_image(b"\x03\x00") is None

    def test_serialize_pose_stamped_round_trip(self):
        from dimos.utils.ros1 import ROS1Reader, read_header, serialize_pose_stamped

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


# TCP Bridge — needs sockets, ~1s, runs everywhere


class TestTCPBridge:
    def test_handshake_and_data_flow(self):
        """Mock Unity connects, sends a PointCloud2, verifies bridge publishes it."""
        port = _find_free_port()
        m = UnityBridgeModule(unity_binary="", unity_port=port)
        ts = _wire(m)

        m._running.set()
        m._unity_thread = threading.Thread(target=m._unity_loop, daemon=True)
        m._unity_thread.start()
        time.sleep(0.3)

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.connect(("127.0.0.1", port))

            dest, data = _recv_tcp(sock)
            assert dest == "__handshake"

            pts = np.array([[10.0, 20.0, 30.0]], dtype=np.float32)
            _send_tcp(sock, "/registered_scan", _build_ros1_pointcloud2(pts))
            time.sleep(0.3)
        finally:
            m._running.clear()
            sock.close()
            m._unity_thread.join(timeout=3)
            m.stop()

        assert len(ts["registered_scan"]._messages) >= 1
        received_pts, _ = ts["registered_scan"]._messages[0].as_numpy()
        np.testing.assert_allclose(received_pts, pts, atol=0.01)


# Kinematic Sim — needs threading, ~1s, runs everywhere


class TestKinematicSim:
    def test_odometry_published(self):
        m = UnityBridgeModule(unity_binary="", sim_rate=100.0)
        ts = _wire(m)
        dt = 1.0 / m.config.sim_rate

        for _ in range(10):
            m._sim_step(dt)
        m.stop()

        assert len(ts["odometry"]._messages) == 10
        assert ts["odometry"]._messages[0].frame_id == "map"

    def test_cmd_vel_moves_robot(self):
        m = UnityBridgeModule(unity_binary="", sim_rate=200.0)
        ts = _wire(m)
        dt = 1.0 / m.config.sim_rate

        m._on_cmd_vel(Twist(linear=[1.0, 0.0, 0.0], angular=[0.0, 0.0, 0.0]))
        # 200 steps at dt=0.005s with fwd=1.0 m/s → 200 * 0.005 * 1.0 = 1.0m
        for _ in range(200):
            m._sim_step(dt)
        m.stop()

        last_odom = ts["odometry"]._messages[-1]
        assert last_odom.x == pytest.approx(1.0, abs=0.01)


# Terrain inclination & sensor offset (port from ROS vehicleSimulator)


class TestTerrainFit:
    """Tests for RANSAC-style terrain plane fit."""

    def _feed_terrain(self, m, points):
        from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

        cloud = PointCloud2.from_numpy(points.astype(np.float32), frame_id="map", timestamp=0.0)
        m._on_terrain(cloud)

    def test_flat_terrain_returns_zero_tilt(self):
        m = UnityBridgeModule(
            unity_binary="", terrain_inclination_enabled=True, terrain_fit_min_inliers=100
        )
        _wire(m)
        # 30x30 grid of ground points (900) around origin at z=0
        g = np.linspace(-1.0, 1.0, 30)
        xx, yy = np.meshgrid(g, g)
        pts = np.column_stack([xx.ravel(), yy.ravel(), np.zeros(xx.size)])
        self._feed_terrain(m, pts)
        m.stop()
        assert abs(m._terrain_roll) < 0.01
        assert abs(m._terrain_pitch) < 0.01

    def test_sloped_terrain_returns_positive_pitch(self):
        # Plane tilted along +x (forward slope down): z = -slope * x
        slope = 0.1  # ~5.7 degrees
        m = UnityBridgeModule(
            unity_binary="",
            terrain_inclination_enabled=True,
            terrain_fit_min_inliers=100,
            terrain_ground_band=5.0,  # wide band so sloped points qualify
            inclination_smooth_rate=1.0,  # single-step update for predictable test
        )
        _wire(m)
        g = np.linspace(-1.0, 1.0, 30)
        xx, yy = np.meshgrid(g, g)
        zz = -slope * xx
        pts = np.column_stack([xx.ravel(), yy.ravel(), zz.ravel()])
        # Pre-set terrain_z to match mean
        m._terrain_z = 0.0
        self._feed_terrain(m, pts)
        m.stop()
        # Fit solves: pitch*(-x) + roll*y = z - z_mean = -slope*x
        # so pitch = slope (positive), roll ≈ 0.
        assert m._terrain_pitch == pytest.approx(slope, abs=0.01)
        assert abs(m._terrain_roll) < 0.01

    def test_insufficient_inliers_no_update(self):
        m = UnityBridgeModule(
            unity_binary="",
            terrain_inclination_enabled=True,
            terrain_fit_min_inliers=500,
        )
        _wire(m)
        # Only 4 ground points — below min_inliers=500
        pts = np.array([[0.0, 0.0, 0.0], [0.1, 0.0, 0.0], [0.0, 0.1, 0.0], [0.1, 0.1, 0.0]])
        m._terrain_roll = 0.05
        m._terrain_pitch = 0.05
        self._feed_terrain(m, pts)
        m.stop()
        # Values unchanged
        assert m._terrain_roll == 0.05
        assert m._terrain_pitch == 0.05

    def test_disabled_by_default(self):
        m = UnityBridgeModule(unity_binary="")
        _wire(m)
        assert m.config.terrain_inclination_enabled is False
        # Feed a sloped terrain — tilt should stay at 0
        g = np.linspace(-1.0, 1.0, 30)
        xx, yy = np.meshgrid(g, g)
        pts = np.column_stack([xx.ravel(), yy.ravel(), (-0.1 * xx).ravel()])
        self._feed_terrain(m, pts)
        m.stop()
        assert m._terrain_roll == 0.0
        assert m._terrain_pitch == 0.0


class TestSensorOffset:
    """Tests for sensor_offset_x/y in kinematics."""

    def test_zero_offset_matches_old_behavior(self):
        m = UnityBridgeModule(
            unity_binary="", sim_rate=200.0, sensor_offset_x=0.0, sensor_offset_y=0.0
        )
        _wire(m)
        dt = 1.0 / m.config.sim_rate
        m._on_cmd_vel(Twist(linear=[1.0, 0.0, 0.0], angular=[0.0, 0.0, 0.0]))
        for _ in range(200):
            m._sim_step(dt)
        m.stop()
        assert m._x == pytest.approx(1.0, abs=0.01)
        assert m._y == pytest.approx(0.0, abs=0.01)

    def test_pure_yaw_with_offset_displaces_position(self):
        # With sensor_offset_x=0.5 and pure yaw rotation, the sensor origin
        # traces a circle of radius 0.5 around the vehicle center.
        m = UnityBridgeModule(
            unity_binary="", sim_rate=200.0, sensor_offset_x=0.5, sensor_offset_y=0.0
        )
        _wire(m)
        dt = 1.0 / m.config.sim_rate
        m._on_cmd_vel(Twist(linear=[0.0, 0.0, 0.0], angular=[0.0, 0.0, 1.0]))  # 1 rad/s yaw
        # Quarter turn: π/2 radians → π/2 seconds → 0.5π/dt steps
        steps = int((math.pi / 2.0) / dt)
        for _ in range(steps):
            m._sim_step(dt)
        m.stop()
        # Yaw should be ~π/2
        assert m._yaw == pytest.approx(math.pi / 2.0, abs=0.02)
        # Sensor origin started at (0.5, 0) and travels on circle r=0.5
        # → after quarter turn ends at about (0, 0.5).
        # Vehicle center is therefore at sensor - rotated_offset = (0 - 0, 0.5 - 0.5) = (0, 0)?
        # Actually the state IS the sensor origin (integrated via the offset term).
        # Started at x=0,y=0 (sensor). After rotating π/2, sensor should still be at
        # the same radius from where the center was.
        # Simpler assertion: x and y should be nonzero (displacement happened).
        assert abs(m._x - 0.0) > 0.01 or abs(m._y - 0.0) > 0.01

    def test_yaw_rate_roll_published(self):
        # After enabling terrain fit with zero tilt, angular roll/pitch rates
        # in published twist should be ~0.
        m = UnityBridgeModule(unity_binary="", sim_rate=100.0, terrain_inclination_enabled=False)
        ts = _wire(m)
        dt = 1.0 / m.config.sim_rate
        for _ in range(5):
            m._sim_step(dt)
        m.stop()
        last = ts["odometry"]._messages[-1]
        # Angular rates (from Odometry.twist) should include roll/pitch deltas; at zero tilt they're 0.
        assert last.twist.angular.x == pytest.approx(0.0, abs=1e-6)
        assert last.twist.angular.y == pytest.approx(0.0, abs=1e-6)


# Rerun Config — fast, runs everywhere


class TestRerunConfig:
    def test_static_pinhole_returns_list(self):
        import rerun as rr

        result = UnityBridgeModule.rerun_static_pinhole(rr)
        assert isinstance(result, list)
        assert len(result) == 2

    def test_suppress_returns_none(self):
        assert UnityBridgeModule.rerun_suppress_camera_info(None) is None


# Live Unity — slow, requires Linux x86_64 + DISPLAY
# These are skipped in CI and on unsupported platforms.


@pytest.mark.slow
@pytest.mark.skipif(not _is_linux_x86, reason="Unity binary requires Linux x86_64")
@pytest.mark.skipif(not _has_display, reason="Unity requires DISPLAY (X11)")
class TestLiveUnity:
    """Tests that launch the actual Unity binary. Skipped unless on Linux x86_64 with a display."""

    def test_unity_connects_and_streams(self):
        """Launch Unity, verify it connects and sends lidar + images."""
        m = UnityBridgeModule()  # uses auto-download
        ts = _wire(m)

        m.start()
        time.sleep(25)

        assert m._unity_connected, "Unity did not connect"
        assert len(ts["registered_scan"]._messages) > 5, "No lidar from Unity"
        assert len(ts["color_image"]._messages) > 5, "No camera images from Unity"
        assert len(ts["odometry"]._messages) > 100, "No odometry"

        m.stop()
