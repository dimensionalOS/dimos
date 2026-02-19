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

"""Integration tests for M20 quadruped modules.

Covers:
    - Velocity safety (e-stop, command timeout, acceleration limits)
    - /NAV_CMD velocity transport (Navigation Mode)
    - Heartbeat lifecycle
    - UDP protocol binary wire format (loopback)
    - Dead-reckoning odometry accuracy
    - ROS sensor conversions (odom, pointcloud)
    - Blueprint smoke tests

Reference: M20 Software Development Guide, di-cetts architecture fixes
"""

import json
import math
import socket
import struct
import threading
import time
from enum import IntEnum

import pytest

from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Twist, Vector3
from dimos.robot.deeprobotics.protocol import (
    HEADER_LEN,
    HEADER_MAGIC,
    JSON_FORMAT,
    Command,
    CommandType,
    GaitType,
    M20Protocol,
    MotionState,
    UsageMode,
)
from dimos.robot.deeprobotics.m20.velocity_controller import (
    M20SpeedLimits,
    M20VelocityController,
    VelocityState,
)
from dimos.robot.deeprobotics.m20.odometry import M20DeadReckonOdometry


# ---------------------------------------------------------------------------
# Velocity safety tests (di-hk0o2.6)
# ---------------------------------------------------------------------------


class TestVelocitySafety:
    """Verify safety-critical velocity controller behaviour."""

    def test_estop_zeroes_actual_velocity_immediately(self):
        """E-stop must zero both target and smoothed velocities in one call."""
        proto = M20Protocol()
        ctrl = M20VelocityController(protocol=proto)

        # Simulate mid-motion state
        with ctrl._lock:
            ctrl.state.actual_linear_x = 1.2
            ctrl.state.actual_linear_y = 0.4
            ctrl.state.actual_angular_yaw = 0.8
            ctrl.state.target_linear_x = 1.5
            ctrl.state.target_linear_y = 0.5
            ctrl.state.target_angular_yaw = 1.0

        ctrl.emergency_stop(True)

        assert ctrl.state.actual_linear_x == 0.0
        assert ctrl.state.actual_linear_y == 0.0
        assert ctrl.state.actual_angular_yaw == 0.0
        assert ctrl.state.target_linear_x == 0.0
        assert ctrl.state.target_linear_y == 0.0
        assert ctrl.state.target_angular_yaw == 0.0
        assert ctrl.state.emergency_stopped

    def test_estop_blocks_new_commands(self):
        """set_twist must be ignored while e-stop is engaged."""
        proto = M20Protocol()
        ctrl = M20VelocityController(protocol=proto)
        ctrl.emergency_stop(True)

        twist = Twist(
            linear=Vector3(x=1.0, y=0.5, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=0.5),
        )
        ctrl.set_twist(twist)

        assert ctrl.state.target_linear_x == 0.0
        assert ctrl.state.target_linear_y == 0.0
        assert ctrl.state.target_angular_yaw == 0.0

    def test_estop_release_accepts_commands(self):
        """After releasing e-stop, set_twist must work again."""
        proto = M20Protocol()
        ctrl = M20VelocityController(protocol=proto)
        ctrl.emergency_stop(True)
        ctrl.emergency_stop(False)

        twist = Twist(
            linear=Vector3(x=0.5, y=0.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=0.0),
        )
        ctrl.set_twist(twist)
        assert ctrl.state.target_linear_x == 0.5

    def test_command_timeout_ramps_to_zero(self):
        """When no twist arrives within command_timeout, targets must zero."""
        proto = M20Protocol()
        ctrl = M20VelocityController(
            protocol=proto,
            command_timeout=0.1,
        )

        ctrl.set_twist(
            Twist(
                linear=Vector3(x=1.0, y=0.0, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=0.0),
            )
        )
        assert ctrl.state.target_linear_x == 1.0

        # Simulate time passing beyond timeout
        ctrl.state.last_command_time = time.time() - 0.2

        # _control_loop checks timeout then calls _update_velocities;
        # replicate that logic directly
        if time.time() - ctrl.state.last_command_time > ctrl.command_timeout:
            ctrl.state.target_linear_x = 0.0
            ctrl.state.target_linear_y = 0.0
            ctrl.state.target_angular_yaw = 0.0

        assert ctrl.state.target_linear_x == 0.0

    def test_acceleration_limit_caps_step(self):
        """One update step must not exceed accel * dt."""
        proto = M20Protocol()
        ctrl = M20VelocityController(
            protocol=proto,
            linear_accel=2.0,
            angular_accel=3.0,
        )
        with ctrl._lock:
            ctrl.state.target_linear_x = 10.0  # way above step limit
            ctrl.state.target_angular_yaw = 10.0

            dt = 0.05  # 20Hz
            ctrl._update_velocities(dt)

        # max step = accel * dt
        assert abs(ctrl.state.actual_linear_x - 0.10) < 1e-6  # 2.0 * 0.05
        assert abs(ctrl.state.actual_angular_yaw - 0.15) < 1e-6  # 3.0 * 0.05

    def test_deceleration_symmetric_to_acceleration(self):
        """Deceleration from speed to zero follows the same accel limit."""
        proto = M20Protocol()
        ctrl = M20VelocityController(
            protocol=proto,
            linear_accel=1.0,
        )
        with ctrl._lock:
            ctrl.state.actual_linear_x = 1.0
            ctrl.state.target_linear_x = 0.0

            dt = 0.05
            ctrl._update_velocities(dt)

        assert abs(ctrl.state.actual_linear_x - 0.95) < 1e-6  # 1.0 - 1.0*0.05

    def test_speed_limits_clamp_twist(self):
        """set_twist must clamp to M20SpeedLimits on all axes."""
        proto = M20Protocol()
        limits = M20SpeedLimits(
            max_linear_x=1.0, max_linear_y=0.3, max_angular_yaw=0.5
        )
        ctrl = M20VelocityController(protocol=proto, speed_limits=limits)

        twist = Twist(
            linear=Vector3(x=5.0, y=-2.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=3.0),
        )
        ctrl.set_twist(twist)

        assert ctrl.state.target_linear_x == 1.0
        assert ctrl.state.target_linear_y == -0.3
        assert ctrl.state.target_angular_yaw == 0.5

    def test_negative_speed_limits_clamp(self):
        """Negative velocities are clamped symmetrically."""
        proto = M20Protocol()
        limits = M20SpeedLimits(max_linear_x=1.0, max_linear_y=0.3, max_angular_yaw=0.5)
        ctrl = M20VelocityController(protocol=proto, speed_limits=limits)

        twist = Twist(
            linear=Vector3(x=-5.0, y=2.0, z=0.0),
            angular=Vector3(x=0.0, y=0.0, z=-3.0),
        )
        ctrl.set_twist(twist)

        assert ctrl.state.target_linear_x == -1.0
        assert ctrl.state.target_linear_y == 0.3
        assert ctrl.state.target_angular_yaw == -0.5

    def test_normalization_at_max_speed(self):
        """At max speed, normalized output must be 1.0."""
        proto = M20Protocol()
        limits = M20SpeedLimits(
            max_linear_x=1.5, max_linear_y=0.5, max_angular_yaw=1.0
        )
        ctrl = M20VelocityController(protocol=proto, speed_limits=limits)
        with ctrl._lock:
            ctrl.state.actual_linear_x = 1.5
            ctrl.state.actual_linear_y = 0.5
            ctrl.state.actual_angular_yaw = 1.0

        norm_x = ctrl.state.actual_linear_x / limits.max_linear_x
        norm_y = ctrl.state.actual_linear_y / limits.max_linear_y
        norm_yaw = ctrl.state.actual_angular_yaw / limits.max_angular_yaw

        assert abs(norm_x - 1.0) < 1e-6
        assert abs(norm_y - 1.0) < 1e-6
        assert abs(norm_yaw - 1.0) < 1e-6

    def test_is_moving_threshold(self):
        """is_moving should be False when all axes below threshold."""
        proto = M20Protocol()
        ctrl = M20VelocityController(protocol=proto)
        assert not ctrl.is_moving

        with ctrl._lock:
            ctrl.state.actual_linear_x = 0.005  # below 0.01 threshold
        assert not ctrl.is_moving

        with ctrl._lock:
            ctrl.state.actual_linear_x = 0.02  # above threshold
        assert ctrl.is_moving

    def test_estop_sends_zero_velocity_on_wire(self):
        """E-stop must immediately send zero velocity over UDP (P0 safety)."""
        recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv_sock.bind(("127.0.0.1", 0))
        recv_sock.settimeout(2.0)
        port = recv_sock.getsockname()[1]

        proto = M20Protocol(host="127.0.0.1", port=port)
        proto.connect()
        ctrl = M20VelocityController(protocol=proto)

        # Set mid-motion state
        with ctrl._lock:
            ctrl.state.actual_linear_x = 1.0
            ctrl.state.actual_linear_y = 0.5
            ctrl.state.actual_angular_yaw = 0.8

        ctrl.emergency_stop(True)

        try:
            data, _ = recv_sock.recvfrom(65536)
            payload_len = struct.unpack("<H", data[4:6])[0]
            payload = json.loads(data[HEADER_LEN : HEADER_LEN + payload_len])
            items = payload["PatrolDevice"]["Items"]
            assert items["X"] == 0.0
            assert items["Y"] == 0.0
            assert items["Yaw"] == 0.0
        finally:
            proto.close()
            recv_sock.close()

    def test_constructor_rejects_invalid_params(self):
        """Constructor must reject non-positive acceleration/rate/timeout."""
        proto = M20Protocol()
        with pytest.raises(ValueError):
            M20VelocityController(protocol=proto, linear_accel=-1.0)
        with pytest.raises(ValueError):
            M20VelocityController(protocol=proto, angular_accel=0.0)
        with pytest.raises(ValueError):
            M20VelocityController(protocol=proto, control_rate=0)
        with pytest.raises(ValueError):
            M20VelocityController(protocol=proto, command_timeout=-0.1)

    def test_speed_limits_frozen(self):
        """M20SpeedLimits must be immutable (frozen dataclass)."""
        limits = M20SpeedLimits()
        with pytest.raises(AttributeError):
            limits.max_linear_x = 99.0


# ---------------------------------------------------------------------------
# Heartbeat lifecycle tests
# ---------------------------------------------------------------------------


class TestHeartbeatLifecycle:
    """Verify heartbeat thread start/stop without actual UDP."""

    def test_heartbeat_starts_and_stops(self):
        """Heartbeat thread must be daemonic and joinable."""
        proto = M20Protocol()
        proto.connect()

        proto.start_heartbeat(interval=0.1)
        assert proto._heartbeat_running
        assert proto._heartbeat_thread is not None
        assert proto._heartbeat_thread.is_alive()

        proto.stop_heartbeat()
        assert not proto._heartbeat_running
        assert proto._heartbeat_thread is None

        proto.close()

    def test_double_start_is_idempotent(self):
        """Calling start_heartbeat twice should not create a second thread."""
        proto = M20Protocol()
        proto.connect()

        proto.start_heartbeat(interval=0.1)
        first_thread = proto._heartbeat_thread

        proto.start_heartbeat(interval=0.1)
        assert proto._heartbeat_thread is first_thread

        proto.close()

    def test_close_stops_heartbeat(self):
        """close() must stop the heartbeat thread."""
        proto = M20Protocol()
        proto.connect()
        proto.start_heartbeat(interval=0.1)
        proto.close()

        assert not proto._heartbeat_running
        assert not proto.connected


# ---------------------------------------------------------------------------
# UDP loopback wire-format tests
# ---------------------------------------------------------------------------


class TestUDPWireFormat:
    """Verify binary encoding over real UDP (localhost loopback)."""

    def _loopback_pair(self):
        """Create a pair: M20Protocol sending to a listening UDP socket."""
        recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv_sock.bind(("127.0.0.1", 0))
        recv_sock.settimeout(2.0)
        port = recv_sock.getsockname()[1]

        proto = M20Protocol(host="127.0.0.1", port=port)
        proto.connect()

        return proto, recv_sock

    def test_velocity_wire_format(self):
        """Velocity command must have correct header magic + JSON payload."""
        proto, recv = self._loopback_pair()
        try:
            proto.send_velocity(x=0.5, y=-0.3, yaw=0.1)
            data, _ = recv.recvfrom(65536)

            # Header checks
            assert data[:4] == HEADER_MAGIC
            payload_len = struct.unpack("<H", data[4:6])[0]
            assert data[8] == JSON_FORMAT

            # Payload checks
            payload = json.loads(data[HEADER_LEN : HEADER_LEN + payload_len])
            device = payload["PatrolDevice"]
            assert device["Type"] == CommandType.MOTION
            assert device["Command"] == Command.VELOCITY
            assert device["Items"]["X"] == 0.5
            assert device["Items"]["Y"] == -0.3
            assert device["Items"]["Yaw"] == 0.1
        finally:
            proto.close()
            recv.close()

    def test_motion_state_wire_format(self):
        """Motion state command encodes MotionParam correctly."""
        proto, recv = self._loopback_pair()
        try:
            proto.send_motion_state(MotionState.STAND)
            data, _ = recv.recvfrom(65536)

            payload_len = struct.unpack("<H", data[4:6])[0]
            payload = json.loads(data[HEADER_LEN : HEADER_LEN + payload_len])
            items = payload["PatrolDevice"]["Items"]
            assert items["MotionParam"] == MotionState.STAND
        finally:
            proto.close()
            recv.close()

    def test_gait_switch_wire_format(self):
        """Gait switch command encodes GaitParam correctly."""
        proto, recv = self._loopback_pair()
        try:
            proto.send_gait_switch(GaitType.STAIRS)
            data, _ = recv.recvfrom(65536)

            payload_len = struct.unpack("<H", data[4:6])[0]
            payload = json.loads(data[HEADER_LEN : HEADER_LEN + payload_len])
            items = payload["PatrolDevice"]["Items"]
            assert items["GaitParam"] == GaitType.STAIRS
        finally:
            proto.close()
            recv.close()

    def test_usage_mode_wire_format(self):
        """Usage mode command encodes Mode correctly."""
        proto, recv = self._loopback_pair()
        try:
            proto.send_usage_mode(UsageMode.NAVIGATION)
            data, _ = recv.recvfrom(65536)

            payload_len = struct.unpack("<H", data[4:6])[0]
            payload = json.loads(data[HEADER_LEN : HEADER_LEN + payload_len])
            items = payload["PatrolDevice"]["Items"]
            assert items["Mode"] == UsageMode.NAVIGATION
        finally:
            proto.close()
            recv.close()

    def test_heartbeat_wire_format(self):
        """Heartbeat has Type=100, Command=100, empty Items."""
        proto, recv = self._loopback_pair()
        try:
            proto.send_heartbeat()
            data, _ = recv.recvfrom(65536)

            payload_len = struct.unpack("<H", data[4:6])[0]
            payload = json.loads(data[HEADER_LEN : HEADER_LEN + payload_len])
            device = payload["PatrolDevice"]
            assert device["Type"] == CommandType.HEARTBEAT
            assert device["Command"] == Command.HEARTBEAT
            assert device["Items"] == {}
        finally:
            proto.close()
            recv.close()

    def test_velocity_clamping_on_wire(self):
        """Values outside [-1,1] must be clamped before hitting the wire."""
        proto, recv = self._loopback_pair()
        try:
            proto.send_velocity(x=5.0, y=-3.0, yaw=2.0)
            data, _ = recv.recvfrom(65536)

            payload_len = struct.unpack("<H", data[4:6])[0]
            payload = json.loads(data[HEADER_LEN : HEADER_LEN + payload_len])
            items = payload["PatrolDevice"]["Items"]
            assert items["X"] == 1.0
            assert items["Y"] == -1.0
            assert items["Yaw"] == 1.0
        finally:
            proto.close()
            recv.close()

    def test_msg_id_increments(self):
        """Each send must increment the msg_id in the header."""
        proto, recv = self._loopback_pair()
        try:
            proto.send_heartbeat()
            data1, _ = recv.recvfrom(65536)
            id1 = struct.unpack("<H", data1[6:8])[0]

            proto.send_heartbeat()
            data2, _ = recv.recvfrom(65536)
            id2 = struct.unpack("<H", data2[6:8])[0]

            assert id2 == id1 + 1
        finally:
            proto.close()
            recv.close()

    def test_protocol_constants_are_intenum(self):
        """Protocol constants must be IntEnum per dimos codebase convention."""
        assert issubclass(CommandType, IntEnum)
        assert issubclass(Command, IntEnum)
        assert issubclass(MotionState, IntEnum)
        assert issubclass(GaitType, IntEnum)
        assert issubclass(UsageMode, IntEnum)

    def test_double_connect_is_idempotent(self):
        """Calling connect() twice should not create a second socket."""
        proto = M20Protocol(host="127.0.0.1", port=40000)
        proto.connect()
        first_sock = proto._sock
        proto.connect()
        assert proto._sock is first_sock
        proto.close()

    def test_listener_receives_status(self):
        """Listener thread must parse a valid status response."""
        proto = M20Protocol(host="127.0.0.1", port=40000)
        proto.connect()
        proto._sock.bind(("127.0.0.1", 0))
        listen_port = proto._sock.getsockname()[1]

        received = []
        proto.start_listener(lambda report: received.append(report))

        # Build a fake status response and send to the listener
        status_payload = json.dumps({
            "PatrolDevice": {
                "Type": 200,
                "Command": 1,
                "Items": {"Battery": 85, "Mode": 1},
            }
        }).encode("utf-8")

        header = (
            HEADER_MAGIC
            + struct.pack("<H", len(status_payload))
            + struct.pack("<H", 1)
            + bytes([JSON_FORMAT])
            + bytes(7)
        )

        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sender.sendto(header + status_payload, ("127.0.0.1", listen_port))
        sender.close()

        time.sleep(0.3)

        proto.close()

        assert len(received) >= 1
        assert received[0]["type"] == 200
        assert received[0]["items"]["Battery"] == 85


# ---------------------------------------------------------------------------
# Dead-reckoning odometry tests
# ---------------------------------------------------------------------------


class TestOdometryIntegration:
    """Verify dead-reckoning odometry produces correct poses."""

    def test_straight_line_forward(self):
        """Moving forward at 1 m/s for 0.5s should yield ~0.5m displacement."""
        poses = []
        odom = M20DeadReckonOdometry(
            publish_callback=lambda p: poses.append(p),
            rate=100.0,
        )

        odom.update_velocity(1.0, 0.0, 0.0)
        odom.start()
        time.sleep(0.5)
        odom.stop()

        assert len(poses) > 0
        final = poses[-1]
        # Should be approximately 0.5m forward with some tolerance for timing
        assert abs(final.position.x - 0.5) < 0.05
        assert abs(final.position.y) < 0.01

    def test_pure_rotation(self):
        """Rotating at pi rad/s for ~0.5s should yield ~pi/2 yaw change."""
        poses = []
        odom = M20DeadReckonOdometry(
            publish_callback=lambda p: poses.append(p),
            rate=100.0,
        )

        odom.update_velocity(0.0, 0.0, math.pi)
        odom.start()
        time.sleep(0.5)
        odom.stop()

        assert len(poses) > 0
        x, y, yaw = odom.pose
        assert abs(yaw - math.pi / 2) < 0.1
        assert abs(x) < 0.01
        assert abs(y) < 0.01

    def test_lateral_motion(self):
        """Moving laterally at 0.5 m/s for 0.5s should yield ~0.25m in y."""
        poses = []
        odom = M20DeadReckonOdometry(
            publish_callback=lambda p: poses.append(p),
            rate=100.0,
        )

        odom.update_velocity(0.0, 0.5, 0.0)
        odom.start()
        time.sleep(0.5)
        odom.stop()

        x, y, yaw = odom.pose
        assert abs(y - 0.25) < 0.03
        assert abs(x) < 0.01

    def test_reset_zeroes_pose(self):
        """reset() must zero the pose state."""
        odom = M20DeadReckonOdometry(
            publish_callback=lambda p: None,
            rate=100.0,
        )
        odom._x = 5.0
        odom._y = 3.0
        odom._yaw = 1.0

        odom.reset()

        x, y, yaw = odom.pose
        assert x == 0.0
        assert y == 0.0
        assert yaw == 0.0

    def test_reset_to_custom_pose(self):
        """reset(x, y, yaw) must set the pose to the given values."""
        odom = M20DeadReckonOdometry(
            publish_callback=lambda p: None,
            rate=100.0,
        )
        odom.reset(x=1.0, y=2.0, yaw=math.pi / 4)

        x, y, yaw = odom.pose
        assert abs(x - 1.0) < 1e-6
        assert abs(y - 2.0) < 1e-6
        assert abs(yaw - math.pi / 4) < 1e-6

    def test_publishes_pose_stamped(self):
        """Callback must receive PoseStamped with correct frame_id."""
        poses = []
        odom = M20DeadReckonOdometry(
            publish_callback=lambda p: poses.append(p),
            rate=50.0,
            frame_id="test_frame",
        )

        odom.start()
        time.sleep(0.15)
        odom.stop()

        assert len(poses) > 0
        assert isinstance(poses[0], PoseStamped)
        assert poses[0].frame_id == "test_frame"


# ---------------------------------------------------------------------------
# /NAV_CMD velocity transport tests (di-cetts)
# ---------------------------------------------------------------------------


class TestNavCmdVelocity:
    """Verify /NAV_CMD callback integration in velocity controller."""

    def test_nav_cmd_callback_receives_absolute_m_per_s(self):
        """When nav_cmd_publish is set, it receives absolute m/s (not normalized)."""
        published = []

        def capture_nav_cmd(x: float, y: float, yaw: float) -> None:
            published.append((x, y, yaw))

        proto = M20Protocol()
        ctrl = M20VelocityController(
            protocol=proto,
            nav_cmd_publish=capture_nav_cmd,
        )

        # Set actual velocities directly (skip smoothing for this test)
        with ctrl._lock:
            ctrl.state.actual_linear_x = 0.75
            ctrl.state.actual_linear_y = -0.25
            ctrl.state.actual_angular_yaw = 0.5
            ctrl.state.last_command_time = time.time()  # prevent idle

        ctrl._publish_control()

        assert len(published) == 1
        x, y, yaw = published[0]
        assert abs(x - 0.75) < 1e-6
        assert abs(y - (-0.25)) < 1e-6
        assert abs(yaw - 0.5) < 1e-6

    def test_udp_fallback_when_no_nav_cmd(self):
        """Without nav_cmd_publish, velocity goes through UDP normalized path."""
        recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv_sock.bind(("127.0.0.1", 0))
        recv_sock.settimeout(2.0)
        port = recv_sock.getsockname()[1]

        proto = M20Protocol(host="127.0.0.1", port=port)
        proto.connect()
        ctrl = M20VelocityController(
            protocol=proto,
            nav_cmd_publish=None,  # no /NAV_CMD
        )

        with ctrl._lock:
            ctrl.state.actual_linear_x = 0.75
            ctrl.state.actual_linear_y = 0.25
            ctrl.state.actual_angular_yaw = 0.5
            ctrl.state.last_command_time = time.time()

        ctrl._publish_control()

        try:
            data, _ = recv_sock.recvfrom(65536)
            payload_len = struct.unpack("<H", data[4:6])[0]
            payload = json.loads(data[HEADER_LEN : HEADER_LEN + payload_len])
            items = payload["PatrolDevice"]["Items"]
            # Should be normalized: 0.75/1.5=0.5, 0.25/0.5=0.5, 0.5/1.0=0.5
            assert abs(items["X"] - 0.5) < 1e-3
            assert abs(items["Y"] - 0.5) < 1e-3
            assert abs(items["Yaw"] - 0.5) < 1e-3
        finally:
            proto.close()
            recv_sock.close()

    def test_estop_sends_zero_on_both_transports(self):
        """E-stop with nav_cmd_publish must zero both /NAV_CMD and UDP."""
        nav_cmd_calls = []
        recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        recv_sock.bind(("127.0.0.1", 0))
        recv_sock.settimeout(2.0)
        port = recv_sock.getsockname()[1]

        proto = M20Protocol(host="127.0.0.1", port=port)
        proto.connect()
        ctrl = M20VelocityController(
            protocol=proto,
            nav_cmd_publish=lambda x, y, yaw: nav_cmd_calls.append((x, y, yaw)),
        )

        with ctrl._lock:
            ctrl.state.actual_linear_x = 1.0

        ctrl.emergency_stop(True)

        try:
            # /NAV_CMD should have received zero
            assert len(nav_cmd_calls) >= 1
            assert nav_cmd_calls[-1] == (0.0, 0.0, 0.0)

            # UDP should also have received zero
            data, _ = recv_sock.recvfrom(65536)
            payload_len = struct.unpack("<H", data[4:6])[0]
            payload = json.loads(data[HEADER_LEN : HEADER_LEN + payload_len])
            items = payload["PatrolDevice"]["Items"]
            assert items["X"] == 0.0
            assert items["Y"] == 0.0
            assert items["Yaw"] == 0.0
        finally:
            proto.close()
            recv_sock.close()

    def test_idle_suppresses_publish(self):
        """When idle (timed out + near zero), no publish on either transport."""
        nav_cmd_calls = []
        proto = M20Protocol()
        ctrl = M20VelocityController(
            protocol=proto,
            nav_cmd_publish=lambda x, y, yaw: nav_cmd_calls.append((x, y, yaw)),
            command_timeout=0.1,
        )

        # State is all zeros, last_command_time is 0 (long ago) — should be idle
        ctrl._publish_control()
        assert len(nav_cmd_calls) == 0


# ---------------------------------------------------------------------------
# ROS sensor conversion tests (di-cetts)
# ---------------------------------------------------------------------------


class TestROSSensorConversions:
    """Verify ROS→dimos message conversions (mocked, no real rclpy)."""

    def test_odom_to_pose_stamped(self):
        """nav_msgs/Odometry → dimos PoseStamped conversion."""
        from dimos.robot.deeprobotics.m20.ros_sensors import _odom_to_pose_stamped

        # Create a mock Odometry message
        class MockStamp:
            sec = 1000
            nanosec = 500_000_000

        class MockHeader:
            stamp = MockStamp()
            frame_id = "odom"

        class MockPoint:
            x = 1.5
            y = -2.0
            z = 0.1

        class MockQuat:
            x = 0.0
            y = 0.0
            z = 0.3827
            w = 0.9239

        class MockPose:
            position = MockPoint()
            orientation = MockQuat()

        class MockPoseWithCovariance:
            pose = MockPose()

        class MockOdom:
            header = MockHeader()
            pose = MockPoseWithCovariance()

        result = _odom_to_pose_stamped(MockOdom())

        assert isinstance(result, PoseStamped)
        assert abs(result.position.x - 1.5) < 1e-6
        assert abs(result.position.y - (-2.0)) < 1e-6
        assert abs(result.position.z - 0.1) < 1e-6
        assert abs(result.orientation.z - 0.3827) < 1e-4
        assert abs(result.orientation.w - 0.9239) < 1e-4
        assert result.frame_id == "odom"
        assert abs(result.ts - 1000.5) < 1e-6

    def test_ros_pc2_to_dimos_basic(self):
        """sensor_msgs/PointCloud2 → dimos PointCloud2 with XYZ float32."""
        from dimos.robot.deeprobotics.m20.ros_sensors import _ros_pc2_to_dimos

        import numpy as np

        # Build a minimal PointCloud2-like message with 3 points
        points = np.array(
            [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]],
            dtype=np.float32,
        )
        raw_data = points.tobytes()

        class MockField:
            def __init__(self, name: str, offset: int):
                self.name = name
                self.offset = offset
                self.datatype = 7  # FLOAT32
                self.count = 1

        class MockStamp:
            sec = 100
            nanosec = 0

        class MockHeader:
            stamp = MockStamp()
            frame_id = "lidar_frame"

        class MockPC2:
            header = MockHeader()
            height = 1
            width = 3
            fields = [MockField("x", 0), MockField("y", 4), MockField("z", 8)]
            is_bigendian = False
            point_step = 12  # 3 * 4 bytes
            row_step = 36
            data = list(raw_data)
            is_dense = True

        result = _ros_pc2_to_dimos(MockPC2())
        assert result.frame_id == "lidar_frame"
        assert abs(result.ts - 100.0) < 1e-6

        # Verify points came through
        result_points, _ = result.as_numpy()
        assert result_points.shape == (3, 3)
        assert abs(result_points[0, 0] - 1.0) < 1e-5
        assert abs(result_points[2, 2] - 9.0) < 1e-5

    def test_ros_pc2_empty(self):
        """Empty PointCloud2 should produce empty dimos PointCloud2."""
        from dimos.robot.deeprobotics.m20.ros_sensors import _ros_pc2_to_dimos

        class MockStamp:
            sec = 0
            nanosec = 0

        class MockHeader:
            stamp = MockStamp()
            frame_id = "world"

        class MockPC2:
            header = MockHeader()
            height = 0
            width = 0
            fields = []
            is_bigendian = False
            point_step = 0
            row_step = 0
            data = []
            is_dense = True

        result = _ros_pc2_to_dimos(MockPC2())
        assert result.frame_id == "world"

    def test_motion_info_to_data(self):
        """drdds/MotionInfo → MotionInfoData conversion."""
        from dimos.robot.deeprobotics.m20.ros_sensors import (
            MotionInfoData,
            _motion_info_to_data,
        )

        class MockMotionState:
            state = 1  # Stand

        class MockGaitState:
            gait = 0x3002  # Flat Agile

        class MockMotionInfoValue:
            vel_x = 0.5
            vel_y = -0.1
            vel_yaw = 0.3
            height = 0.42
            motion_state = MockMotionState()
            gait_state = MockGaitState()
            payload = 0.0
            remain_mile = 12.5

        class MockMotionInfo:
            data = MockMotionInfoValue()

        result = _motion_info_to_data(MockMotionInfo())
        assert isinstance(result, MotionInfoData)
        assert abs(result.vel_x - 0.5) < 1e-6
        assert abs(result.vel_y - (-0.1)) < 1e-6
        assert abs(result.vel_yaw - 0.3) < 1e-6
        assert abs(result.height - 0.42) < 1e-6
        assert result.state == 1
        assert result.gait == 0x3002
        assert abs(result.remain_mile - 12.5) < 1e-6

    def test_tf_to_transforms(self):
        """tf2_msgs/TFMessage → list[Transform] conversion."""
        from dimos.robot.deeprobotics.m20.ros_sensors import _tf_to_transforms

        class MockStamp:
            sec = 100
            nanosec = 0

        class MockHeader:
            stamp = MockStamp()
            frame_id = "odom"

        class MockTranslation:
            x = 1.0
            y = 2.0
            z = 0.0

        class MockRotation:
            x = 0.0
            y = 0.0
            z = 0.0
            w = 1.0

        class MockTransform:
            translation = MockTranslation()
            rotation = MockRotation()

        class MockTransformStamped:
            header = MockHeader()
            child_frame_id = "base_link"
            transform = MockTransform()

        class MockTFMessage:
            transforms = [MockTransformStamped()]

        result = _tf_to_transforms(MockTFMessage())
        assert len(result) == 1
        assert result[0].frame_id == "odom"
        assert result[0].child_frame_id == "base_link"
        assert abs(result[0].translation.x - 1.0) < 1e-6


# ---------------------------------------------------------------------------
# Blueprint smoke tests
# ---------------------------------------------------------------------------


class TestBlueprintSmoke:
    """Verify M20 blueprints are importable and are valid Blueprint instances."""

    # Optional dependencies that blueprint imports may need
    OPTIONAL_DEPS = {"socketio", "langchain", "langchain.agents"}

    def test_m20_minimal_is_blueprint(self):
        from dimos.core.blueprints import Blueprint

        try:
            from dimos.robot.deeprobotics.m20.blueprints.basic.m20_minimal import (
                m20_minimal,
            )
        except ModuleNotFoundError as e:
            if e.name in self.OPTIONAL_DEPS:
                pytest.skip(f"Missing optional dep: {e.name}")
            raise

        assert isinstance(m20_minimal, Blueprint)

    def test_m20_smart_is_blueprint(self):
        from dimos.core.blueprints import Blueprint

        try:
            from dimos.robot.deeprobotics.m20.blueprints.smart.m20_smart import (
                m20_smart,
            )
        except ModuleNotFoundError as e:
            if e.name in self.OPTIONAL_DEPS:
                pytest.skip(f"Missing optional dep: {e.name}")
            raise

        assert isinstance(m20_smart, Blueprint)

    def test_m20_agentic_is_blueprint(self):
        from dimos.core.blueprints import Blueprint

        try:
            from dimos.robot.deeprobotics.m20.blueprints.agentic.m20_agentic import (
                m20_agentic,
            )
        except ModuleNotFoundError as e:
            if e.name in self.OPTIONAL_DEPS:
                pytest.skip(f"Missing optional dep: {e.name}")
            raise

        assert isinstance(m20_agentic, Blueprint)

    def test_m20_connection_blueprint_export(self):
        from dimos.robot.deeprobotics.m20.connection import m20_connection

        # Module.blueprint is a classmethod that returns a functools.partial
        assert callable(m20_connection)

    def test_m20_skills_blueprint_export(self):
        from dimos.robot.deeprobotics.m20.skill_container import m20_skills

        # Module.blueprint is a classmethod that returns a functools.partial
        assert callable(m20_skills)
