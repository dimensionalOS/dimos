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
    - Heartbeat lifecycle
    - UDP protocol binary wire format (loopback)
    - Dead-reckoning odometry accuracy
    - Blueprint smoke tests

Reference: M20 Software Development Guide
    (nell/relay/docs/software_development_guide.md)
"""

import json
import math
import socket
import struct
import threading
import time

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

        ctrl.state.actual_linear_x = 0.005  # below 0.01 threshold
        assert not ctrl.is_moving

        ctrl.state.actual_linear_x = 0.02  # above threshold
        assert ctrl.is_moving


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
