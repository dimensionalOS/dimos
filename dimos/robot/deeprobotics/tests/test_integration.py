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
    - Heartbeat lifecycle
    - UDP protocol binary wire format (loopback)

Reference: M20 Software Development Guide, di-cetts architecture fixes
"""

from enum import IntEnum
import json
import socket
import struct
import time

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

    def test_agile_gait_wire_format(self):
        """Agile gait values (0x3002, 0x3003) encode correctly for Navigation Mode."""
        proto, recv = self._loopback_pair()
        try:
            proto.send_gait_switch(GaitType.AGILE_FLAT)
            data, _ = recv.recvfrom(65536)

            payload_len = struct.unpack("<H", data[4:6])[0]
            payload = json.loads(data[HEADER_LEN : HEADER_LEN + payload_len])
            items = payload["PatrolDevice"]["Items"]
            assert items["GaitParam"] == 0x3002
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
