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

"""Tests for the Lynx M20 connection: command mapping + patrol wire protocol.

These cover the risky parts — the Twist->axis normalization, the deadman, and
the 16-byte header / JSON ASDU framing — without a real robot or the dimos
coordinator. A tiny fake UDP endpoint stands in for the M20.
"""

import json
import socket
import struct
import time
from typing import TypedDict

from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.BatteryState import POWER_SUPPLY_STATUS_DISCHARGING, BatteryState

from .connection import PatrolLink, _apply_deadman, _axes_from_twist, _battery_from, _status_from
from .msgs.status import M20Status


class Asdu(TypedDict):
    """The PatrolDevice ASDU envelope. Items are float-valued for the control
    frames these tests decode (heartbeat = empty, move = normalized axes)."""

    Type: int
    Command: int
    Time: str
    Items: dict[str, float]


def _decode(frame: bytes) -> tuple[bytes, int, int, int, Asdu]:
    """Decode a patrol frame into (sync, asdu_len, msg_id, fmt_byte, PatrolDevice)."""
    sync = frame[:4]
    (length,) = struct.unpack_from("<H", frame, 4)
    (msg_id,) = struct.unpack_from("<H", frame, 6)
    fmt = frame[8]
    body: Asdu = json.loads(frame[16:].decode())["PatrolDevice"]
    return sync, length, msg_id, fmt, body


class TestAxesFromTwist:
    def test_forward_and_yaw_normalized(self) -> None:
        # 0.5 m/s of 1.0 max -> 0.5; 0.75 rad/s of 1.5 max -> 0.5.
        axes = _axes_from_twist(Twist(Vector3(0.5, 0.0, 0.0), Vector3(0.0, 0.0, 0.75)), 1.0, 1.5)
        assert axes["X"] == 0.5
        assert axes["Yaw"] == 0.5
        assert axes["Y"] == 0.0
        # Pose axes are unused in Basic/Stair gait and must stay zero.
        assert axes["Z"] == axes["Roll"] == axes["Pitch"] == 0.0

    def test_clipped_to_unit_range(self) -> None:
        # Commands beyond max speed saturate at the [-1, 1] envelope.
        axes = _axes_from_twist(Twist(Vector3(5.0, -5.0, 0.0), Vector3(0.0, 0.0, -9.0)), 1.0, 1.5)
        assert axes["X"] == 1.0
        assert axes["Y"] == -1.0
        assert axes["Yaw"] == -1.0


class TestTelemetryTypes:
    """Telemetry-group -> typed message mapping + message round-trips."""

    def test_battery_mapping(self) -> None:
        # Live shape from the M20: dual battery, percentage as 0..100.
        items = {
            "VoltageLeft": 80.0,
            "BatteryLevelLeft": 82.0,
            "battery_temperatureLeft": 37.3,
            "chargeLeft": False,
            "serialLeft": "ABC",
        }
        bat = _battery_from(items, "Left", "left")
        assert bat.voltage == 80.0
        assert bat.percentage == 0.82  # 82% -> ROS 0..1 fraction
        assert bat.temperature == 37.3
        assert bat.power_supply_status == POWER_SUPPLY_STATUS_DISCHARGING
        assert bat.location == "left"
        assert bat.serial_number == "ABC"
        assert bat.present is True

    def test_status_mapping_keeps_raw_pro_values(self) -> None:
        # M20 Pro reports values outside the documented enums; keep them raw.
        bs = {
            "MotionState": 17,
            "Gait": 4097,
            "ControlUsageMode": 0,
            "HES": 0,
            "Version": "PRO",
            "Sleep": 0,
        }
        st = _status_from(bs)
        assert st.motion_state == 17
        assert st.gait == 4097
        assert st.version == "PRO"
        assert st.hard_estop is False

    def test_battery_lcm_roundtrip(self) -> None:
        bat = BatteryState(voltage=80.0, percentage=0.82, temperature=37.3, location="right")
        out = BatteryState.lcm_decode(bat.lcm_encode())
        assert out.voltage == 80.0
        assert abs(out.percentage - 0.82) < 1e-6
        assert out.location == "right"

    def test_status_lcm_roundtrip(self) -> None:
        st = M20Status(motion_state=17, gait=4097, hard_estop=True, version="PRO")
        out = M20Status.lcm_decode(st.lcm_encode())
        assert out.motion_state == 17
        assert out.gait == 4097
        assert out.hard_estop is True
        assert out.version == "PRO"


class TestApplyDeadman:
    def test_fresh_command_passes_through(self) -> None:
        cmd = Twist(Vector3(1.0, 0.0, 0.0), Vector3())
        assert _apply_deadman(cmd, age=0.1, timeout=0.4) is cmd

    def test_stale_command_zeroed(self) -> None:
        cmd = Twist(Vector3(1.0, 0.0, 0.0), Vector3())
        assert _apply_deadman(cmd, age=0.5, timeout=0.4).is_zero()

    def test_missing_command_zeroed(self) -> None:
        assert _apply_deadman(None, age=0.0, timeout=0.4).is_zero()


class TestPatrolLink:
    """Exercise framing + telemetry against a local fake-robot UDP socket."""

    def _fake_robot(self) -> socket.socket:
        srv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        srv.bind(("127.0.0.1", 0))  # ephemeral port; read it back via getsockname()
        srv.settimeout(2.0)
        return srv

    def test_heartbeat_framing(self) -> None:
        srv = self._fake_robot()
        port = srv.getsockname()[1]
        link = PatrolLink("127.0.0.1", port=port)
        try:
            link.start()
            frame, _ = srv.recvfrom(65535)
            sync, length, msg_id, fmt, body = _decode(frame)

            assert sync == b"\xeb\x91\xeb\x90"  # spec-table sync word
            assert fmt == 0x01  # JSON
            assert length == len(frame) - 16  # header is fixed 16 bytes
            assert msg_id == 0  # first frame; counter starts at 0
            assert body["Type"] == 100 and body["Command"] == 100  # heartbeat
            assert body["Items"] == {}
        finally:
            link.stop()
            srv.close()

    def test_move_command_payload(self) -> None:
        srv = self._fake_robot()
        port = srv.getsockname()[1]
        link = PatrolLink("127.0.0.1", port=port)
        try:
            link.start()
            link.send(2, 21, _axes_from_twist(Twist(Vector3(1.0, 0.0, 0.0), Vector3()), 1.0, 1.5))

            # Drain until we see the move frame (heartbeats are interleaved).
            move = None
            deadline = time.time() + 2.0
            while move is None and time.time() < deadline:
                _, _, _, _, body = _decode(srv.recvfrom(65535)[0])
                if body["Type"] == 2 and body["Command"] == 21:
                    move = body
            assert move is not None, "no move frame received"
            assert move["Items"]["X"] == 1.0
            assert move["Items"]["Yaw"] == 0.0
        finally:
            link.stop()
            srv.close()

    def test_telemetry_basicstatus_parsed(self) -> None:
        srv = self._fake_robot()
        port = srv.getsockname()[1]
        link = PatrolLink("127.0.0.1", port=port)
        try:
            link.start()
            _, client_addr = srv.recvfrom(65535)  # first heartbeat tells us where to reply

            asdu = json.dumps(
                {
                    "PatrolDevice": {
                        "Type": 1002,
                        "Command": 6,
                        "Time": "2026-06-18 00:00:00",
                        "Items": {"BasicStatus": {"MotionState": 6, "Gait": 1, "Version": "STD"}},
                    }
                }
            ).encode()
            hdr = bytearray(16)
            hdr[0:4] = b"\xeb\x91\xeb\x90"
            struct.pack_into("<H", hdr, 4, len(asdu))
            hdr[8] = 0x01
            srv.sendto(bytes(hdr) + asdu, client_addr)

            deadline = time.time() + 2.0
            while not link.latest_status and time.time() < deadline:
                time.sleep(0.02)
            assert link.latest_status.get("MotionState") == 6
            assert link.latest_status.get("Version") == "STD"
        finally:
            link.stop()
            srv.close()
