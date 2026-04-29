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

"""Unit tests for the Damiao MIT-mode driver — no hardware required.

Uses ``can.Bus(interface="virtual")`` for loopback.
"""

from __future__ import annotations

import struct
import time

import pytest

can = pytest.importorskip("can")

from dimos.hardware.manipulators.openarm.driver import (
    CTRL_MODE_MIT,
    KD_MAX,
    KP_MAX,
    DamiaoMotor,
    MotorType,
    OpenArmBus,
    float_to_uint,
    pack_mit_frame,
    pack_write_param_frame,
    parse_state_frame,
    uint_to_float,
)


def test_float_to_uint_endpoints_and_roundtrip() -> None:
    # Endpoints
    assert float_to_uint(-12.5, -12.5, 12.5, 16) == 0
    assert float_to_uint(12.5, -12.5, 12.5, 16) == (1 << 16) - 1
    # Midpoint is half the full range (rounded down)
    mid = float_to_uint(0.0, -12.5, 12.5, 16)
    assert mid in ((1 << 16) // 2 - 1, (1 << 16) // 2)
    # Out-of-range clamps
    assert float_to_uint(-100.0, -12.5, 12.5, 16) == 0
    assert float_to_uint(100.0, -12.5, 12.5, 16) == (1 << 16) - 1


def test_roundtrip_all_gain_ranges() -> None:
    # Quantization error should be tiny
    for bits, lo, hi in [(16, -12.5, 12.5), (12, 0.0, KP_MAX), (12, 0.0, KD_MAX)]:
        step = (hi - lo) / ((1 << bits) - 1)
        for k in range(0, 1 << bits, max(1, (1 << bits) // 50)):
            x = lo + k * step
            u = float_to_uint(x, lo, hi, bits)
            x2 = uint_to_float(u, lo, hi, bits)
            assert abs(x - x2) <= step


def test_mit_frame_kp_kd_zero_and_pos_zero() -> None:
    # q=dq=kp=kd=tau=0 → q_u = 32767 (16-bit midpoint), dq_u = 2047 (12-bit),
    # tau_u = 2047. kp_u = kd_u = 0 (min of their 0-positive range).
    data = pack_mit_frame(MotorType.DM4310, 0.0, 0.0, 0.0, 0.0, 0.0)
    assert len(data) == 8
    # Reconstruct fields from bytes
    q_u = (data[0] << 8) | data[1]
    dq_u = (data[2] << 4) | (data[3] >> 4)
    kp_u = ((data[3] & 0xF) << 8) | data[4]
    kd_u = (data[5] << 4) | (data[6] >> 4)
    tau_u = ((data[6] & 0xF) << 8) | data[7]
    assert kp_u == 0
    assert kd_u == 0
    # 16-bit midpoint of symmetric range
    assert q_u in (32767, 32768)
    assert dq_u in (2047, 2048)
    assert tau_u in (2047, 2048)


def test_mit_frame_full_positive() -> None:
    # Command at every max → every _u field saturates.
    data = pack_mit_frame(MotorType.DM4310, 12.5, 30.0, 500.0, 5.0, 10.0)
    q_u = (data[0] << 8) | data[1]
    dq_u = (data[2] << 4) | (data[3] >> 4)
    kp_u = ((data[3] & 0xF) << 8) | data[4]
    kd_u = (data[5] << 4) | (data[6] >> 4)
    tau_u = ((data[6] & 0xF) << 8) | data[7]
    assert q_u == 0xFFFF
    assert dq_u == 0xFFF
    assert kp_u == 0xFFF
    assert kd_u == 0xFFF
    assert tau_u == 0xFFF


def test_parse_state_roundtrip() -> None:
    # Build a synthetic reply frame with known values and verify decode.
    # Byte layout for state: [echo, q_hi, q_lo, dq_hi, dq_lo|tau_hi, tau_lo, t_mos, t_rotor]
    motor = MotorType.DM4340
    p_max, v_max, t_max = 12.5, 8.0, 28.0
    q_u = float_to_uint(0.3, -p_max, p_max, 16)
    dq_u = float_to_uint(-1.0, -v_max, v_max, 12)
    tau_u = float_to_uint(2.0, -t_max, t_max, 12)
    data = bytes(
        [
            0x03,
            (q_u >> 8) & 0xFF,
            q_u & 0xFF,
            (dq_u >> 4) & 0xFF,
            ((dq_u & 0xF) << 4) | ((tau_u >> 8) & 0xF),
            tau_u & 0xFF,
            33,
            28,
        ]
    )
    state = parse_state_frame(motor, data)
    assert state is not None
    assert abs(state.q - 0.3) < 0.001
    assert abs(state.dq - (-1.0)) < 0.01
    assert abs(state.tau - 2.0) < 0.02
    assert state.t_mos == 33
    assert state.t_rotor == 28


def test_parse_state_rejects_short_frames() -> None:
    assert parse_state_frame(MotorType.DM4310, b"\x00" * 4) is None


def test_pack_write_param_ctrl_mode_mit() -> None:
    data = pack_write_param_frame(0x05, 10, CTRL_MODE_MIT)
    assert data[0] == 0x05
    assert data[1] == 0x00
    assert data[2] == 0x55
    assert data[3] == 10
    assert struct.unpack("<I", data[4:8])[0] == 1


def _make_bus(channel: str, motors: list[DamiaoMotor]) -> OpenArmBus:
    return OpenArmBus(channel=channel, motors=motors, fd=False, interface="virtual")


def test_bus_validates_unique_ids() -> None:
    with pytest.raises(ValueError, match="duplicate send_id"):
        OpenArmBus(
            channel="v0",
            motors=[
                DamiaoMotor(0x01, MotorType.DM4310),
                DamiaoMotor(0x01, MotorType.DM4310),
            ],
            fd=False,
            interface="virtual",
        )


def test_bus_empty_motor_list_rejected() -> None:
    with pytest.raises(ValueError):
        OpenArmBus(channel="v0", motors=[], fd=False, interface="virtual")


def test_rx_thread_populates_state_cache() -> None:
    # Two peers on the same virtual channel loop back to each other.
    motors = [
        DamiaoMotor(0x01, MotorType.DM8006),
        DamiaoMotor(0x05, MotorType.DM4310),
    ]
    bus = _make_bus("openarm-test-rx", motors)
    # A raw sender on the same virtual channel injects state replies.
    sender = can.Bus(interface="virtual", channel="openarm-test-rx")
    try:
        bus.open()
        # Forge a reply for motor 0x01 (recv 0x11) at q = 0.25 rad
        q_u = float_to_uint(0.25, -12.5, 12.5, 16)
        dq_u = float_to_uint(0.0, -45.0, 45.0, 12)
        tau_u = float_to_uint(0.0, -40.0, 40.0, 12)
        payload = bytes(
            [
                0x01,
                (q_u >> 8) & 0xFF,
                q_u & 0xFF,
                (dq_u >> 4) & 0xFF,
                ((dq_u & 0xF) << 4) | ((tau_u >> 8) & 0xF),
                tau_u & 0xFF,
                30,
                28,
            ]
        )
        sender.send(can.Message(arbitration_id=0x11, data=payload, is_extended_id=False))
        # Poll briefly for the RX thread to consume it
        deadline = time.monotonic() + 0.5
        s = None
        while s is None and time.monotonic() < deadline:
            s = bus.get_state(0x01)
            time.sleep(0.01)
        assert s is not None, "RX thread did not pick up synthetic state reply"
        assert abs(s.q - 0.25) < 0.001
        # Motor 0x05 never got a reply → state should still be None
        assert bus.get_state(0x05) is None
    finally:
        bus.close()
        sender.shutdown()


def test_send_mit_many_fans_out_one_per_motor() -> None:
    motors = [
        DamiaoMotor(0x01, MotorType.DM8006),
        DamiaoMotor(0x02, MotorType.DM8006),
        DamiaoMotor(0x05, MotorType.DM4310),
    ]
    bus = _make_bus("openarm-test-send", motors)
    listener = can.Bus(interface="virtual", channel="openarm-test-send")
    try:
        bus.open()
        bus.send_mit_many(
            [
                (0.1, 0.0, 10.0, 0.5, 0.0),
                (0.2, 0.0, 10.0, 0.5, 0.0),
                (0.3, 0.0, 10.0, 0.5, 0.0),
            ]
        )
        seen_ids: set[int] = set()
        deadline = time.monotonic() + 0.5
        while len(seen_ids) < 3 and time.monotonic() < deadline:
            msg = listener.recv(timeout=0.1)
            if msg is not None:
                seen_ids.add(int(msg.arbitration_id))
        assert seen_ids == {0x01, 0x02, 0x05}
    finally:
        bus.close()
        listener.shutdown()


def test_send_mit_many_size_mismatch() -> None:
    bus = _make_bus(
        "openarm-test-mismatch",
        [DamiaoMotor(0x01, MotorType.DM4310), DamiaoMotor(0x02, MotorType.DM4310)],
    )
    try:
        bus.open()
        with pytest.raises(ValueError):
            bus.send_mit_many([(0.0, 0.0, 0.0, 0.0, 0.0)])
    finally:
        bus.close()


def test_enable_disable_frames_sent() -> None:
    bus = _make_bus(
        "openarm-test-enable",
        [DamiaoMotor(0x01, MotorType.DM4310), DamiaoMotor(0x05, MotorType.DM4310)],
    )
    listener = can.Bus(interface="virtual", channel="openarm-test-enable")
    try:
        bus.open()
        bus.enable_all()
        seen = {}
        deadline = time.monotonic() + 0.3
        while len(seen) < 2 and time.monotonic() < deadline:
            msg = listener.recv(timeout=0.1)
            if msg is not None:
                seen[int(msg.arbitration_id)] = bytes(msg.data)
        assert set(seen) == {0x01, 0x05}
        for data in seen.values():
            assert data == bytes([0xFF] * 7 + [0xFC])
    finally:
        bus.close()
        listener.shutdown()
