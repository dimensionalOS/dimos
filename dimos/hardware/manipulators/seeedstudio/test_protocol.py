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

from collections.abc import Iterator
import struct

import pytest
from pytest_mock import MockerFixture

from dimos.hardware.manipulators.seeedstudio.protocol import (
    STREAM_PACKET_INTERVAL,
    DmSerialTransport,
    FrameParser,
    MotorParameters,
    decode_feedback,
    encode_frame,
)


def rx(can_id: int, data: bytes, flags: int = 8) -> bytes:
    return b"\xaa\x11" + bytes([flags]) + struct.pack("<I", can_id) + data + b"\x55"


def test_tx_matches_bridge_wire_format() -> None:
    packet = encode_frame(0x7FF, bytes.fromhex("0100330a00000000"))
    assert packet.hex() == "55aa1e03010000000a00000000ff070000000800000100330a0000000000"


def test_parser_fragmentation_noise_and_frame_flags() -> None:
    parser = FrameParser()
    frame = rx(0x11, bytes(range(8)))
    assert parser.feed(b"noise\xaa" + frame[:7]) == []
    assert parser.feed(frame[7:] + rx(0x12, bytes(8), 0x48) + frame) == [
        (0x11, bytes(range(8))),
        (0x11, bytes(range(8))),
    ]


def test_feedback_scaling_and_status() -> None:
    params = MotorParameters(1, 17, 2, 12.5, 10, 28, 0)
    feedback = decode_feedback(bytes.fromhex("a180008008002a2b"), params)
    assert feedback.status == 10
    assert feedback.position == pytest.approx(12.5 / 65535)
    assert feedback.velocity == pytest.approx(10 / 4095)
    assert feedback.effort == pytest.approx(28 / 4095)
    assert (feedback.mos_temperature, feedback.rotor_temperature) == (42, 43)


@pytest.fixture
def bus(mocker: MockerFixture) -> Iterator[DmSerialTransport]:
    mocker.patch("serial.Serial")
    transport = DmSerialTransport("test-only", timeout=0.001)
    transport.open()
    yield transport
    transport.close()


def test_register_rejects_wrong_id_and_wrong_register(
    bus: DmSerialTransport, mocker: MockerFixture
) -> None:
    port = mocker.patch.object(bus, "send")
    serial_port = mocker.patch("serial.Serial").return_value
    # Patch only serial IO; keep actual request framing, parser, and matcher.
    bus.close()
    bus.open()
    serial_port.in_waiting = 64
    serial_port.read.return_value = (
        rx(0x12, bytes.fromhex("0100330a02000000"))
        + rx(0x11, bytes.fromhex("0100330802000000"))
        + rx(0x11, bytes.fromhex("0100330a02000000"))
    )
    assert bus.read_register(1, 17, 10) == 2
    serial_port.reset_input_buffer.assert_called_once()
    port.assert_called_once_with(0x7FF, bytes.fromhex("0100330a00000000"))


def test_missing_reply_times_out_after_discarding_input(
    bus: DmSerialTransport, mocker: MockerFixture
) -> None:
    mocker.patch.object(bus, "send")
    serial_port = mocker.patch("serial.Serial").return_value
    bus.close()
    bus.open()
    serial_port.in_waiting = 0
    serial_port.read.return_value = b""
    with pytest.raises(TimeoutError, match="motor 1"):
        bus.read_register(1, 17, 10)
    serial_port.reset_input_buffer.assert_called_once()


def test_short_write_is_an_error(bus: DmSerialTransport, mocker: MockerFixture) -> None:
    serial_port = mocker.patch("serial.Serial").return_value
    bus.close()
    bus.open()
    serial_port.write.return_value = 4
    with pytest.raises(OSError, match="Incomplete"):
        bus.send(1, bytes(8))


@pytest.mark.parametrize(
    "payload", [bytes.fromhex("0100330a02000000"), bytes.fromhex("0280008008002a2b")]
)
def test_register_and_other_motor_are_not_sensor_feedback(payload: bytes) -> None:
    with pytest.raises(ValueError):
        decode_feedback(payload, MotorParameters(1, 17, 2, 12.5, 10, 28, 0))


def test_control_writes_are_spaced_and_queries_do_not_add_delay(mocker: MockerFixture) -> None:
    serial_port = mocker.patch("serial.Serial").return_value
    serial_port.write.return_value = 30
    clock = [1.0]
    mocker.patch(
        "dimos.hardware.manipulators.seeedstudio.protocol.time.monotonic",
        side_effect=lambda: clock[0],
    )

    def advance(seconds: float) -> None:
        clock[0] += seconds

    sleep = mocker.patch(
        "dimos.hardware.manipulators.seeedstudio.protocol.time.sleep", side_effect=advance
    )
    transport = DmSerialTransport("test-only")
    try:
        transport.open()
        transport.send(1, bytes(8))
        transport.send(2, bytes(8))
        transport.send(0x7FF, bytes(8))
        transport.send(0x7FF, bytes(8))
        transport.send(0x101, bytes(8), STREAM_PACKET_INTERVAL)
        transport.send(0x102, bytes(8), STREAM_PACKET_INTERVAL)
        assert [call.args[0] for call in sleep.call_args_list] == pytest.approx(
            [0.02, 0.02, STREAM_PACKET_INTERVAL]
        )
        assert serial_port.write.call_count == 6
    finally:
        transport.close()
