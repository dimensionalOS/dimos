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

"""Damiao serial bridge framing and B601-DM motor feedback.

Wire format: MotorBridge motor_core/src/dm_serial.rs and
motor_vendors/damiao/src/protocol.rs. No SDK controller with auto-enable.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import math
import struct
import threading
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import serial

QUERY_ID = 0x7FF
POS_VEL_MODE = 2
POS_VEL_OFFSET = 0x100
ENABLE = b"\xff" * 7 + b"\xfc"
DISABLE = b"\xff" * 7 + b"\xfd"
BAUDRATE = 921600
# Enable/disable frames are paced conservatively. Streamed POS_VEL targets are
# sent back to back like the vendor's 500 Hz loop, with only a short gap.
CONTROL_PACKET_INTERVAL = 0.02
STREAM_PACKET_INTERVAL = 0.001
JOINT_NAMES = (*(f"joint{i}" for i in range(1, 7)), "gripper_motor")
MOTOR_IDS = tuple(range(1, 8))
FEEDBACK_IDS = tuple(range(0x11, 0x18))
STATUS_NAMES = {
    0: "disabled",
    1: "enabled",
    8: "over_voltage",
    9: "under_voltage",
    10: "over_current",
    11: "mos_over_temperature",
    12: "rotor_over_temperature",
    13: "communication_lost",
    14: "overload",
}


@dataclass(frozen=True)
class MotorParameters:
    motor_id: int
    feedback_id: int
    mode: int
    pmax: float
    vmax: float
    tmax: float
    timeout: int

    def __post_init__(self) -> None:
        if any(not math.isfinite(v) or v <= 0 for v in (self.pmax, self.vmax, self.tmax)):
            raise ValueError("Motor feedback ranges must be finite and positive")


@dataclass(frozen=True)
class Feedback:
    position: float
    velocity: float
    effort: float
    status: int
    mos_temperature: int
    rotor_temperature: int
    received_at: float


def encode_frame(can_id: int, payload: bytes) -> bytes:
    if not 0 <= can_id <= 0x7FF or len(payload) != 8:
        raise ValueError("Expected an 11-bit CAN ID and eight payload bytes")
    packet = bytearray(30)
    packet[:4] = b"\x55\xaa\x1e\x03"
    struct.pack_into("<II", packet, 4, 1, 10)
    struct.pack_into("<I", packet, 13, can_id)
    packet[18] = 8
    packet[21:29] = payload
    return bytes(packet)


class FrameParser:
    """Incremental parser, resynchronizing one byte at a time after noise."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def clear(self) -> None:
        self._buffer.clear()

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        self._buffer.extend(data)
        frames = []
        while len(self._buffer) >= 16:
            if self._buffer[:3] != b"\xaa\x11\x08" or self._buffer[15] != 0x55:
                del self._buffer[0]
                continue
            can_id = struct.unpack_from("<I", self._buffer, 3)[0]
            payload = bytes(self._buffer[7:15])
            del self._buffer[:16]
            if can_id <= 0x7FF:
                frames.append((can_id, payload))
        return frames


def is_sensor_reply(data: bytes, motor_id: int) -> bool:
    # Register replies have overlapping bit patterns with sensor frames.
    # Reject ambiguous register-shaped data, as the vendor SDK does.
    return (data[0] & 15) == motor_id and not (data[1] <= 15 and data[2] in (0x33, 0x55))


def decode_feedback(data: bytes, params: MotorParameters) -> Feedback:
    if len(data) != 8 or not is_sensor_reply(data, params.motor_id):
        raise ValueError("Not sensor feedback from the expected motor")
    position = ((data[1] << 8) | data[2]) * (2 * params.pmax) / 65535 - params.pmax
    velocity = ((data[3] << 4) | (data[4] >> 4)) * (2 * params.vmax) / 4095 - params.vmax
    effort = (((data[4] & 15) << 8) | data[5]) * (2 * params.tmax) / 4095 - params.tmax
    return Feedback(position, velocity, effort, data[0] >> 4, data[6], data[7], time.monotonic())


class DmSerialTransport:
    """One synchronous owner for request/response transactions on a serial port.

    Each transaction discards queued input before sending its request. This
    prevents accepting an old software cache as a successful motor response.
    The bridge has no sequence numbers, so delayed on-wire replies cannot be
    distinguished from a current reply with the same ID and payload type.
    """

    def __init__(self, address: str, timeout: float = 0.15) -> None:
        if not address:
            raise ValueError("An explicit serial port is required")
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("timeout must be finite and positive")
        self._address = address
        self._timeout = timeout
        self._port: serial.Serial | None = None
        self._parser = FrameParser()
        self._next_write_at = 0.0
        self._lock = threading.RLock()

    def open(self) -> None:
        # Optional serial dependency: registry discovery does not require it.
        import serial

        with self._lock:
            if self._port is None:
                self._port = serial.Serial(
                    self._address,
                    BAUDRATE,
                    timeout=min(0.01, self._timeout),
                    write_timeout=self._timeout,
                    exclusive=True,
                )

    def close(self) -> None:
        with self._lock:
            if self._port is not None:
                try:
                    self._port.close()
                finally:
                    self._port = None
                    self._parser.clear()

    def is_open(self) -> bool:
        with self._lock:
            return self._port is not None and self._port.is_open

    def send(self, can_id: int, payload: bytes, interval: float = CONTROL_PACKET_INTERVAL) -> None:
        packet = encode_frame(can_id, payload)
        with self._lock:
            if self._port is None:
                raise OSError("Serial port is not open")
            delay = self._next_write_at - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            if self._port.write(packet) != len(packet):
                raise OSError("Incomplete USB-CAN serial write")
            # Query transactions are already paced by their response. Control
            # writes have no acknowledgement at the bridge and must not burst.
            if can_id != QUERY_ID:
                self._next_write_at = time.monotonic() + interval

    def _request(
        self, motor_id: int, feedback_id: int, payload: bytes, accept: Callable[[bytes], bool]
    ) -> bytes:
        with self._lock:
            if self._port is None:
                raise OSError("Serial port is not open")
            self._port.reset_input_buffer()
            self._parser.clear()
            self.send(QUERY_ID, payload)
            deadline = time.monotonic() + self._timeout
            while time.monotonic() < deadline:
                chunk = self._port.read(min(256, max(1, self._port.in_waiting)))
                for can_id, data in self._parser.feed(chunk):
                    if can_id == feedback_id and accept(data):
                        return data
            raise TimeoutError(
                f"No fresh reply from motor {motor_id} (feedback ID {feedback_id:#x})"
            )

    def read_register(
        self, motor_id: int, feedback_id: int, register: int, *, floating: bool = False
    ) -> float | int:
        prefix = struct.pack("<HBB", motor_id, 0x33, register)

        def matches(data: bytes) -> bool:
            return data[:4] == prefix

        reply = self._request(motor_id, feedback_id, prefix + bytes(4), matches)
        value: float | int = struct.unpack("<f" if floating else "<I", reply[4:])[0]
        return value

    def parameters(self, motor_id: int, feedback_id: int) -> MotorParameters:
        actual_id = self.read_register(motor_id, feedback_id, 8)
        actual_feedback = self.read_register(motor_id, feedback_id, 7)
        if actual_id != motor_id or actual_feedback != feedback_id:
            raise ValueError(f"Motor {motor_id} has unexpected ID registers")
        return MotorParameters(
            motor_id,
            feedback_id,
            int(self.read_register(motor_id, feedback_id, 10)),
            float(self.read_register(motor_id, feedback_id, 21, floating=True)),
            float(self.read_register(motor_id, feedback_id, 22, floating=True)),
            float(self.read_register(motor_id, feedback_id, 23, floating=True)),
            int(self.read_register(motor_id, feedback_id, 9)),
        )

    def feedback(self, params: MotorParameters) -> Feedback:
        def matches(data: bytes) -> bool:
            return is_sensor_reply(data, params.motor_id)

        request = struct.pack("<H", params.motor_id) + b"\xcc" + bytes(5)
        data = self._request(params.motor_id, params.feedback_id, request, matches)
        return decode_feedback(data, params)
