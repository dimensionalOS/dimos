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

"""Damiao MIT-mode CAN driver for OpenArm. SI units throughout.

Ported from ``enactic/openarm_can`` (C++). No dimos deps — testable with
``can.Bus(interface="virtual")``.
"""

from __future__ import annotations

from dataclasses import dataclass
import enum
import errno
import struct
import threading
import time
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import can


class MotorType(str, enum.Enum):
    """Damiao motor types used on OpenArm. Values match the reference library."""

    DM3507 = "DM3507"
    DM4310 = "DM4310"
    DM4310_48V = "DM4310_48V"
    DM4340 = "DM4340"
    DM4340_48V = "DM4340_48V"
    DM6006 = "DM6006"
    DM8006 = "DM8006"
    DM8009 = "DM8009"
    DM10010L = "DM10010L"
    DM10010 = "DM10010"
    DMH3510 = "DMH3510"
    DMH6215 = "DMH6215"
    DMG6220 = "DMG6220"


# (p_max [rad], v_max [rad/s], t_max [Nm])
_MOTOR_LIMITS: dict[MotorType, tuple[float, float, float]] = {
    MotorType.DM3507: (12.5, 50.0, 5.0),
    MotorType.DM4310: (12.5, 30.0, 10.0),
    MotorType.DM4310_48V: (12.5, 50.0, 10.0),
    MotorType.DM4340: (12.5, 8.0, 28.0),
    MotorType.DM4340_48V: (12.5, 10.0, 28.0),
    MotorType.DM6006: (12.5, 45.0, 20.0),
    MotorType.DM8006: (12.5, 45.0, 40.0),
    MotorType.DM8009: (12.5, 45.0, 54.0),
    MotorType.DM10010L: (12.5, 25.0, 200.0),
    MotorType.DM10010: (12.5, 20.0, 200.0),
    MotorType.DMH3510: (12.5, 280.0, 1.0),
    MotorType.DMH6215: (12.5, 45.0, 10.0),
    MotorType.DMG6220: (12.5, 45.0, 10.0),
}

# MIT gain ranges (protocol-fixed, same for every motor type)
KP_MIN, KP_MAX = 0.0, 500.0
KD_MIN, KD_MAX = 0.0, 5.0

# Broadcast/control CAN IDs
_BROADCAST_ID = 0x7FF
_CMD_ENABLE = 0xFC
_CMD_DISABLE = 0xFD
_RID_CTRL_MODE = 10
CTRL_MODE_MIT = 1


def _clamp(x: float, lo: float, hi: float) -> float:
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def float_to_uint(x: float, lo: float, hi: float, bits: int) -> int:
    x = _clamp(x, lo, hi)
    return int((x - lo) / (hi - lo) * ((1 << bits) - 1))


def uint_to_float(u: int, lo: float, hi: float, bits: int) -> float:
    return u / ((1 << bits) - 1) * (hi - lo) + lo


def pack_mit_frame(
    motor_type: MotorType,
    q: float,
    dq: float,
    kp: float,
    kd: float,
    tau: float,
) -> bytes:
    p_max, v_max, t_max = _MOTOR_LIMITS[motor_type]
    q_u = float_to_uint(q, -p_max, p_max, 16)
    dq_u = float_to_uint(dq, -v_max, v_max, 12)
    kp_u = float_to_uint(kp, KP_MIN, KP_MAX, 12)
    kd_u = float_to_uint(kd, KD_MIN, KD_MAX, 12)
    tau_u = float_to_uint(tau, -t_max, t_max, 12)
    return bytes(
        [
            (q_u >> 8) & 0xFF,
            q_u & 0xFF,
            (dq_u >> 4) & 0xFF,
            ((dq_u & 0xF) << 4) | ((kp_u >> 8) & 0xF),
            kp_u & 0xFF,
            (kd_u >> 4) & 0xFF,
            ((kd_u & 0xF) << 4) | ((tau_u >> 8) & 0xF),
            tau_u & 0xFF,
        ]
    )


@dataclass(frozen=True)
class MotorState:
    """Decoded state from a Damiao reply frame."""

    q: float  # rad
    dq: float  # rad/s
    tau: float  # Nm
    t_mos: int  # °C
    t_rotor: int  # °C
    timestamp: float  # monotonic seconds when received


def parse_state_frame(motor_type: MotorType, data: bytes) -> MotorState | None:
    """Decode an 8-byte Damiao state reply. Returns None if too short."""
    if len(data) < 8:
        return None
    p_max, v_max, t_max = _MOTOR_LIMITS[motor_type]
    q_u = (data[1] << 8) | data[2]
    dq_u = (data[3] << 4) | (data[4] >> 4)
    tau_u = ((data[4] & 0x0F) << 8) | data[5]
    return MotorState(
        q=uint_to_float(q_u, -p_max, p_max, 16),
        dq=uint_to_float(dq_u, -v_max, v_max, 12),
        tau=uint_to_float(tau_u, -t_max, t_max, 12),
        t_mos=int(data[6]),
        t_rotor=int(data[7]),
        timestamp=time.monotonic(),
    )


def _pack_control_command(cmd: int) -> bytes:
    return bytes([0xFF] * 7 + [cmd & 0xFF])


def pack_write_param_frame(send_id: int, rid: int, value_u32: int) -> bytes:
    """Broadcast parameter-write frame sent to CAN id 0x7FF."""
    val = struct.pack("<I", value_u32 & 0xFFFFFFFF)
    return bytes(
        [
            send_id & 0xFF,
            (send_id >> 8) & 0xFF,
            0x55,
            rid & 0xFF,
            val[0],
            val[1],
            val[2],
            val[3],
        ]
    )


@dataclass(frozen=True)
class DamiaoMotor:
    """One Damiao motor on a CAN bus. recv_id defaults to send_id | 0x10."""

    send_id: int
    motor_type: MotorType
    recv_id: int | None = None

    @property
    def effective_recv_id(self) -> int:
        return self.recv_id if self.recv_id is not None else (self.send_id | 0x10)

    @property
    def limits(self) -> tuple[float, float, float]:
        return _MOTOR_LIMITS[self.motor_type]


class OpenArmBus:
    """One SocketCAN bus with a background RX thread caching latest state."""

    def __init__(
        self,
        channel: str,
        motors: list[DamiaoMotor],
        *,
        fd: bool = False,
        interface: str = "socketcan",
    ) -> None:
        if not motors:
            raise ValueError("OpenArmBus needs at least one motor")
        # Enforce unique IDs — silent overlap would make state routing ambiguous.
        send_ids = [m.send_id for m in motors]
        if len(set(send_ids)) != len(send_ids):
            raise ValueError(f"duplicate send_id in {send_ids}")
        recv_ids = [m.effective_recv_id for m in motors]
        if len(set(recv_ids)) != len(recv_ids):
            raise ValueError(f"duplicate recv_id in {recv_ids}")

        self._channel = channel
        self._motors = list(motors)
        self._fd = fd
        self._interface = interface
        self._by_recv: dict[int, DamiaoMotor] = {m.effective_recv_id: m for m in motors}

        self._bus: can.BusABC | None = None
        self._rx_thread: threading.Thread | None = None
        self._rx_stop = threading.Event()
        self._state_lock = threading.Lock()
        self._states: dict[int, MotorState] = {}

    def open(self) -> None:
        """Open the CAN bus and start the background RX thread."""
        if self._bus is not None:
            return
        import can  # local import — python-can is optional

        self._bus = can.Bus(interface=self._interface, channel=self._channel, fd=self._fd)
        self._rx_stop.clear()
        self._rx_thread = threading.Thread(
            target=self._rx_loop, name=f"openarm-rx-{self._channel}", daemon=True
        )
        self._rx_thread.start()

    def close(self) -> None:
        """Stop the RX thread and close the CAN bus."""
        self._rx_stop.set()
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=1.0)
            self._rx_thread = None
        if self._bus is not None:
            try:
                self._bus.shutdown()
            finally:
                self._bus = None

    def enable_all(self) -> None:
        for m in self._motors:
            self._send_raw(m.send_id, _pack_control_command(_CMD_ENABLE))

    def disable_all(self) -> None:
        for m in self._motors:
            self._send_raw(m.send_id, _pack_control_command(_CMD_DISABLE))

    def write_ctrl_mode(self, send_id: int, mode: int = CTRL_MODE_MIT) -> None:
        self._send_raw(
            _BROADCAST_ID,
            pack_write_param_frame(send_id, _RID_CTRL_MODE, mode),
        )

    def send_mit_many(
        self,
        commands: list[tuple[float, float, float, float, float]],
    ) -> None:
        """One MIT frame per motor; commands[i] → self.motors[i] = (q, dq, kp, kd, tau)."""
        if len(commands) != len(self._motors):
            raise ValueError(f"expected {len(self._motors)} commands, got {len(commands)}")
        for motor, cmd in zip(self._motors, commands, strict=False):
            q, dq, kp, kd, tau = cmd
            data = pack_mit_frame(motor.motor_type, q, dq, kp, kd, tau)
            self._send_raw(motor.send_id, data)

    def get_state(self, send_id: int) -> MotorState | None:
        motor = next((m for m in self._motors if m.send_id == send_id), None)
        if motor is None:
            return None
        with self._state_lock:
            return self._states.get(motor.effective_recv_id)

    def get_states(self) -> list[MotorState | None]:
        with self._state_lock:
            return [self._states.get(m.effective_recv_id) for m in self._motors]

    def _send_raw(self, arbitration_id: int, data: bytes) -> None:
        if self._bus is None:
            raise RuntimeError("bus not open — call .open() first")
        import can

        msg = can.Message(
            arbitration_id=arbitration_id,
            data=data,
            is_extended_id=False,
            is_fd=self._fd,
            bitrate_switch=self._fd,
        )
        # Retry on TX buffer full (ENOBUFS) — gs_usb's kernel-side TX queue
        # is small. python-can chains the OSError via `raise ... from`,
        # so the original errno is on __cause__.
        for attempt in range(4):
            try:
                self._bus.send(msg)
                return
            except can.CanOperationError as e:
                cause = e.__cause__ or e
                if getattr(cause, "errno", None) == errno.ENOBUFS and attempt < 3:
                    time.sleep(0.001 * (attempt + 1))
                else:
                    raise

    def _rx_loop(self) -> None:
        assert self._bus is not None
        while not self._rx_stop.is_set():
            msg = self._bus.recv(timeout=0.05)
            if msg is None:
                continue
            motor = self._by_recv.get(int(msg.arbitration_id))
            if motor is None:
                continue
            state = parse_state_frame(motor.motor_type, bytes(msg.data))
            if state is None:
                continue
            with self._state_lock:
                self._states[motor.effective_recv_id] = state


__all__ = [
    "CTRL_MODE_MIT",
    "KD_MAX",
    "KD_MIN",
    "KP_MAX",
    "KP_MIN",
    "DamiaoMotor",
    "MotorState",
    "MotorType",
    "OpenArmBus",
    "float_to_uint",
    "pack_mit_frame",
    "pack_write_param_frame",
    "parse_state_frame",
    "uint_to_float",
]
