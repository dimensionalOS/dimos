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

"""Original Nintendo Joy-Cons over HIDAPI, including Bluetooth on macOS.

The 0x30 report and SPI calibration layout are documented by SDL's Switch
driver: https://github.com/libsdl-org/SDL/blob/main/src/joystick/hidapi/SDL_hidapi_switch.c
Only input-mode configuration and calibration reads are sent; no rumble,
pairing, flash writes or motor control. Each side remains a distinct device.
"""

from dataclasses import dataclass
import time
from typing import Any

try:
    import hid  # type: ignore[import-untyped,import-not-found]
except ImportError:
    hid = None

from dimos.teleop.joycon.mapping import JOYCON_SIDES, JoyConReading, JoyConSide
from dimos.utils.logging_config import setup_logger

logger = setup_logger()
NINTENDO_VENDOR = 0x057E
PRODUCTS: dict[JoyConSide, int] = {"left": 0x2006, "right": 0x2007}
# Button byte offsets include the report ID. Keep the printed upright labels.
BUTTON_BITS: dict[JoyConSide, dict[str, tuple[int, int]]] = {
    "left": {
        "down": (5, 0),
        "up": (5, 1),
        "right": (5, 2),
        "left": (5, 3),
        "SR": (5, 4),
        "SL": (5, 5),
        "L": (5, 6),
        "ZL": (5, 7),
        "minus": (4, 0),
        "stick": (4, 3),
        "capture": (4, 5),
    },
    "right": {
        "Y": (3, 0),
        "X": (3, 1),
        "B": (3, 2),
        "A": (3, 3),
        "SR": (3, 4),
        "SL": (3, 5),
        "R": (3, 6),
        "ZR": (3, 7),
        "plus": (4, 1),
        "stick": (4, 2),
        "home": (4, 4),
    },
}


def unpack_stick(data: bytes) -> tuple[int, int]:
    if len(data) != 3:
        raise ValueError("a packed stick requires three bytes")
    return data[0] | ((data[1] & 15) << 8), (data[1] >> 4) | (data[2] << 4)


@dataclass(frozen=True)
class StickCalibration:
    center: tuple[int, int]
    below: tuple[int, int]
    above: tuple[int, int]

    @classmethod
    def decode(cls, data: bytes, side: JoyConSide) -> "StickCalibration":
        if len(data) != 9:
            raise ValueError("stick calibration requires nine bytes")
        first, second, third = (unpack_stick(data[i : i + 3]) for i in (0, 3, 6))
        center, below, above = (second, third, first) if side == "left" else (first, second, third)
        if not all(0 < n < 4095 for n in (*center, *below, *above)):
            raise ValueError("Joy-Con has invalid stick calibration")
        return cls(center, below, above)

    def normalize(self, raw: tuple[int, int]) -> tuple[float, float]:
        axes = []
        for i, value in enumerate(raw):
            delta = value - self.center[i]
            span = self.above[i] if delta >= 0 else self.below[i]
            axes.append(max(-1.0, min(1.0, delta / span)))
        # HID Y increases upwards; the shared Joy-Con API increases downwards.
        return axes[0], -axes[1]


def decode_report(
    data: bytes, side: JoyConSide, calibration: StickCalibration
) -> JoyConReading | None:
    if not data or data[0] != 0x30 or len(data) < 13:
        return None
    offset = 6 if side == "left" else 9
    return JoyConReading(
        pressed=frozenset(
            label for label, (byte, bit) in BUTTON_BITS[side].items() if data[byte] & (1 << bit)
        ),
        stick=calibration.normalize(unpack_stick(data[offset : offset + 3])),
    )


class _Device:
    def __init__(self, handle: Any, side: JoyConSide) -> None:
        self.handle = handle
        self.side = side
        self.counter = 0
        self.reading: JoyConReading | None = None
        self.last_report = -float("inf")
        self.opened_at = time.monotonic()
        self.calibration: StickCalibration | None = None

    def _send(self, command: int, payload: bytes) -> None:
        neutral_rumble = bytes((0, 1, 0x40, 0x40)) * 2
        packet = (bytes((1, self.counter)) + neutral_rumble + bytes((command,)) + payload).ljust(
            49, b"\0"
        )
        self.counter = (self.counter + 1) % 16
        if self.handle.write(packet) != len(packet):
            raise OSError("incomplete Joy-Con configuration write")

    def _spi(self, address: int, length: int) -> bytes:
        self._send(0x10, address.to_bytes(4, "little") + bytes((length,)))
        deadline = time.monotonic() + 0.25
        while time.monotonic() < deadline:
            packet = bytes(self.handle.read(64))
            if (
                len(packet) >= 20 + length
                and packet[0] == 0x21
                and packet[13] & 0x80
                and packet[14] == 0x10
                and int.from_bytes(packet[15:19], "little") == address
                and packet[19] == length
            ):
                return packet[20 : 20 + length]
            time.sleep(0.002)
        return b""

    def configure(self) -> None:
        self.handle.set_nonblocking(1)
        user = self._spi(0x8010 if self.side == "left" else 0x801B, 11)
        data = (
            user[2:]
            if user[:2] == b"\xb2\xa1"
            else self._spi(0x603D if self.side == "left" else 0x6046, 9)
        )
        self.calibration = StickCalibration.decode(data, self.side)
        self._send(0x03, b"\x30")
        self.opened_at = time.monotonic()

    def read(self, now: float, timeout: float) -> JoyConReading | None:
        assert self.calibration is not None
        # Drain the queue before granting a fresh lease. A backed-up stream must
        # not turn hundreds of old packets into newly timestamped robot commands.
        latest = None
        for _ in range(256):
            data = bytes(self.handle.read(64))
            if not data:
                break
            sample = decode_report(data, self.side, self.calibration)
            if sample is not None:
                latest = sample
        else:
            raise OSError("Joy-Con input queue exceeded the freshness budget")
        if latest is not None:
            self.reading = latest
            self.last_report = now
        if now - max(self.last_report, self.opened_at) > timeout:
            raise OSError("Joy-Con stopped sending input reports")
        return self.reading


class HidJoyConBackend:
    def __init__(self, report_timeout_s: float = 0.2) -> None:
        self.timeout = report_timeout_s
        self.devices: dict[JoyConSide, _Device] = {}
        self._next_scan = 0.0

    def open(self) -> None:
        if hid is None:
            raise ImportError("macOS Joy-Cons require `uv sync --extra joycon-teleop --inexact`")
        self._scan()

    def _scan(self) -> None:
        assert hid is not None
        self._next_scan = time.monotonic() + 1
        candidates = hid.enumerate(NINTENDO_VENDOR, 0)
        for side in JOYCON_SIDES:
            if side in self.devices:
                continue
            found = next(
                (
                    d
                    for d in candidates
                    if d["product_id"] == PRODUCTS[side]
                    and d.get("usage_page") == 1
                    and d.get("usage") == 5
                ),
                None,
            )
            if found is None:
                continue
            handle = hid.device()
            try:
                handle.open_path(found["path"])
                device = _Device(handle, side)
                device.configure()
            except (OSError, ValueError) as exc:
                handle.close()
                logger.warning("Cannot read %s Joy-Con over HID: %s", side, exc)
                continue
            self.devices[side] = device
            logger.info("Joy-Con %s connected over HID", side)

    def read(self) -> dict[JoyConSide, JoyConReading | None]:
        if len(self.devices) < 2 and time.monotonic() >= self._next_scan:
            self._scan()
        readings: dict[JoyConSide, JoyConReading | None] = dict.fromkeys(JOYCON_SIDES)
        for side, device in list(self.devices.items()):
            try:
                readings[side] = device.read(time.monotonic(), self.timeout)
            except OSError as exc:
                device.handle.close()
                del self.devices[side]
                logger.warning("Joy-Con %s disconnected: %s", side, exc)
        return readings

    def close(self) -> None:
        for device in self.devices.values():
            device.handle.close()
        self.devices.clear()
