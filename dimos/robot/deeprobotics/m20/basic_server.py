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

"""Client for the Deep Robotics M20 basic_server protocol."""

from __future__ import annotations

from collections.abc import Callable
from datetime import datetime
import json
import socket
import struct
from threading import Event, Thread
import time
from typing import Any, NamedTuple

from dimos.utils.logging_config import setup_logger
from dimos.utils.sequential_ids import SequentialIds

logger = setup_logger()

USAGE_MODE_NAVIGATION = 1

CHARGE_COMMAND_STOP = 0
CHARGE_COMMAND_START = 1

CHARGE_IDLE = 0
CHARGE_ENTERING = 1
CHARGE_CHARGING = 2
CHARGE_EXITING = 3
CHARGE_ERROR = 4
CHARGE_ON_DOCK = 5

CHARGE_ERROR_NAMES = {
    0x8501: "location timeout",
    0x8502: "reflector localization failed to start",
    0x8503: "no location, reflectors not found",
    0x8506: "no current from the dock",
    0x8509: "dock entry timeout",
    0x8510: "dock exit timeout",
}

_MAGIC = bytes.fromhex("eb91eb90")
_JSON = 1
_HEADER = struct.Struct("<4sHHB7s")
_MODE_TYPE = 1101
_MODE_COMMAND = 5
_HEARTBEAT_TYPE = 100
_HEARTBEAT_COMMAND = 100
_HEARTBEAT_PERIOD_S = 0.5
_REPORT_TYPE = 1002
_ERROR_LIST_COMMAND = 3
_BATTERY_STATUS_COMMAND = 5
_BASIC_STATUS_COMMAND = 6
_CHARGE_TYPE = 2
_CHARGE_COMMAND = 24
_POLL_S = 0.25


class ChargeReport(NamedTuple):
    status: int
    received_at: float


class PackReport(NamedTuple):
    voltage: float
    level: float
    temperature: float


class BatteryReport(NamedTuple):
    left: PackReport
    right: PackReport
    received_at: float


class BasicServerLink:
    """Heartbeat, status reports and commands for the M20 basic_server."""

    def __init__(
        self,
        host: str,
        tcp_port: int,
        udp_port: int,
        timeout_s: float,
        on_report: Callable[[], None],
        on_battery: Callable[[BatteryReport], None],
    ) -> None:
        self._host = host
        self._tcp_port = tcp_port
        self._udp_port = udp_port
        self._timeout_s = timeout_s
        self._on_report = on_report
        self._on_battery = on_battery
        self._message_ids = SequentialIds()
        self._stop = Event()
        self._udp: socket.socket | None = None
        self._threads: list[Thread] = []
        self.charge: ChargeReport | None = None
        self.battery: BatteryReport | None = None
        self.charge_error: int | None = None
        self.usage_mode: int | None = None

    def start(self) -> None:
        self._stop.clear()
        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp.bind(("", 0))
        udp.settimeout(_POLL_S)
        self._udp = udp
        self._threads = [
            Thread(target=self._heartbeat, name="m20-heartbeat", daemon=True),
            Thread(target=self._read_reports, name="m20-reports", daemon=True),
        ]
        for thread in self._threads:
            thread.start()

    def stop(self) -> None:
        self._stop.set()
        for thread in self._threads:
            thread.join(timeout=self._timeout_s)
        if self._udp is not None:
            self._udp.close()
            self._udp = None

    def send_charge(self, command: int) -> bool:
        _message_id, apdu = self._encode(_CHARGE_TYPE, _CHARGE_COMMAND, {"Charge": command})
        try:
            self._send(apdu)
        except OSError as exc:
            logger.warning("M20 basic_server charge command failed: %s", exc)
            return False
        return True

    def set_usage_mode(self, mode: int) -> bool:
        device = self._request(_MODE_TYPE, _MODE_COMMAND, {"Mode": mode})
        if device is None:
            return False
        try:
            error_code = int(device["Items"]["ErrorCode"])
        except (KeyError, TypeError, ValueError):
            logger.error("M20 basic_server returned an invalid usage-mode response")
            return False
        if error_code != 0:
            logger.error("M20 basic_server usage-mode switch failed: error %s", error_code)
            return False
        return True

    def _request(
        self, message_type: int, command: int, items: dict[str, Any]
    ) -> dict[str, Any] | None:
        message_id, apdu = self._encode(message_type, command, items)
        try:
            with socket.create_connection(
                (self._host, self._tcp_port), timeout=self._timeout_s
            ) as connection:
                connection.sendall(apdu)
                magic, length, response_id, encoding, _reserved = _HEADER.unpack(
                    _recv_exact(connection, _HEADER.size)
                )
                if magic != _MAGIC:
                    raise ValueError("invalid APDU magic")
                if response_id != message_id:
                    raise ValueError("response APDU message ID does not match request")
                if encoding != _JSON:
                    raise ValueError("basic_server response is not JSON")
                payload = _recv_exact(connection, length)
        except (OSError, ValueError) as exc:
            logger.error("M20 basic_server request failed: %s", exc)
            return None
        device = _device(payload)
        if device is None:
            logger.error("M20 basic_server returned an invalid response")
        return device

    def _send(self, apdu: bytes) -> None:
        udp = self._udp
        if udp is None:
            raise OSError("the basic_server link is not running")
        udp.sendto(apdu, (self._host, self._udp_port))

    def _heartbeat(self) -> None:
        failing = False
        while not self._stop.is_set():
            _message_id, apdu = self._encode(_HEARTBEAT_TYPE, _HEARTBEAT_COMMAND, {})
            try:
                self._send(apdu)
            except OSError as exc:
                if not failing:
                    logger.warning("M20 basic_server heartbeat failed: %s", exc)
                failing = True
            else:
                if failing:
                    logger.info("M20 basic_server heartbeat recovered")
                failing = False
            self._stop.wait(_HEARTBEAT_PERIOD_S)

    def _read_reports(self) -> None:
        udp = self._udp
        if udp is None:
            return
        while not self._stop.is_set():
            try:
                frame = udp.recv(0xFFFF)
            except TimeoutError:
                continue
            except OSError as exc:
                logger.warning("M20 basic_server read failed: %s", exc)
                self._stop.wait(_POLL_S)
                continue
            self._handle_report(frame)

    def _handle_report(self, frame: bytes) -> None:
        device = _decode_report(frame)
        if device is None or device.get("Type") != _REPORT_TYPE:
            return
        items = device.get("Items")
        if not isinstance(items, dict):
            return
        command = device.get("Command")
        if command == _BASIC_STATUS_COMMAND:
            basic_status = items.get("BasicStatus")
            if not isinstance(basic_status, dict):
                return
            charge = basic_status.get("Charge")
            usage_mode = basic_status.get("ControlUsageMode")
            if isinstance(charge, int):
                self.charge = ChargeReport(charge, time.monotonic())
            if isinstance(usage_mode, int):
                self.usage_mode = usage_mode
        elif command == _ERROR_LIST_COMMAND:
            error_list = items.get("ErrorList")
            if not isinstance(error_list, list):
                return
            codes = [
                error.get("errorCode") if isinstance(error, dict) else None for error in error_list
            ]
            # A malformed list can't say an error cleared.
            if not all(isinstance(code, int) for code in codes):
                return
            # The list is every active error, so a charge error no longer listed has cleared.
            charge_errors = [code for code in codes if code in CHARGE_ERROR_NAMES]
            self.charge_error = charge_errors[-1] if charge_errors else None
        elif command == _BATTERY_STATUS_COMMAND:
            battery = _battery_report(items.get("BatteryStatus"))
            if battery is not None:
                self.battery = battery
                self._on_battery(battery)
            return
        else:
            return
        self._on_report()

    def _encode(self, message_type: int, command: int, items: dict[str, Any]) -> tuple[int, bytes]:
        payload = json.dumps(
            {
                "PatrolDevice": {
                    "Type": message_type,
                    "Command": command,
                    "Time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                    "Items": items,
                }
            },
            separators=(",", ":"),
        ).encode()
        if len(payload) > 0xFFFF:
            raise ValueError("M20 basic_server payload exceeds the APDU limit")

        message_id = self._message_ids.next() & 0xFFFF
        header = _HEADER.pack(_MAGIC, len(payload), message_id, _JSON, b"\0" * 7)
        return message_id, header + payload


def _battery_report(status: Any) -> BatteryReport | None:
    if not isinstance(status, dict):
        return None
    packs = [_pack_report(status, side) for side in ("Left", "Right")]
    if packs[0] is None or packs[1] is None:
        return None
    return BatteryReport(packs[0], packs[1], time.monotonic())


def _pack_report(status: dict[str, Any], side: str) -> PackReport | None:
    voltage = _number(status.get(f"Voltage{side}"))
    level = _number(status.get(f"BatteryLevel{side}"))
    temperature = _number(status.get(f"battery_temperature{side}"))
    if voltage is None or level is None or temperature is None:
        return None
    return PackReport(voltage, level, temperature)


def _number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)


def _decode_report(frame: bytes) -> dict[str, Any] | None:
    if len(frame) < _HEADER.size or not frame.startswith(_MAGIC):
        return None
    return _device(frame[_HEADER.size :])


def _device(payload: bytes) -> dict[str, Any] | None:
    try:
        decoded = json.loads(payload)
    except ValueError:
        return None
    device = decoded.get("PatrolDevice") if isinstance(decoded, dict) else None
    return device if isinstance(device, dict) else None


def _recv_exact(connection: socket.socket, size: int) -> bytes:
    result = bytearray()
    while len(result) < size:
        chunk = connection.recv(size - len(result))
        if not chunk:
            raise OSError("basic_server closed the connection before completing its response")
        result.extend(chunk)
    return bytes(result)
