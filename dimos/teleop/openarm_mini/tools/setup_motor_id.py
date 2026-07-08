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

"""One-shot Feetech motor ID setup for an OpenArm Mini leader motor.

Connect exactly one Feetech motor to the USB controller before running this
script. Writing IDs while multiple motors are attached can address the wrong
device when IDs collide.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

import typer

from dimos.teleop.openarm_mini.feetech import _create_sdk_handlers

FEETECH_ID_ADDRESS = 5
FEETECH_TORQUE_ENABLE_ADDRESS = 40
FEETECH_MIN_MOTOR_ID = 1
FEETECH_MAX_MOTOR_ID = 253
FEETECH_COMM_SUCCESS = 0


def main(
    port: str = typer.Option(..., help="Feetech serial port, e.g. /dev/ttyUSB1"),
    new_id: int = typer.Option(..., "--new-id", help="Target Feetech motor ID"),
    old_id: int | None = typer.Option(
        None,
        "--old-id",
        help="Current motor ID. If omitted, scan for exactly one connected motor.",
    ),
    baudrate: int = typer.Option(..., help="Feetech serial baudrate."),
    yes: bool = typer.Option(False, "--yes", help="Skip the safety confirmation prompt."),
) -> None:
    _validate_motor_id(new_id, "new-id")
    if old_id is not None:
        _validate_motor_id(old_id, "old-id")

    if not yes:
        print("Connect exactly ONE Feetech motor to the controller before continuing.")
        print("If multiple motors share an ID, this write can affect the wrong motor(s).")
        input("Press Enter to continue or Ctrl-C to abort.")

    setup_motor_id(port=port, baudrate=baudrate, new_id=new_id, old_id=old_id)


def setup_motor_id(port: str, baudrate: int, new_id: int, old_id: int | None = None) -> int:
    """Set one connected Feetech motor to ``new_id``.

    Returns the detected or provided previous motor ID.
    """
    _validate_motor_id(new_id, "new-id")
    if old_id is not None:
        _validate_motor_id(old_id, "old-id")

    port_handler, packet_handler = _create_sdk_handlers(port)
    if not port_handler.openPort():
        raise RuntimeError(f"failed to open Feetech port {port}")
    try:
        if not port_handler.setBaudRate(baudrate):
            raise RuntimeError(f"failed to set Feetech baudrate {baudrate}")
        motor_id = old_id if old_id is not None else find_single_motor_id(packet_handler)
        write_motor_id(packet_handler, motor_id, new_id)
    finally:
        port_handler.closePort()

    print(f"Feetech motor ID set: {motor_id} -> {new_id}")
    return motor_id


def find_single_motor_id(packet_handler: Any) -> int:
    """Scan the Feetech bus and return the only responding motor ID."""
    found_ids = [
        motor_id
        for motor_id in range(FEETECH_MIN_MOTOR_ID, FEETECH_MAX_MOTOR_ID + 1)
        if ping_motor_id(packet_handler, motor_id)
    ]
    if not found_ids:
        raise RuntimeError("no Feetech motor responded during ID scan")
    if len(found_ids) > 1:
        raise RuntimeError(
            "multiple Feetech motors responded during ID scan: "
            f"{found_ids}. Connect exactly one motor before setting IDs."
        )
    return found_ids[0]


def ping_motor_id(packet_handler: Any, motor_id: int) -> bool:
    """Return whether ``motor_id`` responds successfully to Feetech ping."""
    _validate_motor_id(motor_id, "motor-id")
    model_number, comm_result, error = packet_handler.ping(motor_id)
    return bool(comm_result == FEETECH_COMM_SUCCESS and error == 0 and model_number != 0)


def write_motor_id(packet_handler: Any, old_id: int, new_id: int) -> None:
    """Disable torque, unlock EEPROM, write the new ID, lock, and verify."""
    _validate_motor_id(old_id, "old-id")
    _validate_motor_id(new_id, "new-id")
    if not ping_motor_id(packet_handler, old_id):
        raise RuntimeError(f"Feetech motor {old_id} did not respond before ID write")
    if old_id == new_id:
        print(f"Feetech motor is already ID {new_id}; no write needed.")
        return

    _ensure_success(
        "disable torque",
        packet_handler.write1ByteTxRx(old_id, FEETECH_TORQUE_ENABLE_ADDRESS, 0),
    )
    _ensure_success("unlock EEPROM", packet_handler.unLockEprom(old_id))
    _ensure_success(
        "write motor ID",
        packet_handler.write1ByteTxRx(old_id, FEETECH_ID_ADDRESS, new_id),
    )
    _ensure_success("lock EEPROM", packet_handler.LockEprom(new_id))
    if not ping_motor_id(packet_handler, new_id):
        raise RuntimeError(f"Feetech motor {new_id} did not respond after ID write")


def _ensure_success(operation: str, result: object) -> None:
    if not _is_success_result(result):
        raise RuntimeError(f"Feetech {operation} failed with result {result!r}")


def _is_success_result(result: object) -> bool:
    if not isinstance(result, Sequence) or isinstance(result, (str, bytes)):
        return False
    if len(result) < 2:
        return False
    comm_result = result[-2]
    error = result[-1]
    return bool(comm_result == FEETECH_COMM_SUCCESS and error == 0)


def _validate_motor_id(motor_id: int, label: str) -> None:
    if not FEETECH_MIN_MOTOR_ID <= motor_id <= FEETECH_MAX_MOTOR_ID:
        raise ValueError(
            f"{label} must be in [{FEETECH_MIN_MOTOR_ID}, {FEETECH_MAX_MOTOR_ID}], got {motor_id}"
        )


if __name__ == "__main__":
    typer.run(main)
