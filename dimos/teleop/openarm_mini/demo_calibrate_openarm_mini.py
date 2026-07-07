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

"""Manual OpenArm Mini leader zero-calibration demo.

This script intentionally talks only to the OpenArm Mini leader Feetech bus. It
does not import or start ControlCoordinator, ManipulationModule, or follower
OpenArm hardware.
"""

from __future__ import annotations

import argparse
from collections.abc import Callable
from pathlib import Path
import time
from typing import Protocol

from dimos.teleop.openarm_mini.adapter import (
    _calibrated_motor_radians,
    _load_scservo_sdk,
    _read_motor_position,
    _ScservoPacketHandler,
    _ScservoPortHandler,
)
from dimos.teleop.openarm_mini.calibration import (
    OPENARM_MINI_ARM_JOINT_NAMES,
    OpenArmMiniCalibration,
    OpenArmMiniMotorCalibration,
    load_calibration,
    save_calibration,
)
from dimos.teleop.openarm_mini.config import OpenArmMiniTeleopConfig, default_calibration_path
from dimos.teleop.openarm_mini.mapping import map_side_readings

DEFAULT_MOTOR_IDS = {
    joint_name: index + 1 for index, joint_name in enumerate(OPENARM_MINI_ARM_JOINT_NAMES)
}
DEFAULT_FLIPS_BY_SIDE: dict[str, frozenset[str]] = {
    "left": frozenset(("joint_1", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7")),
    "right": frozenset(("joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6")),
}


class _RawPositionReader(Protocol):
    def connect(self) -> None: ...

    def disconnect(self) -> None: ...

    def read_raw_positions(self) -> dict[str, int]: ...


class _RawFeetechReader:
    def __init__(self, port: str, baudrate: int) -> None:
        self._port = port
        self._baudrate = baudrate
        self._port_handler: _ScservoPortHandler | None = None
        self._packet_handler: _ScservoPacketHandler | None = None

    def connect(self) -> None:
        sdk = _load_scservo_sdk()
        port_handler = sdk.PortHandler(self._port)
        packet_handler = sdk.sms_sts(port_handler)
        if not port_handler.openPort():
            raise RuntimeError(f"failed to open Feetech port {self._port}")
        if not port_handler.setBaudRate(self._baudrate):
            port_handler.closePort()
            raise RuntimeError(f"failed to set Feetech baudrate {self._baudrate}")
        self._port_handler = port_handler
        self._packet_handler = packet_handler

    def disconnect(self) -> None:
        if self._port_handler is None:
            return
        close_port = getattr(self._port_handler, "closePort", None)
        if callable(close_port):
            close_port()
        self._port_handler = None
        self._packet_handler = None

    def read_raw_positions(self) -> dict[str, int]:
        if self._packet_handler is None:
            raise RuntimeError("Feetech reader is not connected")
        return {
            joint_name: _read_motor_position(self._packet_handler, motor_id)
            for joint_name, motor_id in DEFAULT_MOTOR_IDS.items()
        }


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Zero-calibrate OpenArm Mini leader teleop.")
    parser.add_argument("--side", choices=("left", "right", "both"), default="both")
    parser.add_argument("--port-left", default=OpenArmMiniTeleopConfig.port_left)
    parser.add_argument("--port-right", default=OpenArmMiniTeleopConfig.port_right)
    parser.add_argument("--baudrate", type=int, default=OpenArmMiniTeleopConfig.baudrate)
    parser.add_argument(
        "--left-calibration-path", type=Path, default=default_calibration_path("left")
    )
    parser.add_argument(
        "--right-calibration-path", type=Path, default=default_calibration_path("right")
    )
    parser.add_argument(
        "--left-flips",
        help=(
            "Comma-separated left-side semantic joints to flip. Defaults to the "
            "known OpenArm Mini left leader orientation. Use 'none' for no flips."
        ),
    )
    parser.add_argument(
        "--right-flips",
        help=(
            "Comma-separated right-side semantic joints to flip. Defaults to the "
            "known OpenArm Mini right leader orientation. Use 'none' for no flips."
        ),
    )
    parser.add_argument(
        "--live-readout",
        action="store_true",
        help="Print calibrated arm-joint radians using existing calibration artifacts.",
    )
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    sides = ("left", "right") if args.side == "both" else (args.side,)
    print("OpenArm Mini leader calibration only connects to Feetech leader ports.")
    print("It never starts ControlCoordinator or connects follower OpenArm hardware.")
    print("Place each selected leader side in its natural zero pose before calibration.")
    for side in sides:
        port = args.port_left if side == "left" else args.port_right
        path = args.left_calibration_path if side == "left" else args.right_calibration_path
        flip_arg = args.left_flips if side == "left" else args.right_flips
        flips = _parse_flip_overrides(flip_arg, side)
        if args.live_readout:
            _live_readout(side, port, path, args.baudrate)
        else:
            _calibrate_side(side, port, path, args.baudrate, flips=flips)


def _calibrate_side(
    side: str,
    port: str,
    path: Path,
    baudrate: int,
    *,
    flips: set[str] | frozenset[str] | None = None,
    reader_factory: Callable[[str, int], _RawPositionReader] = _RawFeetechReader,
) -> None:
    reader = reader_factory(port, baudrate)
    reader.connect()
    try:
        print(f"\nCalibrating {side} OpenArm Mini leader on {port}")
        print("Place the leader in its natural zero pose; reading arm-joint motors now.")
        raw_positions = reader.read_raw_positions()
        calibration = _capture_zero_calibration(
            side,
            raw_positions,
            flips if flips is not None else DEFAULT_FLIPS_BY_SIDE[side],
        )
        artifact_path = save_calibration(path, calibration)
        print(_format_calibration_confirmation(calibration))
        print(f"Wrote {side} calibration to {artifact_path}")
    finally:
        reader.disconnect()


def _live_readout(side: str, port: str, path: Path, baudrate: int) -> None:
    calibration = load_calibration(path, side)
    reader = _RawFeetechReader(port, baudrate)
    reader.connect()
    try:
        print(f"\nLive calibrated {side} arm readout from {port}; press Ctrl-C to stop.")
        while True:
            raw_positions = reader.read_raw_positions()
            calibrated_readings = {
                joint_name: _calibrated_motor_radians(raw_position, calibration.motors[joint_name])
                for joint_name, raw_position in raw_positions.items()
            }
            command = map_side_readings(side, calibrated_readings)
            print(
                " ".join(
                    f"{joint_name}={position:+.3f}rad"
                    for joint_name, position in command.positions_by_joint.items()
                )
            )
            time.sleep(0.25)
    except KeyboardInterrupt:
        print("\nStopped live readout.")
    finally:
        reader.disconnect()


def _capture_zero_calibration(
    side: str,
    raw_positions: dict[str, int],
    flips: set[str] | frozenset[str],
) -> OpenArmMiniCalibration:
    _validate_raw_positions(raw_positions)
    invalid_flips = set(flips) - set(OPENARM_MINI_ARM_JOINT_NAMES)
    if invalid_flips:
        raise RuntimeError(f"unknown OpenArm Mini flip joints: {sorted(invalid_flips)}")
    return OpenArmMiniCalibration(
        side=side,
        motors={
            joint_name: OpenArmMiniMotorCalibration(
                id=DEFAULT_MOTOR_IDS[joint_name],
                homing_offset=raw_positions[joint_name],
                flip=joint_name in flips,
            )
            for joint_name in OPENARM_MINI_ARM_JOINT_NAMES
        },
    )


def _format_calibration_confirmation(calibration: OpenArmMiniCalibration) -> str:
    lines = [
        f"Captured {calibration.side} OpenArm Mini leader zero offsets:",
        f"{'Joint':<10} {'ID':>2} {'Zero Raw':>8} {'Flip':>5}",
        "-" * 31,
    ]
    for joint_name in OPENARM_MINI_ARM_JOINT_NAMES:
        motor = calibration.motors[joint_name]
        lines.append(f"{joint_name:<10} {motor.id:>2} {motor.homing_offset:>8} {motor.flip!s:>5}")
    return "\n".join(lines)


def _parse_flip_overrides(value: str | None, side: str) -> set[str]:
    if value is None:
        return set(DEFAULT_FLIPS_BY_SIDE[side])
    stripped = value.strip()
    if not stripped or stripped.lower() == "none":
        return set()
    flips = {entry.strip() for entry in stripped.split(",") if entry.strip()}
    invalid = flips - set(OPENARM_MINI_ARM_JOINT_NAMES)
    if invalid:
        raise RuntimeError(f"unknown OpenArm Mini flip joints: {sorted(invalid)}")
    return flips


def _validate_raw_positions(raw_positions: dict[str, int]) -> None:
    missing = set(OPENARM_MINI_ARM_JOINT_NAMES) - set(raw_positions)
    extra = set(raw_positions) - set(OPENARM_MINI_ARM_JOINT_NAMES)
    if missing or extra:
        raise RuntimeError(
            "OpenArm Mini raw readings must contain exactly arm joints "
            f"{list(OPENARM_MINI_ARM_JOINT_NAMES)}; missing={sorted(missing)}, extra={sorted(extra)}"
        )


if __name__ == "__main__":
    main()
