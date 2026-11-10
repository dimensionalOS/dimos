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

"""Manual leader zero-calibration utility.

This script intentionally talks only to the leader Feetech bus. It
does not import or start ControlCoordinator, ManipulationModule, or follower
follower hardware.
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
import time
from typing import Any, Literal

import typer

from dimos.teleop.leader.calibration import (
    DEFAULT_LEADER_MOTOR_NAMES,
    LEADER_GRIPPER_MOTOR_NAME,
    LeaderCalibration,
    LeaderMotorCalibration,
    LeaderSide,
    default_calibration_path,
    load_calibration,
    save_calibration,
)
from dimos.teleop.leader.cli._errors import exit_for_missing_dependency
from dimos.teleop.leader.feetech import (
    FeetechLeaderReader,
    LeaderDependencyError,
    calibrated_motor_radians,
)

DEFAULT_MOTOR_IDS = {
    joint_name: index + 1 for index, joint_name in enumerate(DEFAULT_LEADER_MOTOR_NAMES)
}
# The trigger servo, when fitted, takes the next ID after the arm joints.
GRIPPER_MOTOR_ID = {LEADER_GRIPPER_MOTOR_NAME: len(DEFAULT_LEADER_MOTOR_NAMES) + 1}
MOTOR_IDS = DEFAULT_MOTOR_IDS | GRIPPER_MOTOR_ID
# Which motors read backwards is a property of each printed arm; pass --left-flips / --right-flips.
DEFAULT_FLIPS_BY_SIDE: dict[LeaderSide, frozenset[str]] = {
    "left": frozenset(),
    "right": frozenset(),
}


def main(
    side: Literal["left", "right", "both"] = typer.Option("both"),
    port_left: str | None = typer.Option(None, help="Left leader Feetech serial port."),
    port_right: str | None = typer.Option(None, help="Right leader Feetech serial port."),
    baudrate: int = typer.Option(..., help="Feetech serial baudrate."),
    left_calibration_path: Path = typer.Option(default_calibration_path("left")),
    right_calibration_path: Path = typer.Option(default_calibration_path("right")),
    left_flips: str | None = typer.Option(
        None,
        help=("Comma-separated left-side motors to flip, e.g. joint_1,joint_7. Default: none."),
    ),
    right_flips: str | None = typer.Option(
        None,
        help=("Comma-separated right-side motors to flip, e.g. joint_1,joint_7. Default: none."),
    ),
    live_readout: bool = typer.Option(
        False,
        help="Print calibrated arm-joint radians using existing calibration artifacts.",
    ),
) -> None:
    """Zero-calibrate a pair of Feetech leader arms."""
    try:
        _run(
            side=side,
            port_left=port_left,
            port_right=port_right,
            baudrate=baudrate,
            left_calibration_path=left_calibration_path,
            right_calibration_path=right_calibration_path,
            left_flips=left_flips,
            right_flips=right_flips,
            live_readout=live_readout,
        )
    except LeaderDependencyError as error:
        exit_for_missing_dependency(error)


def _run(
    *,
    side: Literal["left", "right", "both"],
    port_left: str | None,
    port_right: str | None,
    baudrate: int,
    left_calibration_path: Path,
    right_calibration_path: Path,
    left_flips: str | None,
    right_flips: str | None,
    live_readout: bool,
) -> None:
    sides: tuple[LeaderSide, ...]
    if side == "both":
        sides = ("left", "right")
    elif side == "left":
        sides = ("left",)
    else:
        sides = ("right",)
    print("Leader calibration only connects to the Feetech leader ports.")
    print("It never starts ControlCoordinator or connects follower hardware.")
    print("Place each selected leader side in its natural zero pose before calibration.")
    for selected_side in sides:
        port = _selected_port(selected_side, port_left, port_right)
        path = left_calibration_path if selected_side == "left" else right_calibration_path
        flip_arg = left_flips if selected_side == "left" else right_flips
        flips = _parse_flip_overrides(flip_arg, selected_side)
        if live_readout:
            _live_readout(selected_side, port, path, baudrate)
        else:
            _calibrate_side(selected_side, port, path, baudrate, flips=flips)


def _selected_port(side: LeaderSide, port_left: str | None, port_right: str | None) -> str:
    port = port_left if side == "left" else port_right
    if port is None:
        raise typer.BadParameter(f"--port-{side} is required for --side {side}")
    return port


def _calibrate_side(
    side: LeaderSide,
    port: str,
    path: Path,
    baudrate: int,
    *,
    flips: set[str] | frozenset[str] | None = None,
    reader_factory: Callable[[str, int], Any] = FeetechLeaderReader,
) -> None:
    reader = reader_factory(port, baudrate)
    reader.connect()
    try:
        print(f"\nCalibrating {side} leader on {port}")
        print("Place the leader in its natural zero pose, trigger released; reading motors now.")
        raw_positions = _read_leader(reader)
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


def _live_readout(side: LeaderSide, port: str, path: Path, baudrate: int) -> None:
    calibration = load_calibration(path, side)
    reader = FeetechLeaderReader(port, baudrate)
    reader.connect()
    try:
        print(f"\nLive calibrated {side} leader readout from {port}; press Ctrl-C to stop.")
        motor_ids = {name: motor.id for name, motor in calibration.motors.items()}
        while True:
            raw_positions = reader.read_raw_positions(motor_ids)
            print(
                " ".join(
                    f"{name}={calibrated_motor_radians(raw, calibration.motors[name]):+.3f}rad"
                    for name, raw in raw_positions.items()
                )
            )
            time.sleep(0.25)
    except KeyboardInterrupt:
        print("\nStopped live readout.")
    finally:
        reader.disconnect()


def _capture_zero_calibration(
    side: LeaderSide,
    raw_positions: dict[str, int],
    flips: set[str] | frozenset[str],
) -> LeaderCalibration:
    _validate_raw_positions(raw_positions)
    invalid_flips = set(flips) - set(DEFAULT_LEADER_MOTOR_NAMES)
    if invalid_flips:
        raise RuntimeError(f"unknown flip motors: {sorted(invalid_flips)}")
    return LeaderCalibration(
        side=side,
        motors={
            motor_name: LeaderMotorCalibration(
                id=MOTOR_IDS[motor_name],
                homing_offset=raw_positions[motor_name],
                flip=motor_name in flips,
            )
            for motor_name in MOTOR_IDS
            if motor_name in raw_positions
        },
    )


def _read_leader(reader: Any) -> dict[str, int]:
    """Raw arm-joint ticks, plus the gripper trigger when a servo answers at its ID."""
    raw_positions = dict(reader.read_raw_positions(DEFAULT_MOTOR_IDS))
    try:
        raw_positions.update(reader.read_raw_positions(GRIPPER_MOTOR_ID))
    except RuntimeError:
        print("No gripper trigger servo answered; calibrating arm joints only.")
    return raw_positions


def _format_calibration_confirmation(calibration: LeaderCalibration) -> str:
    lines = [
        f"Captured {calibration.side} leader zero offsets:",
        f"{'Motor':<10} {'ID':>2} {'Zero Raw':>8} {'Flip':>5}",
        "-" * 31,
    ]
    for motor_name, motor in calibration.motors.items():
        lines.append(f"{motor_name:<10} {motor.id:>2} {motor.homing_offset:>8} {motor.flip!s:>5}")
    return "\n".join(lines)


def _parse_flip_overrides(value: str | None, side: LeaderSide) -> set[str]:
    if value is None:
        return set(DEFAULT_FLIPS_BY_SIDE[side])
    stripped = value.strip()
    if not stripped or stripped.lower() == "none":
        return set()
    flips = {entry.strip() for entry in stripped.split(",") if entry.strip()}
    invalid = flips - set(DEFAULT_LEADER_MOTOR_NAMES)
    if invalid:
        raise RuntimeError(f"unknown flip motors: {sorted(invalid)}")
    return flips


def _validate_raw_positions(raw_positions: dict[str, int]) -> None:
    missing = set(DEFAULT_LEADER_MOTOR_NAMES) - set(raw_positions)
    extra = set(raw_positions) - set(MOTOR_IDS)
    if missing or extra:
        raise RuntimeError(
            f"raw readings must contain the arm motors {list(DEFAULT_LEADER_MOTOR_NAMES)} "
            f"and optionally {LEADER_GRIPPER_MOTOR_NAME!r}; "
            f"missing={sorted(missing)}, extra={sorted(extra)}"
        )


if __name__ == "__main__":
    typer.run(main)
