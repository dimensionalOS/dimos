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

"""Rich TUI for inspecting calibrated OpenArm Mini leader arm joints.

This helper only connects to OpenArm Mini leader Feetech ports. It does not start
ControlCoordinator and does not connect follower OpenArm hardware.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import time

from rich.console import Group
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from dimos.teleop.openarm_mini.adapter import _calibrated_motor_radians
from dimos.teleop.openarm_mini.calibration import OPENARM_MINI_ARM_JOINT_NAMES, load_calibration
from dimos.teleop.openarm_mini.config import OpenArmMiniTeleopConfig, default_calibration_path
from dimos.teleop.openarm_mini.demo_calibrate_openarm_mini import _RawFeetechReader
from dimos.teleop.openarm_mini.mapping import map_side_readings


@dataclass(frozen=True)
class OpenArmMiniJointRow:
    side: str
    joint: str
    follower_joint: str
    motor_id: int
    raw: int
    radians: float
    clamped_radians: float
    flip: bool


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Display OpenArm Mini leader joints in a Rich TUI."
    )
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
    parser.add_argument("--refresh-hz", type=float, default=10.0)
    return parser.parse_args()


def main() -> None:
    args = _parse_args()
    refresh_seconds = 1.0 / args.refresh_hz
    sides = ("left", "right") if args.side == "both" else (args.side,)
    readers: dict[str, _RawFeetechReader] = {}
    calibration_paths: dict[str, Path] = {
        "left": args.left_calibration_path,
        "right": args.right_calibration_path,
    }
    try:
        for side in sides:
            port = args.port_left if side == "left" else args.port_right
            reader = _RawFeetechReader(port, args.baudrate)
            reader.connect()
            readers[side] = reader

        with Live(refresh_per_second=args.refresh_hz, screen=True) as live:
            while True:
                rows: list[OpenArmMiniJointRow] = []
                for side, reader in readers.items():
                    rows.extend(
                        _read_side_rows(side, calibration_paths[side], reader.read_raw_positions())
                    )
                live.update(_build_joint_dashboard(rows))
                time.sleep(refresh_seconds)
    except KeyboardInterrupt:
        pass
    finally:
        for reader in readers.values():
            reader.disconnect()


def _read_side_rows(
    side: str,
    calibration_path: Path,
    raw_positions: dict[str, int],
) -> list[OpenArmMiniJointRow]:
    calibration = load_calibration(calibration_path, side)
    calibrated_readings = {
        joint_name: _calibrated_motor_radians(
            raw_positions[joint_name], calibration.motors[joint_name]
        )
        for joint_name in OPENARM_MINI_ARM_JOINT_NAMES
    }
    command = map_side_readings(side, calibrated_readings)
    return [
        OpenArmMiniJointRow(
            side=side,
            joint=joint_name,
            follower_joint=follower_joint,
            motor_id=calibration.motors[joint_name].id,
            raw=raw_positions[joint_name],
            radians=calibrated_readings[joint_name],
            clamped_radians=command.positions_by_joint[follower_joint],
            flip=calibration.motors[joint_name].flip,
        )
        for joint_name, follower_joint in zip(
            OPENARM_MINI_ARM_JOINT_NAMES,
            command.positions_by_joint,
            strict=True,
        )
    ]


def _build_joint_dashboard(rows: list[OpenArmMiniJointRow]) -> Group:
    table = Table(title="OpenArm Mini leader joint readout", expand=True)
    table.add_column("Side", style="cyan", no_wrap=True)
    table.add_column("Joint", no_wrap=True)
    table.add_column("Follower Joint", no_wrap=True)
    table.add_column("ID", justify="right")
    table.add_column("Raw", justify="right")
    table.add_column("Rad", justify="right")
    table.add_column("Clamped Rad", justify="right")
    table.add_column("Flip", justify="center")
    for row in rows:
        clamp_style = "yellow" if abs(row.radians - row.clamped_radians) > 1e-9 else "green"
        table.add_row(
            row.side,
            row.joint,
            row.follower_joint,
            str(row.motor_id),
            str(row.raw),
            f"{row.radians:+.3f}",
            f"[{clamp_style}]{row.clamped_radians:+.3f}[/{clamp_style}]",
            "yes" if row.flip else "no",
        )
    help_text = Text(
        "Leader only: reads Feetech arm joints from calibration, displays raw ticks, "
        "calibrated radians, and sender-side clamped follower radians. Ctrl-C to exit.",
        style="dim",
    )
    return Group(Panel(help_text, title="OpenArm Mini Joint TUI"), table)


if __name__ == "__main__":
    main()
