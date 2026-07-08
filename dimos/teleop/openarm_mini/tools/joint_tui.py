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

from dataclasses import dataclass
from pathlib import Path
import time

from rich.console import Group
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text
import typer

from dimos.teleop.openarm_mini.calibration import (
    OPENARM_MINI_ARM_JOINT_NAMES,
    OpenArmMiniCalibration,
    OpenArmMiniSide,
    default_calibration_path,
    load_calibration,
)
from dimos.teleop.openarm_mini.feetech import (
    OPENARM_MINI_DEFAULT_BAUDRATE,
    FeetechLeaderReader,
    _calibrated_motor_radians,
)
from dimos.teleop.openarm_mini.mapping import map_side_readings
from dimos.teleop.openarm_mini.tools.calibrate import DEFAULT_MOTOR_IDS


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


def main(
    side: OpenArmMiniSide = typer.Option(..., help="Leader side to inspect."),
    port: str = typer.Option(..., help="Leader Feetech serial port."),
    baudrate: int = typer.Option(
        OPENARM_MINI_DEFAULT_BAUDRATE,
        help="Feetech serial baudrate.",
    ),
    calibration_path: Path | None = typer.Option(
        None,
        help="Calibration directory or JSON file. Defaults to the selected side calibration.",
    ),
    refresh_hz: float = typer.Option(10.0),
) -> None:
    """Display one OpenArm Mini leader side in a Rich TUI."""
    refresh_seconds = 1.0 / refresh_hz
    calibration = _load_tui_calibration(side, _resolve_calibration_path(side, calibration_path))
    reader = FeetechLeaderReader(port, baudrate)
    try:
        reader.connect()

        with Live(refresh_per_second=refresh_hz, screen=True) as live:
            while True:
                rows = _read_side_rows(
                    calibration,
                    reader.read_raw_positions(DEFAULT_MOTOR_IDS),
                )
                live.update(_build_joint_dashboard(rows))
                time.sleep(refresh_seconds)
    except KeyboardInterrupt:
        pass
    finally:
        reader.disconnect()


def _resolve_calibration_path(side: OpenArmMiniSide, calibration_path: Path | None) -> Path:
    if calibration_path is not None:
        return calibration_path
    return default_calibration_path(side)


def _load_tui_calibration(
    side: OpenArmMiniSide,
    calibration_path: Path,
) -> OpenArmMiniCalibration:
    return load_calibration(calibration_path, side)


def _read_side_rows(
    calibration: OpenArmMiniCalibration,
    raw_positions: dict[str, int],
) -> list[OpenArmMiniJointRow]:
    side = calibration.side
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
    typer.run(main)
