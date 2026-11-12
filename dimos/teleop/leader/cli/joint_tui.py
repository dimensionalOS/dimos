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

"""Rich TUI for inspecting calibrated leader arm joints.

This helper only connects to the leader Feetech ports. It does not start
ControlCoordinator and does not connect follower hardware.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import time
from typing import TYPE_CHECKING

import typer

if TYPE_CHECKING:
    from rich.console import Group

from dimos.teleop.leader.calibration import (
    LeaderCalibration,
    LeaderSide,
    default_calibration_path,
    load_calibration,
)
from dimos.teleop.leader.cli._errors import exit_for_missing_dependency
from dimos.teleop.leader.feetech import (
    FEETECH_DEFAULT_BAUDRATE,
    FeetechLeaderReader,
    LeaderDependencyError,
    calibrated_motor_radians,
)


@dataclass(frozen=True)
class LeaderJointRow:
    side: str
    joint: str
    motor_id: int
    raw: int
    radians: float
    flip: bool


def main(
    side: LeaderSide = typer.Option(..., help="Leader side to inspect."),
    port: str = typer.Option(..., help="Leader Feetech serial port."),
    baudrate: int = typer.Option(
        FEETECH_DEFAULT_BAUDRATE,
        help="Feetech serial baudrate.",
    ),
    calibration_path: Path | None = typer.Option(
        None,
        help="Calibration directory or JSON file. Defaults to the selected side calibration.",
    ),
    refresh_hz: float = typer.Option(10.0),
) -> None:
    """Display one leader side in a Rich TUI."""
    try:
        _run(
            side=side,
            port=port,
            baudrate=baudrate,
            calibration_path=calibration_path,
            refresh_hz=refresh_hz,
        )
    except LeaderDependencyError as error:
        exit_for_missing_dependency(error)


def _run(
    *,
    side: LeaderSide,
    port: str,
    baudrate: int,
    calibration_path: Path | None,
    refresh_hz: float,
) -> None:
    # Deferred because this command module is imported by the global hardware CLI.
    from rich.live import Live

    refresh_seconds = 1.0 / refresh_hz
    calibration = _load_tui_calibration(side, _resolve_calibration_path(side, calibration_path))
    motor_ids = {name: motor.id for name, motor in calibration.motors.items()}
    reader = FeetechLeaderReader(port, baudrate)
    try:
        reader.connect()

        with Live(refresh_per_second=refresh_hz, screen=True) as live:
            while True:
                rows = _read_side_rows(calibration, reader.read_raw_positions(motor_ids))
                live.update(_build_joint_dashboard(rows))
                time.sleep(refresh_seconds)
    except KeyboardInterrupt:
        pass
    finally:
        reader.disconnect()


def _resolve_calibration_path(side: LeaderSide, calibration_path: Path | None) -> Path:
    if calibration_path is not None:
        return calibration_path
    return default_calibration_path(side)


def _load_tui_calibration(
    side: LeaderSide,
    calibration_path: Path,
) -> LeaderCalibration:
    return load_calibration(calibration_path, side)


def _read_side_rows(
    calibration: LeaderCalibration,
    raw_positions: dict[str, int],
) -> list[LeaderJointRow]:
    return [
        LeaderJointRow(
            side=calibration.side,
            joint=motor_name,
            motor_id=motor.id,
            raw=raw_positions[motor_name],
            radians=calibrated_motor_radians(raw_positions[motor_name], motor),
            flip=motor.flip,
        )
        for motor_name, motor in calibration.motors.items()
    ]


def _build_joint_dashboard(rows: list[LeaderJointRow]) -> Group:
    # Deferred because this command module is imported by the global hardware CLI.
    from rich.console import Group
    from rich.panel import Panel
    from rich.table import Table
    from rich.text import Text

    table = Table(title="Leader joint readout", expand=True)
    table.add_column("Side", style="cyan", no_wrap=True)
    table.add_column("Motor", no_wrap=True)
    table.add_column("ID", justify="right")
    table.add_column("Raw", justify="right")
    table.add_column("Rad", justify="right")
    table.add_column("Flip", justify="center")
    for row in rows:
        table.add_row(
            row.side,
            row.joint,
            str(row.motor_id),
            str(row.raw),
            f"{row.radians:+.3f}",
            "yes" if row.flip else "no",
        )
    help_text = Text(
        "Leader only: reads the Feetech motors named in the calibration, shows raw ticks "
        "and calibrated radians. Ctrl-C to exit.",
        style="dim",
    )
    return Group(Panel(help_text, title="Leader joint TUI"), table)


if __name__ == "__main__":
    typer.run(main)
