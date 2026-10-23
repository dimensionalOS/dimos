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

"""Read-only diagnostics for the Seeed Studio reBot B601-DM."""

from __future__ import annotations

from dataclasses import dataclass

import typer

from dimos.hardware.manipulators.seeedstudio.adapter import (
    ARM_DOF,
    ARM_LOWER,
    ARM_UPPER,
    gripper_turn_offset,
)
from dimos.hardware.manipulators.seeedstudio.protocol import (
    FEEDBACK_IDS,
    JOINT_NAMES,
    MOTOR_IDS,
    POS_VEL_MODE,
    STATUS_NAMES,
    DmSerialTransport,
)

app = typer.Typer(help="Seeed Studio reBot B601-DM robot commands")

_USB_VENDOR_ID = 0x2E88
_USB_PRODUCT_ID = 0x4603
_GUIDE = "docs/capabilities/manipulation/seeedstudio.md"


@dataclass(frozen=True)
class MotorReport:
    joint: str
    motor_id: int
    mode: int | None = None
    status: int | None = None
    position: float | None = None
    mos_temperature: int | None = None
    error: str | None = None


def find_bridge_ports() -> list[str]:
    """Serial devices whose USB identity matches the HDSC CAN bridge."""
    # pyserial ships with the optional `control` extra; keep `dimos hardware` importable without it.
    from serial.tools import list_ports

    return [
        p.device
        for p in list_ports.comports()
        if p.vid == _USB_VENDOR_ID and p.pid == _USB_PRODUCT_ID
    ]


def inspect_motors(port: str, timeout: float = 0.15) -> list[MotorReport]:
    """Send only parameter reads and feedback requests, then close the port.

    Each motor is reported independently so a missing downstream cable is
    visible. Never enables, disables, zeroes, clears faults or writes config.
    """
    bus = DmSerialTransport(port, timeout)
    reports: list[MotorReport] = []
    try:
        bus.open()
        for name, mid, fid in zip(JOINT_NAMES, MOTOR_IDS, FEEDBACK_IDS, strict=True):
            try:
                params = bus.parameters(mid, fid)
                feedback = bus.feedback(params)
            except (OSError, ValueError) as exc:
                reports.append(MotorReport(name, mid, error=str(exc)))
                continue
            reports.append(
                MotorReport(
                    name,
                    mid,
                    mode=params.mode,
                    status=feedback.status,
                    position=feedback.position,
                    mos_temperature=feedback.mos_temperature,
                )
            )
    finally:
        bus.close()
    return reports


def _problems(report: MotorReport) -> list[str]:
    if report.error is not None:
        return [report.error]
    problems = []
    if report.mode != POS_VEL_MODE:
        problems.append(f"control mode {report.mode}, expected POS_VEL ({POS_VEL_MODE})")
    if report.status not in (0, 1):
        problems.append(f"fault {STATUS_NAMES.get(report.status or 0, report.status)}")
    arm_index = report.motor_id - 1
    if arm_index < len(ARM_LOWER) and report.position is not None:
        lower, upper = ARM_LOWER[arm_index], ARM_UPPER[arm_index]
        if not lower <= report.position <= upper:
            problems.append(f"position outside [{lower}, {upper}] rad")
    return problems


def _position_text(report: MotorReport) -> str:
    assert report.position is not None, "healthy reports carry a position"
    if report.motor_id - 1 != ARM_DOF:
        return f"{report.position:+.4f} rad"
    # The gripper's travel crosses the motor's one-turn wrap at power-up.
    offset = gripper_turn_offset(report.position)
    if not offset:
        return f"{report.position:+.4f} rad"
    return f"{report.position - offset:+.4f} rad in travel (reported {report.position:+.4f})"


@app.command()
def doctor(
    port: str | None = typer.Argument(
        None, help="Serial port of the CAN bridge. Omit to find it by USB identity."
    ),
    timeout: float = typer.Option(0.15, "--timeout", help="Per-reply timeout in seconds"),
) -> None:
    """Check the USB bridge and all seven motors without moving anything."""
    if port is None:
        ports = find_bridge_ports()
        if len(ports) != 1:
            typer.echo(
                f"ERROR: expected one HDSC CAN bridge, found {len(ports)}: {ports}. "
                "Pass the port explicitly.",
                err=True,
            )
            raise typer.Exit(1)
        port = ports[0]
    typer.echo(f"Port: {port}")
    try:
        reports = inspect_motors(port, timeout)
    except (OSError, ValueError) as exc:
        typer.echo(f"ERROR: cannot open {port}: {exc}", err=True)
        raise typer.Exit(1) from exc

    failures = 0
    for report in reports:
        problems = _problems(report)
        failures += bool(problems)
        detail = (
            "; ".join(problems)
            if problems
            else f"{STATUS_NAMES.get(report.status or 0, report.status)}, "
            f"{_position_text(report)}, {report.mos_temperature} C"
        )
        typer.echo(
            f"{'FAIL' if problems else 'PASS'} {report.joint} (motor {report.motor_id}): {detail}"
        )
    if failures:
        typer.echo(f"Seeed doctor found {failures} problem(s). See {_GUIDE}.", err=True)
        raise typer.Exit(1)
    typer.echo("Seeed doctor passed.")
