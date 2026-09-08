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

"""CAN interface discovery and Linux SocketCAN management commands."""

from __future__ import annotations

import os
import shlex
import subprocess
import sys

import can_motor_control
import typer

app = typer.Typer(help="Discover and configure CAN interfaces", no_args_is_help=True)

GS_USB_VENDOR_ID = 0x1D50
GS_USB_PRODUCT_ID = 0x606F


def _run_ip(*args: str, privileged: bool = False) -> subprocess.CompletedProcess[str]:
    command = ["ip", *args]
    if privileged and os.geteuid() != 0:
        command = ["sudo", "--", *command]
    if privileged:
        typer.echo(f"Running: {shlex.join(command)}")
    try:
        # Let sudo prompt and report errors through the caller's terminal.
        return subprocess.run(
            command,
            check=True,
            capture_output=not privileged,
            text=True,
        )
    except FileNotFoundError as exc:
        executable = command[0]
        raise typer.BadParameter(f"the '{executable}' command is not installed") from exc
    except subprocess.CalledProcessError as exc:
        stderr = exc.stderr.strip() if exc.stderr else ""
        stdout = exc.stdout.strip() if exc.stdout else ""
        detail = stderr or stdout or f"exit code {exc.returncode}"
        typer.echo(f"CAN interface command failed: {detail}", err=True)
        raise typer.Exit(1) from exc


@app.command("list")
def list_devices() -> None:
    """List selectable CAN devices for this host."""
    if sys.platform == "linux":
        result = _run_ip("-brief", "link", "show", "type", "can")
        output = result.stdout.rstrip()
        typer.echo(output or "No SocketCAN interfaces found")
        return
    if sys.platform == "darwin":
        try:
            devices = can_motor_control.list_gs_usb_devices(
                vendor_id=GS_USB_VENDOR_ID,
                product_id=GS_USB_PRODUCT_ID,
            )
        except can_motor_control.TransportError as exc:
            typer.echo(f"CAN device discovery failed: {exc}", err=True)
            raise typer.Exit(1) from exc
        if not devices:
            typer.echo("No gs_usb adapters found (expected USB ID 1d50:606f)")
            return
        typer.echo("INDEX  SERIAL")
        for device in devices:
            typer.echo(f"{device.index:<5}  {device.serial_number or '<missing>'}")
        serials = [device.serial_number for device in devices]
        if not all(serials) or len(serials) != len(set(serials)):
            typer.echo(
                "Adapters cannot be assigned reliably unless their firmware exposes "
                "unique, non-empty USB serials.",
                err=True,
            )
        return
    typer.echo(f"CAN discovery is unsupported on {sys.platform}", err=True)
    raise typer.Exit(1)


@app.command("status")
def status(interface: str = typer.Argument(..., help="Linux CAN interface name")) -> None:
    """Show detailed CAN interface state and queue statistics."""
    result = _run_ip("-details", "-statistics", "link", "show", "dev", interface)
    typer.echo(result.stdout.rstrip())


@app.command("down")
def down(interface: str = typer.Argument(..., help="Linux CAN interface name")) -> None:
    """Bring a CAN interface down."""
    _run_ip("link", "set", "dev", interface, "down", privileged=True)
    typer.echo(f"CAN interface {interface} is down")


@app.command("up")
def up(interface: str = typer.Argument(..., help="Linux CAN interface name")) -> None:
    """Bring an already configured CAN interface up."""
    _run_ip("link", "set", "dev", interface, "up", privileged=True)
    typer.echo(f"CAN interface {interface} is up")


@app.command("setup")
def setup(
    interface: str = typer.Argument(..., help="Linux CAN interface name"),
    bitrate: int = typer.Option(
        1_000_000,
        min=10_000,
        help="Nominal CAN bitrate in bits per second",
    ),
    txqueuelen: int = typer.Option(1_000, min=1, help="Kernel transmit queue length"),
) -> None:
    """Configure, bring up, and verify a classic CAN interface."""
    setup_interface(interface, bitrate=bitrate, txqueuelen=txqueuelen)


def setup_interface(interface: str, *, bitrate: int, txqueuelen: int = 1_000) -> None:
    """Configure and verify one classic CAN interface."""

    _run_ip("link", "show", "dev", interface)
    _run_ip("link", "set", "dev", interface, "down", privileged=True)
    _run_ip(
        "link", "set", "dev", interface, "type", "can", "bitrate", str(bitrate), privileged=True
    )
    _run_ip("link", "set", "dev", interface, "txqueuelen", str(txqueuelen), privileged=True)
    _run_ip("link", "set", "dev", interface, "up", privileged=True)
    result = _run_ip("-details", "-statistics", "link", "show", "dev", interface)
    typer.echo(result.stdout.rstrip())
    typer.echo(f"Configured {interface}: bitrate={bitrate}, txqueuelen={txqueuelen}")
