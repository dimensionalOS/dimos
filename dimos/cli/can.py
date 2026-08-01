# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""Linux CAN interface management commands."""

from __future__ import annotations

import os
import shlex
import subprocess

import typer

app = typer.Typer(help="Configure and inspect Linux CAN interfaces", no_args_is_help=True)


def _run_ip(*args: str, privileged: bool = False) -> subprocess.CompletedProcess[str]:
    command = ["ip", *args]
    if privileged and os.geteuid() != 0:
        command = ["sudo", "--", *command]
    if privileged:
        typer.echo(f"Running: {shlex.join(command)}")
    try:
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


def _positive(value: int, name: str) -> None:
    if value <= 0:
        raise typer.BadParameter(f"{name} must be greater than zero")


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
    bitrate: int = typer.Option(1_000_000, help="Nominal CAN bitrate in bits per second"),
    txqueuelen: int = typer.Option(1_000, help="Kernel transmit queue length"),
) -> None:
    """Configure, bring up, and verify a classic CAN interface."""
    _positive(bitrate, "bitrate")
    _positive(txqueuelen, "txqueuelen")
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
