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

from __future__ import annotations

import typer

from dimos.cli.can import setup_interface

app = typer.Typer(help="Piper robot commands")


@app.command("can-activate")
def can_activate(
    interface: str = typer.Argument(..., help="CAN interface to configure"),
    bitrate: int = typer.Option(1_000_000, "--bitrate", min=1, help="CAN bitrate"),
) -> None:
    """Configure an existing Piper SocketCAN interface."""
    if not typer.confirm(
        "This will request sudo to configure CAN. Continue?",
        default=False,
    ):
        typer.echo("Aborted.")
        raise typer.Exit(1)

    setup_interface(interface, bitrate=bitrate)
