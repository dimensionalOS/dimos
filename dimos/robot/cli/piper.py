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

from collections.abc import Iterator
from contextlib import contextmanager
from importlib import resources
from pathlib import Path
import subprocess

import typer

app = typer.Typer(help="Piper robot commands")


@contextmanager
def _can_activation_helper() -> Iterator[Path]:
    """Materialize the bundled CAN helper for this invocation."""
    package = resources.files("dimos.robot.manipulators.piper.scripts")
    with resources.as_file(package / "can_activate.sh") as helper:
        yield helper


@app.command("can-activate")
def can_activate(
    interface: str = typer.Argument(..., help="CAN interface to configure"),
    bitrate: int = typer.Option(1_000_000, "--bitrate", help="CAN bitrate"),
) -> None:
    """Configure a Piper CAN interface using the bundled helper."""
    if not typer.confirm(
        "This will request sudo to configure CAN. Continue?",
        default=False,
    ):
        typer.echo("Aborted.")
        raise typer.Exit(1)

    with _can_activation_helper() as helper:
        subprocess.run([str(helper), interface, str(bitrate)], check=True)
