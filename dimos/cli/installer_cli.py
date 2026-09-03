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

"""Verbs the Rust installer owns, so they still work from inside an activated venv."""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, NoReturn

import typer

INSTALLER = Path.home() / ".local" / "bin" / "dimos"
FORWARDED = ("setup", "update", "service", "uninstall", "robot")
INSTALL_LINE = "curl -fsSL https://raw.githubusercontent.com/dimensionalOS/dimos/main/scripts/install.sh | bash"
PASSTHROUGH: dict[str, Any] = {
    "allow_extra_args": True,
    "ignore_unknown_options": True,
    "help_option_names": [],
}
# The installer sets this on the child it execs; seeing it means the two CLIs point at each other.
GUARD = "DIMOS_FORWARDED"


def _reinstall(reason: str) -> NoReturn:
    typer.echo(f"{reason}:\n  {INSTALL_LINE}", err=True)
    raise typer.Exit(2)


def forward(ctx: typer.Context) -> None:
    """Exec the installer binary with this verb and everything after it."""
    if os.environ.pop(GUARD, None):
        _reinstall(f"{INSTALLER} is not the installer binary (it forwarded back here); reinstall")
    if not INSTALLER.exists():
        _reinstall(f"installer not found at {INSTALLER}; install it")
    os.environ[GUARD] = "1"
    os.execv(str(INSTALLER), [str(INSTALLER), *ctx.command_path.split()[1:], *ctx.args])


def register(app: typer.Typer, *names: str) -> None:
    """Register one passthrough command per installer verb."""
    for name in names:
        app.command(name, context_settings=PASSTHROUGH, help=f"{name} (DimOS installer)")(forward)
