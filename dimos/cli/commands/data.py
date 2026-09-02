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

"""`dimos data` commands; the implementation lives in dimos.cloud.cli."""

from __future__ import annotations

from pathlib import Path

import typer

data_app = typer.Typer(help="Dimensional cloud data", no_args_is_help=True)

_SINCE_UNITS = {"m": 60, "h": 3600, "d": 86400}


def _since_s(s: str | None) -> float | None:
    if s is None:
        return None
    return float(s[:-1]) * _SINCE_UNITS[s[-1]] if s[-1] in _SINCE_UNITS else float(s)


@data_app.command()
def upload(
    path: Path | None = typer.Argument(None, help="Recording or file; default: newest recording"),
    robot: str | None = typer.Option(None, "--robot", help="Robot id for attribution"),
    kind: str | None = typer.Option(None, "--kind", help="Override the inferred kind"),
    since: str | None = typer.Option(None, "--since", help="Upload everything newer than e.g. 1h"),
    chunk: int | None = typer.Option(None, "--chunk", help="Upload part size in MB"),
) -> None:
    """Upload to Dimensional cloud. Resumable: re-run after any failure and only
    missing parts transfer."""
    from dimos.cloud import cli

    cli.upload(path, robot, kind, _since_s(since), chunk)


@data_app.command("ls")
def data_ls(
    interactive: bool = typer.Option(
        False, "--interactive", "-i", help="Browse uploads in a TUI: enter=detail, p=pull"
    ),
) -> None:
    from dimos.cloud import cli

    cli.ls(interactive)


@data_app.command()
def pull(
    upload_id: str | None = typer.Argument(None, help="Id or prefix from `ls`; default: newest"),
    dest: Path | None = typer.Option(None, "--dest"),
) -> None:
    from dimos.cloud import cli

    cli.pull(upload_id, dest)


@data_app.command("status")
def data_status(upload_id: str) -> None:
    from dimos.cloud import cli

    cli.status(upload_id)


@data_app.command()
def quota() -> None:
    from dimos.cloud import cli

    cli.quota()
