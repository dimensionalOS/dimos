# Copyright 2025-2026 Dimensional Inc.
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

import json
from pathlib import Path
from typing import Annotated

from rich.live import Live
import typer

from dimos.cli.network.display import dashboard, print_report, terminal_console
from dimos.cli.network.model import Report, Settings
from dimos.cli.network.peer import peer_main
from dimos.cli.network.runner import run_check

network_app = typer.Typer(
    help="Bounded synthetic Zenoh quality checks before starting a robot stack."
)


@network_app.command()
def check(
    remote: Annotated[str, typer.Argument(help="Existing SSH host alias or user@host.")],
    remote_dimos: Annotated[
        str, typer.Option(help="Absolute remote DimOS executable path; nothing is installed.")
    ],
    peer_host: Annotated[
        str | None, typer.Option(help="Direct TCP hostname/IP if different from SSH alias.")
    ] = None,
    listen_host: Annotated[
        str, typer.Option(help="Remote listen IP; restrict it to the intended interface.")
    ] = "0.0.0.0",
    port: Annotated[
        int, typer.Option(min=0, max=65535, help="Remote TCP port; 0 selects a free port.")
    ] = 0,
    max_mbps: Annotated[float, typer.Option(help="Offered-rate cap, not a capacity claim.")] = 50,
    step_seconds: float = 5,
    idle_seconds: float = 5,
    max_seconds: float = 90,
    max_bytes: int = 256 * 1024 * 1024,
    payload_bytes: int = 64 * 1024,
    probe_hz: float = 50,
    probe_timeout: float = 0.5,
    min_goodput_mbps: Annotated[
        float | None,
        typer.Option(
            help="Optional goodput target; stop early when all supplied thresholds are met."
        ),
    ] = None,
    max_rtt_p95_ms: float | None = None,
    max_missing_pct: float | None = None,
    json_output: Annotated[
        bool,
        typer.Option("--json", help="Only machine-readable JSON on stdout; progress on stderr."),
    ] = False,
    json_file: Annotated[
        Path | None, typer.Option(help="Also save full samples and step results as JSON.")
    ] = None,
) -> None:
    """One local command owns the remote peer and runs sequential bidirectional ramps."""
    settings = Settings(
        max_mbps=max_mbps,
        step_seconds=step_seconds,
        idle_seconds=idle_seconds,
        max_seconds=max_seconds,
        max_bytes=max_bytes,
        payload_bytes=payload_bytes,
        probe_hz=probe_hz,
        probe_timeout=probe_timeout,
        min_goodput_mbps=min_goodput_mbps,
        max_rtt_p95_ms=max_rtt_p95_ms,
        max_missing_pct=max_missing_pct,
    )
    try:
        settings.validate()
    except ValueError as error:
        raise typer.BadParameter(str(error)) from error
    console = terminal_console(stderr=True)
    live: Live | None = None

    def progress(message: str, report: Report) -> None:
        if live is not None:
            live.update(dashboard(report, console.width, message, fancy=not console.no_color))
        else:
            console.print(message, markup=False)

    try:
        if console.is_terminal:
            live = Live(console=console, transient=True, refresh_per_second=4)
            live.start()
        report = run_check(
            remote,
            remote_dimos,
            settings,
            peer_host=peer_host,
            listen_host=listen_host,
            port=port,
            progress=progress,
        )
    except ValueError as error:
        raise typer.BadParameter(str(error)) from error
    finally:
        if live is not None:
            live.stop()
    data = json.dumps(report.to_dict(), indent=2, allow_nan=False)
    if json_file is not None:
        json_file.write_text(data + "\n")
    if json_output:
        typer.echo(data)
    else:
        print_report(report, terminal_console())
    if report.status == "cancelled":
        raise typer.Exit(130)
    if report.status != "completed":
        raise typer.Exit(1)
    if report.verdict == "not_met":
        raise typer.Exit(2)


@network_app.command(hidden=True)
def peer(
    stdio: Annotated[
        bool, typer.Option(help="Require the SSH-owned stdin/stdout protocol.")
    ] = False,
) -> None:
    if not stdio:
        raise typer.BadParameter("peer is internal; use network check")
    peer_main()


if __name__ == "__main__":
    network_app()
