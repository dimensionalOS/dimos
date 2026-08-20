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

"""Transport benchmark commands."""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
import sys

import typer

from dimos.constants import STATE_DIR
from dimos.protocol.pubsub.benchmark.model import Environment, NetworkProfile, Stack

app = typer.Typer(help="Run reproducible performance benchmarks", no_args_is_help=True)
transport_app = typer.Typer(help="Benchmark transport application paths", no_args_is_help=True)
app.add_typer(transport_app, name="transport")


@transport_app.command("run")
def run_transport_benchmark(
    suite: str = typer.Option("smoke", help="Campaign suite: smoke or public"),
    output: Path | None = typer.Option(None, help="Output directory for versioned artifacts"),
    stack: list[Stack] | None = typer.Option(None, help="Stack filter; repeat this option"),
    environment: list[Environment] | None = typer.Option(
        None, help="Environment filter; repeat this option"
    ),
    profile: list[NetworkProfile] | None = typer.Option(
        None, help="Network profile filter; repeat this option"
    ),
    repetitions: int | None = typer.Option(
        None, min=1, help="Override independent trials per cell"
    ),
    warmup_s: float | None = typer.Option(None, min=0.0, help="Override warmup seconds"),
    duration_s: float | None = typer.Option(None, min=0.01, help="Override measurement seconds"),
    drain_s: float | None = typer.Option(None, min=0.0, help="Override drain seconds"),
    seed: int = typer.Option(7, help="Randomization and bootstrap seed"),
    image: str = typer.Option(
        "dimos-transport-benchmark:local",
        help="Pinned benchmark image used by emulated trials",
    ),
) -> None:
    """Run a transport benchmark campaign and write a self-contained report."""
    from dimos.protocol.pubsub.benchmark.runner import run_campaign

    if suite not in {"smoke", "public"}:
        raise typer.BadParameter("choose 'smoke' or 'public'", param_hint="--suite")
    if output is None:
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        output = STATE_DIR / "transport-benchmarks" / timestamp
    run_campaign(
        suite=suite,
        output_dir=output,
        stacks=set(stack) if stack else None,
        environments=set(environment) if environment else None,
        profiles=set(profile) if profile else None,
        repetitions=repetitions,
        warmup_s=warmup_s,
        duration_s=duration_s,
        drain_s=drain_s,
        seed=seed,
        command=sys.argv,
        image=image,
    )
    typer.echo(output)


@transport_app.command("report")
def report_transport_benchmark(
    run_dir: Path = typer.Argument(..., exists=True, file_okay=False, readable=True),
) -> None:
    """Regenerate the interactive report from a completed campaign."""
    from dimos.protocol.pubsub.benchmark.report import generate_report

    typer.echo(generate_report(run_dir))
