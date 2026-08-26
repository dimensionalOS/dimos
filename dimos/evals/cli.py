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

"""``dimos evals`` — run and list eval suites. Heavy imports stay inside
command bodies (test_cli_startup budget)."""

from __future__ import annotations

import importlib

import typer

app = typer.Typer(help="Run agent evals on recordings, sim, or a live robot.")


@app.command("run")
def run(
    suite: str = typer.Argument(
        help="Dotted suite module exporting SUITE, e.g. dimos.evals.suites.go2_smoke"
    ),
    tags: str = typer.Option("", help="Comma-separated tag filter"),
    model: str = typer.Option("", help="Override chat model"),
    blind: bool = typer.Option(False, help="Withhold observations (guessing ablation)"),
    attach: bool = typer.Option(False, help="Drive an already-running dimos (interactive cases)"),
    limit: int = typer.Option(0, help="Run at most N cases"),
    live_db: str = typer.Option("recording.db", help="Live Recorder db (interactive cases)"),
) -> None:
    from dimos.evals.runner import EvalRunner, summarize

    cases = importlib.import_module(suite).SUITE
    overrides: dict[str, object] = {"blind": blind, "attach": attach, "live_db": live_db}
    if model:
        overrides["model"] = model
    runner = EvalRunner(**overrides)
    results = runner.run(
        cases,
        tags=frozenset(t for t in tags.split(",") if t) if tags else frozenset(),
        limit=limit,
    )

    for r in results:
        status = "ERROR" if r.error else ("PASS" if r.passed else "fail")
        detail = r.error or f"score={r.score:.2f} answer={r.outputs[:60]!r}"
        typer.echo(f"{status:5} {r.case_id:30} {detail} ({r.duration_s:.1f}s)")
    s = summarize(results)
    typer.echo(
        f"\n{s.n} cases | mean {s.mean_score:.2f} | pass {s.pass_rate:.0%} "
        f"| errors {s.errors} | {s.duration_s:.0f}s | {runner.run_dir}"
    )


@app.command("list")
def list_() -> None:
    from dimos.evals.module import list_suites

    for name in list_suites():
        typer.echo(name)
