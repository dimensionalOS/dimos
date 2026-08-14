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

from collections.abc import Callable
import importlib
from pathlib import Path
from typing import TYPE_CHECKING, cast

import typer

if TYPE_CHECKING:
    from dimos.evals.types import Suite

app = typer.Typer(help="Run agent evals on recordings, sim, or a live robot.")


@app.command("run")
def run(
    suite: str = typer.Argument(help="Dotted module exporting SUITE or load_suite(dataset)"),
    tags: str = typer.Option("", help="Comma-separated tag filter"),
    model: str = typer.Option("", help="Override chat model"),
    blind: bool = typer.Option(False, help="Withhold observations (guessing ablation)"),
    attach: bool = typer.Option(False, help="Drive an already-running dimos (interactive cases)"),
    limit: int = typer.Option(0, help="Run at most N cases"),
    live_db: str = typer.Option("recording.db", help="Live Recorder db (interactive cases)"),
    dataset: Path | None = typer.Option(
        None,
        help="Dataset directory for a suite module exporting load_suite(dataset)",
        exists=True,
        file_okay=False,
        readable=True,
    ),
) -> None:
    from dimos.evals.runner import EvalRunner, summarize

    cases = _load_suite(suite, dataset)
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


def _load_suite(suite: str, dataset: Path | None) -> Suite:
    """Load a static SUITE or construct one from a dataset directory."""
    module = importlib.import_module(suite)
    if dataset is not None:
        loader = getattr(module, "load_suite", None)
        if loader is None:
            raise typer.BadParameter(f"{suite} does not support --dataset")
        return cast("Callable[[Path], Suite]", loader)(dataset)
    cases = getattr(module, "SUITE", None)
    if cases is None:
        raise typer.BadParameter(f"{suite} requires --dataset")
    return cast("Suite", cases)
