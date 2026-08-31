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

from collections.abc import Iterable
import importlib
import json
from typing import Any

import typer

app = typer.Typer(help="Run agent evals on recordings, sim, or a live robot.")


def agent_class(module: str) -> type:
    """The one agent class defined in *module* — an agent is a module."""
    mod = importlib.import_module(module)
    found = [
        v
        for v in vars(mod).values()
        if isinstance(v, type)
        and v.__module__ == mod.__name__
        and callable(getattr(v, "run", None))
        and callable(getattr(v, "preflight", None))
    ]
    if len(found) != 1:
        raise TypeError(f"{module} defines {len(found)} agents, expected one")
    return found[0]


def _value(text: str) -> Any:
    """A ``--set`` value: JSON where it parses (``10``, ``null``, ``true``), else text."""
    try:
        return json.loads(text)
    except ValueError:
        return text


def load_agent(module: str, overrides: Iterable[str] = ()) -> Any:
    """``--agent module --set field=value ...``: the agent class in *module*,
    constructed with the overrides. A field the agent does not have is the
    constructor's own ``TypeError``."""
    pairs = (item.partition("=") for item in overrides)
    return agent_class(module)(**{name: _value(text) for name, _, text in pairs})


@app.command("run")
def run(
    suite: str = typer.Argument(
        help="Dotted suite module exporting SUITE, e.g. dimos.evals.suites.go2_smoke"
    ),
    agent: str = typer.Option(
        ..., help="Dotted agent module, e.g. dimos.evals.agents.question_answer"
    ),
    set_: list[str] = typer.Option(
        [], "--set", help="Agent field override, e.g. --set model=gpt-5.6-luna --set max_steps=10"
    ),
    tags: str = typer.Option("", help="Comma-separated tag filter"),
    limit: int = typer.Option(0, help="Run at most N cases"),
) -> None:
    from dimos.evals.runner import EvalRunner, summarize

    cases = importlib.import_module(suite).SUITE
    runner = EvalRunner()
    results = runner.run(
        cases,
        load_agent(agent, set_),
        tags=frozenset(t for t in tags.split(",") if t) if tags else frozenset(),
        limit=limit,
    )

    for r in results:
        status = "ERROR" if r.error else ("PASS" if r.passed else "fail")
        detail = r.error or (f"score={r.score:.2f} steps={r.steps} answer={r.final_answer[:60]!r}")
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
