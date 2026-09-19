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
import inspect
import json
from pathlib import Path
import re
from typing import TYPE_CHECKING, Any

import typer

if TYPE_CHECKING:
    from dimos.evals.agents.base import Agent

app = typer.Typer(help="Run agent evals on recordings, sim, or a live robot.")


def agent_class(module: str) -> type[Agent]:
    """The one agent class defined in *module* — an agent is a module."""
    from dimos.evals.agents.base import Agent

    mod = importlib.import_module(module)
    found = [
        v
        for v in vars(mod).values()
        if isinstance(v, type)
        and v.__module__ == mod.__name__
        and issubclass(v, Agent)
        and not inspect.isabstract(v)
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


def agent_kwargs(overrides: Iterable[str]) -> dict[str, Any]:
    pairs = (item.partition("=") for item in overrides)
    return {name: _value(text) for name, _, text in pairs}


def _key_words(key: str) -> list[str]:
    """``accessToken`` -> ["access", "token"], ``max_output_tokens`` -> ["max", "output", "tokens"]."""
    spaced = re.sub(r"(?<=[a-z0-9])(?=[A-Z])", " ", key)
    return re.split(r"[\s_\-]+", spaced.casefold())


def _has_secret(value: Any) -> bool:
    words = {"key", "apikey", "token", "secret", "password", "credential", "authorization"}
    if isinstance(value, dict):
        return any(
            bool(words & set(_key_words(str(key)))) or _has_secret(item)
            for key, item in value.items()
        )
    return isinstance(value, list) and any(_has_secret(item) for item in value)


def run_provenance(source: dict[str, Any], module: str, kwargs: dict[str, Any]) -> dict[str, Any]:
    unavailable = _has_secret(kwargs)
    try:
        json.dumps(kwargs, allow_nan=False)
    except (TypeError, ValueError):
        unavailable = True
    agent = {"module": module, "kwargs": None if unavailable else kwargs}
    if unavailable:
        agent["unavailable_reason"] = "agent arguments could not be safely serialized"
    return {"source": source, "agent": agent}


def load_agent(module: str, overrides: Iterable[str] = ()) -> Agent:
    """``--agent module --set field=value ...``: the agent class in *module*,
    constructed with the overrides. A field the agent does not have is the
    constructor's own ``TypeError``."""
    return agent_class(module)(**agent_kwargs(overrides))


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
    allow: str | None = typer.Option(
        None, "--allow", help="Allowed tool names, e.g. bash,grep; empty string disables tools"
    ),
    exclude: str | None = typer.Option(
        None,
        "--exclude",
        help="Deny tool calls mentioning these keywords, e.g. dimos,dimensionalos",
    ),
    tags: str = typer.Option("", help="Comma-separated tag filter"),
    limit: int = typer.Option(0, min=0, help="Run at most N cases"),
    case: list[str] = typer.Option([], "--case", help="Run only these case IDs"),
    parallel: int = typer.Option(1, min=1, help="Cases at once, one dimos each; needs --container"),
    container: str = typer.Option("", help="Docker image that runs each case (docker/eval)"),
    repeat: int = typer.Option(1, min=1, help="Trials per case"),
    video: bool = typer.Option(False, "--video", help="Capture the viewer as viewer.mp4 per case"),
) -> None:
    from dimos.evals.runner import EvalRunner, summarize

    cases = [c for c in importlib.import_module(suite).SUITE if not case or c.id in case]
    if video:
        for c in cases:
            if hasattr(c.environment.config, "video"):
                c.environment.config.video = True
        set_ = [*set_, "video=true"] if parallel > 1 or container or repeat > 1 else set_
    kwargs = agent_kwargs(set_)
    if allow is not None:
        if "allowed_tools" in kwargs:
            raise typer.BadParameter("Use --allow or --set allowed_tools, not both")
        names = [name.strip() for name in allow.split(",")] if allow.strip() else []
        if any(not name for name in names) or len(names) != len(set(names)):
            raise typer.BadParameter("Tool names must be nonempty and unique", param_hint="--allow")
        kwargs["allowed_tools"] = names
    if exclude is not None:
        if "excluded_keywords" in kwargs:
            raise typer.BadParameter("Use --exclude or --set excluded_keywords, not both")
        kwargs["excluded_keywords"] = [w.strip() for w in exclude.split(",") if w.strip()]
    provenance = run_provenance({"kind": "suite_module", "value": suite}, agent, kwargs)
    selected = frozenset(t for t in tags.split(",") if t) if tags else frozenset()
    if parallel > 1 or container or repeat > 1:
        if parallel > 1 and not container:
            raise typer.BadParameter(
                "parallel cases need --container: one dimos per host otherwise"
            )
        from dimos.evals.parallel import run_parallel

        chosen = [c for c in cases if not selected or selected & c.tags][: limit or None]
        run_dir, results = run_parallel(
            suite, agent, set_, chosen,
            parallel=parallel, container=container, repeat=repeat, manifest=provenance,
        )  # fmt: skip
    else:
        runner = EvalRunner()
        results = runner.run(
            cases, agent_class(agent)(**kwargs), tags=selected, limit=limit, provenance=provenance
        )
        run_dir = runner.run_dir

    for r in results:
        status = "ERROR" if r.error else ("PASS" if r.passed else "fail")
        detail = r.error or (f"score={r.score:.2f} steps={r.steps} answer={r.final_answer[:60]!r}")
        typer.echo(f"{status:5} {r.case_id:30} {detail} ({r.duration_s:.1f}s)")
    s = summarize(results)
    typer.echo(
        f"\n{s.n} cases | mean {s.mean_score:.2f} | pass {s.pass_rate:.0%} "
        f"| errors {s.errors} | {s.duration_s:.0f}s | {run_dir}"
    )


@app.command("media")
def media(
    runs: list[str] = typer.Argument(help="Run directories with <case>/viewer.mp4"),
    out: str = typer.Option("media", help="Output directory"),
    grid: bool = typer.Option(False, "--grid", help="Also tile every case's videos, one per run"),
) -> None:
    """Caption each case video with its arm, case and score; optionally tile arms side by side."""
    from dimos.evals.media import caption_runs, tile

    captioned = caption_runs([Path(r) for r in runs], Path(out))
    for path in captioned.values():
        typer.echo(path)
    if grid:
        by_case: dict[str, list[Path]] = {}
        for (case_id, _), path in captioned.items():
            by_case.setdefault(case_id, []).append(path)
        for case_id, paths in by_case.items():
            typer.echo(tile(paths, Path(out) / f"{case_id}-grid.mp4"))


@app.command("list")
def list_() -> None:
    from dimos.evals.module import list_suites

    for name in list_suites():
        typer.echo(name)
