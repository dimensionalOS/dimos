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
import os
import re
import secrets
import subprocess
import time
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
    docker: bool = typer.Option(
        False,
        "--docker",
        help="Run this eval in a fresh, detached worker container (docker/evals/compose.yaml) "
        "and return at once; one container per invocation, gone when the eval ends",
    ),
) -> None:
    if docker:
        _run_in_docker(suite, agent, set_, allow, exclude, tags, limit)
        return

    from dimos.evals.runner import EvalRunner, summarize

    cases = importlib.import_module(suite).SUITE
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
    runner = EvalRunner()
    results = runner.run(
        cases,
        agent_class(agent)(**kwargs),
        tags=frozenset(t for t in tags.split(",") if t) if tags else frozenset(),
        limit=limit,
        provenance=run_provenance({"kind": "suite_module", "value": suite}, agent, kwargs),
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


def _run_in_docker(
    suite: str,
    agent: str,
    set_: list[str],
    allow: str | None,
    exclude: str | None,
    tags: str,
    limit: int,
) -> None:
    """The same ``dimos evals run`` in a one-off worker of docker/evals/compose.yaml.

    The container has its own network namespace, so any number of these run
    side by side on one host without sharing a port or a multicast bus. It is
    detached: this returns as soon as it is started, and it removes itself when
    the eval ends. Results, recordings and Rerun files land under the runs
    directory the compose file mounts on ``/state``.
    """
    from dimos.constants import DIMOS_PROJECT_ROOT

    argv = [suite, "--agent", agent]
    for item in set_:
        argv += ["--set", item]
    if allow is not None:
        argv += ["--allow", allow]
    if exclude is not None:
        argv += ["--exclude", exclude]
    if tags:
        argv += ["--tags", tags]
    if limit:
        argv += ["--limit", str(limit)]

    stamp = time.strftime("%Y%m%d-%H%M%S")
    name = f"evals-{stamp}-{suite.rsplit('.', 1)[-1]}-{secrets.token_hex(2)}"
    # COMPOSE_FILE set in the environment (e.g. to add compose.gpu.yaml) wins.
    compose = DIMOS_PROJECT_ROOT / "docker" / "evals" / "compose.yaml"
    files = [] if os.environ.get("COMPOSE_FILE") else ["-f", str(compose)]
    command = ["docker", "compose", *files, "run", "--rm", "-d", "--name", name, "worker"]
    command += ["dimos", "evals", "run", *argv]
    subprocess.run(command, check=True, stdout=subprocess.DEVNULL)
    runs = os.environ.get("EVAL_RUNS_DIR") or str(compose.parent / "eval-runs")
    typer.echo(f"{name}: started (detached)")
    typer.echo(f"  follow:   docker logs -f {name}")
    typer.echo(
        f"  results:  {runs}/dimos/evals/   recordings + rerun.rrd: {runs}/dimos/recordings/"
    )


@app.command("list")
def list_() -> None:
    from dimos.evals.module import list_suites

    for name in list_suites():
        typer.echo(name)
