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

"""Agent and operator entry points for durable autoresearch sessions."""

from __future__ import annotations

import json
from pathlib import Path
import shlex
import sys
from typing import Annotated

import typer

from dimos.autoresearch.command import CommandEvaluator
from dimos.autoresearch.session import Budget, Session, SessionInfo
from dimos.autoresearch.skill import USER_SKILLS_DIR, export_artifact

autoresearch_app = typer.Typer(help="Run and compare experiments", no_args_is_help=True)


@autoresearch_app.command()
def init(
    session_dir: Path,
    evaluator: Annotated[
        str,
        typer.Option(
            metavar="COMMAND",
            help="Python script or quoted command to run for each trial. Reads JSON on stdin.",
        ),
    ],
    task: Annotated[str, typer.Option(help="What to improve")],
    metric: Annotated[str, typer.Option(help="What a score measures")],
    minimize: bool = False,
    candidates: Annotated[int, typer.Option(min=1)] = 15,
    dev_trials: Annotated[int, typer.Option(min=1)] = 5,
    heldout_trials: Annotated[int, typer.Option(min=1)] = 10,
    timeout: Annotated[
        float, typer.Option(help="Seconds per evaluator invocation", min=0.001)
    ] = 120,
) -> None:
    """Start a session using a project-specific evaluator in the current working directory."""
    if not task.strip() or not metric.strip():
        raise typer.BadParameter("task and metric must have a value")
    try:
        argv = shlex.split(evaluator)
    except ValueError as exc:
        raise typer.BadParameter(str(exc), param_hint="--evaluator") from exc
    if not argv or not argv[0]:
        raise typer.BadParameter("provide a script or command", param_hint="--evaluator")
    if argv[0].endswith(".py"):
        argv.insert(0, sys.executable)
    backend = CommandEvaluator(argv=argv, cwd=Path.cwd(), timeout_s=timeout)
    info = SessionInfo(
        task=task,
        metric=metric,
        experiment="command",
        direction="minimize" if minimize else "maximize",
        settings=backend.model_dump(mode="json"),
        budget=Budget(
            max_candidates=candidates, dev_trials=dev_trials, heldout_trials=heldout_trials
        ),
    )
    session = Session.create(session_dir, info)
    typer.echo(session.dir)


@autoresearch_app.command()
def submit(
    session_dir: Path,
    artifact: Annotated[Path, typer.Argument(exists=True, dir_okay=False)],
    hypothesis: Annotated[str, typer.Option(help="Why this candidate should improve the metric")],
) -> None:
    """Snapshot a candidate file and print its number."""
    session = Session(session_dir)
    CommandEvaluator.from_session(session.info())
    typer.echo(session.submit(artifact.name, artifact.read_bytes(), hypothesis))


@autoresearch_app.command()
def run(session_dir: Path, candidate: int) -> None:
    """Evaluate pending development trials and print the recorded scores and evidence."""
    session = Session(session_dir)
    session.evaluate(candidate, "dev", CommandEvaluator.from_session(session.info()))
    results(session_dir, candidate)


@autoresearch_app.command()
def freeze(session_dir: Path, candidate: int) -> None:
    """Freeze the final candidate and run or resume its held-out evaluation."""
    session = Session(session_dir)
    backend = CommandEvaluator.from_session(session.info())
    session.freeze(candidate)
    session.evaluate(candidate, "heldout", backend)
    status(session_dir)


@autoresearch_app.command()
def results(session_dir: Path, candidate: int) -> None:
    """Read a candidate's artifact, hypothesis and trial evidence."""
    session = Session(session_dir)
    typer.echo(
        json.dumps(
            dict(
                artifact=str(session.artifact(candidate)),
                hypothesis=session.hypothesis(candidate),
                trials=[r.model_dump(mode="json") for r in session.results(candidate)],
            ),
            indent=2,
        )
    )


@autoresearch_app.command()
def status(session_dir: Path) -> None:
    """Read the session state, development leaderboard and final score."""
    session = Session(session_dir)
    typer.echo(
        json.dumps(
            dict(
                session=session.info().model_dump(mode="json"),
                leaderboard=[s.model_dump() for s in session.leaderboard()],
                notes=[str(path) for path in session.library()],
            ),
            indent=2,
        )
    )


@autoresearch_app.command()
def note(session_dir: Path, title: str, text: str) -> None:
    """Save a finding and the conditions under which it applies."""
    typer.echo(Session(session_dir).note(title, text))


@autoresearch_app.command()
def export(
    session_dir: Path,
    name: str | None = None,
    root: Path = USER_SKILLS_DIR,
    instructions: Annotated[
        Path | None,
        typer.Option(
            exists=True, dir_okay=False, help="Markdown explaining how to use the winning artifact"
        ),
    ] = None,
    force: Annotated[
        bool, typer.Option(help="Allow exporting an unscored or unsuccessful arm program")
    ] = False,
) -> None:
    """Publish the winner as an agent skill. Arm sessions include their program runner."""
    session = Session(session_dir)
    if session.info().experiment == "mujoco":
        from dimos.autoresearch.mujoco.skill import export_skill

        target = export_skill(session, root, name, force=force)
    else:
        if instructions is None:
            raise typer.BadParameter("supply --instructions for using this artifact")
        target = export_artifact(session, root, instructions.read_text(), name)
    typer.echo(target)
