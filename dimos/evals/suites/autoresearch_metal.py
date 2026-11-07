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

"""One unattended autoresearch session per task on the Metal arm in MuJoCo.

The case prompt is the ASPIRE actor loop: read the task and scene, submit
immutable candidate programs, run them on dev trials, read the traces and
keyframes, repair by failure mode, search with several candidates when
stuck, freeze one. The lab scores the frozen candidate on held-out trials
and the grader reads that rate from ``session.json``. The run needs
``ANTHROPIC_API_KEY`` and Pi on PATH; the keyword guard denies every way
around the lab's tools:

    dimos evals run dimos.evals.suites.autoresearch_metal --agent dimos.evals.agents.pi \\
        --set provider=anthropic --set model=<model> --set max_steps=600 \\
        --exclude shell,porcelain,sdk,topic,select,pick_object,place_at
"""

from __future__ import annotations

from datetime import datetime
import json
import re
from typing import Any

from dimos.autoresearch.prompt import actor_prompt
from dimos.autoresearch.session import Budget, Session
from dimos.evals.environments.autoresearch import AutoresearchEnvironment
from dimos.evals.types import EvalCase, Outcome, Step, Suite, Trajectory

TOOLS = ("task", "scene", "submit", "run", "stop_run", "results", "leaderboard", "freeze", "note")

BUDGET = Budget(iterations=3, candidates_per_round=3, dev_trials=3, heldout_trials=5, threshold=0.6)

TASKS = ("lift the cube",)

REPORT_NAME = "report.json"


def heldout_rate(outcome: Outcome) -> float:
    """The frozen candidate's held-out pass rate from ``session.json``, which must be
    scored. Also writes ``report.json`` in the session: leaderboard, iteration count
    and tokens and seconds per iteration (one iteration is one submitted candidate)."""
    session_dir = outcome.artifacts["session"]
    info = json.loads((session_dir / "session.json").read_text())
    if info.get("status") != "scored":
        raise RuntimeError(f"session {session_dir} is {info.get('status')!r}, not scored")
    session = Session(session_dir)
    frozen = session.frozen()
    if frozen is None:
        raise RuntimeError(f"session {session_dir} is scored but froze no candidate")
    rate = float(info["heldout_rate"])
    per_iteration = iterations(outcome.trajectory)
    report = {
        "task": info["task"],
        "frozen": frozen,
        "heldout_rate": rate,
        "threshold": info["budget"]["threshold"],
        "leaderboard": [score.model_dump() for score in session.leaderboard()],
        "iterations": len(per_iteration),
        "per_iteration": per_iteration,
    }
    (session_dir / REPORT_NAME).write_text(json.dumps(report, indent=2))
    return rate


def iterations(trajectory: Trajectory) -> list[dict[str, Any]]:
    """Tokens and seconds per submitted candidate: the model turns from the
    previous ``submit`` call up to and including this one, in submission order."""
    rows: list[dict[str, Any]] = []
    prompt = completion = 0
    started: str | None = None
    for step in trajectory.steps:
        if step.source != "agent":
            continue
        started = started or step.timestamp
        if step.metrics:
            prompt += step.metrics.prompt_tokens
            completion += step.metrics.completion_tokens
        if calls_tool(step, "submit"):
            latency = step.extra.latency_s if step.extra else 0.0
            rows.append(
                {
                    "candidate": len(rows) + 1,
                    "prompt_tokens": prompt,
                    "completion_tokens": completion,
                    "seconds": seconds_between(started, step.timestamp) + latency,
                }
            )
            prompt = completion = 0
            started = None
    return rows


def calls_tool(step: Step, tool: str) -> bool:
    """Whether the step called the lab tool natively or as ``dimos mcp call <tool>`` from a shell."""
    pattern = re.compile(rf"\bmcp call {re.escape(tool)}\b")
    return any(
        call.function_name == tool or bool(pattern.search(json.dumps(call.arguments)))
        for call in step.tool_calls or ()
    )


def seconds_between(start: str, end: str) -> float:
    return (datetime.fromisoformat(end) - datetime.fromisoformat(start)).total_seconds()


def case(task: str, budget: Budget = BUDGET) -> EvalCase:
    return EvalCase(
        id=re.sub(r"[^a-z0-9]+", "_", task.lower()).strip("_"),
        inputs=actor_prompt(task, budget),
        environment=AutoresearchEnvironment(task=task, budget=budget),
        grade=heldout_rate,
        timeout_s=2 * 3600.0,
        threshold=budget.threshold,
        tags=frozenset({"mujoco", "manipulation", "autoresearch"}),
    )


SUITE: Suite = [case(task) for task in TASKS]
