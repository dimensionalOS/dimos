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

"""EvalRunner — the one engine behind CLI, MCP skill, and pytest.

Per case: preflight both sides, start the environment, run the agent under
the case's timeout, stop the environment, check the declared artifacts exist,
grade once. The runner never branches on the agent type.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import json
from pathlib import Path
import subprocess
import threading
import time
from typing import Any

from dimos.constants import STATE_DIR
from dimos.evals.types import (
    Agent,
    EvalCase,
    EvalResult,
    Outcome,
    RunningEnvironment,
    Suite,
    Trajectory,
)
from dimos.protocol.service.spec import BaseConfig, Configurable
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class EvalRunnerConfig(BaseConfig):
    threshold: float = 1.0  # passed = score >= threshold
    strict: bool = False  # preflight failure aborts the whole run
    out_dir: Path = STATE_DIR / "evals"


@dataclass(frozen=True, kw_only=True)
class RunSummary:
    n: int
    mean_score: float
    pass_rate: float
    errors: int
    duration_s: float


def summarize(results: list[EvalResult]) -> RunSummary:
    scored = [r for r in results if not r.error]
    return RunSummary(
        n=len(results),
        mean_score=sum(r.score for r in scored) / len(scored) if scored else 0.0,
        pass_rate=sum(r.passed for r in scored) / len(scored) if scored else 0.0,
        errors=sum(1 for r in results if r.error),
        duration_s=sum(r.duration_s for r in results),
    )


def agent_record(agent: Agent) -> dict[str, Any]:
    """The agent as it goes into ``summary.json``: its class and every
    constructor argument, so runs are comparable later."""
    cls = type(agent)
    return {"class": f"{cls.__module__}.{cls.__qualname__}", **vars(agent)}


class EvalRunner(Configurable):
    config: EvalRunnerConfig

    def __init__(self, **kwargs: Any) -> None:
        Configurable.__init__(self, **kwargs)
        self._run_dir: Path | None = None

    def run(
        self,
        cases: Suite,
        agent: Agent,
        *,
        tags: frozenset[str] = frozenset(),
        limit: int = 0,
    ) -> list[EvalResult]:
        selected = [c for c in cases if not tags or tags & c.tags]
        if limit:
            selected = selected[:limit]
        self._run_dir = self._new_run_dir()

        results: list[EvalResult] = []
        runnable: list[EvalCase] = []
        for case in selected:
            error = self._preflight(case, agent)
            if error:
                results.append(EvalResult(case_id=case.id, error=error))
            else:
                runnable.append(case)

        for case in runnable:
            result = self.run_case(case, agent)
            logger.info(
                "eval case done",
                case=case.id,
                score=round(result.score, 3),
                ended_by=result.ended_by or None,
                error=result.error or None,
            )
            results.append(result)

        self._write_artifacts(results, agent)
        return results

    def _preflight(self, case: EvalCase, agent: Agent) -> str:
        """Both sides checked before anything starts: the failure text, or ``""``."""
        try:
            case.environment.preflight(agent)
            agent.preflight(case.environment)
        except Exception as e:
            if self.config.strict:
                raise
            logger.warning("preflight failed", case=case.id, error=str(e))
            return f"preflight: {e}"
        return ""

    def run_case(self, case: EvalCase, agent: Agent) -> EvalResult:
        t0 = time.monotonic()
        case_dir = self.run_dir / case.id
        case_dir.mkdir(parents=True, exist_ok=True)
        trajectory: Trajectory | None = None
        try:
            env = case.environment.start(agent.modules)
            tools = list(dict.fromkeys(agent.available_tools(tuple(_tools_exposed(env.mcp_url)))))
            trajectory = self._run_agent(case, agent, env, case_dir)
            # The agent phase is over before grading: for a live environment
            # that closes the recording, and a timed-out agent thread can no
            # longer act on what the grader reads.
            case.environment.stop()
            _write_trajectory(case_dir, trajectory, tools)
            missing = [n for n in case.environment.artifacts if not env.artifacts[n].exists()]
            if missing:
                return self._result(case, t0, trajectory, error=f"missing artifacts: {missing}")
            score = case.grade(Outcome(trajectory=trajectory, artifacts=env.artifacts))
            return self._result(case, t0, trajectory, score=score)
        except Exception as e:
            return self._result(case, t0, trajectory, error=repr(e))
        finally:
            case.environment.stop()

    def _run_agent(
        self, case: EvalCase, agent: Agent, env: RunningEnvironment, case_dir: Path
    ) -> Trajectory:
        """``agent.run`` under the case's wall-clock limit. A timed-out agent
        yields an empty trajectory marked ``timeout``; the world is still graded."""
        box: dict[str, Any] = {}

        def target() -> None:
            try:
                box["trajectory"] = agent.run(case.inputs, env, case_dir)
            except BaseException as e:
                box["error"] = e

        t0 = time.monotonic()
        thread = threading.Thread(target=target, name=f"eval-{case.id}", daemon=True)
        thread.start()
        thread.join(case.timeout_s)
        if thread.is_alive():
            logger.warning("agent timed out", case=case.id, timeout_s=case.timeout_s)
            return Trajectory(
                final_answer="",
                steps=(),
                model="",
                duration_s=time.monotonic() - t0,
                ended_by="timeout",
                raw_dir=case_dir / "raw",
            )
        if "error" in box:
            raise box["error"]
        trajectory: Trajectory = box["trajectory"]
        return trajectory

    def _result(
        self,
        case: EvalCase,
        t0: float,
        trajectory: Trajectory | None,
        *,
        score: float = 0.0,
        error: str = "",
    ) -> EvalResult:
        result = EvalResult(
            case_id=case.id,
            score=score,
            passed=score >= self.config.threshold and not error,
            duration_s=time.monotonic() - t0,
            error=error,
        )
        if trajectory is None:
            return result
        return replace(
            result,
            final_answer=trajectory.final_answer,
            steps=len(trajectory.steps),
            input_tokens=trajectory.input_tokens,
            output_tokens=trajectory.output_tokens,
            ended_by=trajectory.ended_by,
            trajectory=str(self.run_dir / case.id / "trajectory.json"),
        )

    @property
    def run_dir(self) -> Path:
        assert self._run_dir is not None, "run_dir is available only during run()"
        return self._run_dir

    def _new_run_dir(self) -> Path:
        run_dir = self.config.out_dir / time.strftime("run-%Y%m%d-%H%M%S")
        run_dir.mkdir(parents=True, exist_ok=True)
        return run_dir

    def _write_artifacts(self, results: list[EvalResult], agent: Agent) -> None:
        lines = [json.dumps(asdict(r)) for r in results]
        (self.run_dir / "results.jsonl").write_text("\n".join(lines) + "\n")
        summary: dict[str, Any] = asdict(summarize(results))
        summary |= {"agent": agent_record(agent), "git": _git_sha()}
        (self.run_dir / "summary.json").write_text(json.dumps(summary, indent=2, default=repr))


def _tools_exposed(mcp_url: str) -> list[str]:
    """What the MCP server exposed at start — the tool set is the blueprint."""
    if not mcp_url:
        return []
    from dimos.agents.mcp.mcp_adapter import McpAdapter

    return [str(t["name"]) for t in McpAdapter(mcp_url).list_tools()]


def _write_trajectory(case_dir: Path, trajectory: Trajectory, tools: list[str]) -> None:
    record = {"tools": tools, **asdict(trajectory)}
    (case_dir / "trajectory.json").write_text(json.dumps(record, indent=2, default=str))


def _git_sha() -> str:
    try:
        return subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            capture_output=True,
            text=True,
            timeout=5,
            check=False,
        ).stdout.strip()
    except OSError:
        return ""
