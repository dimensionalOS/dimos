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
the case's timeout, let the world settle on the budget the agent didn't use,
stop the environment, check the declared artifacts exist, grade once. The
runner never branches on the agent type.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import json
from pathlib import Path
import subprocess
import tempfile
import threading
import time
from typing import Any

from dimos.constants import DIMOS_PROJECT_ROOT, STATE_DIR
from dimos.evals.agents.lib.trajectory_builder import TrajectoryBuilder
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
    cost_usd: float


def summarize(results: list[EvalResult]) -> RunSummary:
    scored = [r for r in results if not r.error]
    return RunSummary(
        n=len(results),
        mean_score=sum(r.score for r in scored) / len(scored) if scored else 0.0,
        pass_rate=sum(r.passed for r in scored) / len(scored) if scored else 0.0,
        errors=sum(1 for r in results if r.error),
        duration_s=sum(r.duration_s for r in results),
        cost_usd=sum(r.cost_usd for r in results),
    )


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
        provenance: dict[str, Any] | None = None,
    ) -> list[EvalResult]:
        selected = [c for c in cases if not tags or tags & c.tags]
        if limit:
            selected = selected[:limit]
        ids = [case.id for case in selected]
        reserved = {".", "..", "manifest.json", "summary.json", "results.jsonl"}
        for case_id in ids:
            if not case_id or set(case_id) & {"/", "\\"} or case_id in reserved:
                raise ValueError(f"unsafe eval case ID: {case_id!r}")
        if len(ids) != len(set(ids)):
            raise ValueError("duplicate eval case IDs")
        self._run_dir = self._new_run_dir()
        provenance = provenance or {"source": {"kind": "unavailable"}, "agent": None}
        manifest = {
            "schema_version": 1,
            "source": provenance["source"],
            "selection": {"tags": sorted(tags), "limit": limit, "case_ids": ids},
            "agent": provenance["agent"],
            "runner": {"threshold": self.config.threshold, "strict": self.config.strict},
            "code": _git_record(),
        }
        with (self.run_dir / "manifest.json").open("x") as stream:
            json.dump(manifest, stream, indent=2, allow_nan=False)

        results: dict[str, EvalResult] = {}
        runnable: list[EvalCase] = []
        for case in selected:
            error = self._preflight(case, agent)
            if error:
                results[case.id] = EvalResult(case_id=case.id, error=error)
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
            results[case.id] = result

        ordered = [results[case_id] for case_id in ids]
        self._write_artifacts(ordered)
        return ordered

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
            env = case.environment.start(agent.modules, trace_dir=case_dir / "raw")
            tools = list(dict.fromkeys(agent.available_tools(tuple(_tools_exposed(env.mcp_url)))))
            trajectory, agent_s = self._run_agent(case, agent, env, case_dir)
            case.environment.settle(max(0.0, case.timeout_s - agent_s))
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
    ) -> tuple[Trajectory, float]:
        """``agent.run`` under the case's wall-clock limit, and the seconds it
        took. A timed-out agent yields a trajectory of the instruction alone,
        marked ``timeout``; the world is still graded."""
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
            empty = TrajectoryBuilder(case.inputs, name=type(agent).__name__)
            return empty.build("timeout"), time.monotonic() - t0
        if "error" in box:
            raise box["error"]
        trajectory: Trajectory = box["trajectory"]
        return trajectory, time.monotonic() - t0

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
        totals = trajectory.final_metrics
        return replace(
            result,
            final_answer=trajectory.final_answer,
            steps=totals.total_steps,
            prompt_tokens=totals.total_prompt_tokens,
            completion_tokens=totals.total_completion_tokens,
            cached_tokens=totals.total_cached_tokens,
            reasoning_tokens=sum(s.extra.reasoning_tokens for s in trajectory.steps if s.extra),
            cost_usd=totals.total_cost_usd,
            ended_by=trajectory.extra.ended_by,
            trajectory=str(self.run_dir / case.id / "trajectory.json"),
        )

    @property
    def run_dir(self) -> Path:
        assert self._run_dir is not None, "run_dir is available only during run()"
        return self._run_dir

    def _new_run_dir(self) -> Path:
        self.config.out_dir.mkdir(parents=True, exist_ok=True)
        prefix = time.strftime("run-%Y%m%d-%H%M%S-")
        return Path(tempfile.mkdtemp(prefix=prefix, dir=self.config.out_dir))

    def _write_artifacts(self, results: list[EvalResult]) -> None:
        lines = [json.dumps(asdict(r)) for r in results]
        (self.run_dir / "results.jsonl").write_text("\n".join(lines) + "\n")
        summary: dict[str, Any] = asdict(summarize(results))
        summary["manifest"] = "manifest.json"
        (self.run_dir / "summary.json").write_text(json.dumps(summary, indent=2))


def _tools_exposed(mcp_url: str) -> list[str]:
    """What the MCP server exposed at start — the tool set is the blueprint."""
    if not mcp_url:
        return []
    from dimos.agents.mcp.mcp_adapter import McpAdapter

    return [str(t["name"]) for t in McpAdapter(mcp_url).list_tools()]


def _write_trajectory(case_dir: Path, trajectory: Trajectory, tools: list[str]) -> None:
    """The ATIF document, with the tools the agent could call and no nulls."""
    agent = replace(trajectory.agent, tool_definitions=tuple({"name": n} for n in tools))
    record = _without_none(asdict(replace(trajectory, agent=agent)))
    (case_dir / "trajectory.json").write_text(json.dumps(record, indent=2, default=str))


def _without_none(value: Any) -> Any:
    if isinstance(value, dict):
        return {k: _without_none(v) for k, v in value.items() if v is not None}
    if isinstance(value, (list, tuple)):
        return [_without_none(v) for v in value]
    return value


def _git(*args: str) -> str:
    return subprocess.run(
        ["git", *args],
        cwd=DIMOS_PROJECT_ROOT,
        capture_output=True,
        text=True,
        timeout=5,
        check=False,
    ).stdout.strip()


def _git_record() -> dict[str, Any]:
    try:
        return {
            "git_sha": _git("rev-parse", "HEAD") or None,
            "dirty": bool(_git("status", "--porcelain")),
        }
    except (OSError, subprocess.SubprocessError):
        return {"git_sha": None, "dirty": False}
