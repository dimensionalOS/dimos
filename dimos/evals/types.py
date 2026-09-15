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

"""Eval cases, results, and Harbor ATIF trajectory records."""

from __future__ import annotations

from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal

from pydantic import JsonValue

if TYPE_CHECKING:
    from dimos.evals.environments.base import Environment
    from dimos.memory.store.base import Store
    from dimos.memory.stream import Stream

# https://www.harborframework.com/docs/agents/trajectory-format


@dataclass(frozen=True, kw_only=True)
class ToolCall:
    tool_call_id: str
    function_name: str
    arguments: dict[str, JsonValue]


@dataclass(frozen=True, kw_only=True)
class ObservationResult:
    source_call_id: str  # the ToolCall this answers
    content: str


@dataclass(frozen=True, kw_only=True)
class Observation:
    results: tuple[ObservationResult, ...]


@dataclass(frozen=True, kw_only=True)
class Metrics:
    prompt_tokens: int  # everything sent, cache reads included
    completion_tokens: int
    cached_tokens: int = 0  # the part of prompt_tokens read from the provider's cache
    cost_usd: float | None = None  # reported or estimated by the adapter; None if unknown


def total_cost(costs: Iterable[float | None]) -> float | None:
    """Sum reported costs, preserving an unknown total if any cost is missing."""
    total = 0.0
    for cost in costs:
        if cost is None:
            return None
        total += cost
    return total


@dataclass(frozen=True, kw_only=True)
class StepExtra:
    request: Path  # the exact payload sent to the provider for this call
    response: Path  # the exact payload received
    latency_s: float = 0.0
    reasoning_tokens: int = 0  # the part of completion_tokens spent reasoning


@dataclass(frozen=True, kw_only=True)
class Step:
    """One turn: the instruction (``user``) or one model call (``agent``) with
    the tool executions it caused."""

    step_id: int  # 1-based, sequential
    timestamp: str  # ISO 8601
    source: Literal["user", "agent", "system"]
    message: str
    # agent steps only
    model_name: str | None = None
    reasoning_content: str | None = None
    tool_calls: tuple[ToolCall, ...] | None = None
    observation: Observation | None = None
    metrics: Metrics | None = None
    extra: StepExtra | None = None


@dataclass(frozen=True, kw_only=True)
class AgentInfo:
    name: str  # the Agent class
    version: str
    model_name: str  # what actually ran, as reported by the provider
    tool_definitions: tuple[dict[str, Any], ...] | None = None


@dataclass(frozen=True, kw_only=True)
class FinalMetrics:
    total_prompt_tokens: int
    total_completion_tokens: int
    total_cached_tokens: int
    total_cost_usd: float | None
    total_steps: int


EndedBy = Literal["answer", "max_steps", "timeout", "error"]


@dataclass(frozen=True, kw_only=True)
class RunExtra:
    ended_by: EndedBy
    error: str = ""


@dataclass(frozen=True, kw_only=True)
class Trajectory:
    """The ATIF document for one run: the instruction, then one agent step per
    model call, in order. The messages array actually sent per call is *not*
    the previous call plus one message (harnesses compact and re-inject), so
    each call's request is a whole record named in its step, never
    reconstructed from the steps."""

    schema_version: str = "ATIF-v1.7"
    agent: AgentInfo
    steps: tuple[Step, ...]
    final_metrics: FinalMetrics
    extra: RunExtra

    @property
    def final_answer(self) -> str:
        """The last agent message that called no tool, "" if none."""
        return next(
            (s.message for s in reversed(self.steps) if s.source == "agent" and not s.tool_calls),
            "",
        )


@dataclass(frozen=True, kw_only=True)
class RunningEnvironment:
    mcp_url: str  # "" when there is no robot
    streams: Sequence[Stream[Any, Any]]  # what the agent may look at. Dataset: the selection
    artifacts: Mapping[str, Path]  # files produced by the environment, by name


@dataclass(frozen=True, kw_only=True)
class Outcome:
    trajectory: Trajectory  # the agent's final reply is trajectory.final_answer
    artifacts: Mapping[str, Path]  # what the environment produced, by name


def recording(o: Outcome) -> Store:
    """The memory recording an environment produced, opened for grading."""
    from dimos.memory.store.sqlite import SqliteStore

    return SqliteStore(path=str(o.artifacts["recording"]), must_exist=True)


@dataclass(frozen=True, kw_only=True)
class EvalCase:
    id: str
    inputs: str  # the user message
    environment: Environment
    grade: Callable[[Outcome], float]  # 0..1, called once after the agent finishes
    tags: frozenset[str] = frozenset()
    timeout_s: float = 60.0  # wall-clock is a task property; max_steps is the agent's
    threshold: float = 1.0  # passed = score >= threshold; the case knows its own pass bar


Suite = Sequence[EvalCase]


@dataclass(frozen=True, kw_only=True)
class EvalResult:
    case_id: str
    score: float = 0.0
    passed: bool = False
    duration_s: float = 0.0
    error: str = ""
    final_answer: str = ""
    steps: int = 0  # every step, the instruction included
    model_turns: int = 0
    tool_calls: int = 0
    request_attempts: int = 0
    agent_duration_s: float = 0.0
    prompt_tokens: int = 0  # everything sent, cache reads included
    completion_tokens: int = 0
    cached_tokens: int = 0
    reasoning_tokens: int = 0
    cost_usd: float | None = None
    ended_by: str = ""
    trajectory: str = ""  # path of <case_id>/trajectory.json, when an agent ran
