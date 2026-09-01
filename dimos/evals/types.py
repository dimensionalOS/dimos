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

"""Eval primitives: the task and the agent are separate things.

A :class:`EvalCase` is an environment, an instruction, and a grader; nothing
else. Which agent runs it is the run's business. The same case is benchmarked
under a bare model, the production agent, or an external CLI without
changing, so scores stay comparable across years.

- :class:`Environment` says what exists: a frozen recording (``Dataset``), a
  standalone image (``ImageFile``), or a live simulator (``Sim``). It starts,
  yields a :class:`RunningEnvironment` (an MCP url when there is a robot, the
  memory recording, and artifact paths), and stops.
- :class:`Agent` delivers the instruction, acts, and returns a
  :class:`Trajectory` (Harbor's ATIF document) with every provider
  request/response saved whole.
- ``grade(Outcome) -> float`` runs once, after the agent finishes, over the
  trajectory and the artifacts.

Suites are Python modules exporting ``SUITE: Suite``; an agent is a module
defining one :class:`Agent` class (:mod:`dimos.evals.agents`), constructed
from the command line with ``--set field=value`` overrides.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal, Protocol

if TYPE_CHECKING:
    from dimos.memory.store.base import Store
    from dimos.memory.stream import Stream

Select = Callable[["Store"], "Stream[Any, Any]"]
"""Stream selector — what a frozen recording holds for a case::

    lambda s: s.streams.lidar.limit(1)
    lambda s: s.streams.odom.range_time(0, 600)
"""


# -- trajectory (Harbor ATIF) ----------------------------------------------------------
# https://www.harborframework.com/docs/agents/trajectory-format


@dataclass(frozen=True, kw_only=True)
class ToolCall:
    tool_call_id: str
    function_name: str
    arguments: dict[str, Any]


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
    cost_usd: float | None = None  # when the provider reports it


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
    total_cost_usd: float
    total_steps: int


EndedBy = Literal["answer", "max_steps", "timeout", "error"]


@dataclass(frozen=True, kw_only=True)
class RunExtra:
    ended_by: EndedBy


@dataclass(frozen=True, kw_only=True)
class Trajectory:
    """The ATIF document for one run: the instruction, then one agent step per
    model call, in order. The messages array actually sent per call is *not*
    the previous call plus one message (harnesses compact and re-inject), so
    each call's request is a whole record named in its step, never
    reconstructed from the steps."""

    schema_version: str = (
        "ATIF-v1.7"  # v1.8 only adds audio parts; Harbor's validator stops at v1.7
    )
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


# -- environment -----------------------------------------------------------------------


@dataclass(frozen=True, kw_only=True)
class RunningEnvironment:
    mcp_url: str  # "" when there is no robot
    recording: Store  # the one sensor channel. Dataset: what the case selected. Sim: live
    artifacts: Mapping[str, Path]  # name -> path; every declared name is present


class Environment(Protocol):
    """What exists for a case. Implementations: :mod:`dimos.evals.environments`."""

    @property
    def artifacts(self) -> tuple[str, ...]:
        """Names this environment produces, e.g. ``("recording",)``."""

    @property
    def has_robot(self) -> bool:
        """True: ``start()`` returns an MCP url on its own, with no modules
        from the agent (``Sim``; ``Dataset`` only with ``mcp_url``)."""

    def preflight(self, agent: Agent) -> None:
        """Raise if this environment can't run *agent* (it adds modules this
        environment can't launch; a stream a case selected is missing).
        Cheap: no data read, no process started."""

    def start(self, modules: str) -> RunningEnvironment:
        """Start, with *modules* (what the agent adds) on top of this
        environment's own stack where it launches one."""

    def settle(self, budget_s: float) -> None:
        """Block until the world has finished reacting to the agent's actions,
        at most *budget_s* seconds. Skills can return before the motion they
        started completes; the case is over when the world is at rest, not
        when the agent stops talking. An environment where nothing keeps
        happening returns immediately."""

    def stop(self) -> None: ...


@dataclass(frozen=True, kw_only=True)
class Outcome:
    trajectory: Trajectory  # the agent's final reply is trajectory.final_answer
    artifacts: Mapping[str, Path]  # what the environment produced, by name


def recording(o: Outcome) -> Store:
    """The memory recording an environment produced, opened for grading."""
    from dimos.memory.store.sqlite import SqliteStore

    return SqliteStore(path=str(o.artifacts["recording"]), must_exist=True)


# -- agent -----------------------------------------------------------------------------


class Agent(Protocol):
    """How the instruction reaches a model and how the model acts.

    How the recording reaches the model is what an agent *is*: ``QuestionAnswer``
    encodes the whole recording into one prompt, ``Blind`` never looks,
    ``McpClientAgent`` acts through tools. No agent knows anything about
    any particular case.
    """

    modules: str
    """Blueprint atoms this agent brings; ``""`` for none. ``Sim`` appends
    them to its launch (``dimos run <case stack> <modules>``), ``Dataset``
    launches exactly them (a frozen recording has no stack to add to), and
    ``ImageFile`` rejects a non-empty value in ``preflight``."""

    def preflight(self, environment: Environment) -> None:
        """Raise if this agent can't run in *environment* (needs a robot and
        there is none; needs a stream it lacks). Runs before any environment
        starts."""

    def available_tools(self, environment_tools: tuple[str, ...]) -> tuple[str, ...]:
        """Every tool this agent can call. *environment_tools* are the MCP
        tools exposed by the running environment."""

    def run(self, inputs: str, env: RunningEnvironment, run_dir: Path) -> Trajectory:
        """Deliver *inputs* and let the agent act until it stops or hits its
        own step limit. Write raw provider request/response bodies under
        ``run_dir``. Return the trajectory. Synchronous. Never grades."""


# -- case ------------------------------------------------------------------------------


@dataclass(frozen=True, kw_only=True)
class EvalCase:
    id: str
    inputs: str  # the user message
    environment: Environment
    grade: Callable[[Outcome], float]  # 0..1, called once after the agent finishes
    tags: frozenset[str] = frozenset()
    timeout_s: float = 60.0  # wall-clock is a task property; max_steps is the agent's


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
    prompt_tokens: int = 0  # everything sent, cache reads included
    completion_tokens: int = 0
    cached_tokens: int = 0
    reasoning_tokens: int = 0
    cost_usd: float = 0.0
    ended_by: str = ""
    trajectory: str = ""  # path of <case_id>/trajectory.json, when an agent ran
