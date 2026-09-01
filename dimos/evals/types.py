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
  :class:`Trajectory` with every provider request/response saved whole.
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


# -- trajectory ------------------------------------------------------------------------


@dataclass(kw_only=True)
class ToolCall:
    id: str
    name: str
    args: dict[str, Any]
    result: str | None = None  # None when the call never completed


@dataclass(frozen=True, kw_only=True)
class Step:
    """One model call, its completed tool executions, and exact provider payloads."""

    index: int
    t: float  # seconds since run start
    message: str  # assistant text for this step
    reasoning: str = ""  # readable reasoning text, "" when the provider returns none
    tool_calls: tuple[ToolCall, ...] = ()
    input_tokens: int = 0  # non-cached input only; cache reads are excluded
    output_tokens: int = 0
    reasoning_tokens: int = 0  # the part of output_tokens spent reasoning
    latency_s: float = 0.0
    request: Path  # the exact payload sent to the provider for this call
    response: Path  # the exact payload received


EndedBy = Literal["answer", "max_steps", "timeout", "error"]


@dataclass(frozen=True, kw_only=True)
class Trajectory:
    """Append-only: one :class:`Step` per model call, in order. The messages
    array actually sent per call is *not* the previous call plus one message
    (harnesses compact and re-inject), so each call's request is a whole
    record under ``raw_dir``, never reconstructed from the steps."""

    final_answer: str  # last non-tool assistant message, "" if none
    steps: tuple[Step, ...]
    model: str  # what actually ran, as reported by the provider
    input_tokens: int = 0  # non-cached input; total sent = input_tokens + cached_tokens
    output_tokens: int = 0
    cached_tokens: int = 0  # input read from the provider's prompt cache
    reasoning_tokens: int = 0  # the part of output_tokens spent reasoning
    cost: float = 0.0  # USD
    duration_s: float
    ended_by: EndedBy
    raw_dir: Path


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
        """True: ``start()`` launches a blueprint and returns an MCP url."""

    def preflight(self, agent: Agent) -> None:
        """Raise if this environment can't run *agent* (it adds modules and
        nothing is launched here; a stream a case selected is missing).
        Cheap: no data read, no process started."""

    def start(self, modules: str, trace_dir: Path | None = None) -> RunningEnvironment:
        """Start, with *modules* (what the agent adds) on top of this
        environment's own stack where it launches one. *trace_dir* is where
        a launched ``McpClient`` writes its raw LLM request/response bodies;
        an environment that launches nothing ignores it."""

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
    encodes the whole recording into one prompt, ``Blind`` never looks, ``Pi``
    and ``McpClientAgent`` act through tools. No agent knows anything about
    any particular case.

    Everything that decides what an agent does (model, prompt, step limit,
    what it adds to the stack) is a constructor argument; the runner records
    ``vars(agent)``. A limit an agent cannot honor is not a parameter it has.
    """

    modules: str
    """Blueprint atoms this agent adds to a ``Sim`` case's stack (``dimos run
    <case stack> <modules>``); ``""`` for none. An environment that launches
    nothing rejects a non-empty value in ``preflight``."""

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
    steps: int = 0
    input_tokens: int = 0  # non-cached input; total sent = input_tokens + cached_tokens
    output_tokens: int = 0
    cached_tokens: int = 0
    reasoning_tokens: int = 0
    cost: float = 0.0  # USD
    ended_by: str = ""
    trajectory: str = ""  # path of <case_id>/trajectory.json, when an agent ran
