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

"""Eval case primitives.

Two taxonomies, orthogonal:

- **Passive** evals: world state is immutable (a frozen frame or replay, any
  time window). The model's output never feeds back into its input. Cheap,
  deterministic, repeatable.
- **Interactive** evals: actions feed back into observations; state is
  mutable. Needs sim or a real robot, scored by sampling the live memory
  store the robot's Recorder writes.

Suites are Python modules exporting ``SUITE: Suite`` (behavior is typed code;
JSON holds only data rows). memory is the source of truth for all input and
perception: context selectors return real :class:`~dimos.memory.stream.Stream`
objects and scoring reads :class:`~dimos.memory.store.base.Store`.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Generic, Protocol, TypeVar

from pydantic import BaseModel

from dimos.evals.scorers import exact, final

if TYPE_CHECKING:
    from dimos.e2e_tests.dim_sim_client import DimSimClient
    from dimos.memory.store.base import Store
    from dimos.memory.stream import Stream

T = TypeVar("T")
ResponseT = TypeVar("ResponseT", bound=BaseModel)

Select = Callable[["Store"], "Stream[Any, Any]"]
"""Context selector — hands the model real mem2 streams, whole or windowed::

    lambda s: s.streams.lidar.limit(1)
    lambda s: s.streams.odom.range_time(0, 600)
"""


@dataclass(frozen=True, kw_only=True)
class EvalResult:
    case_id: str
    outputs: str = ""
    score: float = 0.0
    passed: bool = False
    duration_s: float = 0.0
    error: str = ""
    series: tuple[tuple[float, float], ...] = ()  # (t, score) — interactive only
    transcript: str = ""  # path within the run dir, when an agent loop ran


class EvalRig(Protocol):
    """What a case may ask of the runner. :class:`EvalRunner` implements this
    structurally — no import cycle, mypy-checked at call sites, and a fake rig
    in tests is any object with these methods."""

    @property
    def blind(self) -> bool: ...
    @property
    def mcp_url(self) -> str: ...

    def open_dataset(self, name: str) -> Store: ...
    def live_store(self) -> Store: ...
    def encode(self, stream: Stream[Any, Any]) -> list[dict[str, Any]]: ...
    def ask(self, context: Sequence[dict[str, Any]], question: str) -> str: ...
    def ask_structured(
        self,
        context: Sequence[dict[str, Any]],
        question: str,
        schema: type[ResponseT],
    ) -> ResponseT: ...
    def call_skill(self, name: str, args: Mapping[str, object]) -> str: ...
    def agent_loop(self, case: EvalCase) -> str: ...
    def mcp_ready(self) -> bool: ...
    def setup_env(self, case: InteractiveEval) -> None: ...
    def check_env(self, case: InteractiveEval) -> None: ...
    def instruct(self, text: str) -> None: ...
    def sample(
        self, score: Callable[[Store], float], interval_s: float, timeout_s: float
    ) -> list[tuple[float, float]]: ...


@dataclass(frozen=True, kw_only=True)
class EvalCase(ABC):
    """Common surface the runner, report, and filters operate on.

    ``skill`` set -> score one tool call (no agent loop): ``detect()`` on a
    replay, ``grasp()`` in sim. ``skill`` empty -> subclass decides.
    """

    id: str
    inputs: str
    skill: str = ""
    skill_args: Mapping[str, object] = field(default_factory=dict)
    tags: frozenset[str] = frozenset()
    timeout_s: float = 60.0

    @abstractmethod
    def evaluate(self, rig: EvalRig) -> EvalResult:
        """Produce this case's result using the rig's resources."""

    def preflight(self, rig: EvalRig) -> None:
        """Raise with a precise message if this case cannot run on this rig.

        Cheap: resolves resources, reads no data, starts no processes.
        """
        if self.skill and not rig.mcp_ready():
            raise RuntimeError(f"{self.id}: needs MCP at {rig.mcp_url}, nothing listening")


@dataclass(frozen=True, kw_only=True)
class PassiveEval(EvalCase, Generic[T]):
    """World state immutable; ``T`` ties ``expected``/``parse``/``score``
    together so mypy checks the triple agrees per case."""

    expected: T
    parse: Callable[[str], T]
    score: Callable[[T, T], float] = exact
    context: tuple[Select, ...] = ()
    dataset: str = "go2_short"
    tools: bool = False  # True: full agent loop over the frozen store

    def evaluate(self, rig: EvalRig) -> EvalResult:
        store = rig.open_dataset(self.dataset)
        try:
            if self.skill:
                outputs = rig.call_skill(self.skill, self.skill_args)
            elif self.tools:
                outputs = rig.agent_loop(self)
            else:
                blocks = (
                    [] if rig.blind else [b for sel in self.context for b in rig.encode(sel(store))]
                )
                outputs = rig.ask(blocks, self.inputs)
        finally:
            store.stop()
        got = self.parse(outputs)
        return EvalResult(case_id=self.id, outputs=outputs, score=self.score(self.expected, got))

    def preflight(self, rig: EvalRig) -> None:
        store = rig.open_dataset(self.dataset)  # raises: dataset unresolvable
        try:
            for sel in self.context:
                sel(store)  # raises: "No stream 'x'. Available: [...]" — no data read
        finally:
            store.stop()
        if (self.skill or self.tools) and not rig.mcp_ready():
            raise RuntimeError(f"{self.id}: needs MCP at {rig.mcp_url}, nothing listening")


def _no_setup(sim: DimSimClient) -> None:
    return None


@dataclass(frozen=True, kw_only=True)
class InteractiveEval(EvalCase):
    """Actions feed back into observations. The case names its environment so
    the eval is reproducible; the runner only decides attach-vs-launch."""

    score: Callable[[Store], float]  # sampled every interval_s against live mem2
    aggregate: Callable[[Sequence[float]], float] = final
    interval_s: float = 1.0
    timeout_s: float = 300.0
    blueprint: str = "unitree-go2-agentic"
    simulator: str = "dimsim"  # "" = attach to a running dimos / real robot
    scene: str = "apartment"  # --dimsim-scene name (ScenePackage name later)
    setup: Callable[[DimSimClient], None] = _no_setup

    def evaluate(self, rig: EvalRig) -> EvalResult:
        rig.setup_env(self)
        if self.skill:
            rig.call_skill(self.skill, self.skill_args)
        else:
            rig.instruct(self.inputs)
        series = rig.sample(self.score, self.interval_s, self.timeout_s)
        if not series:
            return EvalResult(case_id=self.id, error=f"{self.id}: no samples collected")
        return EvalResult(
            case_id=self.id,
            score=self.aggregate([s for _, s in series]),
            series=tuple(series),
        )

    def preflight(self, rig: EvalRig) -> None:
        rig.check_env(self)


Suite = Sequence[EvalCase]
