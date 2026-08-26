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
import math
from types import MappingProxyType
from typing import TYPE_CHECKING, Any, Generic, Protocol, TypeVar

from dimos.evals.scorers import exact, final
from dimos.sim2.evaluation import TrialIsolationMode

if TYPE_CHECKING:
    from dimos.e2e_tests.dim_sim_client import DimSimClient
    from dimos.memory.store.base import Store
    from dimos.memory.stream import Stream
    from dimos.porcelain.dimos import Dimos
    from dimos.sim2.episodes import PublicEpisodeContext
    from dimos.sim2.evaluation import (
        EpisodeEvaluationResult,
        PreparedEpisode,
        ProviderEpisodeRequestContract,
    )

T = TypeVar("T")

Select = Callable[["Store"], "Stream[Any, Any]"]
"""Context selector — hands the model real mem2 streams, whole or windowed::

    lambda s: s.streams.lidar.limit(1)
    lambda s: s.streams.odom.range_time(0, 600)
"""

InteractiveAction = Callable[["Dimos", "PublicEpisodeContext"], str]
"""Public DimOS behavior run after an exact interactive episode is reset."""

InteractiveOutputScore = Callable[[str, "PreparedEpisode"], float]
"""Score public action output against evaluator-only prepared episode truth."""


@dataclass(frozen=True, kw_only=True)
class EvalResult:
    case_id: str
    provider: str = ""
    episode_id: str = ""
    episode_case_id: str = ""
    outputs: str = ""
    score: float = 0.0
    passed: bool = False
    duration_s: float = 0.0
    error: str = ""
    series: tuple[tuple[float, float], ...] = ()  # (t, score) — interactive only
    trials: tuple[InteractiveTrialResult, ...] = ()
    transcript: str = ""  # path within the run dir, when an agent loop ran
    oracle: str = ""  # private provider result; never written to the public Store
    metrics: Mapping[str, float] = field(default_factory=dict)

    def to_json_dict(self) -> dict[str, Any]:
        """Return the stable JSON representation written by the eval runner."""

        return {
            "case_id": self.case_id,
            "provider": self.provider,
            "episode_id": self.episode_id,
            "episode_case_id": self.episode_case_id,
            "outputs": self.outputs,
            "score": self.score,
            "passed": self.passed,
            "duration_s": self.duration_s,
            "error": self.error,
            "series": list(self.series),
            "trials": [trial.to_json_dict() for trial in self.trials],
            "transcript": self.transcript,
            "oracle": self.oracle,
            "metrics": dict(self.metrics),
        }


@dataclass(frozen=True, kw_only=True)
class InteractiveTrialResult:
    trial_index: int
    sample_index: int
    episode_id: str = ""
    sample_digest: str = ""
    outputs: str = ""
    score: float = 0.0
    error: str = ""
    oracle: str = ""
    metrics: Mapping[str, float] = field(default_factory=dict)
    provenance: Mapping[str, str | int | float | bool] = field(default_factory=dict)
    series: tuple[tuple[float, float], ...] = ()
    transcript: str = ""  # path within the run dir, when an agent loop ran

    def __post_init__(self) -> None:
        for field_name in ("trial_index", "sample_index"):
            value = getattr(self, field_name)
            if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                raise ValueError(f"interactive trial {field_name} must be non-negative")
        if not math.isfinite(self.score):
            raise ValueError("interactive trial score must be finite")
        if self.sample_digest and (
            len(self.sample_digest) != 64
            or any(character not in "0123456789abcdef" for character in self.sample_digest)
        ):
            raise ValueError("interactive trial sample digest must be lowercase SHA-256")
        metrics = {str(name): float(value) for name, value in self.metrics.items()}
        if any(not name.strip() for name in metrics) or any(
            not math.isfinite(value) for value in metrics.values()
        ):
            raise ValueError("interactive trial metrics require names and finite values")
        provenance: dict[str, str | int | float | bool] = {}
        for raw_name, value in self.provenance.items():
            name = str(raw_name).strip()
            if not name:
                raise ValueError("interactive trial provenance keys must not be empty")
            if not isinstance(value, str | int | float | bool):
                raise TypeError("interactive trial provenance values must be scalar")
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError("interactive trial provenance floats must be finite")
            provenance[name] = value
        series = tuple((float(elapsed), float(value)) for elapsed, value in self.series)
        if any(
            not math.isfinite(elapsed) or elapsed < 0.0 or not math.isfinite(value)
            for elapsed, value in series
        ):
            raise ValueError("interactive trial series values must be finite and time-positive")
        object.__setattr__(self, "metrics", MappingProxyType(metrics))
        object.__setattr__(self, "provenance", MappingProxyType(provenance))
        object.__setattr__(self, "series", series)

    def to_json_dict(self) -> dict[str, Any]:
        """Return a JSON-safe trial record without weakening runtime immutability."""

        return {
            "trial_index": self.trial_index,
            "sample_index": self.sample_index,
            "episode_id": self.episode_id,
            "sample_digest": self.sample_digest,
            "outputs": self.outputs,
            "score": self.score,
            "error": self.error,
            "oracle": self.oracle,
            "metrics": dict(self.metrics),
            "provenance": dict(self.provenance),
            "series": list(self.series),
            "transcript": self.transcript,
        }


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
    def call_skill(self, name: str, args: Mapping[str, object]) -> str: ...
    def agent_loop(self, case: EvalCase) -> str: ...
    def mcp_ready(self) -> bool: ...
    def setup_env(self, case: InteractiveEval) -> None: ...
    def check_env(self, case: InteractiveEval) -> None: ...
    def instruct(self, text: str) -> None: ...
    def episode_instruction(self) -> str: ...
    def run_action(self, action: InteractiveAction) -> str: ...
    def score_output(self, score: InteractiveOutputScore, output: str) -> tuple[float, str]: ...
    def episode_identity(self) -> tuple[str, str, str]: ...
    def sample(
        self, score: Callable[[Store], float], interval_s: float, timeout_s: float
    ) -> list[tuple[float, float]]: ...
    def sample_episode(
        self, interval_s: float, timeout_s: float
    ) -> tuple[list[tuple[float, float]], EpisodeEvaluationResult]: ...
    def run_interactive_trials(self, case: InteractiveEval) -> EvalResult: ...


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
class InteractiveEnvironment:
    """Legacy or hardware environment for an episode-less interactive eval.

    Provider-backed episodes own their launch environment and do not use this
    declaration.
    """

    simulator: str = ""  # empty means attach to a running DimOS instance
    scene: str = ""
    setup: Callable[[DimSimClient], None] = _no_setup

    def __post_init__(self) -> None:
        if self.scene and not self.simulator:
            raise ValueError("interactive environment scene requires a simulator")


@dataclass(frozen=True, kw_only=True)
class InteractiveEval(EvalCase):
    """Evaluate one DimOS blueprint in a provider-owned physical episode."""

    # Exactly one scoring path is selected:
    # - score: sample normal public mem2 state;
    # - output_score: compare one public action result with prepared episode truth;
    # - neither, with episode set: poll the provider's private physical goal.
    score: Callable[[Store], float] | None = None
    output_score: InteractiveOutputScore | None = None
    aggregate: Callable[[Sequence[float]], float] = final
    interval_s: float = 1.0
    timeout_s: float = 300.0
    inputs: str = ""
    episode: ProviderEpisodeRequestContract | None = None
    action: InteractiveAction | None = None
    instruction_override: str | None = None
    trials: int = 1
    trial_aggregate: Callable[[Sequence[float]], float] = final
    trial_isolation: TrialIsolationMode = TrialIsolationMode.EPISODE_BOUNDARY
    blueprint: str = "unitree-go2-agentic"
    required_modules: tuple[str, ...] = ()
    required_roles: tuple[str, ...] = ()
    environment: InteractiveEnvironment | None = None

    def __post_init__(self) -> None:
        blueprint = self.blueprint.strip()
        if not blueprint:
            raise ValueError("interactive eval blueprint must not be empty")
        object.__setattr__(self, "blueprint", blueprint)
        if self.score is not None and self.output_score is not None:
            raise ValueError("interactive eval cannot combine store and output scoring")
        if self.episode is None and self.score is None:
            raise ValueError("interactive eval without an episode requires a public Store score")
        if self.episode is not None:
            from dimos.sim2.evaluation import ProviderEpisodeRequestContract

            if not isinstance(self.episode, ProviderEpisodeRequestContract):
                raise TypeError("interactive episode must expose provider_name and case_id")
            if not self.episode.provider_name.strip() or not self.episode.case_id.strip():
                raise ValueError("interactive episode provider_name and case_id must not be empty")
            if self.environment is not None:
                raise ValueError("provider-backed interactive eval cannot declare an environment")
        elif self.environment is None:
            raise ValueError("interactive eval without an episode requires an environment")
        if self.output_score is not None and self.episode is None:
            raise ValueError("interactive output scoring requires an exact episode")
        if self.output_score is not None and self.action is None and not self.skill:
            raise ValueError("interactive output scoring requires an action or skill result")
        if self.action is not None and self.episode is None:
            raise ValueError("public action callbacks require an exact episode")
        if self.action is not None and self.skill:
            raise ValueError("interactive eval cannot combine action and skill")
        if isinstance(self.trials, bool) or not isinstance(self.trials, int) or self.trials < 1:
            raise ValueError("interactive eval trials must be a positive integer")
        if self.trials > 1 and self.episode is None:
            raise ValueError("repeated interactive trials require an episode provider")
        for field_name in ("required_modules", "required_roles"):
            values = tuple(value.strip() for value in getattr(self, field_name))
            if any(not value for value in values) or len(values) != len(set(values)):
                raise ValueError(
                    f"interactive eval {field_name} must contain unique non-empty names"
                )
            object.__setattr__(self, field_name, values)
        object.__setattr__(self, "trial_isolation", TrialIsolationMode(self.trial_isolation))
        if self.instruction_override is not None:
            override = self.instruction_override.strip()
            if not override:
                raise ValueError("interactive instruction_override must not be empty")
            if self.episode is None:
                raise ValueError("interactive instruction_override requires an exact episode")
            object.__setattr__(self, "instruction_override", override)

    @property
    def provider_name(self) -> str:
        """Return the provider selected by the episode reference."""

        return "" if self.episode is None else self.episode.provider_name

    def evaluate(self, rig: EvalRig) -> EvalResult:
        if self.trials > 1:
            return rig.run_interactive_trials(self)

        rig.setup_env(self)
        provider, episode_id, episode_case_id = (
            rig.episode_identity() if self.episode is not None else ("", "", "")
        )
        outputs = ""
        if self.action is not None:
            outputs = rig.run_action(self.action)
        elif self.skill:
            outputs = rig.call_skill(self.skill, self.skill_args)
        else:
            instruction = (
                self.instruction_override or rig.episode_instruction()
                if self.episode is not None
                else self.inputs
            )
            rig.instruct(instruction)

        oracle = ""
        metrics: Mapping[str, float] = {}
        if self.output_score is not None:
            value, oracle = rig.score_output(self.output_score, outputs)
            series = [(0.0, value)]
        elif self.score is not None:
            series = rig.sample(self.score, self.interval_s, self.timeout_s)
        else:
            series, evaluation = rig.sample_episode(self.interval_s, self.timeout_s)
            oracle = evaluation.summary
            metrics = evaluation.metrics
        if not series:
            return EvalResult(case_id=self.id, error=f"{self.id}: no samples collected")
        return EvalResult(
            case_id=self.id,
            provider=provider,
            episode_id=episode_id,
            episode_case_id=episode_case_id,
            outputs=outputs,
            score=self.aggregate([value for _elapsed, value in series]),
            series=tuple(series),
            oracle=oracle,
            metrics=metrics,
        )

    def preflight(self, rig: EvalRig) -> None:
        rig.check_env(self)


Suite = Sequence[EvalCase]
