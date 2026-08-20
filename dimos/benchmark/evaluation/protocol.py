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

"""Public contracts for native evaluations and CodePolicy execution."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from enum import StrEnum
import json
from pathlib import Path
from typing import TYPE_CHECKING, Any, Literal, Protocol, runtime_checkable

from pydantic import BaseModel

from dimos.benchmark.evaluation.models import (
    ArtifactReference,
    EvaluationReport,
    RuntimeIdentity,
)
from dimos.benchmark.evaluation.progress import ProgressSink

if TYPE_CHECKING:
    from dimos.memory2.store.base import Store
    from dimos.memory2.stream import Stream


@dataclass(frozen=True)
class PolicyArtifact:
    """A task-level callable captured during exploration."""

    source_path: Path
    serialized_path: Path
    sha256: str


@dataclass(frozen=True)
class PolicyExecution:
    status: Literal["completed", "policy_error", "stopped", "infrastructure_error"]
    duration_seconds: float
    error: str | None
    output: str


@dataclass(frozen=True)
class TrialOutcome:
    success: bool
    reward: float | None
    status: Literal["completed", "policy_error", "timed_out", "infrastructure_error"]
    error: str | None
    duration_seconds: float


@dataclass(frozen=True)
class TrialRun:
    """Read-only postmortem handle for one fully stopped policy trial."""

    run_id: str
    outcome: TrialOutcome
    artifacts: Path
    log_path: Path
    memory_path: Path
    policy_output: str

    def read_logs(
        self,
        *,
        module: str | None = None,
        tail: int | None = None,
    ) -> tuple[dict[str, object], ...]:
        from dimos.core.log_viewer import read_log

        records = [json.loads(line) for line in read_log(self.log_path, count=None)]
        if module is not None:
            records = [record for record in records if record.get("module") == module]
        if tail is not None:
            if tail < 0:
                raise ValueError("tail must be non-negative")
            records = records[-tail:] if tail else []
        return tuple(records)

    def open_memory(self) -> Store:
        """Open the completed trial's Memory2 recording read-only."""
        from dimos.memory2.store.sqlite import SqliteStore

        return SqliteStore(path=str(self.memory_path), must_exist=True, read_only=True)


class EvidenceCategory(StrEnum):
    """Recorded evidence classes available for deterministic drilldown."""

    OUTCOME = "outcome"
    TIMELINE = "timeline"
    LOGS = "logs"
    POLICY_OUTPUT = "policy_output"
    MEMORY = "memory"
    FRAMES = "frames"
    ARTIFACTS = "artifacts"


@dataclass(frozen=True)
class EvidenceFrame:
    """A stable pointer to a recorded visual observation."""

    stream: str
    position: Literal["initial", "terminal", "failure_adjacent"]
    timestamp: float


@dataclass(frozen=True)
class TrialEvidenceSummary:
    """Small non-diagnostic view returned automatically to the Agent."""

    candidate_id: str
    success: bool
    status: str
    duration_seconds: float
    remaining_submissions: int
    categories: tuple[EvidenceCategory, ...]
    frames: tuple[EvidenceFrame, ...]


@dataclass(frozen=True)
class TrialEvidence:
    """Progressively disclosed, read-only evidence for one Debug Trial."""

    candidate_id: str
    trial: TrialRun
    remaining_submissions: int

    @property
    def summary(self) -> TrialEvidenceSummary:
        categories = [
            EvidenceCategory.OUTCOME,
            EvidenceCategory.TIMELINE,
            EvidenceCategory.LOGS,
            EvidenceCategory.POLICY_OUTPUT,
            EvidenceCategory.MEMORY,
            EvidenceCategory.ARTIFACTS,
        ]
        frames = self.frame_index()
        if frames:
            categories.append(EvidenceCategory.FRAMES)
        return TrialEvidenceSummary(
            candidate_id=self.candidate_id,
            success=self.trial.outcome.success,
            status=self.trial.outcome.status,
            duration_seconds=self.trial.outcome.duration_seconds,
            remaining_submissions=self.remaining_submissions,
            categories=tuple(categories),
            frames=frames,
        )

    def timeline(
        self, *, module: str | None = None, tail: int | None = None
    ) -> tuple[dict[str, object], ...]:
        """Return recorded log events in their persisted order."""
        return self.trial.read_logs(module=module, tail=tail)

    def logs(
        self, *, module: str | None = None, tail: int | None = None
    ) -> tuple[dict[str, object], ...]:
        """Return all logs, or deterministically filter them by module."""
        return self.trial.read_logs(module=module, tail=tail)

    @property
    def policy_output(self) -> str:
        return self.trial.policy_output

    @property
    def artifacts(self) -> Path:
        return self.trial.artifacts

    def open_memory(self) -> Store:
        return self.trial.open_memory()

    def frame_index(self) -> tuple[EvidenceFrame, ...]:
        """Index initial and terminal frames from recorded color streams."""
        if not self.trial.memory_path.is_file():
            return ()
        frames: list[EvidenceFrame] = []
        with self.open_memory() as memory:
            for name in sorted(memory.list_streams()):
                if not name.endswith("color_image"):
                    continue
                stream: Stream[Any] = memory.stream(name)
                if not stream.exists() or stream.count() == 0:
                    continue
                initial = stream.first()
                terminal = stream.last()
                frames.append(EvidenceFrame(name, "initial", initial.ts))
                frames.append(EvidenceFrame(name, "terminal", terminal.ts))
        return tuple(frames)

    def frame(self, stream: str, position: Literal["initial", "terminal"] = "terminal") -> object:
        """Load one selected visual observation, or report it unavailable."""
        with self.open_memory() as memory:
            if stream not in memory.list_streams() or not stream.endswith("color_image"):
                raise LookupError(f"visual evidence unavailable for stream {stream!r}")
            recorded: Stream[Any] = memory.stream(stream)
            if not recorded.exists() or recorded.count() == 0:
                raise LookupError(f"visual evidence unavailable for stream {stream!r}")
            return recorded.first() if position == "initial" else recorded.last()

    def __repr__(self) -> str:
        summary = self.summary
        return (
            "TrialEvidence("
            f"candidate_id={summary.candidate_id!r}, success={summary.success!r}, "
            f"status={summary.status!r}, duration_seconds={summary.duration_seconds:.3f}, "
            f"remaining_submissions={summary.remaining_submissions}, "
            f"categories={tuple(category.value for category in summary.categories)!r}, "
            f"frames={summary.frames!r})"
        )


@dataclass(frozen=True)
class PolicyCandidate:
    """An immutable submitted policy and its Debug Trial evidence."""

    id: str
    policy: PolicyArtifact
    evidence: TrialEvidence

    @property
    def trial(self) -> TrialRun:
        return self.evidence.trial

    def __repr__(self) -> str:
        return f"PolicyCandidate(id={self.id!r}, evidence={self.evidence!r})"


DebugTrialSubmitter = Callable[[PolicyArtifact, int, Path], TrialRun]


@runtime_checkable
class EvaluationRuntime(Protocol):
    """Evaluated-behavior runtime selected by an Evaluation."""

    @property
    def identity(self) -> RuntimeIdentity: ...

    @property
    def prompt_evidence(self) -> tuple[ArtifactReference, ...]: ...

    @property
    def runtime_artifacts(self) -> tuple[ArtifactReference, ...]: ...


@dataclass(frozen=True)
class ExplorationOutcome:
    status: Literal["valid", "invalid"]
    policy: PolicyArtifact | None
    trials: tuple[TrialRun, ...]
    final_text: str
    tool_call_count: int
    duration_seconds: float
    error: str | None = None


@dataclass(frozen=True)
class LiveAgentOutcome:
    final_text: str
    tool_call_count: int
    duration_seconds: float


@runtime_checkable
class LiveAgentExecutionHandle(Protocol):
    """A prepared live model session waiting behind an evaluator start gate."""

    def start(self) -> None: ...

    def failure(self) -> BaseException | None: ...

    def finish(self) -> LiveAgentOutcome: ...


@runtime_checkable
class LiveAgentRuntime(EvaluationRuntime, Protocol):
    def prepare(
        self,
        *,
        prompt: str,
        system_prompt: str,
        memory_path: Path,
        episode_timeout_s: float,
    ) -> LiveAgentExecutionHandle: ...


@runtime_checkable
class CodePolicyRuntime(EvaluationRuntime, Protocol):
    """Fixed Pi runtime injected into complete native evaluations."""

    def explore(
        self,
        *,
        evaluation_protocol: str,
        task_input: str,
        submit_debug_trial: DebugTrialSubmitter,
        max_submissions: int = 5,
    ) -> ExplorationOutcome: ...

    def prepare(
        self,
        policy: PolicyArtifact,
        *,
        memory_path: Path,
        startup_timeout_s: float,
    ) -> PolicyExecutionHandle: ...


@runtime_checkable
class PolicyExecutionHandle(Protocol):
    """A loaded policy waiting behind an evaluator-owned start gate."""

    def start(self) -> None: ...

    def finish(self, *, grace_s: float = 1.0) -> PolicyExecution: ...


@dataclass(frozen=True)
class EvaluationContext:
    run_id: str
    spec_dir: Path
    workspace: Path
    runtime: EvaluationRuntime
    progress: ProgressSink | None


@runtime_checkable
class Evaluation(Protocol):
    """A complete benchmark integration with native result semantics."""

    name: str
    runtime_profile: Literal["code-policy-v1", "live-agent-v1"]
    config_model: type[BaseModel]

    def run(self, config: BaseModel, context: EvaluationContext) -> EvaluationReport: ...
