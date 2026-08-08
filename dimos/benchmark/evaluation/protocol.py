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

"""The complete Evaluation extension point."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Protocol, runtime_checkable

from pydantic import BaseModel

from dimos.benchmark.evaluation.models import EvaluationReport
from dimos.benchmark.evaluation.progress import ProgressSink


@runtime_checkable
class CodePolicyRuntime(Protocol):
    """Factory supplied to evaluations for evaluation-owned agent sessions."""

    def open_session(self, environment: BaseModel) -> CodePolicySessionHandle: ...


@runtime_checkable
class CodePolicySessionHandle(Protocol):
    def __enter__(self) -> CodePolicySessionHandle: ...

    def __exit__(self, *args: object) -> None: ...

    def run(self, *, evaluation_protocol: str, task_input: str) -> AgentOutcome: ...


@dataclass(frozen=True)
class AgentOutcome:
    final_text: str
    tool_call_count: int
    duration_seconds: float


@dataclass(frozen=True)
class EvaluationContext:
    run_id: str
    spec_dir: Path
    workspace: Path
    agent: CodePolicyRuntime
    progress: ProgressSink | None


@runtime_checkable
class Evaluation(Protocol):
    """A complete executable evaluation with native result semantics."""

    name: str
    config_model: type[BaseModel]

    def run(self, config: BaseModel, context: EvaluationContext) -> EvaluationReport: ...
