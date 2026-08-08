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

"""Small tagged contracts for one frozen agent-evaluation case."""

from __future__ import annotations

import math
from pathlib import PurePosixPath
from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator


class BaseEvalModel(BaseModel):
    """Strict immutable base for the compact evaluation contracts."""

    model_config = ConfigDict(extra="forbid", frozen=True, strict=True)
    schema_version: Literal["1.0"] = "1.0"


NonEmpty = Annotated[str, Field(min_length=1)]


class FrozenRecordingSource(BaseEvalModel):
    kind: Literal["frozen_memory"] = "frozen_memory"
    recording: NonEmpty
    progress: float = Field(ge=0, le=1, allow_inf_nan=False)

    @model_validator(mode="after")
    def finite_progress(self) -> FrozenRecordingSource:
        if not math.isfinite(self.progress):
            raise ValueError("recording progress must be finite")
        return self


class IntegerQuestionTask(BaseEvalModel):
    kind: Literal["integer_question"] = "integer_question"
    prompt: NonEmpty
    answer_marker: Literal["ANSWER:"] = "ANSWER:"


class ExactIntegerValidatorRef(BaseEvalModel):
    kind: Literal["exact_integer"] = "exact_integer"
    revision: NonEmpty
    private_path: NonEmpty

    @model_validator(mode="after")
    def safe_relative_path(self) -> ExactIntegerValidatorRef:
        path = PurePosixPath(self.private_path)
        if path.is_absolute() or not path.parts or ".." in path.parts:
            raise ValueError("validator private_path must be a safe relative path")
        return self


SourceSpec = Annotated[FrozenRecordingSource, Field(discriminator="kind")]
TaskSpec = Annotated[IntegerQuestionTask, Field(discriminator="kind")]
ValidatorRef = Annotated[ExactIntegerValidatorRef, Field(discriminator="kind")]


class EvalCase(BaseEvalModel):
    case_id: NonEmpty
    source: SourceSpec
    task: TaskSpec
    validator: ValidatorRef


class PiAgentConfig(BaseEvalModel):
    backend: Literal["pi"] = "pi"
    model: Literal["gpt-5.6-luna"] = "gpt-5.6-luna"
    thinking_level: Literal["medium"] = "medium"
    api_key_env: str = Field(default="OPENAI_API_KEY", min_length=1)


class EvalRunConfig(BaseEvalModel):
    agent: PiAgentConfig = Field(default_factory=PiAgentConfig)


class CompactEvalResult(BaseEvalModel):
    case_id: str
    recording: str
    progress: float
    model: str
    thinking_level: str
    final_response: str = ""
    prediction_status: Literal["parsed", "invalid", "not_evaluated"]
    integer_answer: int | None = None
    passed: bool | None = None
    validator_revision: str
    tool_call_count: int = Field(ge=0)
    duration_seconds: float = Field(ge=0)
    infra_error: str | None = None

    @property
    def attempt_status(self) -> Literal["completed", "failed"]:
        return "failed" if self.infra_error is not None else "completed"

    @property
    def task_result(self) -> Literal["passed", "failed", "not_evaluated"]:
        if self.passed is None:
            return "not_evaluated"
        return "passed" if self.passed else "failed"
