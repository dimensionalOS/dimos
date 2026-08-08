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

"""Universal contracts for requesting and recording evaluation runs."""

from __future__ import annotations

from datetime import datetime
from pathlib import PurePosixPath
from typing import Annotated, Any, Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator


class EvaluationModel(BaseModel):
    """Strict immutable base for persisted evaluation contracts."""

    model_config = ConfigDict(extra="forbid", frozen=True, strict=True)


class EvaluationReference(EvaluationModel):
    name: str = Field(min_length=1)
    config: dict[str, Any] = Field(default_factory=dict)


class CodePolicyAgentConfig(EvaluationModel):
    profile: Literal["code-policy-v1"] = "code-policy-v1"
    model: Literal["gpt-5.6-luna"] = "gpt-5.6-luna"
    thinking_level: Literal["medium"] = "medium"


class EvaluationRunSpecification(EvaluationModel):
    schema_version: Literal["1.0"] = "1.0"
    evaluation: EvaluationReference
    agent: CodePolicyAgentConfig = Field(default_factory=CodePolicyAgentConfig)


class ArtifactReference(EvaluationModel):
    path: str = Field(min_length=1)
    label: str = Field(min_length=1)
    media_type: str | None = Field(default=None, min_length=1)

    @model_validator(mode="after")
    def path_is_safe_and_relative(self) -> ArtifactReference:
        path = PurePosixPath(self.path)
        if path.is_absolute() or not path.parts or ".." in path.parts:
            raise ValueError("artifact path must be a safe relative POSIX path")
        return self


SummaryValue = str | int | float | bool | None


class SummaryItem(EvaluationModel):
    key: str = Field(min_length=1, pattern=r"^[a-z][a-z0-9_]*$")
    label: str = Field(min_length=1)
    value: SummaryValue


class InlineNativeResult(EvaluationModel):
    kind: Literal["inline"] = "inline"
    value: Any


class ArtifactNativeResult(EvaluationModel):
    kind: Literal["artifact"] = "artifact"
    artifact: ArtifactReference


NativeResult = Annotated[
    InlineNativeResult | ArtifactNativeResult,
    Field(discriminator="kind"),
]


class EvaluationReport(EvaluationModel):
    summary: tuple[SummaryItem, ...] = ()
    native_result: NativeResult
    artifacts: tuple[ArtifactReference, ...] = ()


class EvaluationIdentity(EvaluationModel):
    name: str = Field(min_length=1)
    provider: str = Field(min_length=1)
    version: str = Field(min_length=1)


class RuntimeIdentity(EvaluationModel):
    profile: Literal["code-policy-v1"] = "code-policy-v1"
    driver: Literal["pi"] = "pi"
    driver_version: str = Field(min_length=1)
    model: str = Field(min_length=1)
    thinking_level: str = Field(min_length=1)


class EvaluationRunError(EvaluationModel):
    stage: Literal["evaluation", "publication"]
    error_type: str = Field(min_length=1)
    message: str = Field(min_length=1)


class EvaluationRun(EvaluationModel):
    schema_version: Literal["1.0"] = "1.0"
    run_id: str = Field(min_length=1)
    specification: EvaluationRunSpecification
    evaluation: EvaluationIdentity
    runtime: RuntimeIdentity
    status: Literal["completed", "failed", "cancelled"]
    started_at: datetime
    finished_at: datetime
    duration_seconds: float = Field(ge=0)
    report: EvaluationReport | None = None
    error: EvaluationRunError | None = None
    runtime_artifacts: tuple[ArtifactReference, ...] = ()
    prompt_evidence: tuple[ArtifactReference, ...] = ()

    @model_validator(mode="after")
    def status_matches_payload(self) -> EvaluationRun:
        if self.status == "completed" and self.report is None:
            raise ValueError("completed evaluation runs require a report")
        if self.status != "completed" and self.error is None:
            raise ValueError("non-completed evaluation runs require an error")
        if self.status == "completed" and self.error is not None:
            raise ValueError("completed evaluation runs cannot contain an error")
        if self.status != "completed" and self.report is not None:
            raise ValueError("non-completed evaluation runs cannot contain a report")
        return self
