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

"""Strict generic records for one attached-service agent evaluation attempt."""

from __future__ import annotations

from datetime import datetime
from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, JsonValue, model_validator

from dimos.benchmark.dimsim.models import OpaqueId, Pose2, Sha256

NonEmpty = Annotated[str, Field(min_length=1)]
AttemptId = Annotated[str, Field(pattern=r"^attempt_[0-9a-f]{32}$")]
OperationId = Annotated[str, Field(pattern=r"^operation_[0-9a-f]{32}$")]
CodePolicySessionId = Annotated[str, Field(pattern=r"^code_policy_session_[0-9a-f]{32}$")]


class EvalModel(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, strict=True)
    schema_version: Literal["1.0"] = "1.0"


class PiSettings(EvalModel):
    record_type: Literal["pi-settings"] = "pi-settings"
    model: NonEmpty
    thinking_level: Literal["off", "minimal", "low", "medium", "high"]
    auth_mode: Literal["environment", "subscription"]
    credential_env: NonEmpty | None = None
    credential_path: NonEmpty | None = None

    @model_validator(mode="after")
    def environment_auth_names_credential(self) -> PiSettings:
        if self.auth_mode == "environment" and (
            self.credential_env is None or self.credential_path is not None
        ):
            raise ValueError("environment authentication requires only credential_env")
        if self.auth_mode == "subscription" and (
            self.credential_path is None or self.credential_env is not None
        ):
            raise ValueError("subscription authentication requires only credential_path")
        return self


class InfrastructureTimeouts(EvalModel):
    record_type: Literal["infrastructure-timeouts"] = "infrastructure-timeouts"
    readiness_s: Annotated[float, Field(gt=0)]
    mcp_call_s: Annotated[float, Field(gt=0)]
    reset_s: Annotated[float, Field(gt=0)]
    evaluation_start_s: Annotated[float, Field(gt=0)]
    cancellation_s: Annotated[float, Field(gt=0)]


class DimSimBackendOptions(EvalModel):
    record_type: Literal["dimsim-backend-options"] = "dimsim-backend-options"
    endpoint: NonEmpty
    expected_scene_id: NonEmpty


class SmokeConfig(EvalModel):
    record_type: Literal["agent-eval-smoke-config"] = "agent-eval-smoke-config"
    release_root: NonEmpty
    task_id: OpaqueId
    output_root: NonEmpty
    mcp_endpoint: NonEmpty
    pi: PiSettings
    timeouts: InfrastructureTimeouts
    episode_timeout_s: Annotated[float, Field(gt=0)] = 180.0
    dimsim: DimSimBackendOptions


class ResolvedSmokeConfig(EvalModel):
    record_type: Literal["agent-eval-resolved-config"] = "agent-eval-resolved-config"
    release_root: NonEmpty
    task_id: OpaqueId
    output_root: NonEmpty
    mcp_endpoint: NonEmpty
    pi_model: NonEmpty
    pi_thinking_level: Literal["off", "minimal", "low", "medium", "high"]
    auth_mode: Literal["environment", "subscription"]
    credential_binding_sha256: Sha256
    timeouts: InfrastructureTimeouts
    episode_timeout_s: Annotated[float, Field(gt=0)]
    dimsim: DimSimBackendOptions


class BackendEpisodeReference(EvalModel):
    record_type: Literal["backend-episode-reference"] = "backend-episode-reference"
    backend: NonEmpty
    episode_id: NonEmpty
    opaque: dict[str, JsonValue] = Field(default_factory=dict)


class ResetReceipt(EvalModel):
    record_type: Literal["backend-reset-receipt"] = "backend-reset-receipt"
    attempt_id: AttemptId
    operation_id: OperationId
    task_id: OpaqueId
    episode: BackendEpisodeReference
    requested_pose: Pose2
    applied_pose: Pose2
    reset_generation: Annotated[int, Field(ge=0)]
    verified_source_revisions: dict[str, NonEmpty] = Field(min_length=1)
    source_digest: Sha256
    initial_predicate_satisfied: bool
    acknowledged_at: datetime


class EvaluationHandle(EvalModel):
    record_type: Literal["backend-evaluation-handle"] = "backend-evaluation-handle"
    attempt_id: AttemptId
    operation_id: OperationId
    task_id: OpaqueId
    episode: BackendEpisodeReference
    opaque: dict[str, JsonValue] = Field(default_factory=dict)


class ArtifactReference(EvalModel):
    record_type: Literal["artifact-reference"] = "artifact-reference"
    path: NonEmpty
    sha256: Sha256
    size_bytes: Annotated[int, Field(ge=0)]

    @model_validator(mode="after")
    def path_is_relative(self) -> ArtifactReference:
        if self.path.startswith("/") or ".." in self.path.split("/"):
            raise ValueError("artifact path must be attempt-relative")
        return self


class NativeResultReference(EvalModel):
    record_type: Literal["native-result-reference"] = "native-result-reference"
    backend: NonEmpty
    artifact: ArtifactReference
    native_result_id: NonEmpty


class LifecycleEvent(EvalModel):
    record_type: Literal["agent-eval-lifecycle-event"] = "agent-eval-lifecycle-event"
    sequence: Annotated[int, Field(ge=1)]
    attempt_id: AttemptId
    operation_id: OperationId | None = None
    occurred_at: datetime
    monotonic_offset_s: Annotated[float, Field(ge=0)]
    kind: NonEmpty
    payload: dict[str, JsonValue] = Field(default_factory=dict)


class AttemptManifest(EvalModel):
    record_type: Literal["agent-eval-attempt-manifest"] = "agent-eval-attempt-manifest"
    attempt_id: AttemptId
    created_at: datetime
    trust_mode: Literal["trusted-unsandboxed-simulation"]
    source_release_root: NonEmpty
    release_id: OpaqueId
    task_id: OpaqueId
    contract_sha256: Sha256
    expected_outcome_id: OpaqueId
    expected_outcome_sha256: Sha256
    code_policy_session_id: CodePolicySessionId | None = None
    pi_session_id: NonEmpty | None = None
    verified_source_revisions: dict[str, NonEmpty] = Field(default_factory=dict)
    artifacts: tuple[ArtifactReference, ...] = ()


class NormalizedOutcome(EvalModel):
    record_type: Literal["agent-eval-outcome"] = "agent-eval-outcome"
    attempt_id: AttemptId
    attempt_status: Literal["completed", "failed"]
    task_result: Literal["passed", "failed", "not_evaluated"]
    terminal_stage: NonEmpty
    reason: NonEmpty
    required_artifacts_complete: bool
    native_result: NativeResultReference | None = None
    finished_at: datetime
    duration_s: Annotated[float, Field(ge=0)]

    @model_validator(mode="after")
    def infrastructure_and_task_states_are_consistent(self) -> NormalizedOutcome:
        if self.attempt_status == "failed" and self.task_result != "not_evaluated":
            raise ValueError("failed infrastructure cannot report a task result")
        if self.attempt_status == "completed" and self.task_result == "not_evaluated":
            raise ValueError("completed evaluation must report pass or fail")
        if self.attempt_status == "completed" and not self.required_artifacts_complete:
            raise ValueError("completed evaluation requires complete artifacts")
        return self
