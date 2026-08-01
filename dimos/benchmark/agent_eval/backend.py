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

"""Simulator-independent backend seam used by the local evaluation runner."""

from __future__ import annotations

from typing import Any, Protocol

from pydantic import Field, JsonValue

from dimos.benchmark.agent_eval.models import (
    AttemptId,
    BackendEpisodeReference,
    EvalModel,
    EvaluationHandle,
    NonEmpty,
    OperationId,
    ResetReceipt,
)
from dimos.benchmark.dimsim.models import OpaqueId, Pose2, Sha256


class BackendReadiness(EvalModel):
    record_type: str = "backend-readiness"
    backend: NonEmpty
    ready: bool
    detail: NonEmpty


class BackendResetRequest(EvalModel):
    record_type: str = "backend-reset-request"
    attempt_id: AttemptId
    operation_id: OperationId
    task_id: OpaqueId
    episode: BackendEpisodeReference
    start_pose: Pose2
    source_revisions: dict[str, NonEmpty] = Field(min_length=1)


class BackendEvaluationRequest(EvalModel):
    record_type: str = "backend-evaluation-request"
    attempt_id: AttemptId
    operation_id: OperationId
    task_id: OpaqueId
    episode: BackendEpisodeReference
    contract_digest: Sha256
    contract_payload: dict[str, JsonValue]


class SimulatorBackend(Protocol):
    """Narrow lifecycle contract; native result values remain backend-owned."""

    def readiness(self, timeout_s: float) -> BackendReadiness: ...

    def reset(self, request: BackendResetRequest, timeout_s: float) -> ResetReceipt: ...

    def start_evaluation(
        self, request: BackendEvaluationRequest, timeout_s: float
    ) -> EvaluationHandle: ...

    def wait_result(self, handle: EvaluationHandle, timeout_s: float) -> Any | None: ...

    def cancel(self, handle: EvaluationHandle, timeout_s: float) -> None: ...

    def cleanup(self) -> None: ...
