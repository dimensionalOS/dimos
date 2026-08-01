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

"""DimSim implementation of the simulator-agnostic evaluation backend seam."""

from __future__ import annotations

from datetime import UTC, datetime
import hashlib
import time
from typing import Protocol

from dimos.benchmark.agent_eval.backend import (
    BackendEvaluationRequest,
    BackendReadiness,
    BackendResetRequest,
)
from dimos.benchmark.agent_eval.models import (
    EvaluationHandle,
    ResetReceipt,
)
from dimos.benchmark.dimsim.models import (
    NavigateContract,
    Pose2,
    SceneOracleView,
    TaskContract,
)
from dimos.benchmark.dimsim.utilities import model_bytes, oracle_view_digest
from dimos.benchmark.spatial.utilities import canonical_json
from dimos.simulation.dimsim.attached_control import AttachedDimSimControl
from dimos.simulation.dimsim.evaluation import (
    AuthoritativeBodySample,
    DimSimNativeEvaluator,
    DimSimNativeResult,
    DimSimResetCoordinator,
    DimSimResetRequest,
    NavigationRubric,
)


class _AttachedControl(Protocol):
    def start(self) -> None: ...

    def stop(self) -> None: ...

    def clear_motion(self) -> None: ...

    def settle_motion(self) -> None: ...

    def teleport(self, pose: Pose2) -> None: ...

    def wait_body_sample(self, timeout_s: float) -> AuthoritativeBodySample: ...

    def latest_body_sample(self, timeout_s: float) -> AuthoritativeBodySample: ...

    def oracle_view(self) -> SceneOracleView: ...


class DimSimEvaluationBackend:
    """Attach to one generated task and retain raw native result objects."""

    backend_name = "dimsim-attached-v1"

    def __init__(
        self,
        *,
        host: str,
        port: int,
        selected_contract: TaskContract,
        control: _AttachedControl | None = None,
        oracle_timeout_s: float = 30.0,
    ) -> None:
        self._control = control or AttachedDimSimControl(
            host=host,
            port=port,
            scene_timeout_s=oracle_timeout_s,
        )
        self._selected_contract = selected_contract
        self._reset = DimSimResetCoordinator(
            self._control,
            scene_id=selected_contract.source.scene_id,
            profile_revision=selected_contract.source.profile_revision,
            reset_revision=selected_contract.source.reset_revision,
        )
        self._evaluator = DimSimNativeEvaluator()
        self._fresh_view: SceneOracleView | None = None
        self._started = False

    @property
    def fresh_oracle_view(self) -> SceneOracleView | None:
        return self._fresh_view

    def readiness(self, timeout_s: float) -> BackendReadiness:
        del timeout_s
        try:
            if not self._started:
                self._control.start()
                self._started = True
            view = self._control.oracle_view()
            if view.scene_id != self._selected_contract.source.scene_id:
                raise ValueError("attached DimSim scene does not match selected task")
        except Exception as error:
            return BackendReadiness(
                backend=self.backend_name,
                ready=False,
                detail=f"{type(error).__name__}: {error}",
            )
        return BackendReadiness(
            backend=self.backend_name,
            ready=True,
            detail="attached DimSim control, odometry, and private oracle are ready",
        )

    def reset(self, request: BackendResetRequest, timeout_s: float) -> ResetReceipt:
        self._require_selected_request(
            request.attempt_id,
            request.task_id,
            request.source_revisions,
        )
        acknowledgement = self._reset.reset(
            DimSimResetRequest(
                attempt_id=request.attempt_id,
                operation_id=request.operation_id,
                task_id=request.task_id,
                episode_id=request.episode.episode_id,
                scene_id=self._selected_contract.source.scene_id,
                profile_revision=self._selected_contract.source.profile_revision,
                reset_revision=self._selected_contract.source.reset_revision,
                requested_start_pose=request.start_pose,
                expected_previous_generation=self._reset.generation,
            ),
            timeout_s,
        )
        self._fresh_view = self._control.oracle_view()
        contract = self._selected_contract.contract
        if not isinstance(contract, NavigateContract):
            raise ValueError("DimSim navigation backend requires NavigateContract")
        target = next(
            (
                entity
                for entity in self._fresh_view.entities
                if entity.entity_id == contract.target_entity_id
            ),
            None,
        )
        if target is None:
            raise ValueError("navigation target is absent from fresh oracle view")
        reset_sample = AuthoritativeBodySample(
            pose=acknowledgement.applied_start_pose,
            linear_speed_m_s=acknowledgement.linear_speed_m_s,
            angular_speed_rad_s=acknowledgement.angular_speed_rad_s,
            simulated_time_s=0.0,
            pose_timestamp_s=acknowledgement.pose_timestamp_s,
        )
        initially_satisfied = NavigationRubric(
            contract,
            target.footprint,
            self._fresh_view.embodiment.footprint_radius_m,
        ).instantaneous_condition(reset_sample)
        return ResetReceipt(
            attempt_id=acknowledgement.attempt_id,
            operation_id=acknowledgement.operation_id,
            task_id=acknowledgement.task_id,
            episode=request.episode,
            requested_pose=acknowledgement.requested_start_pose,
            applied_pose=acknowledgement.applied_start_pose,
            reset_generation=acknowledgement.reset_generation,
            verified_source_revisions={
                "scene_id": self._fresh_view.scene_id,
                "scene_revision": self._fresh_view.scene_revision,
                "reset_revision": self._fresh_view.reset_revision,
                "upstream_revision": self._fresh_view.upstream_revision,
                "profile_revision": self._fresh_view.profile_revision,
            },
            source_digest=oracle_view_digest(self._fresh_view),
            initial_predicate_satisfied=initially_satisfied,
            acknowledged_at=_timestamp_to_datetime(acknowledgement.pose_timestamp_s),
        )

    def start_evaluation(
        self, request: BackendEvaluationRequest, timeout_s: float
    ) -> EvaluationHandle:
        contract_digest = _sha256_model(self._selected_contract)
        if request.contract_digest != contract_digest:
            raise ValueError("evaluation contract digest mismatch")
        expected_payload = self._selected_contract.contract.model_dump(mode="json")
        if canonical_json(request.contract_payload) != canonical_json(expected_payload):
            raise ValueError("evaluation contract payload mismatch")
        self._require_selected_request(request.attempt_id, request.task_id, None)
        if self._fresh_view is None:
            raise RuntimeError("DimSim must be authoritatively reset before evaluation")
        contract = self._selected_contract.contract
        if not isinstance(contract, NavigateContract):
            raise ValueError("DimSim navigation backend requires NavigateContract")
        target = next(
            (
                entity
                for entity in self._fresh_view.entities
                if entity.entity_id == contract.target_entity_id
            ),
            None,
        )
        if target is None:
            raise ValueError("navigation target is absent from fresh oracle view")
        initial = self._control.latest_body_sample(timeout_s)
        rubric = NavigationRubric(
            contract,
            target.footprint,
            self._fresh_view.embodiment.footprint_radius_m,
        )
        start = self._evaluator.start(
            attempt_id=request.attempt_id,
            operation_id=request.operation_id,
            task_id=request.task_id,
            episode_id=request.episode.episode_id,
            reset_generation=self._reset.generation,
            deadline_s=float(request.episode.opaque.get("deadline_s", 180.0)),
            rubric=rubric,
            initial_sample=initial,
        )
        return EvaluationHandle(
            attempt_id=request.attempt_id,
            operation_id=request.operation_id,
            task_id=request.task_id,
            episode=request.episode,
            opaque={
                "evaluation_id": start.evaluation_id,
                "reset_generation": start.reset_generation,
            },
        )

    def wait_result(self, handle: EvaluationHandle, timeout_s: float) -> DimSimNativeResult | None:
        evaluation_id = _evaluation_id(handle)
        deadline = time.monotonic() + timeout_s
        while True:
            existing = self._evaluator.result(evaluation_id)
            if existing is not None:
                return existing
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            sample = self._control.latest_body_sample(min(remaining, 1.0))
            result = self._evaluator.observe(evaluation_id, sample)
            if result is not None:
                return result

    def cancel(self, handle: EvaluationHandle, timeout_s: float) -> None:
        del timeout_s
        self._evaluator.cancel(_evaluation_id(handle))
        self._control.clear_motion()

    def cleanup(self) -> None:
        if self._started:
            self._control.stop()
            self._started = False

    def _require_selected_request(
        self,
        attempt_id: str,
        task_id: str,
        source_revisions: dict[str, str] | None,
    ) -> None:
        if not attempt_id:
            raise ValueError("attempt identity is required")
        if task_id != self._selected_contract.task_id:
            raise ValueError("backend request task identity mismatch")
        if source_revisions is not None:
            expected = {
                "scene_id": self._selected_contract.source.scene_id,
                "profile_revision": self._selected_contract.source.profile_revision,
                "reset_revision": self._selected_contract.source.reset_revision,
                "upstream_revision": self._selected_contract.source.upstream_revision,
            }
            if source_revisions != expected:
                raise ValueError("backend request source revisions mismatch")


def _evaluation_id(handle: EvaluationHandle) -> str:
    value = handle.opaque.get("evaluation_id")
    if not isinstance(value, str):
        raise ValueError("evaluation handle lacks native identity")
    return value


def _sha256_model(contract: TaskContract) -> str:
    return hashlib.sha256(model_bytes(contract)).hexdigest()


def _timestamp_to_datetime(timestamp_s: float) -> datetime:
    return datetime.fromtimestamp(timestamp_s, UTC)
