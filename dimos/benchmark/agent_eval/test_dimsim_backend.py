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

import hashlib

import pytest

from dimos.benchmark.agent_eval.backend import (
    BackendEvaluationRequest,
    BackendResetRequest,
)
from dimos.benchmark.agent_eval.dimsim_backend import DimSimEvaluationBackend
from dimos.benchmark.agent_eval.models import BackendEpisodeReference
from dimos.benchmark.dimsim.fixture import apartment_oracle_fixture
from dimos.benchmark.dimsim.generation import generate_destination
from dimos.benchmark.dimsim.models import Pose2, SceneOracleView
from dimos.benchmark.dimsim.utilities import model_bytes
from dimos.simulation.dimsim.evaluation import AuthoritativeBodySample

_ATTEMPT_ID = "attempt_" + "1" * 32
_RESET_OPERATION_ID = "operation_" + "2" * 32
_EVAL_OPERATION_ID = "operation_" + "3" * 32
_TEST_SPAWN = Pose2(x_m=8.0, z_m=8.0, yaw_rad=0.0)


class _FakeControl:
    def __init__(
        self,
        view: SceneOracleView,
        samples: list[AuthoritativeBodySample],
        *,
        readiness_error: BaseException | None = None,
    ) -> None:
        self.view = view
        self.samples = samples
        self.readiness_error = readiness_error
        self.started = False
        self.stopped = False
        self.motion_clears = 0
        self.teleports: list[Pose2] = []

    def start(self) -> None:
        self.started = True

    def stop(self) -> None:
        self.stopped = True

    def clear_motion(self) -> None:
        self.motion_clears += 1

    def settle_motion(self) -> None:
        pass

    def teleport(self, pose: Pose2) -> None:
        self.teleports.append(pose)

    def wait_body_sample(self, timeout_s: float) -> AuthoritativeBodySample:
        del timeout_s
        return self.samples.pop(0)

    def latest_body_sample(self, timeout_s: float) -> AuthoritativeBodySample:
        del timeout_s
        return self.samples.pop(0)

    def oracle_view(self) -> SceneOracleView:
        if self.readiness_error is not None:
            raise self.readiness_error
        return self.view


def _sample(
    pose: Pose2,
    time_s: float,
    *,
    linear: float = 0.0,
) -> AuthoritativeBodySample:
    return AuthoritativeBodySample(
        pose=pose,
        linear_speed_m_s=linear,
        angular_speed_rad_s=0.0,
        simulated_time_s=time_s,
        pose_timestamp_s=1_800_000_000.0 + time_s,
    )


def _episode() -> BackendEpisodeReference:
    return BackendEpisodeReference(
        backend="dimsim-attached-v1",
        episode_id="episode-1",
        opaque={"deadline_s": 10.0},
    )


def _view() -> SceneOracleView:
    view = apartment_oracle_fixture()
    return view.model_copy(
        update={"embodiment": view.embodiment.model_copy(update={"canonical_spawn": _TEST_SPAWN})}
    )


def _source_revisions(compiled) -> dict[str, str]:
    source = compiled.contract.source
    return {
        "scene_id": source.scene_id,
        "profile_revision": source.profile_revision,
        "reset_revision": source.reset_revision,
        "upstream_revision": source.upstream_revision,
    }


def _reset_request(compiled) -> BackendResetRequest:
    return BackendResetRequest(
        attempt_id=_ATTEMPT_ID,
        operation_id=_RESET_OPERATION_ID,
        task_id=compiled.public.task_id,
        episode=_episode(),
        start_pose=_TEST_SPAWN,
        source_revisions=_source_revisions(compiled),
    )


def _evaluation_request(compiled) -> BackendEvaluationRequest:
    return BackendEvaluationRequest(
        attempt_id=_ATTEMPT_ID,
        operation_id=_EVAL_OPERATION_ID,
        task_id=compiled.public.task_id,
        episode=_episode(),
        contract_digest=hashlib.sha256(model_bytes(compiled.contract)).hexdigest(),
        contract_payload=compiled.contract.contract.model_dump(mode="json"),
    )


def test_dimsim_backend_runs_reset_and_returns_raw_native_success() -> None:
    view = _view()
    compiled = generate_destination(view)
    spawn = view.embodiment.canonical_spawn
    beside_bathtub = Pose2(x_m=2.0, z_m=3.0, yaw_rad=0.0)
    control = _FakeControl(
        view,
        [
            _sample(spawn, 0.0),
            _sample(spawn, 0.1),
            _sample(beside_bathtub, 1.0),
            _sample(beside_bathtub, 2.0),
        ],
    )
    backend = DimSimEvaluationBackend(
        host="localhost",
        port=8090,
        selected_contract=compiled.contract,
        control=control,
    )

    assert backend.readiness(1.0).ready
    reset = backend.reset(_reset_request(compiled), 1.0)
    handle = backend.start_evaluation(_evaluation_request(compiled), 1.0)
    result = backend.wait_result(handle, 1.0)

    assert reset.reset_generation == 1
    assert result is not None
    assert result.record_type == "dimsim-native-result"
    assert result.passed
    assert result.attempt_id == _ATTEMPT_ID
    backend.cleanup()
    assert control.stopped


def test_dimsim_backend_rejects_changed_contract_digest() -> None:
    view = _view()
    compiled = generate_destination(view)
    spawn = view.embodiment.canonical_spawn
    control = _FakeControl(view, [_sample(spawn, 0.0)])
    backend = DimSimEvaluationBackend(
        host="localhost",
        port=8090,
        selected_contract=compiled.contract,
        control=control,
    )
    request = _evaluation_request(compiled).model_copy(update={"contract_digest": "0" * 64})

    with pytest.raises(ValueError, match="digest mismatch"):
        backend.start_evaluation(request, 1.0)


def test_dimsim_backend_readiness_reports_private_oracle_failure() -> None:
    view = _view()
    compiled = generate_destination(view)
    control = _FakeControl(
        view,
        [],
        readiness_error=RuntimeError("scene unavailable"),
    )
    backend = DimSimEvaluationBackend(
        host="localhost",
        port=8090,
        selected_contract=compiled.contract,
        control=control,
    )

    readiness = backend.readiness(1.0)

    assert not readiness.ready
    assert "scene unavailable" in readiness.detail


def test_dimsim_backend_cancel_is_idempotent_and_clears_motion() -> None:
    view = _view()
    compiled = generate_destination(view)
    spawn = view.embodiment.canonical_spawn
    control = _FakeControl(view, [_sample(spawn, 0.0), _sample(spawn, 0.1)])
    backend = DimSimEvaluationBackend(
        host="localhost",
        port=8090,
        selected_contract=compiled.contract,
        control=control,
    )
    backend.readiness(1.0)
    backend.reset(_reset_request(compiled), 1.0)
    handle = backend.start_evaluation(_evaluation_request(compiled), 1.0)

    backend.cancel(handle, 1.0)
    backend.cancel(handle, 1.0)

    assert control.motion_clears == 3
