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

from dataclasses import dataclass, field
import threading
from typing import cast

from dimos.robot_learning.policy_rollout.models import (
    BackendBatch,
    BackendOutputEnvelope,
    PolicyBackendDescription,
    RobotPolicyAction,
    RobotPolicyActionChunk,
    RobotPolicyObservation,
)
from dimos.robot_learning.policy_rollout.robot_policy_module import (
    RobotPolicyModule,
    RobotPolicyModuleConfig,
)


@dataclass
class FakeBackend:
    initialized: bool = False
    reset_count: int = 0
    closed: bool = False
    block_infer: threading.Event | None = None
    batches: list[BackendBatch] = field(default_factory=list)

    def initialize(self) -> None:
        self.initialized = True

    def reset_episode(self) -> None:
        self.reset_count += 1

    def infer_batch(self, batch: BackendBatch) -> BackendOutputEnvelope:
        if self.block_infer is not None:
            self.block_infer.wait(timeout=2.0)
        self.batches.append(batch)
        return BackendOutputEnvelope(output=(0.1, 0.2, 0.3), metadata={"backend": "fake"})

    def close(self) -> None:
        self.closed = True

    def describe(self) -> PolicyBackendDescription:
        return PolicyBackendDescription(backend_type="fake", checkpoint_id="fake-checkpoint")


@dataclass
class FakeContract:
    samples: list[RobotPolicyObservation] = field(default_factory=list)
    outputs: list[BackendOutputEnvelope] = field(default_factory=list)

    def to_backend_batch(self, sample: RobotPolicyObservation) -> BackendBatch:
        self.samples.append(sample)
        return BackendBatch(payload={"observations": dict(sample.observations)})

    def from_backend_output(self, output: BackendOutputEnvelope) -> RobotPolicyAction:
        self.outputs.append(output)
        assert isinstance(output.output, tuple)
        assert not isinstance(output.output[0], tuple)
        values = cast("tuple[float, ...]", output.output)
        return RobotPolicyAction(space_id="fake.action.v1", values=values)

    def chunk_from_backend_output(self, output: BackendOutputEnvelope) -> RobotPolicyActionChunk:
        self.outputs.append(output)
        assert isinstance(output.output, tuple)
        if output.output and isinstance(output.output[0], tuple):
            values = cast("tuple[tuple[float, ...], ...]", output.output)
        else:
            values = (cast("tuple[float, ...]", output.output),)
        return RobotPolicyActionChunk(space_id="fake.action.v1", values=values)


def test_infer_action_uses_contract_backend_and_emits_action() -> None:
    backend = FakeBackend()
    contract = FakeContract()
    module = RobotPolicyModule(backend=backend, contract=contract)
    sample = RobotPolicyObservation(observations={"id": "obs-1"})

    try:
        action = module.infer_action(sample)

        assert backend.initialized
        assert contract.samples == [sample]
        assert backend.batches == [BackendBatch(payload={"observations": {"id": "obs-1"}})]
        assert len(contract.outputs) == 1
        assert action == RobotPolicyAction(space_id="fake.action.v1", values=(0.1, 0.2, 0.3))
        assert module.last_action == action
    finally:
        module.close()


def test_public_reset_resets_backend_episode_state_and_clears_last_action() -> None:
    backend = FakeBackend()
    contract = FakeContract()
    module = RobotPolicyModule(backend=backend, contract=contract)
    try:
        module.infer_action(RobotPolicyObservation(observations={"id": "obs-1"}))

        module.reset_episode(episode_id="episode-2")

        assert backend.initialized
        assert backend.reset_count == 1
        assert module.last_action is None
    finally:
        module.close()


def test_backend_description_and_close_delegate_to_seams() -> None:
    backend = FakeBackend()
    contract = FakeContract()
    module = RobotPolicyModule(backend=backend, contract=contract)
    try:
        assert module.describe_backend().backend_type == "fake"

        module.close()

        assert backend.closed
    finally:
        module.close()


def test_module_config_defaults_are_registry_names() -> None:
    module = RobotPolicyModuleConfig()

    assert module.backend_type == "lerobot"
    assert module.contract_type == "vla_jepa_libero"


def test_update_observation_stores_latest_live_policy_observation() -> None:
    module = RobotPolicyModule(backend=FakeBackend(), contract=FakeContract())
    observation = RobotPolicyObservation(observations={"id": "obs-1"})

    try:
        module.update_observation(observation)

        assert module.latest_observation == observation
    finally:
        module.close()


def test_infer_action_chunk_publishes_chunk_and_preserves_sync_action() -> None:
    backend = FakeBackend()
    contract = FakeContract()
    module = RobotPolicyModule(backend=backend, contract=contract)
    published: list[RobotPolicyActionChunk] = []
    module.policy_action_chunk.subscribe(published.append)

    try:
        chunk = module.infer_action_chunk(RobotPolicyObservation(observations={"id": "obs-1"}))
        action = module.infer_action(RobotPolicyObservation(observations={"id": "obs-2"}))

        assert chunk.shape == (1, 3)
        assert published == [chunk]
        assert module.last_action_chunk == chunk
        assert action.values == (0.1, 0.2, 0.3)
        assert module.last_action == action
    finally:
        module.close()


def test_trigger_action_chunk_inference_requires_live_observation() -> None:
    module = RobotPolicyModule(backend=FakeBackend(), contract=FakeContract())

    try:
        status = module.trigger_action_chunk_inference()

        assert not status.accepted
        assert status.status == "not_ready"
        assert status.message is not None
    finally:
        module.close()


def test_trigger_action_chunk_inference_returns_before_chunk_publication() -> None:
    release_infer = threading.Event()
    backend = FakeBackend(block_infer=release_infer)
    module = RobotPolicyModule(backend=backend, contract=FakeContract())
    published_event = threading.Event()
    published: list[RobotPolicyActionChunk] = []

    def on_chunk(chunk: RobotPolicyActionChunk) -> None:
        published.append(chunk)
        published_event.set()

    module.policy_action_chunk.subscribe(on_chunk)

    try:
        module.update_observation(RobotPolicyObservation(observations={"id": "obs-1"}))

        status = module.trigger_action_chunk_inference()
        duplicate = module.trigger_action_chunk_inference()

        assert status.accepted
        assert status.status == "started"
        assert not duplicate.accepted
        assert duplicate.status == "already_in_flight"
        assert published == []

        release_infer.set()
        assert published_event.wait(timeout=2.0)
        assert published[0].sequence == status.sequence
        assert module.last_action_chunk == published[0]
    finally:
        release_infer.set()
        module.close()


def test_trigger_action_chunk_inference_allows_next_request_after_completion() -> None:
    backend = FakeBackend()
    module = RobotPolicyModule(backend=backend, contract=FakeContract())
    published_event = threading.Event()
    module.policy_action_chunk.subscribe(lambda _chunk: published_event.set())

    try:
        module.update_observation(RobotPolicyObservation(observations={"id": "obs-1"}))
        first = module.trigger_action_chunk_inference()
        assert first.accepted
        assert published_event.wait(timeout=2.0)

        second = module.trigger_action_chunk_inference()

        assert second.accepted
        assert second.sequence == 1
    finally:
        module.close()
