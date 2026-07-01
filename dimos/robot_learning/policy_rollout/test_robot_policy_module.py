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

from dimos.robot_learning.policy_rollout.models import (
    BackendBatch,
    BackendOutputEnvelope,
    PolicyBackendDescription,
    RobotPolicyAction,
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
    batches: list[BackendBatch] = field(default_factory=list)

    def initialize(self) -> None:
        self.initialized = True

    def reset_episode(self) -> None:
        self.reset_count += 1

    def infer_batch(self, batch: BackendBatch) -> BackendOutputEnvelope:
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
        return RobotPolicyAction(space_id="fake.action.v1", values=output.output)


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
