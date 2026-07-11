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

from collections.abc import Callable
from dataclasses import dataclass, field
import threading
from typing import Any, Protocol, cast

from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.robot_learning.policy_rollout.backends.backend import PolicyBackend
from dimos.robot_learning.policy_rollout.contract import RobotPolicyContract
from dimos.robot_learning.policy_rollout.models import (
    PolicyBackendDescription,
    RobotPolicyAction,
    RobotPolicyActionChunk,
    RobotPolicyObservation,
)
from dimos.robot_learning.policy_rollout.registry import (
    policy_backend_registry,
    robot_policy_contract_registry,
)
from dimos.spec.utils import Spec


class RobotPolicyModuleConfig(ModuleConfig):
    """Config for registry-backed robot policy inference modules."""

    backend_type: str = "lerobot"
    backend_params: dict[str, object] = Field(default_factory=dict)
    contract_type: str = "vla_jepa_libero"
    contract_params: dict[str, object] = Field(default_factory=dict)
    initialize_on_start: bool = False


@dataclass(frozen=True)
class PolicyChunkInferenceStatus:
    """Status for live policy chunk inference triggers."""

    accepted: bool
    status: str
    sequence: int | None = None
    message: str | None = None
    metadata: dict[str, object] = field(default_factory=dict)


class RobotPolicyChunkInferenceSpec(Spec, Protocol):
    """RPC surface used by control tasks to request live policy chunk refill."""

    def trigger_action_chunk_inference(self) -> PolicyChunkInferenceStatus: ...


class RobotPolicyModule(Module):
    """Policy inference seam for robot-learning rollouts.

    The module owns backend lifecycle, episode reset, contract conversion,
    backend inference, and action emission. Benchmark/runtime lifecycle,
    sidecar reset/step, scoring, success gates, and artifact writing remain
    outside this class.
    """

    config: RobotPolicyModuleConfig
    policy_observation: In[RobotPolicyObservation]
    policy_action_chunk: Out[RobotPolicyActionChunk]

    def __init__(
        self,
        backend: PolicyBackend | None = None,
        contract: RobotPolicyContract | None = None,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        self._backend: PolicyBackend = backend or cast(
            "PolicyBackend",
            policy_backend_registry.create(self.config.backend_type, **self.config.backend_params),
        )
        self._contract: RobotPolicyContract = contract or cast(
            "RobotPolicyContract",
            robot_policy_contract_registry.create(
                self.config.contract_type, **self.config.contract_params
            ),
        )
        self._initialized = False
        self._last_action: RobotPolicyAction | None = None
        self._last_action_chunk: RobotPolicyActionChunk | None = None
        self._latest_observation: RobotPolicyObservation | None = None
        self._backend_lock = threading.RLock()
        self._live_lock = threading.Lock()
        self._inference_in_flight = False
        self._next_chunk_sequence = 0
        self._observation_unsubscribe: Callable[[], None] | None = None

    @property
    def last_action(self) -> RobotPolicyAction | None:
        return self._last_action

    @property
    def last_action_chunk(self) -> RobotPolicyActionChunk | None:
        return self._last_action_chunk

    @property
    def latest_observation(self) -> RobotPolicyObservation | None:
        return self._latest_observation

    @rpc
    def start(self) -> None:
        super().start()
        self._observation_unsubscribe = self.policy_observation.subscribe(self.update_observation)
        if self.config.initialize_on_start:
            self.initialize()

    @rpc
    def stop(self) -> None:
        if self._observation_unsubscribe is not None:
            self._observation_unsubscribe()
            self._observation_unsubscribe = None
        self.close()
        super().stop()

    def initialize(self) -> None:
        with self._backend_lock:
            if self._initialized:
                return
            self._backend.initialize()
            self._initialized = True

    @rpc
    def reset_episode(self, episode_id: str | None = None) -> None:
        del episode_id
        with self._backend_lock:
            self.initialize()
            self._backend.reset_episode()
        self._last_action = None
        self._last_action_chunk = None

    @rpc
    def reset(self, episode_id: str | None = None) -> None:
        self.reset_episode(episode_id=episode_id)

    @rpc
    def infer_action(self, sample: RobotPolicyObservation) -> RobotPolicyAction:
        with self._backend_lock:
            self.initialize()
            batch = self._contract.to_backend_batch(sample)
            backend_output = self._backend.infer_batch(batch)
            action = self._contract.from_backend_output(backend_output)
        return self.emit_action(action)

    @rpc
    def infer_action_chunk(self, sample: RobotPolicyObservation) -> RobotPolicyActionChunk:
        chunk = self._infer_action_chunk_from_observation(sample)
        return self.emit_action_chunk(chunk)

    @rpc
    def update_observation(self, observation: RobotPolicyObservation) -> None:
        with self._live_lock:
            self._latest_observation = observation

    @rpc
    def trigger_action_chunk_inference(self) -> PolicyChunkInferenceStatus:
        with self._live_lock:
            observation = self._latest_observation
            if observation is None:
                return PolicyChunkInferenceStatus(
                    accepted=False,
                    status="not_ready",
                    message="no live robot policy observation has been received",
                )
            if self._inference_in_flight:
                return PolicyChunkInferenceStatus(
                    accepted=False,
                    status="already_in_flight",
                    sequence=self._next_chunk_sequence - 1
                    if self._next_chunk_sequence > 0
                    else None,
                )
            sequence = self._next_chunk_sequence
            self._next_chunk_sequence += 1
            self._inference_in_flight = True

        thread = threading.Thread(
            target=self._run_action_chunk_inference,
            args=(observation, sequence),
            name=f"RobotPolicyModule-ChunkInference-{sequence}",
            daemon=True,
        )
        thread.start()
        return PolicyChunkInferenceStatus(
            accepted=True,
            status="started",
            sequence=sequence,
        )

    def emit_action(self, action: RobotPolicyAction) -> RobotPolicyAction:
        self._last_action = action
        return action

    def emit_action_chunk(self, chunk: RobotPolicyActionChunk) -> RobotPolicyActionChunk:
        self._last_action_chunk = chunk
        self.policy_action_chunk.publish(chunk)
        return chunk

    def _run_action_chunk_inference(
        self, observation: RobotPolicyObservation, sequence: int
    ) -> None:
        try:
            chunk = self._infer_action_chunk_from_observation(observation)
            if chunk.sequence is None:
                chunk = RobotPolicyActionChunk(
                    space_id=chunk.space_id,
                    values=chunk.values,
                    sequence=sequence,
                    timestamps=chunk.timestamps,
                    metadata=chunk.metadata,
                )
            self.emit_action_chunk(chunk)
        finally:
            with self._live_lock:
                self._inference_in_flight = False

    def _infer_action_chunk_from_observation(
        self, observation: RobotPolicyObservation
    ) -> RobotPolicyActionChunk:
        with self._backend_lock:
            self.initialize()
            batch = self._contract.to_backend_batch(observation)
            backend_output = self._backend.infer_batch(batch)
            return self._contract.chunk_from_backend_output(backend_output)

    def close(self) -> None:
        with self._backend_lock:
            self._backend.close()
            self._initialized = False
        with self._live_lock:
            self._inference_in_flight = False
        self._close_module()

    @rpc
    def describe_backend(self) -> PolicyBackendDescription:
        return self._backend.describe()
