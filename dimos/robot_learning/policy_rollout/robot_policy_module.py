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

from typing import Any, cast

from pydantic import Field

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.robot_learning.policy_rollout.backends.backend import PolicyBackend
from dimos.robot_learning.policy_rollout.contract import RobotPolicyContract
from dimos.robot_learning.policy_rollout.models import (
    PolicyBackendDescription,
    RobotPolicyAction,
    RobotPolicyObservation,
)
from dimos.robot_learning.policy_rollout.registry import (
    policy_backend_registry,
    robot_policy_contract_registry,
)


class RobotPolicyModuleConfig(ModuleConfig):
    """Config for registry-backed robot policy inference modules."""

    backend_type: str = "lerobot"
    backend_params: dict[str, object] = Field(default_factory=dict)
    contract_type: str = "vla_jepa_libero"
    contract_params: dict[str, object] = Field(default_factory=dict)
    initialize_on_start: bool = False


class RobotPolicyModule(Module):
    """Policy inference seam for robot-learning rollouts.

    The module owns backend lifecycle, episode reset, contract conversion,
    backend inference, and action emission. Benchmark/runtime lifecycle,
    sidecar reset/step, scoring, success gates, and artifact writing remain
    outside this class.
    """

    config: RobotPolicyModuleConfig

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

    @property
    def last_action(self) -> RobotPolicyAction | None:
        return self._last_action

    @rpc
    def start(self) -> None:
        super().start()
        if self.config.initialize_on_start:
            self.initialize()

    @rpc
    def stop(self) -> None:
        self.close()
        super().stop()

    def initialize(self) -> None:
        if self._initialized:
            return
        self._backend.initialize()
        self._initialized = True

    @rpc
    def reset_episode(self, episode_id: str | None = None) -> None:
        del episode_id
        self.initialize()
        self._backend.reset_episode()
        self._last_action = None

    @rpc
    def reset(self, episode_id: str | None = None) -> None:
        self.reset_episode(episode_id=episode_id)

    @rpc
    def infer_action(self, sample: RobotPolicyObservation) -> RobotPolicyAction:
        self.initialize()
        batch = self._contract.to_backend_batch(sample)
        backend_output = self._backend.infer_batch(batch)
        action = self._contract.from_backend_output(backend_output)
        return self.emit_action(action)

    def emit_action(self, action: RobotPolicyAction) -> RobotPolicyAction:
        self._last_action = action
        return action

    def close(self) -> None:
        self._backend.close()
        self._initialized = False
        self._close_module()

    @rpc
    def describe_backend(self) -> PolicyBackendDescription:
        return self._backend.describe()
