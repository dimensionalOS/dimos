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

"""Fixed-action backend for lightweight policy rollout tests."""

from collections.abc import Sequence

from dimos.robot_learning.policy_rollout.models import (
    BackendBatch,
    BackendOutputEnvelope,
    PolicyBackendDescription,
)


class FixedActionBackend:
    """Policy backend test double that emits a configured 7D action."""

    def __init__(self, action: Sequence[float], *, use_action_chunk: bool = False) -> None:
        if len(action) != 7:
            raise ValueError("fixed action must have exactly 7 values")
        self._action = tuple(float(value) for value in action)
        self._use_action_chunk = use_action_chunk
        self._initialized = False
        self._episode_resets = 0

    def initialize(self) -> None:
        self._initialized = True

    def reset_episode(self) -> None:
        self._episode_resets += 1

    def infer_batch(self, batch: BackendBatch) -> BackendOutputEnvelope:
        if not self._initialized:
            raise RuntimeError("FixedActionBackend was not initialized")
        output: tuple[float, ...] | tuple[tuple[float, ...], ...] = self._action
        if self._use_action_chunk:
            output = (self._action,)
        return BackendOutputEnvelope(
            output=output,
            metadata={
                "backend_type": "fixed_action",
                "batch_metadata": dict(batch.metadata),
                "episode_resets": self._episode_resets,
                "use_action_chunk": self._use_action_chunk,
            },
        )

    def close(self) -> None:
        self._initialized = False

    def describe(self) -> PolicyBackendDescription:
        return PolicyBackendDescription(
            backend_type="fixed_action",
            checkpoint_id=None,
            supports_episode_reset=True,
            metadata={"action": list(self._action), "episode_resets": self._episode_resets},
        )
