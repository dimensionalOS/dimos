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

from typing import Protocol

from dimos.robot_learning.policy_rollout.models import (
    BackendBatch,
    BackendOutputEnvelope,
    RobotPolicyAction,
    RobotPolicyActionChunk,
    RobotPolicyObservation,
)


class RobotPolicyContract(Protocol):
    """Semantic boundary between runtime samples, backend batches, and actions."""

    def to_backend_batch(self, sample: RobotPolicyObservation) -> BackendBatch: ...

    def from_backend_output(self, output: BackendOutputEnvelope) -> RobotPolicyAction: ...

    def chunk_from_backend_output(
        self, output: BackendOutputEnvelope
    ) -> RobotPolicyActionChunk: ...
