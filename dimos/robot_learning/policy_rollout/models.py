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

from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import TypeAlias

JsonObject: TypeAlias = Mapping[str, object]
BackendPayload: TypeAlias = Mapping[str, object]
BackendActionOutput: TypeAlias = tuple[float, ...] | tuple[tuple[float, ...], ...]


@dataclass(frozen=True)
class RobotPolicyObservation:
    """Runtime-independent policy observation for one inference step.

    Producers such as benchmark runners, simulators, replay loaders, or future
    temporal observation assemblers populate semantically named observation roles.
    Policy contracts then convert those roles into backend-specific batches.
    """

    observations: BackendPayload
    timestamps: Mapping[str, float] = field(default_factory=dict)
    metadata: JsonObject = field(default_factory=dict)


@dataclass(frozen=True)
class BackendBatch:
    """Backend-ready policy inference batch produced by a robot contract."""

    payload: BackendPayload
    metadata: JsonObject = field(default_factory=dict)


@dataclass(frozen=True)
class BackendOutputEnvelope:
    """Backend-native policy output and inference metadata."""

    output: BackendActionOutput
    metadata: JsonObject = field(default_factory=dict)


@dataclass(frozen=True)
class PolicyBackendDescription:
    """Serializable metadata describing a loaded policy backend."""

    backend_type: str
    checkpoint_id: str | None = None
    resolved_checkpoint_metadata: JsonObject = field(default_factory=dict)
    device: str | None = None
    policy_class: str | None = None
    supports_episode_reset: bool = True
    metadata: JsonObject = field(default_factory=dict)


@dataclass(frozen=True)
class RobotPolicyAction:
    """Runtime-independent robot-learning policy action.

    Runtime adapters convert this action into benchmark runtime frames, control
    task commands, or other execution-specific command surfaces.
    """

    space_id: str
    values: tuple[float, ...]
    sequence: int | None = None
    metadata: JsonObject = field(default_factory=dict)
    kind: str = "robot_policy_action"


@dataclass(frozen=True)
class RobotPolicyActionChunk:
    """Runtime-independent ordered chunk of robot-learning policy actions."""

    space_id: str
    values: tuple[tuple[float, ...], ...]
    sequence: int | None = None
    timestamps: tuple[float, ...] | None = None
    metadata: JsonObject = field(default_factory=dict)
    kind: str = "robot_policy_action_chunk"

    def __post_init__(self) -> None:
        if not self.values:
            raise ValueError("RobotPolicyActionChunk must contain at least one action row")
        action_dim = len(self.values[0])
        if action_dim == 0:
            raise ValueError("RobotPolicyActionChunk action rows must not be empty")
        if any(len(row) != action_dim for row in self.values):
            raise ValueError("RobotPolicyActionChunk action rows must have consistent dimensions")
        if self.timestamps is not None and len(self.timestamps) != len(self.values):
            raise ValueError("RobotPolicyActionChunk timestamps must match horizon")

    @property
    def horizon(self) -> int:
        return len(self.values)

    @property
    def action_dim(self) -> int:
        return len(self.values[0])

    @property
    def shape(self) -> tuple[int, int]:
        return (self.horizon, self.action_dim)

    def first_action(self) -> RobotPolicyAction:
        return RobotPolicyAction(
            space_id=self.space_id,
            values=self.values[0],
            sequence=self.sequence,
            metadata=self.metadata,
        )


RuntimeActionOutput: TypeAlias = RobotPolicyAction | RobotPolicyActionChunk


@dataclass(frozen=True)
class RolloutEpisodeRecord:
    """Per-episode rollout record for benchmark artifacts."""

    episode_id: str
    task_id: str
    init_state_id: str
    success: bool
    steps: int
    reward_sum: float = 0.0
    done: bool = False
    failure_reason: str | None = None
    metadata: JsonObject = field(default_factory=dict)


@dataclass(frozen=True)
class RolloutSummary:
    """Aggregate rollout summary for benchmark artifacts."""

    episodes: int
    successes: int
    success_rate: float
    metadata: JsonObject = field(default_factory=dict)
