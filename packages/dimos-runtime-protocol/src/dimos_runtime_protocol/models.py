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

"""Pydantic models for the DimOS runtime sidecar protocol."""

from __future__ import annotations

from enum import Enum


class StrEnum(str, Enum):
    """Python 3.10-compatible subset of enum.StrEnum."""

    def __str__(self) -> str:
        return self.value


from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, NonNegativeFloat, NonNegativeInt, PositiveInt

from dimos_runtime_protocol.types import JsonObject, JsonValue
from dimos_runtime_protocol.version import PROTOCOL_VERSION


class StrictModel(BaseModel):
    """Base model that rejects drift between client and sidecar schemas."""

    model_config = ConfigDict(extra="forbid")


class ProtocolVersion(StrictModel):
    """Protocol version advertised by a client or runtime sidecar."""

    version: str = PROTOCOL_VERSION
    min_compatible: str = PROTOCOL_VERSION


class CommandMode(StrEnum):
    """Supported motor command modes."""

    POSITION = "position"
    VELOCITY = "velocity"
    TORQUE = "torque"
    PD_TAU = "pd_tau"


class ObservationKind(StrEnum):
    """Kinds of observations that can cross the runtime protocol."""

    IMAGE = "image"
    DEPTH = "depth"
    SEGMENTATION = "segmentation"
    STATE = "state"
    TEXT = "text"


class HealthResponse(StrictModel):
    """Health check response for sidecar readiness."""

    ok: bool
    runtime_id: str
    protocol: ProtocolVersion = Field(default_factory=ProtocolVersion)
    detail: str = ""


class MotorDescription(StrictModel):
    """A single ordered motor exposed by a robot runtime."""

    name: str
    index: NonNegativeInt
    units: Literal["rad", "m"] = "rad"
    lower: float | None = None
    upper: float | None = None


class RobotMotorSurface(StrictModel):
    """Ordered whole-body motor surface for one robot."""

    robot_id: str
    surface_type: Literal["whole_body"] = "whole_body"
    motors: list[MotorDescription]
    supported_command_modes: list[CommandMode] = Field(
        default_factory=lambda: [CommandMode.POSITION]
    )
    state_fields: list[Literal["q", "dq", "tau"]] = Field(
        default_factory=lambda: ["q", "dq", "tau"]
    )


class RuntimeDescription(StrictModel):
    """Runtime metadata returned by a sidecar before DimOS launch."""

    runtime_id: str
    backend: str
    protocol: ProtocolVersion = Field(default_factory=ProtocolVersion)
    capabilities: list[str] = Field(default_factory=list)
    robot_surfaces: list[RobotMotorSurface]
    control_step_hz: PositiveInt
    observation_streams: list[str] = Field(default_factory=list)
    metadata: JsonObject = Field(default_factory=dict)


class EpisodeResetRequest(StrictModel):
    """Request to reset a backend episode."""

    episode_id: str
    task_id: str
    seed: int | None = None
    options: JsonObject = Field(default_factory=dict)


class EpisodeResetResponse(StrictModel):
    """Response after sidecar episode reset."""

    episode_id: str
    runtime_description: RuntimeDescription
    observations: list[ObservationFrame] = Field(default_factory=list)


class MotorActionFrame(StrictModel):
    """Ordered motor command frame sent to a sidecar."""

    robot_id: str
    mode: CommandMode = CommandMode.POSITION
    names: list[str]
    q: list[float]
    dq: list[float] = Field(default_factory=list)
    kp: list[float] = Field(default_factory=list)
    kd: list[float] = Field(default_factory=list)
    tau: list[float] = Field(default_factory=list)
    sequence: NonNegativeInt = 0


class MotorStateFrame(StrictModel):
    """Ordered motor state frame returned by a sidecar."""

    robot_id: str
    names: list[str]
    q: list[float]
    dq: list[float]
    tau: list[float]
    sequence: NonNegativeInt = 0
    timestamp_s: NonNegativeFloat = 0.0


class ObservationFrame(StrictModel):
    """Backend-neutral observation metadata or small payload."""

    stream: str
    kind: ObservationKind
    encoding: str = ""
    shape: list[int] = Field(default_factory=list)
    dtype: str = ""
    data_ref: str | None = None
    inline_text: str | None = None
    metadata: JsonObject = Field(default_factory=dict)


class StepRequest(StrictModel):
    """One synchronous runtime step request."""

    episode_id: str
    tick_id: NonNegativeInt
    action: MotorActionFrame


class StepResponse(StrictModel):
    """One synchronous runtime step response."""

    episode_id: str
    tick_id: NonNegativeInt
    motor_state: MotorStateFrame
    observations: list[ObservationFrame] = Field(default_factory=list)
    reward: float = 0.0
    done: bool = False
    success: bool | None = None
    info: JsonObject = Field(default_factory=dict)


class ScoreOutput(StrictModel):
    """Normalized score metadata for an episode."""

    episode_id: str
    success: bool
    score: float
    reason: str = ""
    metrics: JsonObject = Field(default_factory=dict)


class ArtifactOutput(StrictModel):
    """Sidecar-produced artifact metadata."""

    episode_id: str
    artifacts: dict[str, str] = Field(default_factory=dict)
    logs: list[str] = Field(default_factory=list)


class ErrorResponse(StrictModel):
    """Protocol-level error payload."""

    code: str
    message: str
    detail: JsonObject = Field(default_factory=dict)


_TYPES_NAMESPACE = {
    "JsonObject": JsonObject,
    "JsonValue": JsonValue,
    "ObservationFrame": ObservationFrame,
}

for _model in (
    RuntimeDescription,
    EpisodeResetRequest,
    EpisodeResetResponse,
    ObservationFrame,
    StepResponse,
    ScoreOutput,
    ErrorResponse,
):
    _model.model_rebuild(_types_namespace=_TYPES_NAMESPACE)
