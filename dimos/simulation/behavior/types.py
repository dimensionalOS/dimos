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

"""Shared BEHAVIOR contracts; no simulator dependencies."""

from enum import Enum
from typing import Any, Literal

from pydantic import BaseModel, Field


class ControlMode(str, Enum):
    DIMOS = "dimos"
    NATIVE = "native"
    PRIMITIVE = "primitive"


class TaskSelection(BaseModel):
    scene: str = "house_double_floor_lower"
    activity: str = "picking_up_trash"
    definition: int = Field(default=0, ge=0)
    instance: int = Field(default=0, ge=0)


class Operation(BaseModel):
    id: str
    kind: str
    episode: str
    state: Literal["running", "succeeded", "failed", "cancelled"] = "running"
    error: str | None = None


class Episode(BaseModel):
    id: str
    task: TaskSelection | None = None
    step: int = 0
    reward: float = 0.0
    success: bool = False
    terminated: bool = False
    truncated: bool = False
    info: dict[str, Any] = Field(default_factory=dict)
    execution_kinds: list[str] = Field(default_factory=list)
    grasping_mode: str = "sticky"


class BehaviorStatus(BaseModel):
    state: Literal["loading", "running", "paused", "finished", "error", "stopped"] = "loading"
    control: ControlMode = ControlMode.DIMOS
    episode: Episode
    operation: Operation | None = None
    error: str | None = None
    achieved_hz: float = 0.0
