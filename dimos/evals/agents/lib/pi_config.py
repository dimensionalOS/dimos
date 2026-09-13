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

"""Typed run paths and the configuration exchanged with Pi and dimcode."""

from dataclasses import dataclass, field
import hashlib
from pathlib import Path
from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, JsonValue, TypeAdapter
from pydantic.alias_generators import to_camel

from dimos.constants import CACHE_DIR, CONFIG_DIR

Provider = Literal["openai", "anthropic"]
Thinking = Literal["off", "minimal", "low", "medium", "high", "xhigh", "max"]


@dataclass(frozen=True)
class RunPaths:
    workspace: Path
    config: Path
    cache: Path

    @classmethod
    def for_run(cls, workspace: Path) -> "RunPaths":
        identity = hashlib.sha256(str(workspace.resolve()).encode()).hexdigest()[:20]
        return cls(workspace, CONFIG_DIR / "evals" / identity, CACHE_DIR / "evals")


class RuntimeConfig(BaseModel):
    provider: Provider
    base_url: str
    key_env: str
    allowed_tools: tuple[str, ...] | None = None
    max_output_tokens: int | None = Field(default=None, ge=1)


class ToolPolicyState(BaseModel):
    tools: tuple[str, ...]
    unknown: tuple[str, ...]


class McpEndpoint(BaseModel):
    name: str
    url: str


class DimcodeConfig(BaseModel):
    workspace: Path
    python: Path
    dimos: Path | None = None
    mcp: tuple[McpEndpoint, ...] = ()


class SessionSettings(BaseModel):
    model_config = ConfigDict(alias_generator=to_camel, populate_by_name=True)

    default_provider: Provider
    default_model: str
    default_thinking_level: Thinking


@dataclass(frozen=True)
class NewSession:
    cwd: str
    type: Literal["new_session"] = field(default="new_session", init=False)


@dataclass(frozen=True)
class SelectModel:
    provider: Provider
    model_id: Annotated[str, Field(alias="modelId")]
    type: Literal["set_model"] = field(default="set_model", init=False)


@dataclass(frozen=True)
class Prompt:
    message: str
    type: Literal["prompt"] = field(default="prompt", init=False)


@dataclass(frozen=True)
class Shutdown:
    type: Literal["shutdown"] = field(default="shutdown", init=False)


GatewayCommand = Annotated[
    NewSession | SelectModel | Prompt | Shutdown, Field(discriminator="type")
]


@dataclass(frozen=True)
class GatewayRequest:
    id: str
    command: GatewayCommand


gateway_request: TypeAdapter[GatewayRequest] = TypeAdapter(GatewayRequest)


class GatewayEvent(BaseModel):
    type: Literal["event"]
    event: dict[str, JsonValue]


class GatewayResponse(BaseModel):
    type: Literal["response"]
    id: str
    error: str | None = None


GatewayPacket = Annotated[GatewayEvent | GatewayResponse, Field(discriminator="type")]
gateway_packet: TypeAdapter[GatewayPacket] = TypeAdapter(GatewayPacket)
