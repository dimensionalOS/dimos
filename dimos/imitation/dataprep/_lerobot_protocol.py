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

"""Private typed wire shared by the host and isolated LeRobot process."""

from pathlib import Path
from typing import Annotated, Any, Literal, TypeAlias

from pydantic import BaseModel, Field, TypeAdapter

from dimos.imitation.dataprep.core import DataPrepConfig


class BuildRequest(BaseModel):
    command: Literal["build"] = "build"
    config: DataPrepConfig


class InspectRequest(BaseModel):
    command: Literal["inspect"] = "inspect"
    path: Path


Request: TypeAlias = Annotated[BuildRequest | InspectRequest, Field(discriminator="command")]
REQUEST_ADAPTER: TypeAdapter[Request] = TypeAdapter(Request)


class BuildResult(BaseModel):
    command: Literal["build"] = "build"
    path: Path


class InspectResult(BaseModel):
    command: Literal["inspect"] = "inspect"
    info: dict[str, Any]


Result: TypeAlias = Annotated[BuildResult | InspectResult, Field(discriminator="command")]
RESULT_ADAPTER: TypeAdapter[Result] = TypeAdapter(Result)
