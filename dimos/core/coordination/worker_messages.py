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
from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from dimos.core.module import ModuleBase


@dataclass(frozen=True)
class DeployModuleRequest:
    module_id: int
    module_class: type[ModuleBase]
    kwargs: dict[str, Any]


@dataclass(frozen=True)
class SetRefRequest:
    module_id: int
    ref: Any


@dataclass(frozen=True)
class GetAttrRequest:
    module_id: int
    name: str


@dataclass(frozen=True)
class CallMethodRequest:
    module_id: int
    name: str
    args: tuple[Any, ...]
    kwargs: dict[str, Any]


@dataclass(frozen=True)
class UndeployModuleRequest:
    module_id: int


@dataclass(frozen=True)
class SuppressConsoleRequest:
    pass


@dataclass(frozen=True)
class ShutdownRequest:
    pass


WorkerRequest = (
    DeployModuleRequest
    | SetRefRequest
    | GetAttrRequest
    | CallMethodRequest
    | UndeployModuleRequest
    | SuppressConsoleRequest
    | ShutdownRequest
)


@dataclass(frozen=True)
class WorkerResponse:
    result: Any = None
    error: str | None = None
