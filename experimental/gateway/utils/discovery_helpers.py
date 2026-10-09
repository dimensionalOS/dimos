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

from typing import TYPE_CHECKING, Annotated, Any

from fastapi import Path as PathParam

from experimental.gateway.server.state import ServerState
from experimental.gateway.utils.http import ApiError

if TYPE_CHECKING:
    from experimental.gateway.server.state import ServerState
EXTRAS_TTL_S = 10.0
ModuleParam = Annotated[
    str,
    PathParam(
        description="a module's registry name, class name or `module.Class`",
        examples=["go2-connection"],
    ),
]
JobParam = Annotated[str, PathParam(description="a job id", examples=["extras-1-1791000000"])]


class DiscoveryHelpers:
    def __init__(self, state: ServerState) -> None:
        self.state = state

    async def probe(self) -> dict[str, Any]:
        s = self.state
        discovery = s.discovery
        assert discovery is not None

        async def compute() -> dict[str, Any]:
            answer = await discovery.child_answer("packages")
            if "error" in answer:
                raise ApiError(500, answer["error"])
            return answer

        result: dict[str, Any] = await s.cache.get("packages", EXTRAS_TTL_S, compute)
        return result
