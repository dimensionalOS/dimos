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

from typing import Any

from fastapi import FastAPI

from experimental.gateway.server.openapi import route_doc
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils import models


def register(app: FastAPI, state: ServerState) -> None:
    discovery = state.discovery
    assert discovery is not None

    @app.post(
        "/dimos/discovery/refresh",
        response_model=models.DiscoveryStatus,
        **route_doc(
            "discovery",
            "Check the checkout and packages again now, and rescan what changed (or everything, `full`)",
            "Wakes the discovery loop: it recomputes the cache key and scans what's missing; with `full` it forgets the answer and imports everything again. Answers the status at once; `discovery` events follow the scan. The body is optional.",
            answer="`DiscoveryStatus`",
        ),
    )
    async def discovery_refresh(request: models.DiscoveryRefresh | None = None) -> dict[str, Any]:
        discovery.refresh("requested", bool(request and request.full))
        return dict(discovery.status)
