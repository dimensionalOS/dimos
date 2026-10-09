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

    @app.get(
        "/dimos/discovery",
        response_model=models.DiscoveryStatus,
        **route_doc(
            "discovery",
            "Where the discovery scan (every blueprint and module imported once, cached) is: progress, errors",
            "The scan starts when the gateway starts. A restart with the same checkout commit, dirty files and installed packages answers from the disk cache at once (no scan); a change is noticed within 30 s and rescanned (the old answer is served meanwhile, `stale: true`). `discovery` events follow it. No side effects.",
            agent=True,
            answer="`DiscoveryStatus`: `{ state, reason, key, stale, blueprints_total, blueprints_done, importable, not_importable, modules_total, modules_done, current, errors, ... }`",
        ),
    )
    async def discovery_status() -> dict[str, Any]:
        discovery.update_counts()
        return dict(discovery.status)
