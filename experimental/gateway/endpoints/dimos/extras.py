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

import asyncio
from typing import Any

from fastapi import FastAPI

from experimental.gateway.server.openapi import route_doc
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils import extras, models
from experimental.gateway.utils.discovery_helpers import DiscoveryHelpers


def register(app: FastAPI, state: ServerState) -> None:
    s = state
    helpers = DiscoveryHelpers(state)

    @app.get(
        "/dimos/extras",
        response_model=models.ExtrasList,
        **route_doc(
            "extras",
            "dimos's optional extras (`dimos[sim]`, ...): which are installed, what's missing, a download-size hint",
            "Extras from the checkout's pyproject.toml (else the installed dimos's metadata); installed packages are asked of the checkout's python in a child process (cached 10 s). `download_bytes` is an upper-bound hint from uv.lock. No side effects.",
            agent=True,
            answer="`{ mode, python, extras: [{ name, installed, applicable, requires, includes, missing, download_bytes }] }`",
        ),
    )
    async def extras_list() -> dict[str, Any]:
        found = await helpers.probe()
        listed = await asyncio.to_thread(extras.status, s.dimos_dir, found)
        return {
            "mode": "checkout" if extras.is_checkout(s.dimos_dir) else "library",
            "python": found["python"],
            "extras": listed,
        }
