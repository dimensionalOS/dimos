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
from experimental.gateway.utils import config, models


def register(app: FastAPI, state: ServerState) -> None:
    s = state

    @app.get(
        "/dimos/info",
        response_model=models.Info,
        **route_doc(
            "server",
            "The dimos checkout this gateway drives: dir, version, installed",
            "Reads the checkout's pyproject.toml and whether its `.venv/bin/dimos` exists, and checks the version against the range Desktop supports ($DESKTOP_DIMOS_RANGE). A launch is refused while `inRange` is false. No side effects.",
            agent=True,
            answer="`{ dir, found, installed, version, range, inRange }`",
        ),
    )
    async def info() -> dict[str, Any]:
        return config.info(s.dimos_dir).to_json()
