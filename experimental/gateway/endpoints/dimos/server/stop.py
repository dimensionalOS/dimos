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
import os
from typing import Any

from fastapi import FastAPI

from experimental.gateway.server.openapi import route_doc
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils import models


def register(app: FastAPI, state: ServerState) -> None:
    s = state

    @app.post(
        "/dimos/server/stop",
        response_model=models.Stopping,
        **route_doc(
            "server",
            "Make the dimos gateway exit (Desktop starts it again when needed)",
            "Answers, then exits 0.2 s later. A running upload's worker is killed and the upload is queued again for the next start (dimos resumes it); launched blueprints keep running (their own sessions).",
            answer="`{ stopping: true }`",
        ),
    )
    async def stop_server() -> dict[str, Any]:
        s.uploads.shutdown()
        loop = asyncio.get_running_loop()
        loop.call_later(0.2, s.exit or (lambda: os._exit(0)))
        return {"stopping": True}
