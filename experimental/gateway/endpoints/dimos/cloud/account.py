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
from experimental.gateway.utils.http import FreshQuery


def register(app: FastAPI, state: ServerState) -> None:
    s = state

    @app.get(
        "/dimos/cloud/account",
        response_model=models.Account,
        **route_doc(
            "cloud",
            "Whether this machine is logged in to Dimensional cloud, and as whom",
            "Asks Dimensional cloud who the stored key (or DIMOS_API_KEY) belongs to, in a child process (90 s timeout). Cached for 20 s; `fresh` asks again. A logged-in answer restarts an upload queue that was waiting for a login.",
            errors=(400, 500),
            agent=True,
            answer="`{ loggedIn, email, scopes, source, cloudUrl, error }`",
        ),
    )
    async def cloud_account(fresh: FreshQuery = False) -> dict[str, Any]:
        return await s.uploads.account(fresh)
