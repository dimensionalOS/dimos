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
    s = state

    @app.post(
        "/dimos/cloud/logout",
        response_model=models.Account,
        **route_doc(
            "cloud",
            "Log this machine out of Dimensional cloud",
            "Forgets the stored cloud key (dimos's `logout`) and resets the login, then answers the account as GET /dimos/cloud/account?fresh=1 would.",
            answer="`{ loggedIn, email, scopes, source, cloudUrl, error }`, logged out",
        ),
    )
    async def cloud_logout() -> dict[str, Any]:
        return await s.uploads.logout()
