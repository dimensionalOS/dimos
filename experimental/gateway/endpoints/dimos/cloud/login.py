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

    @app.get(
        "/dimos/cloud/login",
        response_model=models.Login,
        **route_doc(
            "cloud",
            "The cloud login in progress: state (idle, starting, pending, approved, denied, expired, failed), url, code",
            "The device login's state; a pending one past its expiry turns expired. No other side effects.",
            agent=True,
            answer="`Login`: `{ state, url, urlComplete, code, expiresAt, email, error }`",
        ),
    )
    async def cloud_login() -> dict[str, Any]:
        return s.uploads.login_state()

    @app.post(
        "/dimos/cloud/login",
        response_model=models.Login,
        **route_doc(
            "cloud",
            "Start logging this machine in to Dimensional cloud: returns a URL and a code the user approves in any signed-in browser",
            "Starts dimos's device login in a child process (or returns the one already waiting) and answers once the code is known (pending), within 30 s. Show the URL and code; `cloud-login` events (or GET /dimos/cloud/login) follow it. Once approved, dimos stores the key and a waiting upload queue goes on.",
            agent=True,
            answer="`Login`",
        ),
    )
    async def start_cloud_login() -> dict[str, Any]:
        return await s.uploads.start_login()

    @app.delete(
        "/dimos/cloud/login",
        response_model=models.Login,
        **route_doc(
            "cloud",
            "Cancel the pending cloud login",
            "Kills the login's child process; a starting or pending login goes back to idle. Answers the login state.",
            answer="`Login`",
        ),
    )
    async def cancel_cloud_login() -> dict[str, Any]:
        return s.uploads.cancel_login()
