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
from experimental.gateway.utils import models, python_env
from experimental.gateway.utils.http import ApiError


def register(app: FastAPI, state: ServerState) -> None:
    s = state

    @app.get(
        "/dimos/python",
        response_model=models.PythonCommand,
        **route_doc(
            "server",
            "The python dimos runs with, for an agent to run scripts with dimos's python API",
            "The interpreter's absolute path (the gateway's own python when it is the checkout's venv, else the checkout's `.venv/bin/python`, else the gateway's), checked once by running `import dimos` in it from another folder, then cached. `env` is what to set for that import to find the checkout (empty when it is installed there, else `PYTHONPATH`). 500 when no python imports the checkout's dimos. No side effects.",
            agent=True,
            answer="`{ python, command, dimosDir, version, dimosVersion, env, example }`",
        ),
    )
    async def python_command() -> dict[str, Any]:
        try:
            return await asyncio.to_thread(python_env.python_command, s.dimos_dir)
        except python_env.NoPythonError as error:
            raise ApiError(500, str(error))
