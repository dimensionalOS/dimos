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

from fastapi import APIRouter, Body
from fastapi.responses import JSONResponse, Response
from pydantic import BaseModel, JsonValue

from experimental.gateway.skills import MCP_PROTOCOL, MCP_TOOLS, SkillError, mcp_tool
from experimental.gateway.state import API_VERSION, ApiError, GatewayState

router = APIRouter()


class SkillCallRequest(BaseModel):
    skill: str
    args: dict[str, JsonValue] = {}
    module: str | None = None
    runId: str | None = None


@router.get("/dimos/skills")
async def skill_list(state: GatewayState) -> dict[str, Any]:
    return await asyncio.to_thread(state.skills.listed)


@router.post("/dimos/skills/call")
async def skill_call(state: GatewayState, request: SkillCallRequest) -> dict[str, Any]:
    try:
        return await asyncio.to_thread(
            state.skills.call, request.skill, request.args, request.module, request.runId
        )
    except SkillError as error:
        raise ApiError(error.status, str(error))


@router.post("/dimos/mcp")
async def mcp(state: GatewayState, body: Any = Body(default=None)) -> Response:
    if not isinstance(body, dict) or not isinstance(body.get("method"), str):
        raise ApiError(400, "a JSON-RPC 2.0 request object, with a method")
    if "id" not in body:
        return Response(status_code=202)
    method, params, request_id = body["method"], body.get("params") or {}, body["id"]

    def result(value: Any) -> JSONResponse:
        return JSONResponse({"jsonrpc": "2.0", "id": request_id, "result": value})

    if method == "initialize":
        return result(
            {
                "protocolVersion": params.get("protocolVersion") or MCP_PROTOCOL,
                "capabilities": {"tools": {}},
                "serverInfo": {"name": "dimos-skills", "version": API_VERSION},
                "instructions": "The robot's skills: list_skills, then call_skill.",
            }
        )
    if method == "ping":
        return result({})
    if method == "tools/list":
        return result({"tools": MCP_TOOLS})
    if method == "tools/call":
        name, arguments = str(params.get("name")), params.get("arguments") or {}
        text, failed = await asyncio.to_thread(mcp_tool, state.skills, name, arguments)
        return result({"content": [{"type": "text", "text": text}], "isError": failed})
    error = {"code": -32601, "message": f"Unknown: {method}"}
    return JSONResponse({"jsonrpc": "2.0", "id": request_id, "error": error})
