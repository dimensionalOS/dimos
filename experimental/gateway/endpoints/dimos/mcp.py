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

from fastapi import Body, FastAPI, Request
from fastapi.responses import JSONResponse, Response

from experimental.gateway.server.openapi import API_VERSION, route_doc
from experimental.gateway.server.state import ServerState
from experimental.gateway.utils.http import ApiError
from experimental.gateway.utils.skills_helpers import SkillsHelpers

MCP_PROTOCOL = "2025-06-18"
MCP_TOOLS: list[dict[str, Any]] = [
    {
        "name": "list_skills",
        "description": "The skills of the robot's running dimos blueprint (a module's @skill methods: moving, sport commands such as jump or sit, speaking, navigating, ...): name, module, description and JSON-schema params. Empty when no blueprint runs. Call this before call_skill: a skill such as Go2's execute_sport_command takes the command (FrontJump, Sit, ...) as an argument.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "query": {
                    "type": "string",
                    "description": "only skills whose name, module or description mentions this word",
                }
            },
        },
    },
    {
        "name": "call_skill",
        "description": "Call one skill of the running blueprint and wait for its answer. This acts on the robot (it may move it), as the blueprint's own agent would.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "skill": {"type": "string", "description": "the skill's name, from list_skills"},
                "args": {"type": "object", "description": "its arguments, by its params schema"},
                "module": {
                    "type": "string",
                    "description": "only when two modules have a skill of that name",
                },
            },
            "required": ["skill"],
        },
    },
]


def register(app: FastAPI, state: ServerState) -> None:
    helpers = SkillsHelpers(state)

    @app.post(
        "/dimos/mcp",
        **route_doc(
            "skills",
            "An MCP server (Streamable HTTP, JSON answers) with two tools: list_skills and call_skill",
            "For an agent: the tools stay the same whatever runs, so a session that connected before a blueprint started can call its skills (`list_skills`, then `call_skill`, as GET /dimos/skills and POST /dimos/skills/call). Takes one JSON-RPC 2.0 request (initialize, tools/list, tools/call, ping); a notification is answered 202 with no body.",
            errors=(400,),
            ok={
                "content": {
                    "application/json": {
                        "schema": {
                            "type": "object",
                            "properties": {
                                "jsonrpc": {"type": "string", "const": "2.0"},
                                "id": {"description": "the request's id"},
                                "result": {"type": "object", "description": "the method's answer"},
                                "error": {
                                    "type": "object",
                                    "description": "`{ code, message }` for an unknown method",
                                },
                            },
                        }
                    }
                }
            },
            answer="a JSON-RPC 2.0 response",
        ),
    )
    async def mcp(request: Request, body: Any = Body(default=None)) -> Response:
        if not isinstance(body, dict) or not isinstance(body.get("method"), str):
            raise ApiError(400, "a JSON-RPC 2.0 request object, with a method")
        if "id" not in body:
            return Response(status_code=202)
        (method, params, req_id) = (body["method"], body.get("params") or {}, body["id"])

        def result(value: Any) -> JSONResponse:
            return JSONResponse({"jsonrpc": "2.0", "id": req_id, "result": value})

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
            (text, failed) = await helpers.mcp_call(
                str(params.get("name")), params.get("arguments") or {}
            )
            return result({"content": [{"type": "text", "text": text}], "isError": failed})
        return JSONResponse(
            {
                "jsonrpc": "2.0",
                "id": req_id,
                "error": {"code": -32601, "message": f"Unknown: {method}"},
            }
        )
