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

"""The skills routes: GET /dimos/skills, POST /dimos/skills/call, and POST /dimos/mcp, an MCP server with two fixed
tools (list_skills, call_skill) so an agent that connects once, before any blueprint runs, can still call the skills of
whichever one runs later."""

from __future__ import annotations

import asyncio
import json
from typing import Any

from fastapi import Body, FastAPI, Request
from fastapi.responses import JSONResponse, Response

from dimos.gateway import models, skills
from dimos.gateway.openapi import API_VERSION, route_doc

MCP_PROTOCOL = "2025-06-18"

MCP_TOOLS: list[dict[str, Any]] = [
    {
        "name": "list_skills",
        "description": "The skills of the robot's running dimos blueprint (a module's @skill methods: moving, sport "
        "commands such as jump or sit, speaking, navigating, ...): name, module, description and JSON-schema "
        "params. Empty when no blueprint runs; only agentic blueprints (with an MCP server) have skills. Call this "
        "before call_skill: a skill such as Go2's execute_sport_command takes the command (FrontJump, Sit, ...) as "
        "an argument.",
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
        "description": "Call one skill of the running blueprint and wait for its answer. This acts on the robot (it "
        "may move it), as the blueprint's own agent would.",
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


def _matches(skill: dict[str, Any], query: str) -> bool:
    text = f"{skill['name']} {skill['module'] or ''} {skill['description']}".lower()
    return all(word in text for word in query.lower().split())


def add(app: FastAPI) -> None:
    from dimos.gateway.app import ApiError

    async def listed() -> dict[str, Any]:
        servers = await asyncio.to_thread(skills.live_servers)
        return await asyncio.to_thread(skills.list_skills, servers)

    async def called(
        skill: str, args: dict[str, Any], module: str | None, run_id: str | None
    ) -> dict[str, Any]:
        servers = await asyncio.to_thread(skills.live_servers)
        return await asyncio.to_thread(skills.call_skill, servers, skill, args, module, run_id)

    @app.get(
        "/dimos/skills",
        response_model=models.SkillList,
        **route_doc(
            "skills",
            "The running blueprints' skills (what their agent can call): name, module, description, params",
            "Asks each live run's MCP server (an agentic blueprint's McpServer, at its `mcp_port`, default 9990) for "
            "its tools and which module has each. Empty `skills` when nothing runs; a run without an MCP server is in "
            "`runs` with `up: false`. No side effects.",
            agent=True,
            answer="`{ skills: [{ name, module, description, params, required, lifecycle, uses, runId, blueprint }], "
            "runs: [{ runId, blueprint, mcpUrl, up, error }] }`",
        ),
    )
    async def skill_list() -> dict[str, Any]:
        return await listed()

    @app.post(
        "/dimos/skills/call",
        response_model=models.SkillCallResult,
        **route_doc(
            "skills",
            "Call a skill of the running blueprint and wait for its answer",
            "Calls it through the run's MCP server (`tools/call`, as the blueprint's own agent does: a skill whose "
            "capability another holds waits or is refused, a background skill answers at once). This acts on the "
            "robot. Waits up to 300 s. `ok` false when the skill itself failed. 404 when no running blueprint has "
            "the skill (or `runId` isn't live), 409 when nothing runs, 500 when the call failed.",
            errors=(400, 404, 409, 500),
            agent=True,
            answer="`{ skill, module, runId, blueprint, ok, text, content }`",
        ),
    )
    async def skill_call(request: models.SkillCallRequest) -> dict[str, Any]:
        try:
            return await called(request.skill, request.args, request.module, request.runId)
        except skills.SkillError as error:
            raise ApiError(error.status, str(error))

    async def mcp_call(name: str, arguments: dict[str, Any]) -> tuple[str, bool]:
        if name == "list_skills":
            answer = await listed()
            query = str(arguments.get("query") or "")
            found = [
                {
                    key: s[key]
                    for key in ("name", "module", "description", "params", "lifecycle", "blueprint")
                }
                for s in answer["skills"]
                if _matches(s, query)
            ]
            down = [run["error"] for run in answer["runs"] if not run["up"]]
            if not found and not answer["runs"]:
                return (
                    "No blueprint is running, so there are no skills. Launch an agentic one first.",
                    False,
                )
            return json.dumps({"skills": found, "problems": down}), False
        if name == "call_skill":
            skill = arguments.get("skill")
            args = arguments.get("args") or {}
            if not isinstance(skill, str) or not isinstance(args, dict):
                return "call_skill needs skill (a string) and args (an object)", True
            try:
                result = await called(skill, args, arguments.get("module"), None)
            except skills.SkillError as error:
                return str(error), True
            return result["text"] or "(no answer text)", not result["ok"]
        return f"no tool {name}", True

    @app.post(
        "/dimos/mcp",
        **route_doc(
            "skills",
            "An MCP server (Streamable HTTP, JSON answers) with two tools: list_skills and call_skill",
            "For an agent: the tools stay the same whatever runs, so a session that connected before a blueprint "
            "started can call its skills (`list_skills`, then `call_skill`, as GET /dimos/skills and POST "
            "/dimos/skills/call). Takes one JSON-RPC 2.0 request (initialize, tools/list, tools/call, ping); a "
            "notification is answered 202 with no body.",
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
        method, params, req_id = body["method"], body.get("params") or {}, body["id"]

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
            text, failed = await mcp_call(str(params.get("name")), params.get("arguments") or {})
            return result({"content": [{"type": "text", "text": text}], "isError": failed})
        return JSONResponse(
            {
                "jsonrpc": "2.0",
                "id": req_id,
                "error": {"code": -32601, "message": f"Unknown: {method}"},
            }
        )
