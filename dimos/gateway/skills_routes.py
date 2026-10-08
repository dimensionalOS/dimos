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
whichever one runs later. A skill is called only when a person or agent asks for it here."""

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
        "params. Empty when no blueprint runs. Call this before call_skill: a skill such as Go2's execute_sport_command takes the command (FrontJump, Sit, ...) as "
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
        return await asyncio.to_thread(skills.list_skills)

    async def called(
        skill: str, args: dict[str, Any], module: str | None, run_id: str | None
    ) -> dict[str, Any]:
        return await asyncio.to_thread(skills.call_skill, skill, args, module, run_id)

    @app.get(
        "/dimos/skills",
        response_model=models.SkillList,
        **route_doc(
            "skills",
            "The running blueprint's skills: name, module, description, params",
            "Every `@skill` method of the running blueprint's modules, whether or not it has an agent: over dimos's "
            "module RPC, `Coordinator/list_modules` then each module's `get_skills` (the JSON schema McpServer gives "
            "an agent). Empty `skills` and null `run` when nothing runs; `errors` names a module that didn't answer. "
            "No side effects.",
            agent=True,
            answer="`{ skills: [{ name, module, description, params, required, lifecycle, uses, runId, blueprint }], "
            "run: { runId, blueprint } | null, errors: [{ module, error }] }`",
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
            "Calls `<module>/<skill>` over dimos's module RPC, as the coordinator calls a module's start. A skill that "
            "holds a capability (`uses`) goes through the run's McpServer instead when one answers (`via: mcp`), so "
            "its agent's capability locks cover it: it waits or is refused while another skill holds one. A "
            "background skill answers at once. This acts on the robot. Waits up to 300 s. `ok` false when the skill "
            "itself failed. 400 for a missing or unknown argument (checked against `params`) or a name two modules "
            "share without `module`, 404 when the running blueprint has no such skill (or `runId` isn't it), 409 when "
            "nothing runs, 500 when the MCP call failed or it didn't answer in time.",
            errors=(400, 404, 409, 500),
            agent=True,
            answer="`{ skill, module, runId, blueprint, via, ok, text, content }`",
        ),
    )
    async def skill_call(request: models.SkillCallRequest) -> dict[str, Any]:
        try:
            return await called(request.skill, request.args, request.module, request.runId)
        except skills.SkillError as error:
            raise ApiError(error.status, str(error))

    @app.get(
        "/dimos/rpc",
        response_model=models.RpcList,
        **route_doc(
            "skills",
            "The running blueprint's module RPC methods: module, method, params, docstring",
            "Every `@rpc` method of the running blueprint's modules (`Coordinator/list_modules`), skills included, "
            "with its signature and docstring when its class imports in the gateway. The lifecycle methods (start, "
            "stop, build, set_transport, set_module_ref) are left out. Empty `rpcs` and null `run` when nothing "
            "runs. No side effects.",
            agent=True,
            answer="`{ rpcs: [{ module, method, class, known, params: [{ name, type, default, required, kind }], "
            "return_type, doc, skill, runId, blueprint }], run: { runId, blueprint } | null }`",
        ),
    )
    async def rpc_list() -> dict[str, Any]:
        return await asyncio.to_thread(skills.list_rpcs)

    @app.post(
        "/dimos/rpc/call",
        response_model=models.RpcCallResult,
        **route_doc(
            "skills",
            "Call a module's RPC method and wait for its answer",
            "Calls `<module>/<method>` over dimos's module RPC. `args` is an object (by name) or an array (by "
            "position), checked against its `params`. This can act on the robot. Waits up to 300 s. `ok` false when "
            "the method raised. 400 for start or stop (or another lifecycle method: the coordinator runs those) and "
            "for a missing, extra or unknown argument, 404 when the running blueprint has no such module or method, "
            "409 when nothing runs, 500 when it didn't answer in time.",
            errors=(400, 404, 409, 500),
            agent=True,
            answer="`{ module, method, runId, blueprint, ok, result, text }`",
        ),
    )
    async def rpc_call(request: models.RpcCallRequest) -> dict[str, Any]:
        try:
            return await asyncio.to_thread(
                skills.call_rpc, request.module, request.method, request.args
            )
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
            if answer["run"] is None:
                return "No blueprint is running, so there are no skills. Launch one first.", False
            problems = [f"{e['module']}: {e['error']}" for e in answer["errors"]]
            return json.dumps({"skills": found, "problems": problems}), False
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
