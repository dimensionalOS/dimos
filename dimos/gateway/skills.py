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

"""The running blueprints' skills (a module's `@skill` methods), read and called through each run's own MCP server
(dimos.agents.mcp.McpServer, which an agentic blueprint includes) with dimos's McpAdapter: the same `tools/call` its
agent makes, so capability locks and background skills behave as they do for it."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
import json
from typing import Any

from dimos.agents.mcp.mcp_adapter import McpAdapter, McpError

LIST_TIMEOUT_S = 3
# a skill runs until it returns (move_to waits up to 100 s for the robot to get there); a background one returns at once
CALL_TIMEOUT_S = 300


class SkillError(Exception):
    def __init__(self, status: int, message: str) -> None:
        super().__init__(message)
        self.status = status


@dataclass
class Server:
    """A live run's MCP server."""

    run_id: str
    blueprint: str
    url: str


def _argv_port(argv: list[str]) -> str | None:
    """`--mcp-port N` / `--mcp-port=N` in a run's command line (the registry's overrides miss flags given after
    `run <blueprint>`)."""
    port = None
    for at, arg in enumerate(argv):
        if arg in ("--mcp-port", "--mcp_port") and at + 1 < len(argv):
            port = argv[at + 1]
        elif arg.startswith(("--mcp-port=", "--mcp_port=")):
            port = arg.split("=", 1)[1]
    return port


def mcp_url(entry: Any) -> str:
    """Where a run's McpServer listens: its own `mcp_port` (override or command line), else GlobalConfig's (9990)."""
    from dimos.core.global_config import global_config

    overrides = getattr(entry, "config_overrides", None) or {}
    port = (
        overrides.get("mcp_port")
        or _argv_port(list(getattr(entry, "original_argv", None) or []))
        or global_config.mcp_port
    )
    return f"http://localhost:{int(port)}/mcp"


def live_servers() -> list[Server]:
    """Each live run's MCP url, newest run first; runs sharing one url (only one of them can hold the port) count once,
    as the newest."""
    from dimos.core.run_registry import list_runs

    servers: dict[str, Server] = {}
    for entry in sorted(list_runs(alive_only=True), key=lambda e: str(e.run_id), reverse=True):
        url = mcp_url(entry)
        servers.setdefault(url, Server(str(entry.run_id), str(entry.blueprint), url))
    return list(servers.values())


def _modules_of(adapter: McpAdapter, tools: list[dict[str, Any]]) -> dict[str, str]:
    """skill -> its module's class, from McpServer's own `list_modules` skill (none when the server has no such)."""
    if not any(tool.get("name") == "list_modules" for tool in tools):
        return {}
    try:
        modules = json.loads(adapter.call_tool_text("list_modules")).get("modules", {})
    except (McpError, OSError, ValueError, AttributeError):
        return {}
    return {skill: module for module, skills in modules.items() for skill in skills}


def _skill(server: Server, tool: dict[str, Any], module: str | None) -> dict[str, Any]:
    meta = tool.get("_meta") or {}
    params = tool.get("inputSchema") or {"type": "object", "properties": {}}
    return {
        "name": tool["name"],
        "module": module,
        "description": tool.get("description") or "",
        "params": params,
        "required": list(params.get("required") or []),
        "lifecycle": meta.get("dimos/lifecycle", "instant"),
        "uses": list(meta.get("dimos/uses", [])),
        "runId": server.run_id,
        "blueprint": server.blueprint,
    }


def list_skills(
    servers: list[Server], adapter_for: Callable[[str, int], McpAdapter] = McpAdapter
) -> dict[str, Any]:
    """Every skill of every live run with an MCP server, and each run's server: `up`, or why it isn't."""
    skills: list[dict[str, Any]] = []
    runs: list[dict[str, Any]] = []
    for server in servers:
        adapter = adapter_for(server.url, LIST_TIMEOUT_S)
        try:
            tools = adapter.list_tools()
        except (McpError, OSError, ValueError) as error:
            # OSError covers requests' ConnectionError/Timeout: a blueprint without McpServer, or one still starting
            runs.append(
                {
                    "runId": server.run_id,
                    "blueprint": server.blueprint,
                    "mcpUrl": server.url,
                    "up": False,
                    "error": f"no MCP server answers at {server.url} ({type(error).__name__}): only agentic "
                    "blueprints (with McpServer) expose skills",
                }
            )
            continue
        modules = _modules_of(adapter, tools)
        skills += [_skill(server, tool, modules.get(tool["name"])) for tool in tools]
        runs.append(
            {
                "runId": server.run_id,
                "blueprint": server.blueprint,
                "mcpUrl": server.url,
                "up": True,
                "error": None,
            }
        )
    return {"skills": skills, "runs": runs}


def call_skill(
    servers: list[Server],
    skill: str,
    args: dict[str, Any],
    module: str | None = None,
    run_id: str | None = None,
    adapter_for: Callable[[str, int], McpAdapter] = McpAdapter,
) -> dict[str, Any]:
    """Calls `skill` on the live run that has it (`run_id`/`module` pick one when several do) and waits for its
    answer."""
    if run_id is not None:
        servers = [server for server in servers if server.run_id == run_id]
        if not servers:
            raise SkillError(404, f"no live run {run_id}")
    if not servers:
        raise SkillError(409, "no blueprint is running, so there are no skills to call")
    listed = list_skills(servers, adapter_for)
    matches = [
        s
        for s in listed["skills"]
        if s["name"] == skill and (module is None or s["module"] == module)
    ]
    if not matches:
        known = sorted({s["name"] for s in listed["skills"]})
        where = f" of module {module}" if module else ""
        raise SkillError(
            404,
            f"no running blueprint has a skill {skill}{where}"
            + (
                f"; its skills: {', '.join(known)}"
                if known
                else "; none of the running blueprints has an MCP server"
            ),
        )
    target = matches[0]
    server = next(server for server in servers if server.run_id == target["runId"])
    try:
        result = adapter_for(server.url, CALL_TIMEOUT_S).call_tool(skill, args)
    except (McpError, OSError, ValueError) as error:
        raise SkillError(500, f"calling {skill} failed: {type(error).__name__}: {error}")
    content = result.get("content") or []
    text = "\n".join(str(block.get("text", "")) for block in content if block.get("type") == "text")
    return {
        "skill": skill,
        "module": target["module"],
        "runId": target["runId"],
        "blueprint": target["blueprint"],
        # dimos answers a missing tool or a busy capability as a plain text result, not an error
        "ok": not result.get("isError", False),
        "text": text,
        "content": content,
    }
