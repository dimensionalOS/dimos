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
import json
from typing import Any

from experimental.gateway.server.state import ServerState
from experimental.gateway.utils import skills

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


def _matches(skill: dict[str, Any], query: str) -> bool:
    text = f"{skill['name']} {skill['module'] or ''} {skill['description']}".lower()
    return all(word in text for word in query.lower().split())


class SkillsHelpers:
    def __init__(self, state: ServerState) -> None:
        self.state = state

    async def listed(self) -> dict[str, Any]:
        return await asyncio.to_thread(skills.list_skills)

    async def called(
        self, skill: str, args: dict[str, Any], module: str | None, run_id: str | None
    ) -> dict[str, Any]:
        return await asyncio.to_thread(skills.call_skill, skill, args, module, run_id)

    async def mcp_call(self, name: str, arguments: dict[str, Any]) -> tuple[str, bool]:
        if name == "list_skills":
            answer = await self.listed()
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
                return ("No blueprint is running, so there are no skills. Launch one first.", False)
            problems = [f"{e['module']}: {e['error']}" for e in answer["errors"]]
            return (json.dumps({"skills": found, "problems": problems}), False)
        if name == "call_skill":
            skill = arguments.get("skill")
            args = arguments.get("args") or {}
            if not isinstance(skill, str) or not isinstance(args, dict):
                return ("call_skill needs skill (a string) and args (an object)", True)
            try:
                result = await self.called(skill, args, arguments.get("module"), None)
            except skills.SkillError as error:
                return (str(error), True)
            return (result["text"] or "(no answer text)", not result["ok"])
        return (f"no tool {name}", True)
