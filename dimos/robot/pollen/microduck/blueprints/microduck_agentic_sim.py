# Copyright 2025-2026 Dimensional Inc.
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

"""Agentic Microduck room simulation - drive the duck from humancli.

Adds the MCP agent stack on top of ``microduck-sim`` so natural-language
commands ("explore the room", "walk to the red ball") become skill calls:

    humancli -> /human_input -> McpClient (LLM) -> McpServer tools
        -> begin_exploration / go_to_object / move_to / ...

Usage:
    dimos run microduck-agentic-sim            # needs OPENAI_API_KEY
    dimos run microduck-agentic-sim-ollama     # local LLM via ollama
    # in a second terminal:
    humancli
"""

from __future__ import annotations

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.ollama_agent import ollama_installed
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.pollen.microduck.blueprints.microduck_sim import (
    MICRODUCK_ROOM_OBJECTS,
    microduck_sim,
)
from dimos.robot.pollen.microduck.skills import MicroduckSkillContainer

MICRODUCK_SYSTEM_PROMPT = """\
You are the brain of Microduck, a tiny (25 cm tall) two-legged duck robot
walking around a small room in simulation. You walk slowly (about 0.1 m/s),
so trips across the room take a minute or two - that is normal.

You can:
- list_objects / where_am_i to orient yourself
- begin_exploration / end_exploration to roam and map the room
- go_to_object(name) to walk right up to a known object
- move_to(x, y) for raw coordinates, stop_moving to halt, wait(seconds)

IMPORTANT: call movement tools strictly ONE AT A TIME and wait for each
result before calling the next - never combine begin_exploration,
go_to_object, or move_to in the same step. To explore "for a while", call
begin_exploration, then wait(seconds), then end_exploration, each as its
own step.

Keep answers short and playful - you are a duck. When asked to find or go
to something, prefer go_to_object. Report what you did once actions finish.
"""

_skills = MicroduckSkillContainer.blueprint(objects=MICRODUCK_ROOM_OBJECTS)

microduck_agentic_sim = autoconnect(
    microduck_sim,
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=MICRODUCK_SYSTEM_PROMPT),
    _skills,
)

microduck_agentic_sim_ollama = autoconnect(
    microduck_sim,
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=MICRODUCK_SYSTEM_PROMPT, model="ollama:qwen3:8b"),
    _skills,
).requirements(
    ollama_installed,
)
