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

"""Agentic recorded-memory world blueprints."""

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.teleop.memory_world.module import MemoryWorldModule

MEMORY_WORLD_SYSTEM_PROMPT = """You answer questions about a recorded robot memory.
Use the analyze_memory tool for every question about the recording. Write complete
Python that inspects the available mem2 streams and assigns a dictionary to `result`.
The dictionary requires `answer` and may include:

- `focus_point`: one world-frame [x, y, z] answer location
- `regions`: objects with `points`, `label`, `color`, and `opacity`
- `evidence_paths`: objects with `points`, `label`, and `color`
- `points`: objects with `position`, `label`, and `color`
- `observation_ids`: color_image observation IDs supporting the answer

All geometry uses world-frame meters and every point has exactly three coordinates.
Prefer robust aggregates such as medians over single extreme observations. Include
the observations or geometry that support the answer. Keep result geometry concise.
Do not provide `route`; the viewer automatically draws a collision-aware route when
you provide focus_point and the recording contains a global_costmap stream.
Do not claim that a visualization was shown unless analyze_memory succeeds.
"""

memory_world_agent = autoconnect(
    MemoryWorldModule.blueprint(),
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=MEMORY_WORLD_SYSTEM_PROMPT),
).global_config(n_workers=4)
