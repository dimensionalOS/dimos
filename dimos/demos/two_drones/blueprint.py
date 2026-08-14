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

"""Per-drone agent stack for the two-drone dimsim demo.

One fully isolated dimensional stack per drone, namespaced under the drone's
name. Two of these processes share ONE dimsim world (multi-robot bridge,
``--robots droneA,droneB``) and ONE unprefixed ``/radio`` topic — everything
else (sensors, mapping, navigation, the LLM agent) is private to the drone.

Environment:
    DRONE_NAME      droneA | droneB   (default droneA)
    DRONE_MCP_PORT  MCP server port   (default 9990; use 9991 for droneB)
    DRONE_MODEL     LLM for the agent (default ollama:gemma4 — local, no key)

Run (one process per drone, after starting the shared sim):
    DRONE_NAME=droneA DRONE_MCP_PORT=9990 \
        dimos --transport=lcm --simulation dimsim run dimsim-drone --dimsim-external
"""

import os

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.demos.two_drones.drone_skills import DroneSkillContainer
from dimos.demos.two_drones.radio import RadioModule
from dimos.mapping.costmapper import CostMapper
from dimos.mapping.voxels.module import VoxelGridMapper
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.unitree.go2.connection import GO2Connection

DRONE_NAME = os.environ.get("DRONE_NAME", "droneA")
DRONE_MCP_PORT = int(os.environ.get("DRONE_MCP_PORT", "9990"))
DRONE_MODEL = os.environ.get("DRONE_MODEL", "ollama:gemma4")

_SYSTEM_PROMPT = f"""You are {DRONE_NAME}, an autonomous search drone flying inside a shared
simulated arena with one partner drone. You cannot see the partner directly —
everything you know about it arrives over the radio, and everything it knows
about you is what you transmit. Work as a team:

1. When given a search mission, first negotiate over the radio: claim a search
   sector with claim_sector so you and your partner sweep DIFFERENT areas, then
   sweep_area your sector methodically.
2. The moment a [SENSOR] message says you see the target, immediately
   report_sighting so your partner can converge, then fly_to the target and
   keep following it (fly_to its latest coordinates whenever they update).
3. If your partner reports a sighting, stop your sweep and fly_to the reported
   coordinates to converge.
4. If the target is lost, tell your partner and split the re-search: claim a
   new sector near where it was last seen.

Keep radio messages short, factual, and cooperative. Always answer sensor and
radio prompts with actions (tools), not just words."""

dimsim_drone = (
    autoconnect(
        GO2Connection.blueprint(camera=False),
        VoxelGridMapper.blueprint(emit_every=5),
        CostMapper.blueprint(),
        ReplanningAStarPlanner.blueprint(),
        MovementManager.blueprint(),
        McpServer.blueprint(),
        McpClient.blueprint(
            model=DRONE_MODEL,
            mcp_server_url=f"http://localhost:{DRONE_MCP_PORT}/mcp",
            system_prompt=_SYSTEM_PROMPT,
        ),
        DroneSkillContainer.blueprint(),
        RadioModule.blueprint(drone_name=DRONE_NAME),
    )
    .namespace(DRONE_NAME)
    .global_config(n_workers=10, mcp_port=DRONE_MCP_PORT, robot_model="unitree_go2")
)
