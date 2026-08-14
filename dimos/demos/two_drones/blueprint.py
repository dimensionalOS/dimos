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
from dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.robot.unitree.go2.connection import GO2Connection

DRONE_NAME = os.environ.get("DRONE_NAME", "droneA")
DRONE_MCP_PORT = int(os.environ.get("DRONE_MCP_PORT", "9990"))
DRONE_MODEL = os.environ.get("DRONE_MODEL", "ollama:gemma4")
DRONE_BOUND_X = float(os.environ.get("DRONE_BOUND_X", "10.2"))
DRONE_BOUND_Y = float(os.environ.get("DRONE_BOUND_Y", "6.2"))

_SYSTEM_PROMPT = f"""You are {DRONE_NAME}, an autonomous search drone flying inside a shared
simulated arena with one partner drone. You cannot see the partner directly —
everything you know about it arrives over the radio, and everything it knows
about you is what you transmit. Work as a team.

Your target sensor is a forward cone: field of view 140 degrees, range 15 m,
blocked by walls. Plan coverage with that footprint in mind (sweep lanes up to
~12-18 m apart still overlap; hugging walls wastes half the cone).

Search doctrine:
1. Negotiate halves over the radio first: claim_sector YOUR half. If your
   partner already claimed a sector (check radio_status), do NOT contest it —
   claim the complementary area.
2. Then cover your half: fly_to a point inside it, then begin_exploration
   (autonomous frontier exploration — it keeps moving toward unmapped space
   on its own). sweep_area is the alternative for methodical lanes. Only one
   movement mode can run at a time: call end_exploration or stop_moving
   before starting a different movement tool.
3. The INSTANT a [SENSOR] message says the target is in sight:
   report_sighting(x, y), then end_exploration and intercept(x, y).
4. The INSTANT your partner reports a sighting over the radio: end_exploration
   and intercept(x, y) at the reported coordinates — BOTH drones fly straight
   at the target and keep updating as new positions arrive.
5. If the target is lost: tell your partner where it was last seen and split a
   LOCAL re-search around that position (small sectors, not the whole arena).

Keep radio messages short, factual, and cooperative. Always answer sensor and
radio prompts with actions (tools), not just words."""

dimsim_drone = (
    autoconnect(
        GO2Connection.blueprint(camera=False),
        VoxelGridMapper.blueprint(emit_every=5),
        CostMapper.blueprint(),
        ReplanningAStarPlanner.blueprint(),
        # Tuned for the 42x68 m arena: allow far frontier goals so exploration
        # strides across the map instead of orbiting the spawn corner.
        WavefrontFrontierExplorer.blueprint(
            goal_timeout=30.0,
            max_explored_distance=28.0,
            lookahead_distance=10.0,
        ),
        MovementManager.blueprint(),
        McpServer.blueprint(),
        McpClient.blueprint(
            model=DRONE_MODEL,
            mcp_server_url=f"http://localhost:{DRONE_MCP_PORT}/mcp",
            system_prompt=_SYSTEM_PROMPT,
        ),
        DroneSkillContainer.blueprint(bound_x=DRONE_BOUND_X, bound_y=DRONE_BOUND_Y),
        RadioModule.blueprint(drone_name=DRONE_NAME),
    )
    .namespace(DRONE_NAME)
    .global_config(n_workers=11, mcp_port=DRONE_MCP_PORT, robot_model="unitree_go2")
)
