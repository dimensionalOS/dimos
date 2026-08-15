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
DRONE_BOUND_X = float(os.environ.get("DRONE_BOUND_X", "10.2"))
DRONE_BOUND_Y = float(os.environ.get("DRONE_BOUND_Y", "6.2"))
DRONE_SENSOR_RANGE = float(os.environ.get("DRONE_SENSOR_RANGE", "15.0"))
# Desired overlap between the two drones' radar footprints (m).
DRONE_RADAR_OVERLAP = float(os.environ.get("DRONE_RADAR_OVERLAP", "2.0"))
# Radio interference schedule (s): channel up, then jammed.
RADIO_ON_S = float(os.environ.get("RADIO_ON_S", "0"))
RADIO_OFF_S = float(os.environ.get("RADIO_OFF_S", "0"))

_SYSTEM_PROMPT = f"""You are {DRONE_NAME}, an autonomous search drone flying inside a shared
simulated arena with one partner drone. You cannot see the partner directly —
everything you know about it arrives over the radio, and everything it knows
about you is what you transmit. Work as a team.

Your target sensor is a forward cone: field of view 140 degrees, range 15 m,
blocked by walls. Plan coverage with that footprint in mind (sweep lanes up to
~12-18 m apart still overlap; hugging walls wastes half the cone).

MISSION: find the moving red target and get within 1 metre of it.

Search doctrine:
1. Negotiate halves over the radio first: claim_sector YOUR half. If your
   partner already claimed a sector (check radio_status), do NOT contest it —
   claim the complementary area.
2. Then call begin_coordinated_search. It searches cooperatively on its own:
   striding toward unexplored space, steering AWAY from walls it has already
   detected, keeping the planned radar overlap with your partner's last known
   position, and avoiding ground either of you already covered. Let it run —
   you do not need to micro-manage waypoints.
3. Your flight firmware AUTO-INTERCEPTS target sightings (your own sensor's
   and the ones your partner reports by radio) and closes to within 1 m.
   Your job on a sighting is communication: report_sighting(x, y) the INSTANT
   your sensor sees it, and keep re-reporting while you can see it.
4. When your partner reports a sighting, acknowledge briefly by radio. Your
   firmware is already converging on the reported position.
5. THE RADIO LINK IS UNRELIABLE — it suffers periodic interference and some of
   your transmissions never arrive. You cannot tell when. So: repeat important
   information (sightings above all) several times, spaced out, instead of
   assuming one transmission got through. Use radio_status to see whether your
   partner is actually being heard from.

Keep radio messages short, factual, and cooperative. Always answer sensor and
radio prompts with actions (tools), not just words."""

dimsim_drone = (
    autoconnect(
        GO2Connection.blueprint(camera=False),
        # Walls don't move: mapping only has to be fresh enough to feed the
        # away-from-walls search prior. A 42x68 m arena makes every voxel/
        # costmap pass expensive, and starving the host stalls the sim itself.
        VoxelGridMapper.blueprint(emit_every=20),
        CostMapper.blueprint(),
        ReplanningAStarPlanner.blueprint(),
        MovementManager.blueprint(),
        McpServer.blueprint(),
        McpClient.blueprint(
            model=DRONE_MODEL,
            mcp_server_url=f"http://localhost:{DRONE_MCP_PORT}/mcp",
            system_prompt=_SYSTEM_PROMPT,
        ),
        DroneSkillContainer.blueprint(
            bound_x=DRONE_BOUND_X,
            bound_y=DRONE_BOUND_Y,
            sensor_range_m=DRONE_SENSOR_RANGE,
            radar_overlap_m=DRONE_RADAR_OVERLAP,
        ),
        RadioModule.blueprint(
            drone_name=DRONE_NAME,
            interference_on_s=RADIO_ON_S,
            interference_off_s=RADIO_OFF_S,
        ),
    )
    .namespace(DRONE_NAME)
    .global_config(n_workers=9, mcp_port=DRONE_MCP_PORT, robot_model="unitree_go2")
)
