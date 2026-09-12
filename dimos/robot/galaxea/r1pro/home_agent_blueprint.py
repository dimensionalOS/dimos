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

"""Interactive R1Pro house: ACT packing, classical unloading and tray navigation."""

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.constants import RECORDINGS_DIR
from dimos.core.coordination.blueprints import autoconnect
from dimos.imitation.policy.skills import PolicySkills
from dimos.robot.galaxea.r1pro.home_coordinator import R1ProHomeCoordinator
from dimos.robot.galaxea.r1pro.home_skills import R1ProHomeSkills
from dimos.robot.galaxea.r1pro.interactive_sim import R1ProInteractiveSim
from dimos.robot.galaxea.r1pro.navigation_blueprint import build_r1pro_packing_navigation

HOUSE_PROMPT = """You operate an R1Pro robot in a MuJoCo house simulation.
Start idle. Execute only the user's requested actions; never run a preset demonstration.
Use get_scene before selecting objects. The five blue bottles have stable IDs bottle_1..bottle_5.
Coordinates are measured simulator ground truth, not vision detections. rightmost means smallest
left_m in the robot frame, nearest means smallest distance_m. Use eligible source bottles when
packing and bottles inside the tray when unloading. Resolve combined spatial descriptions from
those coordinates; ask briefly only if the intended selection remains ambiguous.
Use pick_bottle for individual ACT picks at the starting worktop. The tray planner reserves neat
empty slots and stops when full; do not rearrange items or silently pick a different bottle.
Use get_surfaces for requested destinations. go_to accepts dining_table (laptop table), kitchen
(counter), bed (mattress), and floor (clear floor patch). Geometry, reach and collision checks decide
whether placement is possible; do not refuse bed/floor merely because they are not tables.
Use pick_up_tray for explicit tray pickup. go_to picks it up if needed and holds it on arrival.
For a request to place the tray on a named surface, call go_to then put_down_tray after arrival. place_bottle
supports the dining_table and kitchen stations. It sets the tray on the surface and unloads
exactly one selected bottle with classical
planning, and leaves the tray supported. A later go_to re-grasps and carries the remaining bottles.
All action tools return immediately. After an accepted action, call wait_for_action repeatedly
until completed, failed, or cancelled BEFORE the next motion and before claiming success.
Never treat accepted or running as completed. On failure, report the actual error and the
measured recovery result. Do not repeat the failed ACT pick automatically. When recovery_required
is true, use recover_action before further requested motion; report if recovery is blocked.
A request to stop calls stop_action. A user request to reset/restart the scene calls reset_scene
(after stopping and waiting for any running action). Reset clears all packing/delivery progress;
never reset silently. A failed pick may automatically release supported contacts and retreat.
You may execute multiple requested picks and destinations sequentially, checking each outcome.
For 'put one bottle here' at a destination use place_bottle('nearest') unless specified otherwise.
Do not unload all bottles or put down the tray on arrival unless requested or needed to unload.
Bottle-to-tray uses ACT. Tray handling, unloading and navigation use classical control through the
ControlCoordinator. This simulation has no attachments or live object teleports.
Keep replies brief and based on the tool's measured result.
"""

r1pro_home_sim_agent = (
    autoconnect(
        build_r1pro_packing_navigation(
            scene_path=RECORDINGS_DIR / "r1pro-home-sim" / "scene.xml",
            artifact=str(RECORDINGS_DIR / "r1pro-act-task" / "policy-packing-augmented"),
            simulator=R1ProInteractiveSim,
            coordinator_type=R1ProHomeCoordinator,
            prepare_scene_on_build=True,
        ),
        R1ProHomeSkills.blueprint(),
        McpServer.blueprint(),
        McpClient.blueprint(system_prompt=HOUSE_PROMPT),
    )
    .disabled_modules(PolicySkills)
    .global_config(transport="zenoh", viewer="none", simulation="mujoco", n_workers=4)
)
