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

"""PACK MIND — live Go2 blueprint (one dog per laptop, shared memory over HTTP).

Each laptop runs THIS blueprint against its own dog; the two processes share one
``PackCoordinator`` over the LAN. There is no in-process multi-instance problem
because the two dogs are two separate `dimos run` processes — they only meet at
the coordinator. Configure per laptop with env vars::

    # Laptop A (also runs the coordinator):
    python -m dimos.experimental.pack_mind.pack_coordinator_server --zones north,east,south,west &
    PACK_DOG_NAME=alpha PACK_COORDINATOR_URL=http://127.0.0.1:8090 \\
        dimos run unitree-go2-pack --robot-ip <DOG_A> --listen-host 0.0.0.0

    # Laptop B:
    PACK_DOG_NAME=bravo PACK_COORDINATOR_URL=http://<LAPTOP_A_LAN_IP>:8090 \\
        dimos run unitree-go2-pack --robot-ip <DOG_B> --listen-host 0.0.0.0

The detection→coordinator bridge needs NO custom module: the agent calls
``look_out_for([target], then={"name": "report_finding", "args": {...}})`` so a VLM
sighting auto-reports the finding to the pack (perceive_loop_skill dispatches the
``then`` continuation).

NOTE: untested without hardware/GPU — bring up per the H0 gate in the runbook.
"""

from __future__ import annotations

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.skills.person_follow import PersonFollowSkillContainer
from dimos.core.coordination.blueprints import autoconnect
from dimos.experimental.pack_mind.pack_search_skills import PackSearchSkills
from dimos.experimental.pack_mind.red_detector import RedObjectDetector
from dimos.experimental.security_demo.security_module import SecurityModule
from dimos.robot.unitree.go2.blueprints.agentic._common_agentic import _common_agentic
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2_spatial import unitree_go2_spatial

PACK_SYSTEM_PROMPT = """\
You are one dog in a PACK that shares a single operational memory through tools.
You never command another dog and never exchange coordinates — you read and write
the shared memory by ZONE NAME only.

MISSION LOOP (follow exactly):
1. When given a target (e.g. "find the red kit"), call start_search(target) ONCE.
2. Repeat:
   a. Call next_zone to get YOUR assigned zone. If it returns empty, the search is
      over — STOP and report status.
   b. navigate_with_text("<that zone name>") to travel there.
   c. Call look_for_red to check the camera for the red object. If it reports one,
      it has already told the whole pack — STOP.
   d. If no red object, call report_cleared("<that zone name>") and continue.
   e. Check should_stop — if true, a packmate found it; STOP.
3. If asked where the target is, call where_is(target) and act on the answer even
   if you never saw it yourself — that is acting on the pack's shared memory.

Keep moving. Be concise. Zones by name, never coordinates."""

# One dog per laptop. PackSearchSkills reads PACK_DOG_NAME + PACK_COORDINATOR_URL
# from the environment, so this single blueprint serves both laptops.
#
# EdgeTAM (segmentation) HARD-requires a CUDA GPU. It reaches the spatial stack two
# ways, both fatal on a CPU/CoreML host (e.g. a Mac ground station):
#   1. unitree_go2_spatial -> SecurityModule.__init__ -> EdgeTAMProcessor()  → deploy crash
#   2. PersonFollowSkillContainer.follow_person -> EdgeTAMProcessor()         → mid-demo crash
# The pack demo uses neither (we move via skills + look_out_for, not security
# patrol or person-follow), so disable both. The blueprint then deploys clean on
# CPU — no --disable flag and no call-time landmine.
unitree_go2_pack = autoconnect(
    unitree_go2_spatial,
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=PACK_SYSTEM_PROMPT),
    _common_agentic,
    PackSearchSkills.blueprint(),
    RedObjectDetector.blueprint(),  # fast GPU-free "red object" find (vs slow moondream)
).disabled_modules(SecurityModule, PersonFollowSkillContainer)

__all__ = ["unitree_go2_pack", "PACK_SYSTEM_PROMPT"]
