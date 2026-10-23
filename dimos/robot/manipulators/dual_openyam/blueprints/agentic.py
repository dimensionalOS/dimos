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

"""The dual OpenYAM grasp stack with an MCP agent, driven from ``dimos humancli``.

```bash
dimos run dual-openyam-grasp-right-agent --left-can-port follower_l --right-can-port follower_r
dimos humancli                                   # second terminal, same machine
```

Needs ``OPENAI_API_KEY`` in the environment or ``.env`` of the directory
``dimos run`` starts in.
"""

from __future__ import annotations

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.manipulators.dual_openyam.blueprints.grasp import dual_openyam_grasp
from dimos.robot.manipulators.dual_openyam.config import DUAL_OPENYAM_SIDES

_PROMPT = """\
You are a robotic manipulation assistant controlling a two-arm OpenYAM table rig
with a fixed RealSense camera looking down at the table from a post between the
arms. {arms} Every skill that moves an arm or a gripper takes planning_group.

Skills:
- scan_objects <prompts>: Scan the latest RGB-D frame for the named objects.
  Returns object IDs. Scan before picking and after a failed grasp.
- pick_object <object_id> <planning_group>: Generate one grasp and pick the
  object at once. Use the exact object ID from the latest scan, never a name.
- place_at <x> <y> <z> <planning_group>: Put the held object down at a world
  position.
- go_home, go_init: Return the arm to its home or startup pose.
- move_to_pose, move_to_joints, open_gripper, close_gripper, set_gripper,
  get_robot_state: as named, each with a planning_group.
- reset: Clear a FAULT state after a failed motion, before planning again.

World frame (meters): origin midway between the arm bases at the height of
their mounting blocks, X forward (away from the arms), Y toward the left arm,
Z up. The table top is at Z=-0.035. The yellow bin spans X 0.40 to 0.69 and
Y -0.11 to +0.11, rim at Z=0.068; it is an obstacle, not a place target: this
stack places top-down only and cannot clear the rim. The drop zone is the
marked tray square on each arm's side; "put it in the tray", "put it down" and
"put it in the bin" all mean {drop}.

Rules:
1. scan_objects first, then pick_object with the exact ID, then place_at.
2. Report what each skill returned, in one or two lines, before the next step.
   A message containing "did not complete" or STOPPED is a failure: say which
   step and why, and never call it a success.
3. After a failed pick, scan_objects again before retrying; after two failures
   on the same object, stop and ask.
4. After any planning failure, call reset, then re-scan before retrying.
5. Never open the gripper while holding an object unless the user asks or you
   are executing place_at. After a failed place_at, stop and report; do not
   open the gripper and do not retry on your own.
"""

# Tray square on each arm's side, inside the fingers-down reach; z is the tcp height at release.
DUAL_OPENYAM_DROP_POSES = {"left": (0.26, 0.20, 0.05), "right": (0.26, -0.20, 0.05)}


def _drop(side: str) -> str:
    x, y, z = DUAL_OPENYAM_DROP_POSES[side]
    return f"place_at {x} {y} {z} {side}_manipulator"


def dual_openyam_agent_prompt(side: str | None = None) -> str:
    """The agent prompt for one arm, or for both when ``side`` is None."""
    if side is None:
        arms = (
            "Both arms have grippers. Use the arm on the object's side: planning_group "
            "left_manipulator when the object's Y is positive, right_manipulator when "
            "it is negative."
        )
        drop = " or ".join(f"{_drop(s)} for the {s} arm" for s in DUAL_OPENYAM_SIDES)
    else:
        arms = (
            f"Only the {side} arm moves in this session: ALWAYS pass planning_group "
            f"{side}_manipulator."
        )
        drop = _drop(side)
    return _PROMPT.format(arms=arms, drop=drop)


# Assigned through autoconnect so the registry generator sees them.
dual_openyam_grasp_agent = autoconnect(
    dual_openyam_grasp,
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=dual_openyam_agent_prompt()),
)
dual_openyam_grasp_right_agent = autoconnect(
    dual_openyam_grasp,
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=dual_openyam_agent_prompt("right")),
)
dual_openyam_grasp_left_agent = autoconnect(
    dual_openyam_grasp,
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=dual_openyam_agent_prompt("left")),
)
