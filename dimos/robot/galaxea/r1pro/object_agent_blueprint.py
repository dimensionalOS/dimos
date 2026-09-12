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

"""Interactive ACT picking of randomly generated objects in the house."""

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.constants import RECORDINGS_DIR
from dimos.core.coordination.blueprints import autoconnect
from dimos.imitation.policy.lerobot.module import R1ProObjectPackingPolicy
from dimos.imitation.policy.skills import PolicySkills
from dimos.robot.galaxea.r1pro.grasping_blueprint import build_r1pro_manipulation
from dimos.robot.galaxea.r1pro.object_coordinator import R1ProObjectCoordinator
from dimos.robot.galaxea.r1pro.object_packing import OBJECT_PACKING_TASK
from dimos.robot.galaxea.r1pro.object_packing_sim import R1ProObjectPackingSim
from dimos.robot.galaxea.r1pro.object_skills import R1ProObjectSkills

OBJECT_PROMPT = """You control an R1Pro in a MuJoCo house. Start idle and execute only requested actions.
Call get_scene before choosing a source object. IDs object_1..object_5 are stable; index is zero-based.
Positions and RGBA colors are simulator ground truth. Rightmost means minimum left_m, leftmost maximum,
nearest minimum distance_m, furthest maximum. Exclude objects already inside the tray. For combined
shape/color/spatial descriptions, resolve the actual requested object from get_scene, then pass its ID.
Ask briefly if multiple objects remain ambiguous; never substitute the middle object or another target.
Use pick_object to put one requested object into the next geometrically available tray spot with ACT.
Respect the requested arm. This checkpoint supports right-hand grasps only; an explicit left-hand
request must be reported as unsupported, never silently executed with the right hand. Do not claim
bimanual grasping, new household shapes, unloading or tray navigation with this object blueprint.
Each motion tool returns accepted/running immediately. After acceptance, call wait_for_action repeatedly
until completed, failed or cancelled before the next action or a success claim. Accepted is not success.
Multiple requested picks run sequentially. On failure report the actual outcome and recovery result,
then stop that sequence. Do not retry ACT or reset automatically. If recovery_required is true,
recover_action can release supported contacts and return home; unsupported objects remain held.
stop_action cancels and holds. reset_scene clears progress only when the user explicitly requests a reset.
A full tray is a normal geometric limit: report it without rearranging or forcing another object in.
Grasp, lift and placement are ACT. Empty-hand recovery uses classical SDK planning and is never a
successful grasp. Keep replies brief and grounded in measured outcomes.
"""

r1pro_objects_sim = (
    autoconnect(
        build_r1pro_manipulation(
            scene_path=RECORDINGS_DIR / "r1pro-object-sim" / "scene.xml",
            artifact=str(RECORDINGS_DIR / "r1pro-act-task/policy-objects-interactive"),
            device="cuda",
            headless=False,
            simulator=R1ProObjectPackingSim,
            policy_module=R1ProObjectPackingPolicy,
            coordinator_type=R1ProObjectCoordinator,
            task_description=OBJECT_PACKING_TASK,
            prepare_scene_on_build=True,
            simulator_options={"generate_scene": True},
            background_camera_rendering=True,
            viewer_lookat=(0.3, -0.2, 0.8),
            viewer_distance=2.0,
            viewer_azimuth=130,
            viewer_elevation=-35,
        ),
        R1ProObjectSkills.blueprint(),
        McpServer.blueprint(),
    )
    .disabled_modules(PolicySkills)
    .global_config(transport="zenoh", viewer="none", simulation="mujoco", n_workers=4)
)

r1pro_objects_sim_agent = autoconnect(
    r1pro_objects_sim, McpClient.blueprint(system_prompt=OBJECT_PROMPT)
)
