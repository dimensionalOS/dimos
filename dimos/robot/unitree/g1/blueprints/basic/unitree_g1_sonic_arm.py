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

"""G1 left-arm skill over an independently launched, guarded SONIC controller, via MCP.

The operator launches and stops native control and selects planner idle; this blueprint never
does either. Run it on the robot PC, beside the controller, with the SONIC v1.1 encoder:

    dimos run unitree-g1-sonic-arm \\
        --sonicarmskillcontainer.encoder-path=<deploy>/policy/sonic_v1_1/model_encoder.onnx
    dimos mcp call move_left_arm_forward --arg angle_deg=20
"""

from dimos.agents.mcp.mcp_server import McpServer
from dimos.constants import STATE_DIR
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.g1.sonic_arm_skill import SonicArmSkillContainer
from dimos.robot.unitree.g1.sonic_execution_module import SonicExecutionModule
from dimos.robot.unitree.g1.sonic_session import SonicModelProfile

unitree_g1_sonic_arm = autoconnect(
    SonicExecutionModule.blueprint(
        # The guarded native launcher's fixed local endpoints (reference in, telemetry out).
        reference_bind="tcp://127.0.0.1:5556",
        feedback_endpoint="tcp://127.0.0.1:5557",
        journal_dir=STATE_DIR / "sonic-arm",
        # The skill streams its 11 s plan as one open-loop chunk, as commissioned.
        profile=SonicModelProfile(
            name="g1-sonic-arm",
            source_fps=50,
            right_hand_order="native",
            handback_policy="native_planner",
            max_observation_age=12.0,
        ),
    ),
    SonicArmSkillContainer.blueprint(),
    McpServer.blueprint(),
)
