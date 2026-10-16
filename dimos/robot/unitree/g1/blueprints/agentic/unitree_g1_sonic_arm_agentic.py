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

"""unitree-g1-sonic-arm plus an LLM agent that can call the left-arm skill.

Needs the model's API key (e.g. OPENAI_API_KEY) in the environment or .env. Talk to it with
`dimos agent-send "swing your left arm forward 17 degrees"`.
"""

from dimos.agents.mcp.mcp_client import McpClient
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_sonic_arm import unitree_g1_sonic_arm
from dimos.robot.unitree.g1.system_prompt import G1_SONIC_ARM_SYSTEM_PROMPT

unitree_g1_sonic_arm_agentic = autoconnect(
    unitree_g1_sonic_arm,
    McpClient.blueprint(system_prompt=G1_SONIC_ARM_SYSTEM_PROMPT),
)
