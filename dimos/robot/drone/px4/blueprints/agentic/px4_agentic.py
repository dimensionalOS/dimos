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

"""PX4 stack with MCP-based agent control."""

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.web_human_input import WebInput
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.drone.px4.blueprints.basic.px4_basic import px4_basic

PX4_SYSTEM_PROMPT = """\
You control a PX4 drone through MAVSDK.
Use the available flight skills only when the user explicitly requests a flight action.
Never arm, take off, land, disarm, or move based on an assumption or an ambiguous request.
Use enter_offboard before move or goto. The move skill is watchdog-protected: call it repeatedly
to maintain velocity, or once for a brief adjustment that automatically returns to zero velocity.
Use hold to stop Offboard control and hold position.
Report command failures exactly and do not claim that a command succeeded when a tool reports failure.
"""

px4_agentic = autoconnect(
    px4_basic,
    McpServer.blueprint(),
    McpClient.blueprint(system_prompt=PX4_SYSTEM_PROMPT),
    WebInput.blueprint(),
).global_config(n_workers=11, robot_model="px4_agentic")
