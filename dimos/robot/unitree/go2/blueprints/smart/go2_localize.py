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

"""Real Go2 stack that localizes objects from the camera and lidar as you drive.

It keeps the usual Go2 navigation blueprint and its teleop through
``MovementManager``, feeds the camera and the lidar into ``PerceptionLocalize``,
and starts an MCP server so you can ask where things are.
"""

from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.perception.localize.localize_params import GO2_PARAMS
from dimos.perception.localize.module import PerceptionLocalize
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2

go2_localize = autoconnect(
    unitree_go2,
    MovementManager.blueprint(),
    PerceptionLocalize.blueprint(geometry="lidar", mobile=True, params=GO2_PARAMS),
    McpServer.blueprint(),
).global_config(n_workers=12)
