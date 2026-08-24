#!/usr/bin/env python3
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

"""``unitree-go2-agentic`` plus a recording the agent can query in Python.

``Go2Memory`` records lidar/camera/odom as the robot drives; ``MemoryQuerySkill``
opens that same db and execs agent-authored code against it. Ask the robot
pointcloud questions over ``dimos humancli`` and it answers from the recording
instead of guessing.

Live hardware only — ``Recorder`` no-ops under ``--replay``, so there would be
nothing to query.
"""

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.agents.skills.memory_query import MemoryQuerySkill
from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.go2.blueprints.agentic._common_agentic import _common_agentic
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import Go2Memory
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2_spatial import unitree_go2_spatial

unitree_go2_agentic_memory = autoconnect(
    unitree_go2_spatial,
    Go2Memory.blueprint(),
    MemoryQuerySkill.blueprint(),
    McpServer.blueprint(),
    McpClient.blueprint(),
    _common_agentic,
).global_config(n_workers=12, robot_model="unitree_go2")
