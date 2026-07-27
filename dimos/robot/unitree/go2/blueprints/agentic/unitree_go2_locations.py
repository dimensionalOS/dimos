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

"""Agentic Go2 with durable saved locations that survive a restart.

``RelocalizationModule`` (which publishes the durable ``world -> map`` frame) shipped
only in ``unitree_go2_relocalization``, while ``tag_location`` lives in the agentic
stack. Locations tagged in one run are only meaningful in another if both are
running, so this composes them and adds ``LocationMemory`` to own the store.

Requires a prebuilt map::

    dimos run unitree-go2-agentic-locations --robot-ip 192.168.123.161 \\
        -o relocalizationmodule.map_file=my_office_map

Without ``map_file`` relocalization stays disabled, and every tagged location is
saved run-local — usable within the session, gone on restart, and the skill says
so when you tag it.
"""

from dimos.agents.mcp.mcp_client import McpClient
from dimos.agents.mcp.mcp_server import McpServer
from dimos.core.coordination.blueprints import autoconnect
from dimos.mapping.relocalization.module import RelocalizationModule
from dimos.memory2.location_module import LocationMemory
from dimos.robot.unitree.go2.blueprints.agentic._common_agentic import _common_agentic
from dimos.robot.unitree.go2.blueprints.smart.unitree_go2_spatial import unitree_go2_spatial

unitree_go2_agentic_locations = autoconnect(
    unitree_go2_spatial,
    RelocalizationModule.blueprint(),
    LocationMemory.blueprint(),
    McpServer.blueprint(),
    McpClient.blueprint(),
    _common_agentic,
).global_config(n_workers=12)
