#!/usr/bin/env python3

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

"""Blueprint for Go2 fleet — multiple Go2 robots controlled together.

Usage:
    ROBOT_IPS=10.0.0.102,10.0.0.209 dimos run unitree-go2-fleet
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.protocol.service.system_configurator.clock_sync import ClockSyncConfigurator
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import with_vis
from dimos.robot.unitree.go2.fleet_connection import Go2FleetConnection
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule

unitree_go2_fleet = (
    autoconnect(
        with_vis,
        Go2FleetConnection.blueprint(),
        WebsocketVisModule.blueprint(),
    )
    .global_config(n_workers=4, robot_model="unitree_go2")
    .configurators(ClockSyncConfigurator())
)

__all__ = ["unitree_go2_fleet"]
