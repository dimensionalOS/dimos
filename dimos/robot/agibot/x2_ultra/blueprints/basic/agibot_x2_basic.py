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

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.agibot.x2_ultra.blueprints.primitive.agibot_x2_primitive import (
    agibot_x2_primitive,
)
from dimos.robot.agibot.x2_ultra.connection import X2Connection
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule

agibot_x2_basic = (
    autoconnect(
        agibot_x2_primitive,
        X2Connection.blueprint(),
    )
    .remappings(
        [
            (WebsocketVisModule, "tele_cmd_vel", "cmd_vel"),
        ]
    )
    .global_config(n_workers=4, robot_model="agibot_x2_ultra")
)

__all__ = [
    "agibot_x2_basic",
]
