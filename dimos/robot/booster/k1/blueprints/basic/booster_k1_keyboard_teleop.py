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

"""Booster K1 keyboard teleop: WASD direct to the connection, camera in rerun.

Two control surfaces both publish Twist on /cmd_vel -> K1Connection.move(): the
pygame KeyboardTeleop window, and the rerun/Dashboard WASD overlay (its tele_cmd_vel
remapped to cmd_vel). Camera renders in the rerun viewer (from booster_k1_basic).

Controls: W/S fwd/back, Q/E strafe, A/D turn, Space e-stop, Shift 2x, Ctrl 0.5x, ESC quit.

Usage:
    dimos --robot-ip <ip> --viewer rerun run booster-k1-keyboard-teleop
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.booster.k1.blueprints.basic.booster_k1_basic import booster_k1_basic
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule

# publish_only_when_active sends one zero Twist on key release then stays silent,
# so the two /cmd_vel publishers don't fight over an idle robot.
booster_k1_keyboard_teleop = autoconnect(
    booster_k1_basic,
    KeyboardTeleop.blueprint(publish_only_when_active=True),
).remappings(
    [
        (WebsocketVisModule, "tele_cmd_vel", "cmd_vel"),
        (RerunWebSocketServer, "tele_cmd_vel", "cmd_vel"),
    ]
)
