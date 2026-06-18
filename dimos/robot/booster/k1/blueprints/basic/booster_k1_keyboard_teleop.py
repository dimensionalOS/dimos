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

"""Booster K1 keyboard teleop: WASD from two surfaces, camera in rerun.

Two control surfaces, both publishing Twist on /cmd_vel, both driving the bot:
  1. pygame KeyboardTeleop window  -> cmd_vel              -> K1Connection.move()
  2. Dimos Dashboard "Keyboard Control" (browser, tele_cmd_vel
     remapped -> cmd_vel)          -> cmd_vel              -> K1Connection.move()

The camera renders in the Rerun viewer (from booster_k1_basic). The Dashboard
tab is the browser cockpit; the Rerun tab is the camera view.

Controls (pygame window or Dashboard keyboard mode):
    W/S forward/back, Q/E strafe, A/D turn, Space e-stop, Shift 2x, Ctrl 0.5x, ESC quit.

Usage:
    dimos --robot-ip <ip> --viewer rerun --rerun-open native run booster-k1-keyboard-teleop
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.booster.k1.blueprints.basic.booster_k1_basic import booster_k1_basic
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop
from dimos.visualization.rerun.websocket_server import RerunWebSocketServer
from dimos.web.websocket_vis.websocket_vis_module import WebsocketVisModule

# publish_only_when_active: stay silent unless a movement key is held (one zero
# Twist on release, then nothing). With the connection's dead-man timer, the
# robot halts as soon as you let go.
#
# remappings: the Dashboard's WASD output (tele_cmd_vel) defaults to /tele_cmd_vel,
# which nothing listens to. Rename it to cmd_vel so it publishes to /cmd_vel,
# the same topic K1Connection consumes. Both surfaces now drive the bot; with
# publish_only_when_active each stays silent while idle, so they don't fight.
booster_k1_keyboard_teleop = autoconnect(
    booster_k1_basic,
    KeyboardTeleop.blueprint(publish_only_when_active=True),
).remappings(
    [
        (WebsocketVisModule, "tele_cmd_vel", "cmd_vel"),
        (RerunWebSocketServer, "tele_cmd_vel", "cmd_vel"),
    ]
)

__all__ = ["booster_k1_keyboard_teleop"]
