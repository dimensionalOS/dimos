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

"""Unitree Go2 keyboard teleop via WebRTC with Rage Mode enabled.

Same topology as unitree-go2-webrtc-keyboard-teleop, but GO2Connection is
configured with rage_mode=True so FsmRageMode is toggled on after
BalanceStand at connection start.

Usage:
    dimos run unitree-go2-webrtc-rage-keyboard-teleop
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_webrtc_keyboard_teleop import (
    unitree_go2_webrtc_keyboard_teleop,
)
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop

unitree_go2_webrtc_rage_keyboard_teleop = autoconnect(
    unitree_go2_webrtc_keyboard_teleop,
    GO2Connection.blueprint(mode="rage"),
    KeyboardTeleop.blueprint(linear_speed=1.25, angular_speed=1.2),
).global_config(obstacle_avoidance=True)

__all__ = ["unitree_go2_webrtc_rage_keyboard_teleop"]
