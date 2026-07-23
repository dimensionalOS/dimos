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

"""Booster K1 keyboard teleop through the ControlCoordinator path.

WASD -> KeyboardTeleop.cmd_vel -> /cmd_vel -> Coordinator.twist_command
   -> velocity task -> transport_lcm adapter -> /booster_k1/cmd_vel -> K1Connection.

Usage:
    dimos --robot-ip <ip> run booster-k1-coordinator-keyboard-teleop
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.booster.k1.blueprints.basic.booster_k1_coordinator import booster_k1_coordinator
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop

booster_k1_coordinator_keyboard_teleop = autoconnect(
    booster_k1_coordinator,
    KeyboardTeleop.blueprint(publish_only_when_active=True),
)
