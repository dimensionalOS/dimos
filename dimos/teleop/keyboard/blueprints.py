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

"""Keyboard teleop blueprints for mobile base control.

Standalone keyboard teleop publishes Twist on cmd_vel over LCM.
Compose with a subscriber module or robot blueprint to consume the
Twist output (e.g. ``autoconnect(teleop_keyboard, my_robot_stack)``).

For Go2-specific keyboard teleop blueprints, see
dimos/robot/unitree/go2/blueprints/basic/.

Usage:
    dimos run teleop-keyboard
"""

from dimos.core.coordination.blueprints import autoconnect
from dimos.teleop.keyboard.twist_keyboard_teleop import KeyboardTeleop

# Standalone keyboard teleop — publishes Twist on /cmd_vel.
# Use for development, testing, or composing with any robot that
# subscribes to cmd_vel: In[Twist].
teleop_keyboard = autoconnect(
    KeyboardTeleop.blueprint(),
)


__all__ = ["teleop_keyboard"]
