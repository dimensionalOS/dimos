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

"""R1 Lite keyboard teleop client.

Publishes a Twist to the /cmd_vel bus that the running r1lite-coordinator
consumes. Run alongside r1lite-coordinator, not instead of it. WASD drives the
chassis; arms, torso and grippers are not driven here. Needs RC mode 5.

    dimos run r1lite-keyboard-teleop
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop

r1lite_keyboard_teleop = autoconnect(
    KeyboardTeleop.blueprint(linear_speed=0.2, angular_speed=0.4),
).transports({("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist)})
