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

"""Go2 quadruped: thumbstick velocity teleop + camera streamed to the headset.

Lives here rather than in ``dimos/teleop/quest/blueprints.py`` so that importing
any other robot's Quest blueprint does not pull in ``GO2Connection``, and with
it the Unitree WebRTC SDK. That SDK cannot be installed on every robot: the
``unitree`` extra depends on ``mapping``, whose arm64 wheels start at cp311.

Usage:
    dimos run teleop-quest-go2
"""

from __future__ import annotations

from dimos.constants import DEFAULT_CAPACITY_COLOR_IMAGE
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport, pSHMTransport
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.teleop.quest.quest_extensions import Go2TeleopModule

teleop_quest_go2 = (
    autoconnect(
        Go2TeleopModule.blueprint(),
        GO2Connection.blueprint(),
    )
    .transports(
        {
            ("cmd_vel", Twist): LCMTransport("/cmd_vel", Twist),
            ("color_image", Image): pSHMTransport(
                "color_image", default_capacity=DEFAULT_CAPACITY_COLOR_IMAGE
            ),
        }
    )
    .global_config(robot_model="unitree_go2")
)
