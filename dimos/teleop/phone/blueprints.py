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
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_fleet import unitree_go2_fleet
from dimos.teleop.phone.phone_extensions import SimplePhoneTeleop

# Simple phone teleop (mobile base axis filtering + cmd_vel output)
teleop_phone = autoconnect(
    SimplePhoneTeleop.blueprint(),
)

# Phone teleop wired to Unitree Go2
teleop_phone_go2 = autoconnect(
    SimplePhoneTeleop.blueprint(),
    unitree_go2_basic,
)

# Phone teleop wired to Go2 fleet — twist commands sent to all robots
teleop_phone_go2_fleet = autoconnect(
    SimplePhoneTeleop.blueprint(),
    unitree_go2_fleet,
)


__all__ = ["teleop_phone", "teleop_phone_go2", "teleop_phone_go2_fleet"]
