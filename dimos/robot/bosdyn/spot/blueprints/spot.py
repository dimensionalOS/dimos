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

"""Boston Dynamics Spot keyboard teleop with a Rerun viewer.

Pygame keyboard -> Twist on `cmd_vel` -> SpotCmdVel -> robot, plus a Rerun
bridge that spawns the viewer. WASD to move/turn, QE to strafe, Space for
e-stop, ESC to quit.

The ip auto-detects: with no `-o spotcmdvel.ip=` given, it probes Spot's WiFi
AP address (192.168.80.3) then the Ethernet address (10.0.0.3) and uses
whichever answers.

Usage:
    dimos run spot \
        -o spotcmdvel.username=admin \
        -o spotcmdvel.password=<password>
    # or force an address:
    dimos run spot ... -o spotcmdvel.ip=10.0.0.3
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.bosdyn.spot.cmd_vel import SpotCmdVel
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop
from dimos.visualization.rerun.bridge import RerunBridgeModule

spot = autoconnect(
    SpotCmdVel.blueprint(),
    KeyboardTeleop.blueprint(),
    RerunBridgeModule.blueprint(),
)
