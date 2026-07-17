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

"""Unitree G1 GR00T WBC + Quest teleop + WASD arming panel, in one launch.

``unitree-g1-teleop`` (Quest thumbsticks walk the robot, both triggers
engage hand tracking through the dual-arm IK task) plus the
:class:`G1GrootWbcTeleop` pygame panel. On real hardware the panel is the
arming UI (Enter = dry-run off + activate, 10 s ramp) and doubles as a
WASD fallback if the headset drops; its twists go through the
MovementManager's tele_cmd_vel arbitration.

Usage:
    dimos run unitree-g1-teleop-demo                      # real hardware
    dimos --simulation mujoco run unitree-g1-teleop-demo  # sim rehearsal
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.g1.blueprints.basic.unitree_g1_teleop import unitree_g1_teleop
from dimos.robot.unitree.g1.g1_groot_wbc_teleop import G1GrootWbcTeleop

unitree_g1_teleop_demo = autoconnect(
    unitree_g1_teleop,
    G1GrootWbcTeleop.blueprint(),
).remappings([(G1GrootWbcTeleop, "cmd_vel", "tele_cmd_vel")])
