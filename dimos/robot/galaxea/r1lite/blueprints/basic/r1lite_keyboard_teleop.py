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

"""R1 Lite keyboard teleop — drives the swerve chassis Twist via WASD.

Composes ``r1lite_coordinator`` (R1LiteConnection + ControlCoordinator with
the chassis BASE + r1lite WHOLE_BODY hardware) with ``KeyboardTeleop``
(Twist out). KeyboardTeleop's ``cmd_vel`` Out is auto-wired to the
coordinator's ``twist_command`` In, which routes through the
``vel_chassis`` task into the chassis ``transport_lcm`` adapter and out to
the robot. The connection module streams the chassis command with a
dead-man: release the keys (or kill this process) and the chassis gets an
explicit zero-velocity stream.

Robot-side prerequisites: RC ON, all switches position 1 (mode 5) — the
chassis VCU only honors software while the RC grants it. See
scripts/r1lite_test/RUNBOOK.md.

Keys (from KeyboardTeleop):
    W/S  ±linear.x          A/D  ±angular.z          Q/E  ±linear.y (strafe)
    Shift  2x boost         Ctrl 0.5x slow           Space  emergency stop

Arms / torso / grippers are NOT driven by this blueprint.

Usage:
    dimos run r1lite-keyboard-teleop
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.galaxea.r1lite.blueprints.basic.r1lite_coordinator import r1lite_coordinator
from dimos.robot.unitree.keyboard_teleop import KeyboardTeleop

r1lite_keyboard_teleop = autoconnect(
    r1lite_coordinator,
    # Gentler than the 0.5/0.8 defaults for indoor bench driving
    # (validated test_03 creep was 0.05 m/s; Shift still boosts 2x).
    KeyboardTeleop.blueprint(linear_speed=0.2, angular_speed=0.4),
)

__all__ = ["r1lite_keyboard_teleop"]
