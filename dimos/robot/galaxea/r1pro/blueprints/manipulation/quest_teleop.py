#!/usr/bin/env python3
# Copyright 2026 Dimensional Inc.
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

"""R1 Pro Quest teleoperation against the REAL robot, with no broker.

The headset connects straight to the robot's own WebXR page, so this needs no
API key and no internet — the fewest moving parts between an operator and the
arms. Use it to prove tracking and IK on hardware before adding the broker.

``teleop-quest-r1pro`` is the mock-hardware sibling: same operator experience,
in-memory adapter, nothing physical moves. ``r1pro-hosted-teleop-quest`` is the
same control stack driven through the broker instead of the local page.

THE ARMS MOVE. Have the physical E-stop in reach, and boot the Galaxea driver
first (see dimos/robot/galaxea/r1pro/README.md).

Usage:
    dimos run r1pro-quest-teleop
    # then open https://<robot-ip>:8443/teleop in the headset, hold X + A
"""

from __future__ import annotations

from dimos.core.coordination.blueprints import Blueprint, autoconnect
from dimos.robot.galaxea.r1pro.blueprints.basic.r1pro_coordinator import r1pro_control
from dimos.robot.galaxea.r1pro.blueprints.manipulation.hosted_teleop import (
    R1ProTeleopCoordinator,
    r1pro_teleop_tasks,
)
from dimos.robot.galaxea.r1pro.ready_pose import R1ProReadyPoseModule
from dimos.teleop.quest.quest_extensions import HeadsetArmTeleopModule


# The same task set the hosted blueprint runs, so hardware behaviour is
# identical whichever way the operator connects. Here the head target really is
# the headset: HeadsetArmTeleopModule publishes the viewer pose, and the torso
# follows the operator's height instead of a thumbstick.
def _quest_teleop(*, torso: bool) -> Blueprint:
    return autoconnect(
        HeadsetArmTeleopModule.blueprint(),
        # Arms come up to the tray pose before the operator engages; disable
        # with --r1proreadyposemodule.enabled=false.
        R1ProReadyPoseModule.blueprint(),
        r1pro_control(
            tasks=r1pro_teleop_tasks(torso=torso),
            coordinator_cls=R1ProTeleopCoordinator,
        ),
    ).remappings(
        [
            (HeadsetArmTeleopModule, "left_controller_output", "left_cartesian_command"),
            (HeadsetArmTeleopModule, "right_controller_output", "right_cartesian_command"),
            (HeadsetArmTeleopModule, "headset_output", "head_cartesian_command"),
        ]
    )


r1pro_quest_teleop = autoconnect(_quest_teleop(torso=True))

# Same stack with the torso held still. Both arms hang off torso_link4, so the
# head target and the two hand targets pull on the same four torso joints; if
# wrist orientation feels muddy, run this and compare.
r1pro_quest_teleop_arms_only = autoconnect(_quest_teleop(torso=False))
