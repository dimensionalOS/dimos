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

"""Hosted teleop blueprints (WebRTC transport)."""

from dimos.control.blueprints.teleop import coordinator_teleop_xarm7
from dimos.core.coordination.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic
from dimos.teleop.quest.quest_types import Buttons
from dimos.teleop.quest_hosted.hosted_extensions import (
    HostedArmTeleopModule,
    HostedTwistTeleopModule,
)

# Single XArm7 teleop via the hosted (WebRTC) client. Pass `--simulation` to
# run the coordinator inside MuJoCo, omit it for real hardware.
# rerun_open="none" suppresses the robot-side rerun window (operator gets video
# over WebRTC).
teleop_hosted_xarm7 = (
    autoconnect(
        HostedArmTeleopModule.blueprint(task_names={"right": "teleop_xarm"}),
        coordinator_teleop_xarm7,
    )
    .transports(
        {
            ("right_controller_output", PoseStamped): LCMTransport(
                "/coordinator/cartesian_command", PoseStamped
            ),
            ("buttons", Buttons): LCMTransport("/teleop/buttons", Buttons),
        }
    )
    .global_config(rerun_open="none")
)


# viewer="none" drops the rerun window (operator gets video over WebRTC, so the
# robot-side rerun view is unwanted here).
teleop_hosted_go2 = autoconnect(
    HostedTwistTeleopModule.blueprint(),
    unitree_go2_basic,
).global_config(n_workers=8, viewer="none")


__all__ = [
    "teleop_hosted_go2",
    "teleop_hosted_xarm7",
]
