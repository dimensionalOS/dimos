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

"""The OpenYAM in MuJoCo, on the Dual OpenYAM's table, behind the ports of its connection.

The arm is built from the planning model's URDF and stands where the Dual
OpenYAM's left arm does, beside the overhead camera. Its jaw and wrist camera
follow the Dual OpenYAM's, measured again on this arm's own meshes.
"""

from __future__ import annotations

from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.manipulators.dual_openyam.sim_connection import (
    DamiaoSimConnection,
    DamiaoSimConnectionConfig,
    WristPorts,
)
from dimos.robot.manipulators.dual_openyam.sim_scene import ArmLayout, SimWorkcell
from dimos.robot.manipulators.openyam.config import OPENYAM

#: The camera on the hand.
OPENYAM_WRIST_CAMERA = "wrist_camera"
#: Where the URDF's root, the joint1 pivot, stands on the Dual OpenYAM's rig,
#: metres: on the left arm's place, 76.4 mm up, where the base mesh's underside
#: rests on the plate.
OPENYAM_ROOT_POSITION = (0.0, 0.31, 0.0764)

# The fingers hang from arm_link_6 and slide along its y axis. The URDF's
# joint7 and joint8 range from 0.01081 m, where the fingertips touch, to
# 0.05946 m, where they stand 97 mm apart; the robot's gripper calls that lower
# end open, so a command to open drives them to the upper end.
_FINGER_TRAVEL = (0.01081, 0.0594641)
# What grips, as (centre, half size) boxes in arm_link_6's frame with the jaw
# closed, read off hand_right_finger's mesh: the flat nose, then the shank
# behind it, 3 mm further back. The left finger mirrors y.
_PADS = (
    ((0.140, 0.003, 0.0), (0.010, 0.003, 0.010)),
    ((0.1075, 0.007, 0.0), (0.0175, 0.004, 0.018)),
)

#: The OpenYAM's one arm: the fingers point along the hand's x, its top is z.
OPENYAM_WORKCELL = SimWorkcell(
    name="OpenYAM",
    urdf=OPENYAM.model,
    arms=(
        ArmLayout(
            prefix="",
            gripper="hand",
            fingers_along=(1.0, 0.0, 0.0),
            top_along=(0.0, 0.0, 1.0),
            fingertips=("hand_right_finger", "hand_left_finger"),
            finger_axes=((0.0, 1.0, 0.0), (0.0, -1.0, 0.0)),
            finger_travel=_FINGER_TRAVEL,
            pads=_PADS,
            wrist_camera=OPENYAM_WRIST_CAMERA,
        ),
    ),
    root_position=OPENYAM_ROOT_POSITION,
)


class OpenYamSimConnectionConfig(DamiaoSimConnectionConfig):
    """Settings of the simulated OpenYAM."""

    source: str = "yam"


class OpenYamSimConnection(DamiaoSimConnection):
    """One OpenYAM arm and jaw in MuJoCo, where ``OpenYamConnection`` drives the robot.

    The wrist camera publishes on ``wrist_image`` and ``wrist_camera_info``,
    the names the Quest collection records, in the frames of a "wrist"
    RealSense.
    """

    robot = OPENYAM
    workcell = OPENYAM_WORKCELL
    config: OpenYamSimConnectionConfig
    wrist_image: Out[Image]
    wrist_camera_info: Out[CameraInfo]

    def _wrist_ports(self) -> dict[str, WristPorts]:
        return {OPENYAM_WRIST_CAMERA: (self.wrist_image, self.wrist_camera_info, "wrist")}
