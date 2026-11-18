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

"""The Dual Metal in MuJoCo, behind the ports of ``DualMetalConnection`` and its cameras.

The simulation is the rig of :mod:`dimos.robot.manipulators.dual_openyam.sim_rig`
with the Dual Metal's arms on the plate: the same table, objects and overhead
camera, and a camera on each gripper.
"""

from __future__ import annotations

from dataclasses import replace

from dimos.core.stream import Out
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.manipulators.dual_makermods_metal.config import DUAL_METAL, SIDES
from dimos.robot.manipulators.dual_openyam.sim_connection import (
    DamiaoSimConnection,
    DamiaoSimConnectionConfig,
    WristPorts,
)
from dimos.robot.manipulators.dual_openyam.sim_scene import ArmLayout, SimWorkcell
from dimos.robot.manipulators.makermods_metal.config import METAL_GRIPPER_RANGE, METAL_SIM_HOME
from dimos.robot.manipulators.makermods_metal.mjcf import METAL_ARMATURE, METAL_CAMERA_POS

#: The Dual Metal as the rig simulates it: each arm starts where the single
#: Metal's sim does, its jaw pointing down over the table. At the robot's
#: resting pose, all zeros, the arms lean on their joint stops, and a
#: simulated stop gives a little, so the joints read just outside the
#: planner's limits.
DUAL_METAL_SIM = replace(DUAL_METAL, home_joints=[*METAL_SIM_HOME, *METAL_SIM_HOME])
#: Each gripper's camera, by side.
DUAL_METAL_WRIST_CAMERAS = {side: f"{side}_wrist_camera" for side in SIDES}

# What grips, as (centre, half size) boxes in gripper_base's frame with the jaw
# closed, read off link7's mesh: the last 2.7 cm of the finger, whose face
# meets the other finger's, then the shank behind it, 2.5 mm further back.
# link8 mirrors y.
_PADS = (
    ((0.0, 0.003, 0.078), (0.007, 0.003, 0.0135)),
    ((0.0, 0.0055, 0.047), (0.011, 0.003, 0.017)),
)


def _metal_arm(side: str) -> ArmLayout:
    # In gripper_base the fingers point along z and close along y. joint7
    # slides link7 along -y as it grows, from fully open at -0.05 m to closed
    # at 0; joint8 copies it mirrored, so link8 travels the other way.
    open_, closed = METAL_GRIPPER_RANGE
    return ArmLayout(
        prefix=f"{side}_",
        gripper=f"{side}_gripper_base",
        fingers_along=(0.0, 0.0, 1.0),
        top_along=(1.0, 0.0, 0.0),
        fingertips=(f"{side}_link7", f"{side}_link8"),
        finger_axes=((0.0, -1.0, 0.0), (0.0, 1.0, 0.0)),
        finger_travel=(closed, open_),
        pads=_PADS,
        wrist_camera=DUAL_METAL_WRIST_CAMERAS[side],
        # Where the single Metal's simulated wrist camera sits.
        camera_above=METAL_CAMERA_POS[0],
        camera_out=METAL_CAMERA_POS[2],
        armature=METAL_ARMATURE,
    )


#: Both Metal arms, 0.3 m either side of the middle of the plate, as the
#: robot's model puts them.
DUAL_METAL_WORKCELL = SimWorkcell(
    name="Dual Metal",
    urdf=DUAL_METAL.gravity_model,
    arms=tuple(_metal_arm(side) for side in SIDES),
)


class DualMetalSimConnectionConfig(DamiaoSimConnectionConfig):
    """Settings of the simulated Dual Metal."""

    source: str = "dual_metal"


class DualMetalSimConnection(DamiaoSimConnection):
    """Both Metal arms and jaws in MuJoCo, where ``DualMetalConnection`` drives the robot.

    Each wrist camera publishes on ``{side}_wrist_image`` and
    ``{side}_wrist_camera_info``, the streams the Dual Metal's Quest
    collection records, in the frames of a "{side}_wrist" RealSense.
    """

    robot = DUAL_METAL_SIM
    workcell = DUAL_METAL_WORKCELL
    config: DualMetalSimConnectionConfig
    left_wrist_image: Out[Image]
    left_wrist_camera_info: Out[CameraInfo]
    right_wrist_image: Out[Image]
    right_wrist_camera_info: Out[CameraInfo]

    def _wrist_ports(self) -> dict[str, WristPorts]:
        return {
            DUAL_METAL_WRIST_CAMERAS["left"]: (
                self.left_wrist_image,
                self.left_wrist_camera_info,
                "left_wrist",
            ),
            DUAL_METAL_WRIST_CAMERAS["right"]: (
                self.right_wrist_image,
                self.right_wrist_camera_info,
                "right_wrist",
            ),
        }
