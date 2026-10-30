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

"""One robot definition, reused twice. No engine imports or duplicated mount poses."""

import math

from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.robot.assets.model import RobotModel
from dimos.sim2.types import Camera, Drive, Imu, Instance, Lidar, World

ASSETS = DIMOS_PROJECT_ROOT / "assets" / "sim2_lab"
PUSHER = (
    RobotModel.from_file(ASSETS / "pusher.urdf")
    .with_fixed_frame(
        "camera", "tool", xyz=(-0.25, 0, 0.6), rpy=(-3 * math.pi / 4, 0, -math.pi / 2)
    )
    .with_fixed_frame("lidar", "tool", xyz=(0, 0, 0.12))
    .with_fixed_frame("imu", "tool")
)


def lab_world() -> World:
    robots = tuple(
        Instance(
            name,
            PUSHER.with_prefix(name + "/"),
            xyz=(0, y, 0),
            drives=tuple(Drive(name + "/" + joint) for joint in ("slide_x", "slide_y")),
        )
        for name, y in (("left", 0.0), ("right", 1.2))
    )
    return World(
        (
            *robots,
            Instance(
                "box", RobotModel.from_file(ASSETS / "box.urdf"), xyz=(0, 0, 0.055), floating=True
            ),
            Instance(
                "door",
                RobotModel.from_file(ASSETS / "door.urdf").with_prefix("door/"),
                xyz=(0.6, 0.4, 0),
            ),
        ),
        tuple(
            profile(f"{name}/{profile.kind}")
            for name in ("left", "right")
            for profile in (Camera, Lidar, Imu)
        ),
    )
