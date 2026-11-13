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

from __future__ import annotations

from dataclasses import dataclass, field
from functools import cache
from typing import Any

from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.robot.assets.model import RobotModel
from dimos.robot.manipulators.openarm.config import OPENARM_DESCRIPTION_ROOT
from dimos.utils.data import LfsPath

DEFAULT_ADDRESS = "172.6.2.20:11323"
ALFRED_DESCRIPTION = LfsPath("alfred_description")
ALFRED_URDF = ALFRED_DESCRIPTION / "urdf" / "alfred_v1.urdf"
# The Mid-360 sees the lower body box out to 0.61 m.
LIO_BLIND_M = 0.65


@cache
def alfred_model() -> RobotModel:
    """The whole robot with the lift at its top stop and the arms hanging, every joint welded."""
    model = RobotModel.from_file(
        ALFRED_URDF,
        package_paths={
            "alfred_description": ALFRED_DESCRIPTION,
            "openarm_description": OPENARM_DESCRIPTION_ROOT,
        },
    )
    return model.with_fixed_joints(
        *(joint.name for joint in model.load().joints if joint.type != "fixed")
    )


@dataclass(frozen=True)
class AlfredConfig:
    """Physical metadata used by Alfred navigation and sensor blueprints."""

    name: str
    height_clearance: float
    width_clearance: float
    mid360_ip: str
    d455_serial: str
    internal_odom_offsets: dict[str, Any] = field(default_factory=dict)


ALFRED_HARDWARE = AlfredConfig(
    name="alfred",
    height_clearance=2.0,  # meters
    width_clearance=1.0,
    # The Mid-360 is on the Jetson's wired 192.168.1.100/24 link.
    mid360_ip="192.168.1.189",
    # The mast D455. The rear D435i stays plugged in, so the device is pinned by serial.
    d455_serial="260922302422",
    internal_odom_offsets={
        # Mid-360 lidar: a bit forward, and a bit to the right of base center, above ground.
        "mid360_link": Pose(0.20, -0.20, 0.30, *Quaternion.from_euler(Vector3(0, 0, 0))),
    },
)
