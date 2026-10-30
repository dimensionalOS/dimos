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

"""One G1 description for TF, visuals, planning and physics; walking drive profile.

The GR00T policy's home pose, gains and rotor inertias are a controller
profile, not another robot geometry/kinematic description. URDF limits prevail.
Mounts: pelvis IMU from the GR00T model; head camera/lidar are prototype sensors.
"""

import math
from pathlib import Path

from dimos.control.components import make_humanoid_joints
from dimos.control.tasks.g1_groot_wbc_task.profile import (
    G1_GROOT_DEFAULT_POSITIONS,
    G1_GROOT_KD,
    G1_GROOT_KP,
)
from dimos.robot.assets.model import RobotModel
from dimos.sim2.types import Drive
from dimos.utils.data import LfsPath

G1_MODEL = RobotModel.from_file(
    Path(__file__).with_name("g1.urdf"),
    package_paths={"g1_description": LfsPath("g1_urdf")},
).with_subtree_rooted_at("pelvis")
G1_SENSOR_MODEL = (
    # GR00T observes the floating base, not the independently actuated torso.
    G1_MODEL.with_fixed_frame("imu", "pelvis", xyz=(0.04525, 0, -0.08339))
    .with_fixed_frame(
        "camera", "torso_link", xyz=(0.05, 0, 0.40), rpy=(-math.pi / 2, 0, -math.pi / 2)
    )
    .with_fixed_frame("lidar", "torso_link", xyz=(0.07, 0, 0.42))
)

# GR00T DDS order, gains and zero-offset pose; the existing task owns these facts.
# Reflected inertia/friction follow the existing GR00T simulation motor profile.
WALKING_DRIVES = tuple(
    Drive(name.partition("/")[2] + "_joint", kp, kd, home, 0.01, 0.1)
    for name, kp, kd, home in zip(
        make_humanoid_joints("g1"),
        G1_GROOT_KP,
        G1_GROOT_KD,
        G1_GROOT_DEFAULT_POSITIONS,
        strict=True,
    )
)
