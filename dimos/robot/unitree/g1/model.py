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

The installed ONNX policy's home pose, gains and rotor inertias are a controller
profile, not another robot geometry/kinematic description. URDF limits prevail.
Mounts: pelvis IMU from Unitree's model; head camera/lidar are prototype sensors.
"""

import math
from pathlib import Path

from dimos.robot.assets.model import RobotModel
from dimos.sim2.types import Drive
from dimos.utils.data import LfsPath

G1_MODEL = RobotModel.from_file(
    Path(__file__).with_name("g1.urdf"),
    package_paths={"g1_description": LfsPath("g1_urdf")},
).with_subtree_rooted_at("pelvis")
G1_SENSOR_MODEL = (
    G1_MODEL.with_fixed_frame("imu", "pelvis", xyz=(0.04525, 0, -0.08339))
    .with_fixed_frame(
        "camera", "torso_link", xyz=(0.05, 0, 0.40), rpy=(-math.pi / 2, 0, -math.pi / 2)
    )
    .with_fixed_frame("lidar", "torso_link", xyz=(0.07, 0, 0.42))
)

# Policy order is explicit; never infer it from a backend's array ordering.
_LEG = (
    ("hip_pitch", -0.312, 75, 2, 0.01017752004),
    ("hip_roll", 0, 75, 2, 0.025101925),
    ("hip_yaw", 0, 75, 2, 0.01017752004),
    ("knee", 0.669, 75, 2, 0.025101925),
    ("ankle_pitch", -0.363, 20, 1, 0.00721945),
    ("ankle_roll", 0, 2, 0.2, 0.00721945),
)
_ARM = (
    ("shoulder_pitch", 0.2, 75, 2, 0.003609725),
    ("shoulder_roll", 0.2, 75, 2, 0.003609725),
    ("shoulder_yaw", 0, 75, 2, 0.003609725),
    ("elbow", 0.6, 75, 2, 0.003609725),
    ("wrist_roll", 0, 2, 0.2, 0.003609725),
    ("wrist_pitch", 0, 2, 0.2, 0.00425),
    ("wrist_yaw", 0, 2, 0.2, 0.00425),
)
WALKING_DRIVES = (
    *(
        Drive(f"{side}_{name}_joint", kp, kd, home, inertia, 0.1)
        for side in ("left", "right")
        for name, home, kp, kd, inertia in _LEG
    ),
    Drive("waist_yaw_joint", 75, 2, 0, 0.01017752004, 0.1),
    Drive("waist_roll_joint", 75, 2, 0, 0.00721945, 0.1),
    Drive("waist_pitch_joint", 75, 2, 0.073, 0.00721945, 0.1),
    *(
        Drive(
            f"{side}_{name}_joint",
            kp,
            kd,
            -home if side == "right" and name == "shoulder_roll" else home,
            inertia,
            0.1,
        )
        for side in ("left", "right")
        for name, home, kp, kd, inertia in _ARM
    ),
)
