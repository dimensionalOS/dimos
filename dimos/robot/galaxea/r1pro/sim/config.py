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

"""Mobile R1 Pro model and demonstration servo tuning."""

from dimos.robot.assets.model import RobotModel
from dimos.robot.galaxea.r1pro.config import R1PRO_COLLISION_EXCLUSIONS
from dimos.robot.galaxea.r1pro.joints import UPPER_BODY_JOINTS
from dimos.robot.galaxea.r1pro.sim.model import (
    FINGER_JOINTS,
    MODEL_PATH,
    STEER_JOINTS,
    WHEEL_JOINTS,
)
from dimos.sim2.description import Drive, Instance, SceneConfig
from dimos.sim2.module import ControllerConfig

UPPER_JOINTS = tuple(f"r1pro/{name}" for name in UPPER_BODY_JOINTS)
SERVO_JOINTS = (*UPPER_JOINTS, *FINGER_JOINTS)
JOINTS = (*SERVO_JOINTS, *STEER_JOINTS, *WHEEL_JOINTS)
HOME = (0.0,) * 18
# The folding knee bears the upper-body load when lowered; its previous gain
# allowed ~0.10 rad of static sag at the bottom of the demonstrated height range.
KP = (800.0, 2400.0, 800.0, 800.0) + (180.0,) * 14 + (1000.0,) * 4 + (120.0,) * 3 + (0.0,) * 3
KD = (60.0, 100.0, 60.0, 60.0) + (12.0,) * 14 + (10.0,) * 4 + (12.0,) * 3 + (3.0,) * 3
CONTROL = ControllerConfig("r1pro/servo", 20_000_000, JOINTS)
MODEL = RobotModel.from_file(MODEL_PATH)
SCENE = SceneConfig(
    instances=(
        Instance(
            "r1pro",
            xyz=(0, 0, 0.005),
            floating=True,
            drives=tuple(
                Drive(name, max(kp, 1.0), kd, armature=0.02)
                for name, kp, kd in zip(JOINTS, KP, KD, strict=True)
            ),
            collision_exclusions=tuple(
                (f"r1pro/{a}", f"r1pro/{b}") for a, b in R1PRO_COLLISION_EXCLUSIONS
            ),
        ),
    )
)
